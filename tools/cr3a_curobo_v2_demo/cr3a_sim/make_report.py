#!/usr/bin/env python3
# [만듦 2026-10-13] WMX 로그로 CR3A 를 움직이는 뷰어 만들기 → <결과 폴더>/viewer.html, summary.json
#   python3 make_report.py <결과 폴더>      (wmx_r2 컨테이너 안, admin. run_demo.sh 가 마지막에 부름)
#   읽는 것: plan.json (cuRoboV2 계획), exec.json (R2 액션 결과), wmx_axis.csv (WMX 엔진 축 로그, 1 ms), API 로그 txt,
#            Dobot URDF + STL 메시
#   계산
#     ① WMX 로그에서 동작 구간 찾기: 명령 위치(CMDPOS)가 움직인 구간 = R2 가 부른 StartCSplinePos 실행 구간
#     ② 계획 궤적을 그 구간 시작에 맞춰 놓고 비교: 명령 − 계획 (R2/WMX 가 입력대로 했나), 실제 − 명령 (따라갔나)
#     ③ 실제 위치(FBPOS)로 FK → 상자/바닥 거리 (계획 때와 같은 함수 cr3a_scene.clearances)
#   뷰어: 3D 로봇은 WMX 로그 값으로 움직인다 (기본: 실제 위치 FBPOS). three.js 는 CDN 에서 받음 (PC 인터넷 필요)
import base64
import csv
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import cr3a_scene as S


def read_wmx_csv(path):
    with open(path) as f:
        r = csv.reader(f)
        head = next(r)
        rows = [row for row in r if len(row) == len(head)]
    a = np.array(rows, dtype=float)
    col = {h: i for i, h in enumerate(head)}
    t = (a[:, col['TIME']] - a[0, col['TIME']]) / 1e6          # TIME 은 µs
    g = lambda p: np.stack([a[:, col[f'{p}{j}']] for j in range(6)], axis=1)
    return {'t': t, 'cycle': a[:, col['CYCLE']], 'cmd': g('CMDPOS'), 'fb': g('FBPOS'), 'cmdvel': g('CMDVEL'),
            'fbvel': g('FBVEL'), 'inpos': g('INPOSITION'), 'srvon': g('SRVON'), 'opstate': g('OPSTATE')}


def find_segments(t, cmd, eps=1e-7, gap=0.05):
    """명령 위치가 움직인 구간 [(i0, i1)]. 1 주기 사이 변화가 eps 보다 크면 움직이는 중. gap 초보다 짧은 멈춤은 이어 붙임"""
    mv = np.any(np.abs(np.diff(cmd, axis=0)) > eps, axis=1)
    segs, i = [], 0
    n = len(mv)
    while i < n:
        if mv[i]:
            j = i
            while j < n and mv[j]:
                j += 1
            segs.append([i, j])
            i = j
        else:
            i += 1
    merged = []
    for s in segs:
        if merged and t[s[0]] - t[merged[-1][1]] < gap:
            merged[-1][1] = s[1]
        else:
            merged.append(s)
    return [(a, b + 1) for a, b in merged]


def parse_api_csplines(path):
    """WMX API 로그(wmx3-apilog-parse 가 만든 txt)에서 StartCSplinePos 호출과 그 입력 점들을 꺼낸다.
    형식 (2026-10-13 확인): '... StartCSplinePos: channel: 0, numDimensions: 6, numPoints: 81, axis[0]: 0, ...'
                           다음 줄들 'point[i]: pos[0]: x pos[1]: y ..., timeMilliseconds: t'"""
    import re
    calls, cur = [], None
    rp = re.compile(r'point\[(\d+)\]: (.*?), timeMilliseconds: (-?[\d.]+)')
    rv = re.compile(r'pos\[(\d+)\]: (-?[\d.]+)')
    for line in open(path, errors='ignore'):
        if 'StartCSplinePos:' in line:
            m = re.search(r'numDimensions: (\d+), numPoints: (\d+)', line)
            ax = [int(a) for a in re.findall(r'axis\[\d+\]: (\d+)', line)]
            ts = line[:23]
            cur = {'time': ts, 'dims': int(m.group(1)), 'n': int(m.group(2)), 'axes': ax, 'pos': [], 't_ms': []}
            calls.append(cur)
            continue
        if cur is not None and len(cur['pos']) < cur['n']:
            m = rp.search(line)
            if m:
                vals = {int(i): float(v) for i, v in rv.findall(m.group(2))}
                cur['pos'].append([vals.get(j, 0.0) for j in range(cur['dims'])])
                cur['t_ms'].append(float(m.group(3)))
    return calls


def interp_plan(tp, qp, vp, tq):
    """계획 점 (tp, 위치 qp[n,6], 속도 vp[n,6]) 을 시각 tq 에서 3차 곡선(Hermite: 점마다 위치+속도)으로 이음. 구간 밖은 끝값.
    [바꿈 2026-10-13] 처음엔 직선(선형) 보간 → WMX 스플라인과 점 사이에서 '가속 × 간격² ÷ 8' 만큼 어긋나 20 ms 톱니가 생겼음
    (사용자 질문 "WMX 명령은 왜 자글자글?"). 3차 곡선 기준이면 톱니 없이 최대 ~0.19 mrad"""
    dt = tp[1] - tp[0]
    tq = np.clip(tq, tp[0], tp[-1])
    k = np.clip(((tq - tp[0]) // dt).astype(int), 0, len(tp) - 2)
    s = ((tq - tp[k]) / dt)[:, None]
    h00, h10, h01, h11 = 2 * s**3 - 3 * s**2 + 1, s**3 - 2 * s**2 + s, -2 * s**3 + 3 * s**2, s**3 - s**2
    return h00 * qp[k] + h10 * dt * vp[k] + h01 * qp[k + 1] + h11 * dt * vp[k + 1]


def urdf_chain(path):
    root = ET.parse(path).getroot()
    joints = []
    for j in root.findall('joint'):
        o = j.find('origin')
        ax = j.find('axis')
        joints.append({'name': j.get('name'), 'type': j.get('type'), 'parent': j.find('parent').get('link'),
                       'child': j.find('child').get('link'),
                       'xyz': [float(v) for v in (o.get('xyz') if o is not None and o.get('xyz') else '0 0 0').split()],
                       'rpy': [float(v) for v in (o.get('rpy') if o is not None and o.get('rpy') else '0 0 0').split()],
                       'axis': [float(v) for v in (ax.get('xyz') if ax is not None else '0 0 1').split()]})
    meshes = {}
    for l in root.findall('link'):
        m = l.find('visual/geometry/mesh')
        if m is not None:
            fn = m.get('filename').replace('package://', '')
            meshes[l.get('name')] = base64.b64encode((S.HERE / fn).read_bytes()).decode()
    return {'root': root.findall('link')[0].get('name'), 'joints': joints, 'meshes': meshes}


def r(x, nd):
    return np.round(np.asarray(x, dtype=float), nd).tolist()


def main():
    out = Path(sys.argv[1])
    plan = json.loads((out / 'plan.json').read_text())
    ex = json.loads((out / 'exec.json').read_text()) if (out / 'exec.json').exists() else {'events': []}
    w = read_wmx_csv(out / 'wmx_axis.csv')
    t = w['t']
    segs = find_segments(t, w['cmd'])
    print(f'WMX 로그: {len(t)} 샘플, {t[-1]:.2f}초, 동작 구간 {len(segs)}개:',
          [(round(t[a], 3), round(t[b - 1], 3)) for a, b in segs])
    moves = plan['moves']
    if len(segs) < len(moves):
        print('주의: 동작 구간이 계획 동작 수보다 적음 — 로그가 잘렸거나 실행이 안 됨')

    kin = S.make_kinematics()
    # 3D/그래프용 간격: 2 ms, 로그가 길면 (랜덤 왕복 시험) 1.5만 점 안쪽이 되게 넓힘. 비교 계산은 1 ms 전체로
    step = max(2, int(np.ceil(len(t) / 15000)))
    default_box = plan['scene']['cuboid']['box']
    import torch
    from curobo.types import JointState
    q = torch.tensor(w['fb'][::step], device='cuda', dtype=torch.float32)
    tcp_fb = kin.compute_kinematics(JointState.from_position(q, joint_names=S.JOINTS)).tool_poses.get_link_pose(
        S.TOOL).position.reshape(-1, 3).cpu().numpy()

    summ = {'log_seconds': float(t[-1]), 'samples': int(len(t)), 'segments': [], 'moves': []}
    plan_out = []
    for k, mv in enumerate(moves):
        tp = np.arange(mv['n']) * mv['dt']
        qp = np.array(mv['position'])
        vp = np.array(mv['velocity'])
        obs_k = mv['obstacles'] if 'obstacles' in mv else [mv.get('box') or default_box]
        if 'clearance' in mv:
            plan_min = min(mv['clearance']['box']) * 1000
        else:      # plan_random.py (장애물 여러 개) 는 계획 거리를 따로 안 넣음 → 여기서 계산
            plan_min = min(S.clearances(kin, mv['position'], obs_k)['box']) * 1000 if obs_k else None
        item = {'name': mv['name'], 'plan_duration': mv['duration'], 'plan_points': mv['n'], 'plan_dt': mv['dt'],
                'plan_min_box_mm': plan_min}
        if k < len(segs):
            a, b = segs[k]
            t0 = t[a]
            tq = t[a:b] - t0
            qplan = interp_plan(tp, qp, vp, tq)
            cmd, fb = w['cmd'][a:b], w['fb'][a:b]
            e_cp = np.abs(cmd - qplan).max(axis=0)
            e_fc = np.abs(fb - cmd).max(axis=0)
            # 끝난 뒤 정착: 구간 끝 이후 200 ms 실제 위치와 계획 마지막 점
            b2 = min(len(t), b + 200)
            e_end = np.abs(w['fb'][b2 - 1] - qp[-1])
            # 실제 위치로 잰 장애물 거리: 이 동작 때 놓인 장애물로 (랜덤 시험은 동작마다 다름, 여러 개면 가장 가까운 것)
            cl_fb =S.clearances(kin, w['fb'][a:b], obs_k if obs_k else [{'dims': [0.01] * 3, 'pose': [0, 0, -5, 1, 0, 0, 0]}])
            box_fb = np.array(cl_fb['box'])
            i_min = int(box_fb.argmin())
            inpos_t = None
            ip = np.all(w['inpos'][a:] > 0.5, axis=1)
            if ip.any():
                inpos_t = float(t[a + int(np.argmax(ip))] - t0)
            # 계획 점 시각(knot)에서만 비교: StartCSplinePos 의 3차 스플라인은 점을 지나므로, 점 시각의 명령 위치 = 계획이어야 함.
            #  시작 시각은 "명령이 처음 바뀐 주기"로 잡아서 1 ms 안쪽 어긋남이 있을 수 있음 → ±5 ms 를 0.1 ms 간격으로 찾아 가장 맞는 값
            best = None
            for d in np.arange(-0.005, 0.0051, 0.0001):
                tk = t0 + d + tp
                ck = np.stack([np.interp(tk, t, w['cmd'][:, j]) for j in range(6)], axis=1)
                e = np.abs(ck - qp).max()
                if best is None or e < best[0]:
                    best = (e, d, ck - qp, tk)
            knot_err, knot_d, knot_e, knot_t = best
            item.update({'knot_max_err_mrad': float(knot_err * 1000), 'knot_time_shift_ms': float(knot_d * 1000)})
            item.update({'wmx_start_s': float(t0), 'wmx_duration': float(t[b - 1] - t0),
                         'cmd_minus_plan_max_mrad': r(e_cp * 1000, 4), 'fb_minus_cmd_max_mrad': r(e_fc * 1000, 4),
                         'final_err_mrad': r(e_end * 1000, 4), 'wmx_min_box_mm': float(box_fb[i_min] * 1000),
                         'wmx_min_box_link': cl_fb['box_link'][i_min], 'wmx_min_box_t': float(tq[i_min]),
                         'wmx_min_floor_mm': float(min(cl_fb['floor']) * 1000), 'first_inpos_s': inpos_t})
            plan_out.append({'name': mv['name'], 't0': float(t0), 'dt': mv['dt'], 'q': r(qp, 6), 'v': r(vp, 6),
                             'knot_t': r(knot_t, 5), 'knot_err': r(knot_e * 1000, 5),
                             'obstacles': [{'kind': o.get('kind', '상자'), 'dims': o['dims'], 'pose': o['pose']}
                                           for o in obs_k if o['pose'][2] > -1]})
        summ['moves'].append(item)
        print(json.dumps(item, ensure_ascii=False))
    for ev in ex.get('events', []):
        summ.setdefault('r2_events', []).append({kk: ev.get(kk) for kk in ('name', 'accepted', 'result_status', 'error_code')}
                                                | {'result_after_s': (ev.get('result_time') or 0) - ev['sent'],
                                                   'settled_after_s': (ev['settled'] - ev['sent']) if ev.get('settled') else None})
    if 'straight_line' in plan:
        sl = plan['straight_line']
        summ['straight_line_min_box_mm'] = min(sl['clearance']['box']) * 1000
        summ['straight_line_hits'] = sum(1 for d in sl['clearance']['box'] if d < 0)
    # 랜덤 왕복 시험: 모든 시도 (계획 실패 포함) + 실행된 것은 WMX 로그 결과를 붙임 → 뷰어의 동작 목록
    trials = None
    if plan.get('trials'):
        by_name = {m['name']: m for m in summ['moves']}
        trials = []
        for tr_ in plan['trials']:
            key = f'#{tr_["k"]:02d} {tr_["name"]}'
            m = by_name.get(key, {})
            obs_t = tr_.get('obstacles', [tr_['box']] if tr_.get('box') else [])
            obs_t = [o for o in obs_t if o['pose'][2] > -1]
            trials.append({'key': key, 'k': tr_['k'], 'dir': tr_['name'], 'plan_ok': tr_['plan_ok'],
                           'plan_ms': tr_['plan_time'] * 1000, 'duration': tr_.get('duration'),
                           'obstacles': [o.get('kind', '상자') for o in obs_t],
                           'executed': key in by_name, 'r2_ok': tr_.get('result_status') == 4,
                           'wmx_min_box_mm': m.get('wmx_min_box_mm'), 'plan_min_box_mm': tr_.get('plan_min_box_mm'),
                           'straight_min_box_mm': tr_.get('straight_min_box_mm')})
        tt = [x for x in trials if x['k'] > 0]
        summ['random'] = {'seed': plan.get('seed'), 'tries': len(tt), 'plan_ok': sum(x['plan_ok'] for x in tt),
                          'executed': sum(x['executed'] for x in tt), 'r2_ok': sum(x['r2_ok'] for x in tt),
                          'wmx_collisions': sum(1 for x in tt if x['wmx_min_box_mm'] is not None and x['wmx_min_box_mm'] < 0),
                          'blocking_boxes': sum(1 for x in tt if (x['straight_min_box_mm'] or 1) < 0),
                          'plan_ms_max': max((x['plan_ms'] for x in tt), default=None)}
        summ['trials'] = trials
        print('랜덤 시험:', json.dumps(summ['random'], ensure_ascii=False))

    # API 로그에서 R2 가 부른 WMX 함수 세기
    api = {}
    for f in out.glob('*.txt'):
        txt = f.read_text(errors='ignore')
        for key in ('StartCSplinePos', 'SetServoOn', 'ClearAmpAlarm', 'ImportAndSetAll', 'StartCommunication'):
            n = txt.count(key)
            if n:
                api[key] = max(api.get(key, 0), n)
    summ['api_calls'] = api
    # R2 가 WMX 에 실제로 넘긴 입력 (API 로그의 StartCSplinePos) ↔ cuRobo 계획: 점 수, 위치, 시각
    #   API 로그의 위치는 소수 6자리로 찍혀서 0.0005 mrad 까지는 반올림 차이
    calls = []
    for f in sorted(out.glob('*_api_*.txt')):
        calls = parse_api_csplines(f)
    summ['api_input'] = []
    for k, c in enumerate(calls):
        item = {'time': c['time'], 'numPoints': c['n'], 'axes': c['axes'], 'parsed_points': len(c['pos'])}
        if k < len(moves) and c['pos']:
            qp = np.array(moves[k]['position'])
            n = min(len(c['pos']), len(qp))
            pa = np.array(c['pos'])[:n, :6]
            tp = np.arange(n) * moves[k]['dt'] * 1000
            item.update({'plan_points': len(qp), 'pos_max_diff_mrad': float(np.abs(pa - qp[:n]).max() * 1000),
                         'time_max_diff_ms': float(np.abs(np.array(c['t_ms'][:n]) - tp).max())})
        summ['api_input'].append(item)
        print('API 입력:', json.dumps(item, ensure_ascii=False))
    (out / 'summary.json').write_text(json.dumps(summ, ensure_ascii=False, indent=1))

    # [바꿈 2026-10-13] 뷰어에는 3D 재생과 그래프 3개(관절 위치, 명령−계획, 실제−명령)에 쓰는 것만 넣는다.
    #  요약 수치·직선 보간·상자 거리는 사용자 요청으로 뷰어에서 뺐고 summary.json 에만 남긴다.
    title = ('CR3A 랜덤 장애물 A↔B 왕복 — WMX 로그 재생' if trials else 'CR3A 장애물 회피 — WMX 로그 재생') + f' ({out.name})'
    data = {
        'title': title,
        'joints': S.JOINTS, 'scene': plan['scene'], 'A_pos': plan['A_pos'], 'B_pos': plan['B_pos'],
        't': r(t[::step], 4), 'fb': r(w['fb'][::step], 6), 'cmd': r(w['cmd'][::step], 6),
        'tcp_fb': r(tcp_fb, 4), 'segments': [[float(t[a]), float(t[b - 1])] for a, b in segs],
        'plans': plan_out, 'move_names': [m['name'] for m in moves], 'trials': trials,
        'spheres': S.spheres_local(kin), 'urdf': urdf_chain(S.URDF),
    }
    html = TEMPLATE.replace('/*__DATA__*/', 'window.DATA = ' + json.dumps(data, separators=(',', ':')) + ';')
    (out / 'viewer.html').write_text(html)
    print('뷰어:', out / 'viewer.html', f'({(out / "viewer.html").stat().st_size / 1e6:.1f} MB)')


TEMPLATE = (Path(__file__).resolve().parent / 'viewer_template.html').read_text()

if __name__ == '__main__':
    main()
