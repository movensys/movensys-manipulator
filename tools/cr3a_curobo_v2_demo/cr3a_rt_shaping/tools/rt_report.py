#!/usr/bin/env python3
"""CR3A 실시간 회피 결과 (WMX 실행) → <결과 폴더>/report.html + summary.json
   python3 -m tools.rt_report <결과 폴더>   (wmx_r2 컨테이너 안 cr3a_rt_shaping 에서, admin. tools/rt_demo.sh 가 마지막에 부름)
읽는 것: wmx_axis.csv (WMX 엔진 축 로그 1 ms: 명령·실제 위치·속도, 시스템 시각), rt_run.json (rt_run 기록), r2_launch.log
이상적 모드 기록 (WMX 로그 없음) 은 python3 -m tools.rt_tools ideal 로 본다.
계산
  ① 시각 맞추기: WMX 로그의 SYSTEM_TIME (유닉스 시각, 100 ns) 을 그대로 쓰고, /joint_states 로 받은 실제 위치와 비교해 어긋남을 잼
  ② 실제 형상 거리 (summary.json 만): WMX 실제 위치(FBPOS)로 로봇 메시 꼭짓점을 FK → 장애물 상자·바닥까지 최소 거리 (10 ms 마다)
     [2026-10-16] 로봇 메시·충돌 구는 실행 기록의 모델 (rt_model.of_run: 'tool' = 끝단 공구 붙음, 기록에 없으면 'bare')
  ③ 보낸 명령 (rt_run 이 실제로 보낸 5 ms 궤적) vs WMX 명령 (CMDPOS·CMDVEL, 가속 = 10 ms 속도 변화): 관절 위치·속도·가속
  ④ 명령 전달: 엔진이 본 1 ms 가속 = |ΔCMDVEL| ÷ 1 ms (CyclicBuffer maxAcc 가 보는 값), 전달 지연 (WMX 명령이 보낸 명령과 가장 잘
     맞는 시각 차) 과 그 지연을 맞춘 뒤 남는 차이, 명령을 보낸 여유 (실행 시각 − 보낸 시각, 3 ms 아래면 안 보냄)
뷰어(rt_viewer_template.html): 3D 재생(로봇 = WMX 실제 위치, 실제 상자, 카메라 지도 칸 = 움직이는 물체·정지 물체로 본 칸, 팔 지우기 영역,
카메라) + 그래프 6 개. 수치는 summary.json. [2026-10-14 사용자 요청] 거리 그래프·표, 실제 − 명령, 단계별 가속은 뺌
[2026-10-27 R-1] 기록 읽기 (WMX 축 로그·실행된 u·전달 지연 맞춤·R2 로그) = rt_records, 실제 형상 거리 = tools.rt_geom (rt_tools 와 같은 것)
[2026-10-27 R-1 안전 ⑮] GUI 서버 기록을 바르게 읽음 (예전엔 일부 수치가 말없이 틀림):
  바닥 = 기록의 충돌 바닥 max(table_z, 0) (예전 z = 0) · 성형기를 바꿨으면 성형기마다 실행된 u (u_exec_k<i>.npz + u_exec.npz, 예전엔
  마지막 것만 — 3D 의 u·닿음·성형 확인이 그 앞을 못 봄) · 비상정지 감속 구간 (성형 확인에서 뺌, 3D 의 u·닿음은 보낸 감속 점으로) ·
  실제 카메라 기록은 기둥·시나리오 물체가 참값이 아님 → 물체 거리·닿음 없음 (바닥만) · 가속 한계 = 기록 모델의 cuRobo 설정 (예전 7.78 고정) ·
  A·B 손끝 = 기록 qA·qB 의 FK (GUI 기록은 목표를 사용자가 골라 안 그림) · 닿음 셈 = 로봇 탓 + 물체가 와서 (10-16 사용자 기준),
  카메라가 못 본 물체는 셈 안 함 (물체가 와서 닿은 것도)
"""
import argparse
import base64
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import rt_model
from rt_records import (JOINTS, estop_windows, fit_delay, floor_z, has_truth, is_gui, load_run, outside_windows, r2_counts, r2_max_acc,
                        read_text, read_wmx_csv, u_exec_runs, u_series)
from rt_scenarios import obstacle_pose
from rt_shaping import shape_fir
from tools.rt_geom import RobotMesh

HERE = Path(__file__).resolve().parent          # 보고서 틀 (rt_viewer_template.html) 이 같은 폴더


def urdf_chain(m: rt_model.Model):
    """보고서 3D 의 로봇: 관절 사슬 + 링크 메시 (base64 STL). 모델에 보고서용 가벼운 메시 폴더 (view_dir) 가 있으면 그것을 넣음
    (끝단 공구 계산용 메시는 닿음 판정 때문에 모서리 6 mm 로 나눠 무거움 — robot_tool/make_tool_meshes.py)"""
    root = ET.parse(m.urdf).getroot()
    joints = []
    for j in root.findall('joint'):
        o = j.find('origin')
        ax = j.find('axis')
        joints.append({'name': j.get('name'), 'type': j.get('type'), 'parent': j.find('parent').get('link'),
                       'child': j.find('child').get('link'),
                       'xyz': [float(v) for v in (o.get('xyz') if o is not None and o.get('xyz') else '0 0 0').split()],
                       'rpy': [float(v) for v in (o.get('rpy') if o is not None and o.get('rpy') else '0 0 0').split()],
                       'axis': [float(v) for v in (ax.get('xyz') if ax is not None else '0 0 1').split()]})
    meshes, colors = {}, {}
    for ln in root.findall('link'):
        col = ln.find('visual/material/color')
        if ln.get('name').startswith('tool_') and col is not None:          # 끝단 공구 부품만 제 색 (팔은 예전처럼 한 색)
            colors[ln.get('name')] = [float(v) for v in col.get('rgba').split()[:3]]
        g = ln.find('visual/geometry/mesh')
        if g is not None:
            f = m.asset / g.get('filename').replace('package://', '')
            if m.view_dir and (f.parent / m.view_dir / f.name).exists():
                f = f.parent / m.view_dir / f.name
            meshes[ln.get('name')] = base64.b64encode(f.read_bytes()).decode()
    return {'root': root.findall('link')[0].get('name'), 'joints': joints, 'meshes': meshes, 'colors': colors}


def model_info(m: rt_model.Model, sph):
    """보고서 머리말의 로봇 형상 줄: 모델 이름, 충돌 구 수 (팔 / 공구), 공구 덮개 두께 (robot_tool/tool_spheres.json → tool_cover_spheres.json)"""
    n_tool = sum(len(v) for k, v in sph.items() if k.startswith('tool_'))
    out = {'model': m.name, 'n_sph_arm': sum(len(v) for v in sph.values()) - n_tool, 'n_sph_tool': n_tool, 'tip_mm': round(m.tip_m * 1000, 1)}
    try:
        k = json.loads((m.asset / 'tool_spheres.json').read_text(encoding='utf-8'))['cover_k']
        c = json.loads((m.asset / 'tool_cover_spheres.json').read_text(encoding='utf-8'))[str(k)]
        out.update(thick_med_mm=c['thick_med_mm'], thick_max_mm=c['thick_max_mm'])
    except (OSError, KeyError, ValueError):
        pass
    return out


def shaping_check(out: Path, rec, wt, wq, w_cmdvel, legs, modes):
    """[2026-10-14] 입력성형 확인 (WMX 로그 기준). u_exec.npz = rt_run 이 기록한 실행된 성형 전 명령 u (1 ms, 세그먼트별).
    ① 전체 컨볼루션 y = u 전체를 성형기로 한 번에 (shape_fir = np.convolve: 실시간 계산 rt_stream 과 다른 경로)
    ② WMX 명령 (CMDPOS, 1 ms) 과 비교: 전달 지연 L 을 맞춘 뒤 차이 = WMX(t + L) − y(t) (mrad)
    ③ (summary 만) 가정 진동 모드 (성형기의 주파수·감쇠, ±10·20 %) 에 성형 전 u / WMX 명령의 가속을 넣은 관절 처짐 (mrad):
       e'' + 2ζω e' + ω² e = −q̈. 다리 도착 뒤 1 s 안 최대 (남는 진동) 와 움직이는 동안 최대.
    [2026-10-27 R-1 ⑮] GUI 서버 기록: 성형기마다 그 파일 (rec['kernels']) 을 그 성형기로 (예전 마지막 u_exec.npz 만), 비상정지 감속 구간
    (estop_windows — 성형을 건너뛴 명령) 은 뺌 (서버가 실행된 u 를 그 스탬프에서 끊어 보통은 뺄 점이 없음 — 끊김이 어긋나도 성형 차이로
    잡히지 않게). 성형기가 둘 이상이면 성형기마다 따로 (지연도 따로 맞춤 — R2 버퍼를 다시 열면 지연이 바뀜) 'kernels'."""
    runs = u_exec_runs(out, rec)
    if not runs:
        return None, None
    g = runs[-1][3]
    ts, Y, U, SG, KI = [], [], [], [], []
    for ki, (_, segs, h, gk) in enumerate(runs):
        for T0, b, q in segs:
            ts.append(T0 + np.arange(len(q)) * gk)
            Y.append(shape_fir(q, h, before=b))
            U.append(q)
            SG.append(np.full(len(q), len(SG)))
            KI.append(np.full(len(q), ki))
    ts, Y, U, SG, KI = np.concatenate(ts), np.concatenate(Y), np.concatenate(U), np.concatenate(SG), np.concatenate(KI)
    W = estop_windows(rec)
    m = (ts >= wt[0] + 0.1) & (ts <= wt[-1] - 0.2) & outside_windows(ts, W)
    ts, Y, U, SG, KI = ts[m], Y[m], U[m], SG[m], KI[m]
    vy = np.abs(np.gradient(Y, g, axis=0)).max(axis=1)
    mov = vy > 0.05
    sub = np.nonzero(mov)[0][::10]
    L = fit_delay(lambda L: float(np.median(np.abs(np.stack([np.interp(ts[sub] + L, wt, wq[:, j]) for j in range(6)], 1) - Y[sub]).max(axis=1))))
    Wq = np.stack([np.interp(ts + L, wt, wq[:, j]) for j in range(6)], 1)
    res = (Wq - Y) * 1000.0
    eff = np.abs(Wq - U).max(axis=1) * 1000.0              # 성형이 바꾼 양 (WMX 명령 − 성형 전 u)
    absr = np.abs(res).max(axis=1)
    out_d = {'delay_ms': round(L * 1000, 2), 'moving_s': round(float(mov.sum() * g), 2),
             'median_mrad': round(float(np.median(absr[mov])), 4), 'p99_mrad': round(float(np.percentile(absr[mov], 99)), 4),
             'max_mrad': round(float(absr[mov].max()), 4), 'max_all_mrad': round(float(absr.max()), 4),
             'wmx_minus_u_max_mrad': round(float(eff.max()), 2), 'wmx_minus_u_median_moving_mrad': round(float(np.median(eff[mov])), 2)}
    if W:
        out_d['estop_windows'] = {'n': len(W), 's': round(sum(b - a for a, b in W), 3)}     # 뺀 비상정지 감속 구간
    if len(runs) > 1:
        out_d['kernels'] = []
        for ki, (kd, segs, _, gk) in enumerate(runs):
            mk = KI == ki
            row = {'label': kd['label'], 'file': kd['file'], 'until': kd['until'], 'segments': len(segs),
                   'moving_s': round(float((mov & mk).sum() * gk), 2)}
            if (mov & mk).sum() > 20:
                sk = np.nonzero(mov & mk)[0][::10]
                Lk = fit_delay(lambda L: float(np.median(np.abs(np.stack([np.interp(ts[sk] + L, wt, wq[:, j]) for j in range(6)], 1) - Y[sk]).max(axis=1))))
                rk = np.abs(np.stack([np.interp(ts[mk] + Lk, wt, wq[:, j]) for j in range(6)], 1) - Y[mk]).max(axis=1) * 1000.0
                mm = mov[mk]
                row.update(delay_ms=round(Lk * 1000, 2), median_mrad=round(float(np.median(rk[mm])), 4),
                           p99_mrad=round(float(np.percentile(rk[mm], 99)), 4), max_mrad=round(float(rk[mm].max()), 4))
            out_d['kernels'].append(row)
    k = slice(None, None, 10)                                   # 그래프는 10 ms
    graph = {'t': np.round(ts[k], 4).tolist(), 'res': np.round(res[k], 3).tolist()}
    # ③ 가정 진동 모드 (summary 만. 잔류 진동 검증은 사용자가 함 — 2026-10-16, 보고서 그래프·보고에 쓰지 않음)
    vib = {}
    if modes:
        # WMX 명령 가속: CMDVEL 1 ms 차분, 성형 전 u 가속: 위치 2 차 차분 — 같은 시각 격자 (ts) 에서
        a_w = np.stack([np.interp(ts + L, wt[1:], np.diff(w_cmdvel[:, j]) / np.diff(wt)) for j in range(6)], 1)
        a_u = np.zeros_like(U)
        for i in np.unique(SG):                                 # 세그먼트 (다시 계획 사이) 마다 따로: 사이의 서 있던 시간은 기록에 없음
            k = SG == i
            if k.sum() >= 3:
                a_u[k] = np.gradient(np.gradient(U[k], g, axis=0), g, axis=0)
        ends = [x['t1'] for x in legs if x.get('t1')]
        for name, ff, zz in modes:
            for sc in (0.8, 0.9, 1.0, 1.1, 1.2):
                w = 2 * np.pi * ff * sc
                row = {}
                # e'' + 2ζω e' + ω² e = −q̈ 를 1 ms 영차 유지로 이산화 → 관절마다 IIR (scipy)
                from scipy.signal import cont2discrete, lfilter
                bd, ad, _ = cont2discrete(([-1.0], [1.0, 2 * zz * w, w * w]), g, method='zoh')
                for lab, a in (('u', a_u), ('wmx', a_w)):
                    E = np.zeros_like(a)
                    for i in np.unique(SG):
                        k = SG == i
                        E[k] = np.stack([lfilter(np.ravel(bd), ad, a[k, j]) for j in range(6)], 1)
                    after = np.zeros(len(ts), bool)
                    for t1 in ends:
                        after |= (ts >= t1) & (ts < t1 + 1.0)
                    row[lab] = {'max_mrad': round(float(np.abs(E).max() * 1000), 3),
                                'after_arrive_mrad': round(float(np.abs(E[after]).max() * 1000), 4) if after.any() else None}
                vib[f'{name}@{ff:g}Hz×{sc:g}'] = row
    out_d['virtual_mode'] = vib
    return out_d, graph


def acc_limits(m: rt_model.Model):
    """모델 cuRobo 설정의 관절 가속 한계 (rad/s², cspace max_acceleration, 관절 차례) — 그래프의 '로봇 가속 한계'. [2026-10-27 R-1 ⑮] 예전 7.78
    고정 (bare 설정의 최대). GUI 서버는 이 한계 × LIMIT_FRAC (0.95) 로 계획·MPC (rt_server) — 그래프 선은 로봇 한계 그대로. 못 읽으면 None."""
    import yaml
    try:
        y = yaml.safe_load(open(m.yml))
        return [float(v) for v in (y.get('robot_cfg') or y)['kinematics']['cspace']['max_acceleration']]
    except (OSError, KeyError, TypeError, ValueError):
        return None


def header_info(st, summ):
    """보고서 머리말 값 (rt_viewer_template.html 이 말머리 목록으로 그림). st = 기록 settings, summ = summary.json 내용."""
    num = lambda x: float(f'{x:.1f}')  # noqa: E731
    return {'cam_fps': num(st['cam_fps']), 'rate_hz': num(st['rate_hz']), 'jerk': num(st.get('jerk', 0)), 'scenario': st.get('scenario', 'moving5'),
            'act_new_cm': num(st['act_new'] * 100), 'act_static_cm': num(st['act_static'] * 100), 'mask_cm': num(st['mask'] * 100),
            'guards': summ['guard_brakes'], 'skipped': summ['skipped_msgs'], 'legs_ok': summ['legs_ok'],
            'guard_kind': summ.get('guard_kind'), 'late_sends': summ.get('late_sends'), 'contacts': {k: v for k, v in (summ.get('robot_contacts') or {}).items() if k != 'episodes'} or None,
            'leg_sum_s': round(sum(L['duration'] or 0 for L in summ['legs'] if not L['label'].startswith('시작')), 1),
            'leg_sum_n': sum(1 for L in summ['legs'] if not L['label'].startswith('시작')),     # [2026-10-26] GUI 기록은 다리 수가 16 이 아님
            'style': st.get('style', 'carrot'), 'shaping': (st.get('shaping') or {}).get('label', 'none'),
            'mpc_rate_hz': num(((st.get('config') or {}).get('cmpc') or {}).get('rate_hz', st['rate_hz'])),
            'shaping_delay_ms': (st.get('shaping') or {}).get('mean_delay_ms'), 'shaping_len_ms': (st.get('shaping') or {}).get('duration_ms'),
            # [2026-10-16] 정지·움직임 구분 없는 지도, 멈추지 않는 다시 계획 (rt_config MapCfg.uniform, PlanCfg.ahead_check·live_replan)
            'uniform': bool(st.get('map_uniform', False)), 'live_replan': bool(st.get('live_replan', False)),
            'ahead_check': bool(st.get('ahead_check', False)), 'live': summ.get('live'), 'live_late': summ.get('live_late'),
            'act_floor_cm': num(float(((st.get('config') or {}).get('dist') or {}).get('act_floor', st['act_static'])) * 100),
            'ahead_from_s': ((st.get('config') or {}).get('plan') or {}).get('ahead_from_s'),
            'ahead_to_s': ((st.get('config') or {}).get('plan') or {}).get('ahead_to_s'),
            # [2026-10-16 작업기록 5-21] 서 있을 때 다가오는 물체: 물러나기 · 경계 채우기 기울기 · 성형 규칙 (2)(3) 다가갈 때만
            'back_off': bool(st.get('back_off', False)), 'fill_grad': bool(st.get('fill_grad', False)),
            'back_style': ((st.get('config') or {}).get('plan') or {}).get('back_style', 'path'),      # [2026-10-17] 'yield' = 그 자리에서 비키기
            'back_max_s': ((st.get('config') or {}).get('plan') or {}).get('back_max_s'),
            'cp_dyn_approach': bool(st.get('cp_dyn_approach', False)), 'back': summ.get('back'),
            # [2026-10-27 R-1 ⑮] GUI 서버 기록: 카메라 (실제면 물체 참값 없음), 충돌 바닥, 성형기 목록 (바꿨으면), 비상정지 감속 구간 수
            'camera': summ.get('camera'), 'truth': summ.get('ground_truth'), 'floor_z_mm': summ.get('floor_z_mm'),
            'kernels': summ.get('kernels'), 'estop_n': len(summ.get('estop_windows') or [])}


def live_late(rec, late_ms=3.0):
    """[2026-10-16] 늦은 주기 (보낼 때 스탬프까지 여유 < late_ms = 안 보냄) 중 멈추지 않는 다시 계획이 도는 동안 (요청 → 새 경로·실패 사건)
    인 것 — 실시간에서 계획기 스레드가 GPU·파이썬 실행권을 같이 써서 제어 주기가 느려지는지. 실시간 기록이 아니면 None."""
    spans, t_req = [], None
    for e in rec.get('events', []):
        if '→ 멈추지 않고 다시 계획' in e['text']:
            t_req = e['t']
        elif t_req is not None and ('새 경로로 (멈추지 않음' in e['text'] or '다시 계획 실패 (멈추지 않음' in e['text']):
            spans.append((t_req, e['t']))
            t_req = None
    cy = [c for c in rec.get('cycles', []) if c.get('lead_ms') is not None]
    if not cy:
        return None
    t = np.array([c['t'] for c in cy])
    late = np.array([c['lead_ms'] for c in cy]) < late_ms
    inside = np.zeros(len(t), bool)
    for a, b in spans:
        inside |= (t >= a) & (t <= b + 0.05)
    cyc = np.array([c['cyc_ms'] for c in cy])
    return {'late': int(late.sum()), 'late_during_plan': int((late & inside).sum()), 'plan_time_s': round(sum(b - a for a, b in spans), 2),
            'cyc_ms_median_plan': round(float(np.median(cyc[inside])), 1) if inside.any() else None,
            'cyc_ms_median_else': round(float(np.median(cyc[~inside])), 1) if (~inside).any() else None}


def rc_cfg_ms(st, key, default=0.003):
    """기록된 메시지 시간표 설정 (rt_config.StreamCfg, s) → ms. 설정이 없는 옛 기록은 default."""
    return round(float(((st.get('config') or {}).get('stream') or {}).get(key, default)) * 1000, 2)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('out', help='결과 폴더 (rt_run.json · wmx_axis.csv · u_exec*.npz · r2_launch.log) — report.html · summary.json 을 여기에 씀')
    out = Path(ap.parse_args(argv).out)
    rec = load_run(out)
    w = read_wmx_csv(out / 'wmx_axis.csv')
    st = rec['settings']
    cyc = rec['cycles']                                        # MPC 주기 (20 Hz)
    fr = rec.get('frames') or cyc                             # 카메라 영상 (15 fps): 측정 관절·지도 거리
    rc_t = np.array([c['t'] for c in fr])
    rc_q = np.array([c['q'] for c in fr])
    t_go = rec.get('t_go')
    t_end = rec.get('t_go_end')                                # [2026-10-26] GUI 실행: 시험 물체를 치운 시각 (그 뒤는 물체 없음)

    # ① 시각: WMX SYSTEM_TIME vs 회피 프로그램 (/joint_states 받은 값). 같은 시각의 WMX 실제 위치와 비교
    fb_at = np.stack([np.interp(rc_t, w['unix'], w['fb'][:, j]) for j in range(6)], axis=1)
    lag_best = None
    for lag in np.arange(-0.10, 0.10001, 0.002):            # 회피 프로그램이 받은 값 = WMX 의 (t - lag) 값
        f = np.stack([np.interp(rc_t - lag, w['unix'], w['fb'][:, j]) for j in range(6)], axis=1)
        e = float(np.median(np.abs(f - rc_q).max(axis=1)))
        if lag_best is None or e < lag_best[1]:
            lag_best = (float(lag), e)
    sync = {'js_minus_wmx_median_mrad': float(np.median(np.abs(fb_at - rc_q).max(axis=1)) * 1000),
            'best_lag_ms': lag_best[0] * 1000, 'at_best_median_mrad': lag_best[1] * 1000}

    # 보고 구간: 회피 프로그램 기록 처음 ~ 끝 (WMX 로그는 그보다 김)
    m = (w['unix'] >= rc_t[0] - 0.2) & (w['unix'] <= rc_t[-1] + 0.5)
    ix = np.nonzero(m)[0]
    T1 = w['unix'][ix]
    step = 10                                                  # 10 ms 로 줄여 보고 (가속은 1 ms 로 계산)
    sel = ix[::step]
    T = w['unix'][sel]
    fb, cmd = w['fb'][sel], w['cmd'][sel]

    # ② 실제 형상 거리 (10 ms). [2026-10-27 R-1 ⑮] 바닥 = 기록의 충돌 바닥 (실제 카메라는 테이블 윗면), 실제 카메라 기록은 물체 참값 없음
    M = rt_model.of_run(rec)
    md = RobotMesh(M)
    fz, truth = floor_z(rec), has_truth(rec)
    obs = ([{'name': st['pillar'][0], 'dims': st['pillar'][2], 'static': True, 'center': st['pillar'][1]}] +
           [{'name': o['name'], 'dims': o['dims'], 'static': False, 'wps': o['wps']} for o in st['obstacles']]) if truth else []
    dist = [[] for _ in obs]
    floor = []
    for t, q in zip(T, fb):
        p = md.points(q)                                      # FK 한 번, 장애물마다 거리
        floor.append(round(md.floor_dist(p, fz) * 1000, 1))
        for o, dd in zip(obs, dist):
            if o['static']:
                c = o['center']
            elif t_go is None or t < t_go or (t_end is not None and t >= t_end):
                dd.append(None)
                continue
            else:
                c = obstacle_pose(o['wps'], t - t_go)
            dd.append(round(md.box_dist(p, c, o['dims']) * 1000, 1))
    dist_np = [np.array([np.nan if v is None else v for v in dd]) for dd in dist]
    moving = np.nanmin(np.stack(dist_np[1:]), axis=0) if len(dist_np) > 1 else np.full(len(T), np.nan)

    # ③ 지도 거리 − 실제 (회피 프로그램 주기 시각에서)
    true_at_rc = np.interp(rc_t, T, np.nan_to_num(moving, nan=np.inf))
    near_new = np.array([np.nan if c['near_new_mm'] is None else c['near_new_mm'] for c in fr])
    mmt = np.where(np.isfinite(true_at_rc) & np.isfinite(near_new), near_new - true_at_rc, np.nan)

    # ④ 엔진이 본 1주기 가속 (1 ms) → 10 ms 창의 최댓값, 실제 − 명령
    v1 = w['cmdvel'][ix]
    a1 = np.abs(np.diff(v1, axis=0)) / 0.001
    nwin = len(a1) // step
    acc = a1[:nwin * step].reshape(nwin, step, 6).max(axis=1)
    t_acc = T1[1:nwin * step + 1:step]
    fbmc = (w['fb'][ix] - w['cmd'][ix]) * 1000

    # ⑤ 단계별 가속 (관절 중 최대, rad/s²): MPC 출력 → JTC 기준 → WMX 엔진 명령. 어디서 가속이 커지는지 보려고
    #    MPC = 보낸 점들의 가속 |a| 최대와 연속 두 주기 속도 차 ÷ 주기 중 큰 것 (시각 = 그 메시지를 JTC 가 실행할 스탬프)
    #    JTC = controller_state 의 기준 속도 차 ÷ 스탬프 차, WMX = |ΔCMDVEL| ÷ 1 ms (10 ms 창 최대)
    # [2026-10-13 고침] 주기 사이 속도 차 ÷ 주기는 뺌: 늦어서 안 보낸 메시지·감속으로 바뀐 주기에서는 뜻이 없어 14.7 같은 가짜 값이 나왔다.
    #  보낸 메시지의 점들 가속 최대만 (안 보낸 주기는 빈칸)
    sa = np.array([np.nan if c.get('skipped') else max(c.get('seq_a_max', 0.0), float(np.abs(c['cmd_a']).max())) for c in cyc])
    mpc_acc = sa[1:]
    jt = rec.get('jtc') or []
    jtc_t, jtc_acc = [], []
    if len(jt) > 2:
        jst = np.array([x[0] for x in jt])
        jv = np.array([x[3] if x[3] is not None else [np.nan] * 6 for x in jt], dtype=float)
        o = np.argsort(jst)
        jst, jv = jst[o], jv[o]
        keep = np.diff(jst) > 1e-4
        jtc_t = jst[1:][keep]
        jtc_acc = (np.abs(np.diff(jv, axis=0)).max(axis=1) / np.maximum(np.diff(jst), 1e-4))[keep]
    chain = {'mpc': float(np.nanmax(mpc_acc)) if np.isfinite(mpc_acc).any() else None,
             'jtc': float(np.nanmax(jtc_acc)) if len(jtc_acc) else None,
             'wmx': float(acc.max()), 'jtc_samples': len(jtc_acc),
             'jtc_rate_hz': float(1.0 / np.median(np.diff(jtc_t))) if len(jtc_t) > 2 else None}

    # ⑥ 보낸 명령 (rt_run 의 sent_traj: 실제로 실행된 메시지 점 5 ms + 멈출 때 꼬리) — 그 전 기록은 보낸 주기의 다음 스탬프 상태 (20 Hz)
    sent = rec.get('sent_traj')
    if sent:
        s_t, s_q, s_v, s_a = (np.array(sent[k], float) for k in ('t', 'q', 'v', 'a'))
    else:
        sc = [c for c in cyc if not c.get('skipped')]
        s_t = np.array([c.get('stamp', c['t']) for c in sc]) + 1.0 / st['rate_hz']
        s_q, s_v, s_a = (np.array([c[k] for c in sc], float) for k in ('cmd', 'cmd_v', 'cmd_a'))
    o = np.argsort(s_t, kind='stable')
    s_t, s_q, s_v, s_a = s_t[o], s_q[o], s_v[o], s_a[o]
    sw = (s_t >= T[0]) & (s_t <= T[-1])
    s_t, s_q, s_v, s_a = s_t[sw], s_q[sw], s_v[sw], s_a[sw]

    # ⑦ WMX 명령 속도 (10 ms 마다) 와 가속 (10 ms 동안 속도 변화 ÷ 10 ms, 가운데 시각)
    wv = w['cmdvel'][sel]
    k0 = np.arange(0, len(ix) - step, step)
    wa_t = 0.5 * (T1[k0] + T1[k0 + step])
    wa = (w['cmdvel'][ix][k0 + step] - w['cmdvel'][ix][k0]) / (step * 1e-3)

    # ⑧ 전달 지연: WMX 명령 위치 (t + 지연) 가 보낸 명령 위치 (t) 와 가장 잘 맞는 지연 (움직이는 점들에서 관절 최대 차의 중앙값이 최소).
    #    지연을 맞춘 뒤 남는 차이 = R2 의 1 ms 변환·JTC 보간이 궤적 모양을 바꾼 양
    wt, wq = w['unix'], w['cmd']
    mov = np.abs(s_v).max(axis=1) > 0.05
    lag, res = {'ms': None}, None
    if mov.sum() > 20:
        tm = s_t[mov]
        err = lambda L: float(np.median(np.abs(np.stack([np.interp(tm + L, wt, wq[:, j]) for j in range(6)], 1) - s_q[mov]).max(axis=1)))  # noqa: E731
        L_best = fit_delay(err)
        e_best = err(L_best)
        res = (np.stack([np.interp(s_t + L_best, wt, wq[:, j]) for j in range(6)], 1) - s_q) * 1000      # mrad
        lag = {'ms': round(L_best * 1000, 2), 'median_mrad': round(e_best * 1000, 3),
               'p99_mrad': round(float(np.percentile(np.abs(res[mov]).max(axis=1), 99)), 3),
               'max_mrad': round(float(np.abs(res[mov]).max()), 3)}

    # ⑩ 입력성형 확인 (WMX 명령 − 실행된 u 전체 컨볼루션) + 가정 진동 모드 (summary)
    modes = [tuple(m) for m in ((st.get('shaping') or {}).get('modes') or [])]
    shp, shp_graph = shaping_check(out, rec, wt, wq, w['cmdvel'], rec['legs'], modes)

    # ⑪ [2026-10-16] 3D 의 성형 전 명령 u (실행된 u, 1 ms 기록 → 보고 시각 10 ms) 와 닿음 (실제 형상, 로봇 탓·성형 탓 가르기 = rt_tools contacts)
    #  [2026-10-27 R-1 ⑮] u = 모든 성형기의 실행된 u + 비상정지 감속 구간 (보낸 감속 점), 닿음도 같은 범위 (rt_tools.robot_contacts)
    u_T, contacts = None, []
    us = u_series(out, rec)
    if us is not None:
        from tools.rt_tools import robot_contacts
        ut, uq = us
        u_T = np.stack([np.interp(T, ut, uq[:, j]) for j in range(6)], 1)
        cr = robot_contacts(out, md)
        Tf = (rec.get('frames') or cyc)[0]['t']
        for e in cr['episodes']:
            # 사용자 기준: 로봇 탓 + (10-16 부터) 멈춘 로봇에 물체가 와서 닿음 = 셈. 카메라가 못 본 물체 (못 보는 면·팔 지우기) 는 어느 쪽이든 셈 안 함
            kind = ('이어짐' if e.get('robot') is None else
                    '카메라 못 봄' if e.get('seen') is False else
                    '성형 탓' if e.get('robot') and str(e.get('cause', '')).startswith('성형 탓') else
                    '로봇 탓' if e.get('robot') else '물체가 와서')
            contacts.append({'t': round(Tf + e['t'], 3), 'dur': e['dur_s'], 'obj': e['obj'], 'kind': kind,
                             'y_mm': e['min_y_mm'], 'u_mm': e.get('min_u_mm'),
                             'cause': e.get('cause') if e.get('robot') else None,      # 성형 탓 가르기는 로봇이 움직여 닿은 것만 뜻이 있음
                             'v': e.get('v_rad_s')})

    # ⑨ 명령을 보낸 여유 (실시간 실행만): 주기마다 실행 시각(스탬프) − 메시지를 다 만든 시각. 3 ms 아래면 안 보냄 (앞 메시지 계속)
    lead_c = [c for c in cyc if c.get('lead_ms') is not None]

    # 다리
    legs = []
    floor_np = np.array(floor)
    for L in rec['legs']:
        t0, t1 = L['t0'], L.get('t1')
        t1e = t1 if t1 else (L['t0'] + max(L.get('duration') or 0, 25.0))
        mm_ = (T >= t0) & (T <= t1e)
        mv, sv = (None, None), (None, None)
        if mm_.any():
            for o, d in list(zip(obs, dist_np)) + [({'name': '바닥', 'static': True}, floor_np)]:
                if np.isfinite(d[mm_]).any():
                    v = float(np.nanmin(d[mm_]))
                    cur = sv if o['static'] else mv
                    if cur[0] is None or v < cur[0]:
                        if o['static']:
                            sv = (v, o['name'])
                        else:
                            mv = (v, o['name'])
        guards = sum(1 for c in cyc if c.get('guard') and t0 <= c['t'] <= t1e)
        legs.append({'label': L['label'], 't0': t0, 't1': t1, 'ok': L['ok'], 'why': L.get('why'),
                     'duration': L.get('duration'), 'plans': len(L['plans']), 'plan_s': L['plans'],
                     'min_move_mm': mv[0], 'min_move_obj': mv[1], 'min_static_mm': sv[0], 'min_static_obj': sv[1], 'guards': guards})

    acc_lim = acc_limits(M)

    # R2 로그
    r2 = read_text(out / 'r2_launch.log')
    max_acc = r2_max_acc(r2)
    r2s = r2_counts(r2)

    # 요약 수치 (summary.json)
    go = T >= (t_go or T[0])
    summ = {'folder': out.name, 'settings': {k: st.get(k) for k in ('robot', 'scenario', 'legs', 'rate_hz', 'lead_s', 'jerk',
                                                                   'act_new', 'act_static', 'mask', 'voxel', 'cam_fps')},
            'guard_brakes': sum(1 for c in cyc if c.get('guard')), 'skipped_msgs': sum(1 for c in cyc if c.get('skipped')),
            # [2026-10-16] 안 보낸 메시지 = 늦어서 + 안전 검사가 '앞 메시지 계속' 을 고름 → 나눠 셈. 닿음 = 로봇이 움직여 닿은 것만 (사용자 기준)
            'live': rec.get('live'),                                   # 멈추지 않는 다시 계획 {요청, 새 경로, 실패, 버림}
            'live_late': live_late(rec),                               # 늦은 주기 중 다시 계획 (계획기 스레드) 이 도는 동안
            'back': rec.get('back'),                                   # 물러나기 {시작, 새 경로로 끝, 다시 앞으로} (rt_run._back_check)
            'guard_kind': {'brake': sum(1 for c in cyc if c.get('guard') == 'brake'), 'keep': sum(1 for c in cyc if c.get('guard') == 'keep')},
            'late_sends': sum(1 for c in cyc if c.get('skipped') and c.get('guard') != 'keep'),
            'robot_contacts': {'counted': sum(1 for e in contacts if e['kind'] in ('로봇 탓', '성형 탓', '물체가 와서')),
                               'robot': sum(1 for e in contacts if e['kind'] in ('로봇 탓', '성형 탓')),
                               'shaping': sum(1 for e in contacts if e['kind'] == '성형 탓'),
                               'object_came': sum(1 for e in contacts if e['kind'] == '물체가 와서'),
                               'camera_blind': sum(1 for e in contacts if e['kind'] == '카메라 못 봄'), 'episodes': contacts} if u_T is not None else None,
            'urdf_acc_max': max(acc_lim) if acc_lim else None, 'robot_acc_limits': acc_lim,
            # [2026-10-27 R-1 ⑮] 이 보고서가 무엇을 참값으로 봤나: 카메라 (실제면 물체 거리·닿음 없음), 충돌 바닥 높이, 성형기 목록, 비상정지 감속
            'camera': st.get('camera', 'virtual'), 'ground_truth': truth, 'floor_z_mm': round(fz * 1000, 1),
            'kernels': [{'label': k['label'], 'until': k['until'], 'file': k['file']} for k in (rec.get('kernels') or [])] or None,
            'estop_windows': estop_windows(rec),
            'r2_max_acc': max_acc, 'r2_log': r2s, 'time_sync': sync,
            'legs': legs, 'legs_ok': f"{sum(L['ok'] for L in legs)}/{len(legs)}",
            'min_dist_mm': {**{o['name']: (float(np.nanmin(d[go])) if np.isfinite(d[go]).any() else None) for o, d in zip(obs, dist_np)},
                            '바닥': float(np.min(np.array(floor)))},
            'contact_ms': {**{o['name']: int(np.sum(d[go] < 0) * step) for o, d in zip(obs, dist_np)}, '바닥': int(np.sum(np.array(floor) < 0) * step)},
            'map_minus_true_mm': {'median': float(np.nanmedian(mmt)) if np.isfinite(mmt).any() else None,
                                  'p95_abs': float(np.nanpercentile(np.abs(mmt), 95)) if np.isfinite(mmt).any() else None},
            'mpc_ms': {'median': float(np.median([c['mpc_ms'] for c in cyc])), 'max': float(max(c['mpc_ms'] for c in cyc))},
            'map_ms': {'median': float(np.median([c['map_ms'] for c in fr])), 'max': float(max(c['map_ms'] for c in fr))},
            'frames': len(fr), 'cycles': len(cyc),
            'engine_acc_max': [round(float(x), 3) for x in acc.max(axis=0)],
            'chain_acc_max': chain,
            'lead_ms': ({'min': min(ld), 'median': float(np.median(ld))} if (ld := [c['lead_ms'] for c in cyc if c.get('lead_ms') is not None]) else None),
            'late_cycles': sum(1 for c in cyc if c.get('late_s', c.get('resync'))),   # 계산 시작이 늦어 스탬프를 미룬 주기 (2026-10-13 전 기록은 'resync')
            'fb_minus_cmd_max_mrad': [round(float(x), 4) for x in np.abs(fbmc).max(axis=0)],
            'send_delay': lag, 'sent_points': len(s_t), 'sent_source': 'sent_traj' if sent else 'cycles',
            'style': st.get('style', 'carrot'), 'shaping': st.get('shaping'), 'shaping_check': shp,
            'shaping_rule': {k: v for k, v in ((st.get('config') or {}).get('guard') or {}).items() if k.startswith('cp')},   # 성형 여유 보존 설정
            'events': rec['events']}
    (out / 'summary.json').write_text(json.dumps(summ, ensure_ascii=False, indent=1), encoding='utf-8')

    # 뷰어
    due_t = np.array([c.get('stamp', c['t']) for c in cyc]) + 1.0 / st['rate_hz']   # 주기 끝 상태(cmd)를 JTC 가 지나갈 시각
    r = lambda x, nd: np.round(np.asarray(x, dtype=float), nd).tolist()  # noqa: E731
    leg_view = [{k: L[k] for k in ('label', 't0', 't1', 'ok', 'why', 'duration', 'plans', 'guards')} for L in legs]
    data = {
        'title': f'CR3A{" + 끝단 공구 (그리퍼·D455)" if M.name == "tool" else ""} 실시간 장애물 회피 — WMX 로그 재생 ({out.name})',
        'info': {**header_info(st, summ), **model_info(M, spheres_local(M))},          # 머리말 (뷰어가 말머리 목록으로 씀)
        'joints': JOINTS, 't': r(T, 4), 'fb': r(fb, 5), 'cmd': r(cmd, 5), 'cmdv': r(wv, 4), 't_go': t_go, 't_go_end': t_end,
        'cmda': {'t': r(wa_t, 4), 'a': r(wa, 3)},
        'sent': {'t': r(s_t, 4), 'q': r(s_q, 5), 'v': r(s_v, 4), 'a': r(s_a, 3)},
        'lag': {**lag, 'res': None if res is None else r(res[::2], 2), 't': r(s_t[::2], 4)},
        'lead': {'t': r([c.get('stamp', c['t']) for c in lead_c], 4), 'ms': [c['lead_ms'] for c in lead_c],
                 'late': [bool(c.get('skipped')) and c.get('guard') != 'keep' for c in lead_c],
                 'keep': [c.get('guard') == 'keep' for c in lead_c], 'limit_ms': rc_cfg_ms(st, 'late_send_s')},
        'obstacles': obs, 'cam': {'pose': st['cam_pose'], 'K': st['cam_K']}, 'floor_z': fz, 'truth': truth,
        'vox': {'grid': st.get('grid'), 'static': rec.get('static_vox'), 'snaps': rec['newvox'], 'mask': st['mask']},
        # A·B 손끝 (파란 점): 기록 qA·qB 의 FK ([R-1 ⑮] 예전 고정 좌표). GUI 기록은 목표를 사용자가 골라 안 그림. qA 없는 옛 기록은 예전 좌표
        'ab_tool': [] if is_gui(rec) else ([np.round(md.tip(st[k]), 4).tolist() for k in ('qA', 'qB')] if 'qA' in st
                                          else [[0.42, 0.34, 0.20], [0.42, -0.34, 0.20]]),
        'urdf': urdf_chain(M), 'spheres': spheres_local(M), 'model': M.name, 'tip_m': M.tip_m,
        'legs': leg_view,
        'rc': {'t': r(due_t, 4), 'cmd': [c['cmd'] for c in cyc]},
        'acc': r(acc, 3), 't_acc': r(t_acc, 4), 'max_acc': max_acc or 0.0, 'urdf_acc': max(acc_lim) if acc_lim else None,
        'shp': None if shp is None else {**{k: v for k, v in shp.items() if k != 'virtual_mode'}, **shp_graph,
                                         'label': (st.get('shaping') or {}).get('label'),
                                         'mean_delay_ms': (st.get('shaping') or {}).get('mean_delay_ms')},
        'u': None if u_T is None else r(u_T, 5),                  # 3D 초록 = 성형 전 명령 u (10 ms, T 와 같은 칸)
        'contacts': contacts,
    }
    tpl = (HERE / 'rt_viewer_template.html').read_text(encoding='utf-8')
    html = tpl.replace('/*__DATA__*/', 'window.DATA = ' + json.dumps(data, separators=(',', ':'), ensure_ascii=False) + ';')
    (out / 'report.html').write_text(html, encoding='utf-8')
    print(json.dumps({k: summ[k] for k in ('legs_ok', 'min_dist_mm', 'contact_ms', 'map_minus_true_mm', 'mpc_ms', 'r2_log', 'guard_brakes', 'skipped_msgs',
                                           'map_ms', 'engine_acc_max', 'chain_acc_max', 'lead_ms', 'late_cycles', 'time_sync',
                                           'send_delay', 'sent_source', 'shaping_check')}, ensure_ascii=False))
    print('보고서:', out / 'report.html', f'({(out / "report.html").stat().st_size / 1e6:.1f} MB)')


def spheres_local(m: rt_model.Model):
    """MPC 모델 충돌 구 (링크 좌표) — 모델의 cuRobo 설정 (bare: cr3a_sim/cr3a_curobo.yml, tool: robot_tool/cr3a_tool_curobo.yml)"""
    import yaml
    y = yaml.safe_load(open(m.yml))
    out = {}
    for ln, lst in (y['kinematics'].get('collision_spheres') or {}).items():
        out[ln] = [[round(c, 5) for c in s['center']] + [round(s['radius'], 5)] for s in lst if s['radius'] > 0]
    return out


if __name__ == '__main__':
    main()
