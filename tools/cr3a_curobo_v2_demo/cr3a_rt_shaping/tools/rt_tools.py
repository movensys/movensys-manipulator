#!/usr/bin/env python3
"""rt_run 결과 분석 도구 (wmx_r2 컨테이너 안). 2026-10-13 조사 때 쓴 일회성 스크립트들을 모음.
[2026-10-27 R-1] tools/ 로 옮김 — 저장소 맨 위 (cr3a_rt_shaping) 에서 python3 -m tools.rt_tools <명령> … 로 돌림 (sys.path 고치기 없음)

  python3 -m tools.rt_tools compare <폴더A> <폴더B>     두 실행 기록이 같은가 (리팩터링 검증: 이상적 모드는 결정적)
  python3 -m tools.rt_tools ideal <폴더>...              실행 요약: 다리, 관절 총 이동, MPC 한계 넘음·불가능 해, 안전 검사, 실제 형상 최소 거리
  python3 -m tools.rt_tools scenario [이름]              물체 시나리오 미리 보기: 기둥·바닥 겹침, A·B 자세·이동 자세와의 거리, 지도 안 비율
  python3 -m tools.rt_tools messages <폴더>              보낸 메시지 연속성 (--record-sent 로 돌린 기록): 메시지 안 가속, 갈아타기 점 차이
  python3 -m tools.rt_tools peaks <폴더>                 (WMX) 엔진 1 ms 가속 최고점들과 그때의 주기 상태
  python3 -m tools.rt_tools contact <폴더> <물체 이름>   한 물체에 가장 가까웠던 때 앞뒤 (WMX 로그가 있으면 실제 위치, 없으면 기록 관절값)
  python3 -m tools.rt_tools arrival <폴더>               다리마다 도착 때 목표를 지나친 양 (로봇·명령·MPC 당근)
  python3 -m tools.rt_tools table <폴더>...              [2026-10-14] 비교 표 (한 줄씩, 끝에 JSON): 시나리오·성형, 다리, 다리 시간 합, 관절 이동,
                                                         안전 검사, 움직이는 물체·기둥·바닥 최소 거리와 닿은 영상 수 (실제 형상, 영상 시각 관절값)
  python3 -m tools.rt_tools shaping <폴더>               [2026-10-14] 입력성형 확인 (이상적 모드): 실행된 u (u_exec.npz) 전체를 한 번에 컨볼루션한 것
                                                         = 실제로 보낸 메시지 점 (sent_traj) 인가 (차이 ≈ 0 이어야 함). 성형이 바꾼 양 (y − u) 도
  python3 -m tools.rt_tools contacts <폴더>...           [2026-10-15] 로봇이 움직여서 닿은 것만 (사용자 기준) + 성형 탓 가르기: 실행된 u (u_exec.npz)
                                                         로 10 ms 마다 u·y 를 다시 만들어 실제 형상 거리. 닿음 묶음마다 시작 때 로봇 몫 vs 물체 몫,
                                                         성형기 길이 안에 u 도 닿았나 (아니면 '성형 탓' = 휜 경로·늦음). 끝에 JSON
  python3 -m tools.rt_tools contacts_sum <폴더>...       [2026-10-15] 위를 설정 (지도·방식·성형) 별로 묶은 표 (시나리오 묶음 비교)
  python3 -m tools.rt_tools replay <폴더> <다리 이름> <끝 s> [full|pos|restart|none] [자세히 볼 시각 s ...]
                                                         한 다리의 MPC 를 기록대로 다시 돌림 (GPU): 기록된 영상으로 지도를 다시 만들고, 다리 시작에서 MPC 를
                                                         새로 시작해 주기마다 기록된 목표를 주고, 안전 검사가 개입한 주기에서 지정한 방법으로 맞춤 (none = 안 맞춤,
                                                         안 주면 기록의 방법 — [R-1] 뒤 기록은 restart, 10-14 전 기록은 full).
                                                         기록 당근과 비교 + 자세히 볼 시각에는 MPC 비용 항목·계획 자세. 시각은 첫 영상 기준 (보고서와 같음)
                                                         가상 카메라 기록만 (상자로 깊이를 다시 렌더). GUI 기록은 서버의 한계 배율을 다시 만들지 않음 (주의만)
실제 형상 거리 = 로봇 메시 꼭짓점 ↔ 상자 (tools.rt_geom.RobotMesh). [2026-10-16] 로봇 메시는 실행마다 그 기록의 모델 (rt_model.of_run:
'tool' = 끝단 공구 붙음, 기록에 없으면 'bare'). 기록 없는 scenario 는 환경 변수 CR3A_MODEL 또는 기본 ('tool').
[2026-10-27 R-1] 기록 읽기 = rt_records (load·obstacles·moving_on·executed_uy 는 그 이름 그대로), 메시 거리 = tools.rt_geom
[2026-10-27 R-1 안전 ⑮] GUI 서버 기록: 바닥 = 기록의 충돌 바닥 max(table_z, 0) (예전 z = 0), 성형기를 바꿨으면 성형기마다 실행된 u
(예전 마지막 것만), 비상정지 감속 구간 (닿음 판정은 보낸 감속 점으로, 성형 확인에서는 뺌), 실제 카메라 기록은 기둥·시나리오 물체가 참값이
아님 (물체 거리·닿음 없음 — 바닥만). 닿음 셈 = 로봇 탓 + 물체가 와서 (10-16 사용자 기준 — 예전 '(셈 안 함)' 문구는 틀림), 카메라가 못 본
물체는 셈 안 함 (물체가 와서 닿은 것도)
"""
import json
import re
import sys
from pathlib import Path

import numpy as np

import rt_model
from rt_records import (estop_windows, executed_uy, floor_z, has_truth, is_gui, kernels, moving_on, outside_windows, read_wmx_csv,
                        u_exec_runs)
from rt_records import load_run as load
from rt_records import scene as obstacles


def robot_mesh(m):
    """모델별 실제 형상 메시 (tools.rt_geom — torch·depth_sim 은 쓸 때만 불러옴: compare·shaping·messages·peaks·arrival 은 numpy 만)."""
    from tools.rt_geom import robot_mesh as rm
    return rm(m)


def true_dists(md, q, t, t_go, scene, t_end=None, fz=0.0):
    """관절 q 의 로봇 메시 ↔ 바닥 (z = fz, rt_records.floor_z) ·물체들 거리 (m). 움직이는 물체는 GO 전·치운 뒤면 None."""
    from rt_scenarios import obstacle_pose
    p = md.points(np.asarray(q, float))
    out = {'바닥': md.floor_dist(p, fz)}
    for name, dims, wps, center in scene:
        if wps is None:
            out[name] = md.box_dist(p, center, dims)
        else:
            out[name] = md.box_dist(p, obstacle_pose(wps, t - t_go), dims) if moving_on(t, t_go, t_end) else None
    return out


# ---------------- compare ----------------
CYCLE_KEYS = ['t', 'stamp', 'q', 'carrot', 'cmd', 'cmd_v', 'cmd_a', 'seq_a_max', 'raw_acc_ratio', 'lim_dev', 'goal', 'mode', 'leg', 'boxes',
              'feasible', 'clr_now_mm', 'clr_seg_mm', 'guard', 'msg_t0', 'n_pre', 'skipped', 'clamp', 'late_s', 'resync_mpc', 'sent']
FRAME_KEYS = ['t', 'q', 'fill', 'near_mm', 'near_new_mm']


def _event_key(e):
    """사건 비교: 계획에 실제로 걸린 시간 (실행마다 다름) 은 뺌. '메시지 늦음' 은 2026-10-13 전 기록이 안전 검사 '앞 메시지 계속' 에도 남겨서 뺌."""
    if e['text'].startswith('메시지 늦음'):
        return None
    return round(e['t'], 6), re.sub(r'\d+\.\d+초', '#초', e['text'])


def _diff(x, y):
    if isinstance(x, (list, tuple)) and isinstance(y, (list, tuple)):
        try:
            return float(np.max(np.abs(np.asarray(x, float) - np.asarray(y, float))))
        except (ValueError, TypeError):
            return 'shape'
    return None


def cmd_compare(a, b):
    ra, rb = load(a), load(b)
    ok = True
    for sec, keys in (('cycles', CYCLE_KEYS), ('frames', FRAME_KEYS)):
        A, B = ra[sec], rb[sec]
        n = min(len(A), len(B))
        first = None
        for i in range(n):
            bad = [k for k in keys if k in A[i] and k in B[i] and A[i][k] != B[i][k]]   # 한쪽에만 있는 값 (판이 다른 기록) 은 뺌
            if bad:
                first = (i, bad)
                break
        if first is None and len(A) == len(B):
            print(f'{sec}: {len(A)} 개 모두 같음')
            continue
        ok = False
        print(f'{sec}: A {len(A)} 개, B {len(B)} 개', '' if first is None else f'— 처음 다른 곳 #{first[0]} (t {A[first[0]]["t"]})')
        if first:
            for k in first[1][:6]:
                x, y = A[first[0]].get(k), B[first[0]].get(k)
                d = _diff(x, y)
                print(f'   {k}: 최대 차 {d}' if d is not None else f'   {k}: {x} vs {y}')
    ea = [k for k in map(_event_key, ra['events']) if k]
    eb = [k for k in map(_event_key, rb['events']) if k]
    if ea == eb:
        print(f'events: {len(ea)} 개 모두 같음')
    else:
        ok = False
        i = next((i for i, (x, y) in enumerate(zip(ea, eb)) if x != y), min(len(ea), len(eb)))
        print(f'events: A {len(ea)} 개, B {len(eb)} 개 — 처음 다른 곳 #{i}:', ea[i] if i < len(ea) else '-', '|', eb[i] if i < len(eb) else '-')
    la = [(L['label'], L['ok'], L.get('why'), L['t0'], L.get('t1'), len(L['plans'])) for L in ra['legs']]
    lb = [(L['label'], L['ok'], L.get('why'), L['t0'], L.get('t1'), len(L['plans'])) for L in rb['legs']]
    print('legs:', '모두 같음' if la == lb else f'다름\n   A {la}\n   B {lb}')
    ok &= la == lb
    va = [(v['t'], v.get('v', v.get('p'))) for v in ra['newvox']]     # 'v' = 칸 번호 글자 (2026-10-14 뒤), 'p' = 칸 중심 mm (그 전)
    vb = [(v['t'], v.get('v', v.get('p'))) for v in rb['newvox']]
    print('newvox:', '모두 같음' if va == vb else '다름')
    ok &= va == vb
    print('결과:', '같음' if ok else '다름')
    return ok


# ---------------- ideal ----------------
def cmd_ideal(dirs):
    for d in dirs:
        r = load(d)
        md = robot_mesh(rt_model.of_run(r))
        st, cy, fr = r['settings'], r['cycles'], r['frames']
        mv = [c for c in cy if c['mode'] == 'move']
        travel = np.degrees(np.abs(np.diff(np.array([c['cmd'] for c in cy]), axis=0)).sum(0))
        # [R-1 ⑮ Thor 시험] GUI 서버 기록은 raw_acc_ratio 가 None (그 값을 안 셈) — 예전엔 여기서 멈췄음 (TypeError) → 있는 것만, 없으면 '기록 없음'
        ra = np.array([c['raw_acc_ratio'] for c in mv if c.get('raw_acc_ratio') is not None])
        ra_s = f'{int((ra > 1).sum())} (최대 ×{ra.max():.2f})' if len(ra) else '기록 없음'
        legs = [(L['label'], L['ok'], round(L.get('duration') or 0, 1), len(L['plans'])) for L in r['legs']]
        print(Path(d).name, st.get('robot'), st.get('scenario'), f'다리 {sum(L[1] for L in legs)}/{len(legs)}', legs)
        print(f'   관절 총 이동 {int(travel.sum())}° {np.round(travel).astype(int).tolist()} | 이동 주기 {len(mv)}: MPC 가속 한계 넘음 '
              f'{ra_s}, 불가능 해 {sum(1 for c in mv if not c["feasible"])} | 안전 검사 '
              f'{sum(1 for c in cy if c["guard"])}, 안 보냄 {sum(1 for c in cy if c["skipped"])} | 추종 오차 최대 '
              f'{max((c["lim_dev"] or 0) for c in cy):.3f} rad | MPC 중앙값 {np.median([c["mpc_ms"] for c in cy if c["mpc_ms"]] or [0]):.1f} ms')
        scene, fz = (obstacles(st) if has_truth(r) else []), floor_z(r)       # [R-1 ⑮] 실제 카메라 = 물체 참값 없음, 바닥 = 충돌 바닥
        rows = [(f['t'], true_dists(md, f['q'], f['t'], r.get('t_go'), scene, r.get('t_go_end'), fz))
                for f in fr if r.get('t_go') is None or f['t'] >= r['t_go']]
        names = list(rows[0][1]) if rows else []
        mins = {}
        for n in names:
            v = [(x[n], t) for t, x in rows if x[n] is not None]
            if v:
                m, t = min(v)
                mins[n] = f'{m * 1000:.1f} mm @ {t - fr[0]["t"]:.1f}s' + (f' (닿음 {sum(1 for x, _ in v if x < 0)} 장)' if m < 0 else '')
        print('   실제 형상 최소 거리 (GO 뒤, 영상 시각 관절값):', mins)
        if not has_truth(r) or fz:
            print(f'   ({"실제 카메라 기록 — 기둥·시나리오 물체는 참값이 아니라 뺌, " if not has_truth(r) else ""}바닥 = z {fz * 1000:+.1f} mm)')


# ---------------- scenario ----------------
def cmd_scenario(name='moving5'):
    from rt_config import MapCfg, TaskCfg
    from rt_scenarios import get_scenario, obstacle_pose
    M = rt_model.get()
    md, t, m = robot_mesh(M), TaskCfg(), MapCfg()
    qa, qb = (np.array(g) for g in t.goals(M.name))
    print(f'로봇 모델 {M.name} (환경 변수 CR3A_MODEL 로 바꿈)')
    pa, pb = md.points(qa), md.points(qb)
    pc, pd = np.array(t.pillar[1]), np.array(t.pillar[2])
    lo, hi = np.array(m.center) - np.array(m.size) / 2, np.array(m.center) + np.array(m.size) / 2
    ts = np.arange(0, 120, 0.1)
    path = [md.points(qa + (qb - qa) * s) for s in np.linspace(0, 1, 10)]   # A↔B 관절 직선 자세 10 개 (전역 경로 근사)
    for nm, dims, w in get_scenario(name):
        P = np.array([obstacle_pose(w, x) for x in ts])
        h = np.array(dims) / 2
        overlap = int((np.max(np.abs(P - pc) - (pd / 2 + h), axis=1) < 0).sum())
        da = np.array([md.box_dist(pa, list(p), dims) for p in P[::5]])
        db = np.array([md.box_dist(pb, list(p), dims) for p in P[::5]])
        dm = np.array([min(md.box_dist(pp, list(p), dims) for pp in path) for p in P[::5]])
        inside = np.all((P >= lo) & (P <= hi), axis=1)
        print(f'{nm}: 기둥 겹침 {overlap} | 바닥 최소 {(P[:, 2] - h[2]).min():.3f} m | A 자세와 최소 {da.min() * 1000:.0f} mm '
              f'| B 자세와 최소 {db.min() * 1000:.0f} mm | 이동 자세들과 7 cm 안 {100 * (dm < 0.07).mean():.0f}% (최소 {dm.min() * 1000:.0f} mm) '
              f'| 지도 안 {100 * inside.mean():.0f}%')


# ---------------- messages ----------------
def cmd_messages(d):
    cy = load(d)['cycles']
    if 'sent' not in cy[0]:
        sys.exit('보낸 메시지 기록이 없음 (rt_run.py --record-sent 로 다시 돌려야 함)')
    worst, worst_sw, where = 0.0, 0.0, None
    for i, c in enumerate(cy):
        pts = np.array(c['sent'])
        t, q = c['msg_t0'] + pts[:, 0], pts[:, 1:]
        h = np.diff(t)
        for k in range(1, len(t) - 1):                   # 같은 간격 점들의 유한차분 가속
            if abs(h[k - 1] - h[k]) < 1e-6:
                a = np.abs((q[k + 1] - 2 * q[k] + q[k - 1]) / h[k] ** 2).max()
                if a > worst:
                    worst, where = a, (i, round(t[k] - c['stamp'], 4), c['guard'])
        if i + 1 < len(cy) and not cy[i + 1]['skipped']:  # 다음 메시지가 앞에 붙인 점 = 이 메시지 점이어야 함
            p2 = np.array(cy[i + 1]['sent'])
            for k2, tt in enumerate(cy[i + 1]['msg_t0'] + p2[:, 0]):
                m = np.nonzero(np.abs(t - tt) < 1e-6)[0]
                if len(m) and tt < cy[i + 1]['stamp'] + 0.03:
                    worst_sw = max(worst_sw, float(np.abs(q[m[0]] - p2[k2, 1:]).max()))
    print(f'메시지 안 유한차분 가속 최대 {worst:.2f} rad/s² (주기, 스탬프 뒤 s, 안전 검사) {where}; 갈아타기 겹치는 점 차이 최대 {worst_sw * 1000:.4f} mrad')


# ---------------- peaks (WMX) ----------------
def cmd_peaks(d, n=4):
    w, r = read_wmx_csv(Path(d) / 'wmx_axis.csv'), load(d)
    u, cv, t0 = w['unix'], w['cmdvel'], r['frames'][0]['t']
    a1 = np.abs(np.diff(cv, axis=0)) / 0.001
    seen = []
    for k in np.argsort(a1.max(axis=1))[::-1]:
        if any(abs(u[k] - s) < 0.05 for s in seen):
            continue
        seen.append(u[k])
        j, T = int(np.argmax(a1[k])), u[k + 1]
        c = min(r['cycles'], key=lambda c: T - c['stamp'] if c['stamp'] <= T else 9)
        print(f'가속 {a1[k, j]:.2f} rad/s² 축 {j} @ {T - t0:.2f}s | 직전 메시지 스탬프 {1000 * (c["stamp"] - T):.0f} ms, '
              f'안전 검사 {c["guard"]}, 안 보냄 {c["skipped"]}, 늦음 {c.get("late_s")}, lead {c["lead_ms"]} ms')
        print('   1 ms 가속 앞뒤:', np.round(np.diff(cv[max(k - 8, 0):k + 6, j]) / 0.001, 2).tolist())
        if len(seen) >= n:
            break


# ---------------- arrival (오버슈트) ----------------
def cmd_arrival(d):
    """다리마다 도착 때 목표를 지나친 양 (관절마다, 가는 방향으로 목표를 넘은 최대값, °) 과 다시 돌아와 0.1° 안에 든 시각.
    로봇 = WMX 실제 위치 (있으면) 또는 영상 시각 관절값, 명령 = 주기 명령 (cmd), 당근 = MPC 상태 (carrot, 2026-10-13 뒤 기록)."""
    r = load(d)
    cy = r['cycles']
    sent = [c for c in cy if not c['skipped']]                 # 안 보낸 주기의 명령은 실행되지 않음
    series = {'명령': (np.array([c['stamp'] for c in sent]) + 1.0 / r['settings']['rate_hz'], np.array([c['cmd'] for c in sent]))}
    if 'carrot' in cy[0]:
        series['당근'] = (np.array([c['t'] for c in cy]), np.array([c['carrot'] for c in cy]))
    if (Path(d) / 'wmx_axis.csv').exists():
        w = read_wmx_csv(Path(d) / 'wmx_axis.csv')
        series['로봇'] = (w['unix'][::5], w['fb'][::5])
    else:
        series['로봇'] = (np.array([f['t'] for f in r['frames']]), np.array([f['q'] for f in r['frames']]))
    qa, qb = np.array(r['settings']['qA']), np.array(r['settings']['qB'])
    worst = {k: 0.0 for k in series}
    for L in r['legs']:
        if not L['ok']:
            continue
        goal = qb if 'A→B' in L['label'] else qa
        row = []
        for k, (t, q) in series.items():
            m = (t >= max(L['t0'], L['t1'] - 1.0)) & (t <= L['t1'] + 0.5)     # 마지막 접근 1 s + 도착 뒤 0.5 s
            if m.sum() < 2:
                continue
            qq = q[m]
            gap = goal - qq[0]
            dirn = np.where(np.abs(gap) > np.radians(0.2), np.sign(gap), 0.0)  # 이미 목표에 있던 관절은 뺌
            over = np.degrees(np.max((qq - goal) * dirn, axis=0).clip(min=0))
            j = int(np.argmax(over))
            worst[k] = max(worst[k], float(over[j]))
            row.append(f'{k} {over[j]:.2f}° (J{j + 1})')
        print(f"{L['label']:>9} {L['duration']:.2f}s 지나침 최대: " + ', '.join(row))
    print('전체 최대 지나침 (°):', {k: round(v, 2) for k, v in worst.items()})


# ---------------- contact ----------------
def cmd_contact(d, name):
    from rt_scenarios import obstacle_pose
    r = load(d)
    st, t_go, T0 = r['settings'], r.get('t_go'), r['frames'][0]['t']
    if not has_truth(r):
        sys.exit('실제 카메라 기록 — 기록의 기둥·시나리오 물체는 가상이라 (참값 없음) 물체 거리를 셀 수 없음')
    hit = [o for o in obstacles(st) if o[0] == name]
    if not hit:
        sys.exit(f'물체 이름: {[o[0] for o in obstacles(st)]}')
    _, dims, wps, center = hit[0]
    if (Path(d) / 'wmx_axis.csv').exists():
        w = read_wmx_csv(Path(d) / 'wmx_axis.csv')
        T, Q = w['unix'][::10], w['fb'][::10]
    else:
        T, Q = np.array([f['t'] for f in r['frames']]), np.array([f['q'] for f in r['frames']])
    md = robot_mesh(rt_model.of_run(r))
    pos = (lambda t: center) if wps is None else (lambda t: obstacle_pose(wps, t - t_go))
    sel = [i for i in range(len(T)) if wps is None or moving_on(T[i], t_go, r.get('t_go_end'))]
    dd = np.array([md.box_dist(md.points(Q[i]), pos(T[i]), dims) for i in sel])
    k = int(np.argmin(dd))
    Tm = T[sel[k]]
    print(f'{name} 최소 {dd[k] * 1000:.1f} mm @ {Tm - T0:.2f}s, 물체 {np.round(pos(Tm), 3).tolist()}')
    m = (T[sel] > Tm - 1.6) & (T[sel] < Tm + 0.6)
    print('  실제 거리 (mm):', np.round(dd[m] * 1000, 0).astype(int).tolist())
    for f in r['frames']:
        if Tm - 1.4 < f['t'] < Tm + 0.5:
            print(f"  영상 {f['t'] - Tm:+.2f}s 지도 새 물체 {f['near_new_mm']} mm, MPC 지도 {f['near_mm']} mm, 경계 채움 {f['fill']}")
    for c in r['cycles']:
        if Tm - 1.4 < c['t'] < Tm + 0.5:
            print(f"  주기 {c['t'] - Tm:+.2f}s {c['leg']} {c['mode']} 안전 검사 {c['guard']} 실제 거리 지금/궤적 {c['clr_now_mm']}/{c['clr_seg_mm']} "
                  f"|v| {max(abs(x) for x in c['cmd_v']):.2f} 안 보냄 {int(c['skipped'])}")
    for e in r['events']:
        if Tm - 3 < e['t'] < Tm + 2:
            print(f"  사건 {e['t'] - Tm:+.2f}s {e['text']}")


# ---------------- replay (MPC 다시 돌리기) ----------------
STATIC_HARD = ('정지 물체에 닿음', '정지 물체에서 떨어짐')       # (10-16 전 기록) rt_run.Controller._static_hard 사건 — [R-1] 엔진에서 뺌


def _guard_state(cy, i, c, mpc):
    """안전 검사가 개입한 주기 i 뒤 로봇 명령 상태 (q, v, a) = MPC 를 맞춘 상태. 반환 (상태 또는 None, 설명).
    2026-10-14 뒤 기록은 그대로 ('resync_to'). 그 전 기록: 보냈으면 그 주기 명령, 안 보냈으면 (앞 메시지 계속) 앞 메시지 (주기 i−1) 의
    다음 스탬프 상태를 다시 계산 — 주기 i−1 이 브레이크를 보냈으면 정지 궤적, 아니면 당근 쫓기 (주기 i−2 명령 상태에서)."""
    x = cy[i]
    if 'resync_to' in x:
        return tuple(np.array(s, float) for s in x['resync_to']), '기록된 상태'
    if not x['skipped']:
        return tuple(np.array(x[k], float) for k in ('cmd', 'cmd_v', 'cmd_a')), '기록된 명령'
    if i < 2:
        return None, '앞 주기가 창 밖이라 상태를 모름 → 안 맞춤'
    from rt_motion import brake, track_target
    p1, p0 = cy[i - 1], cy[i - 2]
    s0 = tuple(np.array(p0[k], float) for k in ('cmd', 'cmd_v', 'cmd_a'))
    m, dt = c.motion, 1.0 / c.mpc.rate_hz
    n, k = int(round(m.cycles * dt / m.step_s)), int(round(dt / m.step_s)) - 1
    jmax = np.full(6, float(m.jerk))
    if p1['guard'] == 'brake' and not p1['skipped']:
        Q, V, A, _ = brake(*s0, np.arange(1, n + 1) * m.step_s, mpc.amax, jmax)
    else:
        Q, V, A = track_target(*s0, np.array(p1['carrot'], float), n, m.step_s, mpc.vmax, mpc.amax, jmax, dt, sub=m.sub, qlim=mpc.qlim)
    chk = float(np.degrees(np.abs(Q[k] - np.array(p1['cmd'], float)).max()))
    rough = p1['skipped'] or p0['skipped'] or p1.get('held') or p1.get('clamp')    # 기록에 없는 상태·목표가 끼어 근사
    return (Q[-1], V[-1], A[-1]), f'앞 메시지를 다시 계산 (확인 {chk:.3f}°{", 근사" if rough else ""})'


def cmd_replay(d, leg, t_end, how=None, *detail):
    """[2026-10-14] rt_20261014_015202 13 번 다리 MPC 발산 원인을 찾은 방법 (작업기록 13-10): 기록대로 (full) 돌리면 발산이 재현되고,
    안전 검사 뒤 맞출 때 가속을 빼면 정상. MPC 만 다시 돌림 — 로봇 명령·영상 시각 관절값·안전 검사 개입·정지 물체 거리 바꿈·첫 메시지
    늦음은 기록 그대로 따름. how 를 안 주면 기록의 방법 (mpc.resync, 그 전 기록은 full).
    [2026-10-27 R-1] 엔진에서 되돌리기 설정 (mpc.resync·motion.final_clamp 등) 과 정지 물체 거리 바꿈 (map.uniform False 의 _static_hard) 을
    뺌 → 기록의 맞춤 방법: resync 설정이 있으면 그것 (10-14 ~ R-1 전), 없으면 R-1 뒤 기록 (늘 restart — 설정 map.floor_band_m 이 있음) 또는
    10-14 전 기록 (full). Mpc.resync 는 full (q·v·a)·pos·restart 를 그대로 받음 (rt_run 은 restart 만 씀). 지금 엔진에 없는 기록 설정은 알리고 뺌,
    정지 물체 거리 바꿈 사건이 있는 기록 (10-16 전) 은 그 구간이 기록과 다르게 돎 (알림). 실제 카메라 기록은 안 함 (깊이를 상자로 다시 렌더하므로)."""
    from rt_config import Config
    from rt_mpc import Mpc
    from rt_scenarios import obstacle_pose
    from rt_world import World, robot_cfg
    D = np.degrees
    r = load(d)
    st, T0, t_go = r['settings'], r['frames'][0]['t'], r.get('t_go')
    if not has_truth(r):
        sys.exit('실제 카메라 기록 — 다시 돌리기는 기록의 상자로 깊이를 다시 렌더하므로 실제 카메라 지도와 다름 (안 함)')
    if is_gui(r):
        print('주의: GUI 서버 기록 — 서버의 cuRobo 한계 배율 (rt_server LIMIT_FRAC·JERK_MULT) 과 속도 배율은 다시 만들지 않음 → MPC 가 기록과 다를 수 있음')
    cfg = st['config']
    rec_how = cfg['mpc'].get('resync') or ('restart' if 'floor_band_m' in (cfg.get('map') or {}) else 'full')
    how = how or rec_how
    if how not in ('full', 'pos', 'restart', 'none'):
        sys.exit(f'맞춤 방법은 full | pos | restart | none (받은 값 {how!r}). 자세히 볼 시각은 방법 다음에')
    if how != rec_how:
        print(f'기록은 {rec_how} 로 맞췄음 → {how} 로 바꿔 돌림')
    c = Config(model=rt_model.of_run(r).name)
    gone = []
    for sec in ('dist', 'mpc', 'motion'):
        for k, v in cfg[sec].items():
            if not hasattr(getattr(c, sec), k):
                gone.append(f'{sec}.{k}={v}')
                continue
            cur = getattr(getattr(c, sec), k)
            same = cur == v if isinstance(cur, str) else np.allclose(np.asarray(cur, float), np.asarray(v, float))
            if not same:
                print(f'주의: 설정이 기록과 다름 {sec}.{k} 기록 {v}, 지금 {cur}')
    if gone:
        print(f'(기록에만 있는 설정 — [R-1] 되돌리기 설정을 뺌, 지금 동작으로 돌림: {" ".join(gone)})')
    if any(e['text'].startswith(STATIC_HARD) for e in r['events']):
        print('주의: 정지 물체 거리 바꿈 사건이 있는 기록 (10-16 전 정지·움직임 나눈 지도) — 지금 엔진에는 그 지도가 없어 ([R-1] map.uniform False 뺌) '
              '그 구간은 기록과 다르게 돎')
    L = next((x for x in r['legs'] if x['label'] == leg), None)
    if L is None:
        sys.exit(f'다리 이름: {[x["label"] for x in r["legs"]]}')
    t_stop = min([T0 + float(t_end)] + [e['t'] for e in r['events'] if e['t'] > L['t0'] and e['text'].startswith(f'{leg} 막힘 → 다시 계획')])
    cy = [x for x in r['cycles'] if x['leg'] == leg and x['mode'] == 'move' and L['t0'] <= x['t'] <= t_stop]
    if not cy:
        sys.exit(f'{leg}: {float(t_end):.2f} s 전에 따라가기 주기가 없음 (시각은 첫 영상 기준 s)')
    prev = [x for x in r['cycles'] if x['t'] < cy[0]['t']][-1]
    pillar = st['pillar']

    def boxes(t):
        out = [(list(pillar[1]), list(pillar[2]), [1, 0, 0, 0])]
        if moving_on(t, t_go, r.get('t_go_end')):
            out += [(obstacle_pose(o['wps'], t - t_go), o['dims'], [1, 0, 0, 0]) for o in st['obstacles']]
        return out

    robot = robot_cfg(c.model)
    W = World(robot, c, log=lambda *a: None)
    mpc = Mpc(robot, c.mpc, c.dist.act_new, W.cm.esdf_mpc(), [W.floor])
    W.cm.start_background()                         # 지도: 기록 첫 영상부터 같은 관절값·물체 위치로 다시 렌더 (배경도 같은 장 수)
    frames, fi = r['frames'], 0
    mpc.start(np.array(prev['cmd'], float))         # 다리 시작: 서 있는 자리에서 새로 시작 (rt_run._plan_from_rest 와 같음)
    mpc.path, mpc.mode = None, 'hold'               # 경로 대신 기록된 목표를 주기마다 줌
    detail = [float(x) for x in detail]
    first_late = [e['t'] for e in r['events'] if '첫 메시지' in e['text']]
    print(f'{leg}: {cy[0]["t"] - T0:.2f} ~ {cy[-1]["t"] - T0:.2f} s, 안전 검사 뒤 맞춤 = {how}')
    for i, x in enumerate(cy):
        while fi < len(frames) and frames[fi]['t'] <= x['t']:
            W.frame(np.array(frames[fi]['q'], float), boxes(frames[fi]['t']))
            fi += 1
        mpc.set_goal(np.array(x['goal'], float))
        info = mpc.step()
        car, rec = info['target'], np.array(x['carrot'], float)
        t = x['t'] - T0
        print(f"{t:6.2f} 당근 {np.round(D(car)).astype(int).tolist()} 기록 {np.round(D(rec)).astype(int).tolist()} "
              f"차 {D(np.abs(car - rec).max()):5.1f}° 당근-명령 {D(np.abs(car - np.array(x['cmd'], float)).max()):5.1f}° "
              f"해 {'있음' if info['feasible'] else '없음'} (기록 {'있음' if x['feasible'] else '없음'}) 안전 검사 {x['guard']}")
        if any(abs(t - s) < 0.026 for s in detail):
            cc = mpc.mpc.trajectory_execution_manager.get_current_metrics().costs_and_constraints
            terms = {n: round(float(v.float().sum()), 1) for coll in (cc.costs, cc.constraints, cc.hybrid_costs_constraints)
                     if coll is not None for n, v in zip(coll.names, coll.values) if float(v.float().sum()) != 0.0}
            print('      비용 항목 (계획 전체 합):', terms)
            P = mpc.mpc.trajectory_execution_manager.get_robot_state_sequence().joint_state.position.view(-1, 6).cpu().numpy()
            idx = [0, 4, 8, 16, 32, len(P) - 1]
            cl = W.clearances(P[idx], ('raw', 'mpc'))
            for j, k in enumerate(idx):
                print(f'      계획 {k:2d} 번 점 {np.round(D(P[k])).astype(int).tolist()} 실제 거리 {cl["raw"][j] * 1000:6.1f} mm, MPC 지도 {cl["mpc"][j] * 1000:6.1f} mm')
        t_next = cy[i + 1]['t'] if i + 1 < len(cy) else np.inf
        if any(x['t'] <= tl < t_next for tl in first_late):
            mpc.resync(np.array(prev['cmd'], float))        # 첫 메시지 늦음: 서 있는 자리로 (rt_run.cycle 과 같음)
            print('      (첫 메시지 늦음 → MPC 를 서 있는 자리로 — 기록 사건)')
        if x['guard'] is not None and how != 'none':
            s, why = _guard_state(cy, i, c, mpc)
            if s is None:
                print(f'      ({why})')
                continue
            q, v, a = s
            if how == 'full':
                mpc.resync(q, v, a)
            else:
                mpc.resync(q, restart=how == 'restart')
            print(f'      → 맞춤 ({how}, {why}): 명령 {np.round(D(q)).astype(int).tolist()} |v| {np.abs(v).max():.2f} |a| {np.abs(a).max():.2f}')


# ---------------- shaping (입력성형 확인) ----------------
def cmd_shaping(d):
    """실행된 u 전체 (u_exec.npz, 1 ms, 세그먼트별) 를 rt_shaping.shape_fir 로 한 번에 컨볼루션 → 보낸 메시지 점 (sent_traj = 실행된 y 점,
    5 ms + 꼬리 10 ms) 시각에서 비교. 실시간 계산 (rt_stream: 메시지마다 기억 + 후보) 과 다른 계산 경로.
    [2026-10-27 R-1 ⑮] GUI 서버 기록: 성형기마다 그 파일·그 성형기로 (예전 마지막 u_exec.npz 만 — 앞 성형기 동안은 비교 점이 없었음),
    보낸 점은 그 성형기를 쓴 시각 범위에서만 (서버는 성형기마다 Stream 을 새로 만들어 'seg' 번호가 0 부터 다시), 비상정지 감속 점은 뺌."""
    from rt_shaping import shape_fir
    rec = load(d)
    runs = u_exec_runs(d, rec)
    st = rec.get('sent_traj')
    if st is None:
        sys.exit('sent_traj 없음 (실행 기록이 옛 판)')
    ts, qs = np.array(st['t']), np.array(st['q'])
    sg = np.array(st['seg']) if 'seg' in st else None
    keep = outside_windows(ts, estop_windows(rec))                # 비상정지 감속 점 (성형 없음) 은 뺌
    errs, eff, per = [], [], []
    for kd, segs, h, g in runs:
        inside = keep & (ts >= kd['t0']) & (ts < kd['t1'])        # 이 성형기를 쓰는 동안 보낸 점
        ek = []
        for i, (T0, b, q) in enumerate(segs):
            y = shape_fir(q, h, before=b)
            tt = T0 + np.arange(len(q)) * g
            m = (ts >= tt[0] - 1e-9) & (ts <= tt[-1] + 1e-9) & inside
            if sg is not None:
                m &= sg == i                                      # 이 세그먼트에서 보낸 점만 (이상적 모드 시각 겹침)
            if not m.any():
                continue
            k = np.rint((ts[m] - T0) / g).astype(int)
            ok = np.abs(ts[m] - (T0 + k * g)) < 2e-4             # sent_traj 시각은 0.1 ms 반올림 기록
            ek.append(np.abs(y[k[ok]] - qs[m][ok]).max(axis=1))
            eff.append(np.abs(y[k[ok]] - q[k[ok]]).max(axis=1))
        errs += ek
        per.append((kd, len(segs), np.concatenate(ek) if ek else np.zeros(0)))
    E, F = np.concatenate(errs), np.concatenate(eff)
    sh = rec['settings'].get('shaping') or {}
    print(f"성형 {sh.get('label')} (칸 {sh.get('taps')}, 평균 지연 {sh.get('mean_delay_ms')} ms), 세그먼트 {sum(n for _, n, _ in per)}, 비교 점 {len(E)}")
    if len(per) > 1:
        for kd, n, ek in per:
            print(f"  성형기 {kd['label']} ({kd['file']}, ~{'지금' if kd['until'] is None else round(kd['until'] - rec['frames'][0]['t'], 2)} s): "
                  f"세그먼트 {n}, 비교 점 {len(ek)}, 보낸 y − 전체 컨볼루션 최대 {ek.max() if len(ek) else float('nan'):.2e} rad")
    print(f'  보낸 y − 전체 컨볼루션: 최대 {E.max():.2e} rad (기록 반올림 1e-6 수준이면 같음)')
    print(f'  성형이 바꾼 양 |y − u|: 최대 {np.degrees(F.max()):.3f}°, 중앙 {np.degrees(np.median(F)):.4f}°')
    return E.max()


# ---------------- contacts (로봇 탓 닿음 + 성형 탓 가르기, 2026-10-15) ----------------
CT_DT = 0.01            # 실행된 u·y 를 다시 만들어 보는 간격 (s)
CT_ONSET = 0.05         # 닿기 시작할 때 로봇 몫·물체 몫을 재는 앞 구간 (s)
CT_ROBOT_MM = 0.5       # 로봇 몫이 이보다 깊으면 (mm) '로봇이 움직여 닿음' (아니면 '물체가 와서'). 사용자 기준: 10-14 엔 멈춘 로봇에 물체가 와서
                        #  닿은 것은 셈 안 했지만 10-16 부터 셈 (서 있어도 다가오면 피해야) — 셈 = 로봇 탓 + 물체가 와서, 카메라 못 봄은 빼고
CT_STATIC_MM = 0.01     # [2026-10-15 고침] 정지 물체 (기둥·바닥) 는 다가올 수 없으므로 로봇 몫이 조금이라도 음수면 로봇 탓. 0.5 mm 기준은
                        #  느리게 밀고 들어간 닿음 (WMX EI 실행: 0.01 rad/s 로 기둥 −1.5 mm, 4.7 s) 을 '물체가 와서' 로 잘못 셌다
CT_BATCH = 128          # 한 번에 기구학·거리를 계산할 자세 수 (GPU 메모리)
# [2026-10-16 사용자 기준] '카메라 측정되어 물체로 감지한 경우에 대하여 관통했는지' — 카메라 지도가 닿은 자리의 물체를 못 봤으면 (못 보는 면·
#  팔 지우기) 셈 안 함. 봤다 = 닿기 CT_SEEN_LOOK_S 전 ~ 닿는 동안 지도 기록의 그 물체 칸 (실제 상자 ± CT_SEEN_PAD) 을 닿은 시각까지 물체가
#  움직인 만큼 옮겼을 때 로봇 메시에서 CT_SEEN_MM 안 (지도 1 cm 칸 두 개)
CT_SEEN_MM = 20.0
CT_SEEN_PAD = 0.015
CT_SEEN_LOOK_S = 1.0


def robot_contacts(d, bm=None):
    """한 실행의 닿음 묶음 (실제 형상): 로봇 탓 여부 (시작 CT_ONSET 동안 로봇만 움직였을 때 거리 변화 vs 물체만 움직였을 때) 와
    성형 탓 여부 (성형기 길이 안에서 같은 물체에 u 도 닿았나 — 아니면 성형된 y 만 닿음 = 휜 경로·늦음 탓).
    [2026-10-27 R-1 ⑮] 실행된 명령 = 모든 성형기의 세그먼트 (그 성형기로) + 비상정지 감속 구간 (보낸 감속 점, 성형 없음), 바닥 = 기록의 충돌
    바닥 (floor_z), 실제 카메라 기록은 기둥·시나리오 물체 대신 바닥만 (참값 없음)."""
    from rt_scenarios import obstacle_pose
    rec = load(d)
    st, tgo, tend, Tf = rec['settings'], rec.get('t_go'), rec.get('t_go_end'), rec['frames'][0]['t']
    M = rt_model.of_run(rec)
    if bm is None or getattr(bm, 'model', None) != M.name:     # 실행 기록의 로봇 모델 메시로
        bm = robot_mesh(M)
    fz, truth = floor_z(rec), has_truth(rec)
    segs = executed_uy(d, rec)
    nb = max(1, int(round(CT_ONSET / CT_DT)))
    pil = st['pillar']
    objs = ([('기둥', pil[2], None, pil[1])] if truth else []) + [('바닥', None, None, None)] + \
           ([(o['name'], o['dims'], o['wps'], None) for o in st['obstacles']] if truth else [])

    def where(o, t):
        """물체 o 의 시각 t 위치 (움직이는 물체는 GO 전·치운 뒤면 None = 없음)."""
        name, dims, wps, center = o
        if wps is None:
            return center
        return obstacle_pose(wps, t - tgo) if moving_on(t, tgo, tend) else None

    def dists(P, o, T):
        name, dims, wps, center = o
        if name == '바닥':
            return bm.floor_d(P, fz)
        C = [where(o, t) for t in T]
        out = np.full(len(T), np.inf)
        m = np.array([c is not None for c in C])
        if m.any():
            idx = bm.torch.as_tensor(np.nonzero(m)[0], device=P.device)
            out[m] = bm.box_d(P.index_select(0, idx), [c for c in C if c is not None], dims)
        return out

    # [2026-10-16 사용자 기준] "카메라 측정되어 물체로 감지한 경우에 대하여 관통했는지" — 카메라 지도가 닿은 자리의 물체를 봤나
    grid, snaps = st.get('grid'), rec.get('newvox') or []
    snap_t = np.array([s['t'] for s in snaps], float)
    uniform = bool(st.get('map_uniform', ((st.get('config') or {}).get('map') or {}).get('uniform', False)))
    vcache = {}

    def centers(key, code):
        """지도 기록 (pack_voxels 글자) → 막힌 칸 중심 [n, 3] (m)."""
        if key not in vcache:
            from rt_world import unpack_voxels
            idx = unpack_voxels(code) if code else np.zeros(0, np.int64)
            ny, nz = grid['shape'][1], grid['shape'][2]
            iz, r = idx % nz, idx // nz
            iy, ix = r % ny, r // ny
            vcache[key] = np.asarray(grid['origin'], float) + (np.stack([ix, iy, iz], 1) + 0.5) * grid['voxel']
        return vcache[key]

    def seen_mm(o, t0, t1, tm, qm):
        """물체 o 의 닿음 (시작 t0 ~ 끝 t1, 가장 깊은 시각 tm 의 로봇 자세 qm) 을 카메라 지도가 봤나: t0 − CT_SEEN_LOOK_S ~ t1 의 지도
        기록 (새 물체 칸, 영상 3 장마다; 정지 물체는 켤 때 지도) 에서 o 의 칸 (그 시각 실제 상자 ± CT_SEEN_PAD) 을 물체가 tm 까지 움직인
        만큼 옮겨 (참값), tm 의 로봇 메시까지 가장 가까운 칸 겉면 거리 (mm). 칸이 없으면 inf. 바닥 (계획기가 상자로 앎) 은 None."""
        name, dims, wps, center = o
        if name == '바닥' or not grid:
            return None
        if wps is None and not uniform:
            sets = [(tm, centers('static', rec.get('static_vox') or ''))]
        else:                                           # [2026-10-16] 구분 없는 지도 (map.uniform): 정지 물체도 실행 중 지도 기록으로
            ks = np.nonzero((snap_t >= t0 - CT_SEEN_LOOK_S) & (snap_t <= t1))[0]
            if not len(ks):
                k = int(np.searchsorted(snap_t, t0, 'right')) - 1
                ks = [k] if k >= 0 else []
            sets = [(snap_t[k], centers(int(k), snaps[k]['v'])) for k in ks]
        cm = where(o, tm)
        if cm is None:
            return None
        P = bm.points_batch(np.asarray(qm, float)[None])[0]
        best = np.inf
        for ts, C in sets:
            c = where(o, ts)
            if c is None or not len(C):
                continue
            half = np.asarray(dims, float) / 2 + CT_SEEN_PAD
            m = np.all(np.abs(C - np.asarray(c, float)) <= half, axis=1)
            if not m.any():
                continue
            V = bm.torch.as_tensor(C[m] + (np.asarray(cm, float) - np.asarray(c, float)), dtype=bm.torch.float32, device=P.device)
            best = min(best, float(bm.torch.cdist(V, P).min()) - grid['voxel'] / 2)
        return best * 1000.0

    eps = []
    for sgm in segs:
        shaped = len(sgm.h) > 1
        step = max(1, int(round(CT_DT / sgm.g)))
        nk = int(np.ceil((len(sgm.h) - 1) * sgm.g / CT_DT)) + 1
        t, u, y = sgm.t[::step], sgm.u[::step], sgm.y[::step]
        if len(t) < 2:
            continue
        Dy = {o[0]: np.zeros(len(t)) for o in objs}
        Du = {o[0]: np.zeros(len(t)) for o in objs}
        for k in range(0, len(t), CT_BATCH):
            sl = slice(k, k + CT_BATCH)
            Py = bm.points_batch(y[sl])
            Pu = bm.points_batch(u[sl]) if shaped else Py
            for o in objs:
                Dy[o[0]][sl] = dists(Py, o, t[sl])
                Du[o[0]][sl] = dists(Pu, o, t[sl]) if shaped else Dy[o[0]][sl]
        for o in objs:
            dy, du = Dy[o[0]], Du[o[0]]

            def shares(X, dX, i, j, o=o):
                """자세들 X 의 칸 j → i 사이 거리 변화를 (로봇만 움직였을 때, 물체만 움직였을 때) 로 나눔 (m)."""
                if o[2] is None:                # 정지 물체·바닥: 물체 몫 0
                    return float(dX[i] - dX[j]), 0.0
                P2 = bm.points_batch(np.stack([X[i], X[j]]))
                cj, ci = where(o, t[j]), where(o, t[i])
                if cj is None:
                    cj = ci
                return float(bm.box_d(P2[:1], [cj], o[1])[0] - dX[j]), float(bm.box_d(P2[1:], [ci], o[1])[0] - dX[j])

            c = dy < 0
            i = 0
            while i < len(t):
                if not c[i]:
                    i += 1
                    continue
                k = i
                while k + 1 < len(t) and c[k + 1]:
                    k += 1
                e = {'obj': o[0], 't': round(float(t[i] - Tf), 2), 'dur_s': round((k - i + 1) * CT_DT, 2),
                     'min_y_mm': round(float(dy[i:k + 1].min()) * 1000, 1)}
                if i == 0:
                    e.update(kind='세그먼트 시작에 이미 닿음', robot=None)
                else:
                    j = max(0, i - nb)
                    rob, ob = shares(y, dy, i, j)
                    thr = (CT_STATIC_MM if o[2] is None else CT_ROBOT_MM) / 1000
                    rc = rob < ob and rob < -thr
                    e.update(robot=rc, robot_mm=round(rob * 1000, 1), obj_mm=round(ob * 1000, 1),
                             v_rad_s=round(float(np.abs(y[i] - y[j]).max() / max((i - j) * CT_DT, 1e-9)), 2))
                    a = max(0, i - nk)
                    hits = np.nonzero(du[a:k + 1] < 0)[0]
                    e['min_u_mm'] = round(float(du[a:k + 1].min()) * 1000, 1)
                    if not shaped:
                        e['cause'] = '비상정지 감속 (성형 없음)' if sgm.estop else '성형 없음'
                    elif len(hits) == 0:
                        e['cause'] = '성형 탓'                  # 성형기 길이 안에 u 는 안 닿음 (휜 경로·늦음)
                    else:
                        # u 도 닿았으면 그 닿음이 'u 가 움직여서' 인가 — 아니면 (u 는 서 있는데 물체가 옴) 성형 없음에서는 셈 안 하는 닿음이었을 것
                        #  → y 가 늦게 서느라 움직이며 닿은 것 = 성형 탓 (2026-10-15)
                        iu = a + int(hits[0])
                        while iu > 0 and du[iu - 1] < 0:
                            iu -= 1
                        if iu == 0:
                            e['cause'] = 'u 도 닿음 (세그먼트 시작부터)'
                        else:
                            ru, ou = shares(u, du, iu, max(0, iu - nb))
                            e['u_robot_mm'] = round(ru * 1000, 1)
                            e['cause'] = 'u 도 닿음' if (ru < ou and ru < -thr) else '성형 탓 (u 였다면 물체가 와서)'
                    e['kind'] = '로봇 탓' if rc else '물체가 와서 닿음'
                im = i + int(np.argmin(dy[i:k + 1]))
                sm = seen_mm(o, t[i], t[k], t[im], y[im])
                e['seen'] = None if sm is None else bool(sm <= CT_SEEN_MM)
                e['seen_mm'] = round(sm, 1) if (sm is not None and np.isfinite(sm)) else None
                if e.get('robot') is not None and e['seen'] is False:    # 카메라가 못 본 물체 = 셈 안 함 (물체가 와서 닿은 것도 — 사용자 기준)
                    e['kind'] = '카메라 못 봄 (셈 안 함)'
                eps.append(e)
                i = k + 1
    cfg = st.get('config') or {}
    variant = []                                    # 같은 지도·방식·성형 안에서 설정이 다른 실행을 따로 묶음
    gd = cfg.get('guard') or {}
    if gd.get('cp_thr', 0) > 0:
        variant.append(f"여유보존{gd['cp_thr'] * 1000:.0f}mm")
        if abs(gd.get('cp_eps', 0.0005) - 0.0005) > 1e-9:
            variant.append(f"+{-gd['cp_eps'] * 1000:.0f}mm")
        if gd.get('cp_margin_approach'):
            variant.append('여유는다가갈때만')
        if gd.get('cp_vobj', 0) > 0:
            variant.append(f"물체{gd['cp_vobj']:.2f}m/s")
    if st.get('style') == 'cmpc' and (cfg.get('cmpc') or {}).get('knot_s', 0) > 0:
        variant.append(f"매듭{cfg['cmpc']['knot_s'] * 1000:.0f}ms")
    # [2026-10-16] 반응·지우기 거리, 정지·움직임 구분 없는 지도, 멈추지 않는 다시 계획 (그 지도) 이 다른 실행도 따로
    variant.append(f"반응{st.get('act_new', 0.07) * 100:.0f}/{st.get('act_static', 0.02) * 100:.0f}지우기{st.get('mask', 0.05) * 100:.0f}")
    if st.get('map_uniform'):
        variant.append('구분없음')
    pl = cfg.get('plan') or {}
    if st.get('live_replan'):
        variant.append(f"멈추지않는계획:{pl.get('live_map', 'mpc')}")
    # [2026-10-16 작업기록 5-21] 물러나기, 경계 채우기 기울기, 성형 규칙 (2)(3) 다가갈 때만
    if st.get('back_off'):
        variant.append('물러나기')
    if st.get('fill_grad'):
        variant.append('채우기기울기')
    if st.get('cp_dyn_approach') and gd.get('cp_thr', 0) > 0:
        variant.append('성형23다가갈때만')
    # [2026-10-16 작업기록 5-22] 경로 고집 (막힘 이어짐·안 막혔으면 유지), 물러날 때 다시 계획 안 함, 계획 시작값
    if pl.get('ahead_persist_s', 0) > 0:
        variant.append(f"막힘{pl['ahead_persist_s']:g}s")
    if pl.get('keep_if_clear'):
        variant.append('안막혔으면유지')
    if st.get('back_off') and not pl.get('back_replan', True):
        variant.append('물러날때계획안함')
    if pl.get('plan_seeds'):
        variant.append('계획시작값')
    return {'run': Path(d).name, 'scenario': st.get('scenario'), 'shaping': (st.get('shaping') or {}).get('label', 'none'),
            'style': st.get('style', 'carrot'), 'map': st.get('map', 'camera'), 'variant': ' '.join(variant),
            'legs_ok': f"{sum(L['ok'] for L in rec['legs'])}/{len(rec['legs'])}",
            'leg_sum_s': round(sum((L.get('duration') or 0) for L in rec['legs'] if L['label'] != '시작→A'), 1),
            'guards': sum(1 for c in rec['cycles'] if c['guard']), 'episodes': eps,
            # [R-1 ⑮] 무엇을 봤나: 카메라 (실제면 물체 참값 없음), 충돌 바닥, 성형기 (파일이 있는 것), 비상정지 감속 구간 수
            'camera': st.get('camera', 'virtual'), 'truth': truth, 'floor_z_mm': round(fz * 1000, 1),
            'kernels': [k['label'] for k in kernels(rec) if k['file'] and (Path(d) / k['file']).exists()],
            'estop_windows': len(estop_windows(rec))}


def counted(eps):
    """닿음 묶음 → (로봇 탓, 그중 성형 탓, 물체가 와서, 카메라 못 봄) — 사용자 기준: 셈 = 로봇 탓 + 물체가 와서 (10-16 부터), 카메라가 못 본
    물체는 어느 쪽이든 셈 안 함, 세그먼트 시작에 이미 닿아 있던 것 (이어짐, robot None) 은 앞 닿음의 이어짐이라 따로 안 셈."""
    rob = [e for e in eps if e.get('robot') and e.get('seen') is not False]
    shp = [e for e in rob if e.get('cause', '').startswith('성형 탓')]
    walk = [e for e in eps if e.get('robot') is False and e.get('seen') is not False]
    blind = [e for e in eps if e.get('robot') is not None and e.get('seen') is False]
    return rob, shp, walk, blind


def cmd_contacts(*dirs):
    bm = None
    out = []
    for d in dirs:
        r = robot_contacts(d, bm)
        out.append(r)
        rob, shp, walk, blind = counted(r['episodes'])
        cnt = rob + walk
        print(f"{r['run']:26s} {r['map']:6s} {r['style']:6s} {r['shaping']:6s} 다리 {r['legs_ok']:6s} 합 {r['leg_sum_s']:6.1f}s 검사 {r['guards']:3d} | "
              f"닿음 (셈) {len(cnt):2d} = 로봇 탓 {len(rob)} (성형 탓 {len(shp)}) + 물체가 와서 {len(walk)} | 최저 "
              f"{min([e['min_y_mm'] for e in cnt], default=0.0):6.1f} mm | 카메라 못 봄 {len(blind)} (셈 안 함)"
              + (f" (그중 로봇 탓·성형 탓 판정 {sum(1 for e in blind if e.get('robot') and e.get('cause', '').startswith('성형 탓'))})" if blind else ''))
        if not r['truth'] or r['floor_z_mm'] or len(r['kernels']) > 1 or r['estop_windows']:
            print(f"    ({'실제 카메라 기록 — 기둥·시나리오 물체는 참값이 아니라 바닥만 봄 · ' if not r['truth'] else ''}바닥 z {r['floor_z_mm']:+.1f} mm · "
                  f"성형기 {' → '.join(map(str, r['kernels'])) or '-'} · 비상정지 감속 구간 {r['estop_windows']})")
        for e in rob + walk + blind:
            sm = '지도에 없음' if e.get('seen_mm') is None else f"{e['seen_mm']:.1f} mm"
            print(f"    {e['obj']:4s} {e['t']:7.2f}s {e['dur_s']:5.2f}s 최저 y {e['min_y_mm']:6.1f} / u {e.get('min_u_mm', 0.0):6.1f} mm "
                  f"| 앞 {CT_ONSET * 1000:.0f} ms 로봇 몫 {e['robot_mm']:+6.1f} 물체 몫 {e['obj_mm']:+6.1f} | {e['v_rad_s']:.2f} rad/s | "
                  f"{e['cause'] if e.get('robot') else '-'}"                  # (성형 탓 가르기는 로봇이 움직여 닿은 것만 뜻이 있음)
                  + ('' if e.get('seen') is not False else f" | 카메라 못 봄 (지도의 그 물체 칸 ↔ 로봇 {sm})") + f" | {e['kind']}")
    print('JSON', json.dumps(out, ensure_ascii=False))
    return out


def cmd_contacts_sum(*dirs):
    """[2026-10-15] contacts 를 여러 실행에 돌려 설정 (지도·방식·성형) 별로 묶은 표 — 시나리오 묶음 비교 (작업기록 5-11)."""
    bm = None
    groups = {}
    for d in dirs:
        if not (Path(d) / 'rt_run.json').exists():
            continue
        r = robot_contacts(d, bm)
        groups.setdefault((r['map'], r['style'], r['shaping'], r['variant']), []).append(r)
    print('지도   방식   성형  설정 | 실행 다리 실패 | 다리 시간 합 평균 s | 검사 평균 | 로봇 탓: 기둥 (번, 최저 mm, 성형 탓) | 움직이는 물체 (번, 시나리오, '
          '최저 mm, 성형 탓) | 물체가 와서 (셈) | 닿음 셈 합 | 카메라 못 봄 (셈 안 함)')
    out = []
    for key, rs in sorted(groups.items()):
        eps = [(r['scenario'], e) for r in rs for e in r['episodes']]
        rob = [(s, e) for s, e in eps if e.get('robot') and e.get('seen') is not False]
        n_walk = len(counted([e for _, e in eps])[2])
        pil = [e for s, e in rob if e['obj'] in ('기둥', '바닥')]
        mov = [(s, e) for s, e in rob if e['obj'] not in ('기둥', '바닥')]
        fail = sum(int(r['legs_ok'].split('/')[1]) - int(r['legs_ok'].split('/')[0]) for r in rs)
        row = {'map': key[0], 'style': key[1], 'shaping': key[2], 'variant': key[3], 'runs': len(rs), 'leg_fail': fail,
               'leg_sum_mean': round(float(np.mean([r['leg_sum_s'] for r in rs])), 1), 'guards_mean': round(float(np.mean([r['guards'] for r in rs])), 1),
               'pillar_n': len(pil), 'pillar_min': min([e['min_y_mm'] for e in pil], default=0.0),
               'pillar_shaping': sum(1 for e in pil if e.get('cause', '').startswith('성형 탓')),
               'moving_n': len(mov), 'moving_scen': len({s for s, _ in mov}), 'moving_min': min([e['min_y_mm'] for _, e in mov], default=0.0),
               'moving_shaping': sum(1 for _, e in mov if e.get('cause', '').startswith('성형 탓')),
               'walk_in': n_walk, 'counted': len(rob) + n_walk,
               'blind': len(counted([e for _, e in eps])[3])}
        out.append(row)
        print(f"{row['map']:6s} {row['style']:6s} {row['shaping']:5s} {row['variant'] or '-':10s} | {row['runs']:3d} {row['leg_fail']:4d} | {row['leg_sum_mean']:7.1f} | "
              f"{row['guards_mean']:6.1f} | {row['pillar_n']:3d} {row['pillar_min']:6.1f} {row['pillar_shaping']:3d} | "
              f"{row['moving_n']:3d} {row['moving_scen']:3d} {row['moving_min']:6.1f} {row['moving_shaping']:3d} | {row['walk_in']:3d} | {row['counted']:3d} | "
              f"{row['blind']:3d}")
    print('JSON', json.dumps(out, ensure_ascii=False))
    return out


# ---------------- table (비교 표) ----------------
def cmd_table(*dirs):
    out = []
    for d in dirs:
        r = load(d)
        md = robot_mesh(rt_model.of_run(r))
        st, cy, fr = r['settings'], r['cycles'], r['frames']
        legs = r['legs']
        scene, fz = (obstacles(st) if has_truth(r) else []), floor_z(r)       # [R-1 ⑮] 실제 카메라 = 물체 참값 없음, 바닥 = 충돌 바닥
        moving = {o['name'] for o in st['obstacles']}
        rows = [true_dists(md, f['q'], f['t'], r.get('t_go'), scene, r.get('t_go_end'), fz)
                for f in fr if r.get('t_go') is not None and f['t'] >= r['t_go']]
        def mn(names):
            v = [x[n] for x in rows for n in names if x.get(n) is not None]
            return (round(min(v) * 1000, 1), sum(1 for x in rows if any(x.get(n) is not None and x[n] < 0 for n in names))) if v else (None, 0)
        mv, pil, flo = mn(moving), mn([st['pillar'][0]]), mn(['바닥'])
        travel = float(np.degrees(np.abs(np.diff(np.array([c['cmd'] for c in cy]), axis=0)).sum()))
        sh = (st.get('shaping') or {}).get('label', 'none')
        row = {'run': Path(d).name, 'scenario': st.get('scenario'), 'shaping': sh, 'style': st.get('style', 'carrot'),
               'legs_ok': f"{sum(L['ok'] for L in legs)}/{len(legs)}",
               'leg_sum_s': round(sum((L.get('duration') or 0) for L in legs if L['label'] != '시작→A'), 1),
               'travel_deg': round(travel), 'guards': sum(1 for c in cy if c['guard']),
               'moving_min_mm': mv[0], 'moving_contact_frames': mv[1], 'pillar_min_mm': pil[0], 'pillar_contact_frames': pil[1],
               'floor_min_mm': flo[0], 'camera': st.get('camera', 'virtual'), 'floor_z_mm': round(fz * 1000, 1)}
        out.append(row)
        print(f"{row['run']:28s} {row['scenario']:10s} {sh:8s} 다리 {row['legs_ok']:6s} 합 {row['leg_sum_s']:6.1f}s 이동 {row['travel_deg']:5d}° "
              f"검사 {row['guards']:3d} 움직이는 물체 {row['moving_min_mm']} mm (닿음 {row['moving_contact_frames']}) 기둥 {row['pillar_min_mm']} mm "
              f"(닿음 {row['pillar_contact_frames']}) 바닥 {row['floor_min_mm']} mm")
    print('JSON', json.dumps(out, ensure_ascii=False))
    return out


if __name__ == '__main__':
    cmds = {'compare': cmd_compare, 'ideal': lambda *d: cmd_ideal(d), 'scenario': cmd_scenario, 'messages': cmd_messages,
            'peaks': cmd_peaks, 'contact': cmd_contact, 'arrival': cmd_arrival, 'replay': cmd_replay, 'shaping': cmd_shaping, 'table': cmd_table,
            'contacts': lambda *d: cmd_contacts(*d), 'contacts_sum': lambda *d: cmd_contacts_sum(*d)}
    if len(sys.argv) > 1 and sys.argv[1] in ('-h', '--help'):
        print(__doc__)
        sys.exit(0)
    if len(sys.argv) < 2 or sys.argv[1] not in cmds:
        sys.exit(__doc__)
    res = cmds[sys.argv[1]](*sys.argv[2:])
    if sys.argv[1] == 'compare':
        sys.exit(0 if res else 1)
