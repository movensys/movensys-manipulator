"""[2026-10-16] 움직임 품질 (사용자 "너무 흔들 거리고 경로 품질이 좋아보이지 않는데 … 너무 불필요하게 움직이는거같은데"):
다리마다 관절이 실제로 움직인 각도 vs 시작·끝 사이 곧은 각도 (더 움직인 몫), 갔다 돌아옴 (관절 방향 바뀜) 수, 끝단 (그리퍼 끝) 경로 길이 vs
곧은 거리, 최고 속도. 갔다 돌아옴 = 관절 각도의 꺾인 점 중 앞뒤로 AMP 넘게 움직인 것 (scipy find_peaks prominence) — 작은 떨림은 뺌.
꺾인 점마다 바로 앞 WIN 초 안의 사건 (물러나기 시작·끝, 새 경로, 멈춰 계획, MPC 발산) 이나 안전 검사 개입 주기로 원인을 나눔.
명령 = WMX 실행이면 wmx_axis.csv 의 CMDPOS (1 ms, 로봇이 실제로 받은 명령 · SYSTEM_TIME = rt_run 시계), 아니면 실행된 성형 y (u_exec*.npz).
  python3 -m tools.motion_quality <실행 폴더> ... [--plot <png 폴더>]   (컨테이너 안 cr3a_rt_shaping 에서: depth_sim·rt_records)
[2026-10-27 R-1] dev/ 에서 tools/ 로 옮김."""
import argparse
import json
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.signal import find_peaks

import rt_model
from rt_records import executed_uy, load_run, read_wmx_csv
from tools.rt_geom import robot_mesh

JOINTS = [f'joint{i}' for i in range(1, 7)]
AMP = np.radians(2.0)       # 갔다 돌아옴: 꺾인 점 앞뒤로 이보다 크게 움직였을 때만
WIN = 0.4                   # 꺾인 점과 사건을 묶는 창 (s)
STEP = 0.005                # 분석 간격 (s)
KINDS = [('물러남', '물러나기 시작'), ('비킴', '비키기 시작'), ('물러나기 끝', '물러나기 끝'), ('비키기 끝', '비키기 끝'),
         ('새 경로로', '새 경로로 바꿈'), ('계획 성공', '멈춰 계획'),
         ('MPC 발산', 'MPC 발산')]


def command(d):
    """(시각 [n], 관절 [n, 6]) 을 STEP 간격으로."""
    p = Path(d) / 'wmx_axis.csv'
    if p.exists():
        w = read_wmx_csv(p)
        t, q = w['unix'], w['cmd']
    else:
        segs = executed_uy(d)            # [R-1 ⑮] 모든 성형기 + 비상정지 감속 구간 (GUI 서버 기록)
        t = np.concatenate([s.t for s in segs])
        q = np.concatenate([s.y for s in segs])
        o = np.argsort(t, kind='stable')
        t, q = t[o], q[o]
    tt = np.arange(t[0], t[-1], STEP)
    return tt, np.stack([np.interp(tt, t, q[:, j]) for j in range(6)], 1)


def turns(t, q):
    """관절마다 꺾인 점 [(시각, 관절)]: 앞뒤로 AMP 넘게 움직인 극대·극소."""
    out = []
    for j in range(6):
        for s in (1.0, -1.0):
            k, _ = find_peaks(s * q[:, j], prominence=AMP)
            out += [(float(t[i]), j) for i in k]
    return sorted(out)


def cause(tp, events, guard_t):
    for e in reversed(events):
        if tp - WIN <= e['t'] <= tp:
            for key, name in KINDS:
                if key in e['text']:
                    return name
    i = np.searchsorted(guard_t, tp - 0.2)
    if i < len(guard_t) and guard_t[i] <= tp + 0.05:
        return '안전 검사 개입'
    return '그 밖 (MPC 가 피함·따라감)'


def analyze(d, plot_dir=None, quiet=False):
    rec = load_run(d)
    M = rt_model.of_run(rec)
    t, q = command(d)
    ev = [e for e in rec['events']]
    cy = rec['cycles']
    guard_t = np.array([c['t'] for c in cy if c['guard']])
    tp_all = turns(t, q)
    fk = robot_mesh(M)
    rows, causes = [], Counter()
    for L in rec['legs']:
        if L['label'].startswith('시작'):
            continue
        t0, t1 = L['t0'], L.get('t1') or (L['t0'] + (L.get('duration') or 25.0))
        m = (t >= t0) & (t <= t1)
        if m.sum() < 3:
            continue
        Q = q[m]
        travel = np.degrees(np.abs(np.diff(Q, axis=0)).sum(0))
        direct = np.degrees(np.abs(Q[-1] - Q[0]))
        P = np.array([fk.tip(x) for x in Q[::4]])
        tip_len = float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum())
        tip_dir = float(np.linalg.norm(P[-1] - P[0]))
        tps = [x for x in tp_all if t0 <= x[0] <= t1]
        cs = Counter(cause(x[0], ev, guard_t) for x in tps)
        causes.update(cs)
        v = np.degrees(np.abs(np.diff(Q, axis=0)).max(1) / STEP)
        rows.append({'leg': L['label'], 'dur': round(t1 - t0, 2), 'plan': L.get('plan_duration'), 'travel': round(float(travel.sum())),
                     'direct': round(float(direct.sum())), 'turns': len(tps), 'tip_len': round(tip_len, 3), 'tip_dir': round(tip_dir, 3),
                     'vmax': round(float(v.max())), 'causes': dict(cs), 'travel_j': np.round(travel).astype(int).tolist(),
                     'direct_j': np.round(direct).astype(int).tolist()})
    name = Path(d).name
    tr, di = sum(r['travel'] for r in rows), sum(r['direct'] for r in rows)
    tl, td = sum(r['tip_len'] for r in rows), sum(r['tip_dir'] for r in rows)
    res = {'run': name, 'travel': tr, 'direct': di, 'turns': sum(r['turns'] for r in rows), 'tip_len': tl, 'tip_dir': td,
           'causes': dict(causes), 'legs': rows, 'leg_s': sum(r['dur'] for r in rows)}
    if quiet:
        return res
    print(f"\n{name}  ({rec['settings'].get('scenario')}, 모델 {M.name})")
    print(f"  다리 {len(rows)} | 관절 움직임 합 {tr}° (곧은 각도 합 {di}°, {tr / max(di, 1):.2f} 배) | 갔다 돌아옴 {sum(r['turns'] for r in rows)} 번 "
          f"| 끝단 경로 {tl:.2f} m (곧은 거리 {td:.2f} m, {tl / max(td, 1e-9):.2f} 배) | 다리 시간 합 {sum(r['dur'] for r in rows):.1f} s")
    print('  갔다 돌아옴 원인: ' + ', '.join(f'{k} {v}' for k, v in causes.most_common()))
    print('  다리        시간 s (계획 s) | 관절 움직임 ° (곧은 °) | 갔다 돌아옴 | 끝단 m (곧은 m) | 최고 °/s | 관절별 움직임 (곧은) | 원인')
    for r in rows:
        pl = f"{r['plan']:.2f}" if r['plan'] else '-'
        print(f"  {r['leg']:9s} {r['dur']:6.2f} ({pl:>5s}) | {r['travel']:5d} ({r['direct']:4d}) | {r['turns']:3d} | {r['tip_len']:5.2f} ({r['tip_dir']:4.2f}) "
              f"| {r['vmax']:4d} | {r['travel_j']} ({r['direct_j']}) | {r['causes']}")
    if any('paths' in L for L in rec['legs']):                # [2026-10-16] 받아 들인 전역 경로 (rt_run._log_path)
        print('  받아 들인 경로: 다리 | 시각 s | 종류 | 길이 s | 관절 움직임 ° (곧은 °) | 곧은 선에서 가장 벗어남 ° (관절별)')
        for L in rec['legs']:
            for p in L.get('paths', []):
                P = np.array(p['q'])
                tr = np.degrees(np.abs(np.diff(P, axis=0)).sum())
                di = np.degrees(np.abs(P[-1] - P[0]).sum())
                s = np.linspace(0, 1, len(P))[:, None]
                dev = np.degrees(np.abs(P - (P[0] + s * (P[-1] - P[0]))).max(0))
                print(f"    {L['label']:9s} {p['t'] - rec['frames'][0]['t']:7.2f} {p['kind']:4s} {p['dt'] * (len(P) - 1):5.2f} | {tr:5.0f} ({di:4.0f}) | "
                      f"{np.round(dev).astype(int).tolist()}")
    if plot_dir:
        plot(name, t, q, rec, rows, tp_all, guard_t, Path(plot_dir))
    return res


def plot(name, t, q, rec, rows, tp_all, guard_t, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.family'] = 'NanumBarunGothic'        # 컨테이너에 있는 한글 글꼴
    plt.rcParams['axes.unicode_minus'] = False
    out.mkdir(parents=True, exist_ok=True)
    T0 = rec['frames'][0]['t']
    col = ['#4e79a7', '#f28e2b', '#59a14f', '#e15759', '#b07aa1', '#76b7b2']
    worst = sorted(rows, key=lambda r: -(r['turns'] * 10 + r['travel'] - r['direct']))[:4]
    legs = {L['label']: L for L in rec['legs']}
    fig, axes = plt.subplots(len(worst), 1, figsize=(13, 3.1 * len(worst)), squeeze=False)
    for ax, r in zip(axes[:, 0], worst):
        L = legs[r['leg']]
        t0, t1 = L['t0'] - 0.3, (L.get('t1') or L['t0'] + 25.0) + 0.3
        m = (t >= t0) & (t <= t1)
        for j in range(6):
            ax.plot(t[m] - T0, np.degrees(q[m, j]), color=col[j], lw=1.3, label=f'J{j + 1}')
        for x, j in tp_all:
            if t0 <= x <= t1:
                ax.plot(x - T0, np.degrees(np.interp(x, t, q[:, j])), 'kx', ms=5)
        for e in rec['events']:
            if not (t0 <= e['t'] <= t1):
                continue
            c = ('red' if ('물러남' in e['text'] or '비킴' in e['text']) else
                 'orange' if ('물러나기 끝' in e['text'] or '비키기 끝' in e['text']) else 'blue' if '새 경로로' in e['text']
                 else 'purple' if 'MPC 발산' in e['text'] else None)
            if c:
                ax.axvline(e['t'] - T0, color=c, lw=1.0, alpha=0.7)
        g = guard_t[(guard_t >= t0) & (guard_t <= t1)]
        ax.plot(g - T0, np.full(len(g), ax.get_ylim()[0]), '|', color='gray', ms=8)
        ax.set_title(f"{r['leg']}: {r['dur']:.1f} s, 관절 움직임 {r['travel']}° (곧은 {r['direct']}°), 갔다 돌아옴 {r['turns']} (x)  "
                     f"— 빨강 물러나기·비키기 시작, 주황 끝, 파랑 새 경로, 보라 MPC 발산, 회색 눈금 안전 검사", fontsize=9)
        ax.set_ylabel('관절 (°)')
        ax.grid(alpha=0.3)
    axes[0, 0].legend(ncol=6, fontsize=8, loc='upper right')
    axes[-1, 0].set_xlabel('시각 (s, 첫 영상 기준)')
    fig.tight_layout()
    p = out / f'{name}_worst_legs.png'
    fig.savefig(p, dpi=80)
    print('  그림:', p)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('dirs', nargs='+', metavar='실행 폴더')
    ap.add_argument('--plot', metavar='png 폴더', help='다리 중 나쁜 4 개를 그림으로')
    a = ap.parse_args(argv)
    res = [analyze(d, a.plot) for d in a.dirs]
    print('\nJSON', json.dumps([{k: v for k, v in r.items() if k != 'legs'} for r in res], ensure_ascii=False))


if __name__ == '__main__':
    main()
