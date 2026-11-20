#!/usr/bin/env python3
"""[2026-10-14 시험 도구] 이상적 모드 기록으로 가짜 WMX 축 로그 (wmx_axis.csv) 를 만들어 rt_report.py 를 WMX 없이 돌려 보기.
  python3 -m tools.fake_wmx_from_ideal <이상적 모드 결과 폴더> <새 폴더>     ([2026-10-27 R-1] tests/ 에서 옮김 — 시험이 아니라 도구)
가짜 로그: CMDPOS = 보낸 명령 (sent_traj, 5 ms 점) 을 3 차 Hermite (위치·속도) 로 1 ms 에 이어 20 ms 늦춘 것, FBPOS = CMDPOS 1 주기 뒤,
CMDVEL = 그 속도. ★ 진짜 WMX 로그가 아님 — 보고서 계산 경로 시험용 ★
"""
import argparse
import csv
import shutil
from pathlib import Path

import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('src', help='이상적 모드 결과 폴더 (rt_run.json 의 sent_traj)')
    ap.add_argument('dst', help='새 폴더 (src 를 복사하고 wmx_axis.csv 를 더함 — 있으면 지우고 다시)')
    a = ap.parse_args()
    src, dst = Path(a.src), Path(a.dst)
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src, dst)
    import json
    rec = json.loads((dst / 'rt_run.json').read_text())
    st = rec['sent_traj']
    t, q, v = np.array(st['t']), np.array(st['q']), np.array(st['v'])
    o = np.argsort(t, kind='stable')
    t, q, v = t[o], q[o], v[o]
    keep = np.r_[True, np.diff(t) > 1e-6]
    t, q, v = t[keep], q[keep], v[keep]
    delay = 0.020
    T = np.arange(t[0] - 0.5, t[-1] + 1.0, 0.001)
    tq = T - delay
    i = np.clip(np.searchsorted(t, tq, side='right') - 1, 0, len(t) - 2)
    h = np.maximum(t[i + 1] - t[i], 1e-9)
    s = np.clip((tq - t[i]) / h, 0.0, 1.0)[:, None]
    p0, p1, m0, m1 = q[i], q[i + 1], v[i] * h[:, None], v[i + 1] * h[:, None]
    big = (h > 0.05)[:, None]                                   # 멈춰 있던 빈 시간 (세그먼트 사이): 직선 대신 앞 점 유지
    Q = (2 * s**3 - 3 * s**2 + 1) * p0 + (s**3 - 2 * s**2 + s) * m0 + (-2 * s**3 + 3 * s**2) * p1 + (s**3 - s**2) * m1
    V = ((6 * s**2 - 6 * s) * p0 + (3 * s**2 - 4 * s + 1) * m0 + (-6 * s**2 + 6 * s) * p1 + (3 * s**2 - 2 * s) * m1) / h[:, None]
    Q = np.where(big, p0, Q)
    V = np.where(big, 0.0, V)
    before = tq < t[0]
    Q[before], V[before] = q[0], 0.0
    FB = np.vstack([Q[:1], Q[:-1]])
    with open(dst / 'wmx_axis.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['SYSTEM_TIME', 'TIME'] + [f'CMDPOS{j}' for j in range(6)] + [f'FBPOS{j}' for j in range(6)] +
                   [f'CMDVEL{j}' for j in range(6)])
        for k in range(len(T)):
            w.writerow([f'{T[k] * 1e7:.0f}', f'{(T[k] - T[0]) * 1e6:.0f}'] + [f'{x:.7f}' for x in Q[k]] + [f'{x:.7f}' for x in FB[k]] +
                       [f'{x:.6f}' for x in V[k]])
    print('가짜 WMX 로그:', dst / 'wmx_axis.csv', len(T), '줄')


if __name__ == '__main__':
    main()
