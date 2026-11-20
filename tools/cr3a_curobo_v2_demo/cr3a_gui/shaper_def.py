#!/usr/bin/env python3
"""입력성형기 정의 (화면 없음 — GUI 패널 shaping_panel.py 와 서버 rt_server.py 가 같이 씀, 성형기 정의는 한 곳).

방식: Unshaped · ZV · ETM4 · ETM5 (켠 주파수 모두 같은 방식, ETM m = 1). 주파수는 3 개까지.
[사용자 2026-10-26] 감쇠 없는 시스템으로 가정 (ζ = 0): ZV [½, ½] · ETM4 [1/6, 1/3, 1/3, 1/6] · ETM5 [1/8, ¼, ¼, ¼, 1/8].
[사용자 2026-10-26] "엄밀한 값으로 계산 후 컨볼루션해서 나온 최종 성형기의 시간 단위를 1 ms 로": 주파수마다 정확한 임펄스 →
  정확한 시각으로 컨볼루션 (cr3a_rt_shaping/rt_shaping.combine) → 최종 임펄스마다 가장 가까운 1 ms 칸 (반올림, 최대 0.5 ms 옮김)
  → 같은 칸에 모인 임펄스는 하나로 (세기 합). 대가: 설계 주파수에 진동이 조금 남음 (ζ 0: ZV 8 Hz 1.3 %, 3 주파수 예 0~1.3 %).
  (시뮬레이션의 rt_shaping.make_kernel 은 칸 사이 임펄스를 양옆 두 칸에 나눔 — GUI·서버는 이 정의를 씀)
화면 표시: "임펄스 x 개 · 지연 시간 x ms" — 지연 시간 = 성형기 길이 = 모션이 끝나는 시각이 늦어지는 시간.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'cr3a_rt_shaping'))
import rt_shaping  # noqa: E402  (성형기 식: 작업 사본 그대로)

METHODS = ('Unshaped', 'ZV', 'ETM4', 'ETM5')
ZETA = 0.0                       # 감쇠 없는 시스템으로 가정 (사용자 2026-10-26)
GRID_S = 0.001                   # 임펄스를 놓는 칸 = WMX 통신 주기 1 ms
WARN_RESIDUAL = 0.02             # 1 ms 반올림 때문에 설계 주파수에 남는 진동 비가 이보다 크면 알림


def build(method, rows):
    """rows = [(켬, Hz)] → dict: 최종 임펄스 (A, t s — 모두 1 ms 칸), 켠 주파수마다 남는 진동 비 resid (1 ms 반올림 몫).
    [2026-10-27 R-1] 아무도 안 쓰던 n0 (컨볼루션한 정확한 임펄스 수)·shift (칸으로 옮긴 시각 최대)·delay (평균 지연) 는 뺌."""
    on = [f for use, f in rows if use]
    if method == 'Unshaped' or not on:
        return {'A': np.array([1.0]), 't': np.array([0.0]), 'resid': {}}
    A0, t0 = rt_shaping.combine([rt_shaping.impulses(method, f, ZETA) for f in on])   # 정확한 값으로 컨볼루션
    k = np.floor(np.round(t0 / GRID_S, 6) + 0.5).astype(int)                            # 최종 성형기를 1 ms 칸으로 (반올림)
    h = np.bincount(k, weights=A0)                                                     # 같은 칸에 모인 임펄스는 하나로
    nz = np.flatnonzero(h > 0)
    A, t = h[nz], nz * GRID_S
    return {'A': A, 't': t, 'resid': {f: rt_shaping.residual(A, t, f, ZETA) for f in on}}


def label(method, rows):
    fs = [f'{f:g}' for use, f in rows if use]
    return 'Unshaped (성형 없음)' if method == 'Unshaped' or not fs else f'{method} {" + ".join(fs)} Hz'


def kernel(method, rows) -> 'rt_shaping.Kernel':
    """실시간 성형기 (rt_stream.Stream 의 h) — 1 ms 칸 세기 h[k] (k ms 의 세기)."""
    res = build(method, rows)
    k = np.round(res['t'] / GRID_S).astype(int)
    h = np.zeros(int(k[-1]) + 1)
    h[k] = res['A']
    modes = tuple((method.upper(), float(f), ZETA) for use, f in rows if use) if method != 'Unshaped' else ()
    return rt_shaping.Kernel(h=h, grid_s=GRID_S, A=res['A'], t=res['t'], modes=modes)


def ms(seconds):
    """초 → 정수 ms (0.5 는 올림 — 199.5 → 200, 부동소수 잔차 무시)."""
    return int(math.floor(round(seconds * 1000, 6) + 0.5))
