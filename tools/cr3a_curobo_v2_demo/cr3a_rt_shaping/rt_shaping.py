"""입력성형기 (순수 numpy, GPU·ROS 없음): ZV · ZVD · EI · ETM4 · ETM5, 다중 주파수, 1 ms 격자.

정의 (f = 진동 주파수 Hz, ζ = 감쇠비, ω_d = 2πf·√(1−ζ²), T_d = 2π/ω_d, K = exp(ζπ/√(1−ζ²)) ≥ 1).
임펄스 (세기 A, 시각 t). 세기 합 = 1, 모두 양수 → 성형된 명령의 속도·가속·저크는 원래 명령의 최대를 넘지 않는다.
  ZV    t = [0, T_d/2]                    A = [K, 1] / (K+1)
  ZVD   t = [0, T_d/2, T_d]               A = [K², 2K, 1] / (K+1)²
  EI    t = [0, t2, T_d]                  Singhose 근사식 (허용 잔류 V = ei_vtol: 설계 주파수에서 V 만큼 남기고 양옆 두 곳에서 0 →
                                           주파수 오차에 가장 둔감. 8 Hz ζ0.02 V 5 %: 0.80~1.21 배에서 잔류 5 % 이하)
  ETM4  t = [0, 1, 2, 3]·T_d/3            A = I·[1/(1+m), K^-2/3, K^-4/3, m/((1+m)K²)],  I = (1+m)K² / (K² + (1+m)(K^4/3 + K^2/3) + m)
  ETM5  t = [0, 1, 2, 3, 4]·T_d/4         A = I·[1/(1+m), K^-1/2, K^-1, K^-3/2, m/((1+m)K²)],  I = (1+m)K² / (K² + (1+m)(K^3/2 + K + K^1/2) + m)
  ETM = Equal shaping-Time and Magnitude (C.-G. Kang, "Impulse vectors for input shaping control", IEEE Control Systems Magazine
  39(4), 2019): 임펄스 벡터 크기 I_2 = … = I_{n−1} = I_1 + I_n, I_n = m·I_1. m = 1 이 견디는 주파수 범위가 가장 넓음 (8 Hz: ETM4
  0.93~1.13 배, m 0.5·2 는 0.93~1.09). ZVD 는 ETM3 (m = 1).
주파수가 여러 개면 주파수마다 만든 임펄스 열을 정확한 시각으로 컨볼루션 (모든 세기의 곱, 모든 시각의 합).
1 ms 격자 (WMX 통신 주기, [사용자 질문 2026-10-14] "1 ms 안 간격의 임펄스는 합쳐서 1 ms 단위로"): 시각 τ 인 임펄스 a 를 양옆 칸
  ⌊τ⌋, ⌊τ⌋+1 에 (1−r)·a, r·a 로 나눠 넣고 (r = τ − ⌊τ⌋, 격자 단위) 같은 칸끼리 더한다 (1 ms 안에 모인 임펄스는 저절로 합쳐짐).
  세기 합·양수·평균 지연이 그대로이고, 반올림 (가장 가까운 칸) 보다 정확: 8 Hz ζ0.02 설계 주파수 잔류 ZVD 반올림 1.18 % / 나눔 0.014 %.
성형 = 1 ms 격자 위 FIR: y[k] = Σ_i h[i]·u[k − i]. 실시간에는 실행된 명령 u 를 기억해서 (rt_stream) 이어 붙인 명령 전체의 컨볼루션이 된다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

METHODS = ('none', 'ZV', 'ZVD', 'EI', 'ETM4', 'ETM5')


def _K(zeta: float) -> float:
    return math.exp(zeta * math.pi / math.sqrt(1.0 - zeta * zeta))


def impulses(method: str, f_hz: float, zeta: float, ei_vtol: float = 0.05, etm_m: float = 1.0) -> Tuple[np.ndarray, np.ndarray]:
    """주파수 하나의 성형기 임펄스 (세기 A, 시각 t s)."""
    m = method.upper()
    if m == 'NONE':
        return np.array([1.0]), np.array([0.0])
    if not (f_hz > 0.0 and 0.0 <= zeta < 1.0):
        raise ValueError(f'{method}: 주파수 > 0, 0 ≤ 감쇠비 < 1 이어야 함 (f {f_hz}, ζ {zeta})')
    Td = 1.0 / (f_hz * math.sqrt(1.0 - zeta * zeta))
    K = _K(zeta)
    if m == 'ZV':
        A, t = [K, 1.0], [0.0, Td / 2]
        A = [a / (K + 1) for a in A]
    elif m == 'ZVD':
        A, t = [K * K, 2 * K, 1.0], [0.0, Td / 2, Td]
        A = [a / (K + 1) ** 2 for a in A]
    elif m == 'EI':
        # Singhose, Seering, Singer 근사식 (감쇠 ζ ≤ 0.3, V ≤ 0.2 에서 맞춤) — Klipper·교재와 같은 식
        V, z = ei_vtol, zeta
        a1 = 0.24968 + 0.24961 * V + 0.80008 * z + 1.23328 * V * z + 0.49599 * z * z + 3.17316 * V * z * z
        a3 = 0.25149 + 0.21474 * V - 0.83249 * z + 1.41498 * V * z + 0.85181 * z * z - 4.90094 * V * z * z
        t2 = (0.4999 + 0.46159 * V * z + 4.26169 * V * z * z + 1.75601 * V * z ** 3
              + 8.57843 * V * V * z - 108.644 * V * V * z * z + 336.989 * V * V * z ** 3)
        A, t = [a1, 1.0 - a1 - a3, a3], [0.0, t2 * Td, Td]
    elif m == 'ETM4':
        mm = etm_m
        I = (1 + mm) * K * K / (K * K + (1 + mm) * (K ** (4 / 3) + K ** (2 / 3)) + mm)
        A = [I / (1 + mm), I / K ** (2 / 3), I / K ** (4 / 3), mm * I / ((1 + mm) * K * K)]
        t = [0.0, Td / 3, 2 * Td / 3, Td]
    elif m == 'ETM5':
        mm = etm_m
        I = (1 + mm) * K * K / (K * K + (1 + mm) * (K ** 1.5 + K + K ** 0.5) + mm)
        A = [I / (1 + mm), I / K ** 0.5, I / K, I / K ** 1.5, mm * I / ((1 + mm) * K * K)]
        t = [0.0, Td / 4, Td / 2, 3 * Td / 4, Td]
    else:
        raise ValueError(f'성형 방식 {method!r} — {METHODS} 중 하나')
    A, t = np.array(A, float), np.array(t, float)
    if np.any(A < 0) or abs(A.sum() - 1.0) > 1e-9:
        raise ValueError(f'{method} f {f_hz} ζ {zeta}: 세기가 음수이거나 합이 1 이 아님 {A}')
    return A, t


def combine(trains: Sequence[Tuple[np.ndarray, np.ndarray]]) -> Tuple[np.ndarray, np.ndarray]:
    """임펄스 열들의 컨볼루션 (정확한 시각): 세기는 곱, 시각은 합. 같은 시각은 합침."""
    A, t = np.array([1.0]), np.array([0.0])
    for a2, t2 in trains:
        A, t = np.outer(A, a2).ravel(), np.add.outer(t, t2).ravel()
    o = np.argsort(t, kind='stable')
    A, t = A[o], t[o]
    keep = np.r_[True, np.diff(t) > 1e-12]
    grp = np.cumsum(keep) - 1
    return np.bincount(grp, weights=A), t[keep]


def to_grid(A: np.ndarray, t: np.ndarray, grid_s: float = 0.001) -> np.ndarray:
    """임펄스를 격자 칸 세기 h[k] (시각 k·grid_s) 로: 양옆 두 칸에 거리 비율로 나눠 넣고 같은 칸끼리 더함."""
    d = np.asarray(t, float) / grid_s
    i = np.floor(d + 1e-9).astype(int)
    r = np.clip(d - i, 0.0, 1.0)
    r[r < 1e-9] = 0.0
    h = np.zeros(int(i.max()) + 2)
    np.add.at(h, i, A * (1.0 - r))
    np.add.at(h, i + 1, A * r)
    while len(h) > 1 and h[-1] == 0.0:
        h = h[:-1]
    return h


def residual(A, t, f_hz: float, zeta: float) -> float:
    """임펄스 열 (A, t) 이 진동 모드 (f, ζ) 에 남기는 잔류 진동 비 (성형 없음 = 1). 격자 칸 세기도 (h, k·grid) 로 넣으면 됨."""
    A, t = np.asarray(A, float), np.asarray(t, float)
    w = 2 * math.pi * f_hz
    wd = w * math.sqrt(1 - zeta * zeta)
    e = np.exp(zeta * w * t)
    return float(math.exp(-zeta * w * t[-1]) * math.hypot(np.sum(A * e * np.cos(wd * t)), np.sum(A * e * np.sin(wd * t))))


@dataclass(frozen=True)
class Kernel:
    """1 ms 격자 위 성형 커널. h[k] = 시각 k·grid_s 의 세기 (h[0] 이 지금 명령)."""
    h: np.ndarray
    grid_s: float
    A: np.ndarray                       # 격자에 넣기 전 정확한 임펄스 (컨볼루션 뒤)
    t: np.ndarray
    modes: Tuple[Tuple[str, float, float], ...] = ()     # (방식, 주파수 Hz, 감쇠비)

    @property
    def L(self) -> int:
        return len(self.h)

    @property
    def duration_s(self) -> float:
        return (self.L - 1) * self.grid_s

    @property
    def mean_delay_s(self) -> float:
        return float(np.sum(self.h * np.arange(self.L)) * self.grid_s)

    def residuals(self, scales=(0.8, 0.9, 1.0, 1.1, 1.2)) -> dict:
        """설계 주파수마다 실제 주파수가 그 배수일 때 잔류 진동 비 (1 ms 격자 커널 기준)."""
        tg = np.arange(self.L) * self.grid_s
        return {f'{m}@{f:g}Hz': {f'{s:g}': round(residual(self.h, tg, f * s, z), 5) for s in scales} for m, f, z in self.modes}

    def info(self) -> dict:
        return {'modes': [list(x) for x in self.modes], 'grid_ms': self.grid_s * 1000, 'taps': len(self.h),
                'impulses': [[round(float(a), 9), round(float(t) * 1000, 6)] for a, t in zip(self.A, self.t)],
                'h': [round(float(x), 12) for x in self.h], 'duration_ms': round(self.duration_s * 1000, 3),
                'mean_delay_ms': round(self.mean_delay_s * 1000, 3), 'residual': self.residuals()}


@dataclass(frozen=True)
class ShapingCfg:
    """입력성형 설정 — 파일 shaping.yaml 에서 읽고 (load_cfg), 명령줄로 덮음 (rt_run --shaper/--shaper-hz/--shaper-zeta).
    method 가 하나면 모든 주파수에 같게, freq_hz 와 같은 길이 목록이면 주파수마다. zeta 도 같음."""
    method: Tuple[str, ...] = ('none',)
    freq_hz: Tuple[float, ...] = ()
    zeta: Tuple[float, ...] = (0.02,)
    ei_vtol: float = 0.05
    etm_m: float = 1.0
    grid_ms: float = 1.0

    def modes(self) -> List[Tuple[str, float, float]]:
        meth = [m.strip() for m in self.method]
        if len(meth) == 1 and meth[0].lower() == 'none':
            return []
        if not self.freq_hz:
            raise ValueError(f'성형 {meth}: 주파수 (freq_hz) 가 없음')
        n = len(self.freq_hz)

        def pick(xs, name):
            if len(xs) == 1:
                return list(xs) * n
            if len(xs) != n:
                raise ValueError(f'{name} 개수 {len(xs)} ≠ 주파수 개수 {n}')
            return list(xs)
        meths, zs = pick(meth, 'method'), pick(self.zeta, 'zeta')
        for m in meths:
            if m.upper() not in [x.upper() for x in METHODS[1:]]:
                raise ValueError(f'성형 방식 {m!r} — {METHODS} 중 하나')
        return [(m.upper(), float(f), float(z)) for m, f, z in zip(meths, self.freq_hz, zs)]

    def label(self) -> str:
        md = self.modes()
        return 'none' if not md else '+'.join(f'{m}{f:g}' for m, f, _ in md)


def make_kernel(c: ShapingCfg) -> Kernel:
    md = c.modes()
    g = c.grid_ms / 1000.0
    if not md:
        return Kernel(h=np.array([1.0]), grid_s=g, A=np.array([1.0]), t=np.array([0.0]), modes=())
    A, t = combine([impulses(m, f, z, c.ei_vtol, c.etm_m) for m, f, z in md])
    return Kernel(h=to_grid(A, t, g), grid_s=g, A=A, t=t, modes=tuple(md))


def _as_tuple(x, typ):
    if x is None:
        return None
    if isinstance(x, (list, tuple)):
        return tuple(typ(v) for v in x)
    if isinstance(x, str) and typ is not str:
        return tuple(typ(v) for v in x.replace(',', ' ').split())
    return (typ(x),)


def load_cfg(path=None, method=None, freq_hz=None, zeta=None, **kw) -> ShapingCfg:
    """파일 (yaml) → ShapingCfg, 그 위에 인자 (None 이 아닌 것만) 를 덮음. 파일이 없으면 기본 (성형 없음)."""
    d = {}
    if path:
        import yaml
        with open(path, encoding='utf-8') as f:
            d = yaml.safe_load(f) or {}
    for k, v in (('method', method), ('freq_hz', freq_hz), ('zeta', zeta)):
        if v is not None:
            d[k] = v
    d.update({k: v for k, v in kw.items() if v is not None})
    out = {}
    if 'method' in d:
        out['method'] = _as_tuple(d['method'], str)
    if 'freq_hz' in d:
        out['freq_hz'] = _as_tuple(d['freq_hz'], float) or ()
    if 'zeta' in d:
        out['zeta'] = _as_tuple(d['zeta'], float)
    for k in ('ei_vtol', 'etm_m', 'grid_ms'):
        if k in d:
            out[k] = float(d[k])
    c = ShapingCfg(**out)
    c.modes()                                   # 설정 검사
    return c


def shape_fir(u: np.ndarray, h: np.ndarray, before=None) -> np.ndarray:
    """전체 컨볼루션 (검증용, 실시간 경로와 다른 계산): u [N, …] 를 1 ms 격자 FIR h 로. before = 시작 전 값 (없으면 u[0] 유지).
    y[k] = Σ_i h[i]·u[k − i], k = 0..N−1."""
    u = np.asarray(u, float)
    pre = np.repeat((u[:1] if before is None else np.asarray(before, float)[None]), len(h) - 1, axis=0)
    x = np.concatenate([pre, u]).reshape(len(pre) + len(u), -1)
    y = np.stack([np.convolve(x[:, j], h)[len(h) - 1:len(h) - 1 + len(u)] for j in range(x.shape[1])], axis=1)
    return y.reshape(u.shape)
