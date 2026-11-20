"""[2026-10-27 R-1] 성형 여유 보존 (rt_run.Controller._cp_slack) 배열 계산 = 예전 점마다 반복 (아래 _cp_slack_loop — 2026-10-27 전 코드
그대로, 비교 기준). 여유·조건 모두 같아야 함 (같은 값이면 앞 점·앞 조건, NaN 은 안 고름, 창이 빈 점은 건너뜀).
rt_run 을 불러오려면 cuRobo 가 있어야 해서 컨테이너에서만 (없으면 건너뜀).   python3 -m pytest   (cr3a_rt_shaping 에서 — pytest.ini)
"""
import dataclasses

import numpy as np
import pytest

from rt_config import Config, GuardCfg

N = 6


def _cp_slack_loop(g, lead_s, uu, cu, pts, c_y):
    """2026-10-27 전 rt_run.Controller._cp_slack (cp_margin_approach·cp_dyn_approach = True 고정, 반환 (여유, 조건))."""
    L, tu, Uq, mv, since = uu['L'], uu['tu'], uu['Uq'], uu['mv'], uu['since']
    lead_ms = lead_s * 1000.0

    def infl(t_ms):
        return g.cp_vobj * np.maximum(0.0, (np.asarray(t_ms, float) + lead_ms) / 1000.0)
    cu_raw, cu_dyn = cu['raw'], cu['dyn'] - infl(tu)
    tms = np.array([int(round(p[0] * 1000)) for p in pts])
    los, his = np.searchsorted(tu, tms - (L - 1), 'left'), np.searchsorted(tu, tms, 'right')
    y_ds = c_y['dyn'] - infl(tms)
    y_now = float(c_y['raw'][0])
    y_now_d = float(c_y['dyn'][0])
    worst, kind = np.inf, 0
    for k in range(1, len(pts)):
        tt, q, v, a = pts[k]
        tm, lo, hi = int(tms[k]), int(los[k]), int(his[k])
        if hi <= lo:
            continue
        j = lo + int(np.argmin(cu_raw[lo:hi]))
        ref = float(cu_raw[j])
        if ref >= g.cp_thr:
            need = g.cp_thr
        else:
            closer = float(c_y['raw'][k]) < y_now - g.cp_eps
            need = ref + (g.cp_margin if (closer and ref < g.cp_margin_zone and np.max(np.abs(q - Uq[j])) > 1e-3) else 0.0)
        s1 = float(c_y['raw'][k]) - need
        if s1 < worst:
            worst, kind = s1, 1
        if g.cp_vobj > 0 and not (float(c_y['dyn'][k]) >= y_now_d - g.cp_eps):
            y_d = float(y_ds[k])
            ref_d = float(cu_dyn[lo:hi].min())
            if ref_d >= 0.0:
                s2 = y_d - min(ref_d, g.cp_margin)
                if s2 < worst:
                    worst, kind = s2, 2
            ku = tm + (L - 1)
            if 0 <= ku < len(mv) and not mv[ku] and float(np.max(np.abs(v))) > 1e-3:
                s3 = float(c_y['dyn'][k]) - g.cp_vobj * since[ku] / 1000.0
                if s3 < worst:
                    worst, kind = s3, 3
    return worst + g.cp_eps, kind


def _case(rng, L):
    """Controller._cp_u·_clearances_with_u 와 같은 모양의 무작위 입력 (거리는 cp_thr 근처, 같은 값·NaN·서 있음 섞음)."""
    n_u = int(rng.integers(101, 400))                         # 후보 u 칸 (지금 + 2 주기 + 멈춤 꼬리)
    nU = (L - 1) + n_u + (L - 1)
    idx = np.arange(0, nU, 5)
    tu = idx - (L - 1)
    mv = rng.random(nU) < 0.6
    since = rng.integers(0, 300, nU)
    Uq = rng.normal(scale=0.3, size=(len(idx), N))
    cu = {'raw': rng.uniform(-0.004, 0.03, len(idx)).astype(np.float32), 'dyn': rng.uniform(-0.004, 0.08, len(idx)).astype(np.float32)}
    t_main = list(np.arange(0, 0.1001, 0.005))
    t_tail = list(np.arange(0.11, 0.11 + 0.01 * int(rng.integers(0, 30)), 0.01))
    ts = [round(x, 6) for x in t_main + t_tail] + [round(max(t_tail + t_main) + 0.01 * float(rng.integers(1, 5)), 6)]
    K = len(ts)
    pts = []
    for k, t in enumerate(ts):
        q = Uq[min(int(rng.integers(0, len(idx))), len(idx) - 1)] + (0 if rng.random() < 0.3 else rng.normal(scale=2e-3, size=N))
        v = np.zeros(N) if rng.random() < 0.3 else rng.normal(scale=0.01, size=N)
        pts.append((t, np.asarray(q, float), v, np.zeros(N)))
    c_y = {'raw': rng.uniform(-0.004, 0.03, K).astype(np.float32), 'dyn': rng.uniform(-0.004, 0.08, K).astype(np.float32)}
    if rng.random() < 0.3:                                   # 같은 값 (같으면 앞 점·앞 조건)
        c_y['raw'][rng.integers(1, K, 4)] = c_y['raw'][1]
        cu['raw'][rng.integers(0, len(idx), 6)] = cu['raw'][0]
    if rng.random() < 0.2:                                   # NaN (예전 반복은 안 고름)
        c_y['dyn'][int(rng.integers(1, K))] = np.nan
        cu['raw'][int(rng.integers(0, len(idx)))] = np.nan
    uu = {'Uq': Uq, 'tu': tu, 'mv': mv, 'since': since, 'L': L}
    return uu, cu, pts, c_y


@pytest.mark.parametrize('vobj', [0.15, 0.0])
def test_cp_slack_equals_loop(vobj):
    rt_run = pytest.importorskip('rt_run')                   # cuRobo 가 있어야 불러와짐 (컨테이너)
    c = Config()
    c = dataclasses.replace(c, guard=dataclasses.replace(c.guard, cp_vobj=vobj))
    ctl = rt_run.Controller.__new__(rt_run.Controller)       # _cp_slack 은 설정만 씀
    ctl.c = c
    rng = np.random.default_rng(11 if vobj else 12)
    kinds = set()
    for trial in range(400):
        L = int(rng.choice([2, 27, 61, 127, 194]))
        uu, cu, pts, c_y = _case(rng, L)
        got = ctl._cp_slack(uu, cu, pts, c_y)
        want = _cp_slack_loop(c.guard, c.stream.lead_s, uu, cu, pts, c_y)
        assert got[1] == want[1] and (got[0] == want[0] or (np.isnan(got[0]) and np.isnan(want[0]))), (trial, L, got, want)
        kinds.add(got[1])
    assert kinds >= ({1, 2, 3} if vobj else {1})             # 세 조건이 다 나와 봄 (물체 속도 0 이면 (1) 만)


def test_cp_slack_short():
    """점이 하나뿐 (앞으로 점 없음) = 여유 inf, 조건 0 (예전 반복과 같음)."""
    rt_run = pytest.importorskip('rt_run')
    ctl = rt_run.Controller.__new__(rt_run.Controller)
    ctl.c = Config()
    rng = np.random.default_rng(13)
    uu, cu, pts, c_y = _case(rng, 127)
    pts, c_y = pts[:1], {k: v[:1] for k, v in c_y.items()}
    assert ctl._cp_slack(uu, cu, pts, c_y) == _cp_slack_loop(GuardCfg(), Config().stream.lead_s, uu, cu, pts, c_y)
