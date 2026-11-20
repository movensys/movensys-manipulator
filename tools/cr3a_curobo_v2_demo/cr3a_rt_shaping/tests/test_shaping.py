"""rt_shaping · rt_stream (입력성형 기억) 단위 시험 (numpy 만, GPU·ROS 없음).   python3 -m pytest   (cr3a_rt_shaping 에서 — pytest.ini)
"""
import math

import numpy as np

from rt_config import StreamCfg
from rt_motion import acc_bounds, brake, brake_fine, braking_tail, track_target
from rt_shaping import ShapingCfg, combine, impulses, make_kernel, residual, shape_fir, to_grid
from rt_stream import Stream

N = 6
VMAX = np.array([3.14, 3.14, 3.93, 3.93, 3.93, 3.93])
AMAX = np.array([6.28, 6.28, 7.78, 7.78, 7.78, 7.78])
JMAX = np.full(N, 500.0)
F0, Z0 = 8.0, 0.02


def test_impulses_basic():
    """세기 양수·합 1, 설계 주파수에서 잔류 0 (EI 는 허용 잔류 V), 길이 = 감쇠 주기의 반 (ZV) / 한 주기 (나머지)."""
    Td = 1.0 / (F0 * math.sqrt(1 - Z0 * Z0))
    for m in ('ZV', 'ZVD', 'EI', 'ETM4', 'ETM5'):
        A, t = impulses(m, F0, Z0)
        assert np.all(A > 0) and abs(A.sum() - 1) < 1e-12, m
        assert abs(t[-1] - (Td / 2 if m == 'ZV' else Td)) < 1e-12, m
        V = residual(A, t, F0, Z0)
        if m == 'EI':
            assert abs(V - 0.05) < 2e-3, V            # Singhose 근사식 (설계 주파수에서 5 % 를 남기는 성형기)
        else:
            assert V < 1e-12, (m, V)
    # ZVD·ETM: 설계 주파수 근처에서 잔류가 2 차 이상으로 작음 (주파수 미분 0) — ZV 는 1 차
    eps = 0.01
    for m in ('ZVD', 'ETM4', 'ETM5'):
        A, t = impulses(m, F0, Z0)
        assert residual(A, t, F0 * (1 + eps), Z0) < 0.01, m
    A, t = impulses('ZV', F0, Z0)
    assert residual(A, t, F0 * (1 + eps), Z0) > 0.01
    # 감쇠 없으면 ETM4 (m=1) = [1, 2, 2, 1]/6, ETM5 = [1, 2, 2, 2, 1]/8 (Kang 2019 의 대칭형)
    assert np.allclose(impulses('ETM4', 10, 0.0)[0], np.array([1, 2, 2, 1]) / 6)
    assert np.allclose(impulses('ETM5', 10, 0.0)[0], np.array([1, 2, 2, 2, 1]) / 8)


def test_grid_and_multi():
    """1 ms 격자: 합 1·양수·평균 지연 그대로, 설계 주파수 잔류 < 0.05 %. 여러 주파수 = 컨볼루션, 각 주파수에서 잔류 ≈ 0."""
    for m in ('ZV', 'ZVD', 'ETM4', 'ETM5'):
        A, t = impulses(m, F0, Z0)
        h = to_grid(A, t)
        tg = np.arange(len(h)) * 1e-3
        assert np.all(h >= 0) and abs(h.sum() - 1) < 1e-12
        assert abs(np.sum(h * tg) - np.sum(A * t)) < 1e-12
        assert residual(h, tg, F0, Z0) < 5e-4, (m, residual(h, tg, F0, Z0))
    k = make_kernel(ShapingCfg(method=('ZVD',), freq_hz=(8.0, 15.0), zeta=(0.02,)))
    assert len(k.A) == 9 and k.L <= 194
    tg = np.arange(k.L) * 1e-3
    for f in (8.0, 15.0):
        assert residual(k.h, tg, f, Z0) < 5e-4, (f, residual(k.h, tg, f, Z0))
    # 같은 시각 임펄스는 하나로 합침
    A, t = combine([impulses('ZV', 10, 0.0), impulses('ZV', 10, 0.0)])
    assert len(A) == 3 and np.allclose(A, [0.25, 0.5, 0.25])
    # 주파수마다 다른 방식
    k2 = make_kernel(ShapingCfg(method=('ZVD', 'ZV'), freq_hz=(8.0, 15.0), zeta=(0.02, 0.05)))
    assert len(k2.A) == 6 and [m for m, _, _ in k2.modes] == ['ZVD', 'ZV']
    assert make_kernel(ShapingCfg()).L == 1


def test_shaped_limits():
    """양수·합 1 커널: 성형된 속도·가속의 최대 ≤ 성형 전 최대 (임의 궤적)."""
    rng = np.random.default_rng(5)
    h = make_kernel(ShapingCfg(method=('ETM5',), freq_hz=(6.0,), zeta=(0.05,))).h
    for _ in range(5):
        q0 = rng.uniform(-1, 1, N)
        _, _, _, F = track_target(q0, np.zeros(N), np.zeros(N), q0 + rng.uniform(-2, 2, N), 200, 0.005, VMAX, AMAX, JMAX, 0.05, fine=True)
        for i in (1, 2):
            y = shape_fir(F[:, i], h, before=np.zeros(N))
            assert np.all(np.abs(y).max(axis=0) <= np.abs(F[:, i]).max(axis=0) + 1e-12)


def test_brake_fine_matches_brake():
    rng = np.random.default_rng(2)
    for _ in range(10):
        q0, v0 = rng.uniform(-2, 2, N), rng.uniform(-1, 1, N) * VMAX
        lo, hi = acc_bounds(v0, VMAX, AMAX, JMAX)
        a0 = rng.uniform(lo, hi)
        F, t_stop = brake_fine(q0, v0, a0, AMAX, JMAX)
        Q, V, A, t2 = brake(q0, v0, a0, np.arange(1, len(F) + 1) * 1e-3, AMAX, JMAX)
        assert abs(t_stop - t2) < 1e-12 and np.array_equal(F[:, 0], Q) and np.array_equal(F[:, 1], V)


def _candidate(st, target, rng, brake_now=False):
    """Controller.cycle 과 같은 후보 u [N, 3, J]: 지금 상태 + 0.1 s (1 ms) + 멈춤 꼬리 (1 ms)."""
    s0 = st.cmd
    if brake_now:
        F, _ = brake_fine(*s0, AMAX, JMAX)
        F = F[:100] if len(F) >= 100 else np.concatenate([F, np.repeat(F[-1:], 100 - len(F), axis=0)])
    else:
        _, _, _, F = track_target(*s0, target, 20, 0.005, VMAX, AMAX, JMAX, 0.05, fine=True)
    T, _ = brake_fine(*F[-1], AMAX, JMAX)
    return np.concatenate([np.stack(s0)[None], F, T])


def _run_stream(h, seed, n_cycles=300):
    """실시간 흉내: 보냄 / 안 보냄 (앞 메시지 계속) / 늦어서 격자 미룸 / 정지 / 다시 시작을 섞어 돌리고
    (실행된 y, 실행된 u 기록) 을 돌려줌. 실행된 y = 메시지마다 다음 메시지가 갈아탈 때까지의 y 칸."""
    rng = np.random.default_rng(seed)
    c = StreamCfg()
    q0 = rng.uniform(-1, 1, N)
    st = Stream(c, 0.05, q0, h=h)
    clock = [0.0]
    executed_y = []            # 세그먼트마다 [(시작 칸, y 칸들)]
    seg_y = []
    target = q0.copy()
    for cyc in range(n_cycles):
        if st.stamp is None:
            st.next_stamp(clock[0])
        else:
            st.next_stamp(clock[0])
            if rng.random() < 0.05:                    # 늦어서 다음 격자로 (실시간 catch_up)
                lag = st.stamp - clock[0] - c.late_before_s + 0.051 * rng.integers(1, 3)
                st.catch_up(lambda: clock[0] + lag)
        if rng.random() < 0.1:
            target = rng.uniform(-2, 2, N)
        r = rng.random()
        if r < 0.1 and st.pts:                         # 안 보냄 → 앞 메시지 계속
            st.kept()
        elif r < 0.13 and st.pts:                      # 다시 시작 (꼬리로 멈춘 뒤)
            st.restart(st.tail_end)
            if seg_y:
                executed_y.append(seg_y)
            seg_y = []
            st.next_stamp(clock[0])
            u = _candidate(st, target, rng)
            y = st.shape(u)
            st.sent(st.points(y, 5, 100, 10), tuple(u[50]), u, y)
            seg_y.append((st.k, y))
        else:
            u = _candidate(st, target, rng, brake_now=r > 0.95)
            y = st.shape(u)
            # 스탬프에서 앞 메시지 y 와 이어짐 (위치·속도·가속 같음)
            if st.msg is not None:
                assert np.allclose(st.y_state(st.k), y[0], atol=1e-12)
            st.sent(st.points(y, 5, 100, 10), tuple(u[50]), u, y)
            seg_y.append((st.k, y))
        clock[0] = st.stamp
    executed_y.append(seg_y)
    ex = st.executed()
    out = []
    for seg_u, sy in zip(ex, executed_y):
        n = len(seg_u['q'])
        ys = np.zeros((n, N))
        for i, (s, y) in enumerate(sy):
            e = sy[i + 1][0] if i + 1 < len(sy) else n
            m = min(e - s, len(y))
            ys[s:s + m] = y[:m, 0]
            if e - s > len(y):
                ys[s + len(y):e] = y[-1, 0]
        out.append((seg_u, ys))
    return out


def test_stream_equals_full_convolution():
    """실행된 메시지들의 y 를 이어 붙인 것 = 실행된 u 전체를 한 번에 컨볼루션 (다른 계산 경로) — 늦음·정지·다시 시작 포함."""
    for cfg in (ShapingCfg(method=('ZVD',), freq_hz=(8.0,)), ShapingCfg(method=('ZV',), freq_hz=(8.0, 15.0)),
                ShapingCfg(method=('EI',), freq_hz=(5.3,), zeta=(0.07,)), ShapingCfg(method=('ETM5',), freq_hz=(11.0,))):
        h = make_kernel(cfg).h
        for seed in range(3):
            res = _run_stream(h, seed)
            assert len(res) >= 2
            for seg_u, ys in res:
                y_full = shape_fir(seg_u['q'], h, before=seg_u['before'])
                # 기록 끝 (마지막 메시지의 u 끝) 뒤로 y 가 L−1 칸 더 움직이므로 u 기록 길이까지만 비교
                err = np.abs(y_full - ys[:len(y_full)]).max()
                assert err < 1e-12, (cfg.label(), seed, err)


def test_none_matches_old_points():
    """성형 없음: 메시지 점 = 2026-10-14 전 코드 (track_target 5 ms 점 + braking_tail) 와 같음."""
    rng = np.random.default_rng(7)
    st = Stream(StreamCfg(), 0.05, np.zeros(N))
    for _ in range(20):
        st.next_stamp(0.0)
        q0, v0 = rng.uniform(-2, 2, N), rng.uniform(-1, 1, N) * VMAX
        lo, hi = acc_bounds(v0, VMAX, AMAX, JMAX)
        s0 = (q0, v0, rng.uniform(lo, hi))
        st.cmd = s0
        tgt = q0 + rng.uniform(-1, 1, N)
        Q, V, A = track_target(*s0, tgt, 20, 0.005, VMAX, AMAX, JMAX, 0.05)
        tail = braking_tail(0.1, Q[-1], V[-1], A[-1], AMAX, JMAX, dt=0.01)
        old = [(0.0, *s0)] + list(zip(np.arange(1, 21) * 0.005, Q, V, A)) + tail
        u = _candidate(st, tgt, rng)
        new = st.points(st.shape(u), 5, 100, 10)
        assert len(old) == len(new)
        for (t1, *x1), (t2, *x2) in zip(old, new):
            assert abs(t1 - t2) < 1e-9 and all(np.array_equal(a, b) for a, b in zip(x1, x2)), (t1, t2)
