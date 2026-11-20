"""[2026-10-27 R-1] 공개 API 시험 — rt_stream.Stream (past·finish_at·set_stamp), rt_mpc.Mpc (path_at·advance_ref).
Stream 은 numpy 만. Mpc 는 rt_mpc 가 cuRobo 를 불러와서 컨테이너에서만 (없으면 건너뜀) — cuRobo MPC 는 만들지 않고 경로 계산만 봄.
   python3 -m pytest   (cr3a_rt_shaping 에서 — pytest.ini)
"""
import numpy as np
import pytest

from rt_config import MpcCfg, StreamCfg
from rt_motion import track_target
from rt_shaping import ShapingCfg, make_kernel
from rt_stream import Stream

N = 6
VMAX = np.array([3.14, 3.14, 3.93, 3.93, 3.93, 3.93])
AMAX = np.array([6.28, 6.28, 7.78, 7.78, 7.78, 7.78])
JMAX = np.full(N, 500.0)


def _u(st, target):
    """Controller.cycle 과 같은 모양의 후보 u: 지금 상태 + 0.1 s (1 ms 칸)."""
    _, _, _, F = track_target(*st.cmd, target, 20, 0.005, VMAX, AMAX, JMAX, 0.05, fine=True)
    return np.concatenate([np.stack(st.cmd)[None], F])


def _stream(seed, n_msgs):
    """성형 (ZVD 8 Hz) 기억이 있는 Stream 에 메시지 n_msgs 개를 보낸 상태."""
    rng = np.random.default_rng(seed)
    st = Stream(StreamCfg(), 0.05, rng.uniform(-1, 1, N), h=make_kernel(ShapingCfg(method=('ZVD',), freq_hz=(8.0,))).h, mem=200)
    t = 0.0
    for _ in range(n_msgs):
        st.next_stamp(t)
        u = _u(st, rng.uniform(-2, 2, N))
        y = st.shape(u)
        st.sent(st.points(y, 5, 100, 10), tuple(u[50]), u, y)
        t = st.stamp
    st.next_stamp(t)
    return st


def test_past_defaults():
    st = _stream(1, 4)
    assert np.array_equal(st.past(), st.past(st.k))
    assert np.array_equal(st.past_q(30), st.past(None, 30)[:, 0])


def test_finish_at_records_until_k():
    """finish_at(k) = 예전 GUI 서버 비상정지 코드 (메시지를 칸 k 앞까지 기록하고 버림) 와 같은 실행 기록, 기억 = 칸 k 앞 실행된 u."""
    for k_off in (3, 50, 149, 400):                     # 메시지 안 · 끝 넘음 (멈춘 자세로 채움)
        a, b = _stream(2, 5), _stream(2, 5)
        k = a.msg['s'] + k_off
        want_hist = a.past(k, a.mem)
        got = a.finish_at(k)
        # 예전 코드 (rt_server._estop 의 실행 기록 줄)
        n = k - b.msg['s']
        uu = b.msg['u'][:, 0]
        ex = uu[:n] if n <= len(uu) else np.concatenate([uu, np.repeat(uu[-1:], n - len(uu), axis=0)])
        b._seg['chunks'].append((b.msg['s'], ex.copy()))
        b.msg = None
        assert a.msg is None and np.array_equal(got, want_hist) and np.array_equal(a.hist, want_hist)
        a.restart(a.tail_end)
        b.restart(b.tail_end)
        ea, eb = a.executed(), b.executed()
        assert len(ea) == len(eb) and all(np.array_equal(x['q'], y['q']) and x['T0'] == y['T0'] for x, y in zip(ea, eb))
    st = _stream(3, 0)                                  # 메시지 없음: 기록할 것 없음, 기억만
    assert st.msg is None and st.finish_at(st.k).shape == (st.mem, 3, N)


def _sent_old(st, pts, cmd_next, u, y):
    """2026-10-27 전 Stream.sent (finish_at 없이) — 비교 기준."""
    S, k = st.stamp, st.k
    past = st.past(k, st.mem)
    if st.msg is not None:
        n = k - st.msg['s']
        uu = st.msg['u'][:, 0]
        ex = uu[:n] if n <= len(uu) else np.concatenate([uu, np.repeat(uu[-1:], n - len(uu), axis=0)])
        st._seg['chunks'].append((st.msg['s'], ex.copy()))
    st.hist = past
    st.msg = {'s': k, 'u': np.asarray(u, float), 'y': np.asarray(y, float)}
    st.cmd = tuple(np.array(x, float) for x in cmd_next)
    st.pts = [(S + tt, q, v, a) for tt, q, v, a in pts]
    st.tail_end, st.tail_done = np.array(pts[-1][1], float), S + pts[-1][0]


def test_sent_uses_finish_at():
    """sent() 가 finish_at 을 쓰도록 바뀐 뒤에도 예전 sent 와 같은 기억·실행 기록 (안 보냄·다시 시작 섞음)."""
    rng = np.random.default_rng(4)
    h = make_kernel(ShapingCfg(method=('ETM4',), freq_hz=(8.0,))).h
    q0 = rng.uniform(-1, 1, N)
    a, b = Stream(StreamCfg(), 0.05, q0, h=h, mem=150), Stream(StreamCfg(), 0.05, q0, h=h, mem=150)
    t = 0.0
    for i in range(40):
        for st in (a, b):
            st.next_stamp(t)
        r = rng.random()
        if r < 0.15 and a.pts:
            a.kept()
            b.kept()
        elif r < 0.2 and a.pts:
            a.restart(a.tail_end)
            b.restart(b.tail_end)
            continue
        else:
            u = _u(a, rng.uniform(-2, 2, N))
            y = a.shape(u)
            assert np.array_equal(y, b.shape(u))
            pts = a.points(y, 5, 100, 10)
            a.sent(pts, tuple(u[50]), u, y)
            _sent_old(b, pts, tuple(u[50]), u, y)
            assert np.array_equal(a.hist, b.hist)
        t = a.stamp
    ea, eb = a.executed(), b.executed()
    assert len(ea) == len(eb) >= 2
    assert all(np.array_equal(x['q'], y['q']) and x['T0'] == y['T0'] and np.array_equal(x['before'], y['before']) for x, y in zip(ea, eb))


def test_set_stamp_moves_message_time():
    st = _stream(5, 3)
    S, k = st.stamp + 2 * st.dt, st.k + 2 * st.n_dt
    st.set_stamp(S, k)
    assert st.stamp == S and st.k == k
    pts = [(0.0, np.zeros(N), np.zeros(N), np.zeros(N))]
    _, t0, n_pre = st.message(pts)
    assert t0 <= S and n_pre >= 0


def _mpc_stub(path, ref_t=0.0, clock=0.0, t_progress=0.0):
    rt_mpc = pytest.importorskip('rt_mpc')             # cuRobo 가 있어야 불러와짐 (컨테이너)
    m = rt_mpc.Mpc.__new__(rt_mpc.Mpc)                 # cuRobo MPC 는 안 만듦 — 경로 계산만
    m.c, m.dt, m.path = MpcCfg(), 0.05, path
    m.ref_t, m.clock, m.t_progress = ref_t, clock, t_progress
    return m


def _path_at_old(path, tt):
    """2026-10-27 전 Mpc._path_at (점 하나씩) — 비교 기준."""
    t, q = path
    tt = min(max(tt, 0.0), t[-1])
    i = int(np.searchsorted(t, tt, side='right')) - 1
    if i >= len(t) - 1:
        return q[-1].copy()
    a = (tt - t[i]) / max(t[i + 1] - t[i], 1e-9)
    return q[i] + a * (q[i + 1] - q[i])


def test_path_at_vectorized_equals_scalar():
    rng = np.random.default_rng(6)
    for trial in range(100):
        n = int(rng.integers(1, 40))
        t = np.cumsum(np.r_[0.0, rng.uniform(0.0, 0.1, n - 1)])
        if trial % 5 == 0 and n > 2:
            t[2] = t[1]                                 # 같은 시각 두 점
        path = (t, rng.normal(size=(n, N)))
        m = _mpc_stub(path)
        ts = np.concatenate([rng.uniform(-0.2, t[-1] + 0.3, 40), t, [0.0, t[-1]]])
        V = m.path_at(ts)
        for k, x in enumerate(ts):
            old = _path_at_old(path, float(x))
            assert np.array_equal(V[k], old) and np.array_equal(m.path_at(float(x)), old)


def _advance_old(m, q_now, dt):
    """2026-10-27 전 Mpc.step 안의 기준 시각 진행 — 비교 기준."""
    t, q = m.path
    end = float(t[-1])
    win = (t >= m.ref_t - m.c.lookahead_s - 1e-9) & (t <= m.ref_t + m.c.lookahead_s + 0.5)
    idx = np.nonzero(win)[0]
    if len(idx):
        d = np.max(np.abs(q[idx] - q_now), axis=1)
        k = int(np.nonzero(d <= d.min() + 1e-4)[0][-1])
        if d[k] < m.c.follow_tol_rad:
            new = min(m.ref_t + dt, float(t[idx[k]]) + m.c.ahead_s, end)
            if new > m.ref_t + 1e-9:
                m.ref_t = new
                m.t_progress = m.clock
    return {'final': m.ref_t + m.c.lookahead_s >= end - 1e-9, 'at_end': m.ref_t >= end - 1e-9, 'stuck': m.clock - m.t_progress > m.c.stuck_s}


def test_advance_ref_equals_old_step_logic():
    rng = np.random.default_rng(7)
    for _ in range(200):
        n = int(rng.integers(2, 60))
        t = np.arange(n) * 0.05
        q = np.cumsum(rng.normal(scale=0.05, size=(n, N)), axis=0)
        ref, clk = float(rng.uniform(0, t[-1])), float(rng.uniform(0, 10))
        tp = clk - float(rng.uniform(0, 5))
        a, b = _mpc_stub((t, q), ref, clk, tp), _mpc_stub((t, q), ref, clk, tp)
        q_now = q[int(rng.integers(0, n))] + rng.normal(scale=0.1, size=N)
        assert a.advance_ref(q_now, 0.05) == _advance_old(b, q_now, 0.05)
        assert (a.ref_t, a.t_progress) == (b.ref_t, b.t_progress)
