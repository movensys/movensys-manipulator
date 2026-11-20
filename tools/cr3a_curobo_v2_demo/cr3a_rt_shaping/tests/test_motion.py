"""rt_motion · rt_stream 단위 시험 (numpy 만, GPU·ROS 없음).   python3 -m pytest   (cr3a_rt_shaping 에서 — pytest.ini)
"""
import numpy as np

from rt_config import StreamCfg
from rt_motion import acc_bounds, brake, braking_tail, clamp_final, stop_speed, track_target
from rt_stream import Stream

N = 6
VMAX = np.array([3.14, 3.14, 3.93, 3.93, 3.93, 3.93])
AMAX = np.array([6.28, 6.28, 7.78, 7.78, 7.78, 7.78])
JMAX = np.full(N, 500.0)
H, CYCLE, EPS = 0.005, 0.05, 1e-9


def _state(rng):
    """한계 안의 무작위 상태 (가속은 속도 한계에 저크로 닿을 수 있는 범위 안: rt_motion.acc_bounds — 명령 궤적이 늘 지키는 조건)."""
    q0 = rng.uniform(-2, 2, N)
    v0 = rng.uniform(-1, 1, N) * VMAX
    lo, hi = acc_bounds(v0, VMAX, AMAX, JMAX)
    return q0, v0, rng.uniform(lo, hi)


def _within_limits(Q, V, A, a0, h, snap=False):
    """속도·가속 한계, 점 사이 저크 ≤ jmax·h (적분 단계마다 jmax·hh, 합 ≤ jmax·h).
    snap = brake 의 멈추는 한 단계 예외: 남은 가속 ≤ 2·jmax·h 를 0 으로 놓음 (rt_motion.brake '이산화 떨림 크기') → 그 단계만 ≤ 3·jmax·h."""
    assert np.all(np.abs(V) <= VMAX + EPS)
    assert np.all(np.abs(A) <= AMAX + EPS)
    dA = np.abs(np.diff(np.vstack([a0, A]), axis=0))
    lim = np.where(snap & (A == 0) & (V == 0), 3.0, 1.0) * JMAX * h
    assert np.all(dA <= lim + 1e-6), dA.max()
    assert np.all(np.isfinite(Q))


def test_track_target_limits_random():
    rng = np.random.default_rng(0)
    for _ in range(50):
        q0, v0, a0 = _state(rng)
        tgt = q0 + rng.uniform(-1.5, 1.5, N)
        Q, V, A = track_target(q0, v0, a0, tgt, 40, H, VMAX, AMAX, JMAX, CYCLE)
        _within_limits(Q, V, A, a0, H)


def test_track_target_reaches_fixed_target():
    q0 = np.zeros(N)
    tgt = np.full(N, 0.5)
    Q, V, A = track_target(q0, np.zeros(N), np.zeros(N), tgt, 600, H, VMAX, AMAX, JMAX, CYCLE)
    assert np.max(np.abs(Q[-1] - tgt)) < 1e-4
    assert np.max(np.abs(V[-1])) < 1e-3


def _track_old(q0, v0, a0, target, n, h, vmax, amax, jmax, cycle, sub=5):
    """2026-10-13 전 track_target (멈출 수 있는 속도 = √(2·a·e), 저크 무시) — 지나침 비교 기준."""
    q, v, a = np.array(q0, float), np.array(v0, float), np.array(a0, float)
    hh, Q = h / sub, np.zeros((n, q.size))
    for k in range(n):
        for _ in range(sub):
            e = target - q
            v_des = np.clip(np.clip(e / cycle, -np.sqrt(2 * amax * np.abs(e)), np.sqrt(2 * amax * np.abs(e))), -vmax, vmax)
            lo, hi = acc_bounds(v, vmax, amax, jmax)
            an = np.clip(np.clip(np.clip((v_des - v) / max(cycle / 4.0, hh), -amax, amax), lo, hi), a - jmax * hh, a + jmax * hh)
            q, v, a = q + v * hh + 0.5 * an * hh * hh, np.clip(v + an * hh, -vmax, vmax), an
        Q[k] = q
    return Q


def test_track_target_overshoot_less_than_before():
    """저크까지 넣은 멈춤 속도 (stop_speed) 는 예전 √(2·a·e) 보다 고정 목표를 덜 지나치고 (0.05 rad 넘는 거리에서 절반 이하) 끝에 닿음."""
    for e0, v0 in [(0.05, 0.0), (0.2, 0.0), (0.5, 0.0), (1.0, 0.0), (2.0, 0.0), (0.3, 1.5), (0.1, 1.0)]:
        args = (np.zeros(N), np.full(N, v0), np.zeros(N), np.full(N, e0), 600, H, VMAX, AMAX, JMAX, CYCLE)
        Q, V, A = track_target(*args)
        new, old = (Q - e0).max(axis=0), (_track_old(*args) - e0).max(axis=0)
        assert np.all(new <= old + 1e-9), (e0, v0, np.degrees(new), np.degrees(old))
        if e0 > 0.05:
            assert np.all(new <= 0.6 * old + 1e-9), (e0, v0, np.degrees(new), np.degrees(old))
        assert np.max(np.abs(Q[-1] - e0)) < 1e-4


def test_stop_speed_matches_jerk_limited_stop():
    """stop_speed(d) 로 출발해 저크 한계로 감속하면 정확히 d 에서 멈춤 (삼각형·사다리꼴 두 경우, 경계 d0 = a³/j² 에서 이어짐)."""
    a, j = 7.78, 500.0
    d0 = a ** 3 / j ** 2
    for d in [1e-5, 0.5 * d0, d0, 2 * d0, 0.05, 0.5, 2.0]:
        v = float(stop_speed(np.array([d]), np.array([a]), np.array([j]))[0])
        _, _, _, t_stop = brake(np.zeros(1), np.array([v]), np.zeros(1), np.arange(1, 10001) * 1e-4, np.array([a]), np.array([j]), h=1e-4)
        Q, _, _, _ = brake(np.zeros(1), np.array([v]), np.zeros(1), [t_stop + 1e-3], np.array([a]), np.array([j]), h=1e-4)
        assert abs(Q[0, 0] - d) <= 0.02 * d + 2e-6, (d, Q[0, 0])
    lo = stop_speed(np.array([d0 * (1 - 1e-9)]), a, j)
    hi = stop_speed(np.array([d0 * (1 + 1e-9)]), a, j)
    assert abs(float(lo[0]) - float(hi[0])) < 1e-6 and float(stop_speed(np.array([0.0]), a, j)[0]) == 0.0


def test_track_target_respects_position_limits():
    """당근이 위치 한계 밖이어도 명령은 한계를 넘지 않음. 출발 상태 = 한계 쪽으로 가되 벽 조건 (한계까지 남은 거리에서 멈출 수 있는
    속도 이하, rt_motion.WALL_DECEL) 을 지키는 상태 — 운전 중 명령은 늘 track_target 이 만들어 이 조건을 지킴."""
    from rt_motion import WALL_DECEL, WALL_INSET
    lim = (np.radians(np.full(N, -154.9)), np.radians(np.full(N, 154.9)))
    rng = np.random.default_rng(3)
    for _ in range(40):
        q0 = rng.uniform(np.radians(60), np.radians(154), N) * rng.choice([-1, 1], N)
        room = np.maximum(lim[1] - WALL_INSET - np.abs(q0), 0.0)
        v0 = np.sign(q0) * np.minimum(rng.uniform(0, 1, N) * stop_speed(room, WALL_DECEL * AMAX, JMAX), VMAX)
        tgt = np.sign(q0) * np.radians(rng.uniform(150, 250, N))            # 한계 근처·밖 당근
        Q, V, A = track_target(q0, v0, np.zeros(N), tgt, 400, H, VMAX, AMAX, JMAX, CYCLE, qlim=lim)
        assert np.all(np.abs(Q) <= lim[1] + 1e-9), np.degrees(np.abs(Q).max(axis=0) - lim[1])
        _within_limits(Q, V, A, np.zeros(N), H)
    Q, _, _ = track_target(np.radians(np.full(N, 160.0)), np.zeros(N), np.zeros(N), np.radians(np.full(N, 170.0)), 300, H, VMAX,
                           AMAX, JMAX, CYCLE, qlim=lim)
    assert np.all(Q[-1] <= lim[1])                                           # 이미 밖이면 안으로 돌아옴


def _track_np(q0, v0, a0, target, n, h, vmax, amax, jmax, cycle, sub=5, stop_decel=1.0, qlim=None, fine=False):
    """2026-10-15 전 track_target (numpy 배열 계산) 그대로 — 관절별 실수 판이 비트까지 같은지 비교 기준."""
    from rt_motion import WALL_DECEL, WALL_INSET, stop_dist
    q, v, a = np.array(q0, float), np.array(v0, float), np.array(a0, float)
    tgt = np.asarray(target, float)
    if qlim is not None:
        plo, phi = np.asarray(qlim[0], float) + WALL_INSET, np.asarray(qlim[1], float) - WALL_INSET
        tgt = np.clip(tgt, plo, phi)
        reach = np.abs(v) * n * h + 0.5 * amax * (n * h) ** 2 + stop_dist(np.minimum(np.abs(v) + amax * n * h, vmax), WALL_DECEL * amax, jmax)
        if np.all((q - reach > plo) & (q + reach < phi)):
            qlim = None
    hh = h / sub
    Q, V, A = np.zeros((n, q.size)), np.zeros((n, q.size)), np.zeros((n, q.size))
    F = np.zeros((n * sub, 3, q.size)) if fine else None
    for k in range(n):
        for s in range(sub):
            e = tgt - q
            v_stop = stop_speed(np.abs(e), stop_decel * amax, jmax)
            v_des = np.clip(np.clip(e / cycle, -v_stop, v_stop), -vmax, vmax)
            if qlim is not None:
                v_des = np.clip(v_des, -stop_speed(np.maximum(q - plo, 0.0), WALL_DECEL * amax, jmax),
                                stop_speed(np.maximum(phi - q, 0.0), WALL_DECEL * amax, jmax))
            a_des = np.clip((v_des - v) / max(cycle / 4.0, hh), -amax, amax)
            lo, hi = acc_bounds(v, vmax, amax, jmax)
            an = np.clip(np.clip(a_des, lo, hi), a - jmax * hh, a + jmax * hh)
            q = q + v * hh + 0.5 * an * hh * hh
            v = np.clip(v + an * hh, -vmax, vmax)
            a = an
            if fine:
                F[k * sub + s] = (q, v, a)
        Q[k], V[k], A[k] = q, v, a
    return (Q, V, A, F) if fine else (Q, V, A)


def _brake_fine_np(q, v, a, amax, jmax, h=0.001, t_max=5.0):
    """2026-10-15 전 brake_fine (numpy 배열 계산) 그대로."""
    q, v, a = np.array(q, float), np.array(v, float), np.array(a, float)
    out = []
    for k in range(1, int(round(t_max / h)) + 1):
        release = (a * v < 0) & (np.abs(v) <= a * a / (2.0 * jmax) + jmax * h * h)
        at = np.where(release, 0.0, -np.sign(v) * amax)
        an = np.clip(at, a - jmax * h, a + jmax * h)
        q = q + v * h + 0.5 * an * h * h
        v = v + an * h
        a = an
        done = (np.abs(v) < 2.0 * jmax * h * h) & (np.abs(a) <= 2.0 * jmax * h)
        v, a = np.where(done, 0.0, v), np.where(done, 0.0, a)
        out.append((q.copy(), v.copy(), a.copy()))
        if np.all(done):
            return np.array(out), k * h
    raise RuntimeError


def test_fast_motion_bit_identical():
    """[2026-10-15] 관절별 실수 판 track_target · brake_fine = 예전 numpy 판 (비트까지, ±0 만 같게 봄). 한계는 cuRobo 처럼 float32 와
    float64 둘 다 (numpy 1.26 형 규칙이 다름), 벽 근처·정지 상태·목표 = 지금 자리·마지막 접근 감속 (0.8) 포함."""
    from rt_motion import brake_fine
    rng = np.random.default_rng(7)
    lim = (np.radians([-360, -180, -154.9, -360, -360, -360.0]), np.radians([360, 180, 154.9, 360, 360, 360.0]))
    for dt in (np.float32, np.float64):
        vmax, amax = VMAX.astype(dt), AMAX.astype(dt)
        for it in range(150):
            q0, v0, a0 = _state(rng)
            if it % 4 == 0:
                q0[2] = np.radians(rng.uniform(140, 160)) * rng.choice([-1, 1])     # 관절 3 벽 근처 (밖 포함)
            if it % 5 == 0:
                v0[:3], a0[:3] = 0.0, 0.0
            tgt = q0 + rng.normal(0, 0.4, N) * rng.choice([0.0, 0.01, 1.0])
            kw = dict(sub=5, stop_decel=0.8 if it % 3 == 0 else 1.0, qlim=lim if it % 2 == 0 else None, fine=True)
            n = 20 if it % 10 else 300
            new = track_target(q0, v0, a0, tgt, n, H, vmax, amax, JMAX, CYCLE, **kw)
            old = _track_np(q0, v0, a0, tgt, n, H, vmax, amax, JMAX, CYCLE, **kw)
            for x, y in zip(new, old):
                assert x.shape == y.shape and np.array_equal(x, y), (dt, it, float(np.max(np.abs(x - y))))
            for s in (tuple(new[3][-1]), (q0, v0, a0), (q0, np.zeros(N), np.zeros(N))):
                (Fn, tn), (Fo, to) = brake_fine(*s, amax, JMAX), _brake_fine_np(*s, amax, JMAX)
                assert Fn.shape == Fo.shape and np.array_equal(Fn, Fo) and tn == to, (dt, it)


def test_clamp_final():
    q, goal = np.array([0.0, 0.0, 1.0, 1.0, 0.5]), np.array([1.0, -1.0, 0.0, 1.0, 0.5 + 5e-5])
    target = np.array([1.2, -0.5, -0.3, 1.3, 0.6])
    out, changed = clamp_final(target, q, goal)
    # 목표를 넘는 쪽 → 목표, 목표 앞은 그대로, 이미 목표에 있는 관절 (|목표 − q| < 1e-4) → 목표
    assert np.allclose(out, [1.0, -0.5, 0.0, 1.0, 0.5 + 5e-5]) and changed
    out, changed = clamp_final(np.array([0.5, -0.5]), np.zeros(2), np.array([1.0, -1.0]))
    assert np.allclose(out, [0.5, -0.5]) and not changed      # 목표 쪽으로 가는 중이면 그대로


def test_brake_stops_within_limits():
    rng = np.random.default_rng(1)
    for _ in range(20):
        q0, v0, a0 = _state(rng)
        ts = np.arange(1, 3001) * 0.001
        Q, V, A, t_stop = brake(q0, v0, a0, ts, AMAX, JMAX)
        assert t_stop is not None
        _within_limits(Q, V, A, a0, 0.001, snap=True)
        k = int(round(t_stop / 0.001))
        assert np.all(V[k - 1:] == 0) and np.all(A[k - 1:] == 0)
        assert np.all(Q[k - 1:] == Q[-1])


def test_braking_tail_ends_at_rest():
    q, v, a = np.zeros(N), np.array([2.0, -1.0, 0.5, 0.0, 3.0, -3.0]), np.array([1.0, 0.0, -2.0, 0.0, 0.0, 5.0])
    tail = braking_tail(0.1, q, v, a, AMAX, JMAX, dt=0.01)
    t = np.array([p[0] for p in tail])
    assert np.all(np.diff(t) > 0) and abs(t[0] - 0.11) < 1e-9
    assert np.all(tail[-1][2] == 0) and np.all(tail[-1][3] == 0)
    # 이미 서 있으면 꼬리 = 점 하나 (dt 뒤, 같은 자리)
    rest = braking_tail(0.1, q, np.zeros(N), np.zeros(N), AMAX, JMAX, dt=0.01)
    assert len(rest) == 1 and np.all(rest[0][1] == q)


def _u(q0, n_ms=100, vel=0.1):
    """1 ms 칸 후보 u [n_ms + 1, 3, J] (rt_stream.shape 의 입력): 등속 vel 로 n_ms, 마지막 칸 = 멈춤."""
    t = np.arange(n_ms + 1) * 1e-3
    q = q0 + vel * t[:, None]
    u = np.stack([q, np.full_like(q, vel), np.zeros_like(q)], axis=1)
    u[-1, 1] = 0.0
    return u


def _send(st, u):
    y = st.shape(u)
    pts = st.points(y, 5, 100, 10)
    st.sent(pts, tuple(u[st.n_dt]), u, y)
    return pts


def test_stream_message_prepends_previous_points():
    c, dt = StreamCfg(), CYCLE
    st = Stream(c, dt, np.zeros(N))
    S0 = st.next_stamp(10.0)
    assert abs(S0 - (10.0 + c.lead_s)) < 1e-12
    u = _u(np.zeros(N))
    msg, t0, n_pre = st.message(st.points(st.shape(u), 5, 100, 10))
    assert n_pre == 0 and t0 == S0 and msg[0][0] == 0.0           # 처음 메시지: 앞 점 없음
    _send(st, u)
    S1 = st.next_stamp(0.0)
    assert abs(S1 - (S0 + dt)) < 1e-12
    u2 = _u(st.cmd[0])
    msg, t0, n_pre = st.message(st.points(st.shape(u2), 5, 100, 10))
    assert n_pre == int(round(dt / H))                             # 앞 메시지의 S1 − pre_s ≤ t < S1 점들 (지난 메시지가 S0 부터라 dt 만큼)
    assert abs(t0 - S0) < 1e-9 and msg[0][0] == 0.0
    times = np.array([m[0] for m in msg])
    assert np.all(np.diff(times) > 0)


def test_stream_kept_and_state_after_end():
    st = Stream(StreamCfg(), CYCLE, np.zeros(N))
    st.next_stamp(0.0)
    u = _u(np.zeros(N), n_ms=120)
    _send(st, u)
    st.next_stamp(0.0)
    q, v, a = st.kept()                                           # 다음 스탬프 시각 = 보낸 메시지의 2 주기 뒤 칸
    assert np.allclose(q, u[100, 0])
    q_end, v_end, _ = st.u_state(st.k + 10000)                    # 메시지 끝 뒤 = 마지막 칸, 멈춤
    assert np.allclose(q_end, u[-1, 0]) and np.all(v_end == 0)


def test_stream_two_keeps_and_restart():
    st = Stream(StreamCfg(), CYCLE, np.zeros(N))
    st.next_stamp(0.0)
    u = _u(np.zeros(N), n_ms=150)
    _send(st, u)
    st.next_stamp(0.0)
    st.kept()                                                     # 다음 스탬프 = 보낸 메시지의 0.10 s 칸
    st.next_stamp(0.0)
    q, v, _ = st.kept()                                           # 그다음 = 0.15 s 칸 (메시지 마지막 = 멈춤)
    assert np.allclose(q, u[150, 0]) and np.all(v == 0)
    st.restart(np.ones(N))
    assert st.stamp is None and np.all(st.cmd[0] == 1) and np.all(st.cmd[1] == 0)
    st.next_stamp(5.0)
    assert st.pts == [] and st.remaining() == []                  # 새 시간표: 앞 메시지 없음
    seg = st.executed()
    assert len(seg) == 1 and len(seg[0]['q']) == len(u)           # 다시 시작 전 메시지는 끝까지 실행된 것으로 기록


def test_stream_catch_up_moves_to_grid():
    c = StreamCfg()
    st = Stream(c, CYCLE, np.zeros(N))
    S0 = st.next_stamp(0.0)
    u = _u(np.zeros(N), n_ms=200)
    _send(st, u)
    st.next_stamp(0.0)
    now = st.stamp - 0.005                                        # 스탬프까지 5 ms (< late_before_s) → 늦음
    late = st.catch_up(lambda: now)
    assert late > 0 and st.stamp - now >= c.lead_s
    k = int(round((st.stamp - S0) / 1e-3))
    assert k == st.k and np.allclose(st.cmd[0], u[k, 0])          # 미룬 스탬프의 앞 메시지 상태에서 다시
    assert st.catch_up(lambda: st.stamp - 1.0) == 0.0              # 넉넉하면 그대로
