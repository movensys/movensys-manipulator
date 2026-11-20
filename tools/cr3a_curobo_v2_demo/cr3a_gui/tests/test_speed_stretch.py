"""[2026-11-02 디버깅 D-5] 속도 배율 = 속도·가속 한계 × 배율 (rt_server.speed_stretch): 늘린 경로가 한계 × 배율 안이고 걸리는 쪽은 그
한계에 닿는지, 배율 1·이미 느린 경로는 그대로인지, 옛 방식 (1/배율) 보다 느려지지 않는지, 늘린 점의 5 차 스플라인 (실시간 회피 끔 명령)
도 같은 비율로 줄고 저크는 늘지 않는지."""
import sys  # noqa: E402
from pathlib import Path  # noqa: E402
sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parents[2] / 'cr3a_rt_shaping')]   # rt_server · 엔진

import numpy as np

import rt_server as rs

VMAX = np.array([2.983, 2.983, 3.695, 3.695, 3.695, 3.695])     # 배율 1 한계 (로봇 한계의 95 % — Thor 서버 로그 값)
AMAX = np.array([5.966, 5.966, 7.391, 7.391, 7.391, 7.391])
SS = (0.01, 0.05, 0.1, 0.2, 0.5, 1.0)


def path(T, amp, n=81):
    """멈춤에서 멈춤 (최소 저크 꼴 10τ³ − 15τ⁴ + 6τ⁵, 최고 속도 1.875·amp/T, 최고 가속 5.77·amp/T²) — 관절마다 amp rad."""
    t = np.linspace(0.0, T, n)
    tau = t / T
    return t, np.outer(10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5, amp)


def usage(t, q):
    """경로의 속도·가속 사용률 (점 사이 직선 · 5 차 스플라인 촘촘히 중 큰 것) — 시험 쪽에서 따로 잼."""
    dt = np.diff(t)
    v = np.diff(q, axis=0) / dt[:, None]
    a = np.diff(v, axis=0) / (0.5 * (dt[1:] + dt[:-1]))[:, None]
    spl = rs._path_spline(t, q)
    xs = np.linspace(0.0, t[-1] - t[0], 20001)
    return (max(np.abs(v / VMAX).max(), np.abs(spl(xs, 1) / VMAX).max()),
            max(np.abs(a / AMAX).max(), np.abs(spl(xs, 2) / AMAX).max()))


LONG = path(1.6, [1.5, 0.4, -0.6, 0.2, 0.3, -1.0])      # 긴 이동 — 속도 사용률 > 가속 (낮은 배율에서 속도가 걸림)
SHORT = path(0.55, [0.3, 0.1, -0.2, 0.05, 0.1, -0.25])  # 짧은 이동 — 가속 사용률 ~0.96 (배율 0.5 에서 가속이 걸림)


def test_limits_reached_not_exceeded():
    for t, q in (LONG, SHORT):
        vu0, au0 = usage(t, q)
        assert vu0 <= 1.0 and au0 <= 1.0
        for s in SS:
            k, vu, au = rs.speed_stretch(t, q, s, VMAX, AMAX)
            assert abs(vu - vu0) < 1e-3 * vu0 and abs(au - au0) < 1e-3 * au0       # 사용률 (늘리기 전) 이 따로 잰 것과 같음
            v1, a1 = usage(t * k, q)
            assert v1 <= s * (1 + 1e-4) and a1 <= s * (1 + 1e-4), (s, k, v1, a1)     # 한계 × 배율 안
            if k > 1.0:
                assert max(v1, a1) >= s * (1 - 1e-3), (s, k, v1, a1)               # 걸리는 쪽은 그 한계에 닿음 (더 느리지 않음)
            assert k <= 1.0 / s + 1e-9                                              # 옛 방식 (시각 × 1/배율) 보다 느리지 않음


def test_which_limit_binds():
    t, q = LONG
    k, vu, au = rs.speed_stretch(t, q, 0.1, VMAX, AMAX)
    assert abs(k - vu / 0.1) < 1e-9                         # 긴 이동 · 낮은 배율 = 속도
    t, q = SHORT
    k, vu, au = rs.speed_stretch(t, q, 0.5, VMAX, AMAX)
    assert abs(k - (au / 0.5) ** 0.5) < 1e-9                # 짧은 이동 · 배율 0.5 = 가속


def test_scale_one_and_slow_path_unchanged():
    for t, q in (LONG, SHORT):
        assert rs.speed_stretch(t, q, 1.0, VMAX, AMAX)[0] == 1.0          # 배율 1 = cuRobo 경로 그대로
    t, q = path(8.0, [0.3, 0.1, -0.2, 0.05, 0.1, -0.25])                     # 이미 느린 경로 (사용률 ~0.03)
    assert rs.speed_stretch(t, q, 0.5, VMAX, AMAX)[0] == 1.0                # 빠르게 하지는 않음


def test_spline_scales_and_jerk_not_up():
    t, q = LONG
    k = rs.speed_stretch(t, q, 0.1, VMAX, AMAX)[0]
    xs = np.linspace(0.0, t[-1], 501)
    s0, s1 = rs._path_spline(t, q), rs._path_spline(t * k, q)
    for d in (0, 1, 2, 3):                                  # 늘린 점의 스플라인 = 늘린 스플라인 (끝 조건 속도·가속 0)
        assert np.allclose(s1(xs * k, d), s0(xs, d) / k ** d, atol=1e-9)
    assert np.abs(s1(xs * k, 3)).max() <= np.abs(s0(xs, 3)).max()          # 저크 한계는 그대로 — 늘리면 줄기만 함


def test_scale_path_keeps_normal_paths():
    """점 사이가 PATH_GAP_MAX_S 안이면 scale_path = speed_stretch 배수로 시각만 늘림 (점 하나까지 같음)."""
    for t, q in (LONG, SHORT):
        for s in SS[1:]:                                                            # (0.01 은 점 사이가 0.5 s 를 넘어 다시 찍음 — 아래 시험)
            k = rs.speed_stretch(t, q, s, VMAX, AMAX)[0]
            t1, q1, k1, _, _ = rs.scale_path(t, q, s, VMAX, AMAX)
            assert np.diff(t * k).max() <= rs.PATH_GAP_MAX_S                       # (이 경로들은 다 0.5 s 안)
            assert k1 == k and len(t1) == len(t) and np.array_equal(q1, q) and np.allclose(t1, t * k, rtol=0, atol=1e-12)
    t, q = LONG
    t2, q2 = rs.densify_path(t, q)
    assert np.array_equal(t2, t) and np.array_equal(q2, q)


def test_scale_path_densifies_very_long_move():
    """[D-5 조건부 다시 찍기] 배율 0.05 · J1 716° 같은 아주 긴 이동: 늘리면 점 사이 ~2 s → 같은 곡선 위 0.1 s 마다, 끝점 그대로,
    한계 × 배율 안이고 여전히 그 한계에 닿음 (다시 찍어도 느려지지 않음)."""
    t, q = path(8.0, [12.5, 0.0, 0.0, 0.0, 0.0, 0.0])         # 속도 사용률 ~0.98
    s = 0.05
    k = rs.speed_stretch(t, q, s, VMAX, AMAX)[0]
    assert np.diff(t * k).max() > rs.PATH_GAP_MAX_S            # 그냥 늘리면 점 사이가 넓음
    t1, q1, k1, _, _ = rs.scale_path(t, q, s, VMAX, AMAX)
    assert np.diff(t1).max() <= rs.PATH_STEP_S + 1e-9 and len(t1) > 10 * len(t)
    assert np.array_equal(q1[0], q[0]) and np.array_equal(q1[-1], q[-1])
    on = rs._path_spline(t * k, q)(t1)                          # 다시 찍은 점 = 늘린 곡선 위
    assert np.abs(on[1:-1] - q1[1:-1]).max() < 1e-9
    assert abs(k1 / k - 1.0) < 1e-3                            # 다시 잼 배수 ~1
    v1, a1 = usage(t1, q1)
    assert v1 <= s * (1 + 1e-4) and a1 <= s * (1 + 1e-4) and v1 >= s * (1 - 1e-3), (v1, a1)


def test_scale_path_min_speed_001():
    """[2026-11-02 사용자: 배율 최소 0.05 → 0.01] 0.01 에서 (SHORT 는 점 사이 1.2 s → 다시 찍음, LONG 은 0.24 s 라 그대로): 점 사이가 0.5 s 안,
    끝점 그대로, 한계 × 0.01 안."""
    s = rs.SPEED_MIN
    assert s == 0.01
    for t, q in (LONG, SHORT):
        t1, q1, k1, _, _ = rs.scale_path(t, q, s, VMAX, AMAX)
        assert np.diff(t1).max() <= rs.PATH_GAP_MAX_S + 1e-9
        assert np.array_equal(q1[0], q[0]) and np.array_equal(q1[-1], q[-1])
        v1, a1 = usage(t1, q1)
        assert v1 <= s * (1 + 1e-4) and a1 <= s * (1 + 1e-4), (v1, a1)
        assert max(v1, a1) >= s * (1 - 1e-3)                                       # 한계에 닿음 (더 느리지 않음)


def test_degenerate():
    assert rs.speed_stretch([0.0, 0.1], np.zeros((2, 6)), 0.1, VMAX, AMAX) == (1.0, 0.0, 0.0)
    assert rs.speed_stretch([0.0, 0.0, 0.0], np.zeros((3, 6)), 0.1, VMAX, AMAX) == (1.0, 0.0, 0.0)
