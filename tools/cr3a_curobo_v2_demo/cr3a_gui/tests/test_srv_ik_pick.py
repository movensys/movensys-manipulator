"""[2026-10-28] GuiController._ik 의 해 고르기 (가짜 cuRobo 해): 2π 감기 (관절 한계 안일 때만), 많이 돌아야 하는 해 거절
(TIP_MAX_J1_DEG · TIP_MAX_JOINT_DEG), 못 풀 때 까닭이 '좌표 이동 불가' 로 시작. torch·cuRobo 가 있는 곳 (Thor 컨테이너) 에서만."""
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parents[2] / 'cr3a_rt_shaping')]   # 서버 · 엔진
torch = pytest.importorskip('torch')
gc_mod = pytest.importorskip('gui_controller')
if not torch.cuda.is_available():
    pytest.skip('CUDA 없음', allow_module_level=True)
import rt_server as rs  # noqa: E402

R = math.radians
Q0 = np.radians([56.31, -40.67, -12.99, -29.16, 83.72, 55.72])        # A 자세
LIM = (np.radians([-360, -360, -155, -360, -360, -360]), np.radians([360, 360, 155, 360, 360, 360]))
T_GOAL = np.eye(4)


def fake(sols, success=None, pe=None, re=None):
    """sols [n, 6] rad 를 돌려주는 IKSolver · 공구 끝 = 늘 목표 (정기구학 확인 통과) · 관절 한계."""
    sols = np.asarray(sols, float).reshape(-1, 6)
    n = len(sols)
    ok = np.ones(n, bool) if success is None else np.asarray(success, bool)
    res = SimpleNamespace(success=torch.tensor(ok[None]), solution=torch.tensor(sols[None], dtype=torch.float32),
                          position_error=torch.tensor((np.zeros(n) if pe is None else np.asarray(pe, float))[None]),
                          rotation_error=torch.tensor((np.zeros(n) if re is None else np.asarray(re, float))[None]))
    solver = SimpleNamespace(solve_pose=lambda *a, **k: res)
    return SimpleNamespace(W=SimpleNamespace(planner=SimpleNamespace(ik_solver=solver)), mpc=SimpleNamespace(qlim=LIM),
                           _tip=lambda q: T_GOAL)


def ik(f, seed=Q0):
    return gc_mod.GuiController._ik(f, T_GOAL, seed)


def test_wrap_picks_equivalent_near_solution():
    far_raw = Q0 + np.array([0.0, -2 * math.pi + 0.1, 0, 0, 0, 0])     # J2 를 −360° 돌린 값 = 실제로는 +0.1 rad
    near = Q0 + np.array([0.3, 0, 0, 0, 0, 0])                          # J1 17°
    q, why = ik(fake([near, far_raw]))
    assert q is not None, why
    assert np.allclose(q, Q0 + np.array([0, 0.1, 0, 0, 0, 0]), atol=1e-5)  # 감은 값이 더 가까워 고름


def test_wrap_only_within_limits():
    seed = Q0.copy()
    seed[2] = R(150.0)
    sol = seed.copy()
    sol[2] = R(-150.0)                                                  # 감으면 210° = J3 한계 (155°) 밖 → 원래 값 → 300° 차 → 거절
    q, why = ik(fake([sol]), seed)
    assert q is None and why.startswith('좌표 이동 불가') and 'J3 300°' in why


@pytest.mark.parametrize('dq_deg, accept', [((40, 0, 0, 0, 80, 0), True), ((50, 0, 0, 0, 0, 0), False),
                                            ((0, 0, 0, 0, 100, 0), False), ((148, 0, 0, 0, 175, 0), False)])
def test_threshold(dq_deg, accept):
    q, why = ik(fake([Q0 + np.radians(dq_deg)]))
    assert (q is not None) == accept, why
    if not accept:
        assert why.startswith('좌표 이동 불가 — 지금 팔 모양으로는'), why
    assert rs.TIP_MAX_J1_DEG == 45.0 and rs.TIP_MAX_JOINT_DEG == 90.0


def test_failure_messages_start_with_refusal():
    q, why = ik(fake([Q0], success=[False], pe=[0.0039], re=[0.002]))
    assert q is None and why.startswith('좌표 이동 불가 — 닿지 않는 자리'), why
    q, why = ik(fake([Q0], success=[False], pe=[0.00001], re=[0.0001]))
    assert q is None and why.startswith('좌표 이동 불가 — 그 자세는 풀리지만 충돌'), why
