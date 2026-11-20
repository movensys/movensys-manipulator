"""[2026-10-27 R-1 ⑬] rt_server 회전 바꾸기 (scipy Rotation) 를 전 손으로 쓴 식과 비교. cuRobo 쿼터니언 = wxyz (scipy 기본 xyzw).
  - 보통 회전 (180° 에서 1° 넘게 떨어짐): 전 식과 같은 값 (쿼터니언 w ≥ 0 까지 같음)
  - 180° 가까이: 새 식은 되돌리면 같은 회전 (R → q → R). 전 R_quat 은 부호를 비대각 차이로 정해서 그 차이가 정확히 0 (비대각이 정확히
    대칭인 180°) 이면 틀림. cm.rpy_R 공구 아래 목표는 부동소수 잡음 부호가 맞아 전 식도 맞았음 (출력으로 셈)
"""
import sys  # noqa: E402
from pathlib import Path  # noqa: E402
sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parents[2] / 'cr3a_rt_shaping')]   # rt_server · 엔진
import math

import numpy as np
import pytest

import cr3a_model as cm
import rt_server as rs


# ---- 전 식 (2026-10-27 R-1 전 rt_server 그대로 — 비교 기준) ----
def old_R_quat(R):
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def old_quat_R(q):
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def old_rotvec(R):
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    a = math.acos(float(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)))
    return 0.5 * w if a < 1e-9 else w * (a / (2 * math.sin(a)))


def rot(axis, ang):
    """축·각 → 3x3 (로드리게스 — 시험 쪽에서 따로)."""
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def random_axes(n, rng):
    v = rng.normal(size=(n, 3))
    return v / np.linalg.norm(v, axis=1, keepdims=True)


@pytest.fixture
def rng():
    return np.random.default_rng(20261027)


def test_generic_rotations_match_old(rng):
    """보통 회전 2000 개: 쿼터니언·행렬·회전 벡터 모두 전 식과 같음."""
    for k, ang in zip(random_axes(2000, rng), rng.uniform(1e-6, math.radians(179.0), 2000)):
        R = rot(k, ang)
        q_new, q_old = rs.R_quat(R), old_R_quat(R)
        assert q_new[0] >= 0.0
        assert np.allclose(q_new, q_old, atol=1e-9), (k, math.degrees(ang), q_new, q_old)
        assert np.allclose(rs.quat_R(q_new), old_quat_R(q_new), atol=1e-12)
        assert np.allclose(rs.quat_R(q_new), R, atol=1e-9)
        assert np.allclose(rs.rotvec(R), old_rotvec(R), atol=1e-7), (k, math.degrees(ang))
        assert np.allclose(rs.rotvec(R), k * ang, atol=1e-9)


def test_near_180_roundtrip(rng):
    """180° 가까이 (179.9°~180°) 2000 개: 새 식은 되돌리면 같은 회전, 회전 벡터 각도 맞음. 전 R_quat 이 틀리는 수도 셈 (정보)."""
    old_bad = 0
    for k, ang in zip(random_axes(2000, rng), rng.uniform(math.radians(179.9), math.pi, 2000)):
        R = rot(k, ang)
        assert np.allclose(rs.quat_R(rs.R_quat(R)), R, atol=1e-9), (k, math.degrees(ang))
        assert abs(np.linalg.norm(rs.rotvec(R)) - ang) < 1e-7
        old_bad += not np.allclose(old_quat_R(old_R_quat(R)), R, atol=1e-6)
    print(f'180° 가까이 2000 개 중 전 R_quat 이 틀린 회전: {old_bad}')


def test_tool_down_targets():
    """공구 끝 목표 rpy (180, ry, rz) — GUI 와 같은 cm.rpy_R (공구가 아래를 봄 = 180° 회전들). 새 식은 늘 맞음.
    전 식은 비대각 차이의 부동소수 잡음 부호에 따라 맞기도 틀리기도 함 (몇 개인지 출력)."""
    old_bad = []
    for ry in (-10.0, 0.0, 10.0):
        for rz in np.arange(-180.0, 180.0, 15.0):
            R = cm.rpy_R(math.pi, math.radians(ry), math.radians(rz))
            assert np.allclose(rs.quat_R(rs.R_quat(R)), R, atol=1e-9), (ry, rz)
            err = math.degrees(np.linalg.norm(rs.rotvec(old_quat_R(old_R_quat(R)).T @ R)))
            if err > 1e-3:
                old_bad.append((ry, float(rz), round(err, 1)))
    print(f'rpy (180, ry, rz) {3 * 24} 개 중 전 R_quat 이 틀린 것: {len(old_bad)} {old_bad[:6]}')


def test_old_R_quat_breaks_exact_180():
    """(회귀 방지) 비대각이 정확히 대칭인 180° 회전 (축 (1, −1, 0)) — 전 식은 축 (1, 1, 0) 으로 바꿔 90° 틀림. 새 식은 맞음."""
    R = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    assert math.degrees(np.linalg.norm(rs.rotvec(old_quat_R(old_R_quat(R)).T @ R))) > 1.0
    assert np.allclose(rs.quat_R(rs.R_quat(R)), R, atol=1e-12)


def test_rot_angle():
    for ang in (0.0, 1e-4, 0.3, 2.0, math.pi - 1e-6):
        assert abs(rs.rot_angle(np.eye(3), rot([1, 2, 3], ang)) - ang) < 1e-7
