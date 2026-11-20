"""[2026-10-27 R-1 ⑦] 켤 때 한계 확인 함수: URDF 속도 한계 읽기 · LIMIT_FRAC 비율 판정 (cuRobo 가 velocity_scale 을 한 번만 곱하는 판이면
√LIMIT_FRAC 우회가 97.5 % 를 내므로 걸려야 함)."""
import sys  # noqa: E402
from pathlib import Path  # noqa: E402
sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parents[2] / 'cr3a_rt_shaping')]   # rt_server · 엔진
import math

import numpy as np

import rt_server as rs
from rt_model import get as model_get

JOINTS = [f'joint{i}' for i in range(1, 7)]


def test_urdf_velocity_tool_model():
    v = rs.urdf_velocity(model_get('tool').urdf, JOINTS)
    assert np.allclose(v, [3.14, 3.14, 3.89, 3.89, 3.89, 3.89])


def test_limit_frac_errors():
    ref = np.array([3.14, 3.14, 3.89, 3.89, 3.89, 3.89])
    assert rs.limit_frac_errors({'ok': (ref * rs.LIMIT_FRAC, ref)}) == {}
    assert rs.limit_frac_errors({'f32': (np.float32(ref * rs.LIMIT_FRAC), ref)}) == {}          # float32 반올림은 통과
    once = rs.limit_frac_errors({'한 번만 곱함': (ref * math.sqrt(rs.LIMIT_FRAC), ref)})        # 버그가 고쳐진 cuRobo
    assert list(once) == ['한 번만 곱함'] and abs(once['한 번만 곱함'][0] - math.sqrt(rs.LIMIT_FRAC)) < 1e-4
    assert 'x' in rs.limit_frac_errors({'x': (ref, ref)})                                     # 배율 1
