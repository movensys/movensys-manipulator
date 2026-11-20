"""[2026-10-28 R-1] RosRobot 받기 쪽 (ROS 없이 — 노드를 안 만들고 받기 함수에 가짜 메시지): wait_js 는 받는 순간 깨어나고 못 받으면 시간 초과,
_on_jtc 는 줄과 monotonic 시각 (t_jtc_mono) 을 남기고, trim_jtc 는 마지막 keep 줄만 남김.
   python3 -m pytest   (cr3a_rt_shaping 에서 — pytest.ini)
"""
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from rt_robot import JOINTS, RosRobot


def _robot():
    """__init__ 의 받기 상태만 (rclpy 노드 없이)."""
    r = RosRobot.__new__(RosRobot)
    r.lock, r.q, r.jtc, r.t_jtc_mono, r._js_ready = threading.Lock(), None, [], None, threading.Event()
    return r


def _js(q):
    return NS(name=list(reversed(JOINTS)), position=list(reversed(q)))         # 이름 차례가 달라도 JOINTS 차례로


def _jcs(k):
    ref = NS(positions=[k + 0.1 * i for i in range(6)], velocities=[0.0] * 6)
    return NS(reference=ref, joint_names=JOINTS, header=NS(stamp=NS(sec=100 + k, nanosec=0)))


def test_wait_js_timeout():
    r = _robot()
    t0 = time.monotonic()
    with pytest.raises(RuntimeError):
        r.wait_js(0.05)
    assert time.monotonic() - t0 < 1.0


def test_wait_js_wakes_on_message():
    r = _robot()
    q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    th = threading.Timer(0.05, r._on_js, args=(_js(q),))
    th.start()
    t0 = time.monotonic()
    got = r.wait_js(5.0)
    assert time.monotonic() - t0 < 1.0
    assert np.array_equal(got, np.array(q))
    got[0] = 9.0                                    # 사본
    assert r.measured()[0] == 0.1


def test_on_jtc_monotonic_and_trim():
    r = _robot()
    t0 = time.monotonic()
    for k in range(10):
        r._on_jtc(_jcs(k))
    assert len(r.jtc) == 10 and r.jtc[-1][0] == 109.0 and r.jtc[-1][2][1] == 9.1
    assert t0 <= r.t_jtc_mono <= time.monotonic()
    assert r.trim_jtc(20) == 0 and len(r.jtc) == 10
    assert r.trim_jtc(3) == 7
    assert [row[0] for row in r.jtc] == [107.0, 108.0, 109.0]
