"""[2026-10-27 R-1] RosRobot.build_msg (관절값 array('d') — rosidl 빠른 길) = 예전 send 의 메시지 (float 목록) — 필드가 모두 같은지.
ROS 메시지 (trajectory_msgs·builtin_interfaces) 를 불러올 수 있을 때만 (컨테이너, ROS 설정 뒤 — 없으면 건너뜀). 노드는 안 만듦.
   python3 -m pytest   (cr3a_rt_shaping 에서 — pytest.ini)
"""
import numpy as np
import pytest

from rt_robot import JOINTS, RosRobot


def _old_msg(t0, points):
    """2026-10-27 전 RosRobot.send 가 만들던 메시지 (비교 기준)."""
    from builtin_interfaces.msg import Duration, Time
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    msg = JointTrajectory()
    msg.header.stamp = Time(sec=int(t0), nanosec=int(round((t0 - int(t0)) * 1e9)) % 1000000000)
    msg.joint_names = JOINTS
    for tt, q, v, a in points:
        pt = JointTrajectoryPoint()
        pt.positions, pt.velocities, pt.accelerations = [float(x) for x in q], [float(x) for x in v], [float(x) for x in a]
        pt.time_from_start = Duration(sec=int(tt), nanosec=int(round((tt - int(tt)) * 1e9)))
        msg.points.append(pt)
    return msg


def test_build_msg_equals_old():
    pytest.importorskip('trajectory_msgs.msg')
    pytest.importorskip('builtin_interfaces.msg')
    r = RosRobot.__new__(RosRobot)                          # 노드·rclpy.init 없이 메시지 형만
    r._load_msg_types()
    rng = np.random.default_rng(21)
    for trial in range(50):
        t0 = 1.8e9 + float(rng.uniform(0, 1e5)) if trial % 2 else float(rng.uniform(0, 3))
        n = int(rng.integers(1, 40))
        ts = np.round(np.cumsum(rng.uniform(0.001, 0.02, n)), 6)
        dt = np.float32 if trial % 3 == 0 else np.float64   # float32 관절값도 (정확히 늘어나야 함)
        points = [(float(t), rng.normal(size=6).astype(dt), rng.normal(size=6).astype(dt), rng.normal(size=6).astype(dt)) for t in ts]
        new, old = r.build_msg(t0, points), _old_msg(t0, points)
        assert (new.header.stamp.sec, new.header.stamp.nanosec) == (old.header.stamp.sec, old.header.stamp.nanosec)
        assert list(new.joint_names) == list(old.joint_names) == JOINTS
        assert len(new.points) == len(old.points) == n
        for a, b in zip(new.points, old.points):
            assert list(a.positions) == list(b.positions) and list(a.velocities) == list(b.velocities)
            assert list(a.accelerations) == list(b.accelerations)
            assert (a.time_from_start.sec, a.time_from_start.nanosec) == (b.time_from_start.sec, b.time_from_start.nanosec)
        assert new == old                                   # rosidl 메시지 비교 (모든 필드)
