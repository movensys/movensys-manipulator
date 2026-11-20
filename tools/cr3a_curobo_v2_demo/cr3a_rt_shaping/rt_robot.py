"""로봇과 시계.
  이상적 (IdealRobot + SimClock): 시험용. ROS 없음, 로봇 = 보낸 명령 그대로, 시계 = 계산 시계 → 같은 입력이면 같은 결과
  ROS    (RosRobot + WallClock):  R2 (tools/rt_demo.sh → cr3a_gui/wmx_r2.sh 가 띄움) 의 /joint_states (WMX 실제 위치) 를 읽고 JTC 에 JointTrajectory 를 보냄
"""
import array
import threading
import time

import numpy as np

JOINTS = [f'joint{i}' for i in range(1, 7)]
CONTROLLER = '/movensys_manipulator_arm_controller'


def _f64(x) -> array.array:
    """관절값 → array('d') (float64 값 그대로 — float32 도 정확히 늘림)."""
    return array.array('d', np.ascontiguousarray(x, dtype=np.float64).tobytes())


class SimClock:
    """계산 시계: 기다림 = 시각을 그만큼 옮김."""
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep_until(self, t):
        self.t = max(self.t, t)

    def advance(self, s):
        self.t += s


class WallClock:
    def now(self):
        return time.time()

    def sleep_until(self, t):
        w = t - time.time()
        if w > 0:
            time.sleep(w)


class IdealRobot:
    """보낸 메시지의 다음 주기 명령 상태에 바로 있음 (측정 = 명령)."""
    def __init__(self, q):
        self.q = np.array(q, float)

    def measured(self):
        return self.q.copy()

    def send(self, t0, points, q_next):
        self.q = np.array(q_next, float)

    def keep(self, q_next):
        """안 보냄 = 앞 메시지를 계속 따름 → 그 메시지의 다음 주기 상태."""
        self.q = np.array(q_next, float)

    def rest(self, q):
        """앞 메시지 꼬리 끝에 멈춤 (전역 계획하는 동안)."""
        self.q = np.array(q, float)

    def close(self):
        pass


class RosRobot:
    """/joint_states (R2 joint_state_broadcaster = WMX 실제 위치) 를 받고, JointTrajectory 를 JTC 로 보냄.
    JTC 가 실제로 낸 기준 (controller_state 의 위치·속도) 도 기록 (jtc) — 명령 → JTC → WMX 세 단계 가속 비교용 (rt_report).
    start = 만들자마자 받기 스레드 (rclpy.spin) 시작 — [2026-10-27 R-1] False 면 구독·발행을 더 만든 뒤 start() (기본 True = 예전과 같음)."""
    def __init__(self, start=True):
        import rclpy
        from control_msgs.msg import JointTrajectoryControllerState as JCS
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState as JS
        self._load_msg_types()
        rclpy.init()
        self.rclpy = rclpy
        self.node = Node('cr3a_rt_avoid')
        self.lock = threading.Lock()
        self.q = None
        self.jtc = []
        self.t_jtc_mono = None                  # [R-1] 마지막 jtc 줄을 받은 monotonic 시각 (줄의 시각은 벽시계 — 나이 재기용)
        self._js_ready = threading.Event()      # [R-1] 첫 /joint_states 를 받음 (wait_js)
        self.node.create_subscription(JS, '/joint_states', self._on_js, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.node.create_subscription(JCS, f'{CONTROLLER}/controller_state', self._on_jtc, 100)
        self.pub = self.node.create_publisher(self.JT, f'{CONTROLLER}/joint_trajectory', 10)
        self.th = None
        if start:
            self.start()

    def start(self):
        """받기 스레드 (rclpy.spin) 시작 — 구독·발행을 다 만든 뒤에. 두 번 불러도 한 번만."""
        if self.th is None:
            self.th = threading.Thread(target=self.rclpy.spin, args=(self.node,), daemon=True)
            self.th.start()

    def _on_js(self, m):
        try:
            q = [m.position[list(m.name).index(n)] for n in JOINTS]
        except ValueError:
            return
        with self.lock:
            self.q = np.array(q, float)
        self._js_ready.set()

    def _on_jtc(self, m):
        ref = getattr(m, 'reference', None) or getattr(m, 'desired', None)
        if ref is None or len(ref.positions) != 6:
            return
        idx = [list(m.joint_names).index(n) for n in JOINTS] if list(m.joint_names) != JOINTS else list(range(6))
        with self.lock:
            self.jtc.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, time.time(),
                             [round(ref.positions[i], 7) for i in idx],
                             [round(ref.velocities[i], 6) for i in idx] if len(ref.velocities) == 6 else None))
            self.t_jtc_mono = time.monotonic()

    def trim_jtc(self, keep: int) -> int:
        """[2026-10-28 R-1] JTC 기준 기록 (jtc — controller_state 마다 한 줄, 100~1000 Hz) 을 마지막 keep 줄만 남김 (오래 켜 둔 서버).
        반환: 버린 줄 수."""
        with self.lock:
            n = max(len(self.jtc) - int(keep), 0)
            del self.jtc[:n]
        return n

    def wait_js(self, timeout=10.0):
        """첫 /joint_states 를 기다려 관절값 사본. [2026-10-28 R-1] 받는 순간 깨어남 (threading.Event — 예전: 20 ms 마다 확인)."""
        if not self._js_ready.wait(timeout):
            raise RuntimeError('/joint_states 를 못 받음 (R2 가 떠 있나?)')
        return self.measured()

    def measured(self):
        with self.lock:
            return self.q.copy()

    def _load_msg_types(self):
        """메시지 형 (JointTrajectory·점·시각) 을 한 번 불러 둠 (예전: 보낼 때마다 import). 노드 없이 메시지만 만들 때도 (tests)."""
        from builtin_interfaces.msg import Duration, Time
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        self.JT, self._JTP, self._Dur, self._Time = JointTrajectory, JointTrajectoryPoint, Duration, Time

    def build_msg(self, t0, points):
        """JointTrajectory 메시지 (t0 = 메시지 시각 유닉스 s, points = [(t0 기준 시각, q, v, a)]). [2026-10-27 R-1] 관절값은 array('d')
        (float64 바이트 그대로) 로 넣음 — rosidl 의 빠른 길 (float 목록이면 값마다 검사·변환), 값은 같음 (tests/test_ros_msg.py)."""
        msg = self.JT()
        msg.header.stamp = self._Time(sec=int(t0), nanosec=int(round((t0 - int(t0)) * 1e9)) % 1000000000)
        msg.joint_names = JOINTS
        pts = []
        for tt, q, v, a in points:
            pt = self._JTP()
            pt.positions, pt.velocities, pt.accelerations = _f64(q), _f64(v), _f64(a)
            pt.time_from_start = self._Dur(sec=int(tt), nanosec=int(round((tt - int(tt)) * 1e9)))
            pts.append(pt)
        msg.points = pts
        return msg

    def send(self, t0, points, q_next):
        """t0 = 메시지 시각 (유닉스 s), points = [(t0 기준 시각, q, v, a)]."""
        self.pub.publish(self.build_msg(t0, points))

    def keep(self, q_next):
        pass                    # JTC 가 앞 메시지를 계속 실행

    def rest(self, q):
        pass

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()
