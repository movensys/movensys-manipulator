#!/usr/bin/env python3
"""CR3A GUI 서버의 제어기 — [2026-10-27 R-1 ⑮] rt_server.make_controller 안에 있던 클래스를 옮김 (동작 같음): GuiClock (이상적 로봇 시계),
GuiRosRobot (R2 로봇 + 서보·Servo·끊김 감시), GuiController (rt_run.Controller + GUI 명령·조그·지도 점검·기록), 공구 끝 T_TIP.
GUI 연결 (link) 은 GuiController 인자로, 로그는 엔진 Controller 의 log= (rt_server.make_controller 가 넣음).
무거운 가져오기 (rt_run → cuRobo·torch) 가 이 파일 맨 위라 rt_server 는 GUI 가 붙은 뒤 (Link) make_controller 안에서 이것을 가져옴.
rt_run 을 cuRobo 보다 먼저 (rt_run 이 cuRobo 실행 설정 — torch.compile — 을 먼저 함). 상수·작은 함수·Abort·JogState 는 rt_server 에서
(rt_server 를 스크립트로 띄우면 __main__ 이지만 그 모듈을 'rt_server' 로도 올려 둠 — 두 번째 사본 없음).
"""
import rt_run  # noqa: I001  (cuRobo 보다 먼저 — 위)

import collections
import gc
import math
import queue
import threading
import time
from pathlib import Path

import numpy as np
import torch
from curobo.scene import Scene, VoxelGrid
from curobo.types import GoalToolPose, JointState, Pose

import camera_io as cio
import cr3a_mapper
import cr3a_model as cm
import shaper_def
from rt_motion import brake_fine, track_target
from rt_robot import CONTROLLER, RosRobot, WallClock
from rt_stream import Stream
from rt_world import FAR_M, JOINTS, World
from rt_server import (  # noqa: E402
    ARM, CAM_STALE_S, CART_AXES, CHECK_MIN_FRAMES, CHECK_S, CHECK_SKIP_S, COMM_STOP_S, COMM_WARN_S, FRAME_BUDGET_S, GC_IDLE_S, GUI_LOST_S,
    HW_NAME, IDLE_S, IDLE_TAIL_PAD_S, IK_SEEDS, IK_TOL_M, IK_TOL_RAD, JOG_TIMEOUT_S, JSB, JS_STALE_S, JTC_REF_AGE_S, LOGGER, MOVES,
    PLAN_END_SNAP_RAD, PUB_S, REBASE_WAIT_S, REC_MAX_S, REFRESH_REAL_S, REFRESH_S, REPEAT_PAUSE_S, SAY_ONCE_S, SERVO_BASE, SERVO_COAST_S,
    SERVO_CODES, SERVO_EXIT_MAX_S, SPECK_DIFF_MM, SPECK_PX, SERVO_HALT_S, SERVO_LIMIT_MARGIN, SERVO_NODE, SERVO_QUIET_S, SERVO_START_PAD_S, SERVO_STILL_RAD,
    SERVO_STILL_S, SERVO_TIP, SERVO_WAIT_MAX_S, SETTLE_OFF_S, SETTLE_PAD_S, SPEED_MIN, STAND_A, STAND_Q, STAND_V, STEP_ACC, STEP_K,
    STEP_TIME_PAD_S, STEP_TOL, STEP_V, STEP_VMIN, TAIL_PAD_S, TIP_MAX_J1_DEG, TIP_MAX_JOINT_DEG, TRIM_CHUNKS, TRIM_JTC, TRIM_S, VOX_MAX, VOX_S, Abort, JogState, R_quat,
    _path_spline, _spline_states, cart_step, drop_specks, quat_R, ramp_to, rot_angle, rotvec, scale_path)

T_TIP = cm.T_of([0.0, 0.0, cm.TIP_M], [0.0, 0.0, 0.0])          # Link6 → 공구 끝
T_TIP_INV = np.linalg.inv(T_TIP)
FREE_M = 1.0                # (지도 끔) 계획기에 넣는 빈 지도의 칸 값 = 거리 1 m (rt_world.World._empty_grid 와 같음)


class GuiWorld(World):
    """[2026-11-02 디버깅 D-7, 사용자 "카메라가 끄기 상태라면 장애물 고려하지 않는 모션을 생성하도록"] 엔진 World + 카메라 지도 끄기 (map_off).
    켜면 계획기 장면 (멈춰서 계획·멈추지 않는 다시 계획·공구 끝 IK) = 바닥 (테이블) 상자 + 빈 지도, 거리 (계획 검증·안전 검사·경로 앞 막힘·
    목표 막힘) = 바닥 면만 → 카메라 지도 칸 (장애물) 을 안 봄. 자기 충돌·관절 한계는 계획기가 그대로 봄. 쌓인 지도는 그대로 둠 (끄면 다시 씀).
    GuiController 가 실시간 회피 끔 + 고정 카메라 꺼짐일 때만 켬 (_choose_map)."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.map_off = False
        self._vg_free = None                # 빈 지도 (MPC 지도와 같은 이름·자리·크기, 처음 쓸 때 만듦)

    def plan_prep(self, grid='mpc', q0=None, clear_m=0.015):
        if not self.map_off:
            return super().plan_prep(grid, q0, clear_m)
        if self._vg_free is None:
            vg = self.cm.esdf_mpc()
            self._vg_free = VoxelGrid(name=vg.name, pose=list(vg.pose), dims=list(vg.dims), voxel_size=vg.voxel_size,
                                      feature_tensor=torch.full_like(vg.feature_tensor, FREE_M))
        self.planner.update_world(Scene(cuboid=[self.floor], voxel=[self._vg_free]).get_collision_check_world())

    def clearances(self, Q, grids=('raw', 'dyn'), floor=True) -> dict:
        if not self.map_off:
            return super().clearances(Q, grids, floor)
        c, r = self.spheres(Q)
        d = torch.full_like(r, FAR_M)
        if floor:
            d = torch.where(self.moving_sph, c[..., 2] - r - self.floor_z, d)      # 바닥 면 (움직이는 구만 — 엔진 clearances 와 같음)
        d = torch.where(r > 0, d, FAR_M).min(dim=1).values.cpu().numpy()
        far = np.full(len(d), FAR_M)
        return {g: far if g == 'dyn' else d for g in grids}                       # 'dyn' (새 물체 지도) 는 엔진도 바닥 면 없음


class GuiClock(WallClock):
    """실제 시계 + 이상적 로봇: 계획에 걸린 시간을 따로 건너뛰지 않음 (이미 실제로 지남)."""
    def advance(self, s):
        pass


class GuiRosRobot(RosRobot):
    """RosRobot + 마지막 /joint_states 를 받은 시각 (끊김 감시) + 관절각 기록 (실제 카메라 영상 시각의 관절각 — 팔 지우기)
    + 서보 켜기/끄기 (R2 ros2_control 하드웨어 활성/비활성).
    R2 하드웨어 (wmx_r2_control WmxSystemHardware): 활성 = 알람 해제 → 서보 ON → 명령을 지금 위치로 → 버퍼 시작,
    비활성 = 버퍼 중단 → 정지 → 서보 OFF. 팔 제어기 (JTC) 는 하드웨어를 끄기 전에 끄고, 켠 뒤에 켬.
    [2026-10-27] + MoveIt Servo (조그): 관절·공구 끝 속도 명령 보내기, 명령 종류 바꾸기, Servo 상태, 팔 제어기 토픽에 Servo 가 낸 최신
    궤적 (마지막 점 = Servo 가 계산한 가장 앞 명령) — 조그 동안 서버는 그 토픽에 안 보내므로 받은 것은 다 Servo 것.
    [2026-10-27 R-1 ⑤] 나이·시간 한도에 쓰는 '받은 시각' 은 time.monotonic (벽시계가 NTP 로 바뀌어도 끊김 감시·조그 시간이 맞게) —
    t_js (벽시계) 는 관절각 기록 (영상 시각과 맞춤) 에만. [R-1 ⑭] 팔 제어기 토픽은 날 것 (raw) 으로 받아 조그 동안 (jt_watch) 만 풂
    (서버 자신의 20 Hz 메시지를 늘 풀던 것)."""
    def __init__(self):
        self.t_js = 0.0                                 # 마지막 /joint_states (벽시계 — 헤더 시각 없을 때 기록 시각)
        self.t_js_mono = 0.0                            #  같은 때 (monotonic — 끊김 감시)
        self._hist = collections.deque(maxlen=300)      # (메시지 시각, q) 100 Hz × 3 s
        self.servo_status = (0, '', 0.0)                # (코드, 글, 받은 시각 monotonic) — moveit_msgs/ServoStatus
        self.jt_last = (0.0, None)                      # (받은 시각 monotonic, 마지막 점 관절각) — 팔 제어기 토픽의 마지막 궤적
        self.jt_watch = False                           # 조그 중 (GuiController 가 켬) — 그때만 팔 제어기 토픽을 풂
        self._servo_type = None
        super().__init__(start=False)                   # [R-1 S2] 받기 스레드는 구독·발행을 다 만든 뒤에 (아래 start)
        from control_msgs.msg import JointJog
        from controller_manager_msgs.srv import ListControllers, ListHardwareComponents, SetHardwareComponentState, SwitchController
        from geometry_msgs.msg import TwistStamped
        from moveit_msgs.msg import ServoStatus
        from moveit_msgs.srv import ServoCommandType
        self._sw = self.node.create_client(SwitchController, '/controller_manager/switch_controller')
        self._hw = self.node.create_client(SetHardwareComponentState, '/controller_manager/set_hardware_component_state')
        self._lh = self.node.create_client(ListHardwareComponents, '/controller_manager/list_hardware_components')
        self._lc = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        self._jj = self.node.create_publisher(JointJog, f'{SERVO_NODE}/delta_joint_cmds', 10)
        self._tw = self.node.create_publisher(TwistStamped, f'{SERVO_NODE}/delta_twist_cmds', 10)
        self._sct = self.node.create_client(ServoCommandType, f'{SERVO_NODE}/switch_command_type')
        self.node.create_subscription(ServoStatus, f'{SERVO_NODE}/status', self._on_servo_status, 10)
        self.node.create_subscription(self.JT, f'{CONTROLLER}/joint_trajectory', self._on_jt, 10, raw=True)
        self.start()

    def _on_servo_status(self, m):
        self.servo_status = (int(m.code), m.message, time.monotonic())

    def _on_jt(self, data):
        """팔 제어기 토픽 (날 것 bytes): 조그 중이 아니면 안 풂 — 조그 판단 (_servo_q·_servo_stopping) 은 조그 시작 뒤 메시지만 씀."""
        if not self.jt_watch:
            return
        from rclpy.serialization import deserialize_message
        m = deserialize_message(data, self.JT)
        if not m.points:
            return
        try:
            idx = [list(m.joint_names).index(n) for n in JOINTS]
        except ValueError:
            return
        p = m.points[-1].positions
        with self.lock:
            self.jt_last = (time.monotonic(), np.array([p[i] for i in idx], float))

    def jtc_ref(self, max_age=JTC_REF_AGE_S):
        """팔 제어기 (JTC) 가 지금 따라가는 기준 위치 (controller_state) — 없거나 오래면 None. 나이 = 엔진 RosRobot._on_jtc 가 줄과 같이
        (lock 안) 적는 monotonic 시각 t_jtc_mono ([2026-10-28 R-1] 전엔 여기서 _on_jtc 를 덮어써 적었음)."""
        with self.lock:
            if not self.jtc:
                return None
            pos, t = self.jtc[-1][2], self.t_jtc_mono
        return np.array(pos, float) if time.monotonic() - t < max_age else None

    def servo_ready(self, timeout):
        return self._sct.wait_for_service(timeout_sec=timeout)

    def servo_type(self, kind):
        """Servo 명령 종류 (0 = 관절 JointJog, 1 = 공구 끝 Twist). 반환: 됐나."""
        if self._servo_type == kind:
            return True
        from moveit_msgs.srv import ServoCommandType
        r = self._call(self._sct, ServoCommandType.Request(command_type=kind), 2.0)
        if r is not None and r.success:
            self._servo_type = kind
            return True
        return False

    def servo_joint(self, j, v):
        """관절 j 를 v rad/s 로 (speed_units — cr3a_servo.yaml)."""
        from control_msgs.msg import JointJog
        m = JointJog()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.joint_names, m.velocities = [JOINTS[j]], [float(v)]
        self._jj.publish(m)

    def servo_twist(self, frame, v, w):
        """공구 끝 속도 v (m/s) · 각속도 w (rad/s), frame 축 기준 (SERVO_BASE 받침 / SERVO_TIP 공구). 돌기는 공구 끝 점 둘레."""
        from geometry_msgs.msg import TwistStamped
        m = TwistStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = frame
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = (float(x) for x in v)
        m.twist.angular.x, m.twist.angular.y, m.twist.angular.z = (float(x) for x in w)
        self._tw.publish(m)

    def _on_js(self, m):
        super()._on_js(m)
        self.t_js, self.t_js_mono = time.time(), time.monotonic()
        ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        with self.lock:
            if self.q is not None:
                self._hist.append((ts if ts > 0 else self.t_js, self.q.copy()))

    def q_at(self, t):
        """시각 t 의 관절각 (기록을 선형 보간, 기록 밖이면 끝 값) — 실제 카메라 영상이 찍힌 순간의 팔 자세."""
        with self.lock:
            h = list(self._hist)
        if not h:
            return self.measured()
        ts = np.array([x[0] for x in h])
        i = int(np.searchsorted(ts, t))
        if i <= 0:
            return h[0][1].copy()
        if i >= len(h):
            return h[-1][1].copy()
        a = (t - ts[i - 1]) / max(ts[i] - ts[i - 1], 1e-6)
        return (1 - a) * h[i - 1][1] + a * h[i][1]

    @staticmethod
    def _call(cli, req, timeout=10.0):
        """서비스 부르기 → 응답 또는 None (서비스 없음·시간 초과). [R-1 ⑨] 끝나기를 Event 로 기다림 (전엔 10 ms 마다 물어봄)."""
        if not cli.wait_for_service(timeout_sec=3.0):
            return None
        fut = cli.call_async(req)               # 노드는 RosRobot 스레드가 돌림 (끝나면 그 스레드가 알림)
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        done.wait(timeout)
        return fut.result() if fut.done() else None

    def hw_state(self):
        """팔 하드웨어 상태 ('active'·'inactive'·'unconfigured'…) 또는 None (못 물음)."""
        from controller_manager_msgs.srv import ListHardwareComponents
        r = self._call(self._lh, ListHardwareComponents.Request(), 5.0)
        return None if r is None else next((c.state.label for c in r.component if c.name == HW_NAME), None)

    def set_servo(self, on):
        """서보 켜기/끄기. 반환 (됐나, 설명)."""
        from builtin_interfaces.msg import Duration
        from controller_manager_msgs.srv import SetHardwareComponentState, SwitchController
        from lifecycle_msgs.msg import State
        target = State(id=State.PRIMARY_STATE_ACTIVE if on else State.PRIMARY_STATE_INACTIVE, label='active' if on else 'inactive')
        sw = SwitchController.Request(strictness=SwitchController.Request.BEST_EFFORT, activate_asap=True,
                                      timeout=Duration(sec=5))
        if not on:
            sw.deactivate_controllers = [ARM]
            r = self._call(self._sw, sw)
            if r is None or not r.ok:
                return False, '팔 제어기를 못 끔'
        r = self._call(self._hw, SetHardwareComponentState.Request(name=HW_NAME, target_state=target), 25.0)
        if r is None or not r.ok:
            return False, f'하드웨어를 {"켜지" if on else "끄지"} 못함 ({None if r is None else r.state.label})'
        if on:
            # [2026-10-27 ⑥] R2 고장 (하드웨어 오류) 뒤에는 컨트롤러 매니저가 joint_state_broadcaster 도 꺼 둠 → 꺼져 있으면 같이 켬
            #  (안 켜면 /joint_states 가 안 와 [알람 해제] 뒤에도 다시 R2 오류). 이미 켜진 것은 빼고 부름
            sw.activate_controllers = self._inactive([ARM, JSB])
            r = self._call(self._sw, sw) if sw.activate_controllers else None
            if sw.activate_controllers and (r is None or not r.ok):
                return False, '팔 제어기를 못 켬'
        return True, self.hw_state() or '?'

    def _inactive(self, names):
        """names 중 켜져 있지 않은 컨트롤러 (못 물으면 다)."""
        from controller_manager_msgs.srv import ListControllers
        r = self._call(self._lc, ListControllers.Request(), 5.0)
        if r is None:
            return list(names)
        st = {c.name: c.state for c in r.controller}
        return [n for n in names if st.get(n) != 'active']


class GuiController(rt_run.Controller):
    def __init__(self, *a, link, shaper_cfg, out=None, real_cam=None, mask_q=None, kind='ideal', speed=1.0, realtime=True, **kw):
        super().__init__(*a, **kw)
        self.link = link                    # GUI 연결 (rt_server.Link) — [R-1 ⑮] 전엔 make_controller 의 닫힌 변수
        self.kind = kind                    # 'ideal' | 'wmx' (WMX 시뮬레이터) | 'real' (EtherCAT 실물) — 제어는 wmx·real 이 같음
        self.real_cam = real_cam            # 실제 카메라 (DepthFeed — 깊이만) — None = 가상 카메라 (rt_world 렌더)
        self.mask_q = mask_q                # 실제 카메라로 팔 지울 고정 자세 (시뮬레이션 로봇일 때 = 엔코더 읽은 실제 팔), None = 영상 시각의 관절각
        self._n_frame = 0
        self.realtime = realtime            # [2026-10-26 사용자 "꺼진 채로 시작"] 실물은 끔으로 시작 (명령할 때만 움직임)
        self.speed = self.speed_req = float(np.clip(speed, SPEED_MIN, 1.0))     # 쓰는 배율 · GUI 가 고른 배율 (다음 일부터)
        self.rt_scale = self.speed          # [D-1 ㄱ] 실시간 회피 추종기 한계에 쓰는 배율 (_track_lim)
        # (실시간 회피 끔) 명령 = 계획 경로 그대로 — _command_carrot. [2026-10-27 R-1 ①] 경로는 id() 가 아니라 그 경로 묶음 자체를 들고
        #  'is' 로 비교: Mpc.hold 가 경로를 놓으면 (path = None) 그 묶음이 풀려 다음 경로가 같은 id 를 받을 수 있었음 → 새 경로인데 옛
        #  스플라인으로 명령 (또는 '벗어남' 판단을 이어 받음). 들고 있으면 같은 id 가 다시 나올 수 없음
        self._exact_path, self._exact, self._spl = None, False, None
        self._direct = False                # 지난 주기가 직접 따라가기였나 (MPC 로 돌아갈 때 새로 시작)
        self._scaled = []                   # 계획한 경로마다 (배율, 계획 시간, 늘린 시간, 끝점 맞춤 mm, 배율 1 속도·가속 사용률, 점 수 — scale_path)
        #                                     — 계획기 스레드도 덧붙임 (list.append 는 스레드에 안전)
        self.link.hello['speed'] = self.speed
        self.estopped = False
        self.job = None
        self.job_goal = None
        self.pending = collections.deque()
        self.quit = False
        self.ready = False
        self.sj = None                      # [2026-10-27] MoveIt Servo 조그 (JogState, _servo_cmd) — 동안 서버는 R2 에 안 보냄
        self.servo_ok = False               # Servo 가 떠 있음 (main 이 확인)
        self._hold_block = False            # 정지·비상정지로 멈춘 누르는 동안 조그: 뗄 때까지 다시 안 움직임
        self._said = ('', -math.inf)        # 마지막 거절 글·시각 (monotonic) — _say_once
        self.settle_until = 0.0
        self.shaper_cfg = shaper_cfg
        self._refreshing = False
        self._map_off_said = False          # [D-7] 이 명령 (goto·repeat …) 에서 '카메라 꺼짐 → 장애물 안 봄' 을 알렸음 (_choose_map)
        # [R-1 ⑤] 아래 간격·나이 시각은 time.monotonic (벽시계가 바뀌어도 맞게). 제어 시계 (clock — 메시지 스탬프) 는 그대로 벽시계
        self._t_pub = self._t_vox = -math.inf
        self._t_trim = time.monotonic()
        self._path_sent = None              # GUI 에 마지막으로 보낸 경로 (묶음 자체 — [R-1 ①] id() 가 다시 쓰일 수 있어서)
        self._vox_err = False               # 지도 칸 보내기 오류를 한 번 알렸음
        self._size_warned = False           # 실제 카메라 영상 크기가 다름을 한 번 알렸음
        self.label = '켜기'
        self.out = out
        self.t_start = time.monotonic()
        self.rec_max_s = REC_MAX_S          # 기록 다 남기는 시간 (시험: --test-rec-max-s)
        self.old_runs = []                  # 바꾸기 전 성형기로 실행된 u (성형 확인용, 성형기마다)
        self.estop_windows = []             # 비상정지 감속 구간 [(스탬프, 선 시각)] — 성형을 건너뜀 (성형 확인에서 뺌)
        self._estop_until = 0.0             # 비상정지 감속이 끝나는 시각 (그 전에는 새 명령 시간표를 시작하지 않음)
        self.r2_fault = None                # (WMX) R2 오류 줄·/joint_states 끊김
        self._fault_done = False
        self._t_gc = time.monotonic()
        self.servo_on = not self.ideal      # (WMX) R2 가 켜질 때 서보 ON (하드웨어 활성)
        self._defer_frames = False          # (WMX) cycle 안 계산 직전: 영상 처리를 보낸 뒤로 미룸
        # ⑤ (WMX) 엔진 통신이 끊기는 시각 (무료 라이선스, time.monotonic 기준 — main 이 넣음), 그 시간 수
        self.comm_deadline = self.comm_hours = None
        self._comm_warned, self._comm_stopped = set(), False
        self.serving = False                # [R-1 ⑥] serve 가 명령을 받는 중 — 그때 SIGTERM·SIGINT 첫 번째 = 끄기 (sig_quit)
        self.sig_quit = None                #  신호 처리기가 넣는 신호 이름 (처리기에서는 잠금 없이 이것만) → _poll 이 [■ 시스템 종료] 처럼
        self._sig_done = False
        # [2026-10-28 D-4] 이미지 기록 (snap_log.SnapLog — setup_snap, 실제 카메라 + 기록 폴더일 때만)
        self.snap = None
        self._snap_trig_m = None            # near 트리거 기준 (m) — 기본 act_new − 5 mm (snap_log.env_near_m, 시험: CR3A_SNAP_NEAR_MM)
        self._snap_below = False            # 지난 영상이 near 기준 안
        self._snap_t_near = -math.inf       # 마지막 near 기록 시각 (벽시계)
        self._snap_guard = None             # 지난 주기 안전 검사 (guard 감속 정지가 새로 시작될 때만)
        self._dfilt = None                  # [D-4] 깊이 거르기 (TemporalFilter | None, 가림 띠 (side, tol) | None) — setup_depth_filters
        self._fx = None

    # ---------------- [2026-10-28 D-4] 이미지 기록 ----------------
    def setup_snap(self, root, pose):
        """실제 카메라 이미지 기록 켜기 (rt_server.make_controller). pose = camera_pose.json (깊이·컬러 내부 값, 받침 ← 카메라)."""
        import snap_log
        s = self.W.cm.s
        T_bd = np.array(pose['T_base_depth'], float)
        T_bc = pose.get('T_base_color')
        if T_bc is None and pose.get('T_color_depth') is not None:
            T_bc = T_bd @ np.linalg.inv(np.array(pose['T_color_depth'], float))
        if T_bc is None or pose.get('color_K') is None:
            self._log('이미지 기록: 카메라 보정에 컬러 값 (color_K·T_base_color) 이 없음 — 이미지 기록 안 함 (카메라 보정을 다시)')
            return
        cam = {'K': np.array(pose['depth_K'], float).reshape(3, 3), 'T_base_depth': T_bd, 'T_base_color': np.array(T_bc, float),
               'Kc': np.array(pose['color_K'], float).reshape(3, 3), 'Dc': pose.get('color_D')}
        q_fn = (lambda st: self.mask_q) if self.mask_q is not None else self.robot.q_at

        def frame_info(stamps):
            idx = {f['t']: f for f in self.rec['frames'][-150:] if f.get('real')}     # (_trim 이 목록을 바꿔 끼우므로 부를 때 읽음)
            out = []
            for st in stamps:
                f = idx.get(round(st, 4))
                out.append(None if f is None else {'q_ctl_deg': np.round(np.degrees(f['q']), 3).tolist(), 'lag_ms': f.get('lag_ms'),
                                                   'map_ms': f.get('map_ms'), 'near_mm': f.get('near_mm'),
                                                   'near_new_mm': f.get('near_new_mm'), 'fill': f.get('fill'), 'speck': f.get('speck'),
                                                   'dfilt': f.get('dfilt')})
            return out

        def near():
            q = np.asarray(self.robot.measured(), float)
            r = self._near_now(q)
            r['q'] = q
            r['clr_mpc_mm'] = None if self._clr_mpc is None else round(self._clr_mpc * 1000, 1)
            return r

        def state():
            cy = self.rec['cycles'][-1] if self.rec['cycles'] else {}
            return {'realtime': self.realtime, 'label': self.label, 'job': None if self.job is None else self.job['label'],
                    'speed': self.speed, 'mode': self.mpc.mode, 'clr_mpc_mm': cy.get('clr_mpc_mm'), 'clr_now_mm': cy.get('clr_now_mm'),
                    'guard': cy.get('guard'), 'mask_q': 'fixed' if self.mask_q is not None else 'encoder q_at(stamp)'}
        self._snap_trig_m = snap_log.env_near_m(self.c.dist.act_new)
        self.snap = snap_log.SnapLog(
            self.real_cam, Path(root), cam, mask_m=s.segment_distance_threshold, near_m=self.c.dist.act_new, floor_z=s.mpc_floor_z_m,
            base_exclude=s.base_exclude, base_ring=s.base_ring, link_names=self.W.sphere_link_names(), q_fn=q_fn,
            spheres_fn=self._snap_spheres, frame_info_fn=frame_info, near_fn=near, state_fn=state, speck=(SPECK_PX, SPECK_DIFF_MM),
            log=self._log, rec_list=self.rec.setdefault('snaps', []))
        self._log(f'이미지 기록 켬: 비킴·경로 앞 막힘·안전 검사 정지·MPC 지도 {self._snap_trig_m * 1000:.0f} mm 안 (실시간 회피 중)·지도 점검 걸림 '
                  f'때 앞뒤 {snap_log.SNAP_PRE_S:g}+{snap_log.SNAP_POST_S:g} s 깊이·컬러 → {root} ({snap_log.SNAP_GAP_S:g} s 에 한 번, '
                  f'{snap_log.SNAP_MAX} 번까지)')

    # ---------------- [2026-10-28 D-4] 실제 카메라 깊이 거르기 (depth_filters) ----------------
    def setup_depth_filters(self):
        """(실제 카메라) 지도에 쌓기 전 거르기 켜기 — 환경 변수 (depth_filters.settings_from_env). 작업기록 D-4 · tools/d4_band_eval.py."""
        import depth_filters as dfl
        temporal, band = dfl.settings_from_env()
        tf = dfl.TemporalFilter(*temporal) if temporal is not None else None
        self._dfilt = (tf, band) if (tf is not None or band is not None) else None
        self._fx = float(self.W.cm.intrinsics[0, 0, 0])
        self.rec['settings']['depth_filters'] = {'temporal': list(temporal) if temporal else None, 'band': list(band) if band else None}
        self._log('깊이 거르기 (D-4 가림 띠): '
                  + (f'시간 확인 지난 {temporal[1]} 장 중 {temporal[0]} 장 깊이 있음'
                     + (f' (가운데값 ±{temporal[2] * 1000:.0f} mm)' if temporal[2] is not None else '')
                     + f' — 깊이가 없던 자리에 새로 나타난 것만 {(temporal[0] - 1) * 1000 / 15:.0f} ms 늦음' if temporal else '시간 확인 끔')
                  + ' · ' + (f'가림 띠 지우기 (로봇 {"왼쪽" if band[0] == "left" else band[0]} {dfl.BASELINE_M * 100:.1f} cm 안, 로봇 깊이 '
                             + {'same': f'±{band[1] * 1000:.0f} mm', 'behind': f'−{band[1] * 1000:.0f} mm 보다 뒤 모두',
                                'behind_adaptive': f'−{band[1] * 1000:.0f} mm 보다 뒤 모두, 띠 폭은 그 줄 배경 거리로 (배경이 멀면 9.5 cm)',
                                'behind_adaptive_2d': f'(위아래 {dfl.band_kw(band[2])["ref_rows"]} 줄 안 가장 가까운 로봇 부분) '
                                                      f'−{band[1] * 1000:.0f} mm 보다 뒤 모두, 띠 폭은 그 줄 배경 거리로 (배경이 멀면 9.5 cm)'
                                }.get(band[2], f'모드 {band[2]}')
                             + ' — 그 자리 실제 물체도 안 보임)' if band else '가림 띠 지우기 끔')
                  + ' (끄기: CR3A_DEPTH_TEMPORAL=0 · CR3A_DEPTH_BAND=0)')

    def _depth_filter(self, num, raw, d, q_mask):
        """깊이 d (m, GPU — 외딴 조각 지운 것) → (거른 깊이, 지운 픽셀 수). 시간 확인은 원래 깊이 (raw uint16) 지난 n 장 (서버가 건너뛴
        영상은 DepthFeed 기록에서 채움), 가림 띠는 팔 지우기 픽셀 (cuRobo segmenter — 쌓기 안에서 한 번 더 함) 기준."""
        import torch
        import depth_filters as dfl
        tf, band = self._dfilt
        cnt, names = [], []
        if tf is not None:
            for m in range(num - tf.n + 1, num):
                if not tf.has(m):
                    r = self.real_cam.frame(m)
                    if r is not None:
                        tf.push(m, torch.from_numpy(r.astype(np.float32)).to('cuda').mul_(0.001))
            keep = tf.keep(num, torch.from_numpy(raw.astype(np.float32)).to('cuda').mul_(0.001))
            cnt.append(((d > 0) & ~keep).sum())
            names.append('tmp')
            d = d * keep
        if band is not None:
            from curobo.types import CameraObservation
            cm = self.W.cm
            h, w = d.shape
            m, _ = cm.segmenter.get_robot_mask_from_active_js(
                CameraObservation(depth_image=d.view(1, h, w).clone(), pose=cm.camera_pose), self.W.js(q_mask))
            rm = dfl.shadow_band(d, m.view(h, w), self._fx, band[0], tol_m=band[1], **dfl.band_kw(band[2]))
            cnt.append(rm.sum())
            names.append('band')
            d = d * ~rm
        return d, dict(zip(names, (int(v) for v in torch.stack(cnt).tolist())))

    SNAP_FK_N = 48              # 구 계산 묶음 크기 (늘 같은 크기 — 기구학 버퍼를 새로 잡지 않게)

    def _snap_spheres(self, Q):
        """관절 [n, 6] → 구 중심 [n, S, 3]·반지름 [n, S] numpy (제어 스레드 — 기구학 버퍼를 같이 씀)."""
        Q = np.asarray(Q, float)
        n = len(Q)
        Qp = np.repeat(Q[-1:], max(self.SNAP_FK_N, n), 0)
        Qp[:n] = Q
        c, r = self.W.spheres(Qp)
        return c[:n].cpu().numpy(), r[:n].cpu().numpy()

    def event(self, text, **kw):
        super().event(text, **kw)
        snap = getattr(self, 'snap', None)
        if snap is not None:
            import snap_log
            why = snap_log.reason_of(text)
            if why is not None:
                snap.trigger(why, text, **{k: v for k, v in kw.items() if isinstance(v, (int, float, str, type(None)))})

    def _snap_check_frame(self, stamp, near_mm):
        """(실시간 회피 중 영상마다) MPC 지도 거리 (영상 시각 관절) 가 near 기준 안 — 들어올 때 · 계속 안이면 SNAP_NEAR_HOLD_S 마다."""
        import snap_log
        below = near_mm is not None and near_mm < self._snap_trig_m * 1000
        if below and (not self._snap_below or stamp - self._snap_t_near >= snap_log.SNAP_NEAR_HOLD_S):
            if self.snap.trigger('near', f'MPC 지도 거리 {near_mm:.0f} mm < {self._snap_trig_m * 1000:.0f} mm (영상 시각 관절)',
                                 t_frame=stamp, near_mm=near_mm):
                self._snap_t_near = stamp
        self._snap_below = below

    def _snap_check_cycle(self):
        """주기 뒤: 안전 검사 감속 정지가 새로 시작되면 기록."""
        cy = self.rec['cycles'][-1] if self.rec['cycles'] else None
        g = cy['guard'] if cy else None
        if g == 'brake' and self._snap_guard != 'brake':
            self.snap.trigger('guard', f'안전 검사 감속 정지 (실제 거리 {cy["clr_now_mm"]} mm · 새 물체 {cy["clr_new_now_mm"]} mm · MPC 지도 '
                                       f'{cy["clr_mpc_mm"]} mm)', clr_now_mm=cy['clr_now_mm'], clr_new_now_mm=cy['clr_new_now_mm'],
                              clr_mpc_mm=cy['clr_mpc_mm'])
        self._snap_guard = g

    # ---------------- GUI 명령 ----------------
    def _poll(self):
        """주기마다: 신호 (끄기)·R2 고장·통신 남은 시간·카메라·GUI 끊김 확인 → 쌓인 GUI 명령 처리."""
        if self.sig_quit and not self._sig_done:
            # [R-1 ⑥] SIGTERM·SIGINT 첫 번째: 처리기는 이름만 넣고 (어디서든 끼어들 수 있어 잠금·로그를 안 씀) 여기 주기 사이에서
            #  [■ 시스템 종료] 와 같은 길 — 일 중이면 Abort('quit') → 성형 거쳐 정지, serve 가 끝나면 main 이 기록 저장·R2 끄기
            self._sig_done = True
            self.event(f'{self.sig_quit} 받음 → 끄기 ([■ 시스템 종료] 와 같음 — 한 번 더 보내면 바로 끝)')
            self._handle({'cmd': 'quit'})
        self._check_faults()
        self._check_comm()
        self._check_camera()
        if self.link.check_silent():
            self._gui_lost(f'GUI 가 {GUI_LOST_S:.0f} s 동안 응답 없음')
        self._take_cmds(0.0)

    def _take_cmds(self, wait_s):
        """쌓인 GUI 명령을 다 처리. wait_s > 0 (서 있음 — _idle) 이면 명령이 없을 때 그만큼 기다리다 오면 바로 처리 ([R-1 ⑨] 전엔
        50 ms 잠든 뒤 다음 _idle 에서). 명령 하나의 오류로 서버가 죽지 않게: GUI·기록에는 짧은 글, 출력 (파일) 에는 traceback 도 ([R-1 ⑩])."""
        try:
            m = self.link.cmds.get(timeout=wait_s) if wait_s > 0 else self.link.cmds.get_nowait()
        except queue.Empty:
            return
        while True:
            try:
                self._handle(m)
            except Abort:
                raise
            except Exception as e:
                self.event(f'명령 처리 오류 {m.get("cmd")}: {e!r}')
                LOGGER.error(f'  (명령 처리 오류 {m.get("cmd")} — traceback)', exc_info=True)
            try:
                m = self.link.cmds.get_nowait()
            except queue.Empty:
                return

    def _check_faults(self):
        """(WMX) R2 오류 줄 또는 /joint_states 끊김 → 한 번 비상정지 (그 뒤 움직이는 명령은 거절 — 시스템을 다시 켜야 함)."""
        if self.ideal or self._fault_done:
            return
        age = time.monotonic() - self.robot.t_js_mono
        if self.r2_fault is None and self.ready and self.servo_on and age > JS_STALE_S:
            self.r2_fault = f'/joint_states 가 {age:.1f} s 동안 안 옴'
        if self.r2_fault is not None:
            self._fault_done = True
            # [2026-10-27 ⑥ 사용자 결정 "R2 고장으로 서보를 끌 때는 바로 비상 정지"] R2 하드웨어 오류면 R2 가 WMX 비상정지 Final (모든 축 서보
            #  즉시 OFF, 브레이크가 팔을 잡음) 을 했음 → 하드웨어가 활성이 아니면 서보 OFF 로 표시 ([알람 해제] 가 다시 켬)
            hw = self.robot.hw_state()
            if hw not in (None, 'active') and self.servo_on:
                self.servo_on = False
                self.link.send({'type': 'servo', 'on': False})
            self.event(f'R2 오류 → 비상정지: {self.r2_fault}' + (f' (R2 하드웨어 {hw} — 서보 OFF)' if hw not in (None, 'active') else ''))
            raise Abort('estop')

    def _comm_left(self):
        return None if self.comm_deadline is None else self.comm_deadline - time.monotonic()

    def _comm_ending(self):
        left = self._comm_left()
        return left is not None and left <= COMM_STOP_S

    def _check_comm(self):
        """⑤ WMX 무료 라이선스 남은 시간: COMM_WARN_S 마다 경고, COMM_STOP_S 아래면 움직이는 일 정지·실시간 회피 끔 (새 이동은 _handle 이 거절)."""
        left = self._comm_left()
        if left is None:
            return
        new = [w for w in COMM_WARN_S if left <= w and w not in self._comm_warned]
        if new:                                     # 한꺼번에 여러 단계를 넘으면 (켤 때 이미 30 분 아래) 한 번만
            self._comm_warned.update(new)
            self.event(f'WMX 통신 남은 시간 {max(left, 0.0) / 60:.0f} 분 (무료 라이선스 {self.comm_hours:g} 시간 — 다 되면 엔진이 통신을 끊어 '
                       f'서보가 꺼짐). ' + ('일을 멈추고 새 이동은 안 받음 — 시스템을 껐다 켜기' if min(new) <= COMM_STOP_S else
                                       '시스템을 껐다 켜면 다시 처음부터'))
        if left <= COMM_STOP_S and not self._comm_stopped:
            self._comm_stopped = True
            self._quiet('WMX 통신 끝나기 직전')

    def _check_camera(self):
        """④ (실제 카메라) 실시간 회피 중 깊이 영상이 CAM_STALE_S 넘게 안 옴 = 지도가 눈을 잃음 → 정지·실시간 회피 끔 (끔이면 이동 직전
        지도 새로 보기에서 거절 — leg)."""
        if self.real_cam is None or not self.realtime or not self.ready:
            return
        age = self._cam_age()
        if age > CAM_STALE_S:
            self._quiet('고정 카메라 꺼짐 (실시간 회피는 카메라 지도가 있어야 함 — 이동은 장애물을 안 보고 함)' if self._cam_off() else
                        f'카메라 영상이 {self._cam_age_text()}')

    def _cam_age(self):
        """(실제 카메라) 마지막 깊이 영상을 받은 뒤 지난 시간 (s, monotonic — ServerCameraFeed)."""
        return time.monotonic() - self.real_cam.t_depth_mono

    def _cam_age_text(self):
        """'카메라 영상이 …' 뒤 글 — [D-7] 켠 뒤 한 번도 안 왔으면 'inf s' 대신."""
        age = self._cam_age()
        return f'{age:.1f} s 동안 안 옴' if math.isfinite(age) else '한 번도 안 옴'

    def _cam_off(self):
        """[D-7] (실제 카메라 지도) 고정 카메라가 꺼짐 = 깊이 영상이 CAM_STALE_S 넘게 안 옴 + 카메라 노드도 없음 (GUI [■ 끄기] — PID 파일).
        노드가 도는데 영상이 안 오면 (켜는 중·USB 끊김) 꺼짐이 아님 — 고장으로 보고 이동 거절 (leg)."""
        return self.real_cam is not None and self._cam_age() > CAM_STALE_S and cio.CameraProcess.pid() is None

    def _choose_map(self, label):
        """[D-7] 계획 직전 (다리·공구 끝 IK): 실시간 회피 끔 + 고정 카메라 꺼짐이면 카메라 지도 없이 (GuiWorld.map_off — 책상·로봇 자기 충돌·
        관절 한계만), 아니면 지도를 씀. 명령마다 한 번 알림 ('주의' = GUI 로그 노랑). 반환 = 지도 안 씀."""
        off = not self.realtime and self._cam_off()
        if off and not self._map_off_said:
            self._map_off_said = True
            self.event(f'{label}: 주의 — 고정 카메라가 꺼져 있음 → 장애물 (카메라 지도) 을 안 보고 계획·이동 '
                       f'(책상·로봇 자기 충돌·관절 한계만 봄)')
        elif not off and self.W.map_off:
            self.event(f'{label}: 고정 카메라가 켜져 있음 → 다시 장애물 (카메라 지도) 을 보고 계획')
        self.W.map_off = off
        return off

    def _quiet(self, why):
        """움직임을 다 그침: 대기 일 취소, 조그 멈춤 (Servo), 실시간 회피 끔, 움직이는 일은 [정지] 처럼 (성형 거쳐 감속). 서보는 그대로."""
        moving = self.job is not None or bool(self.pending) or self.sj is not None or self.realtime
        self.pending.clear()
        if self.realtime:
            self._set_realtime(False)
        self.event(f'{why} → ' + ('정지 · 실시간 회피 끔 · 대기 일 취소' if moving else '서 있음 (할 일 없음)'))
        if self.job is not None:
            raise Abort('stop')
        if moving:
            self._stop(why)

    def _gui_lost(self, why):
        """① GUI 가 끊김 (창 닫기·GUI 멈춤): 아무도 못 보는 채로 움직이지 않게 — 대기 일 취소·실시간 회피 끔·정지. 다시 열면 이어서 조작."""
        self._quiet(f'GUI 연결 끊김 ({why})')

    def _goal_ok(self, q_deg):
        q = np.radians(np.asarray(q_deg, float))
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError('관절 6 개가 아님')
        lo, hi = self.mpc.qlim
        if np.any(q < lo) or np.any(q > hi):
            self.event('목표가 관절 한계 밖 — 안 움직임')
            return None
        return q

    # [2026-10-27 R-1 ⑪] 명령 → 처리 함수 표 (전엔 125 줄 if/elif). 거르는 차례는 그대로: ① 조그 (Servo) 중의 비상정지·정지·끄기 =
    #  Servo 멈춤 ② 움직이는 명령 (MOVES) 은 R2 오류 → 서보 꺼짐 → WMX 통신 끝남 차례로 거절 ③ 명령마다 처리 (모르는 명령은 무시).
    #  (②는 전엔 servo·alarm 다음이었지만 그 둘은 MOVES 가 아니라 같은 차례)
    _CMDS = {'estop': '_cmd_estop', 'stop': '_cmd_stop', 'quit': '_cmd_quit', 'servo': '_cmd_servo', 'alarm': '_cmd_alarm',
             'gui_lost': '_cmd_gui_lost', 'goto': '_cmd_goto', 'repeat': '_cmd_goto', 'goto_pose': '_cmd_goto',
             'repeat_pose': '_cmd_goto', 'jog': '_servo_cmd', 'jog_cart': '_servo_cmd', 'jog_hold': '_servo_cmd',
             'jog_cart_hold': '_servo_cmd', 'jog_release': '_cmd_jog_release', 'realtime': '_cmd_realtime',
             'map_check': '_cmd_map_check', 'shaper': '_cmd_shaper', 'speed': '_cmd_speed', 'objects': '_cmd_objects',
             'map_reset': '_cmd_map_reset'}

    def _handle(self, m):
        k = m.get('cmd')
        if self.sj is not None and k in ('estop', 'stop', 'quit'):
            # 조그 (Servo) 중: 서버 명령 시간표는 쉬는 중 → Servo 를 멈추고 (0 속도), 선 뒤 _servo_exit 가 마무리 (비상정지 표시·끄기)
            self._servo_halt({'estop': '비상정지', 'stop': '정지', 'quit': '끄기'}[k], estop=k == 'estop')
            if k == 'quit':
                self.quit = True
            return
        if k in MOVES:
            why = self._move_refusal()
            if why is not None:
                return self._say_once(why)          # (거절은 SAY_ONCE_S 에 한 번 — 누르는 동안 조그는 50 ms 마다 옴)
        name = self._CMDS.get(k)
        if name is not None:
            getattr(self, name)(m)

    def _move_refusal(self):
        """움직이는 명령을 받지 않을 까닭 (차례대로 첫 번째) 또는 None."""
        if self.r2_fault is not None:
            return f'R2 오류 뒤라 안 움직임 — [알람 해제] 또는 시스템 다시 켜기 ({self.r2_fault})'
        if not self.servo_on and not self.ideal:            # (이상적 모드는 서보가 없음 — 늘 꺼짐으로 보여 막혔었음)
            return '서보가 꺼져 있음 — [서보 켜기] 먼저'
        if self._comm_ending():
            return f'WMX 통신이 곧 끊김 ({max(self._comm_left(), 0.0):.0f} s 남음) — 시스템을 껐다 켠 뒤 움직이기'
        return None

    def _cmd_estop(self, m):
        raise Abort('estop')            # 주기 사이에서 (serve 가 받아 _estop)

    def _cmd_stop(self, m):
        if self.job is not None:
            raise Abort('stop')
        self._stop()

    def _cmd_quit(self, m):
        self.quit = True
        if self.job is not None:
            raise Abort('quit')

    def _cmd_servo(self, m):
        self._set_servo(bool(m['on']))

    def _cmd_alarm(self, m):
        self._clear_alarm()

    def _cmd_gui_lost(self, m):
        self._gui_lost(m.get('why', ''))

    def _cmd_goto(self, m):
        """goto·repeat (관절 목표) · goto_pose·repeat_pose (공구 끝 목표 → IK) → 대기 일."""
        k = m['cmd']
        if self.job is not None or self.sj is not None or self.pending:
            self.event('움직이는 중 — 끝나거나 [정지] 한 뒤에 다시')
            return
        q0 = np.array(self.stream.tail_end, float)
        self._map_off_said = False                          # [D-7] 카메라 꺼짐 알림은 명령마다 한 번
        if k in ('goto_pose', 'repeat_pose'):
            T0 = self._tip(q0)
            T1 = np.array(T0)
            T1[:3, 3] = np.asarray(m['xyz_mm'], float) / 1000
            if m.get('rpy_deg') is not None:
                T1[:3, :3] = cm.rpy_R(*np.radians(np.asarray(m['rpy_deg'], float)))
            lbl = '공구 끝 이동' if k == 'goto_pose' else '공구 끝 반복'
            self._choose_map(lbl)                           # [D-7] 카메라가 꺼졌으면 IK 도 지도 없이
            self.W.plan_prep('start', q0=q0)                # IK 도 지금 지도로 (지금 자세 둘레는 실제 거리 — 경계 채우기 빼고)
            q1, why = self._ik(T1, q0)
            if q1 is None:
                self.event(f'{lbl} 실패: {why} — 안 움직임')     # ('실패' = GUI 로그에 오류로)
                return
            goal = self._goal_ok(np.degrees(q1))
            if goal is not None:
                self.event(f'{lbl}: 공구 끝 목표 {np.round(T1[:3, 3] * 1000, 1).tolist()} mm → 관절 목표 '
                           f'{np.round(np.degrees(goal), 2).tolist()}° (cuRobo IK, {why})')
        else:
            goal = self._goal_ok(m['q_deg'])
            lbl = '목표 이동' if k == 'goto' else '반복'
        if goal is None:
            return
        self.pending.append({'kind': 'repeat' if k in ('repeat', 'repeat_pose') else 'goto', 'label': lbl, 'goal': goal,
                             'a': q0})

    def _cmd_jog_release(self, m):
        self._hold_block = False                # (정지·비상정지로 멈춘 누르는 동안 조그는 뗐다가 다시 눌러야 — _servo_halt)
        if self.sj is not None and self.sj.mode == 'hold':
            self._servo_halt('조그 뗌', soft=True)

    def _cmd_realtime(self, m):
        busy = self.job is not None
        if not m['on'] and self.realtime and busy:
            # [2026-10-27 D-1 독립 검토 HIGH] 일 도중 끄기: 직접 따라가기가 비키던 자리 (경로 밖) 에서 경로로 멈춘 지도를 보며 전체 한계로
            #  되돌아가게 됨 → 그 일을 [정지] (그 자리에서 섬)
            self._set_realtime(False)
            self.event('실시간 회피를 끄면 하던 이동도 멈춤 (비키던 자리에서 멈춘 지도로 경로에 되돌아가지 않게)')
            raise Abort('stop')
        if m['on'] and self.r2_fault is not None:       # [독립 검토 LOW] R2 오류 뒤 (서보 표시가 ON 이어도)
            self._say_once(f'R2 오류 뒤라 실시간 회피 안 켬 — [알람 해제] 또는 시스템 다시 켜기 ({self.r2_fault})')
            self.link.send({'type': 'realtime', 'on': False})
            return
        if m['on'] and not self.servo_on and not self.ideal:
            self.event('서보가 꺼져 있음 — 실시간 회피는 [서보 켜기] 뒤에')
            self.link.send({'type': 'realtime', 'on': False})
            return
        if m['on'] and self._comm_ending():
            self.event('WMX 통신이 곧 끊김 — 실시간 회피는 시스템을 껐다 켠 뒤에')
            self.link.send({'type': 'realtime', 'on': False})
            return
        if m['on'] and self.real_cam is not None and self._cam_age() > CAM_STALE_S:
            self.event('고정 카메라가 꺼져 있음 — 실시간 회피는 카메라를 켠 뒤에' if self._cam_off() else
                       f'카메라 영상이 {self._cam_age_text()} — 실시간 회피 안 켬 (카메라 확인)')
            self.link.send({'type': 'realtime', 'on': False})
            return
        if m['on'] and not self.realtime:
            # [2026-10-27 디버깅 D-1 ㄴ] 켜기 전 지도 점검 (_map_check) — 지도를 새로 쌓는 동안 주기가 멈추므로 로봇이 서 있을 때만
            if busy or self.sj is not None or self.pending or not self._standing():
                self.event('움직이는 중 (또는 서는 중) — 실시간 회피는 로봇이 선 뒤에 켬 (켜기 전 지도 점검)')
                self.link.send({'type': 'realtime', 'on': False})
                return
            if not self._map_check('실시간 회피 켜기 전 지도 점검', turning_on=True):
                self.link.send({'type': 'realtime', 'on': False})
                return
        self._set_realtime(bool(m['on']))

    def _cmd_map_check(self, m):
        if self.job is not None or self.sj is not None or self.pending:
            self.event('움직이는 중 — 지도 점검은 로봇이 선 뒤에')
            return
        self._map_check('지도 점검')

    def _cmd_shaper(self, m):
        self._set_shaper(m['method'], m['rows'])

    def _cmd_speed(self, m):
        self._set_speed(float(m['scale']))

    def _cmd_objects(self, m):
        was = self.t_go
        self.t_go = self.clock.now() if m.get('go') else None
        # 기록: 마지막으로 움직인 구간 (tools.rt_report·rt_tools: 물체 위치 = obstacle_pose(t − t_go), t_go_end 뒤는 없음). [2026-10-26] 전엔
        #  치운 뒤 저장이 t_go = None 으로 덮어써 보고서의 움직이는 물체 거리·닿음이 비었음 (rt_run.save 가 이제 이 값을 둠)
        if self.t_go is not None:
            self.rec['t_go'], self.rec['t_go_end'] = self.t_go, None
        elif was is not None:
            self.rec['t_go_end'] = self.clock.now()
        self.event('시험 물체 움직이기 시작' if self.t_go is not None else '시험 물체 치움')

    def _cmd_map_reset(self, m):
        if self.job is not None or self.sj is not None:
            self.event('움직이는 중 — 지도 새로 쌓기는 멈춘 뒤에')
            return
        self.W.cm.start_background()
        self.event('지도 새로 쌓기')

    # ---------------- 동작 ----------------
    def _brake_point(self):
        # [D-1 ㄱ] 실시간 회피면 지키기 추종기와 같은 (배율) 한계로 — 전체 한계로 잡은 자리는 느린 추종기가 지나쳤다 돌아옴 (10-26 검토).
        #  배율 바꾸기를 먼저 (다음 주기에 줄어든 한계와 어긋나지 않게 — 독립 검토 LOW)
        if self.realtime:
            self._update_rt_scale(self.stream.cmd)
        _, amax, jmax = self._track_lim()
        F, _ = brake_fine(*self.stream.cmd, amax, jmax)
        return (np.array(F[-1, 0], float) if len(F) else np.array(self.stream.cmd[0], float)), len(F) * self.g

    def _stop(self, why='정지'):
        """성형 거쳐 감속: 지금 명령 상태에서 저크 한계로 서는 자리를 MPC 지키기 목표로 (명령은 그 자리로 감속, 성형기를 거침).
        조그 (Servo) 중이면 서버 시간표는 쉬는 중 (조그 전 자리) 이라 쓰면 안 됨 → Servo 를 멈춤."""
        if self.sj is not None:
            return self._servo_halt(why)
        self._live_wait()
        self._back = None
        q_stop, t_brk = self._brake_point()
        self.mpc.resync(q_stop, restart=True)
        self.mpc.hold(q_stop)
        self.settle_until = self.clock.now() + t_brk + self.kernel.duration_s + SETTLE_PAD_S
        self.job_goal = None
        self.event(f'{why}: 성형 거쳐 감속 정지')

    def _halt(self, why):
        """명령 멈춤 (서보 끄기·알람 해제 전): 비상정지 상태로 (서 있으면 감속 없이) — 서 있을 때 메시지를 안 보냄. 감속이 끝날 때까지 기다림."""
        if not self.estopped:
            self._estop(why)
        self.clock.sleep_until(self._estop_until)

    def _rebase(self):
        """서보를 켠 뒤: R2 가 명령을 지금 위치로 맞췄으므로 명령 시간표·성형 기억·MPC 도 지금 위치 (엔코더) 에서 새로 시작."""
        time.sleep(REBASE_WAIT_S)
        self._restart_at(self.robot.measured(), t_done=self.clock.now())

    def _set_servo(self, on):
        if self.ideal:
            self.event('이상적 시뮬레이션에는 서보가 없음 (WMX 시뮬레이터·실물에서)')
            return
        if on == self.servo_on:
            self.event(f'서보가 이미 {"켜져" if on else "꺼져"} 있음')
            self.link.send({'type': 'servo', 'on': self.servo_on})
            return
        if not on and (self.job is not None or self.sj is not None or self.pending):
            # [2026-10-26 WMX 시험] 처음엔 '명령이 서 있나' (_standing) 로 막았는데, 실시간 회피로 지키는 중에는 MPC 가 계속 조금씩
            #  다듬어 늘 '움직이는 중' 이 됐음 → 일·조그만 막고, 지키는 중이면 _halt 가 한계 안 감속으로 세운 뒤 끔
            self.event('움직이는 중 — 서보를 끄기 전에 [정지]')
            return
        self._halt('서보 끄기' if not on else '서보 켜기')
        self.event(f'서보 {"켜는" if on else "끄는"} 중 (R2 하드웨어 {"활성" if on else "비활성"})')
        ok, info = self.robot.set_servo(on)
        if not ok:
            self.event(f'서보 {"켜기" if on else "끄기"} 실패: {info}')
        else:
            self.servo_on = on
            if on:
                self._rebase()
            self.event(f'서보 {"ON" if on else "OFF"} (R2 하드웨어 {info})' + (' — 움직이려면 명령을 주거나 실시간 회피를 켬' if on else ''))
        self.link.send({'type': 'servo', 'on': self.servo_on})

    def _clear_alarm(self):
        """알람 해제 = 하드웨어를 다시 켬 (R2 활성 = 알람 해제 → 서보 ON → 명령을 지금 위치로). R2 오류로 멈춘 뒤에도 씀 (오류 표시 지움)."""
        if self.ideal:
            self.event('이상적 시뮬레이션에는 알람이 없음')
            return
        if self.job is not None or self.sj is not None or self.pending:
            self.event('움직이는 중 — 알람 해제는 멈춘 뒤에')
            return
        self._halt('알람 해제')
        self.event('알람 해제: R2 하드웨어 다시 켜기 (끔 → 켬)')
        self.robot.set_servo(False)                     # 오류 상태면 실패해도 됨 (켜기가 다시 설정함)
        ok, info = self.robot.set_servo(True)
        if ok:
            self.servo_on = True
            self.r2_fault, self._fault_done = None, False
            self.robot.t_js_mono = time.monotonic()         # (끊김 감시를 지금부터 — /joint_states 가 다시 올 때까지)
            self._rebase()
            self.event(f'알람 해제 → 서보 ON (R2 하드웨어 {info})')
        else:
            self.servo_on = False
            self.event(f'알람 해제 실패: {info} — 시스템을 다시 켜기')
        self.link.send({'type': 'servo', 'on': self.servo_on})

    def _estop(self, why='비상정지'):
        """즉시 멈춤 (성형 건너뜀): 로봇이 지금 따라가는 명령 (성형된 y) 의 보낼 수 있는 첫 스탬프 상태에서 가속·저크 한계로 바로 감속
        (rt_motion.brake_fine, 성형기를 안 거침) → 그 메시지를 보냄. 성형 기억 = 선 자세 (새로 시작), 실시간 회피 끔, 서보는 켜진 채.
        [2026-10-26 2 차] 1 차는 이상적 로봇을 그 자리에 바로 세웠음 — WMX 로는 명령이 끊기면 CyclicBuffer maxAcc 에 걸리므로
        한계 안에서 끊김 없이. 감속이 끝날 때까지 새 명령 시간표를 시작하지 않음 (_estop_until).
        조그 (Servo) 중이면 서버 시간표가 조그 전 자리라 쓰면 튐 → Servo 를 멈추고 선 뒤 비상정지 표시 (_servo_exit)."""
        if self.sj is not None:
            self.pending.clear()
            return self._servo_halt(why, estop=True)
        self._live_wait()
        self._back = None
        self.pending.clear()
        st = self.stream
        now = self.clock.now()
        q_stop, t_stop = np.array(st.tail_end, float), max(now, st.tail_done)
        if st.stamp is not None and st.msg is not None and now < st.tail_done:
            S, k = st.stamp + self.dt, st.k + self.n_dt
            while S - self.clock.now() < self.c.stream.lead_s:         # 보낼 시간이 남은 첫 격자 스탬프
                S, k = S + self.dt, k + self.n_dt
            y0 = st.y_state(k)
            if float(np.max(np.abs(y0[1]))) > 1e-9 or float(np.max(np.abs(y0[2]))) > 1e-9:
                F, _ = brake_fine(*y0, self.mpc.amax, self.jmax)
                Y = np.concatenate([np.stack(y0)[None], F]) if len(F) else np.stack(y0)[None]
                last = len(Y) - 1
                idx = list(range(0, last, self.step_n)) + [last]
                pts = [(round(i * self.g, 6), Y[i, 0], Y[i, 1], Y[i, 2]) for i in idx]
                pts[-1] = (pts[-1][0], Y[last, 0], np.zeros_like(Y[last, 0]), np.zeros_like(Y[last, 0]))
                st.set_stamp(S, k)
                msg, t0, _ = st.message(pts)
                self.robot.send(t0, msg, Y[min(self.n_dt, last), 0])
                self._log_sent(S, pts)
                # 실행된 u: 실행 중이던 메시지는 이 스탬프까지만 (뒤는 성형을 건너뛴 감속 — 성형 확인에서 뺄 구간)
                st.finish_at(k)
                q_stop, t_stop = Y[last, 0].copy(), S + pts[-1][0]
                self.estop_windows.append((round(S, 4), round(t_stop, 4)))
        # [R-1 S2] 엔진의 다시 시작 (_restart_at): 시간표·성형 기억·MPC 를 선 자세에서 + 보낸 감속 메시지를 바로 '보낸 명령' 기록에
        self._restart_at(q_stop, t_done=t_stop)
        self._estop_until = t_stop
        if self.ideal:
            self.robot.rest(q_stop)
        self.estopped = True
        self.settle_until = 0.0
        self.job_goal = None
        if self.realtime:
            self.realtime = False
            self.link.hello['realtime'] = False
            self.link.send({'type': 'realtime', 'on': False})
        self.event(f'{why}: 성형 건너뛰고 한계 안에서 바로 감속 ({max(0.0, t_stop - now) * 1000:.0f} ms 뒤 섬), 실시간 회피 끔')

    def _resume(self, why):
        self.estopped = False
        self.event(f'비상정지 풀림 ({why})')

    def _set_realtime(self, on):
        if on == self.realtime:
            return
        self.realtime = on
        self.link.hello['realtime'] = on
        self.link.send({'type': 'realtime', 'on': on})
        if on:
            self.W.map_off = False                      # [D-7] 실시간 회피는 늘 카메라 지도로 (켜기 전 지도 점검도 지도로 함)
            self.cam_next = self.clock.now()            # 멈춰 있던 동안의 영상은 건너뜀
            if self.estopped:
                self._resume('실시간 회피 켬')
            if self.job is None and not self.pending and self.sj is None and self._standing():
                # [2026-10-27 D-1] 쉬던 시간표를 그대로 이으면 첫 주기가 옛 격자로 따라잡아 '주기 늦음 → 7850 ms' 가 남았음
                #  (해는 없었지만 기록이 헷갈림) → 그 자리에서 새로
                self._restart_here()
                self.rt_scale = self.speed_req
            self.event(f'실시간 회피 켬 (지도 새로 봄, 서 있을 때도 피함 · 로봇 명령 속도·가속 한계 × 배율 {self.rt_scale:.2f})')
        else:
            self.settle_until = self.clock.now() + SETTLE_OFF_S + self.kernel.duration_s
            if self.job is None and self.sj is None:
                # [2026-10-27 D-1 독립 검토 HIGH] 그 자리에서 섬: 전엔 끄면 직접 추종기가 MPC 지키기 목표 (비키기 전 지키던 자리) 를 전체
                #  한계로 쫓아, 비키던 로봇이 멈춘 지도로 옛 자리까지 빠르게 되돌아갔음. 이제 끈 뒤의 한계 (전체 — 직접 추종기와 같음) 로
                #  서는 자리를 지키기 목표로 (_stop 과 같은 계산, 일 도중이면 _handle 이 그 일을 [정지]).
                q_stop, t_brk = self._brake_point()
                self.mpc.resync(q_stop, restart=True)
                self.mpc.hold(q_stop)
                self.settle_until = max(self.settle_until, self.clock.now() + t_brk + self.kernel.duration_s + SETTLE_PAD_S)
            self.event('실시간 회피 끔 (지도 멈춤, 서 있을 때 명령 안 보냄 · 그 자리에서 섬)')

    def _track_lim(self):
        """[2026-10-27 디버깅 D-1 ㄱ] 실시간 회피 (MPC 당근 + 한계 안 추종기) 의 로봇 명령 한계 = 배율 1 한계 × 속도 배율 (속도·가속).
        실물에서 배율 0.05 인데 실시간 회피가 2.4 rad/s · 가속 한계 100 % 로 움직였음 — 배율은 계획 경로만 늘렸고 추종기는 배율 1 한계.
        가속도 × 배율 (경로 고르게 늘리기의 × 배율² 이 아님): 0.05 에서 × 배율² = 0.018 rad/s² 면 0.19 rad/s 에서 서는 데 54° 가 들어
        피하기·멈추기가 안 됨 (× 배율 이면 2.6°). 늘린 경로 (speed_stretch — [D-5] 목표 이동도 같은 한계 × 배율 안) 는 따라감. 저크는 그대로 (가속을
        빨리 한계 안으로 되돌리게 — 안전 검사 감속은 전체 한계라 가속이 이 한계 밖일 수 있음). MPC 안 한계는 그대로: cuRobo 가 만들 때 복사해
        두어 (cost_cspace_cfg.set_bounds) 실행 중에 못 바꿈 → 당근은 빨리 움직일 수 있고 로봇 명령만 배율 한계. 배율은 일 도중엔 그 일의
        배율, 서 있을 때는 GUI 배율. 줄일 때는 지금 명령 속도·가속이 새 한계 안일 때만 (추종기가 넘는 속도를 그 자리에서 잘라 명령이 튀지
        않게 — _update_rt_scale). 실시간 회피 끔 (경로 그대로 따라가기·지키기) 은 배율 1 한계 그대로 (경로가 이미 늘어 있음).
        [2026-10-27 2 차 시험] 메시지 멈춤 꼬리 (rt_run._u_tail — 다음 메시지가 늦으면 로봇이 그것으로 섬) 도 이 한계: 전체 한계면 늦은
        메시지마다 배율 0.2 로 가다가 가속 한계 100 % 로 섰다 다시 출발 (배율 0.2 [반복]+물체 46 s 에서 늦은 메시지 40 번, 가속 최대
        1.005 × 한계). 안전 검사 감속 (_u_brake) 과 비상정지는 전체 한계 그대로 (빨리 섬)."""
        if not self.realtime:
            return self.mpc.vmax, self.mpc.amax, self.jmax
        return self.mpc.vmax * self.rt_scale, self.mpc.amax * self.rt_scale, self.jmax

    def _update_rt_scale(self, s0):
        """실시간 회피 주기 시작 (s0 = 지금 명령 상태) 에 배율 바꾸기 (_track_lim 설명): 일 도중엔 그 일의 배율, 서 있을 때는 GUI 배율.
        올릴 때는 바로, 줄일 때는 지금 명령 속도·가속이 새 한계 안일 때만."""
        want = self.speed if self.job is not None else self.speed_req
        if abs(want - self.rt_scale) < 1e-9:
            return
        v, a = (np.asarray(x, float) for x in s0[1:])
        # 가속 중이면 가속을 저크 한계로 0 까지 줄이는 동안 속도가 a²/(2·저크) 더 늘어남 (독립 검토 LOW) — 그것까지 새 한계 안일 때만
        v_peak = np.abs(v) + np.maximum(a * np.sign(v), 0.0) ** 2 / (2.0 * self.jmax)
        if want > self.rt_scale or (np.all(v_peak <= want * self.mpc.vmax + 1e-9) and np.all(np.abs(a) <= want * self.mpc.amax + 1e-9)):
            self.event(f'실시간 회피 속도 배율 {self.rt_scale:.2f} → {want:.2f} (로봇 명령 속도·가속 한계 × {want:.2f})')
            self.rt_scale = want

    def _restart_here(self):
        """서 있을 때 (명령이 멈춘 상태) 명령 시간표·성형 기억·MPC 를 지금 명령 자리에서 새로 시작 (_rebase 와 같음 — 엔코더 대신 명령 자리라
        명령이 안 튐)."""
        self._restart_at(np.array(self.stream.tail_end, float), t_done=self.clock.now())

    # ---------------- [2026-10-27 디버깅 D-1 ㄴ] 실시간 회피 켜기 전 지도 점검 ----------------
    def _near_now(self, q):
        """지금 지도에서 자세 q 의 로봇: 겉면 5.5 cm (팔 지우기 + 1.5 칸 = 경계 채우기가 걸리는 거리, cr3a_mapper._fill_mask_boundary) 안
        지도 칸 B (칸 가운데 [n, 3] m), 그중 가장 가까운 칸의 구 겉면 거리·링크, 지도만의 실제 거리·MPC 지도 거리 (바닥 면 뺌 — 바닥은
        MPC 장면에 따로 있고 책상 가까이 일하는 자세를 막지 않게). [R-1 S2] 엔진 공개 API (World.spheres·clearances(floor=False)·
        sphere_link_names) 로."""
        import torch
        cmap = self.W.cm
        vs, mask = float(cmap.s.esdf_voxel_size), float(cmap.s.segment_distance_threshold)
        Q = np.asarray(q, float)[None]
        cl = self.W.clearances(Q, ('raw', 'mpc'), floor=False)
        out = {'raw': float(cl['raw'][0]), 'mpc': float(cl['mpc'][0]), 'B': np.zeros((0, 3)), 'd_b': None, 'link': None}
        P = cmap.new_voxel_centers()
        if P.shape[0] == 0:
            return out
        c, r = self.W.spheres(Q)                    # (기구학 버퍼를 다시 쓰므로 거리 계산 뒤에 — 아래에서 바로 씀)
        c, r = c[0].float(), r[0].float()
        keep = r > 0                                # 반지름 0 이하 = 안 쓰는 구 (전엔 거리 9 m 로 뺌)
        if not bool(keep.any()):
            return out
        # [2026-10-28 R-1] 엔진 cr3a_mapper.min_dist (행렬곱 없음 — TF32 와 상관없이 정확): 칸마다 구 겉면까지 최소 거리·그 구 번호
        dmin, i = cr3a_mapper.min_dist(P, c[keep], r[keep], return_index=True)
        link = torch.nonzero(keep).view(-1)[i]     # 쓰는 구 안 번호 → 전체 구 번호 (sphere_link_names 순서)
        sel = dmin <= mask + 1.5 * vs
        if bool(sel.any()):
            out['B'] = P[sel].cpu().numpy()
            k = int(torch.argmin(dmin))
            names = self.W.sphere_link_names()
            out['d_b'] = float(dmin[k])
            out['link'] = names[int(link[k])] if int(link[k]) < len(names) else None
        return out

    def _map_check(self, label, turning_on=False):
        """[2026-10-27 디버깅 D-1 ㄴ, 사용자 "ㄴ 수정 … 확인하면서 ㄷ 을 내가 직접"] 실물 첫 실시간 회피 (12:17~12:22) 에서 팔 옆 3~5 cm 의
        실제 깊이 점 (공구 케이블·팔 가장자리) 이 경계 채우기로 MPC 지도 '닿음' (−6~−22 mm) 이 되어, 켜자마자 MPC 가 최고 속도로 허공을
        피했음. 그래서 켜기 전에 본다: 실시간 회피 끔이면 (지도 멈춤) 지도를 CHECK_S 동안 새로 쌓으며 (앞 CHECK_SKIP_S 는 옛 칸이 빠지는
        시간이라 뺌) 영상마다, 켜져 있으면 지금 지도 한 번 — 로봇 겉면 5.5 cm 안 지도 칸 (= 경계 채우기가 걸려 MPC 가 '닿음' 으로 봄)
        이 한 번이라도 있거나 MPC 지도 거리가 0 아래면 실패. 결과 칸은 GUI 3D 에 자홍 점 (type 'near', 통과면 지움). 로봇은 안 움직임
        (서보가 꺼져 있어도 됨 — 케이블 정리 확인용 [지도 점검]). 반환: 통과?"""
        if not self.ready:
            self.event(f'{label}: 아직 준비 중 (지도 배경 모으기) — 잠시 뒤 다시')
            return False
        if not self.realtime and not self._standing():
            # 지도를 새로 쌓는 동안 (1.2 s) 주기가 멈춤 → 명령이 아직 서는 중이면 앞 메시지 꼬리 (빨리 섬) 로 서 버림 [2026-10-27 2 차 시험]
            self.event(f'{label}: 로봇이 아직 서는 중 — 선 뒤에 다시')
            return False
        self.W.map_off = False                          # [D-7] 점검은 카메라 지도로 (다음 이동은 _choose_map 이 다시 정함)
        q = np.asarray(self.robot.measured(), float)
        res = []
        t0 = time.monotonic()
        if self.realtime:
            res.append(self._near_now(q))
        else:
            self._refreshing = True
            try:
                if self.real_cam is not None:
                    while (left := CHECK_S - (time.monotonic() - t0)) > 0:
                        if not self._real_frame():
                            self.real_cam.wait_depth(self._n_frame, left)     # [R-1 ⑨] 새 영상이 오면 바로 (전엔 5 ms 마다 물어봄)
                        elif time.monotonic() - t0 >= CHECK_SKIP_S:
                            res.append(self._near_now(q))
                else:
                    # 가상 카메라 (시뮬레이션): 같은 시간만큼의 영상을 지금 렌더. rt_run._frames_until 에 1.2 s 밀린 차례를 주면 0.5 s
                    #  넘게 밀린 영상은 건너뛰어 (cam_max_lag_s) 1~2 장만 쌓았음 (10-27 시험 — 움직이는 물체가 깜빡임 거르기에 막혀 안 보임)
                    for _ in range(int(round(CHECK_S * self.c.map.cam_fps))):
                        self.W.frame(q, self._boxes(self.clock.now()))
                    res.append(self._near_now(q))
            finally:
                self._refreshing = False
            self.cam_next = max(self.cam_next, self.clock.now())
            # 서 있는 채로 주기가 1.2 s 멈췄으므로 (도착 직후의 '서는 중' 주기 사이에 부르면) 옛 격자로 따라잡지 않게 그 자리에서 새로
            #  [2026-10-27 2 차 시험: 도착 직후 [지도 점검] 뒤 '주기 늦음 → 150 ms']
            self._restart_here()
        vs = float(self.W.cm.s.esdf_voxel_size)
        if not res or (self.real_cam is not None and not self.realtime and len(res) < CHECK_MIN_FRAMES):
            # [독립 검토 MEDIUM] 지도에 쌓인 영상만 셈 — 몇 장 안 되면 (가려졌거나 깊이가 대부분 비었음) 옛 지도로 판정하지 않음
            text = (f'{label} 걸림: {CHECK_S:.1f} s 동안 지도에 쌓인 깊이 영상이 {len(res)} 장뿐 (영상이 안 오거나 카메라가 가려짐) — 점검 못 함'
                    + (' → 실시간 회피 안 켬' if turning_on else ''))
            self.event(text)
            self.link.send({'type': 'near', 'P': np.zeros((0, 3), np.float16), 'vs': vs, 'ok': False, 'text': text, 'q': q.tolist()})
            return False
        pts = {}
        for x in res:
            for p in x['B']:
                pts[tuple(np.round(p, 4))] = p
        B = np.array(list(pts.values())) if pts else np.zeros((0, 3))
        bad = [x for x in res if len(x['B']) or x['mpc'] < 0.0]
        worst = min(res, key=lambda x: x['mpc'])
        near = [x for x in res if x['d_b'] is not None]
        nb = min(near, key=lambda x: x['d_b']) if near else None
        raw = min(x['raw'] for x in res)
        ok = not bad
        if ok:
            text = (f'{label} 통과: 로봇 겉면 5.5 cm 안 지도 점 없음 (영상 {len(res)} 장, 가장 가까운 지도 점 '
                    + (f'{raw * 1000:.0f} mm' if raw < 1.0 else '1 m 넘음')
                    + (f' — MPC 반응 거리 {self.c.dist.act_new * 100:.0f} cm 안이라 조금 비킬 수 있음' if raw < self.c.dist.act_new else '') + ')')
        else:
            text = (f'{label} 걸림: 로봇 겉면 5.5 cm 안 지도 점 {len(B)} 칸 (영상 {len(bad)}/{len(res)} 장'
                    + (f', 가장 가까운 {nb["d_b"] * 1000:.0f} mm · {nb["link"] or "?"}' if nb else '')
                    + f') — MPC 지도 거리 {worst["mpc"] * 1000:.0f} mm (경계 채우기로 0 근처·아래 = 로봇이 물체에 닿았거나 안으로 보여, 켜면 MPC 가'
                      f' 빠져나가려 움직임)'
                    + (' → 실시간 회피 안 켬' if turning_on else '') + '. 3D 화면 자홍 점 = 그 자리 (케이블·물체를 치운 뒤 다시)')
        self.event(text)
        self.rec.setdefault('checks', []).append({
            't': round(self.clock.now(), 3), 'label': label, 'ok': ok, 'n_frames': len(res), 'n_bad': len(bad),
            'q': np.round(q, 6).tolist(), 'B': np.round(B, 3).tolist(), 'mpc_mm': round(worst['mpc'] * 1000, 1),
            'raw_mm': round(min(raw, 9.0) * 1000, 1), 'd_b_mm': None if nb is None else round(nb['d_b'] * 1000, 1),
            'link': None if nb is None else nb['link'], 'secs': round(time.monotonic() - t0, 2)})
        self.link.send({'type': 'near', 'P': B.astype(np.float16), 'vs': vs, 'ok': ok, 'text': text, 'q': q.tolist()})   # q: GUI 가 로봇이 움직이면 지움
        return ok

    def _set_speed(self, s):
        """GUI 속도 배율 → 다음 일 (목표 이동·반복·한 칸 조그) 부터 (움직이는 중이면 지금 일은 그 배율 그대로 끝까지)."""
        s = float(np.clip(s, SPEED_MIN, 1.0))
        if abs(s - self.speed_req) < 1e-9:
            return
        self.speed_req = s
        self.link.hello['speed'] = s
        self.link.send({'type': 'speed', 'scale': s})
        busy = self.job is not None or bool(self.pending)
        self.event(f'속도 배율 {s:.2f} ({"지금 일이 끝난 뒤 " if busy else ""}다음 이동부터: 로봇 명령 속도·가속 한계 × {s:.2f} — 목표 이동은 '
                   f'계획 경로를 그 한계 안에 드는 만큼 고르게 늘림, 실시간 회피도 같은 한계 · 저크 한계·입력성형은 그대로)')

    def _post_solve(self, path, q0, q1):
        """속도 배율 (rt_server 머리말): 계획한 모든 경로 (멈춰 계획·멈추지 않는 다시 계획 — 계획기 스레드에서도 불림) 를 속도·가속 한계 ×
        배율 안에 드는 만큼 고르게 늘림 (같은 길, scale_path — 한계 = 계획기 한계와 같은 MPC vmax·amax, 켤 때 확인. 점 사이가 넓은 아주 긴
        이동은 같은 곡선 위에 점을 다시 찍음). [R-1 S2] 엔진의
        계획 뒤 고치기 자리 (전엔 World.plan_solve 를 인스턴스에 바꿔 끼움). [2026-11-02 D-5] 전엔 시각을 1/배율 로 (속도 × 배율 · 가속 × 배율²)."""
        s, t, q = self.speed, np.asarray(path[0], float), np.array(path[1], float)
        # cuRobo 끝점은 목표에서 조금 어긋남 → 끝점을 목표로 (1 mrad 안일 때만). [2026-10-26 WMX 4 차] 안 맞추면 도착 뒤 지키기가 그 차이만큼
        #  다시 천천히 움직여 (→ B 다리) 성형 전 명령 시간이 0.37~0.44 s 길게 잡힘
        err = float(np.abs(q[-1] - np.asarray(q1, float)).max())
        if err < PLAN_END_SNAP_RAD:
            q[-1] = np.asarray(q1, float)
        T = float(t[-1] - t[0])
        t, q, k, vu, au = scale_path(t, q, s, self.mpc.vmax, self.mpc.amax)
        self._scaled.append((s, round(T, 4), round(float(t[-1] - t[0]), 4), round(err * 1000, 4), round(vu, 4), round(au, 4), len(t)))
        return t, q

    def _leg_timeout_s(self):
        """느린 경로 (배율 < 1) 는 움직임 시간 초과를 1/배율 로 — 계획 실패 되풀이 시간 초과는 그대로 (엔진 _plan_until 이 task 값 그대로 —
        독립 검토: 0.05 배면 500 s 동안 되풀이). [R-1 S2] 전엔 다리 동안 self.c 를 바꿔 끼우고 _plan_until 에서 되돌림."""
        t = self.c.task.leg_timeout_s
        return t / self.speed if self.speed < 1.0 else t

    def leg(self, label, goal):
        """다리 기록에 배율·늘린 경로 (시간 초과는 _leg_timeout_s). 실시간 회피 끔이면 다리마다 계획 직전에 지도를 새로 봄 (_refresh_map).
        [2026-10-27 ④] 실제 카메라 깊이 영상이 CAM_STALE_S 넘게 안 왔으면 옛 지도라 이 다리는 안 함 (전엔 경고 없이 옛 지도로 계획,
        [반복] 은 일을 시작할 때 한 번만 새로 봄).
        [2026-11-02 D-7 사용자 "카메라가 끄기 상태라면 장애물 고려하지 않는 모션을 생성하도록"] 고정 카메라가 꺼져 있으면 (노드 없음 + 영상
        없음 — _cam_off) 거절 대신 카메라 지도 없이 계획·이동 (_choose_map). 노드가 도는데 영상이 안 오면 (고장) 그대로 거절."""
        if not self.realtime and not self._choose_map(label):
            self._refresh_map()
            if self.real_cam is not None and self._cam_age() > CAM_STALE_S:
                self.event(f'{label}: 카메라 영상이 {self._cam_age_text()} → 지도를 새로 못 봐 안 움직임 (카메라 확인 뒤 다시 — '
                           f'카메라 세팅 탭에서 카메라를 끄면 장애물을 안 보고 움직임)')
                self.label = label
                L = {'label': label, 't0': self.clock.now(), 'plans': [], 'ok': False, 'live_ok': 0, 'live_fail': 0,
                     'why': '카메라 영상 끊김', 'speed': self.speed, 'scaled': []}
                return self._end_leg(L)
        n0 = len(self._scaled)
        L = super().leg(label, goal)
        L['speed'] = self.speed
        L['scaled'] = self._scaled[n0:]
        L['map_off'] = self.W.map_off           # [D-7] 카메라 지도 없이 간 다리
        return L

    def _standing(self):
        """명령이 서 있나: 일·조그 없음 + 다음 스탬프의 성형 전 명령 속도·가속 0 + 그 앞 성형 길이 동안 u 가 같은 자리 (= 성형된 y 도
        섬). [2026-10-26 2 차] 1 차는 측정값 (로봇 위치 − 명령) 도 봤으나 실물은 서보 잔떨림이 있어 명령만 봄."""
        if self.job is not None or self.sj is not None or self.pending or self.clock.now() < self._estop_until:
            return False
        st = self.stream
        q, v, a = st.cmd
        if float(np.max(np.abs(v))) > STAND_V or float(np.max(np.abs(a))) > STAND_A:
            return False
        if st.L > 1 and st.stamp is not None:
            P = st.past(st.k + st.n_dt)[:, 0]              # (다음 스탬프 앞 성형 길이 L − 1 칸)
            if len(P) and float(np.max(np.abs(P - q))) > STAND_Q:
                return False
        return True

    def _set_shaper(self, method, rows):
        rows = [(bool(u), float(f)) for u, f in rows]
        if not self._standing():
            self.event('입력성형: 로봇이 움직이는 중이라 적용 안 함 — 멈춘 뒤 다시 [적용]')
            self.link.send({'type': 'shaper_rejected'})
            return
        K = shaper_def.kernel(method, rows)
        st = self.stream
        q = np.array(st.tail_end, float)
        self._flush_sent()
        self.old_runs.append({'label': shaper_def.label(*self.shaper_cfg), 'h': st.h.copy(), 'grid_s': st.g,
                              'segments': st.executed(), 'until': round(self.clock.now(), 4)})
        self.kernel = K
        self.cp = self.c.guard.cp_thr > 0 and K.L > 1
        self.stream = Stream(self.c.stream, self.dt, q, h=K.h, grid_s=K.grid_s, mem=self.mem_n)
        self.stream.tail_done = st.tail_done
        self.robot.rest(q)
        self.mpc.start(q)
        self.shaper_cfg = (method, rows)
        self.link.hello['shaper'] = [method, rows]
        self.link.send({'type': 'shaper', 'cfg': [method, rows]})
        self.event(f'입력성형 적용: {shaper_def.label(method, rows)} — 임펄스 {len(K.A)} 개 · '
                   f'지연 시간 {shaper_def.ms(K.duration_s)} ms')

    # ---------------- 조그: MoveIt Servo ----------------
    # [2026-10-27 사용자 "조그에서는 입력성형 빼고 무브잇만으로 조그하도록 하자. 목표 이동만 입력성형 적용"] 조그 동안 서버는 R2 에 명령을
    #  안 보내고 GUI 조그를 Servo 로 넘김 (Servo → R2 팔 제어기, 입력성형 없음). 조그 하나 = self.sj (JogState):
    #  wait (서버 마지막 메시지의 멈춤 꼬리가 끝나기를) → run (명령 보냄) → halt (0 속도 SERVO_HALT_S → 끊음 → Servo 가 조용하고 관절이
    #  섬) → _servo_exit (서버 명령 시간표·성형 기억·MPC 를 그 자리에서 새로). 멈추는 중 (뗌·도착) 에 새 조그가 오면 그 자리에서 이어 감.
    #  조그 시각은 모두 time.monotonic ([R-1 ⑤] — Servo 상태·팔 제어기 토픽 받은 시각도 같은 시계)
    def _servo_refuse(self):
        if self.ideal:
            return '이상적 모드에는 조그가 없음 (조그 = MoveIt Servo, R2 와 함께 — WMX 시뮬레이터·실물에서)'
        if not self.ready:
            return '아직 준비 중 — 조그는 준비가 끝난 뒤에'
        if not self.servo_ok:
            return 'MoveIt Servo 가 안 떠 있음 — 조그 안 됨 (기록 폴더 servo_launch.log)'
        if self.job is not None or self.pending:
            return '움직이는 중 — 끝나거나 [정지] 한 뒤에 조그'
        return None

    def _say_once(self, text):
        """누르는 동안 명령은 50 ms 마다 와서 같은 거절을 SAY_ONCE_S 에 한 번만."""
        last, t = self._said
        now = time.monotonic()
        if text != last or now - t > SAY_ONCE_S:
            self._said = (text, now)
            self.event(text)

    @staticmethod
    def _jog_speed(m, joint, ax):
        return math.radians(float(m['v_deg'])) if joint else (float(m['v']) / 1000 if ax < 3 else math.radians(float(m['v'])))

    def _servo_cmd(self, m):
        """GUI 조그 명령 → Servo 조그 시작 (누르는 동안 같은 조그면 속도·시각만 새로)."""
        k = m['cmd']
        hold = k in ('jog_hold', 'jog_cart_hold')
        joint = k in ('jog', 'jog_hold')
        ax = int(m['joint']) if joint else CART_AXES.index(m['axis'])
        frame = None if joint else ('tool' if m.get('frame') == 'tool' else 'base')
        sj = self.sj
        if (sj is not None and hold and sj.mode == 'hold' and sj.phase != 'halt'
                and (sj.joint, sj.ax, sj.frame) == (joint, ax, frame)):
            sj.v, sj.t = self._jog_speed(m, joint, ax), time.monotonic()
            return
        if hold and self._hold_block:
            return self._say_once('조그를 멈췄음 — 단추를 뗐다가 다시 누르면 움직임')
        cont = sj is not None and sj.phase == 'halt' and sj.ran and not sj.hard
        if sj is not None and not cont:
            return self._say_once('조그 중 — 멈춘 뒤에 다시')
        why = self._servo_refuse()
        if why:
            return self._say_once(why)
        if not self.robot.servo_type(0 if joint else 1):
            return self._say_once('MoveIt Servo 명령 종류를 못 바꿈 — 조그 안 함')
        if self.estopped:
            self._resume('조그')
        now = time.monotonic()
        name = f'J{ax + 1}' if joint else f'{CART_AXES[ax]} ({"공구" if frame == "tool" else "받침"} 기준)'
        unit = '°' if (joint or ax >= 3) else ' mm'
        new = JogState(mode='hold' if hold else 'step', joint=joint, ax=ax, frame=frame, name=name, t0=now, t=now,
                       v_cmd=0.0 if joint else (np.zeros(3), np.zeros(3)))
        if hold:
            new.v = self._jog_speed(m, joint, ax)
            self.event(f'조그 {name} {float(m["v_deg"] if joint else m["v"]):+g}{unit}/s 누르는 동안 (MoveIt Servo · 입력성형 없음)')
        else:
            d = float(m['delta_deg'] if joint else m['delta'])
            new.d = math.radians(d) if (joint or ax >= 3) else d / 1000
            self.event(f'조그 {name} {d:+g}{unit} 한 칸 (MoveIt Servo · 입력성형 없음, 최고 속도 × 배율 {self.speed_req:.2f})')
        self.sj = new
        self.robot.jt_watch = True                  # [R-1 ⑭] 조그 동안만 팔 제어기 토픽을 풂 (_servo_exit 가 끔)
        self.label = f'조그 {name}'
        if cont:                                    # Servo 가 아직 맡고 있음 (서버는 안 보내는 중) → 기다림 없이
            new.q0 = sj.q0
            self._servo_go(new)

    def _servo_go(self, sj):
        """조그 시작: 지금 자리 (팔 제어기 기준 위치, 없으면 엔코더) 에서. 한 칸이면 목표를 정함."""
        q = self.robot.jtc_ref()
        q = np.asarray(self.robot.measured(), float) if q is None else q
        sj.phase, sj.ran, sj.t_run, sj.q_start = 'run', True, time.monotonic(), q
        if sj.q0 is None:
            sj.q0 = q
        if sj.mode != 'step':
            return
        s = self.speed_req
        if sj.joint:
            j, tgt = sj.ax, q[sj.ax] + sj.d
            lo, hi = self.mpc.qlim
            if not lo[j] + SERVO_LIMIT_MARGIN <= tgt <= hi[j] - SERVO_LIMIT_MARGIN:
                sj.ran = False
                return self._servo_halt(f'조그 {sj.name}: 관절 한계 ({math.degrees(SERVO_LIMIT_MARGIN):.1f}° 앞) 를 넘음 — 안 움직임')
            sj.target, sj.t_max = tgt, abs(sj.d) / (STEP_V[0] * s) + STEP_TIME_PAD_S
        else:
            T0 = self._tip(q)
            sj.target = cart_step(T0, sj.ax, sj.d, sj.frame)
            sj.t_max = abs(sj.d) / (STEP_V[1 if sj.ax < 3 else 2] * s) + STEP_TIME_PAD_S

    def _servo_q(self, sj):
        """Servo 가 낸 가장 앞 명령 (팔 제어기 토픽의 마지막 궤적 끝) — 아직 없으면 시작 자리."""
        t, q = self.robot.jt_last
        return q if (q is not None and t > sj.t_run) else sj.q_start

    def _servo_vel(self, sj, v):
        """축 하나 방향 속도 v (누르는 동안 · 멈출 때 0)."""
        if sj.joint:
            return self.robot.servo_joint(sj.ax, v)
        e, z = np.eye(3)[sj.ax % 3] * v, np.zeros(3)
        self.robot.servo_twist(SERVO_TIP if sj.frame == 'tool' else SERVO_BASE, e if sj.ax < 3 else z, z if sj.ax < 3 else e)

    def _servo_send(self, sj):
        now = time.monotonic()
        if sj.mode == 'hold':
            if now - sj.t > JOG_TIMEOUT_S:                      # 뗌 (jog_release 가 못 왔거나 GUI 멈춤)
                return self._servo_halt('조그 뗌', soft=True)
            return self._servo_vel(sj, sj.v)
        # 한 칸: Servo 가 낸 최신 명령으로 남은 거리 → 속도 (최고 속도 · 끝 최소 속도 · 바꾸기 한계 안), 다 가면 멈춤
        q, s = self._servo_q(sj), self.speed_req
        if sj.joint:
            e = sj.target - q[sj.ax]
            if abs(e) <= abs(sj.v_cmd) * SERVO_COAST_S + STEP_TOL[0]:
                return self._servo_halt('한 칸 도착', soft=True)
            vmax = STEP_V[0] * s
            v = math.copysign(min(max(STEP_K * abs(e), STEP_VMIN[0]), vmax), e)
            dv = STEP_ACC * vmax * self.dt
            sj.v_cmd = float(np.clip(v, sj.v_cmd - dv, sj.v_cmd + dv))
            self.robot.servo_joint(sj.ax, sj.v_cmd)
        else:
            T = self._tip(q)
            ep, er = sj.target[:3, 3] - T[:3, 3], rotvec(sj.target[:3, :3] @ T[:3, :3].T)
            np_, nr = float(np.linalg.norm(ep)), float(np.linalg.norm(er))
            v0, w0 = sj.v_cmd
            tol_p = np.linalg.norm(v0) * SERVO_COAST_S + STEP_TOL[1]
            tol_r = np.linalg.norm(w0) * SERVO_COAST_S + STEP_TOL[2]
            if np_ <= tol_p and nr <= tol_r:
                return self._servo_halt('한 칸 도착', soft=True)
            vl, vr = STEP_V[1] * s, STEP_V[2] * s
            # 남은 것만 (다 간 쪽은 0), 크기 = 남은 거리 × K 를 끝 최소 속도 ~ 최고 속도 안으로
            vt = ep * (min(max(STEP_K * np_, STEP_VMIN[1]), vl) / np_) if np_ > tol_p else np.zeros(3)
            wt = er * (min(max(STEP_K * nr, STEP_VMIN[2]), vr) / nr) if nr > tol_r else np.zeros(3)
            v = ramp_to(vt, v0, STEP_ACC * vl * self.dt)
            w = ramp_to(wt, w0, STEP_ACC * vr * self.dt)
            sj.v_cmd = (v, w)
            self.robot.servo_twist(SERVO_BASE, v, w)
        if now - sj.t_run > sj.t_max:
            code, text, t = self.robot.servo_status
            self._servo_halt(f'조그 {sj.name}: 한 칸을 {sj.t_max:.1f} s 안에 다 못 감'
                             + (f' (MoveIt Servo: {text})' if code != 0 and t > sj.t_run else ''))

    def _servo_halt(self, why, estop=False, soft=False):
        """조그 멈추기 시작 (soft = 뗌·도착, 아니면 정지·비상정지·끊김 — 멈추는 중에 새 조그로 이어 가지 않음)."""
        sj = self.sj
        if sj is None:
            return
        if estop:
            sj.estop = True
        if not soft:
            sj.hard = True
            # 아직 누르는 중에 멈춤 (정지 명령만 오고 뗌은 안 옴 — GUI 끊김·다른 클라이언트) → 누르는 동안 명령이 계속 와도 떼기 (jog_release)
            #  전에는 다시 안 움직임. [2026-10-27 WMX 시험] 이게 없으면 멈춤이 끝나자마자 같은 명령으로 조그가 다시 시작돼 비상정지 표시도
            #  풀렸음. (GUI 의 [정지]·Esc 는 뗌을 먼저 보내고 누르는 동안 명령을 멈춤 → 이미 멈추는 중이라 막지 않음)
            if sj.mode == 'hold' and sj.phase != 'halt':
                self._hold_block = True
        if sj.phase == 'halt':
            return
        sj.phase, sj.t_halt, sj.why = 'halt', time.monotonic(), why
        if not soft:
            self.event(f'{why}: 조그 멈춤 (MoveIt Servo 가 가속 한계 안에서 감속)')

    def _servo_stopping(self, sj):
        """멈추는 중: SERVO_HALT_S 동안 0 속도 (Servo 가 가속 한계로 줄임) → 명령 끊음 → Servo 궤적이 SERVO_QUIET_S 동안 없고 관절이
        서 있으면 끝."""
        now = time.monotonic()
        if not sj.ran:
            return self._servo_exit(sj)             # Servo 는 안 움직였음 (시작 전에 멈춤)
        if now - sj.t_halt < SERVO_HALT_S:
            return self._servo_vel(sj, 0.0)
        # 관절 기록 (q_at) 은 메시지 시각 = 벽시계
        still = float(np.max(np.abs(self.robot.measured() - self.robot.q_at(time.time() - SERVO_STILL_S)))) < SERVO_STILL_RAD
        if now - max(self.robot.jt_last[0], sj.t_halt) > SERVO_QUIET_S and still:
            return self._servo_exit(sj)
        if now - sj.t_halt > SERVO_EXIT_MAX_S:
            self.event(f'조그 {sj.name}: {SERVO_EXIT_MAX_S:.0f} s 안에 Servo 가 조용해지지·관절이 서지 않음 — 그대로 끝냄 (확인 필요)')
            self._servo_exit(sj)

    def _servo_exit(self, sj):
        """조그 끝: 서버 명령 시간표·성형 기억·MPC 를 지금 자리 (팔 제어기 기준 = Servo 의 마지막 명령) 에서 새로 → 다음 목표 이동은 여기서
        성형. 비상정지였으면 비상정지 상태로 (실시간 회피 끔)."""
        self.sj = None
        self.robot.jt_watch = False
        self.settle_until = 0.0
        res = ''
        if sj.ran:
            q = self.robot.jtc_ref()
            q = np.asarray(self.robot.measured(), float) if q is None else q
            self._restart_at(q, t_done=self.clock.now())
            if sj.mode == 'step' and sj.why == '한 칸 도착':
                if sj.joint:
                    res = f' — 남은 {math.degrees(sj.target - q[sj.ax]):+.4f}°'
                else:
                    T = self._tip(q)
                    ep = (sj.target[:3, 3] - T[:3, 3]) * 1000
                    res = (f' — 남은 위치 {np.linalg.norm(ep):.3f} mm (받침 xyz {np.round(ep, 3).tolist()}) · 방향 '
                           f'{math.degrees(np.linalg.norm(rotvec(sj.target[:3, :3] @ T[:3, :3].T))):.4f}°')
            elif sj.joint:
                res = f' — {math.degrees(q[sj.ax] - sj.q0[sj.ax]):+.3f}° 움직임'
            else:
                Ta, Tb = self._tip(sj.q0), self._tip(q)
                res = (f' — 공구 끝 {np.linalg.norm(Tb[:3, 3] - Ta[:3, 3]) * 1000:.2f} mm · 방향 '
                       f'{math.degrees(np.linalg.norm(rotvec(Tb[:3, :3] @ Ta[:3, :3].T))):.3f}° 움직임')
        if sj.estop:
            self.estopped = True
            if self.realtime:
                self.realtime = False
                self.link.hello['realtime'] = False
                self.link.send({'type': 'realtime', 'on': False})
        tm = (f' [누름→명령 {sj.t_run - sj.t0:.2f} s · 명령 {sj.t_halt - sj.t_run:.2f} s]'
              if sj.ran and sj.t_halt is not None else '')
        self.event(f'조그 {sj.name} 끝 ({sj.why}){res}{tm}' + (' · 비상정지 상태 (실시간 회피 끔)' if sj.estop else ''))

    def _servo_watch(self, sj):
        """Servo 상태 (특이점·충돌·관절 한계 …) 를 조그마다 코드별 한 번 알림."""
        code, text, t = self.robot.servo_status
        if code != 0 and t > sj.t0 and code not in sj.said:
            sj.said.add(code)
            self.event(f'조그 {sj.name}: MoveIt Servo — {SERVO_CODES.get(code, "")} ({text})')

    def _servo_cycle(self):
        """조그 동안의 한 주기 (20 Hz, 서버는 R2 에 안 보냄): GUI 명령 → Servo, 멈추면 Servo 가 다 설 때까지 기다린 뒤 _servo_exit."""
        t0 = time.monotonic()
        self._poll()
        sj = self.sj
        if sj is not None:
            if sj.phase == 'wait':
                # 서버가 보낸 마지막 메시지 (멈춤 꼬리 포함) 가 끝나 선 뒤 시작 — 팔 제어기가 Servo 궤적으로 바뀌어도 튀지 않게
                if self.clock.now() >= max(self.stream.tail_done, self._estop_until) + SERVO_START_PAD_S:
                    self._servo_go(sj)
                elif time.monotonic() - sj.t0 > SERVO_WAIT_MAX_S:
                    self._servo_halt(f'서버 명령이 {SERVO_WAIT_MAX_S:.0f} s 안에 안 끝남 — 조그 안 함')
            if sj.phase == 'run':
                self._servo_send(sj)
            if sj.phase == 'halt':
                self._servo_stopping(sj)
            if self.sj is sj:
                self._servo_watch(sj)
        self.cam_next = max(self.cam_next, self.clock.now())       # 조그 동안 지도는 멈춤 (서 있을 때와 같음)
        self._publish(None)
        self._trim()
        time.sleep(max(0.0, self.dt - (time.monotonic() - t0)))

    # ---------------- 공구 끝: cuRobo 정기구학·역기구학 ----------------
    def _tip(self, q):
        """관절 q (rad) → 공구 끝 4x4 (받침 기준 m) = cuRobo 운동학의 Link6 (IK 와 같은 모델) × TIP_M."""
        return self._tips(np.asarray(q, float)[None])[0]

    def _tips(self, Q):
        """관절 [n, 6] → 공구 끝 [n, 4, 4] (cuRobo 운동학 한 번)."""
        Q = np.asarray(Q, float).reshape(-1, len(JOINTS))
        js = JointState.from_position(torch.tensor(Q, device='cuda', dtype=torch.float32), joint_names=JOINTS)
        p = self.W.kin.compute_kinematics(self.W.kin.get_active_js(js)).tool_poses.get_link_pose('Link6')
        T = np.tile(np.eye(4), (len(Q), 1, 1))
        T[:, :3, :3] = quat_R(p.quaternion.reshape(-1, 4).cpu().numpy())
        T[:, :3, 3] = p.position.reshape(-1, 3).cpu().numpy()
        return T @ T_TIP

    def _ik(self, T_tip, q_seed, n=IK_SEEDS):
        """공구 끝 목표 4x4 → 관절 (rad): cuRobo IKSolver (계획기 안 — 관절 한계·자기 충돌·계획기 지도 충돌 함께, 성공 = IK_TOL 안),
        현재 상태 = q_seed (cuRobo 가 씨앗으로도 씀), 성공한 해 n 개 중 q_seed 에 가장 가까운 것 (관절 최대 차 — 해는 비용 순이라 첫 해가
        다른 자세 (팔꿈치·손목 뒤집힘) 일 수 있음). 반환 (해, 정기구학 확인 글) · 못 풀면 (None, 까닭).
        [2026-10-27 R-1 ④] 고른 해를 정기구학으로 다시 봄: 공구 끝이 목표에서 IK_TOL 밖이면 거절 — 성공은 cuRobo r.success 뿐이었고
        IK_TOL 은 만든 뒤 설정 값을 바꿔 넣었음. [2026-10-28] 이제 계획기를 만들 때 넣음 (rt_world.World(ik_tol=…)) — 확인은 그대로 둠.
        [2026-10-28 사용자 결정] 해를 2π 감아 고르고, 고른 해도 J1 > TIP_MAX_J1_DEG 이나 어느 관절 > TIP_MAX_JOINT_DEG 돌아야 하면 거절.
        못 풀 때의 까닭은 모두 '좌표 이동 불가 — …' 로 시작 (GUI 로그에 오류로, 로봇은 안 움직임)."""
        T6 = np.asarray(T_tip, float) @ T_TIP_INV
        pose = Pose(position=torch.tensor(T6[None, :3, 3], device='cuda', dtype=torch.float32),
                    quaternion=torch.tensor(R_quat(T6[:3, :3])[None], device='cuda', dtype=torch.float32))
        cur = JointState.from_position(torch.tensor([list(q_seed)], device='cuda', dtype=torch.float32), joint_names=JOINTS)
        r = self.W.planner.ik_solver.solve_pose(GoalToolPose.from_poses({'Link6': pose}), current_state=cur, return_seeds=n)
        ok = r.success.reshape(-1).cpu().numpy().astype(bool)
        Q = r.solution.reshape(-1, len(JOINTS)).cpu().numpy().astype(float)
        if not ok.any():
            pe = r.position_error.reshape(-1).cpu().numpy().astype(float)
            re = r.rotation_error.reshape(-1).cpu().numpy().astype(float)
            if np.any((pe < IK_TOL_M) & (re < IK_TOL_RAD)):
                return None, '좌표 이동 불가 — 그 자세는 풀리지만 충돌 (지도·자기 충돌) 또는 관절 한계 (cuRobo IK)'
            k = int(np.argmin(pe))
            return None, (f'좌표 이동 불가 — 닿지 않는 자리 (cuRobo IK 가장 가까운 해: 위치 오차 {pe[k] * 1000:.1f} mm · 방향 오차 '
                          f'{math.degrees(re[k]):.2f}°)')
        # [2026-10-28] 해마다 지금 관절에 가장 가까운 같은 각 (q_seed + wrap(q − q_seed)) 으로 — 관절 한계 안일 때만 (밖이면 원래 값).
        #  cuRobo 는 같은 자세를 ±360° 다른 값으로 돌려줌 (J2 −319.6 ≡ +40.4) → 감지 않으면 고르기·계획이 틀렸음
        qs = np.asarray(q_seed, float)
        Q = Q[ok]
        Wq = qs + (Q - qs + np.pi) % (2 * np.pi) - np.pi
        lo, hi = self.mpc.qlim
        Q = np.where((Wq >= lo) & (Wq <= hi), Wq, Q)
        q = Q[int(np.argmin(np.abs(Q - qs).max(axis=1)))]
        d = np.degrees(np.abs(q - qs))
        if d[0] > TIP_MAX_J1_DEG or d.max() > TIP_MAX_JOINT_DEG:
            big = ', '.join(f'J{j + 1} {d[j]:.0f}°' for j in range(len(d)) if d[j] > (TIP_MAX_J1_DEG if j == 0 else TIP_MAX_JOINT_DEG))
            return None, f'좌표 이동 불가 — 지금 팔 모양으로는 그 공구 자세에 닿지 않음 (팔을 크게 돌려야 함: {big})'
        T = self._tip(q)
        dp = float(np.linalg.norm(T[:3, 3] - np.asarray(T_tip, float)[:3, 3]))
        dr = float(rot_angle(T[:3, :3], np.asarray(T_tip, float)[:3, :3]))
        fk = f'정기구학 확인 오차 {dp * 1000:.3f} mm · {math.degrees(dr):.3f}°'
        if dp > IK_TOL_M or dr > IK_TOL_RAD:
            return None, (f'좌표 이동 불가 — IK 해의 공구 끝이 목표에서 벗어남 ({fk} > 허용 {IK_TOL_M * 1000:.1f} mm · '
                          f'{math.degrees(IK_TOL_RAD):.2f}°, cuRobo 는 성공이라 함)')
        return q, fk

    def _command_carrot(self, s0):
        """[2026-10-26 사용자 "unshaped 경로로 했을 때 속도 배율이 1 -> 0.5 로 되었을 때 시간이 2배로"] 실시간 회피 끔 + 경로를 따라가는 중:
        MPC 당근 (경로의 기준 시각 + 0.4 s 앞 점을 쫓음 → 빠른 경로는 늦게, 느린 경로는 당근이 먼저 끝에 닿아 앞질러 끝남 — WMX 실측 배율
        1 → 0.5 에서 계획 2.00 배인데 보낸 성형 전 명령은 1.63 배) 대신 경로를 시각 그대로 따라감: 목표 = 경로의 (기준 시각 + 한 주기) 점을
        한계 안 추종기가 한 주기에 쫓음. 기준 시각은 명령이 경로에서 follow_tol 안일 때만 한 주기씩 (안전 검사가 세우면 기다림, stuck_s
        넘으면 '막힘'). 지키기 (도착 뒤·정지점·누르는 동안 조그) 도 MPC 없이 지키기 목표 (Mpc.goal) 를 추종기가 바로 쫓음 — [2026-10-26
        WMX] MPC 지키기로 넘기면 MPC 가 낡은 warm start 로 도착 직후 0.145 rad/s · 가속 7.4 rad/s² 를 냈고 (→ B 다리마다), 끝내 명령이 안 서서
        ('서 있나' 거짓) 입력성형 적용이 거절됐음. MPC 상태·목표는 주기마다 명령 자리로 맞춰 두고, MPC 로 돌아갈 때 (실시간 회피 켜기·물러나기)
        새로 시작 (reset). 실시간 회피를 켜면 원래 MPC 당근 (피하는 동안 시간은 달라짐)."""
        m = self.mpc
        direct = not self.realtime and (m.mode == 'hold' or (m.mode == 'move' and m.path is not None))
        if not direct:
            if self._direct:                        # 직접 따라가기 → MPC: 낡은 warm start 버리고 지금 명령 자리에서 새로
                m.resync(s0[0], restart=True)
            self._direct = False
            self._update_rt_scale(s0)
            return super()._command_carrot(s0)
        self._direct = True
        t0 = time.perf_counter()
        m.clock += self.dt
        mc = self.c.motion
        F = None
        if m.mode == 'move':
            end = float(m.path[0][-1])
            if self._exact_path is not m.path:      # 새 경로 ([R-1 ①] 묶음 자체로 비교): 처음에는 명령 = 경로 그대로 (아래)
                self._exact_path, self._exact = m.path, True
                self._spl = _path_spline(*m.path)
            if self._exact:
                # [2026-10-26 WMX 3 차] 추종기로 경로 점을 한 주기 뒤에서 쫓으면 빠른 경로 (배율 1) 에서 늦어져 성형 전 명령 시간이 계획의
                #  1.10 배 (0.5 에서는 0.95 배) → 1 → 0.5 가 1.72 배. 그래서 명령 (성형 전 u) = 계획 경로 그대로 (1 ms 칸, 5 차 스플라인 — 양 끝
                #  속도·가속 0): 경로가 이미 한계 (95 %) 안. 명령이 경로에서 벗어나면 (안전 검사 정지·메시지 두 번 늦음·멈추지 않는 다시 계획) 이
                #  경로 남은 동안은 추종기로
                x0 = _spline_states(self._spl, [m.ref_t], end)[0]
                if float(np.max(np.abs(s0[0] - x0[0]))) < 1e-6 and float(np.max(np.abs(s0[1] - x0[1]))) < 1e-4:
                    F = _spline_states(self._spl, m.ref_t + np.arange(1, self.n_main + 1) * self.g, end)
                    m.ref_t = min(m.ref_t + self.dt, end)
                    m.t_progress = m.clock
                else:
                    self._exact = False
                    self.event(f'{self.label}: 명령이 계획 경로에서 벗어남 → 이 경로는 추종기로 따라감')
            if F is None and float(np.max(np.abs(s0[0] - m.path_at(m.ref_t)))) < self.c.mpc.follow_tol_rad:
                m.ref_t = min(m.ref_t + self.dt, end)
                m.t_progress = m.clock
            target = m.path_at(min(m.ref_t + self.dt, end))
            final, at_end = m.ref_t + self.dt >= end - 1e-9, m.ref_t >= end - 1e-9
        else:
            target = np.array(m.goal if m.goal is not None else s0[0], float)
            final, at_end = False, False
        n = int(round(mc.cycles * self.dt / mc.step_s))
        if F is None:
            Q, V, A, F = track_target(*s0, target, n, mc.step_s, m.vmax, m.amax, self.jmax, self.dt, sub=mc.sub,
                                      stop_decel=mc.final_stop_decel if final else 1.0, qlim=m.qlim, fine=True)
        else:
            Q = F[mc.sub - 1::mc.sub, 0]                # 기록 (lim_dev) 용 — 추종기와 같은 간격의 점
        u = np.concatenate([np.stack(s0)[None], F, self._u_tail(F[-1])])
        m.resync(u[self.n_dt, 0])
        if m.mode == 'move':
            m.set_goal(np.asarray(target, float))   # MPC 목표도 같이 (지키기로 바뀔 때 목표 비교가 맞게)
        k = int(round(self.dt / mc.step_s)) - 1
        info = {'mode': m.mode, 'target': target, 'ok': True, 'feasible': True, 'final': final, 'at_end': at_end,
                'stuck': m.mode == 'move' and m.clock - m.t_progress > self.c.mpc.stuck_s, 'raw_acc_ratio': None,
                'ms': round((time.perf_counter() - t0) * 1000, 1)}
        rec = {'carrot': target, 'final': final, 'clamp': False, 'held': False, 'carrot_out': False,
               'lim_dev': round(float(np.max(np.abs(Q[k] - target))), 5), 'raw_acc_ratio': None, 'direct': True}
        return u, info, rec

    # ---------------- 주기 ----------------
    def _frames_until(self, t_end):
        if not self.realtime and not self._refreshing:
            self.cam_next = max(self.cam_next, t_end)      # 실시간 회피 끔: 지도 멈춤
            return
        if self.real_cam is not None:
            if not (self._defer_frames and self.ready):    # 실제 카메라: 계산 직전 말고 보낸 뒤 빈 시간에 새 장 하나
                self._real_frame()
            return
        if self._defer_frames:
            t_end -= FRAME_BUDGET_S                        # 계산 직전 영상은 보낸 뒤로 (cycle 끝에서 처리)
        super()._frames_until(t_end)

    def _refresh_map(self):
        """실시간 회피 끔에서 계획 직전: REFRESH_S 분 영상으로 지도를 새로 봄 (지금 자세의 로봇)."""
        self._refreshing = True
        try:
            if self.real_cam is not None:
                end = time.monotonic() + REFRESH_REAL_S
                while (left := end - time.monotonic()) > 0:
                    if not self._real_frame():
                        self.real_cam.wait_depth(self._n_frame, left)     # [R-1 ⑨] 새 영상이 오면 바로 (전엔 10 ms 마다 물어봄)
            else:
                self.cam_next = self.clock.now() - REFRESH_S
                super()._frames_until(self.clock.now())
        finally:
            self._refreshing = False

    def _real_frame(self):
        """실제 카메라 새 깊이 한 장 → 지도 (rt_world.World.frame 의 렌더 대신). 팔 지우기 = 영상이 찍힌 순간의 관절각 (엔코더 기록 보간)
        또는 고정 자세 (mask_q: 시뮬레이션 로봇 + 실제 카메라 = 실제 팔은 엔코더 읽은 자리에 그대로). 쌓았으면 True."""
        import torch
        got = self.real_cam.take_depth(self._n_frame)
        if got is None:
            return False
        self._n_frame, depth, stamp = got
        s = self.W.cm.s
        if depth.shape != (s.image_height, s.image_width):
            if not self._size_warned:
                self._size_warned = True
                self.event(f'실제 카메라 영상 크기 {depth.shape[1]}×{depth.shape[0]} ≠ 지도 설정 {s.image_width}×{s.image_height} — 안 쌓음')
            return False
        q_ctl = self.robot.measured()
        # 실제 카메라는 mask_q (시뮬레이션 로봇) 또는 실물 (GuiRosRobot — q_at 있음) 일 때만 켜짐 (make_controller 가 막음)
        q_mask = self.mask_q if self.mask_q is not None else self.robot.q_at(stamp)
        t0 = time.perf_counter()
        raw = depth
        depth, n_speck = drop_specks(depth)
        d = torch.from_numpy(depth.astype(np.float32)).to('cuda').mul_(0.001)
        f_cnt = None
        if self._dfilt is not None:                 # [2026-10-28 D-4] 가림 띠 거르기 (depth_filters — setup_depth_filters)
            d, f_cnt = self._depth_filter(self._n_frame, raw, d, q_mask)
        # [R-1 S2] 엔진 공개 API: 쌓기 (팔 지우기 = q_mask) → 경계 채우기 (움직이는 = 제어하는 로봇 q_ctl) → MPC 지도 → q_ctl 진단 거리.
        #  ok = False 면 깊이 있는 픽셀이 너무 적어 안 쌓음 (가려진 카메라)
        ok, nr = self.W.integrate_depth(d, q_mask, q_ctl)
        map_ms = (time.perf_counter() - t0) * 1000
        fr = self.rec['frames']
        fr.append({'t': round(stamp, 4), 'q': np.round(q_ctl, 6).tolist(), 'map_ms': round(map_ms, 1), 'fill': self.W.cm.boundary_fill,
                   'near_mm': nr.get('near_mm'), 'near_new_mm': nr.get('near_new_mm'), 'real': True, 'speck': n_speck,
                   'lag_ms': round((time.time() - stamp) * 1000, 1)})
        if f_cnt is not None:
            fr[-1]['dfilt'] = f_cnt                 # [D-4] 지운 픽셀 수 {'tmp': 시간 확인, 'band': 가림 띠}
        if self.snap is not None and self.realtime and self.ready:
            self._snap_check_frame(stamp, nr.get('near_mm'))        # [D-4] 이미지 기록 near 트리거
        if len(fr) % rt_run.NEWVOX_EVERY == 0 and self.W.cm.bg_ready:
            nv = {'t': round(stamp, 3), 'v': self.W.voxel_code('dyn')}
            if self.W.voxel_cut:                    # [R-1 S2] 엔진 기록과 같게: 칸이 maxn 을 넘어 버린 수
                nv['cut'] = self.W.voxel_cut
            self.rec['newvox'].append(nv)
        return bool(ok)         # [독립 검토 MEDIUM] 전엔 늘 True — 지도 점검이 안 쌓인 영상까지 세어 옛 지도로 통과할 수 있었음

    def cycle(self):
        self._poll()
        lead = self.c.stream.lead_s
        if self.stream.stamp is None and self.clock.now() < self._estop_until - lead:
            self.clock.sleep_until(self._estop_until - lead)     # 비상정지 감속이 끝난 뒤에 새 시간표
        self._defer_frames = not self.ideal
        try:
            info = super().cycle()
        finally:
            self._defer_frames = False
        if self.snap is not None:
            self._snap_check_cycle()            # [D-4] 안전 검사 감속 정지 → 이미지 기록
        self._publish(info)
        self._trim()
        if self.snap is not None:
            self.snap.poll()                    # [D-4] 메시지를 보낸 뒤 (무거운 일은 한 주기에 하나 — GPU 몇 ms)
        if not self.ideal and self.stream.stamp is not None:
            # 다음 계산 시작 (다음 스탬프 − lead) 앞 빈 시간에 영상 처리 (계산 직전에는 FRAME_BUDGET_S 앞까지만)
            self._frames_until(self.stream.stamp + self.dt - lead - FRAME_BUDGET_S)
        return info

    def _run(self, job):
        self.job, self.job_goal = job, job['goal']
        self.speed = self.speed_req             # 속도 배율은 일을 시작할 때 바꿈 (일 도중의 다시 계획·반복 다리는 같은 배율)
        if self.estopped:
            self._resume(job['label'])
        try:
            if job['kind'] == 'repeat':
                n, a, b = 0, job['a'], job['goal']
                while True:
                    n += 1
                    self.job_goal = b
                    if not self.leg(f'반복 {n}: → 목표', b)['ok']:
                        break
                    self.wait_cycles(REPEAT_PAUSE_S)
                    self.job_goal = a
                    if not self.leg(f'반복 {n}: → 처음 자리', a)['ok']:
                        break
                    self.wait_cycles(REPEAT_PAUSE_S)
            else:
                self.leg(job['label'], job['goal'])
        finally:
            self.job = None
            self.settle_until = self.clock.now() + SETTLE_PAD_S + self.kernel.duration_s

    def _after_abort(self, e):
        self.job = None
        if e.kind == 'estop':
            self._estop()
        else:
            self._stop('끄기' if e.kind == 'quit' else '정지')

    def _idle(self):
        self.label = '비상정지' if self.estopped else '서 있음'
        if (not self.estopped and self.job is None and self.sj is None and not self.pending
                and self.clock.now() > self.stream.tail_done + IDLE_TAIL_PAD_S and not self._standing()):
            # [2026-10-27 D-1 독립 검토 MEDIUM] 주기가 멈춘 뒤 (서는 중 시간이 끝남) 로봇은 마지막 메시지 꼬리로 섰는데 명령 상태는 그 전
            #  (움직이는 중) 그대로였음 → '서 있나' 가 계속 거짓이라 실시간 회피 켜기·[지도 점검] 이 계속 거절되고, [정지] 정지점이 낡은
            #  상태에서 잡혀 선 자리보다 뒤로 되돌아갈 수 있었음 → 명령 상태를 선 자리 (꼬리 끝) 에서 새로
            self._restart_here()
        if self.W.map_off and self.job is None and self._cam_age() <= CAM_STALE_S:
            # [D-7] 서 있는 동안 고정 카메라 영상이 다시 옴 → 지도를 다시 씀 (3D·지도 글도 바로 — 다음 이동은 _choose_map 이 다시 정함)
            self.W.map_off = False
            self.event('고정 카메라 영상이 다시 옴 → 다음 이동부터 장애물 (카메라 지도) 을 봄')
        self._poll()
        self.cam_next = max(self.cam_next, self.clock.now())
        self._publish(None)
        self._trim()
        if self.snap is not None:
            self.snap.poll()                    # [D-4] 이미지 기록 (서 있을 때 — 지도 점검 걸림 등)
        if not self.ideal and time.monotonic() - self._t_gc > GC_IDLE_S and self.clock.now() > self.stream.tail_done:
            self._t_gc = time.monotonic()
            gc.collect()                        # 명령을 안 보내며 서 있을 때만 (제어 중 GC 멈춤 → 늦은 메시지)
        self._take_cmds(IDLE_S)                 # [R-1 ⑨] 잠들지 않고 명령을 기다림 (오면 바로 처리)

    def serve(self):
        self.serving = True                     # [R-1 ⑥] 이제부터 SIGTERM·SIGINT 첫 번째 = 끄기 (main 의 처리기)
        if not self.ideal:
            # rt_run.run 과 같음 [2026-10-13] 제어 루프 동안 GC 끔: GC 가 ~0.4 s 멈춰 메시지가 늦게 가 MaxAccError 가 났었음
            gc.collect()
            gc.disable()
        self.mpc.start(self.q0)
        self.W.cm.start_background()
        self.event('켜기: 배경 모으기')
        while not self.quit or self.sj is not None:         # 끄기라도 조그 (Servo) 가 다 설 때까지
            try:
                if self.sj is not None:
                    self._servo_cycle()
                    continue
                if not self.ready:
                    if self.realtime:
                        self.cycle()
                    else:
                        # [2026-10-26] 실시간 회피 끔으로 시작 (실물): 명령을 안 보내고 지도 배경만 모음. 끔이면 _frames_until 이 지도를
                        #  멈춰 배경이 안 모여 준비가 영영 안 끝났음 (실물 대상 넣을 때 찾음). 지키기 MPC 주기도 안 돌림 → 켜는 동안
                        #  지도 때문에 로봇이 움직이지 않음 (10-26 실제 카메라 시험: 켜는 동안 지키기가 J1 을 13° 움직였음)
                        self._refreshing = True
                        try:
                            self._frames_until(self.clock.now())
                        finally:
                            self._refreshing = False
                        self._idle()
                    if self.W.cm.bg_ready or self._cam_off():
                        # [D-7] 고정 카메라가 꺼져 있으면 지도 배경을 기다리지 않음 (영영 안 참) — 이동은 장애물을 안 봄 (_choose_map)
                        self.ready = True
                        self.link.hello['ready'] = True
                        self.event('준비 끝 — 명령을 기다림' + ('' if self.W.cm.bg_ready else
                                                          ' (고정 카메라 꺼짐: 지도 없이 — 이동은 장애물을 안 봄, 카메라를 켜면 지도를 쌓음)'))
                    continue
                if self.pending:
                    self._run(self.pending.popleft())
                    continue
                if (self.realtime and not self.estopped) or self.clock.now() < self.settle_until:
                    if self.job is None:
                        self.label = '지키기' if self.realtime else '서는 중'
                    self.cycle()
                else:
                    self._idle()
            except Abort as e:
                self._after_abort(e)
        # 끄기: 서는 데까지 주기를 마저 돌리고 (정지 감속 + 성형 길이), 마지막 메시지 꼬리가 끝날 때까지 기다림 (서 있을 때 R2 를 끔).
        #  실시간 회피 끔으로 이미 서 있으면 (명령을 안 보내는 중) 건너뜀 — 오래 쉰 뒤 주기를 돌리면 '주기 늦음' 기록만 남았음 (10-26)
        if self.realtime or self.clock.now() < self.settle_until or not self._standing():
            end = max(self.clock.now() + SETTLE_OFF_S + self.kernel.duration_s, self.settle_until)
            while self.clock.now() < end and not self.estopped:
                rt_run.Controller.cycle(self)
        self.clock.sleep_until(max(self.stream.tail_done, self._estop_until) + TAIL_PAD_S)
        self.event('서버 끝')

    # ---------------- GUI 로 ----------------
    def _publish(self, info):
        mono = time.monotonic()
        if mono - self._t_pub < PUB_S:
            return
        self._t_pub = mono
        now = time.time()                       # (GUI 에 보내는 시각은 벽시계 그대로)
        cy = self.rec['cycles'][-1] if (info is not None and self.rec['cycles']) else None
        q = self.robot.measured()
        self.link.send({'type': 'state', 't': now, 'q': q.tolist(),
                   'u': cy['cmd'] if cy else np.asarray(self.stream.cmd[0], float).tolist(),
                   'y': cy['cmd_y'] if cy else q.tolist(),
                   'mode': self.mpc.mode, 'label': self.label, 'job': None if self.job is None else self.job['label'],
                   'goal': None if self.job_goal is None else np.asarray(self.job_goal, float).tolist(),
                   'realtime': self.realtime, 'estop': self.estopped, 'jog': self.sj is not None, 'ready': self.ready,
                   'clr_mm': cy['clr_now_mm'] if cy else None, 'guard': cy['guard'] if cy else None,
                   'skipped': bool(cy['skipped']) if cy else False, 'mpc_ms': cy['mpc_ms'] if cy else None,
                   'cyc_ms': cy['cyc_ms'] if cy else None, 'lead_ms': cy.get('lead_ms') if cy else None,
                   'objects': self.t_go is not None, 'robot': self.kind, 'fault': self.r2_fault, 'speed': self.speed_req,
                   'servo': self.servo_on, 'camera': 'virtual' if self.real_cam is None else 'real',
                   'cam_fps': None if self.real_cam is None else round(self.real_cam.fps(), 1),
                   'js_age_ms': None if self.ideal else round((mono - self.robot.t_js_mono) * 1000, 1),
                   'comm_left_s': None if self.comm_deadline is None else round(self.comm_deadline - mono)})
        path = self.mpc.path
        if path is not None and path is not self._path_sent:      # [R-1 ①] 묶음 자체로 비교 (id 는 다시 쓰일 수 있음)
            self._path_sent = path
            P = np.asarray(path[1], float)
            idx = np.unique(np.linspace(0, len(P) - 1, min(len(P), 120)).round().astype(int))
            self.link.send({'type': 'path', 'q': P[idx].tolist()})
        if mono - self._t_vox >= VOX_S:
            self._t_vox = mono
            try:
                self._send_voxels()
            except Exception as e:                  # 표시용이라 제어는 계속
                if not self._vox_err:
                    self._vox_err = True
                    self.event(f'지도 칸 보내기 오류 (표시만 안 됨): {e!r}')
                    LOGGER.error('  (지도 칸 보내기 오류 — traceback)', exc_info=True)

    def _send_voxels(self):
        import torch
        s = self.W.cm.s
        vs = float(s.esdf_voxel_size)
        if self.W.map_off:                      # [D-7] 카메라 지도를 안 씀 → 3D 에도 칸 없이 (계획기가 보는 그대로), GUI 지도 글 '안 씀'
            self.link.send({'type': 'voxels', 'P': np.zeros((0, 3), np.float16), 'vs': vs, 'off': True})
            return
        occ = self.W.cm.raw_grid().float() <= 0.5 * vs
        occ[:, :, :max(0, int(np.ceil((self.c.map.table_z + self.c.map.floor_band_m - s.origin_m[2]) / vs - 0.5)))] = False   # [D-2] 테이블 위 띠
        idx = torch.nonzero(occ)
        if len(idx) > VOX_MAX:
            idx = idx[torch.randperm(len(idx), device=idx.device)[:VOX_MAX]]
        org = torch.tensor(s.origin_m, device=idx.device, dtype=torch.float32)
        P = (org + (idx.float() + 0.5) * vs).cpu().numpy().astype(np.float16)
        self.link.send({'type': 'voxels', 'P': P, 'vs': vs})

    def _trim(self):
        """기록 줄이기 (TRIM_S 마다): 기록 중 (--out) 이면 켜고 rec_max_s 동안은 다 남김 (끝날 때 저장), 그 뒤나 기록 안 하면 GUI 에 쓰는
        만큼만. [2026-10-27 R-1 ③] + JTC 기준 기록 (RosRobot.jtc — 100~1000 Hz 로 쌓임, jtc_ref 는 마지막만 씀)·바꾸기 전 성형기의
        실행된 u (old_runs)·지금 세그먼트의 실행된 u 조각 (실시간 회피로 오래 지키면 세그먼트가 안 닫혀 한 시간 ~180 MB) 도."""
        now = time.monotonic()
        if now - self._t_trim < TRIM_S:
            return
        self._t_trim = now
        if self.out is not None and now - self.t_start < self.rec_max_s:
            return                              # 기록 중: 끝날 때 저장하려고 다 남김
        r = self.rec
        for key, n in (('cycles', 100), ('frames', 100), ('events', 300), ('legs', 50)):
            r[key] = r[key][-n:]
        r['newvox'] = []
        self._sent_log = []
        self.stream.segments = self.stream.segments[-1:]
        self.stream.trim_executed(TRIM_CHUNKS)  # [2026-10-28 R-1] 엔진 API (전엔 rt_server.trim_stream_chunks — 같은 코드)
        for run in self.old_runs:
            run['segments'] = []                # (저장 기록에는 성형기 이름·바꾼 시각만 — u_exec_k 파일 없음)
        if not self.ideal:
            self.robot.trim_jtc(TRIM_JTC)

    def save_all(self, out):
        """기록 저장: rt_run.json (rt_run.Controller.save) + 실행된 u — 지금 성형기 u_exec.npz, 바꾸기 전 성형기 u_exec_k<i>.npz
        (같은 형식, rt_records.u_exec_runs 로 tools.rt_report·tools.wmx_check 가 읽음) + 비상정지 감속 구간 (성형을 건너뜀)."""
        out = Path(out)
        out.mkdir(parents=True, exist_ok=True)
        self._live_wait()
        rec = self.rec
        rec['estop_windows'] = self.estop_windows
        rec['kernels'] = []
        for i, r in enumerate(self.old_runs):
            f = f'u_exec_k{i}.npz'                  # [R-1 S2] 엔진과 같은 저장 함수 (세그먼트가 없으면 안 씀 → 파일 없음)
            if not rt_run.save_u_exec(out / f, r['segments'], r['grid_s'], r['h']):
                f = None
            rec['kernels'].append({'label': r['label'], 'until': r['until'], 'file': f})
        rec['kernels'].append({'label': shaper_def.label(*self.shaper_cfg), 'until': None, 'file': 'u_exec.npz'})
        rec['gui'] = {'robot': self.kind, 'fault': self.r2_fault, 'speed': self.speed}
        rec['live'], rec['back'] = dict(self.n_live), dict(self.n_back)      # rt_run.run 끝과 같음 (보고서 '다시 계획'·'물러나기' 수)
        self.save(out)
