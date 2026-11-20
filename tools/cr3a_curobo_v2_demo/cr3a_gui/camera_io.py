#!/usr/bin/env python3
"""고정 카메라 (전체를 보는 D455) — 노드 켜기·끄기, 영상 받기, 등록한 카메라 위치 읽기·쓰기 (화면 없음).  ★ wmx_r2 컨테이너 안 (ROS) ★

[2026-10-27 R-1 리팩터링] camera_panel.py 에서 PyQt 없는 부분을 떼어 냄 — rt_server (실제 카메라 지도) 가 PyQt 없이 쓸 수 있게. camera_panel 은
  이 이름들을 그대로 다시 내보냄 (rt_server·작업기록 스크립트는 그대로 됨). tools/cam_capture.py 도 노드 설정·영상 바꾸기·깊이 가운데값을 여기서.
카메라: realsense2_camera 노드를 GUI 와 따로 띄움 (GUI 를 닫아도 계속 — 서버가 실제 카메라 지도에 씀). 시리얼 333422303104
  (= USB 412243062648, 포트 2-3.2). 손목 D455 (333422300962) 는 지금 안 씀 (사용자). 컬러 + 깊이 원본 (+ 컬러 맞춤 깊이: 미리보기).
"""
import atexit
import json
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from common import TMP_DIR

HERE = Path(__file__).resolve().parent
CAM_SERIAL = '333422303104'
CAM_NS = '/cam_fixed'
CAM_TOPIC = f'{CAM_NS}/camera'
CAM_LOG = TMP_DIR / 'cr3a_cam_fixed.log'
CAM_PIDF = TMP_DIR / 'cr3a_cam_fixed.pid'
CAM_YAML = TMP_DIR / 'cr3a_cam_fixed.yaml'
POSE_FILE = HERE / 'camera_pose.json'          # 등록한 카메라 위치 (받침 ← 깊이 광학 좌표). 서버·GUI 3D 가 읽음
ENC_FILE = HERE / 'last_encoders.json'         # 마지막 [엔코더 읽기] (실제 카메라 + 시뮬레이션 로봇: 실제 팔을 이 자세로 지움)
N_DEPTH = 10                                    # 보정 한 번에 모으는 깊이 장 수 (가운데값)
STOP_WAIT_S = 5.0                               # 노드 끄기: SIGINT 뒤 이만큼 안에 안 끝나면 SIGTERM


def load_pose(path=POSE_FILE):
    """등록한 카메라 위치 dict (없으면 None)."""
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return None


def write_json_atomic(path, data, **dump_kw):
    """JSON 을 한 번에 바꿔 씀 (.tmp 에 쓰고 os.replace) — 읽는 쪽 (서버·GUI) 이 쓰는 도중의 반쪽 파일을 읽지 않게."""
    path = Path(path)
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(json.dumps(data, **dump_kw), encoding='utf-8')
    os.replace(tmp, path)


def save_pose(data, session):
    """카메라 위치 등록: 이전 것은 session/camera_pose_before.json 으로 남기고 POSE_FILE 을 한 번에 바꿈 ([D-2 검토] 서버·GUI 가
    반쪽 파일을 읽지 않게) + session/camera_pose.json."""
    session = Path(session)
    session.mkdir(parents=True, exist_ok=True)
    if POSE_FILE.exists():
        (session / 'camera_pose_before.json').write_text(POSE_FILE.read_text(encoding='utf-8'), encoding='utf-8')
    write_json_atomic(POSE_FILE, data, indent=1, ensure_ascii=False)
    (session / 'camera_pose.json').write_text(json.dumps(data, indent=1, ensure_ascii=False), encoding='utf-8')


# ---------------- 노드 설정·영상 (tools/cam_capture.py 와 같이 씀) ----------------
def realsense_yaml(node, serial, width=640, height=480, fps=15):
    """realsense2_camera 노드 설정 (node = '<이름공간>/camera'): 컬러 + 깊이 (+ 컬러 맞춤 깊이), IR·IMU 끔, 컬러·깊이 같은 해상도·fps."""
    return f'''{node}:
  ros__parameters:
    serial_no: "{serial}"
    enable_color: true
    enable_depth: true
    enable_infra1: false
    enable_infra2: false
    enable_gyro: false
    enable_accel: false
    align_depth.enable: true
    rgb_camera.color_profile: {width}x{height}x{fps}
    depth_module.depth_profile: {width}x{height}x{fps}
'''


def realsense_argv(ns, params_file):
    """realsense2_camera 노드 실행 줄 — 실행 파일을 바로 (ros2 run 은 손자로 띄워 PID 로 못 끔)."""
    from ament_index_python.packages import get_package_prefix
    exe = Path(get_package_prefix('realsense2_camera')) / 'lib/realsense2_camera/realsense2_camera_node'
    return [str(exe), '--ros-args', '-r', '__node:=camera', '-r', f'__ns:={ns}', '--params-file', str(params_file)]


def ros_image_to_np(m):
    """sensor_msgs/Image → numpy: 깊이 (16UC1·mono16) = uint16 (h, w) mm, 컬러 = BGR uint8 (rgb8 이면 바꿈)."""
    if m.encoding in ('16UC1', 'mono16'):
        return np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy()
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
    return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a.copy()


def depth_median(frames):
    """깊이 영상 여러 장 (uint16 mm) 의 칸마다 가운데값 — 0 (없음) 은 빼고, 모두 없으면 0 (튀는 점 줄임)."""
    D = np.stack(frames).astype(np.float32)
    D[D == 0] = np.nan
    with np.errstate(all='ignore'):
        return np.nan_to_num(np.nanmedian(D, axis=0), nan=0).astype(np.uint16)


def _proc_state(pid):
    """/proc/<pid>/stat 의 상태 글자 (R·S·D·Z …) — 프로세스가 없으면 None."""
    try:
        s = Path(f'/proc/{pid}/stat').read_text()
    except OSError:
        return None
    return s[s.rfind(')') + 2:][:1] or None


# ---------------- 카메라 프로세스 ----------------
class CameraProcess:
    """고정 카메라 realsense2_camera 노드 (GUI 와 따로 도는 프로세스, PID 파일). 해상도·fps 는 켤 때.
    [2026-10-27 R-1 ②] 이 GUI 가 띄운 노드는 Popen 을 들고 있다가 끌 때 wait 로 거둠 — 전엔 Popen 을 버려 SIGINT 뒤 노드가 GUI 의 좀비로
    남고 /proc/<pid> 가 안 없어져 GUI 스레드에서 5 s 를 다 기다렸음 (화면 멈춤 → 서버가 GUI 끊김으로 보고 로봇 정지·실시간 회피 끔).
    앞 GUI 가 띄운 노드 (자식이 아님) 는 PID 파일 + /proc 상태 (없어지거나 Z = 끝남). 끄기는 GUI 가 QThreadPool 에서 부름."""
    _proc = None            # 이 프로세스가 띄운 노드 (subprocess.Popen) — 없으면 None

    @classmethod
    def pid(cls):
        own = cls._proc
        if own is not None and own.poll() is not None:     # 스스로 끝난 자식은 거둠 (좀비로 남지 않게)
            cls._proc = None
        try:
            p = int(CAM_PIDF.read_text().strip())
            cmd = Path(f'/proc/{p}/cmdline').read_bytes().decode(errors='ignore')
            return p if CAM_NS in cmd else None
        except (OSError, ValueError):
            return None

    @classmethod
    def start(cls, width=640, height=480, fps=15):
        if cls.pid():
            return True, '이미 켜져 있음'
        CAM_YAML.write_text(realsense_yaml(CAM_TOPIC, CAM_SERIAL, width, height, fps))
        p = cls._spawn(realsense_argv(CAM_NS, CAM_YAML))
        return True, f'카메라 노드 켬 (PID {p.pid}, {width}×{height} · {fps} fps, 로그 {CAM_LOG})'

    @classmethod
    def _spawn(cls, argv):
        """노드를 GUI 와 다른 세션으로 (GUI 를 닫아도 계속) + PID 파일. Popen 은 끌 때 거두려고 들고 있음, 로그 파일은 자식만 씀."""
        with open(CAM_LOG, 'w') as log:
            p = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        CAM_PIDF.write_text(str(p.pid))
        cls._proc = p
        return p

    @classmethod
    def stop(cls, wait_s=STOP_WAIT_S):
        """노드 끄기: SIGINT → wait_s 안에 안 끝나면 SIGTERM (2 s 더 기다림). 결과 글을 돌려줌. 그래도 안 끝나면 PID 파일을 남김 (다음
        [카메라 켜기] 가 같은 시리얼로 노드를 하나 더 띄우지 않게 — 다시 [끄기] 또는 확인)."""
        pid = cls.pid()
        if not pid:
            return '이미 꺼져 있음'
        own = cls._proc if (cls._proc is not None and cls._proc.pid == pid) else None
        how = '카메라 노드 끔'
        if cls._kill(pid, signal.SIGINT) and not cls._wait_gone(pid, own, wait_s):
            how = f'카메라 노드 끔 ({wait_s:.0f} s 안에 안 끝나 SIGTERM)'
            if cls._kill(pid, signal.SIGTERM) and not cls._wait_gone(pid, own, 2.0):
                return f'카메라 노드가 SIGINT·SIGTERM 뒤에도 안 끝남 (PID {pid}, 상태 {_proc_state(pid)}) — PID 파일 둠, 다시 [끄기] 또는 확인'
        if own is not None and own.poll() is not None:
            cls._proc = None
        CAM_PIDF.unlink(missing_ok=True)
        return how

    @staticmethod
    def _kill(pid, sig):
        """신호 보내기 — 이미 없는 프로세스면 False (끝난 것)."""
        try:
            os.kill(pid, sig)
            return True
        except ProcessLookupError:
            return False

    @staticmethod
    def _wait_gone(pid, own, timeout):
        """pid 가 끝날 때까지 (최대 timeout s, 끝나면 True). own = 이 프로세스의 자식 (Popen) 이면 wait 로 거둠, 아니면 /proc 상태
        (없음·Z = 끝남 — 거두는 것은 그 부모 몫)."""
        if own is not None:
            try:
                own.wait(timeout)
                return True
            except subprocess.TimeoutExpired:
                return False
        t_end = time.monotonic() + timeout
        while _proc_state(pid) not in (None, 'Z'):
            if time.monotonic() >= t_end:
                return False
            time.sleep(0.05)
        return True


# ---------------- 영상 받기 (ROS) ----------------
class CameraFeed:
    """고정 카메라 토픽 받기 (rclpy, 따로 도는 스레드). ROS 가 없으면 ok = False (PC 시안).
    depth = 깊이 원본도 받기 (서버 지도·보정 — 기본). [R-1] GUI 는 보정 창이 보일 때만 (set_depth) — 깊이 15 fps 를 늘 풀지 않게.
    fps('depth') = 깊이 영상 속도 (서버 상태 cam_fps), fps('color') = 컬러 (GUI 미리보기)."""

    def __init__(self, depth=True):
        self.ok, self.err = False, ''
        self.lock = threading.Lock()
        self.color = self.depth = None
        self.depths = []                 # 최근 깊이 원본 (보정용 가운데값)
        self.cinfo = self.dinfo = self.ext = None
        self.t_color = self.t_depth = 0.0
        self.stamp_depth = 0.0           # 깊이 영상이 찍힌 시각 (메시지 헤더, librealsense 전역 시각 = 이 PC 시계)
        self.n_depth = 0
        self._fps_t = []                 # 깊이 받은 시각 (최근 30)
        self._fps_c = []                 # 컬러 받은 시각 (최근 30)
        self._want_depth = bool(depth)
        self._ctx, self._closed, self._th = None, threading.Event(), None     # rclpy Context (받기 스레드가 만듦)·close 불림·받기 스레드
        try:
            import rclpy  # noqa: F401
        except ImportError as e:
            self.err = f'ROS 없음 ({e})'
            return
        self._th = threading.Thread(target=self._spin, daemon=True, name='camera_feed')
        self._th.start()
        atexit.register(self.close)

    def close(self, timeout=2.0):
        """[2026-10-28 R-1 통합 시험] 프로세스가 끝나기 전 (atexit — 어느 길로 끝나든) rclpy 를 끄고 받기 스레드가 끝나기를 기다림. 전엔 데몬
        스레드가 rclpy (C++) 실행기 안에서 기다리는 채로 인터프리터가 끝나, 카메라를 켠 GUI 가 끝에 abort ('terminate called without an
        active exception', 코어 — 통합 시험 B 03:18). 서버 DepthFeed 의 같은 고침 (32e2910) 과 같은 방법."""
        self._closed.set()
        ctx = self._ctx
        if ctx is not None:
            try:
                ctx.try_shutdown()
            except Exception:
                pass
        if self._th is not None:
            self._th.join(timeout)

    def set_depth(self, on):
        """깊이 받기 켜기·끄기 — 구독을 실제로 바꾸는 것은 rclpy 스레드 (0.2 s 안)."""
        self._want_depth = bool(on)

    def _spin(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
        try:
            from realsense2_camera_msgs.msg import Extrinsics
        except ImportError:
            Extrinsics = None
        ctx = Context()
        rclpy.init(context=ctx)
        self._ctx = ctx
        node = Node('cr3a_gui_camera', context=ctx)
        node.create_subscription(Image, f'{CAM_TOPIC}/color/image_raw', self._on_color, qos_profile_sensor_data)
        node.create_subscription(CameraInfo, f'{CAM_TOPIC}/color/camera_info', lambda m: setattr(self, 'cinfo', m), qos_profile_sensor_data)
        node.create_subscription(CameraInfo, f'{CAM_TOPIC}/depth/camera_info', lambda m: setattr(self, 'dinfo', m), qos_profile_sensor_data)
        if Extrinsics is not None:
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            node.create_subscription(Extrinsics, f'{CAM_TOPIC}/extrinsics/depth_to_color', lambda m: setattr(self, 'ext', m), latched)
        sub = [None]

        def sync_depth():               # 깊이 구독 만들기·지우기는 실행기가 도는 이 스레드에서만 (다른 스레드에서 지우면 위험)
            if self._want_depth and sub[0] is None:
                sub[0] = node.create_subscription(Image, f'{CAM_TOPIC}/depth/image_rect_raw', self._on_depth, qos_profile_sensor_data)
            elif not self._want_depth and sub[0] is not None:
                node.destroy_subscription(sub[0])
                sub[0] = None
        sync_depth()
        node.create_timer(0.2, sync_depth)
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        self.ok = True
        if self._closed.is_set():                   # (만드는 동안 close 가 불림 — close 는 아직 ctx 를 못 봤을 수 있음)
            ctx.try_shutdown()
        try:
            ex.spin()                               # close 의 try_shutdown 이 깨워 돌아옴
        except Exception:                           # (rclpy ExternalShutdownException 등 — 끄는 중)
            pass
        finally:
            try:
                ex.shutdown(timeout_sec=1.0)
                node.destroy_node()
            except Exception:
                pass

    def _on_color(self, m):
        img = ros_image_to_np(m)
        now = time.time()
        with self.lock:
            self.color, self.t_color = img, now
            self._fps_c.append(now)
            del self._fps_c[:-30]

    def _on_depth(self, m):
        d = ros_image_to_np(m)
        now = time.time()
        with self.lock:
            self.depth, self.t_depth = d, now
            self.stamp_depth = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            self.depths.append(d)
            del self.depths[:-N_DEPTH]
            self.n_depth += 1
            self._fps_t.append(now)
            del self._fps_t[:-30]

    def fps(self, kind='depth'):
        with self.lock:
            t = list(self._fps_t if kind == 'depth' else self._fps_c)
        return 0.0 if len(t) < 2 or time.time() - t[-1] > 1.0 else (len(t) - 1) / (t[-1] - t[0])

    def take_depth(self, after):
        """(지도용) after 번째 다음 깊이 영상: (번호, 깊이 uint16 mm, 찍힌 시각) — 새 것이 없으면 None."""
        with self.lock:
            if self.depth is None or self.n_depth <= after:
                return None
            return self.n_depth, self.depth, self.stamp_depth

    def live(self):
        """(컬러, 받은 지 s) — 없으면 (None, None)."""
        with self.lock:
            return (None, None) if self.color is None else (self.color, time.time() - self.t_color)

    def intrinsics(self):
        """(깊이 K, 컬러 K, 컬러 왜곡, 깊이 → 컬러 4x4) 또는 None (아직 못 받음)."""
        if self.cinfo is None or self.dinfo is None or self.ext is None:
            return None
        T = np.eye(4)
        T[:3, :3] = np.array(self.ext.rotation, float).reshape(3, 3).T      # librealsense: 열 우선
        T[:3, 3] = self.ext.translation
        return (np.array(self.dinfo.k, float).reshape(3, 3), np.array(self.cinfo.k, float).reshape(3, 3),
                list(self.cinfo.d), T)

    def snapshot(self, timeout=8.0):
        """보정용 한 장: 새로 받은 깊이 N_DEPTH 장의 가운데값 (0 = 없음은 뺌) + 마지막 컬러 + 내부·외부 값. 못 받으면 None."""
        n0, t0 = self.n_depth, time.time()
        while self.n_depth - n0 < N_DEPTH and time.time() - t0 < timeout:
            time.sleep(0.05)
        intr = self.intrinsics()
        with self.lock:
            if self.n_depth - n0 < N_DEPTH or self.color is None or intr is None:
                return None
            frames = list(self.depths[-N_DEPTH:])
            color = self.color.copy()
        Kd, Kc, Dc, T_cd = intr
        return {'depth': depth_median(frames), 'color': color, 'Kd': Kd, 'Kc': Kc, 'Dc': Dc, 'T_cd': T_cd, 't': time.time()}
