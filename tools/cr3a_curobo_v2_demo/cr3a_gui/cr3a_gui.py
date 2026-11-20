#!/usr/bin/env python3
"""CR3A 제어 GUI (PyQt5) — 입력성형 실물 시험용.

화면 (미로봇 GUI 와 같은 배치)
  위     = 한 줄 상태 (로봇 · WMX · 시스템 · 카메라 · 장애물 회피) + 대상 · 시스템 · 서보
           + [정지] (입력성형을 거쳐 섬, 단축키 없음) · [비상정지] (Esc, 즉시 — 성형 건너뜀)
  왼쪽   = 탭 [3D 뷰] (CR3A + 공구, 장애물 칸, 고정 카메라, 목표, 계획 경로) [관절 그래프]
  오른쪽 = 입력성형 (전역, 탭 밖 — shaping_panel.py) + 탭 [조그] [목표 이동] [카메라 세팅] + 계획 · 실행
  아래   = 로그
계획·회피는 늘 같은 것 (cuRoboV2) 이라 화면에 라이브러리 이름이나 켜기/끄기를 두지 않음.
모듈: cr3a_model.py (URDF·정기구학·메시), scene3d.py (3D 뷰), shaping_panel.py + shaper_def.py (입력성형), scope_panel.py (관절 그래프),
      camera_panel.py (보정 창·일 돌리기) + camera_io.py (카메라 노드·영상, 화면 없음), common.py (포트·경로 등 서버와 같이 쓰는 값),
      gui_mock.py (--mock 화면 시안), rt_server.py (실시간 회피 서버 — [▶ 시스템 시작] 이 띄우고, 같은 PC 안 연결로 명령·상태를 주고받음)
[2026-10-26 1 차 기능] 대상 = 이상적 시뮬레이션만 (소프트웨어 로봇 + 가상 카메라, WMX·실제 로봇 없음). 되는 것: 관절 목표 이동·반복,
  관절 조그 (한 칸씩·누르는 동안), 정지·비상정지, 입력성형 적용, 실시간 회피 켜기/끄기, 지도·경로·목표 3D, 관절 그래프, 시험 물체.
  다음 단계: 공구 끝 좌표 이동·조그 (IK), WMX 시뮬레이터·실제 로봇, 실제 카메라·위치 맞추기, 계획 속도.
[2026-10-26 밤] WMX 시뮬레이터·실제 카메라 지도·카메라 위치 맞추기 연결. 대상 '실제 로봇 (EtherCAT)' (시작 확인 창, 실시간 회피 끔으로
  시작 — 사용자 "꺼진 채로 시작"), 속도 배율 (처음 0.2, 1.0 = 로봇 한계 속도·가속 — 사용자 결정).
[2026-10-27 R-1 리팩터링 — 사용자 "안전·정확성 문제를 다 고친 뒤 정리"] ① 누르는 동안 조그는 GUI 스레드 맥박이 HOLD_STALL_S 끊기면 뗌
  ② [카메라 끄기] 가 GUI 를 5 s 멈추던 것 (좀비) ③ 보정 창이 열린 채 닫으면 앱이 남아 ping 을 보내던 것 ④ 늘 초록이던 칩·안 이어진 단추
  ⑤ 포트를 환경 변수로 (CR3A_GUI_PORT — 시험은 다른 포트) ⑥ 로그 = logging (파일 common.GUI_LOG + 로그 창, 예외도). 정리: 시안은
  gui_mock.py, 겹치던 값은 common.py, 카메라 입출력은 camera_io.py.

실행:  python cr3a_gui.py                  창 (서버가 이미 돌고 있으면 붙음)
       python cr3a_gui.py --mock           화면 시안 (예시 숫자, 서버 없음 — gui_mock.py)
       python cr3a_gui.py --mock --shot <폴더>   시안 화면을 탭마다 PNG 로 저장하고 끝냄
       환경 변수: CR3A_GUI_PORT (서버 포트, 기본 47861) · CR3A_TMP_DIR (/tmp) · CR3A_LOG_ROOT · CR3A_POSES_FILE — common.py
"""
import argparse
import collections
import html
import json
import logging
import math
import os
import queue
import subprocess
import sys
import threading
import time
import traceback
from logging.handlers import RotatingFileHandler
from multiprocessing.connection import Client
from pathlib import Path

import cv2
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

import camera_panel as cp
import cr3a_model as cm
from common import AUTH, GUI_LOG, LINK_TO_OPTICAL, PORT, SERVER_LOG, to_qimage
from scene3d import PRESETS, Scene3D
from scope_panel import ScopePanel
from shaping_panel import ShapingPanel
from shaper_def import build, label as shaper_label, ms
import rt_config  # noqa: E402  (shaper_def 가 cr3a_rt_shaping 경로를 넣음) 거리·가상 카메라 속도는 서버와 같은 설정에서

HERE = Path(__file__).resolve().parent
CUROBO_COPY = HERE.parent / 'curobo_v2_shaping'          # 성형 사본이 쓰는 cuRobo 사본 (rt_demo.sh 와 같은 PYTHONPATH)
COLORS = {'ok': '#1a7f37', 'warn': '#9a6700', 'err': '#cf222e', 'info': '#0969da', 'off': '#6e7781', 'real': '#bc4c00'}
LOG_COLORS = {10: '#6e7781', 20: '#1f2328', 30: '#9a6700', 40: '#cf222e'}     # = logging DEBUG·INFO·WARNING·ERROR
LOG = logging.getLogger('cr3a_gui')
TIP_TEXT = '공구 끝 = 그리퍼 손가락 끝 (플랜지 + 192.5 mm)'
RPY_TIP = 'Rx Ry Rz = 받침 기준 고정 축 x → y → z 순서 회전 (°)'
TARGETS = ('이상적 시뮬레이션 (로봇 없음)', 'WMX 시뮬레이터 (실제 로봇 안 움직임)', '실제 로봇 (EtherCAT) — 실제 CR3A 가 움직임')
TARGET_ROBOT = {0: 'ideal', 1: 'wmx', 2: 'real'}        # 대상 → rt_server.py --robot
SPEED0 = 20                     # 속도 배율 처음 값 % (사용자 결정 2026-10-26 "GUI 초기 세팅 값은 0.2"). 저장하지 않음 — 켤 때마다 0.2
# 고정 카메라: movensys-manipulator 카메라 0 번 자세 — 시뮬레이션 depth_sim.R2_CAMERAS_WORLD[0] 와 같음 (서버가 켜지면 서버 값)
CAM_WORLD = (-0.402, -0.149, 0.947, -0.076, 1.017, 0.696)
WORLD_TO_BASE = (-0.03, 0.275, 0.05, 0.0, 0.0, -1.57)
SCENE_S = 0.05                  # 3D 다시 그리기 최소 간격 — 100 ms 틱에서 보므로 움직이는 동안 실제로는 틱마다 10 Hz ([R-1] GPU 3D 화면은 한 장 ~2 ms, 예전 ~0.1~0.5 s 라 0.25 = 4 Hz)
SCOPE_S = 10.0                  # 관절 그래프 길이
HOLD_JOG_MS = 50                # 누르는 동안 조그: 명령 간격 (서버는 0.4 s 안 오면 뗀 것으로 — rt_server.JOG_TIMEOUT_S)
HOLD_STALL_S = 0.3              # [2026-10-27 R-1 ①] 누르는 동안 조그: GUI 스레드 맥박이 이만큼 끊기면 (화면 멈춤 — 3D 그리기 등) 뗌을 보내고 끝냄
BEAT_MS = 50                    #  GUI 스레드 맥박 간격 (타이머가 _beat 를 새로 씀 — GUI 스레드가 멈추면 안 바뀜)
PING_S = 0.5                    # [2026-10-27 ①] GUI 살아 있음 — 화면 타이머가 보냄 (화면이 멈추면 끊김). 서버는 2 s 조용하면 로봇을 세움
FIRST_MSG_S = 5.0               # [R-1] 붙은 뒤 서버의 첫 메시지 (hello·refused) 를 기다리는 시간
RETRY_HOLD_S = 2.0              # [R-1] [↻ 다시 연결] 뒤 이만큼은 [▶ 시스템 시작] 을 안 받음 (두 번 누름이 같은 포트에 서버를 하나 더 띄우지 않게)
CAM_CHECK_S = 1.0               # [R-1] 고정 카메라 노드가 도는지 (PID 파일 + /proc) 는 이 간격으로만 새로 봄 (0.1 s 틱마다 읽지 않게)
VOX_STALE_S = 2.0               # [R-1 ④] 지도 칸이 이만큼 안 오면 '지도 안 옴' (서버는 0.5 s 마다 보냄 — rt_server.VOX_S)
# [R-1 ④] 거리·가상 카메라 속도: 서버가 쓰는 설정 (cr3a_rt_shaping/rt_config) 에서 — 전엔 화면 글에 6·4·2 cm, 15 Hz 를 박아 둠.
#  (정지·움직이는 물체 구분 없음 = 10-16 부터 늘 — rt_config 가 act_static = act_new 를 확인함)
_DIST = rt_config.Distances()
VCAM_FPS = rt_config.MapCfg().cam_fps
DIST_TEXT = (f'피하는 거리 {_DIST.act_new * 100:g} cm · 팔 지우기 {_DIST.mask * 100:g} cm · 바닥 {_DIST.act_floor * 100:g} cm '
             '(정지·움직이는 물체 구분 없음 — 시뮬레이션과 같은 값, rt_config)')
# [2026-10-27 사용자 "여러 자세를 넣고 지울 수 있게 만들어줘. 하드코딩 된 자세 두개는 지우고"] 자세 목록 = 영점 (cr3a_model.POSES) +
#  저장한 자세 (이 파일 {"poses": [{"name", "q_deg"}]}, 저장한 차례대로). 시험은 CR3A_POSES_FILE 로 다른 파일
POSES_FILE = Path(os.environ.get('CR3A_POSES_FILE', str(HERE / 'poses.json')))


def default_cam_T():
    """받침 ← 고정 카메라 광학 좌표 4x4 (서버가 켜지기 전 3D 표시용)."""
    Twb = cm.T_of(WORLD_TO_BASE[:3], WORLD_TO_BASE[3:])
    Tbc = np.linalg.inv(Twb) @ cm.T_of(CAM_WORLD[:3], CAM_WORLD[3:])
    T = Tbc.copy()
    T[:3, :3] = Tbc[:3, :3] @ LINK_TO_OPTICAL
    return T, Tbc


class ServerLink:
    """rt_server 와의 연결: 받기 스레드 → 큐 (GUI 타이머가 꺼냄). 끊기면 1 초마다 다시 붙어 봄 (서버가 늦게 떠도 됨).
    [2026-10-27 R-1] 서버는 붙자마자 hello (받음) 또는 refused (다른 GUI 가 살아 있어 거절 — rt_server Link._refuse) 를 보냄 → 첫 메시지를
    본 뒤에 'connected'. 거절이면 'refused' 만 알리고 다시 붙지 않음 — retry() ([↻ 다시 연결]) 를 기다림: 저절로 다시 붙으면 다른 GUI 가
    닫히는 순간 이 창이 말없이 서버를 넘겨받고, 그 GUI 를 다시 켜면 거꾸로 거절됨 (서버를 잡은 창을 사람이 고르게)."""

    def __init__(self, port=PORT):
        self.port = port
        self.q = queue.Queue()
        self.conn = None
        self.lock = threading.Lock()
        self._retry = threading.Event()     # 거절된 뒤 다시 붙기 ([↻ 다시 연결])
        threading.Thread(target=self._loop, daemon=True, name='server_link').start()

    def retry(self):
        """거절된 뒤 다시 붙어 봄 (사용자가 누름)."""
        self._retry.set()

    def _loop(self):
        while True:
            try:
                c = Client(('127.0.0.1', self.port), authkey=AUTH)
            except Exception:                           # 서버 없음·인증 실패 등: 1 초 뒤 다시 (스레드가 죽지 않게)
                time.sleep(1.0)
                continue
            try:                                        # 첫 메시지: hello 또는 refused (FIRST_MSG_S 안에 없으면 예전처럼 붙은 것으로)
                first = c.recv() if c.poll(FIRST_MSG_S) else None
            except (EOFError, OSError):                 # (보내지 않고 닫음 — 서버가 끝나는 중 등) 1 초 뒤 다시
                c.close()
                time.sleep(1.0)
                continue
            if isinstance(first, dict) and first.get('type') == 'refused':
                c.close()
                self._retry.clear()                     # (알리기 전에 — 알림을 보고 누른 [↻ 다시 연결] 을 잃지 않게)
                self.q.put(first)
                self._retry.wait()
                continue
            self.conn = c
            self.q.put({'type': 'connected'})
            if first is not None:
                self.q.put(first)
            try:
                while True:
                    self.q.put(c.recv())
            except Exception:
                pass
            self.conn = None
            self.q.put({'type': 'disconnected'})
            time.sleep(0.5)

    def send(self, m) -> bool:
        c = self.conn
        if c is None:
            return False
        try:
            with self.lock:
                c.send(m)
            return True
        except (OSError, EOFError, ValueError):
            return False

    def drain(self, limit=500):
        out = []
        while len(out) < limit:
            try:
                out.append(self.q.get_nowait())
            except queue.Empty:
                break
        return out


# ---------- 로그 ([2026-10-27 R-1 ⑥] logging 으로: 파일 + 로그 창. 단계 10·20·30·40 = DEBUG·INFO·WARNING·ERROR) ----------
def setup_file_log():
    """GUI 로그 파일 (common.GUI_LOG, 2 MB × 4 개 돌려 씀) — 프로세스에 한 번. 못 열면 stderr 에 알리고 파일 없이."""
    LOG.setLevel(logging.DEBUG)
    LOG.propagate = False
    if any(isinstance(h, RotatingFileHandler) for h in LOG.handlers):
        return
    try:
        GUI_LOG.parent.mkdir(parents=True, exist_ok=True)
        fh = RotatingFileHandler(GUI_LOG, maxBytes=2_000_000, backupCount=3, encoding='utf-8')
    except OSError as e:
        sys.stderr.write(f'cr3a_gui: 로그 파일을 못 엶 ({e!r}) — 로그 창에만\n')
        return
    fh.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
    LOG.addHandler(fh)


class _LogLine(QtCore.QObject):
    line = QtCore.pyqtSignal(str)


class ViewLogHandler(logging.Handler):
    """로그 창 (QPlainTextEdit) 으로: 시각 + 단계별 색, 글은 HTML 이스케이프 (서버 줄의 '<'·'&' 가 깨지지 않게). 다른 스레드의 로그도 신호로
    GUI 스레드에서 붙임. extra={'view': False} 인 줄 (예외의 자세한 내용) 은 파일에만."""

    def __init__(self, view):
        super().__init__(logging.DEBUG)
        self._sig = _LogLine()
        self._sig.line.connect(view.appendHtml)

    def emit(self, record):
        if not getattr(record, 'view', True):
            return
        try:
            t = getattr(record, 'stamp', None) or time.strftime('%H:%M:%S', time.localtime(record.created))
            color = LOG_COLORS.get(record.levelno, '#1f2328')
            self._sig.line.emit(f'<span style="color:#8c959f">{html.escape(t)}</span> '
                                f'<span style="color:{color}">{html.escape(record.getMessage())}</span>')
        except RuntimeError:                    # (창이 이미 지워짐)
            pass
        except Exception:
            self.handleError(record)


class JointBar(QtWidgets.QWidget):
    """관절 범위 막대: 회색 = 관절 한계, 파란 점 = 지금 각도 (미로봇 GUI 와 같음)."""

    def __init__(self, lo, hi):
        super().__init__()
        self.lo, self.hi, self.value = lo, hi, None
        self.setMinimumSize(110, 18)

    def set_value(self, v):
        if v != self.value:
            self.value = v
            self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        w, h = self.width(), self.height()

        def x(a):
            return 7 + (w - 14) * (a - self.lo) / (self.hi - self.lo)
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(QtGui.QColor('#d0d7de'))
        p.drawRoundedRect(QtCore.QRectF(7, h / 2 - 4, w - 14, 8), 4, 4)
        p.setPen(QtGui.QPen(QtGui.QColor('#57606a'), 1))
        p.drawLine(QtCore.QPointF(x(0), h / 2 - 7), QtCore.QPointF(x(0), h / 2 + 7))
        if self.value is not None:
            p.setPen(QtGui.QPen(QtGui.QColor('white'), 1.5))
            p.setBrush(QtGui.QColor(COLORS['info']))
            p.drawEllipse(QtCore.QPointF(x(min(max(self.value, self.lo), self.hi)), h / 2), 6, 6)


class MainWindow(QtWidgets.QMainWindow):
    hold_stalled = QtCore.pyqtSignal(float)    # [R-1 ①] 조그 스레드 → GUI 스레드: 화면이 멈춰 누르는 동안 조그를 뗐음 (멈춘 시간 s)

    def __init__(self):
        super().__init__()
        setup_file_log()
        self.robot = cm.Robot()
        fams = QtGui.QFontDatabase().families()
        self.mono = QtGui.QFont(next((f for f in ('D2Coding', 'NanumGothicCoding', 'Consolas') if f in fams), 'Monospace'))
        self.mono.setStyleHint(QtGui.QFont.Monospace)
        self.settings = QtCore.QSettings('cr3a', 'cr3a_gui')
        self.setWindowTitle('CR3A 제어 — 입력성형 실물 시험')
        # 서버 상태
        self.link = None
        self.connected = False
        self.hello = {}
        self.state = None
        self.server_proc = None
        self.t_starting = None              # [▶ 시스템 시작] 을 누른 시각 (연결될 때까지 '켜는 중')
        self.stopping = False
        self.refused_why = None             # [R-1] 서버가 이 창의 연결을 거절한 까닭 (다른 GUI 가 붙어 있음) — [↻ 다시 연결] 전까지 안 붙음
        self._t_retry = -1e9                #  마지막 [↻ 다시 연결] 시각 (monotonic)
        self.n_vox = 0
        self._t_vox = 0.0                   # 마지막 지도 칸 받은 시각 (time.time)
        self.last_event = ''
        self._t_scene = self._t_scope = 0.0
        self._scene_key = None
        self._render_pending = False        # [R-1] 3D 다시 그리기 요청 (request_render 가 0 ms 뒤 한 번으로 모음)
        self.near_q = None                  # 지도 점검한 자세 (rad) — 로봇이 움직이면 점검 점을 지움
        self.scope_buf = collections.deque()
        self.hold_jog = None                # 누르는 동안 조그 ('joint'|'cart', 축, 부호 붙은 속도, 기준) — 보내기는 따로 도는 스레드 (_hold_loop)
        self._hold_lock = threading.Lock()  # [R-1 ①] 'hold_jog 보고 보내기' ↔ '지우고 뗌 보내기' 가 섞이지 않게 — 전엔 뗌 뒤에 jog_hold 가 가서
                                            #  서버가 조그를 0.4 s 다시 움직일 수 있었음
        self._hold_wake = threading.Event() #  누르면 조그 스레드를 깨움 (스레드는 하나 — 빨리 다시 눌러도 둘이 안 생김)
        self._beat = time.monotonic()       #  GUI 스레드 맥박 (BEAT_MS 타이머가 새로 씀)
        self._hold_btn = None               #  누른 조그 단추 (맥박마다 아직 눌려 있나 봄 — released 가 안 오는 경우)
        self.cam_feed = None                # 고정 카메라 영상 (camera_io.CameraFeed, 처음 쓸 때 만듦)
        self._cam_pid, self._t_cam = None, -1e9     # 고정 카메라 노드 PID (CAM_CHECK_S 마다 새로 봄)
        self._cam_busy = False              # 카메라 끄는 중 (QThreadPool)
        self.calib_win = None               # 카메라 위치 맞추기 창
        self.cam_pose = cp.load_pose()      # 등록한 카메라 위치 (camera_pose.json)
        self._vcam_line = ''                # 카메라 세팅 탭: 가상 카메라 위치 줄 (_show_cam_pose)
        self._delay_cfg, self._delay_ms = None, 0      # 입력성형 지연 표시 (성형기가 바뀔 때만 build — 0.1 s 틱마다 안 함)
        self._t_prev = 0.0
        self._t_ping = 0.0
        self.poses, poses_err = self._load_poses()     # 저장한 자세 [(이름, (J1~J6 °))]
        self._build()
        self._view_log = ViewLogHandler(self.log_view)
        LOG.addHandler(self._view_log)
        if poses_err:
            self._log(40, poses_err)
        QtWidgets.QShortcut(QtGui.QKeySequence('Esc'), self, activated=self.on_estop,
                            context=QtCore.Qt.ApplicationShortcut)      # Esc = 비상정지만. [정지] 는 단축키 없음
        self.timer = QtCore.QTimer(self, timeout=self._tick)
        self._beat_timer = QtCore.QTimer(self, interval=BEAT_MS, timeout=self._on_beat)
        self._beat_timer.start()
        self.hold_stalled.connect(self._on_hold_stalled)
        threading.Thread(target=self._hold_loop, daemon=True, name='hold_jog').start()
        self._start()

    def _start(self):
        """창이 뜬 뒤: 저장한 설정·연결 전 표시 → 서버 연결 (없으면 1 초마다 다시 봄) → 0.1 s 틱. (시안은 gui_mock.MockWindow 가 바꿈)"""
        self._offline()
        self.link = ServerLink()
        self.timer.start(100)

    # ---------- 화면 만들기 ----------
    def _build(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(6, 4, 6, 4)
        root.setSpacing(4)
        root.addWidget(self._build_top())
        self.split = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        main = QtWidgets.QWidget()
        mh = QtWidgets.QHBoxLayout(main)
        mh.setContentsMargins(0, 0, 0, 0)
        self.view_tabs = QtWidgets.QTabWidget()
        self.view_tabs.addTab(self._build_scene(), '3D 뷰')
        self.scope = ScopePanel()
        self.scope.btn_run.toggled.connect(self._on_scope_run)
        self.view_tabs.addTab(self.scope, '관절 그래프')
        mh.addWidget(self.view_tabs, 3)
        right = QtWidgets.QWidget()
        rv = QtWidgets.QVBoxLayout(right)
        rv.setContentsMargins(0, 0, 0, 0)
        rv.setSpacing(4)
        self.shaping = ShapingPanel(self.mono)               # 전역 (탭 밖)
        self.shaping.apply_requested.connect(self.on_shaper_apply)
        rv.addWidget(self.shaping)
        self.tabs = QtWidgets.QTabWidget()
        jog_scroll = QtWidgets.QScrollArea(widgetResizable=True, frameShape=QtWidgets.QFrame.NoFrame)
        jog_scroll.setWidget(self._build_jog())
        self.tabs.addTab(jog_scroll, '조그')
        self.tabs.addTab(self._build_target(), '목표 이동')
        self.tabs.addTab(self._build_camera(), '카메라 세팅')
        rv.addWidget(self.tabs, 1)
        rv.addWidget(self._build_plan())
        right.setMinimumWidth(500)
        right.setMaximumWidth(560)
        mh.addWidget(right, 2)
        self.split.addWidget(main)
        self.log_view = QtWidgets.QPlainTextEdit(readOnly=True)
        self.log_view.setMaximumBlockCount(3000)
        self.log_view.setMinimumHeight(30)
        self.split.addWidget(self.log_view)
        self.split.setStretchFactor(0, 1)
        self.split.setStretchFactor(1, 0)
        self.split.setSizes([800, 78])
        root.addWidget(self.split, 1)

    @staticmethod
    def _set_pill(lb, text, color, tip=''):
        key = f'{text}\x1f{color}\x1f{tip}'
        if lb.property('pill') == key:                       # 0.1 초마다 부르므로 바뀔 때만 (스타일 다시 계산이 비쌈)
            return
        lb.setProperty('pill', key)
        lb.setText(text)
        lb.setStyleSheet(f'QLabel{{background:{color};color:white;border-radius:9px;padding:2px 9px;font-weight:bold;}}')
        lb.setToolTip(tip)

    def _build_top(self):
        bar = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(bar)
        h.setContentsMargins(0, 0, 0, 0)
        self.chips = {}
        for key, name in (('robot', '로봇'), ('wmx', 'WMX'), ('sys', '시스템'), ('cam', '카메라'), ('obst', '장애물 회피')):
            h.addWidget(QtWidgets.QLabel(name))
            lb = QtWidgets.QLabel('—')
            self.chips[key] = lb
            h.addWidget(lb)
            h.addSpacing(6)
        h.addSpacing(10)
        h.addWidget(QtWidgets.QLabel('대상'))
        self.cb_target = QtWidgets.QComboBox()
        self.cb_target.addItems(list(TARGETS))
        self.cb_target.setToolTip('시스템을 켤 때 무엇을 움직일지. 시스템이 켜져 있는 동안은 못 바꿈.\n'
                                  '이상적 시뮬레이션 = 소프트웨어 로봇 + 가상 카메라 (WMX·실제 로봇 없음)\n'
                                  'WMX 시뮬레이터 = R2 → WMX 시뮬레이터 (Module.ini 가 시뮬레이터뿐인지 확인하고 켬, 실제 로봇 안 움직임),\n'
                                  '  WMX 축 로그·실행 기록을 ~/workspaces/wmx_r2_logs/gui_wmx_<시각> 에 남김\n'
                                  '실제 로봇 (EtherCAT) = R2 → WMX → CR3A. 켜면 서보 ON, 실시간 회피는 꺼진 채로 시작 (명령할 때만 움직임).\n'
                                  '  Module.ini 를 EtherCAT 으로 바꿔 켜고 끌 때 다시 시뮬레이터로. 기록 gui_real_<시각>')
        self.cb_target.currentIndexChanged.connect(          # 켜기 전 표시: 실물은 실시간 회피 끔으로 시작 (켜지면 서버 값으로 맞춤)
            lambda i: None if self.connected else self._set_rt_check(TARGET_ROBOT.get(i) != 'real'))
        h.addWidget(self.cb_target)
        self.btn_stack = QtWidgets.QPushButton('▶ 시스템 시작', clicked=self.on_toggle_stack)
        self.btn_stack.setToolTip('실시간 회피 서버 (계획·MPC·지도·입력성형) 를 켜고 끕니다. 켜진 채 창을 닫으면 끌지 묻고,\n'
                                  '그냥 닫으면 서버는 계속 돌되 로봇을 세우고 실시간 회피를 끕니다 (GUI 가 2 초 넘게 멈춰도 같음).')
        self.btn_servo = QtWidgets.QPushButton('서보 끄기', clicked=self.on_servo)
        self.btn_servo.setToolTip('CR3A 6 축 서보 켜기/끄기 = R2 하드웨어 활성/비활성 (끄면 버퍼 중단 → 정지 → 서보 OFF,\n'
                                  '켜면 알람 해제 → 서보 ON → 명령을 지금 위치로). 시스템을 켜면 ON, 끄면 OFF.\n'
                                  '움직이는 중에는 안 됨. 절대 엔코더라 호밍은 없음. 이상적 시뮬레이션에는 없음.')
        self.btn_alarm = QtWidgets.QPushButton('알람 해제', clicked=self.on_alarm)
        self.btn_alarm.setToolTip('R2 하드웨어를 다시 켬 (끔 → 켬 = 알람 해제 → 서보 ON → 명령을 지금 위치로).\n'
                                  'R2 오류 (예: 가속 한계 초과) 로 비상정지된 뒤에 씀. 이상적 시뮬레이션에는 없음.')
        for b in (self.btn_stack, self.btn_servo, self.btn_alarm):
            h.addWidget(b)
        h.addStretch(1)
        menu = QtWidgets.QMenu(self)
        act_log = menu.addAction('로그 크게 보기')
        act_log.setCheckable(True)
        act_log.toggled.connect(lambda on: self.split.setSizes([560, 300] if on else [800, 78]))
        menu.addSeparator()
        menu.addAction('시험: 가상 물체 움직이기 시작', lambda: self._cmd({'cmd': 'objects', 'go': True}))
        menu.addAction('시험: 가상 물체 치우기', lambda: self._cmd({'cmd': 'objects', 'go': False}))
        menu.addSeparator()
        menu.addAction(f'서버 로그 위치: {SERVER_LOG}', lambda: self._log(20, f'서버 로그: {SERVER_LOG}'))
        btn_menu = QtWidgets.QToolButton()
        btn_menu.setText('⚙ 설정')
        btn_menu.setPopupMode(QtWidgets.QToolButton.InstantPopup)
        btn_menu.setMenu(menu)
        h.addWidget(btn_menu)
        h.addSpacing(10)
        # [사용자 결정 2026-10-26] 정지 둘: [정지] = 입력성형을 거쳐 섬 (단축키 없음) / [비상정지] = Esc, 즉시 멈춤 (성형 건너뜀)
        self.btn_stop = QtWidgets.QPushButton('정지', clicked=self.on_stop)
        self.btn_stop.setFixedSize(84, 46)
        self.btn_stop.setStyleSheet('QPushButton{background:#9a6700;color:white;font-size:15px;font-weight:bold;'
                                    'border-radius:8px;} QPushButton:pressed{background:#7d4e00;}')
        self.btn_stop.setToolTip('움직임을 멈춥니다 — 다른 움직임처럼 입력성형을 거쳐 감속 정지 (진동 없음).\n'
                                 '성형기 길이만큼 늦게 섭니다. 단축키 없음.\n'
                                 '조그 중이면 MoveIt Servo 가 가속 한계 안에서 감속 정지 (조그는 입력성형 없음).\n'
                                 '실시간 회피가 켜져 있으면 그 자리에 서서 지킵니다 (다가오는 물체는 피함).')
        h.addWidget(self.btn_stop)
        h.addSpacing(6)
        self.btn_estop = QtWidgets.QPushButton('비상정지\nEsc', clicked=self.on_estop)
        self.btn_estop.setFixedSize(104, 46)
        self.btn_estop.setStyleSheet('QPushButton{background:#cf222e;color:white;font-size:15px;font-weight:bold;'
                                     'border-radius:8px;} QPushButton:pressed{background:#a40e26;}')
        self.btn_estop.setToolTip('즉시 멈춥니다 — 입력성형을 건너뛰고 WMX 감속 정지 (서는 순간 진동이 생길 수 있음).\n'
                                  '보낸 명령을 모두 버리고 실시간 회피도 끕니다 (다시 켜야 움직임). 서보는 켜진 채.\n'
                                  '성형기 기억은 멈춘 자세로 다시 시작합니다.\n'
                                  '소프트웨어 정지입니다 — 진짜 비상시에는 로봇의 비상 정지 스위치.')
        h.addWidget(self.btn_estop)
        return bar

    def _build_scene(self):
        box = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(box)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(3)
        self.scene = Scene3D(self.robot)
        v.addWidget(self.scene, 1)
        row = QtWidgets.QHBoxLayout()
        for name in PRESETS:
            b = QtWidgets.QPushButton(name, clicked=lambda _, n=name: self.scene.set_preset(n))
            b.setFixedWidth(44)
            row.addWidget(b)
        b = QtWidgets.QPushButton('카메라 시점', clicked=self.scene.set_camera_mode)
        b.setToolTip('고정 카메라가 보는 방향에서 본 모습. 카메라 위치 숫자가 맞으면 카메라 영상과 같은 구도입니다.')
        row.addWidget(b)
        row.addStretch(1)
        for key, text, tip, on in (('obst', '장애물', '카메라 지도의 물체 칸 (파랑 = 낮음 → 빨강 = 높음)', True),
                                   ('cam', '카메라', '고정 카메라 위치와 시야', True),
                                   ('plan', '경로', '계획한 공구 끝 경로 (주황)', True),
                                   ('u', '성형 전 명령', '입력성형 전에 보낸 명령 자세 (보라 반투명). 실제 로봇은 이것을 늦게 따라가고, 끝은 지연 시간만큼 늦게 섬', False)):
            c = QtWidgets.QCheckBox(text, checked=on)
            c.setToolTip(tip)
            c.toggled.connect(lambda val, k=key: self.scene.set_flag(k, val))
            row.addWidget(c)
        v.addLayout(row)
        return box

    def _jog_buttons(self, grid, row, col, kind, axis):
        """[−] [+] 한 쌍. kind = 'joint' (axis = 관절 번호) / 'cart' (axis = 'X'…'Rz')."""
        for k, (text, sign) in enumerate((('−', -1), ('+', +1))):
            b = QtWidgets.QPushButton(text)
            b.setFixedSize(34, 22)
            b.clicked.connect(lambda _, s=sign: self._jog_click(kind, axis, s))
            b.pressed.connect(lambda s=sign, b=b: self._jog_press(kind, axis, s, b))
            b.released.connect(self._jog_release)
            grid.addWidget(b, row, col + k)

    def _value_label(self, width=78):
        val = QtWidgets.QLabel('—')
        val.setFont(self.mono)
        val.setMinimumWidth(width)
        val.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        return val

    @staticmethod
    def _combo(items, current, tip=''):
        cb = QtWidgets.QComboBox()
        cb.addItems(items)
        cb.setCurrentText(current)
        cb.setToolTip(tip)
        return cb

    def _build_jog(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setSpacing(4)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel('<b>방식</b>'))
        self.rb_step = QtWidgets.QRadioButton('한 칸씩 (계획 → 실행)', checked=True)
        self.rb_hold = QtWidgets.QRadioButton('누르는 동안')
        self.rb_step.setToolTip('누를 때마다 한 칸 (MoveIt Servo, 입력성형 없음). 관절은 그 관절만, 공구 끝은 곧은 선 (돌기는 공구 끝 점 둘레).\n'
                                '속도 = 관절 10°/s · 공구 끝 50 mm/s · 회전 10°/s × 속도 배율, 끝에서 줄이며 도착.')
        self.rb_hold.setToolTip('누르고 있는 동안 고른 속도로 계속 움직이고, 떼면 멈춥니다 (MoveIt Servo, 입력성형 없음 —\n'
                                '가속 한계 안에서 감속해 바로 섬).')
        self.rb_hold.toggled.connect(self._on_jog_mode)
        for rb in (self.rb_step, self.rb_hold):
            row.addWidget(rb)
        row.addStretch(1)
        v.addLayout(row)
        g = QtWidgets.QGridLayout()
        g.setVerticalSpacing(2)
        self.lb_jstep = QtWidgets.QLabel('<b>관절</b> 간격')
        g.addWidget(self.lb_jstep, 0, 0, 1, 2)
        self.cb_jstep = self._combo(['0.5', '1', '2', '5', '10'], '1', '한 칸 (°). 누르는 동안 방식에서는 속도 (°/s)')
        r = QtWidgets.QHBoxLayout()
        r.addWidget(self.cb_jstep)
        self.lb_junit = QtWidgets.QLabel('°')
        r.addWidget(self.lb_junit)
        r.addStretch(1)
        g.addLayout(r, 0, 2)
        self.joint_val, self.joint_bar = [], []
        for k in range(6):
            lo, hi = self.robot.limits_deg[k]
            g.addWidget(QtWidgets.QLabel(f'J{k + 1}'), k + 1, 0)
            val = self._value_label()
            bar = JointBar(lo, hi)
            bar.setToolTip(f'관절 범위 {lo:.0f}° ~ {hi:.0f}°')
            g.addWidget(val, k + 1, 1)
            g.addWidget(bar, k + 1, 2)
            self._jog_buttons(g, k + 1, 3, 'joint', k)
            self.joint_val.append(val)
            self.joint_bar.append(bar)
        g.setColumnStretch(2, 1)
        v.addLayout(g)
        line = QtWidgets.QFrame(frameShape=QtWidgets.QFrame.HLine, frameShadow=QtWidgets.QFrame.Sunken)
        v.addWidget(line)
        r = QtWidgets.QHBoxLayout()
        self.lb_cstep = QtWidgets.QLabel('<b>공구 끝</b> 간격')
        r.addWidget(self.lb_cstep)
        self.cb_cstep = self._combo(['1', '2', '5', '10', '20', '50'], '5', '한 칸 (mm). 누르는 동안 방식에서는 속도 (mm/s)')
        r.addWidget(self.cb_cstep)
        self.lb_cunit = QtWidgets.QLabel('mm  ·  회전')
        r.addWidget(self.lb_cunit)
        self.cb_rstep = self._combo(['0.5', '1', '2', '5', '10'], '2', '한 칸 (°). 누르는 동안 방식에서는 속도 (°/s)')
        r.addWidget(self.cb_rstep)
        self.lb_runit = QtWidgets.QLabel('°')
        r.addWidget(self.lb_runit)
        r.addStretch(1)
        r.addWidget(QtWidgets.QLabel('기준'))
        self.cb_frame = self._combo(['로봇 받침', '공구'], '로봇 받침', '로봇 받침: X 앞, Y 왼쪽, Z 위.\n공구: 공구 끝 자신의 축 방향으로 움직이고 돕니다.')
        r.addWidget(self.cb_frame)
        v.addLayout(r)
        g = QtWidgets.QGridLayout()
        g.setVerticalSpacing(2)
        g.setHorizontalSpacing(4)
        self.cart_val, self.rot_val = [], []
        for k, (n, rn) in enumerate((('X', 'Rx'), ('Y', 'Ry'), ('Z', 'Rz'))):
            g.addWidget(QtWidgets.QLabel(n), k, 0)
            val = self._value_label(84)
            g.addWidget(val, k, 1)
            self._jog_buttons(g, k, 2, 'cart', n)
            g.addItem(QtWidgets.QSpacerItem(18, 1), k, 4)
            lb = QtWidgets.QLabel(rn)
            lb.setToolTip(RPY_TIP)
            g.addWidget(lb, k, 5)
            rv = self._value_label(84)
            rv.setToolTip(RPY_TIP)
            g.addWidget(rv, k, 6)
            self._jog_buttons(g, k, 7, 'cart', rn)
            self.cart_val.append(val)
            self.rot_val.append(rv)
        g.setColumnStretch(9, 1)
        v.addLayout(g)
        note = QtWidgets.QLabel(f'{TIP_TEXT}.<br>조그는 <b>입력성형 없이 MoveIt Servo</b> 로 움직입니다 (입력성형은 목표 이동·반복에만). '
                                f'특이점·관절 한계·충돌 가까이에서는 Servo 가 늦추거나 멈춥니다 — 충돌은 MoveIt 장면 (로봇·테이블·Thor 상자) 만 보고 '
                                f'카메라 지도는 안 봅니다. WMX 시뮬레이터·실물에서만 (이상적 모드에는 없음).')
        note.setWordWrap(True)
        note.setStyleSheet(f'color:{COLORS["off"]}')
        v.addWidget(note)
        v.addStretch(1)
        return w

    def _spin(self, lo, hi, step=1.0, dec=1, prefix='', suffix=''):
        sp = QtWidgets.QDoubleSpinBox(decimals=dec, singleStep=step, prefix=prefix, suffix=suffix)
        sp.setRange(lo, hi)
        sp.setFont(self.mono)
        return sp

    def _move_repeat(self, grid, row, on_move, on_repeat):
        r = QtWidgets.QHBoxLayout()
        b_move = QtWidgets.QPushButton('이동', clicked=lambda: on_move())
        b_move.setToolTip('계획이 나오면 바로 움직입니다 (장애물 지도의 물체를 피함, 입력성형 거침).\n'
                          '아래 "실시간 회피" 가 켜져 있으면 움직이는 동안에도 다가오는 물체를 피합니다.')
        b_move.setStyleSheet('QPushButton:enabled{font-weight:bold;}')
        b_rep = QtWidgets.QPushButton('반복', clicked=lambda: on_repeat())
        b_rep.setToolTip('누른 순간의 지금 위치 ↔ 목표를 계속 왕복합니다. [정지] 로 멈춤 (성형 거쳐 섬, Esc = 비상정지).')
        for b in (b_move, b_rep):
            b.setFixedHeight(26)
            r.addWidget(b)
        grid.addLayout(r, row, 0, 1, 3)

    def _build_target(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setSpacing(4)
        gb = QtWidgets.QGroupBox('관절각으로')
        g = QtWidgets.QGridLayout(gb)
        g.setVerticalSpacing(3)
        self.sp_target = []
        for k in range(6):
            lo, hi = self.robot.limits_deg[k]
            sp = self._spin(math.ceil(lo), math.floor(hi), prefix=f'J{k + 1}  ', suffix='°', dec=2)
            g.addWidget(sp, k // 3, k % 3)
            self.sp_target.append(sp)
        r = QtWidgets.QHBoxLayout()
        r.addWidget(QtWidgets.QPushButton('지금 자세로 채우기', clicked=self._fill_current))
        r.addStretch(1)
        g.addLayout(r, 2, 0, 1, 3)
        # [2026-10-27 사용자 "여러 자세를 넣고 지울 수 있게"] 자세 목록 = 영점 (늘 있음) + 저장한 자세 (POSES_FILE)
        r = QtWidgets.QHBoxLayout()
        r.addWidget(QtWidgets.QLabel('자세 목록'))
        self.cb_pose = self._combo([], '', f'저장해 둔 자세 (영점은 늘 있음). [채우기] 는 J1~J6 칸만 채웁니다 (바로 움직이지 않음).\n'
                                           f'저장한 자세는 {POSES_FILE} 에 남습니다.')
        self.cb_pose.currentIndexChanged.connect(lambda _: self._on_pose_pick())
        r.addWidget(self.cb_pose, 1)
        r.addWidget(QtWidgets.QPushButton('채우기', clicked=lambda: self._fill_pose()))
        b = QtWidgets.QPushButton('저장…', clicked=self._save_pose)
        b.setToolTip('지금 J1~J6 칸의 각도를 이름을 붙여 목록에 넣습니다 (같은 이름이 있으면 바꿀지 물음).\n'
                     '지금 로봇 자세를 넣으려면 [지금 자세로 채우기] 뒤에 [저장…]')
        r.addWidget(b)
        self.btn_pose_del = QtWidgets.QPushButton('삭제', clicked=self._delete_pose)
        self.btn_pose_del.setToolTip('고른 자세를 목록에서 지웁니다 (물어본 뒤). 영점은 못 지움')
        r.addWidget(self.btn_pose_del)
        g.addLayout(r, 3, 0, 1, 3)
        self._refresh_poses()
        self._move_repeat(g, 4, self.on_goto, self.on_repeat)
        v.addWidget(gb)
        gb = QtWidgets.QGroupBox('공구 끝으로')
        gb.setToolTip(TIP_TEXT)
        g = QtWidgets.QGridLayout(gb)
        g.setVerticalSpacing(3)
        self.sp_xyz = [self._spin(-900, 900, prefix=f'{n}  ', suffix=' mm') for n in 'XYZ']
        for k, sp in enumerate(self.sp_xyz):
            sp.setToolTip({0: 'X: 로봇 앞 +', 1: 'Y: 로봇 왼쪽 +', 2: 'Z: 위 + (받침 바닥 = 0)'}[k])
            g.addWidget(sp, 0, k)
        self.sp_rpy = [self._spin(-180, 180, prefix=f'{n}  ', suffix='°') for n in ('Rx', 'Ry', 'Rz')]
        for k, sp in enumerate(self.sp_rpy):
            sp.setWrapping(True)
            sp.setToolTip(RPY_TIP)
            sp.setEnabled(False)
            g.addWidget(sp, 1, k)
        r = QtWidgets.QHBoxLayout()
        self.chk_keep_rot = QtWidgets.QCheckBox('방향은 지금 그대로', checked=True)
        self.chk_keep_rot.setToolTip('체크: Rx Ry Rz 칸은 지금 공구 방향을 보여 주기만 하고, 이동할 때 방향을 바꾸지 않습니다.')
        self.chk_keep_rot.toggled.connect(lambda on: [sp.setEnabled(not on) for sp in self.sp_rpy])
        r.addWidget(self.chk_keep_rot)
        r.addWidget(QtWidgets.QPushButton('지금 위치로 채우기', clicked=self._fill_xyz), 1)
        g.addLayout(r, 2, 0, 1, 3)
        self._move_repeat(g, 3, lambda: self._pose_cmd('goto_pose'), lambda: self._pose_cmd('repeat_pose'))
        v.addWidget(gb)
        self.chk_rt = QtWidgets.QCheckBox('실시간 회피 (움직이는 중에도 · 멈춰 있을 때도 다가오는 물체를 피함)', checked=True)
        self.chk_rt.setStyleSheet('QCheckBox{font-weight:bold;}')
        self.chk_rt.setToolTip('켜면 20 Hz 로 계속 다시 계산하며 로봇을 맡습니다.\n'
                               '  · 멈춰 있을 때: 그 자리를 지키다 물체가 다가오면 비켰다가 돌아옵니다 (로봇이 스스로 움직임).\n'
                               '  · [이동]: 경로를 따라가며 다가오는 물체를 피하고, 막히면 다시 경로를 세웁니다.\n'
                               '  · [반복]: 도착하면 잠깐 서고 바로 다음 구간.\n'
                               '끄면 지도가 그 순간에 멈추고 (계획 직전에만 새로 봄), 서 있을 때 명령을 보내지 않습니다 (피하지 않음).\n'
                               '비상정지를 누르면 꺼집니다.\n'
                               '켤 때: 로봇이 서 있어야 하고, 먼저 지도를 1.2 초 새로 쌓아 로봇 겉면 5.5 cm 안에 지도 점이 있으면 켜지 않습니다\n'
                               '  (그 점 = 3D 화면 자홍 점). 피하는 움직임도 속도 배율을 따릅니다 (속도·가속 한계 × 배율).')
        self.chk_rt.toggled.connect(self.on_realtime)
        r = QtWidgets.QHBoxLayout()
        r.addWidget(self.chk_rt, 1)
        btn = QtWidgets.QPushButton('지도 점검', clicked=lambda: self._cmd({'cmd': 'map_check'}))
        btn.setToolTip('실시간 회피를 켜지 않고, 지금 자세에서 로봇 겉면 5.5 cm 안에 지도 점이 있는지만 봅니다 (로봇은 안 움직임,\n'
                       '서보가 꺼져 있어도 됨). 있으면 3D 화면에 자홍 점으로 — 실시간 회피를 켜면 MPC 가 그 점을 피하려 움직입니다.\n'
                       '팔에 묶은 케이블·팔 옆 물체를 정리한 뒤 확인하는 용도.')
        r.addWidget(btn)
        v.addLayout(r)
        v.addStretch(1)
        return w

    def _build_camera(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setSpacing(6)
        gb = QtWidgets.QGroupBox('카메라 (장애물 인식용 · 고정)')
        g = QtWidgets.QVBoxLayout(gb)
        r = QtWidgets.QHBoxLayout()
        self.btn_cam_on = QtWidgets.QPushButton('▶ 카메라 켜기', clicked=self.on_cam_on)
        self.btn_cam_on.setToolTip(f'전체를 보는 고정 카메라 (D455 {cp.CAM_SERIAL}) 노드를 켭니다 (GUI 를 닫아도 계속 — 서버가 씀).\n'
                                   '손목 카메라는 지금 안 씀.')
        self.btn_cam_off = QtWidgets.QPushButton('■ 끄기', clicked=self.on_cam_off)
        r.addWidget(self.btn_cam_on)
        r.addWidget(self.btn_cam_off)
        r.addStretch(1)
        # 기본값 = 시뮬레이션에서 시험한 값 (rt_config cam_fps 15, depth_sim 640x480 — R2 카메라와 같게). 미로봇 실물은 848x480 · 15 fps
        self.cb_res = self._combo(['640 × 480', '848 × 480', '1280 × 720'], '640 × 480', '깊이 해상도 (켤 때 적용)')
        self.cb_fps = self._combo(['15 fps', '30 fps', '60 fps'], '15 fps',
                                  '깊이 영상 속도 (켤 때 적용). 시뮬레이션은 15 fps 로 시험했음.\n'
                                  '높이면 물체를 더 빨리 보지만 지도 계산이 늘어 제어 주기가 늦어질 수 있고,\n'
                                  '지도 설정 중 영상 장 수로 정한 것 (사라진 물체 지우기·깜빡임 거르기) 의 시간이 짧아짐')
        r.addWidget(self.cb_res)
        r.addWidget(self.cb_fps)
        g.addLayout(r)
        r = QtWidgets.QHBoxLayout()
        self.lb_preview = QtWidgets.QLabel(alignment=QtCore.Qt.AlignCenter)
        self.lb_preview.setFixedSize(212, 120)
        self.lb_preview.setStyleSheet('border:1px solid #d0d7de;color:#6e7781;')
        r.addWidget(self.lb_preview)
        self.lb_caminfo = QtWidgets.QLabel(wordWrap=True, textFormat=QtCore.Qt.RichText)
        self.lb_caminfo.setAlignment(QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft)
        r.addWidget(self.lb_caminfo, 1)
        g.addLayout(r)
        self.lb_campose = QtWidgets.QLabel(wordWrap=True, textFormat=QtCore.Qt.RichText)
        self.lb_campose.setFont(self.mono)
        g.addWidget(self.lb_campose)
        b = QtWidgets.QPushButton('카메라 위치 맞추기…', clicked=self.on_calib)
        b.setToolTip('카메라 영상 위에 로봇 모양을 겹쳐 보면서 카메라 위치 (받침 → 카메라) 를 맞추고 저장하는 보정용 창.\n'
                     '로봇을 여러 자세로 옮겨 가며 자세마다 보정 → 여러 자세 함께 맞추기 → 등록. 카메라를 옮겼을 때만 필요합니다.')
        g.addWidget(b)
        v.addWidget(gb)
        gb = QtWidgets.QGroupBox('장애물 지도')
        g = QtWidgets.QVBoxLayout(gb)
        r = QtWidgets.QHBoxLayout()
        r.addWidget(QtWidgets.QLabel('지도 카메라'))
        self.cb_mapcam = self._combo(['가상 카메라 (시뮬레이션)', '실제 카메라 (고정 D455)'],
                                     self.settings.value('mapcam', '가상 카메라 (시뮬레이션)'),
                                     '시스템을 켤 때 지도를 무엇으로 쌓을지 (켜져 있는 동안은 못 바꿈).\n'
                                     '실제 카메라: 등록한 카메라 위치로 실제 깊이 영상을 쌓음. 팔 지우기 = 영상이 찍힌 순간의 관절각.\n'
                                     '  시뮬레이션 로봇 (이상적·WMX 시뮬레이터) 이면 실제 팔은 안 움직이므로 마지막 [엔코더 읽기] 자세로 지움.')
        self.cb_mapcam.currentTextChanged.connect(lambda t: self.settings.setValue('mapcam', t))
        r.addWidget(self.cb_mapcam, 1)
        g.addLayout(r)
        # [R-1 ④] 칩은 실제 상태로 (_update_chips): 카메라 (가상·실제, fps) · 지도 (쌓는 중·멈춤·안 옴). 늘 같던 '팔 지우기'·'계획이 지도 읽음'
        #  칩, 숨겨 둔 [장애물 회피 켜기] (실시간 회피 체크와 겹침), 안 이어진 '바닥 면도 보이기' (서버가 바닥 띠 칸은 안 보냄) 는 뺌
        r = QtWidgets.QHBoxLayout()
        self.st_chips = {}
        for key in ('cam', 'map'):
            lb = QtWidgets.QLabel('—')
            self.st_chips[key] = lb
            r.addWidget(lb)
        r.addStretch(1)
        g.addLayout(r)
        self.lb_map = QtWidgets.QLabel(textFormat=QtCore.Qt.RichText)
        g.addWidget(self.lb_map)
        self.lb_dist = QtWidgets.QLabel(DIST_TEXT, textFormat=QtCore.Qt.RichText, wordWrap=True)
        self.lb_dist.setStyleSheet(f'color:{COLORS["off"]}')
        g.addWidget(self.lb_dist)
        r = QtWidgets.QHBoxLayout()
        b = QtWidgets.QPushButton('지도 새로 쌓기', clicked=lambda: self._cmd({'cmd': 'map_reset'}))
        b.setToolTip('지도를 비우고 처음부터 다시 쌓습니다 (몇 초). 물체를 치우거나 옮겼을 때. 멈춰 있을 때만.')
        r.addWidget(b)
        r.addStretch(1)
        g.addLayout(r)
        v.addWidget(gb)
        v.addStretch(1)
        return w

    def _build_plan(self):
        box = QtWidgets.QGroupBox('계획 · 실행')
        v = QtWidgets.QVBoxLayout(box)
        v.setSpacing(2)
        v.setContentsMargins(6, 4, 6, 4)
        self.lb_reason = QtWidgets.QLabel(wordWrap=True, textFormat=QtCore.Qt.RichText)
        v.addWidget(self.lb_reason)
        self.lb_plan = QtWidgets.QLabel(wordWrap=True, textFormat=QtCore.Qt.RichText)
        v.addWidget(self.lb_plan)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel('속도 배율'))
        self.sl_speed = QtWidgets.QSlider(QtCore.Qt.Horizontal, minimum=1, maximum=100, value=SPEED0)
        self.sl_speed.setToolTip('속도 배율 = 로봇 명령의 속도·가속 한계 × 배율 (URDF 한계를 낮춘 것과 같음, 저크 한계는 그대로).\n'
                                 '목표 이동·반복: 1.0 = 계획기가 로봇 한계의 95 % 로 낸 경로 그대로. 1 보다 작으면 같은 길을 낮춘 한계 안에\n'
                                 '드는 만큼 고르게 늘림 (낮은 배율에서는 최고 속도가 한계 × 배율에 닿음). 실시간 회피 (피하기·비키기) 도 같은 한계.\n'
                                 '한 칸 조그는 최고 속도 × 배율. 입력성형은 그 뒤에 그대로 (성형된 시간 = 늘어난 시간 + 성형 지연 시간).\n'
                                 '실시간 회피 끔의 지키기·누르는 동안 조그 (°/s 를 고름)·안전 검사 감속·비상정지는 한계 그대로.\n'
                                 '바꾸면 다음 이동부터 (움직이는 중이면 지금 이동은 그대로). 켤 때마다 0.20 으로 시작.')
        self.lb_speed = QtWidgets.QLabel(f'{SPEED0 / 100:.2f}')
        self.lb_speed.setFont(self.mono)
        self.sl_speed.valueChanged.connect(lambda val: self.lb_speed.setText(f'{val / 100:.2f}'))
        self._speed_timer = QtCore.QTimer(self, singleShot=True, interval=400, timeout=self._send_speed)
        self.sl_speed.valueChanged.connect(lambda _: self._speed_timer.start())     # 끌고 있는 동안은 보내지 않음 (멈춘 뒤 0.4 s)
        row.addWidget(self.sl_speed, 3)
        row.addWidget(self.lb_speed)
        v.addLayout(row)
        self.lb_exec = QtWidgets.QLabel()
        self.lb_exec.setStyleSheet(f'color:{COLORS["off"]}')
        v.addWidget(self.lb_exec)
        return box

    # ---------- 로그·명령 ----------
    def _log(self, level, text, stamp=None):
        """로그 창 + 파일 (logging). level 10·20·30·40 = DEBUG·INFO·WARNING·ERROR, stamp = 로그 창에 보일 시각 (없으면 지금)."""
        LOG.log(level, text, extra={'stamp': stamp})

    def _cmd(self, m, quiet=False) -> bool:
        if not (self.connected and self.link.send(m)):
            if not quiet:
                self._log(30, '시스템이 꺼져 있음 — [▶ 시스템 시작] 을 먼저')
            return False
        return True

    def _send_speed(self):
        """속도 배율 → 서버 (시스템이 꺼져 있으면 켤 때 --speed 로 넘김). 서버는 다음 이동부터 씀."""
        if self.connected:
            self._cmd({'cmd': 'speed', 'scale': self.sl_speed.value() / 100}, quiet=True)

    def _set_speed_slider(self, scale):
        """서버가 알려 준 속도 배율로 슬라이더를 맞춤 (보내기 없이)."""
        v = int(round(float(scale) * 100))
        if v != self.sl_speed.value():
            with QtCore.QSignalBlocker(self.sl_speed):
                self.sl_speed.setValue(v)
            self.lb_speed.setText(f'{v / 100:.2f}')

    def _ready(self) -> bool:
        if not (self.connected and self.state and self.state.get('ready')):
            self._log(30, '시스템이 아직 준비 안 됨' if self.connected else '시스템이 꺼져 있음 — [▶ 시스템 시작] 을 먼저')
            return False
        return True

    def closeEvent(self, ev):
        """[2026-10-27 ①] 시스템이 켜져 있으면 끌지 물음. 그냥 닫아도 서버가 GUI 끊김을 보고 로봇을 세우고 실시간 회피를 끔 (서보는 그대로,
        GUI 를 다시 켜면 이어서 조작). [R-1 ③] 닫기로 하면 보정 창이 열려 있어도 프로그램을 끝냄 (_shutdown)."""
        if self.connected or self.t_starting is not None:
            box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Question, '창 닫기',
                                        '시스템이 켜져 있습니다.\n\n'
                                        '[시스템 끄고 닫기]  로봇을 세우고 서보 OFF → R2·서버를 끔\n'
                                        '[그냥 닫기]  서버는 계속 — 로봇은 세우고 실시간 회피 끔 (GUI 를 다시 켜면 이어서 조작)', parent=self)
            b_off = box.addButton('시스템 끄고 닫기', QtWidgets.QMessageBox.AcceptRole)
            b_keep = box.addButton('그냥 닫기', QtWidgets.QMessageBox.DestructiveRole)
            box.addButton('취소', QtWidgets.QMessageBox.RejectRole)
            box.exec_()
            if box.clickedButton() is b_off:
                if self.connected:
                    self._cmd({'cmd': 'quit'})
                elif self.server_proc is not None and self.server_proc.poll() is None:
                    self.server_proc.terminate()            # 켜는 중: SIGTERM → 서버가 저장·R2 끄기 (finally)
            elif box.clickedButton() is not b_keep:
                ev.ignore()
                return
        self._shutdown()
        ev.accept()

    def _shutdown(self):
        """[R-1 ③] 창을 닫기로 한 뒤: 누르는 동안 조그를 떼고, 타이머 (ping 포함) 를 멈추고, 보정 창을 닫고 프로그램을 끝냄. 전엔 보정 창 (따로
        뜬 최상위 창) 이 열려 있으면 앱이 계속 돌며 ping 을 보내 서버가 GUI 끊김을 못 봤음 (로봇을 안 세움). 연결은 프로세스가 끝날 때 닫힘."""
        self._jog_release()
        for t in (self.timer, self._beat_timer, self._speed_timer):
            t.stop()
        if self.calib_win is not None:
            self.calib_win.close()
        LOG.removeHandler(self._view_log)
        QtWidgets.QApplication.quit()

    # ---------- 버튼 ----------
    def on_toggle_stack(self):
        if self.refused_why is not None:            # [R-1] 거절된 뒤 = [↻ 다시 연결]: 같은 포트의 서버가 이미 돎 — 새로 켜지 않고 다시 붙어 봄
            self.refused_why, self._t_retry = None, time.monotonic()
            self._log(20, '서버에 다시 붙어 봄')
            self.link.retry()
            return
        if self.connected:
            self.stopping = True
            self._cmd({'cmd': 'quit'})
            self._log(20, '시스템 끄는 중 (로봇을 세운 뒤 서버 끝)')
            return
        if self.t_starting is not None:
            self._log(30, '켜는 중 — 연결될 때까지 기다리세요 (1~2 분)')
            return
        if time.monotonic() - self._t_retry < RETRY_HOLD_S:
            return self._log(30, '다시 붙는 중 — 잠깐 뒤에')
        robot = TARGET_ROBOT[self.cb_target.currentIndex()]
        speed = self.sl_speed.value() / 100
        if robot == 'real' and self.cb_mapcam.currentIndex() != 1:
            # [2026-10-27 ④] 실물은 실제 카메라 지도만 — 가상이면 진짜 책상·벽·물건을 못 보고 가상 기둥만 피해 계획함 (서버도 막음)
            QtWidgets.QMessageBox.warning(self, '실제 로봇 켜기', "실제 로봇은 지도 카메라를 '실제 카메라 (고정 D455)' 로 두고 켭니다.\n"
                                                             "'카메라 세팅' 탭에서 바꾼 뒤 다시 [▶ 시스템 시작].")
            return self._log(40, "실제 로봇: 지도 카메라가 '가상' — '카메라 세팅' 탭에서 '실제 카메라 (고정 D455)' 로 바꾼 뒤 켜기")
        if robot == 'real':
            ans = QtWidgets.QMessageBox.warning(
                self, '실제 로봇 켜기',
                '실제 CR3A 에 EtherCAT 통신을 켜고 서보를 ON 합니다.\n\n'
                '· 비상정지 버튼을 손에 두세요.\n'
                '· 로봇 팔이 닿는 곳에 사람·물건이 없는지 확인하세요 (투명한 물건은 카메라 지도에 안 보임).\n'
                f'· 실시간 회피는 꺼진 채로 시작합니다 (명령할 때만 움직임). 속도 배율 {speed:.2f}.\n'
                '· WMX 무료 라이선스: 켠 뒤 6 시간이 지나면 엔진이 통신을 끊어 서보가 꺼집니다\n'
                '  (위쪽 WMX 칸에 남은 시간, 2 분 전부터 새 이동 안 받음 — 시스템을 껐다 켜면 다시 6 시간).\n\n켜시겠습니까?',
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.Cancel, QtWidgets.QMessageBox.Cancel)
            if ans != QtWidgets.QMessageBox.Yes:
                return self._log(20, '실제 로봇 켜기 취소')
        method, rows = self.shaping.applied or self.shaping.config()
        cmd = [sys.executable, str(HERE / 'rt_server.py'), '--robot', robot, '--port', str(PORT),
               '--shaper', json.dumps([method, [[bool(u), float(f)] for u, f in rows]]), '--speed', f'{speed:.2f}']
        if self.cb_mapcam.currentIndex() == 1:                  # 실제 카메라로 지도
            if self._cam_busy:                                  # [R-1] 끄는 중인 노드로 켜지 않게
                return self._log(30, '고정 카메라를 끄는 중 — 끝난 뒤에 [▶ 시스템 시작]')
            if cp.load_pose() is None:
                return self._log(40, '실제 카메라: 등록된 카메라 위치가 없음 — [카메라 위치 맞추기…] 먼저')
            if robot in ('ideal', 'wmx'):
                if not cp.ENC_FILE.exists():
                    return self._log(40, '실제 카메라 + 시뮬레이션 로봇: 실제 팔 자세가 필요 — [카메라 위치 맞추기…] 창에서 [엔코더 읽기] 먼저')
                enc = json.loads(cp.ENC_FILE.read_text())
                self._log(30, f'실제 카메라 + 시뮬레이션 로봇: 실제 팔은 안 움직인다고 보고 {enc["time"]} 엔코더 자세로 지움 '
                              f'({np.round(enc["q_deg"], 2).tolist()}°)')
                cmd += ['--mask-q-file', str(cp.ENC_FILE)]
            if cp.CameraProcess.pid() is None:
                self.on_cam_on()
            cmd += ['--camera', 'real']
        env = os.environ.copy()
        if CUROBO_COPY.exists():
            env['PYTHONPATH'] = f'{CUROBO_COPY}{os.pathsep}' + env.get('PYTHONPATH', '')
        try:
            with open(SERVER_LOG, 'w', encoding='utf-8') as logf:      # (서버만 씀 — 띄운 뒤 GUI 쪽 파일은 닫음)
                self.server_proc = subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT, env=env, cwd=str(HERE),
                                                    start_new_session=True)
        except OSError as e:
            self._log(40, f'서버를 못 켬: {e!r}')
            return
        self.t_starting = time.time()
        what = {'ideal': '이상적 시뮬레이션', 'wmx': 'R2 → WMX 시뮬레이터 (Module.ini·엔진이 시뮬레이터뿐인지 확인 → R2 → 서버)',
                'real': '★ 실제 로봇 ★ R2 → WMX (EtherCAT) → CR3A, 서보 ON · 실시간 회피 끔으로 시작'}[robot]
        self._log(30 if robot == 'real' else 20, f'시스템 시작: {what}, 속도 배율 {speed:.2f}, 준비 1~2 분 — 서버 로그 {SERVER_LOG}')

    def on_stop(self):
        self._jog_release()
        self._cmd({'cmd': 'stop'})

    # ---------- 카메라 ----------
    def _ensure_feed(self):
        if self.cam_feed is None:
            self.cam_feed = cp.CameraFeed(depth=False)          # [R-1] 깊이는 보정 창이 보일 때만 (CalibWindow 가 켬)
            if self.cam_feed.err:
                self._log(30, f'카메라 영상을 못 받음: {self.cam_feed.err}')
        return self.cam_feed

    def _cam_running(self):
        """고정 카메라 노드가 도는지 — PID 파일 + /proc 은 CAM_CHECK_S 마다만 새로 봄 (켜기·끄기 뒤에는 바로)."""
        now = time.monotonic()
        if now - self._t_cam >= CAM_CHECK_S:
            self._t_cam, self._cam_pid = now, cp.CameraProcess.pid()
        return self._cam_pid is not None

    def on_cam_on(self):
        if self._cam_busy:
            return self._log(30, '고정 카메라: 끄는 중 — 끝난 뒤에 다시')
        w, h = (int(x) for x in self.cb_res.currentText().replace(' ', '').split('×'))
        fps = int(self.cb_fps.currentText().split()[0])
        try:
            ok, msg = cp.CameraProcess.start(w, h, fps)
        except Exception as e:                                  # ROS 없음 (PC) 등
            ok, msg = False, repr(e)
        self._t_cam = -1e9
        self._log(20 if ok else 40, f'고정 카메라 ({cp.CAM_SERIAL}): {msg}')
        if ok:
            self._ensure_feed()

    def on_cam_off(self):
        """[R-1 ②] 끄기는 QThreadPool 에서 (SIGINT 뒤 노드가 끝나기를 최대 5 s 기다림 — 전엔 GUI 스레드가 멈춰 서버가 GUI 끊김으로 봤음),
        결과는 신호로 로그에."""
        if self._cam_busy:
            return
        self._cam_busy = True
        self.btn_cam_on.setEnabled(False)
        self.btn_cam_off.setEnabled(False)
        self._log(20, '고정 카메라: 끄는 중 …')
        cp.run_job(cp.CameraProcess.stop, on_done=lambda msg: self._cam_stopped(20, msg), on_failed=self._cam_stop_failed)

    def _cam_stop_failed(self, e):
        LOG.error('카메라 끄기 오류: %r', e, exc_info=(type(e), e, e.__traceback__), extra={'view': False})
        self._cam_stopped(40, f'끄기 오류 {e!r}')

    def _cam_stopped(self, level, msg):
        self._cam_busy = False
        self._t_cam = -1e9
        self.btn_cam_on.setEnabled(True)
        self.btn_cam_off.setEnabled(True)
        self._log(level, f'고정 카메라: {msg}')

    def _sys_state(self):
        """보정 창이 보는 시스템 상태: (켜짐?, 로봇 종류 'ideal'·'wmx'·'real'·None, 관절각 ° 또는 None)."""
        if not (self.connected and self.state):
            return False, None, None
        return True, self.state.get('robot'), np.degrees(self.state['q'])

    def on_calib(self):
        if self.calib_win is None:
            self.calib_win = cp.CalibWindow(self._sys_state, self._ensure_feed(), self.robot)
            self.calib_win.pose_registered.connect(self._on_pose_registered)
        self.calib_win.show()
        self.calib_win.raise_()
        self.calib_win.activateWindow()

    def _on_pose_registered(self, data):
        self.cam_pose = data
        tz = data.get('table_z_m')
        self._log(20, f'카메라 위치 등록: camera_link {data["camera_link_xyzrpy"]} · 테이블 높이 '
                      + ('안 잼' if tz is None else f'{tz * 1000:+.1f} mm')
                      + f' (자세 {data["poses"]} 개, {data["why"]}) — 서버 (지도 바닥·충돌 바닥) 는 다음 [▶ 시스템 시작] 때 읽음')
        if not self.connected:
            self.scene.cam_T = np.array(data['T_base_depth'], float)
            self.scene.floor_z = float(tz or 0.0)      # [D-2] 테이블 윗면. [D-2 검토] 연결 중이면 서버가 쓰는 값 (hello) 그대로 — 다음 시작 때 바뀜
            self.request_render()
        self._show_cam_pose(None)

    def _update_preview(self):
        """카메라 세팅 탭이 보일 때 0.5 s 마다 실제 카메라 미리보기."""
        if self.tabs.currentIndex() != 2 or time.time() - self._t_prev < 0.5:
            return
        self._t_prev = time.time()
        on = self._cam_running()
        img, age = self.cam_feed.live() if (on and self.cam_feed is not None) else (None, None)
        if img is None or age > 2.0:
            self.lb_preview.setText('실제 카메라 켜짐\n(영상 기다리는 중)' if on else '실제 카메라 꺼짐\n[▶ 카메라 켜기]')
            return
        small = cv2.resize(img, (212, 120), interpolation=cv2.INTER_AREA)
        self.lb_preview.setPixmap(QtGui.QPixmap.fromImage(to_qimage(small)))

    def _wmx_only(self, what) -> bool:
        if not (self.connected and self.state):
            self._log(30, '시스템이 꺼져 있음 — [▶ 시스템 시작] 을 먼저')
            return False
        if self.state.get('robot') not in ('wmx', 'real'):
            self._log(30, f'이상적 시뮬레이션에는 {what} 없음 (WMX 시뮬레이터·실물에서)')
            return False
        return True

    def on_servo(self):
        if self._wmx_only('서보가'):
            self._cmd({'cmd': 'servo', 'on': not self.state.get('servo', True)})

    def on_alarm(self):
        if self._wmx_only('알람이'):
            self._cmd({'cmd': 'alarm'})

    def on_estop(self):
        self._jog_release()
        if not self._cmd({'cmd': 'estop'}, quiet=True):
            self._log(30, '비상정지: 시스템이 꺼져 있음 (보낼 곳 없음)')

    def on_realtime(self, on):
        if not self._cmd({'cmd': 'realtime', 'on': bool(on)}):
            self._set_rt_check(TARGET_ROBOT.get(self.cb_target.currentIndex()) != 'real')   # 꺼져 있으면 시작 값 (실물만 끔)

    def _set_rt_check(self, on):
        """서버 값으로 체크만 맞춤 (on_realtime 이 다시 보내지 않게 신호를 막음)."""
        with QtCore.QSignalBlocker(self.chk_rt):
            self.chk_rt.setChecked(bool(on))

    def on_shaper_apply(self, cfg):
        method, rows = cfg
        if not self.connected:
            self.shaping.mark_applied(cfg)
            self._save_shaper(cfg)
            self._log(20, f'입력성형 {shaper_label(method, rows)} — 다음 시스템 시작 때 이 값으로')
            return
        if not self._cmd({'cmd': 'shaper', 'method': method, 'rows': [[bool(u), float(f)] for u, f in rows]}):
            self.shaping.reject_pending()

    def _save_shaper(self, cfg):
        self.settings.setValue('shaper', json.dumps([cfg[0], [[bool(u), float(f)] for u, f in cfg[1]]]))

    def _load_shaper(self):
        try:
            m, r = json.loads(self.settings.value('shaper', ''))
            return m, [(bool(u), float(f)) for u, f in r]
        except (TypeError, ValueError):
            return 'ETM4', [(True, 8.0), (False, 15.0), (False, 23.0)]

    def on_goto(self):
        if self._ready():
            self._cmd({'cmd': 'goto', 'q_deg': [sp.value() for sp in self.sp_target]})

    def on_repeat(self):
        if self._ready():
            self._cmd({'cmd': 'repeat', 'q_deg': [sp.value() for sp in self.sp_target]})

    def _pose_cmd(self, cmd):
        """공구 끝으로 이동·반복: X Y Z (mm) + Rx Ry Rz (°, '방향은 지금 그대로' 면 안 보냄 = 지금 방향) → 서버가 cuRobo IK 로 관절 목표."""
        if self._ready():
            self._cmd({'cmd': cmd, 'xyz_mm': [sp.value() for sp in self.sp_xyz],
                       'rpy_deg': None if self.chk_keep_rot.isChecked() else [sp.value() for sp in self.sp_rpy]})

    def _on_jog_mode(self, hold):
        self.lb_jstep.setText('<b>관절</b> 속도' if hold else '<b>관절</b> 간격')
        self.lb_junit.setText('°/s' if hold else '°')
        self.lb_cstep.setText('<b>공구 끝</b> 속도' if hold else '<b>공구 끝</b> 간격')
        self.lb_cunit.setText('mm/s  ·  회전' if hold else 'mm  ·  회전')
        self.lb_runit.setText('°/s' if hold else '°')

    def _cart_amount(self, axis):
        """공구 끝 한 칸 (또는 누르는 동안 속도): X Y Z = mm (/s), Rx Ry Rz = ° (/s)."""
        return float((self.cb_cstep if axis in ('X', 'Y', 'Z') else self.cb_rstep).currentText())

    def _cart_frame(self):
        return 'tool' if self.cb_frame.currentText() == '공구' else 'base'

    def _jog_click(self, kind, axis, sign):
        if self.rb_hold.isChecked():
            return                                   # 누르는 동안 방식은 pressed/released 로
        if not self._ready():
            return
        if kind == 'cart':
            self._cmd({'cmd': 'jog_cart', 'axis': axis, 'delta': sign * self._cart_amount(axis), 'frame': self._cart_frame()})
        else:
            self._cmd({'cmd': 'jog', 'joint': axis, 'delta_deg': sign * float(self.cb_jstep.currentText())})

    def _jog_press(self, kind, axis, sign, btn=None):
        if not self.rb_hold.isChecked():
            return
        if not self._ready():
            return
        if kind == 'cart':
            hj = ('cart', axis, sign * self._cart_amount(axis), self._cart_frame())
        else:
            hj = ('joint', axis, sign * float(self.cb_jstep.currentText()), None)
        with self._hold_lock:
            self._beat = time.monotonic()
            self.hold_jog, self._hold_btn = hj, btn
            self._hold_wake.set()

    def _on_beat(self):
        """GUI 스레드 맥박 (BEAT_MS 타이머) — 조그 스레드가 이것으로 화면이 멈췄는지 봄. [R-1 독립 검토] 누른 단추가 이미 올라와 있으면
        뗌: 단추가 눌린 채 그 밑에서 화면이 움직여 (스크롤 막대가 생기는 등) 마우스를 단추 밖에서 떼면 Qt 가 released 를 안 냄."""
        self._beat = time.monotonic()
        b = self._hold_btn
        if self.hold_jog is not None and b is not None and not b.isDown():
            self._jog_release()

    def _hold_loop(self):
        """누르는 동안 조그 명령을 50 ms 마다 (화면 그리기와 따로 — [2026-10-26 Thor 시험] 3D 를 그리는 동안 GUI 타이머가 밀려
        서버가 '뗌' 으로 보고 조그가 한 번 끊겼음). 서버는 0.4 s 동안 안 오면 뗀 것으로 보고 섬.
        [2026-10-27 R-1 ①] GUI 스레드가 멈추면 (맥박 _beat 가 HOLD_STALL_S 넘게 안 바뀜) 뗌을 한 번 보내고 끝냄 — 다시 눌러야 움직임.
        전엔 GUI 가 멈춰도 이 스레드가 계속 보내 로봇이 계속 움직였고, 서버의 GUI 끊김 (2 s) 도 이 명령이 새로 고쳐 안 걸렸음.
        스레드는 하나 (창과 함께 시작, 누를 때 _hold_wake 로 깨움). 보고 보내기·지우고 뗌 보내기는 _hold_lock 하나로 (순서가 섞이지 않게).
        관절 = jog_hold (°/s), 공구 끝 = jog_cart_hold (mm/s · °/s, 받침·공구 기준)."""
        while True:
            self._hold_wake.wait()
            try:
                with self._hold_lock:
                    hj = self.hold_jog
                    if hj is None:
                        self._hold_wake.clear()
                        continue
                    stale = time.monotonic() - self._beat
                    if stale >= HOLD_STALL_S or not self.connected:
                        self.hold_jog = None
                        self._hold_wake.clear()
                        if self.connected:
                            self.link.send({'cmd': 'jog_release'})
                            self.hold_stalled.emit(stale)
                        continue
                    kind, axis, v, frame = hj
                    if kind == 'cart':
                        self.link.send({'cmd': 'jog_cart_hold', 'axis': axis, 'v': v, 'frame': frame})
                    else:
                        self.link.send({'cmd': 'jog_hold', 'joint': axis, 'v_deg': v})
            except Exception:                       # (생각 못 한 오류로 하나뿐인 이 스레드가 죽지 않게 — 조그는 뗌)
                LOG.exception('누르는 동안 조그 스레드 오류 — 조그를 뗌')
                with self._hold_lock:
                    self.hold_jog = None
                    self._hold_wake.clear()
                    try:
                        if self.connected:
                            self.link.send({'cmd': 'jog_release'})
                    except Exception:
                        pass
                continue
            # [R-1 통합 시험 준비] 다음 보내기 (50 ms) 전이라도 맥박이 HOLD_STALL_S 를 넘는 순간에 다시 봄 — 뗌이 멈춘 지 0.3 s + 한 주기
            #  (0.35 s) 까지 늦지 않게 (Thor 시험 0.309 s). 맥박이 살아 있으면 남은 시간이 0.25 s 넘어 늘 50 ms
            time.sleep(min(HOLD_JOG_MS / 1000.0, max(0.0, self._beat + HOLD_STALL_S - time.monotonic()) + 0.001))

    def _on_hold_stalled(self, stale):
        self._log(30, f'누르는 동안 조그: 화면 (GUI 스레드) 이 {HOLD_STALL_S:.1f} s 넘게 멈춰 조그를 뗌 (멈춘 지 {stale:.2f} s 에 앎) '
                      '— 계속하려면 단추를 뗐다가 다시 누르세요')

    def _jog_release(self):
        with self._hold_lock:
            if self.hold_jog is None:
                return
            self.hold_jog, self._hold_btn = None, None
            self._hold_wake.clear()
            self._cmd({'cmd': 'jog_release'}, quiet=True)

    def _q_now_deg(self):
        if self.state is not None:
            return np.degrees(self.state['q'])
        return np.array(self.scene.q if self.scene.q is not None else np.zeros(6), float)

    def _fill_current(self):
        for sp, v in zip(self.sp_target, self._q_now_deg()):
            sp.setValue(v)

    # ---------- 자세 목록 ----------
    @staticmethod
    def _load_poses():
        """저장한 자세 [(이름, (J1~J6 °))] 와 오류 글 (없으면 None). 파일이 없으면 빈 목록. 못 읽으면 그 파일을 .bad-<시각> 으로 옮겨 두고 빈
        목록 (다음 저장이 덮어쓰지 않게)."""
        try:
            data = json.loads(POSES_FILE.read_text(encoding='utf-8'))
            return [(str(p['name']), tuple(float(x) for x in p['q_deg'])) for p in data['poses'] if len(p['q_deg']) == 6], None
        except FileNotFoundError:
            return [], None
        except (OSError, ValueError, KeyError, TypeError) as e:
            bad = POSES_FILE.with_name(POSES_FILE.name + time.strftime('.bad-%Y%m%d_%H%M%S'))
            try:
                POSES_FILE.rename(bad)
            except OSError:
                pass
            return [], f'자세 목록 파일을 못 읽음 ({e!r}) — {bad.name} 로 옮기고 빈 목록으로 시작'

    def _write_poses(self, what):
        try:
            cp.write_json_atomic(POSES_FILE, {'poses': [{'name': n, 'q_deg': list(q)} for n, q in self.poses]}, ensure_ascii=False, indent=1)
            self._log(20, what)
        except OSError as e:
            self._log(40, f'{what} — 파일에 못 씀 ({e!r}), 이 창을 닫으면 사라짐')

    def _pose_items(self):
        return list(cm.POSES.items()) + self.poses

    @staticmethod
    def _pose_text(q):
        return '  '.join(f'J{k + 1} {v:.2f}°' for k, v in enumerate(q))

    def _refresh_poses(self, select=None):
        cb = self.cb_pose
        cur = cb.currentText() if select is None else select
        with QtCore.QSignalBlocker(cb):
            cb.clear()
            for name, q in self._pose_items():
                cb.addItem(name)
                cb.setItemData(cb.count() - 1, self._pose_text(q), QtCore.Qt.ToolTipRole)
            cb.setCurrentIndex(max(cb.findText(cur), 0))
        self._on_pose_pick()

    def _on_pose_pick(self):
        self.btn_pose_del.setEnabled(self.cb_pose.currentText() not in cm.POSES)

    def _save_pose(self):
        names = [n for n, _ in self.poses]
        k = 1
        while f'자세 {k}' in names:
            k += 1
        q = tuple(round(sp.value(), 2) for sp in self.sp_target)
        name, ok = QtWidgets.QInputDialog.getText(self, '자세 저장', f'지금 J1~J6 칸의 각도를 이 이름으로 저장합니다.\n{self._pose_text(q)}',
                                                  text=f'자세 {k}')
        name = name.strip()
        if not ok or not name:
            return
        if name in cm.POSES:
            QtWidgets.QMessageBox.information(self, '자세 저장', f"'{name}' 은 늘 있는 자세라 못 바꿉니다 — 다른 이름으로 저장하세요.")
            return
        if name in names:
            ans = QtWidgets.QMessageBox.question(self, '자세 저장', f"'{name}' 자세가 이미 있습니다. 지금 칸 값으로 바꿀까요?\n\n"
                                                                f'전: {self._pose_text(self.poses[names.index(name)][1])}\n뒤: {self._pose_text(q)}')
            if ans != QtWidgets.QMessageBox.Yes:
                return
            self.poses[names.index(name)] = (name, q)
            self._write_poses(f"자세 바꿈: '{name}'  {self._pose_text(q)}")
        else:
            self.poses.append((name, q))
            self._write_poses(f"자세 저장: '{name}'  {self._pose_text(q)}")
        self._refresh_poses(select=name)

    def _delete_pose(self):
        name = self.cb_pose.currentText()
        names = [n for n, _ in self.poses]
        if name not in names:
            return
        ans = QtWidgets.QMessageBox.question(self, '자세 삭제', f"'{name}' 자세를 목록에서 지울까요?\n{self._pose_text(self.poses[names.index(name)][1])}")
        if ans != QtWidgets.QMessageBox.Yes:
            return
        del self.poses[names.index(name)]
        self._write_poses(f"자세 지움: '{name}'")
        self._refresh_poses()

    def _fill_pose(self):
        q = dict(self._pose_items()).get(self.cb_pose.currentText())
        if q is None:
            return
        for sp, val in zip(self.sp_target, q):
            sp.setValue(val)

    def _fill_xyz(self):
        T = self.robot.tip(self._q_now_deg())
        for k in range(3):
            self.sp_xyz[k].setValue(T[k, 3] * 1000)
        for sp, a in zip(self.sp_rpy, cm.rpy_from_matrix(T[:3, :3])):
            sp.setValue(a)

    # ---------- 서버 상태 ----------
    def _offline(self):
        self.shaping.set_config(*self._load_shaper())
        cam_T, Tbc = default_cam_T()
        self.scene.cam_T = np.array(self.cam_pose['T_base_depth'], float) if self.cam_pose else cam_T
        self.scene.floor_z = float((self.cam_pose or {}).get('table_z_m') or 0.0)     # [D-2] 테이블 윗면 (카메라 보정이 잰 값)
        self.scene.q = np.zeros(6)                      # (연결 전 표시 — 연결되면 로봇 자세)
        self._show_cam_pose(Tbc)
        self.lb_preview.setText('실제 카메라 꺼짐\n[▶ 카메라 켜기]')
        self._fill_pose()
        self._update_chips()
        self._update_pose(self.scene.q)
        self.lb_reason.setText('시스템이 꺼져 있음 — 대상을 고르고 [▶ 시스템 시작]')
        self.lb_plan.setText('')
        self.lb_exec.setText('')
        self.lb_map.setText('지도: —')
        self.request_render()

    def request_render(self):
        """[R-1] 3D 다시 그리기 요청 — 같은 틱 안의 여러 요청을 0 ms 뒤 scene.refresh() 한 번으로 모음."""
        if not self._render_pending:
            self._render_pending = True
            QtCore.QTimer.singleShot(0, self._render_now)

    def _render_now(self):
        self._render_pending = False
        self.scene.refresh()

    def _show_cam_pose(self, Tbc):
        """카메라 위치 (로봇 받침 기준 camera_link): 등록한 실제 카메라 + (주면) 가상 카메라 (시뮬레이션)."""
        lines = []
        cpz = self.cam_pose
        if cpz:
            x, y, z, rx, ry, rz = cpz['camera_link_xyzrpy']
            tz = cpz.get('table_z_m')
            why = cpz['why'].split(' · 테이블 높이')[0]          # [D-2 검토] 판정 글 끝의 테이블 높이는 아래에 한 번만
            tzs = (f'{tz * 1000:+.1f} mm' if tz is not None else
                   '안 잼 (예전 보정 — 0 으로 씀)' if 'table_z_m' not in cpz else '안 잼 (테이블 점 적음 — 0 으로 씀)')
            lines.append(f'<b>실제 카메라 (등록 {cpz["time"][5:16]}, 자세 {cpz["poses"]} 개, {why})</b><br>'
                         f'x {x:.0f}  y {y:.0f}  z {z:.0f} mm · Rx {rx:.1f}  Ry {ry:.1f}  Rz {rz:.1f}° · 테이블 높이 {tzs}')
        else:
            lines.append('<b>실제 카메라</b>: 등록된 위치 없음 — [카메라 위치 맞추기…]')
        if Tbc is not None:
            rpy = cm.rpy_from_matrix(Tbc[:3, :3])
            p = Tbc[:3, 3] * 1000
            lines.append(f'<span style="color:#6e7781">가상 카메라 (시뮬레이션): x {p[0]:.0f}  y {p[1]:.0f}  z {p[2]:.0f} mm · '
                         f'Rx {rpy[0]:.1f}  Ry {rpy[1]:.1f}  Rz {rpy[2]:.1f}°</span>')
        self._vcam_line = lines[1] if len(lines) > 1 else self._vcam_line
        if Tbc is None and self._vcam_line:
            lines.append(self._vcam_line)
        self.lb_campose.setText('<br>'.join(lines))

    def _tick(self):
        for m in self.link.drain():
            self._on_msg(m)
        if self.connected and time.time() - self._t_ping >= PING_S:
            self._t_ping = time.time()
            self.link.send({'cmd': 'ping'})
        if self.server_proc is not None and self.server_proc.poll() is not None and not self.connected:
            code = self.server_proc.returncode
            self.server_proc = None
            if self.t_starting is not None:
                self._log(40, f'서버가 켜지다 끝남 (코드 {code}) — 서버 로그 {SERVER_LOG}')
            self.t_starting = None
        if self.t_starting is not None and time.time() - self.t_starting > 300:
            self._log(40, f'서버에 5 분 동안 연결 못 함 — 서버 로그 {SERVER_LOG}')
            self.t_starting = None
        if self.state is not None:
            self._update_from_state()
        self._update_chips()
        self._update_preview()

    def _on_msg(self, m):
        t = m.get('type')
        if t == 'connected':
            self.connected, self.t_starting, self.stopping, self.refused_why = True, None, False, None
            self._log(20, '서버에 연결됨')
        elif t == 'refused':                         # [R-1] 서버가 거절 (다른 GUI 가 살아 있음) — 한 번 알리고 [↻ 다시 연결] 을 기다림
            self.refused_why = str(m.get('why') or '서버가 이미 다른 GUI 와 연결돼 있음')
            self._log(40, f'서버가 이 창의 연결을 거절함: {self.refused_why}. 저절로 다시 붙지 않음 — 다른 GUI 를 닫은 뒤 [↻ 다시 연결]')
            self.lb_reason.setText(f'<b style="color:{COLORS["err"]}">연결 거절됨</b> — 다른 GUI 가 서버에 붙어 있음. 그 GUI 를 닫은 뒤 '
                                   '[↻ 다시 연결]')
        elif t == 'disconnected':
            self.connected, self.state = False, None
            self._log(30 if not self.stopping else 20, '서버 연결 끊김' + (' (시스템 꺼짐)' if self.stopping else ''))
            self.stopping = False
            self.scene.path = self.scene.obst = self.scene.near = self.scene.ghost = self.scene.u_q = self.scene.target_T = None
            self.lb_reason.setText('시스템이 꺼져 있음 — [▶ 시스템 시작]')
            self.request_render()
        elif t == 'hello':
            self.hello.update({k: v for k, v in m.items() if k != 'logs'})
            for line in m.get('logs', [])[-30:]:
                self._log(10, f'[서버] {line}')
            if 'cam_T' in m:
                self.scene.cam_T = np.array(m['cam_T'], float)
            if 'table_z' in m:
                self.scene.floor_z = float(m['table_z'] or 0.0)        # [D-2] 서버가 쓰는 테이블 윗면 높이
            if m.get('shaper'):
                method, rows = m['shaper']
                cfg = (method, [(bool(u), float(f)) for u, f in rows])
                self.shaping.set_config(*cfg)
                self._save_shaper(cfg)
            if 'realtime' in m:
                self._set_rt_check(m['realtime'])
            if 'speed' in m:
                self._set_speed_slider(m['speed'])
        elif t == 'speed':
            self._set_speed_slider(m['scale'])
        elif t == 'log':
            line = m['line']
            self.last_event = line
            self._log(40 if ('오류' in line or '실패' in line) else
                      30 if ('정지' in line or '막힘' in line or '걸림' in line or '주의' in line) else 20, line)
        elif t == 'state':
            self.state = m
            self._scope_add(m)
        elif t == 'path':
            self.scene.path = np.degrees(np.array(m['q'], float))
        elif t == 'voxels':
            P = np.asarray(m['P'], np.float32)
            self.scene.obst = (P, float(m['vs']))
            self.n_vox, self._t_vox = len(P), time.time()
            st = self.state or self.hello
            # [R-1 ④] 지도 카메라 = 서버가 쓰는 것 (전엔 실제 카메라여도 '가상 카메라 15 Hz')
            src = (f'실제 카메라 {st.get("cam_fps") or 0.0:.0f} fps' if st.get('camera') == 'real' else f'가상 카메라 {VCAM_FPS:g} Hz')
            if m.get('off'):                         # [D-7] 서버가 카메라 지도를 안 씀 (고정 카메라 꺼짐)
                self.lb_map.setText('지도: <b>안 씀</b> · 고정 카메라 꺼짐 → 이동이 장애물을 안 봄 (책상·자기 충돌만)')
            else:
                self.lb_map.setText(f'지도: 물체 칸 <b>{self.n_vox:,}</b> 개 · {float(m["vs"]) * 100:.0f} cm 칸 · {src}')
            self._scene_key = None                   # 다음 틱에 다시 그림
        elif t == 'shaper':
            method, rows = m['cfg']
            cfg = (method, [(bool(u), float(f)) for u, f in rows])
            self.shaping.mark_applied(cfg)
            self._save_shaper(cfg)
        elif t == 'shaper_rejected':
            self.shaping.reject_pending()
        elif t == 'realtime':
            self._set_rt_check(m['on'])
        elif t == 'near':                            # [2026-10-27 D-1 ㄴ] 지도 점검 결과 (통과면 지움) — 글은 서버 기록 줄로 옴
            P = np.asarray(m['P'], np.float32)
            self.scene.near = P if len(P) else None
            self.near_q = None if m.get('q') is None else np.asarray(m['q'], float)     # 점검한 자세 (로봇이 움직이면 지움)
            self.request_render()
        elif t == 'servo' and self.state is not None:
            self.state['servo'] = bool(m['on'])

    def _scope_add(self, s):
        if not self.scope.running:                   # [R-1 ④] [▶ 그래프 켜기] 일 때만 쌓음 (켤 때마다 비움 — _on_scope_run)
            return
        self.scope_buf.append((s['t'], s['u'], s['y'], s['q']))
        while self.scope_buf and s['t'] - self.scope_buf[0][0] > SCOPE_S:
            self.scope_buf.popleft()

    def _update_pose(self, q_deg):
        for k in range(6):
            self.joint_val[k].setText(f'{q_deg[k]:8.2f}°')
            self.joint_bar[k].set_value(q_deg[k])
        tip = self.robot.tip(q_deg)
        for k in range(3):
            self.cart_val[k].setText(f'{tip[k, 3] * 1000:8.1f} mm')
        for k, a in enumerate(cm.rpy_from_matrix(tip[:3, :3])):
            self.rot_val[k].setText(f'{a:8.1f}°')

    def _update_from_state(self):
        s = self.state
        q = np.degrees(s['q'])
        self._update_pose(q)
        goal = None if s.get('goal') is None else np.degrees(s['goal'])
        sc = self.scene
        nq = self.near_q
        if sc.near is not None and nq is not None and float(np.max(np.abs(np.asarray(s['q'], float) - nq))) > math.radians(0.5):
            sc.near, self.near_q = None, None        # 점검 뒤 로봇이 움직임 → 그 점은 지금 자세의 결과가 아님 (독립 검토 LOW)
        sc.q, sc.u_q = q, np.degrees(s['u'])
        sc.ghost = goal
        sc.target_T = None if goal is None else self.robot.tip(goal)
        now = time.time()
        key = (tuple(np.round(q, 1)), None if goal is None else tuple(np.round(goal, 1)), id(sc.path), id(sc.obst),
               tuple(np.round(sc.u_q, 1)) if sc.flags['u'] else None)
        if key != self._scene_key and now - self._t_scene >= SCENE_S and self.view_tabs.currentIndex() == 0:
            self._scene_key, self._t_scene = key, now
            self.request_render()
        # 계획 · 실행
        if s.get('estop'):
            self.lb_reason.setText(f'<b style="color:{COLORS["err"]}">비상정지</b> — 새 명령을 주거나 실시간 회피를 켜면 풀림')
        elif not s.get('ready'):
            self.lb_reason.setText('<b>준비 중</b> — 지도 배경을 모으는 중')
        else:
            job = s.get('job')
            what = (f'<b>{s["label"]}</b>' if job else (f'<b>{s["label"]}</b> (MoveIt Servo)' if s.get('jog') else
                    ('<b>지키기</b> (다가오는 물체를 피함)' if s.get('realtime') else '<b>서 있음</b> (실시간 회피 끔)')))
            self.lb_reason.setText(f'{what} · MPC {s.get("mode")}')
        parts = []
        if s.get('clr_mm') is not None:                     # 거리 지도는 5 cm 에서 잘림 (그 밖은 같은 값)
            parts.append('5 cm 안에 물체 없음' if s['clr_mm'] >= 49.5 else f'가장 가까운 물체 <b>{s["clr_mm"]:.0f} mm</b>')
        if s.get('mpc_ms') is not None:
            parts.append(f'MPC {s["mpc_ms"]:.0f} ms · 한 주기 {s["cyc_ms"]:.0f} ms')
        if s.get('guard'):
            parts.append(f'<span style="color:{COLORS["warn"]}">안전 검사: {s["guard"]}</span>')
        if s.get('skipped'):
            parts.append(f'<span style="color:{COLORS["warn"]}">앞 메시지 계속</span>')
        cfg = self.shaping.applied or self.shaping.config()
        if cfg != self._delay_cfg:                          # [R-1] 성형기가 바뀔 때만 계산 (전엔 0.1 s 틱마다 build)
            self._delay_cfg, self._delay_ms = cfg, ms(build(*cfg)['t'][-1])
        parts.append(f'입력성형 지연 {self._delay_ms} ms')
        self.lb_plan.setText(' · '.join(parts))
        self.lb_exec.setText(f'마지막 사건: {self.last_event}' if self.last_event else '')
        if s.get('realtime') != self.chk_rt.isChecked():
            self._set_rt_check(s.get('realtime'))
        if (self.scope.running and self.view_tabs.currentIndex() == 1 and now - self._t_scope >= 0.5
                and len(self.scope_buf) > 2):
            self._t_scope = now
            b = list(self.scope_buf)
            t = np.array([x[0] for x in b])
            self.scope.set_data({'t': t - t[-1] + SCOPE_S, 'u': np.degrees([x[1] for x in b]),
                                 'y': np.degrees([x[2] for x in b]), 'fb': np.degrees([x[3] for x in b])})

    def _update_chips(self):
        c = COLORS
        st = self.state
        if self.connected and st is not None:
            kind = st.get('robot')
            wmx = kind in ('wmx', 'real')                       # R2 → WMX (시뮬레이터 또는 EtherCAT 실물)
            if st.get('estop'):
                self._set_pill(self.chips['robot'], '비상정지', c['err'])
            elif kind == 'real':
                self._set_pill(self.chips['robot'], '실제 로봇', c['real'], 'R2 → WMX (EtherCAT) → CR3A (위치 = 엔코더 /joint_states). 실제로 움직임')
            elif wmx:
                self._set_pill(self.chips['robot'], 'WMX 시뮬레이터', c['info'],
                               'R2 → WMX 시뮬레이터 (위치 = R2 /joint_states). 실제 로봇은 안 움직임')
            else:
                self._set_pill(self.chips['robot'], '이상적 로봇', c['info'], '소프트웨어 로봇 (보낸 명령 그대로) — 실제 로봇은 안 움직임')
            if not wmx:
                self._set_pill(self.chips['wmx'], '안 씀', c['off'], '이상적 시뮬레이션은 WMX 를 쓰지 않음')
            elif st.get('fault'):
                self._set_pill(self.chips['wmx'], 'R2 오류', c['err'], f'{st["fault"]}\n[알람 해제] 또는 시스템을 껐다 켜기')
            else:
                on = st.get('servo', True)
                plat = 'EtherCAT' if kind == 'real' else '시뮬레이터'
                # [2026-10-27 ⑤] WMX 무료 라이선스 남은 시간 (엔진을 켠 뒤 6 시간에 통신 끊김): 30 분 아래 주황, 10 분 아래 빨강
                left = st.get('comm_left_s')
                tail = '' if left is None else f' · 통신 {int(max(left, 0) // 3600)}:{int(max(left, 0) % 3600 // 60):02d}'
                color = c['ok'] if on else c['warn']
                if left is not None and left <= 1800:
                    color = c['err'] if left <= 600 else c['warn']
                self._set_pill(self.chips['wmx'], f'{plat} · 서보 ' + ('ON' if on else 'OFF') + tail, color,
                               f'WMX 엔진 = {plat} (켤 때 확인), 서보 {"ON" if on else "OFF"} (R2). '
                               f'/joint_states {st.get("js_age_ms")} ms 전\n'
                               + ('' if left is None else 'WMX 무료 라이선스: 엔진을 켠 뒤 6 시간이 지나면 통신을 끊어 서보가 꺼짐 — 남은 시간 (시:분). '
                                                          '2 분 전부터 새 이동 안 받음, 시스템을 껐다 켜면 다시 6 시간\n')
                               + f'기록: {self.hello.get("out")}')
            self.btn_servo.setText('서보 끄기' if (not wmx or st.get('servo', True)) else '서보 켜기')
            self._set_pill(self.chips['sys'], '켜짐' if st.get('ready') else '준비 중', c['ok'] if st.get('ready') else c['warn'])
            real = st.get('camera') == 'real'
            fps = st.get('cam_fps') or 0.0
            if real:
                self._set_pill(self.chips['cam'], f'실제 {fps:.0f} fps', c['ok'] if fps > 5 else c['err'],
                               f'실제 고정 카메라 (D455 {cp.CAM_SERIAL}) 깊이 → 지도. 등록한 카메라 위치')
            else:
                self._set_pill(self.chips['cam'], f'가상 {VCAM_FPS:g} fps', c['ok'], '가상 카메라 (depth_sim, 640 × 480)')
            self.cb_mapcam.setEnabled(False)
            self._set_pill(self.chips['obst'], '켜짐' if st.get('realtime') else '지도 멈춤',
                           c['ok'] if st.get('realtime') else c['warn'])
            self.btn_stack.setText('■ 시스템 종료')
            self.cb_target.setEnabled(False)
            # [R-1 ④] 카메라 세팅 탭 칩 = 실제 상태 (전엔 연결되면 넷 다 초록 · '가상 카메라' — 실제 카메라여도)
            if real:
                self._set_pill(self.st_chips['cam'], f'실제 카메라 {fps:.0f} fps', c['ok'] if fps > 5 else c['err'],
                               '서버가 받는 실제 고정 카메라 깊이 영상 속도')
            else:
                self._set_pill(self.st_chips['cam'], f'가상 카메라 {VCAM_FPS:g} fps', c['ok'], '시뮬레이션 깊이 (depth_sim, 640 × 480)')
            if not st.get('ready'):
                self._set_pill(self.st_chips['map'], '지도 준비 중', c['warn'], '지도 배경을 모으는 중')
            elif time.time() - self._t_vox > VOX_STALE_S:
                self._set_pill(self.st_chips['map'], '지도 안 옴', c['err'], f'서버에서 지도 칸이 {VOX_STALE_S:.0f} s 넘게 안 옴')
            elif st.get('realtime'):
                self._set_pill(self.st_chips['map'], '지도 쌓는 중', c['ok'], '영상마다 지도를 새로 쌓음 (실시간 회피 켬)')
            else:
                self._set_pill(self.st_chips['map'], '지도 멈춤', c['warn'], '실시간 회피를 끄면 지도가 그 순간에 멈춤 (계획 직전에만 새로 봄)')
            self._set_caminfo(True)
        else:
            starting = self.t_starting is not None
            if self.refused_why is not None:         # [R-1] 서버가 거절 — [↻ 다시 연결] 전까지 (같은 포트에 서버를 새로 켜지 않음)
                self._set_pill(self.chips['robot'], '연결 거절됨', c['err'], f'{self.refused_why}\n그 GUI 를 닫은 뒤 [↻ 다시 연결]')
            else:
                self._set_pill(self.chips['robot'], '연결 안 됨', c['off'])
            self._set_pill(self.chips['wmx'], '—', c['off'])
            self._set_pill(self.chips['sys'], f'켜는 중 {time.time() - self.t_starting:.0f} s' if starting else '꺼짐',
                           c['warn'] if starting else c['off'])
            self._set_pill(self.chips['cam'], '꺼짐', c['off'])
            self._set_pill(self.chips['obst'], '꺼짐', c['off'])
            self.cb_mapcam.setEnabled(not starting)
            self.btn_stack.setText('↻ 다시 연결' if self.refused_why is not None else '▶ 시스템 시작')
            self.cb_target.setEnabled(not starting)
            for key, name in (('cam', '카메라'), ('map', '지도')):
                self._set_pill(self.st_chips[key], name, c['off'])
            self._set_caminfo(False)

    def _set_caminfo(self, running):
        """카메라 세팅 탭 글: 실제 고정 카메라 (켜짐·영상 fps) + 지도에 쓰는 카메라 (서버가 쓰는 것)·지도 상태."""
        c = COLORS
        on = self._cam_running()
        fps = self.cam_feed.fps('color') if (on and self.cam_feed is not None and self.cam_feed.ok) else 0.0
        st = self.state if running else None
        real = bool(st and st.get('camera') == 'real')
        if st is None:
            tail = f' <span style="color:{c["off"]}">시스템 꺼짐</span>'
        elif not st.get('ready'):
            tail = f' <span style="color:{c["warn"]}">준비 중</span>'
        elif st.get('realtime'):
            tail = f' <span style="color:{c["ok"]}">● 쌓는 중</span>'
        else:
            tail = f' <span style="color:{c["warn"]}">멈춤 (실시간 회피 끔)</span>'
        text = (f'<b>실제 카메라</b> D455 {cp.CAM_SERIAL} (고정, 전체): '
                + (f'<span style="color:{c["ok"]}">켜짐 · 영상 {fps:.0f} fps</span>' if on else f'<span style="color:{c["off"]}">꺼짐</span>')
                + '<br><b>지도</b>: ' + ('실제 카메라 (등록 위치)' if real else f'가상 카메라 (시뮬레이션, 640 × 480 · {VCAM_FPS:g} fps)')
                + tail)
        if self.lb_caminfo.property('html') != text:
            self.lb_caminfo.setProperty('html', text)
            self.lb_caminfo.setText(text)

    def _on_scope_run(self, on):
        """[R-1 ④] [▶ 그래프 켜기] — 켤 때마다 비우고 새로 쌓음."""
        if on:
            self.scope_buf.clear()
            self._t_scope = 0.0


def keep_running_on_errors():
    """처리 안 된 예외로 창이 꺼지지 않게: PyQt5 는 기본 예외 처리기일 때 슬롯·그리기의 예외로 프로그램을 끔 (qFatal).
    [2026-10-26] 주파수 칸을 고치다 생긴 1 Hz 에서 임펄스 그림 눈금 계산 예외 → 창이 그냥 꺼졌음.
    → 예외를 로그 창 (빨강 한 줄)·GUI 로그 파일 (전체 추적)·stderr 에 남기고 계속 돎. [R-1 ⑥] logging 으로 (파일 = common.GUI_LOG, 전엔
    /tmp/cr3a_gui_errors.log), 다른 스레드 (조그·서버 연결·카메라) 의 예외도 (threading.excepthook)."""
    def hook(etype, value, tb):
        if sys.stderr:
            traceback.print_exception(etype, value, tb, file=sys.stderr)
        LOG.error('오류 (창은 계속 돎): %s: %s — 자세한 내용 %s', etype.__name__, value, GUI_LOG, exc_info=(etype, value, tb))

    def thread_hook(a):
        if a.exc_type is SystemExit:
            return
        name = a.thread.name if a.thread is not None else '?'
        if sys.stderr:
            traceback.print_exception(a.exc_type, a.exc_value, a.exc_traceback, file=sys.stderr)
        LOG.error('스레드 %s 오류: %s: %s — 자세한 내용 %s', name, a.exc_type.__name__, a.exc_value, GUI_LOG,
                  exc_info=(a.exc_type, a.exc_value, a.exc_traceback))
    sys.excepthook = hook
    threading.excepthook = thread_hook


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--mock', action='store_true', help='화면 시안 (예시 숫자, 서버 없음 — gui_mock.py)')
    ap.add_argument('--shot', help='(--mock) 탭마다 화면을 PNG 로 저장할 폴더 (저장하고 끝냄)')
    ap.add_argument('--size', default='1600x1000')
    args = ap.parse_args(argv)
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    fams = QtGui.QFontDatabase().families()
    fam = next((f for f in ('NanumGothic', 'Malgun Gothic', 'Noto Sans CJK KR') if f in fams), None)
    if fam:
        app.setFont(QtGui.QFont(fam, 9))
    keep_running_on_errors()
    if args.mock or args.shot:
        sys.modules.setdefault('cr3a_gui', sys.modules[__name__])     # (python cr3a_gui.py --mock: gui_mock 이 이 파일을 또 읽지 않게)
        from gui_mock import MockWindow, shots
        win = MockWindow()
    else:
        win = MainWindow()
    win.resize(*map(int, args.size.split('x')))
    if args.shot:
        win.setAttribute(QtCore.Qt.WA_DontShowOnScreen)
        win.show()
        QtCore.QTimer.singleShot(200, lambda: (shots(win, Path(args.shot)), app.quit()))
    else:
        win.show()
    rc = app.exec_()
    cp.finish_jobs()                    # [R-1] 도는 일 (카메라 끄기·보정·엔코더 읽기) 을 기다린 뒤 끝 (Qt 정리 전에)
    sys.exit(rc)


if __name__ == '__main__':
    main()
