#!/usr/bin/env python3
"""CR3A GUI 카메라 — 카메라 위치 맞추기 (보정) 창 + 오래 걸리는 일 돌리기 (QThreadPool).  ★ wmx_r2 컨테이너 안 (ROS) ★

[2026-10-26] 미로봇 GUI camera_panel.py 의 보정 창을 CR3A 로 (계산은 camera_fit.py):
- 카메라: realsense2_camera 노드를 GUI 와 따로 띄움 (GUI 를 닫아도 계속 — 서버가 실제 카메라 지도에 씀). 시리얼 333422303104
  (= USB 412243062648, 포트 2-3.2). 손목 D455 (333422300962) 는 지금 안 씀 (사용자). 컬러 + 깊이 원본 (+ 컬러 맞춤 깊이: 미리보기).
- 보정 창: 실시간 컬러 영상 위에 '지금 관절각' 의 로봇 모델을 지금 카메라 위치로 겹쳐 그림 (겹치면 맞은 것). [이 자세로 보정] = 깊이 10 장
  가운데값 + 그 순간 관절각 → 자동 맞추기 (처음 자세는 시작 값 자동 ~100 s, 다음부터 아는 위치에서 ~20 s) → 줄 하나. 로봇을 다른 자세로
  옮겨 가며 여러 줄 → [여러 자세 함께 맞추기] (camera_fit.fit_many: 단순 평균에서 시작해 모든 자세의 깊이 점을 카메라 위치 하나로) → [등록]
  = camera_pose.json (서버·3D 가 읽음, 이전 것은 보정 폴더에 보관). 미로봇에서 자세 하나의 답은 옆 이동·좌우 돌림 방향으로 자세마다
  2~3 cm·3° 갈렸음 → 여러 자세 권장.
- 관절각 (사용자 2026-10-26 "실물 로봇을 움직일 때도 엔코더 신호 기반"): 실물 연결 = 시스템이 읽는 /joint_states (R2 = 엔코더
  actualPos) — 찍기 전 1 s 동안 멈춰 있어야 함 (움직이면 깊이 영상과 관절각 시각이 어긋남). 시스템이 꺼져 있으면 [엔코더 읽기]
  (wmx_r2.sh encoders: EtherCAT 통신만, 서보 OFF). WMX 시뮬레이터로 켜져 있으면 관절각이 실제 로봇 것이 아니라 막음.
[2026-10-27 R-1 리팩터링] PyQt 없는 부분 (카메라 노드·영상 받기·카메라 위치 파일) 은 camera_io.py — 이름은 여기서 그대로 다시 내보냄
  (rt_server 의 cp.load_pose·cp.CameraFeed·cp.CameraProcess, 작업기록 스크립트). 보정 창은 MainWindow 를 직접 읽지 않고 받은 함수
  (sys_state) 로 시스템 상태를 보고, 로봇 모델은 MainWindow 것을 같이 씀. 오래 걸리는 일 (보정·엔코더 읽기·카메라 끄기) = run_job
  (QThreadPool), 글·결과는 신호로 GUI 스레드에. 영상 타이머·깊이 받기는 창이 보일 때만.
"""
import json
import logging
import subprocess
import threading
import time

import cv2
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

from camera_io import (CAM_LOG, CAM_NS, CAM_PIDF, CAM_SERIAL, CAM_TOPIC, ENC_FILE, HERE, N_DEPTH, POSE_FILE,  # noqa: F401 (다시 내보냄)
                       CameraFeed, CameraProcess, load_pose, save_pose, write_json_atomic)
from common import LOG_ROOT, link_from_optical, to_qimage  # noqa: F401 (LOG_ROOT 다시 내보냄)

STILL_DEG = 0.01                                # 실물 관절각: 찍기 전 1 s 동안 이 안에서 멈춰 있어야 함
LIVE_STEP_M = 0.006                             # 실시간 겹쳐 보기 모델 점 간격 (보정 계산은 3 mm)
VIEW_S = 0.2                                    # 실시간 영상 다시 그리기 간격
LOG = logging.getLogger('cr3a_gui')


# ---------------- 오래 걸리는 일 (QThreadPool) ----------------
_RUNNING = {}                       # 일의 신호 객체 (id → 객체): 결과가 GUI 스레드에 닿을 때까지 붙잡아 둠 (그 전에 지워지지 않게)
CANCEL = threading.Event()          # 프로그램 끝: 보정 계산을 다음 시작 값에서 멈춤 (camera_fit should_stop) — 끝내기가 오래 안 걸리게
FINISH_WAIT_S = 180.0               # 프로그램 끝: 도는 일을 이만큼까지 기다림 (엔코더 읽기 ~1 분이 다 끝나게 — 걸린 일이면 그대로 끝냄)


class _JobSignals(QtCore.QObject):
    done = QtCore.pyqtSignal(object)        # fn 의 반환값
    failed = QtCore.pyqtSignal(object)      # 예외 (받는 쪽이 로그로 — 자세한 것은 exc_info)


class _Job(QtCore.QRunnable):
    def __init__(self, fn, sig):
        super().__init__()
        self.fn, self.sig = fn, sig

    def run(self):
        try:
            try:
                out = self.fn()
            except Exception as e:                  # 일 하나의 오류로 GUI 가 죽지 않게
                self.sig.failed.emit(e)
            else:
                self.sig.done.emit(out)
        except RuntimeError:                        # (프로그램 끝) 신호 객체가 이미 지워짐
            pass


def run_job(fn, on_done=None, on_failed=None):
    """fn() 을 QThreadPool 에서 (GUI 스레드를 막지 않게). 끝나면 on_done(반환값) / on_failed(예외) — GUI 스레드에서 (신호).
    [R-1] 전엔 threading.Thread + 결과를 타이머가 큐·깃발로 가져감."""
    sig = _JobSignals()                             # (GUI 스레드에서 만듦 — 받는 쪽이 다 받은 뒤 아래 마지막 연결이 놓음)
    key = id(sig)
    for s, slot in ((sig.done, on_done), (sig.failed, on_failed)):
        if slot is not None:
            s.connect(slot)
        s.connect(lambda *_, k=key: _RUNNING.pop(k, None))     # 받는 쪽 다음 (연결 차례) — 그 뒤 놓음
    _RUNNING[key] = sig
    QtCore.QThreadPool.globalInstance().start(_Job(fn, sig))


def finish_jobs():
    """프로그램 끝 (app.exec_ 뒤): 보정 계산은 멈추게 하고 (CANCEL) 도는 일이 끝날 때까지 기다림 (최대 FINISH_WAIT_S). [R-1 Thor 시험]
    QRunnable 이 도는 채 끝내면 Qt 가 먼저 정리돼 신호 객체가 지워지고 프로세스가 죽음 (abort). 엔코더 읽기는 끝까지 기다림 — 중간에
    끊으면 WMX 설정 (Module.ini) 이 안 돌아올 수 있음. 보정 계산은 맞추기 한 번 (몇 s), 카메라 끄기는 최대 7 s."""
    CANCEL.set()
    pool = QtCore.QThreadPool.globalInstance()
    if pool.activeThreadCount():
        print(f'cr3a_gui: 도는 일 {pool.activeThreadCount()} 개가 끝나기를 기다림 (최대 {FINISH_WAIT_S:.0f} s)', flush=True)
    if not pool.waitForDone(int(FINISH_WAIT_S * 1000)):        # (기다리는 동안 GIL 을 놓음 — 일 스레드가 계속 돎)
        print('cr3a_gui: 일이 안 끝남 — 그대로 끝냄', flush=True)


# ---------------- 보정 창 ----------------
class CalibWindow(QtWidgets.QWidget):
    """카메라 위치 맞추기 (보정용 창). sys_state() = (시스템 켜짐?, 로봇 종류 'ideal'·'wmx'·'real'·None, 관절각 ° 또는 None) —
    [R-1] MainWindow 를 직접 읽지 않고 받은 함수로. robot = MainWindow 의 cr3a_model.Robot (메시를 두 번 읽지 않게)."""
    pose_registered = QtCore.pyqtSignal(object)
    said = QtCore.pyqtSignal(str)               # [R-1] 일 스레드의 글 → GUI 스레드 (로그 칸). 전엔 큐를 타이머가 꺼냄

    def __init__(self, sys_state, feed: CameraFeed, robot):
        super().__init__(None, QtCore.Qt.Window)
        self.sys_state, self.feed, self.robot = sys_state, feed, robot
        self.setWindowTitle('카메라 위치 맞추기 (보정)')
        self.resize(1180, 760)
        self.q_enc = None                # [엔코더 읽기] 결과 (°, 시각)
        self.rows = []                   # 자세마다 {'snap', 'q', 'src', 'fitter', 'res', 'dir'}
        self.joint = None                # 여러 자세 함께 맞춘 결과
        self.busy = False
        self._surf = self._surf_live = None
        self._q_hist = []                # 실물 관절각 기록 (멈춤 판정)
        self._n_shown, self._joint_shown = -1, None     # 표에 그린 줄 수·여러 자세 결과 (바뀔 때만 다시 채움)
        self._reg_key, self._reg = None, None           # 등록한 카메라 위치 (파일이 바뀔 때만 다시 읽음 — _registered)
        self.session = LOG_ROOT / time.strftime('calib_%Y%m%d_%H%M%S')
        self.said.connect(self._append_log)
        self._build()
        self.timer = QtCore.QTimer(self, interval=int(VIEW_S * 1000), timeout=self._tick)     # 보일 때만 (showEvent·hideEvent)

    # ---- 화면 ----
    def _build(self):
        h = QtWidgets.QHBoxLayout(self)
        left = QtWidgets.QVBoxLayout()
        self.lb_view = QtWidgets.QLabel(alignment=QtCore.Qt.AlignCenter)
        self.lb_view.setMinimumSize(640, 480)
        self.lb_view.setStyleSheet('background:#111;color:#bbb;')
        left.addWidget(self.lb_view, 1)
        r = QtWidgets.QHBoxLayout()
        self.chk_overlay = QtWidgets.QCheckBox('로봇 모델 겹쳐 보기', checked=True)
        self.chk_overlay.setToolTip('지금 관절각의 로봇 모델 (링크마다 색) 을 지금 카메라 위치로 겹쳐 그림 — 실제 팔과 겹치면 맞은 것')
        self.cb_pose_src = QtWidgets.QComboBox()
        self.cb_pose_src.addItems(['등록된 위치', '선택한 줄', '여러 자세 결과'])
        self.cb_pose_src.setToolTip('겹쳐 볼 카메라 위치')
        r.addWidget(self.chk_overlay)
        r.addWidget(QtWidgets.QLabel('카메라 위치:'))
        r.addWidget(self.cb_pose_src)
        r.addStretch(1)
        self.lb_cam = QtWidgets.QLabel()
        r.addWidget(self.lb_cam)
        left.addLayout(r)
        h.addLayout(left, 3)

        right = QtWidgets.QVBoxLayout()
        gb = QtWidgets.QGroupBox('관절각 (로봇의 실제 자세)')
        g = QtWidgets.QVBoxLayout(gb)
        self.lb_q = QtWidgets.QLabel(wordWrap=True, textFormat=QtCore.Qt.RichText)
        g.addWidget(self.lb_q)
        self.btn_enc = QtWidgets.QPushButton('엔코더 읽기 (시스템 꺼져 있을 때)', clicked=self.on_read_encoders)
        self.btn_enc.setToolTip('R2 로 EtherCAT 통신만 켜서 6 축 엔코더 위치를 읽고 끔 (서보는 안 켬 — 로봇 안 움직임).\n'
                                '시스템 (서버) 이 꺼져 있을 때만. 끝나면 WMX 설정은 원래대로 (시뮬레이터였으면 시뮬레이터로).')
        g.addWidget(self.btn_enc)
        right.addWidget(gb)
        gb = QtWidgets.QGroupBox('자세마다 보정 → 여러 자세 함께')
        g = QtWidgets.QVBoxLayout(gb)
        r = QtWidgets.QHBoxLayout()
        self.btn_cap = QtWidgets.QPushButton('이 자세로 보정', clicked=self.on_capture)
        self.btn_cap.setToolTip('깊이 10 장 가운데값 + 지금 관절각 → 카메라 위치 계산 (로봇은 안 움직임).\n'
                                '처음 자세는 시작 값을 자동으로 찾아 ~100 s, 다음 자세부터는 아는 위치에서 ~20 s.\n'
                                '자세를 바꿔 가며 3~5 번 하면 [여러 자세 함께 맞추기] 가 더 정확.')
        self.btn_del = QtWidgets.QPushButton('선택 줄 지우기', clicked=self.on_delete)
        r.addWidget(self.btn_cap)
        r.addWidget(self.btn_del)
        g.addLayout(r)
        self.table = QtWidgets.QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(['#', '로봇 점', '중앙값 mm', '테이블 높이 / 고르기 mm', '판정', 'x y z mm · Rx Ry Rz °'])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.itemSelectionChanged.connect(lambda: self.cb_pose_src.setCurrentIndex(1) if self.rows else None)
        g.addWidget(self.table, 1)
        r = QtWidgets.QHBoxLayout()
        self.btn_joint = QtWidgets.QPushButton('여러 자세 함께 맞추기', clicked=self.on_fit_all)
        self.btn_joint.setToolTip('단순 평균에서 시작해 모든 자세의 깊이 점이 카메라 위치 하나에 붙게 함께 맞춤 (미로봇 fit_many).\n'
                                  '자세를 잘 잡는 방향 (옆 이동·좌우 돌림) 이 자세마다 달라 숫자 평균보다 정확.')
        self.btn_reg = QtWidgets.QPushButton('등록', clicked=self.on_register)
        self.btn_reg.setToolTip('여러 자세 결과 (없으면 선택한 줄) 를 카메라 위치로 저장 → 3D·서버가 씀. 이전 것은 보정 폴더에 보관.')
        r.addWidget(self.btn_joint)
        r.addWidget(self.btn_reg)
        g.addLayout(r)
        self.lb_joint = QtWidgets.QLabel(wordWrap=True, textFormat=QtCore.Qt.RichText)
        g.addWidget(self.lb_joint)
        right.addWidget(gb, 1)
        self.log = QtWidgets.QPlainTextEdit(readOnly=True)
        self.log.setMaximumBlockCount(500)
        self.log.setMinimumHeight(140)
        right.addWidget(self.log)
        h.addLayout(right, 2)

    def showEvent(self, ev):
        self.feed.set_depth(True)               # [R-1] 깊이는 이 창이 보일 때만 받음 (보정 = 깊이 10 장)
        self.timer.start()
        super().showEvent(ev)

    def hideEvent(self, ev):
        self.timer.stop()                       # [R-1] 안 보이면 영상·겹쳐 그리기 타이머를 멈춤
        if not self.busy:                       # (보정 중이면 깊이를 끝까지 받음 — 끝나면 _set_busy 가 끔)
            self.feed.set_depth(False)
        super().hideEvent(ev)

    def say(self, text):
        self.said.emit(text)                    # (일 스레드에서도 — 신호는 GUI 스레드로 줄 서서 감)

    def _append_log(self, text):
        self.log.appendPlainText(time.strftime('%H:%M:%S ') + text)

    def _set_busy(self, on):
        self.busy = on
        for b in (self.btn_cap, self.btn_enc, self.btn_joint, self.btn_reg, self.btn_del):
            b.setEnabled(not on)
        if not on and not self.isVisible():
            self.feed.set_depth(False)

    # ---- 관절각 ----
    def joint_angles(self):
        """보정에 쓸 실제 로봇 관절각 (°) 과 출처 — 못 쓰면 (None, 이유)."""
        on, robot, q = self.sys_state()
        if on and robot == 'real':
            hist = [x for t, x in self._q_hist if time.time() - t < 1.0]
            if len(hist) < 3 or np.max(np.ptp(np.array(hist), axis=0)) > STILL_DEG:
                return None, '로봇이 움직이는 중 — 1 s 멈춘 뒤 다시 (깊이 영상과 관절각 시각이 어긋나지 않게)'
            return q, '실물 엔코더 (시스템)'
        if on:
            return None, (f'시스템이 {"WMX 시뮬레이터" if robot == "wmx" else "이상적 시뮬레이션"} 로 켜져 있어 관절각이 실제 로봇 것이 아님 '
                          '— 시스템을 끄고 [엔코더 읽기]')
        if self.q_enc is None:
            return None, '실제 로봇 관절각이 없음 — [엔코더 읽기] 먼저'
        q, t = self.q_enc
        return q, f'엔코더 읽기 ({time.strftime("%H:%M:%S", time.localtime(t))})'

    def on_read_encoders(self):
        if self.busy:
            return
        on, _, _ = self.sys_state()
        if on:
            self.say('시스템이 켜져 있음 — 엔코더 읽기는 시스템을 끈 뒤 (WMX 엔진을 EtherCAT 으로 켜야 해서)')
            return
        out = self.session / time.strftime('enc_%H%M%S')

        def job():
            self.say('엔코더 읽기: R2 일반 노드로 EtherCAT 통신만 (서보 OFF) …')
            p = subprocess.Popen(['bash', str(HERE / 'wmx_r2.sh'), 'encoders', str(out)], stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True)
            for line in p.stdout:
                if line.strip():
                    self.say('  ' + line.rstrip())
            rc = p.wait()
            try:
                d = json.loads((out / 'encoders.json').read_text())
                if rc == 0 and not any(d['servo_on']):
                    self.q_enc = (np.array(d['q_deg'], float), time.time())
                    write_json_atomic(ENC_FILE, {'q_deg': d['q_deg'], 'time': time.strftime('%Y-%m-%d %H:%M:%S'),
                                                 'file': str(out / 'encoders.json')})
                    self.say(f'엔코더 (°): {np.round(d["q_deg"], 3).tolist()}')
                    return
            except (OSError, ValueError, KeyError):
                pass
            self.say(f'엔코더 읽기 실패 (코드 {rc})')
        self._run(job)

    # ---- 보정 ----
    def _model(self, live=False):
        import camera_fit as cf
        if self._surf is None:
            self._surf = cf.RobotSurface(self.robot, cf.ARM_LINKS)
            self._surf_live = cf.RobotSurface(self.robot, cf.ARM_LINKS + ('tool_gripper',), step=LIVE_STEP_M)
        return self._surf_live if live else self._surf

    def _registered(self):
        """등록한 카메라 위치 (camera_pose.json) — 파일이 바뀔 때만 다시 읽음 (영상 타이머가 0.2 s 마다 부름)."""
        try:
            st = POSE_FILE.stat()
            key = (st.st_ino, st.st_mtime_ns, st.st_size)
        except OSError:
            key = None
        if key != self._reg_key:
            self._reg_key, self._reg = key, (None if key is None else load_pose())
        return self._reg

    def _known_T(self):
        """다음 자세의 시작 값: 여러 자세 결과 > 가장 좋은 줄 > 등록된 위치 > 없음 (자동)."""
        if self.joint is not None:
            return self.joint['T']
        good = [r for r in self.rows if r['res']['ok']]
        if good:
            return max(good, key=lambda r: r['res']['score']['robot'])['res']['T']
        reg = self._registered()
        return None if reg is None else np.array(reg['T_base_depth'], float)

    def on_capture(self):
        if self.busy:
            return
        if not (self.feed.ok and CameraProcess.pid()):
            self.say('카메라가 꺼져 있음 — 카메라 세팅 탭에서 [▶ 카메라 켜기]')
            return
        q, src = self.joint_angles()
        if q is None:
            self.say(src)
            return
        T0 = self._known_T()

        def job():
            import camera_fit as cf
            self.say(f'자세 {len(self.rows) + 1}: 깊이 {N_DEPTH} 장 모으기 ({src}, 관절각 {np.round(q, 2).tolist()}°)')
            snap = self.feed.snapshot()
            if snap is None:
                self.say('영상을 못 받음 (카메라 노드 확인)')
                return
            d = self.session / f'pose{len(self.rows) + 1}'
            d.mkdir(parents=True, exist_ok=True)
            np.save(d / 'depth_raw.npy', snap['depth'])
            cv2.imwrite(str(d / 'color.png'), snap['color'])
            (d / 'encoders.json').write_text(json.dumps({'q_deg': [float(x) for x in q], 'src': src}))
            (d / 'info.json').write_text(json.dumps({'serial': CAM_SERIAL, 'depth': {'K': snap['Kd'].ravel().tolist(),
                                                    'w': int(snap['depth'].shape[1]), 'h': int(snap['depth'].shape[0])},
                                                    'color': {'K': snap['Kc'].ravel().tolist(), 'D': snap['Dc']},
                                                    'T_color_depth': snap['T_cd'].tolist()}))
            fitter = cf.PoseFitter(snap['depth'], snap['Kd'], q, self._model())
            if T0 is None:
                self.say('처음 자세: 시작 값을 자동으로 찾음 (테이블 평면 + 높이 지도, ~100 s)')
                res = cf.calibrate(fitter, log=self.say, should_stop=CANCEL.is_set)
            else:
                res = cf.refine(fitter, T0, log=self.say)
            self.rows.append({'snap': snap, 'q': q, 'src': src, 'fitter': fitter, 'res': res, 'dir': d})
            self.joint = None
            self.say(f'자세 {len(self.rows)} 끝: {res["why"]} (자료 {d})')
        self._run(job)

    def on_delete(self):
        rows = sorted({i.row() for i in self.table.selectedIndexes()}, reverse=True)
        for i in rows:
            if 0 <= i < len(self.rows):
                self.rows.pop(i)
        if rows:
            self.joint = None
            self.say(f'줄 {", ".join(str(i + 1) for i in sorted(rows))} 지움')

    def on_fit_all(self):
        if self.busy or not self.rows:
            return

        def job():
            import camera_fit as cf
            self.say(f'여러 자세 함께 맞추기: 자세 {len(self.rows)} 개')
            out = cf.fit_many([r['fitter'] for r in self.rows], [r['res']['T'] for r in self.rows], log=self.say,
                              should_stop=CANCEL.is_set)
            ok, why = cf.verdict_many(out['scores'], out['agree'], out['tried'], out['starts'])
            out.update(ok=ok, why=why)
            self.joint = out
            d, a = cf.pose_diff(out['T'], out['mean'])
            self.say(f'여러 자세 결과: {why} (단순 평균과 {d:.1f} mm · {a:.2f}° 차)')
        self._run(job)

    def on_register(self):
        import camera_fit as cf
        if self.joint is not None and self.joint.get('T') is not None:
            T, scores, ok, why, n = self.joint['T'], self.joint['scores'], self.joint['ok'], self.joint['why'], len(self.rows)
        else:
            sel = sorted({i.row() for i in self.table.selectedIndexes()})
            if not sel and len(self.rows) == 1:
                sel = [0]
            if len(sel) != 1:
                self.say('등록할 것을 고르기: [여러 자세 함께 맞추기] 를 하거나 줄 하나를 선택')
                return
            r = self.rows[sel[0]]
            T, scores, ok, why, n = r['res']['T'], [r['res']['score']], r['res']['ok'], r['res']['why'], 1
        if not ok:
            b = QtWidgets.QMessageBox.question(self, '판정이 좋지 않음', f'{why}\n그래도 등록할까요?')
            if b != QtWidgets.QMessageBox.Yes:
                return
        snap = self.rows[0]['snap']
        # [2026-10-27 D-2] 테이블 높이 (받침 원점 기준, 자세마다 잰 값의 가운데값) — 서버가 지도 바닥 띠·충돌 바닥에 씀.
        #  [D-2 검토] 못 쟀으면 (테이블 점 적음) None 그대로 — 0 mm 로 저장하면 '잰 값 0' 과 구분이 안 됨 (서버는 None 이면 0 으로 씀)
        tz = [s['table_z_mm'] for s in scores if s and s.get('table_z_mm') is not None]
        table_z = round(float(np.median(tz)) / 1000, 4) if tz else None
        data = {'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'serial': CAM_SERIAL, 'poses': n, 'ok': ok, 'why': why, 'table_z_m': table_z,
                'T_base_depth': T.tolist(), 'T_base_color': (T @ np.linalg.inv(snap['T_cd'])).tolist(),
                'camera_link_xyzrpy': cf.pose_xyzrpy(link_from_optical(T)), 'depth_optical_xyzrpy': cf.pose_xyzrpy(T),
                'depth_K': snap['Kd'].tolist(), 'depth_wh': [int(snap['depth'].shape[1]), int(snap['depth'].shape[0])],
                'color_K': snap['Kc'].tolist(), 'color_D': snap['Dc'], 'T_color_depth': snap['T_cd'].tolist(),
                'scores': scores, 'q_deg': [[round(float(x), 4) for x in r['q']] for r in self.rows], 'session': str(self.session)}
        save_pose(data, self.session)
        tzs = '안 잼 (서버는 0)' if table_z is None else f'{table_z * 1000:+.1f} mm'
        self.say(f'등록: camera_link {data["camera_link_xyzrpy"]} · 테이블 높이 {tzs} (자세 {n} 개, {why}) → {POSE_FILE}'
                 ' — 서버는 다음 [▶ 시스템 시작] 때 읽음')
        self.cb_pose_src.setCurrentIndex(0)
        self.pose_registered.emit(data)

    def _run(self, fn):
        self._set_busy(True)
        run_job(fn, on_done=lambda _: self._set_busy(False), on_failed=self._job_failed)

    def _job_failed(self, e):
        self.say(f'오류: {e!r}')                # 보정 오류로 GUI 가 죽지 않게 (자세한 것은 GUI 로그 파일)
        LOG.error('보정 창 일 오류: %r', e, exc_info=(type(e), e, e.__traceback__), extra={'view': False})
        self._set_busy(False)

    # ---- 주기 (보일 때 0.2 s) ----
    def _pose_for_view(self):
        k = self.cb_pose_src.currentIndex()
        if k == 2 and self.joint is not None:
            return self.joint['T'], self.rows[0]['snap']['T_cd'] if self.rows else None
        if k == 1:
            sel = sorted({i.row() for i in self.table.selectedIndexes()})
            if sel and sel[0] < len(self.rows):
                r = self.rows[sel[0]]
                return r['res']['T'], r['snap']['T_cd']
        reg = self._registered()
        if reg is None:
            return None, None
        return np.array(reg['T_base_depth'], float), np.array(reg['T_color_depth'], float)

    def _tick(self):
        on, robot, q = self.sys_state()
        if on and robot == 'real' and q is not None:
            self._q_hist.append((time.time(), q))
            self._q_hist = [x for x in self._q_hist if time.time() - x[0] < 2.0]
        qv, src = self.joint_angles()
        self.lb_q.setText(('관절각 (°): ' + ' '.join(f'{v:.2f}' for v in qv) + f'<br><span style="color:#57606a">{src}</span>')
                          if qv is not None else f'<span style="color:#9a6700">{src}</span>')
        self._fill_table()
        fps = self.feed.fps()
        self.lb_cam.setText(f'카메라 {"켜짐" if CameraProcess.pid() else "꺼짐"} · 깊이 {fps:.1f} fps' if self.feed.ok else self.feed.err)
        img, age = self.feed.live()
        if img is None or age > 2.0:
            self.lb_view.setText('카메라 영상 없음 — 카메라 세팅 탭에서 [▶ 카메라 켜기]')
            return
        T, T_cd = self._pose_for_view()
        intr = self.feed.intrinsics()
        if self.chk_overlay.isChecked() and T is not None and qv is not None and intr is not None:
            import camera_fit as cf
            P, names = self._model(live=True).at(qv)
            T_color = (T_cd if T_cd is not None else intr[3]) @ np.linalg.inv(T)
            img = cf.overlay(img, T_color, intr[1], intr[2], P, names)          # (새 영상을 만들어 돌려줌 — 받은 영상은 안 바꿈)
        self.lb_view.setPixmap(QtGui.QPixmap.fromImage(to_qimage(img)).scaled(self.lb_view.size(), QtCore.Qt.KeepAspectRatio,
                                                                              QtCore.Qt.SmoothTransformation))

    def _fill_table(self):
        if self.table.rowCount() == len(self.rows) and self._n_shown == len(self.rows):
            if self.joint is None or self._joint_shown is self.joint:
                return
        import camera_fit as cf
        self._n_shown = len(self.rows)
        self.table.setRowCount(len(self.rows))
        for i, r in enumerate(self.rows):
            s = r['res']['score']
            vals = [str(i + 1), str(s['robot']), f'{s["median_mm"]:.2f}',
                    '' if s['plane_mm'] is None else (f'{s["table_z_mm"]:+.1f} / ' if s.get('table_z_mm') is not None else '') + f'{s["plane_mm"]:.2f}',
                    r['res']['why'], ' '.join(f'{v:.1f}' for v in cf.pose_xyzrpy(r['res']['T']))]
            for j, v in enumerate(vals):
                it = QtWidgets.QTableWidgetItem(v)
                if j == 4:
                    it.setForeground(QtGui.QColor('#1a7f37' if r['res']['ok'] else '#cf222e'))
                    it.setToolTip(v)
                self.table.setItem(i, j, it)
        self._joint_shown = self.joint
        if self.joint is not None and self.joint.get('T') is not None:
            self.lb_joint.setText(f'<b>여러 자세 결과</b> ({len(self.rows)} 개): {self.joint["why"]} · 받침 기준 깊이 카메라 '
                                  + ' '.join(f'{v:.1f}' for v in cf.pose_xyzrpy(self.joint['T'])))
        else:
            self.lb_joint.setText('')
