"""카메라 시험 (Thor 컨테이너, 화면 없이 QT_QPA_PLATFORM=offscreen, 시스템 꺼진 채): 카메라 세팅 탭 [▶ 카메라 켜기] → 미리보기 →
[카메라 위치 맞추기…] → ([엔코더 읽기] — --allow-ethercat 일 때만) → [이 자세로 보정] → [여러 자세 함께 맞추기] → [등록] → 겹쳐 보기 캡처
→ [■ 끄기].  ★ 실제 고정 카메라 노드를 켜고 끔 ★
  python3 tests/gui_camera_test.py <캡처 폴더> [--allow-ethercat]
[2026-10-27 R-1] test_shots/ → tests/ (경로는 이 파일 자리에서, 순서 돌리기는 seq_util). [엔코더 읽기] (R2 일반 노드로 EtherCAT 통신 —
 WMX Module.ini 를 EtherCAT 으로 바꿨다가 되돌림, 서보 OFF) 는 --allow-ethercat 을 줄 때만 (기본 = 건너뜀 → 엔코더 자세가 없으면 보정
 단계도 건너뜀). 실물 시스템 (47861 서버) 이 켜져 있으면 안 돎 (끝에 카메라 노드를 끔), GUI 는 시험 포트로 (CR3A_GUI_PORT — 서버는 안
 씀: 47861 에 붙으면 서버가 지금 GUI 연결을 끊고 이것으로 바꿈). 카메라 노드 PID 는 /tmp 그대로 (실제 노드를 켜고 끄는 시험)."""
import sys
from pathlib import Path

import seq_util as U
from seq_util import log

U.guard_test_port(need_tmp=False)
U.refuse_if_live_server()
ALLOW_ETHERCAT = '--allow-ethercat' in sys.argv
ARGS = [a for a in sys.argv[1:] if not a.startswith('--')]
app = U.make_app()
from PyQt5 import QtWidgets  # noqa: E402
import camera_panel as cp  # noqa: E402
import cr3a_gui as G  # noqa: E402
from common import TMP_DIR  # noqa: E402

QtWidgets.QMessageBox.question = staticmethod(lambda *a, **k: QtWidgets.QMessageBox.Yes)     # 화면 없음: 묻는 창은 '예'
OUT = Path(ARGS[0]) if ARGS else TMP_DIR / 'cr3a_cam_test'
OUT.mkdir(parents=True, exist_ok=True)
win = G.MainWindow()
G.keep_running_on_errors()
win.resize(1600, 1000)
win.show()


def shot(name, w=None):
    app.processEvents()
    (w or win).grab().save(str(OUT / f'{name}.png'))
    log('shot', name)


def cw():
    return win.calib_win


def seq():
    yield ('sleep', 1.0)
    U.refuse_real(win)
    win.tabs.setCurrentIndex(2)
    win.on_cam_on()
    yield ('until', lambda: win.cam_feed is not None and win.cam_feed.fps('color') > 5, 40, '카메라 영상')
    yield ('sleep', 1.5)
    log('영상 fps', round(win.cam_feed.fps('color'), 1))
    shot('c01_camera_tab')
    win.on_calib()
    cw().resize(1180, 760)
    yield ('sleep', 2.0)
    shot('c02_calib_live', cw())
    if ALLOW_ETHERCAT:
        cw().on_read_encoders()
        yield ('until', lambda: cw().q_enc is not None and not cw().busy, 180, '엔코더 읽기')
        log('엔코더', None if cw().q_enc is None else [round(float(v), 3) for v in cw().q_enc[0]])
    else:
        log('엔코더 읽기 건너뜀 (--allow-ethercat 없음 — EtherCAT 통신을 켜지 않음)')
    if cw().q_enc is not None:
        cw().on_capture()
        yield ('until', lambda: len(cw().rows) == 1 and not cw().busy, 400, '자세 1 보정')
    else:
        log('보정 건너뜀 (실제 로봇 관절각 없음)')
    if cw().rows:
        cw().table.selectRow(0)
        cw().cb_pose_src.setCurrentIndex(1)
        yield ('sleep', 1.5)
        shot('c03_pose1_overlay', cw())
        cw().on_fit_all()
        yield ('until', lambda: cw().joint is not None and not cw().busy, 200, '여러 자세 함께 맞추기')
        cw().on_register()
        cw().cb_pose_src.setCurrentIndex(0)
        yield ('sleep', 1.5)
        shot('c04_registered', cw())
    win.view_tabs.setCurrentIndex(0)
    shot('c05_main_after')
    log('보정 창 로그:\n' + cw().log.toPlainText())
    win.on_cam_off()
    yield ('until', lambda: cp.CameraProcess.pid() is None and not win._cam_busy, 20, '카메라 끔')
    log('끝')


U.run(seq(), app)
cp.finish_jobs()
