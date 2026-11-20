"""GUI 시험 공용 — 경로·앱 만들기·순서 돌리기·확인 줄 (Thor 컨테이너, 화면 없이 QT_QPA_PLATFORM=offscreen).

[2026-10-27 R-1 리팩터링] test_shots/*.py 두 벌에 있던 것을 한곳에. 경로는 이 파일 자리에서 (전엔 /home/admin/workspaces/movensys_ws/cr3a_gui
  고정 — 사본 폴더에서 돌리면 실물 폴더 코드를 읽었음).
안전: guard_test_port() = 실물 서버 포트 (47861) 로는 안 돎, refuse_real() = 붙은 서버가 실제 로봇이면 아무것도 안 보내고 끝.
"""
import os
import sys
import time
from pathlib import Path

GUI = Path(__file__).resolve().parent.parent            # cr3a_gui 폴더 (이 파일 = cr3a_gui/tests/…)
if str(GUI) not in sys.path:
    sys.path.insert(0, str(GUI))
T0 = time.time()
FAILS = []


def log(*x):
    print(f'[{time.time() - T0:6.1f}s]', *x, flush=True)


def make_app():
    from PyQt5 import QtGui, QtWidgets
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    if 'NanumGothic' in QtGui.QFontDatabase().families():
        app.setFont(QtGui.QFont('NanumGothic', 9))
    return app


def check(name, ok, detail=''):
    """확인 한 줄: PASS / FAIL + 내용. 끝에 summary() 가 FAIL 수로 끝 코드."""
    log('PASS' if ok else 'FAIL', name, f'— {detail}' if detail != '' else '')
    if not ok:
        FAILS.append(name)
    return bool(ok)


def summary():
    log(f'결과: FAIL {len(FAILS)} 개' + (f' ({", ".join(FAILS)})' if FAILS else ''))
    return 1 if FAILS else 0


def run(gen, app, on_timeout=None, period_ms=100):
    """순서 (generator) 를 GUI 타이머로 돌림 (그 사이 Qt 이벤트는 계속). 단계: ('sleep', s) · ('until', 조건, 제한 s, 이름) — 조건은
    period_ms 마다 봄, 제한을 넘으면 on_timeout(이름) 뒤 다음 단계. 순서가 끝나면 0.5 s 뒤 app.quit(). 반환: 시간을 넘긴 단계 이름들."""
    from PyQt5 import QtCore
    cur = [None, 0.0]
    timeouts = []

    def tick():
        try:
            if cur[0] is None:
                cur[0] = next(gen)
                cur[1] = time.time()
                if cur[0][0] == 'until':
                    log('기다림:', cur[0][3])
            if cur[0][0] == 'sleep':
                if time.time() - cur[1] >= cur[0][1]:
                    cur[0] = None
            else:
                _, pred, timeout, what = cur[0]
                if pred():
                    log('됨:', what, f'({time.time() - cur[1]:.1f}s)')
                    cur[0] = None
                elif time.time() - cur[1] > timeout:
                    log('시간 초과:', what)
                    timeouts.append(what)
                    if on_timeout is not None:
                        on_timeout(what)
                    cur[0] = None
        except StopIteration:
            timer.stop()
            QtCore.QTimer.singleShot(500, app.quit)
    timer = QtCore.QTimer(timeout=tick)
    timer.start(period_ms)
    app.exec_()
    return timeouts


def guard_test_port(need_tmp=True):
    """실물 서버에 붙지 않게: CR3A_GUI_PORT 가 47861 이 아닌 포트여야 함. need_tmp = CR3A_TMP_DIR 도 /tmp 가 아니어야 함 (실물 카메라 노드
    PID·서버 로그를 안 건드리게). 아니면 바로 끝냄 (common 을 가져오기 전에 환경 변수가 있어야 함)."""
    import common
    if 'CR3A_GUI_PORT' not in os.environ or common.PORT == 47861:
        sys.exit('시험 중지: CR3A_GUI_PORT 를 47861 이 아닌 포트로 (예 47873) — 실물 서버에 붙지 않게')
    if need_tmp and common.TMP_DIR.resolve() == Path('/tmp').resolve():
        sys.exit('시험 중지: CR3A_TMP_DIR 을 /tmp 가 아닌 폴더로 — 실물 카메라 노드 PID·서버 로그를 안 건드리게')
    log(f'시험 포트 {common.PORT} · 임시 폴더 {common.TMP_DIR}')


def refuse_real(win):
    """붙은 서버가 실제 로봇이면 아무것도 보내지 않고 바로 끝냄 (시험은 이상적·WMX 시뮬레이터만)."""
    if win.connected and 'real' in (win.hello.get('robot'), (win.state or {}).get('robot')):
        log('실제 로봇 서버에 붙음 — 시험 중지 (아무 명령도 안 보냄)')
        os._exit(3)


def listening(port):
    """이 컴퓨터에서 port 로 듣는 TCP 소켓이 있나 — 붙지 않고 /proc/net/tcp·tcp6 로 (붙으면 서버가 지금 GUI 연결을 끊고 이것으로 바꿈)."""
    for f in ('/proc/net/tcp', '/proc/net/tcp6'):
        try:
            rows = Path(f).read_text().splitlines()[1:]
        except OSError:
            continue
        for r in rows:
            p = r.split()
            if len(p) > 3 and p[3] == '0A' and int(p[1].rsplit(':', 1)[1], 16) == port:     # 0A = LISTEN
                return True
    return False


def refuse_if_live_server(port=47861):
    """실물 시스템 (기본 포트 서버) 이 켜져 있으면 끝냄 — 카메라 노드를 끄거나 R2 를 켜는 시험이 돌던 시스템과 섞이지 않게."""
    if listening(port):
        sys.exit(f'시험 중지: 포트 {port} 에 서버가 떠 있음 (실물 시스템이 켜져 있음) — 시스템을 끈 뒤에')
