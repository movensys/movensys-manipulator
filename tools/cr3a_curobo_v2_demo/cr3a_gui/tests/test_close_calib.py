"""[2026-10-27 R-1 ③] 보정 창이 열린 채 주 창을 닫으면 프로세스가 끝나나 — 시험 (c). 전엔 보정 창 (따로 뜬 최상위 창) 때문에 앱이 남아
ping 을 계속 보내 서버가 GUI 끊김을 못 봤음 (로봇을 안 세움).
가짜 서버 (시험 포트·같은 인증, hello·state 만 보내고 받은 것을 적음) + 자식 프로세스 GUI:
  자식: 연결 → [카메라 위치 맞추기…] (보정 창) → 주 창 닫기 → 묻는 창 [그냥 닫기] → (여기서 프로세스가 끝나야 함, 못 끝나면 30 s 뒤)
  부모: 자식이 닫은 뒤 10 s 안에 끝나나 (코드 0) · 가짜 서버가 연결 닫힘을 봤나 · 닫은 뒤 ping 이 안 왔나
[R-1 서버 거절] --refuse: 가짜 서버가 모든 연결을 거절 ({'type': 'refused', 'why'} 보내고 닫음 — rt_server Link._refuse 와 같음):
  자식: 거절 알림 → 4 s 기다림 → [↻ 다시 연결] → 다시 거절 → 창 닫기
  부모: 저절로 다시 붙지 않음 (4 s 동안 연결 시도 1 번) · 알림 한 번 · 칩·단추 · 다시 연결이 서버를 새로 안 띄움 · 닫으면 끝남
  CR3A_GUI_PORT=47873 CR3A_TMP_DIR=<폴더> XDG_CONFIG_HOME=<폴더> python3 tests/test_close_calib.py [--refuse]   (tests/run_r1_tests.sh)
"""
import subprocess
import sys
import threading
import time
from multiprocessing.connection import Listener

import seq_util as U
from seq_util import check

U.guard_test_port()
import common  # noqa: E402

STATE = {'type': 'state', 'q': [0.0] * 6, 'u': [0.0] * 6, 'y': [0.0] * 6, 'mode': 'hold', 'label': '시험', 'job': None, 'goal': None,
         'realtime': True, 'estop': False, 'jog': False, 'ready': True, 'clr_mm': None, 'guard': None, 'skipped': False, 'mpc_ms': None,
         'cyc_ms': None, 'lead_ms': None, 'objects': False, 'robot': 'ideal', 'fault': None, 'speed': 0.2, 'servo': True,
         'camera': 'virtual', 'cam_fps': None, 'js_age_ms': None, 'comm_left_s': None}
WHY = '서버가 이미 다른 GUI 와 연결돼 있음 — 그 GUI 를 닫은 뒤 다시 (시험)'
REFUSE = '--refuse' in sys.argv


def child():
    app = U.make_app()
    from PyQt5 import QtCore, QtWidgets
    import camera_panel as cp
    import cr3a_gui as G
    win = G.MainWindow()
    G.keep_running_on_errors()
    win.show()

    def answer():                                   # 묻는 창 → [그냥 닫기]
        box = QtWidgets.QApplication.activeModalWidget()
        if box is None:
            return QtCore.QTimer.singleShot(100, answer)
        for b in box.buttons():
            if b.text() == '그냥 닫기':
                print('child: [그냥 닫기]', flush=True)
                b.click()

    def seq():
        yield ('until', lambda: win.connected and win.state is not None, 20, '가짜 서버 연결')
        win.on_calib()
        yield ('sleep', 1.0)
        print(f'child: 보정 창 보임 {win.calib_win.isVisible()}', flush=True)
        print(f'child: 닫기 {time.time():.3f}', flush=True)
        QtCore.QTimer.singleShot(300, answer)
        QtCore.QTimer.singleShot(0, win.close)      # (순서 밖에서 — 묻는 창이 이 순서 타이머 안에서 돌지 않게)
        yield ('sleep', 30)                         # 고친 GUI 는 여기서 오래 안 있음 (닫으면 프로그램 끝)

    def seq_refused():
        def notices():
            return win.log_view.toPlainText().count('연결을 거절함')
        yield ('until', lambda: win.refused_why is not None, 20, '거절 알림')
        print(f'child: 거절 {time.time():.3f}', flush=True)
        yield ('sleep', 4.0)                        # (저절로 다시 붙지 않아야 — 부모가 연결 시도를 셈)
        print(f'child: 상태 칩={win.chips["robot"].text()} 단추={win.btn_stack.text()} 연결={win.connected} 알림={notices()}', flush=True)
        win.on_toggle_stack()                       # [↻ 다시 연결]
        print(f'child: 다시 연결 {time.time():.3f} 서버띄움={win.server_proc is not None}', flush=True)
        yield ('until', lambda: win.refused_why is not None, 10, '다시 거절')
        print(f'child: 다시 거절 알림={notices()}', flush=True)
        print(f'child: 닫기 {time.time():.3f}', flush=True)
        QtCore.QTimer.singleShot(0, win.close)      # (연결 안 됨 → 묻지 않고 끝)
        yield ('sleep', 30)
    U.run(seq_refused() if REFUSE else seq(), app)
    print(f'child: 이벤트 고리 끝 {time.time():.3f}', flush=True)
    cp.finish_jobs()


def fake_server():
    lst = Listener(('127.0.0.1', common.PORT), authkey=common.AUTH)
    rec = {'msgs': [], 'eof': None}

    def serve():
        c = lst.accept()
        c.send({'type': 'hello', 'robot': 'ideal', 'camera': 'virtual', 'realtime': True, 'speed': 0.2, 'ready': True})
        stop = threading.Event()

        def pub():
            while not stop.is_set():
                try:
                    c.send(dict(STATE, t=time.time()))
                except Exception:
                    return
                time.sleep(0.1)
        threading.Thread(target=pub, daemon=True).start()
        while True:
            try:
                m = c.recv()
            except (EOFError, OSError):
                rec['eof'] = time.time()
                stop.set()
                return
            rec['msgs'].append((time.time(), m.get('cmd') if isinstance(m, dict) else str(m)))
    threading.Thread(target=serve, daemon=True).start()
    return lst, rec


def fake_refuser():
    """모든 연결을 거절하는 가짜 서버 (rt_server Link._refuse 와 같은 메시지) — 연결 시도 시각을 적음."""
    lst = Listener(('127.0.0.1', common.PORT), authkey=common.AUTH)
    attempts = []

    def serve():
        while True:
            try:
                c = lst.accept()
            except OSError:
                return
            attempts.append(time.time())
            try:
                c.send({'type': 'refused', 'why': WHY})
                c.close()
            except OSError:
                pass
    threading.Thread(target=serve, daemon=True).start()
    return lst, attempts


def run_child(extra):
    """자식 GUI 를 띄워 'child: 닫기 <시각>' 뒤 10 s 안에 끝나기를 기다림 → (줄들, 닫은 시각, 끝 코드, 끝난 시각)."""
    p = subprocess.Popen([sys.executable, __file__, '--child'] + extra, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    lines, t_close = [], [None]

    def read():
        for line in p.stdout:
            lines.append(line.rstrip())
            print('  | ' + line.rstrip(), flush=True)
            if line.startswith('child: 닫기'):
                t_close[0] = float(line.split()[-1])
    threading.Thread(target=read, daemon=True).start()
    t_end = time.time() + 90
    while t_close[0] is None and p.poll() is None and time.time() < t_end:
        time.sleep(0.05)
    try:
        rc = p.wait(timeout=10.0)
        t_exit = time.time()
    except subprocess.TimeoutExpired:
        rc, t_exit = None, None
        p.kill()                                    # (이 시험이 띄운 자식만)
        p.wait()
    time.sleep(0.3)
    return lines, t_close[0], rc, t_exit


def line_of(lines, head):
    return next((s for s in lines if s.startswith(head)), '')


def main_close():
    lst, rec = fake_server()
    lines, tc, rc, t_exit = run_child([])
    check('(c) 보정 창이 열린 채 닫음', any('보정 창 보임 True' in s for s in lines))
    check('(c) 닫은 뒤 10 s 안에 프로세스 끝 (코드 0)', tc is not None and rc == 0, '안 끝남 → 죽임' if rc is None else
          f'{t_exit - tc:.2f} s · 코드 {rc}')
    check('(c) 서버가 연결 닫힘을 봄', tc is not None and rec['eof'] is not None and rec['eof'] - tc < 5.0,
          None if rec['eof'] is None or tc is None else f'닫은 뒤 {rec["eof"] - tc:.2f} s')
    pings = [t for t, c in rec['msgs'] if c == 'ping' and tc is not None and t > tc + 0.5]
    check('(c) 닫은 뒤 ping 없음', not pings and any(c == 'ping' for _, c in rec['msgs']), f'닫기 전 ping 있음, 뒤 {len(pings)} 개')
    lst.close()


def main_refused():
    lst, attempts = fake_refuser()
    lines, tc, rc, t_exit = run_child(['--refuse'])
    t_ref = line_of(lines, 'child: 거절 ').split()
    t_retry = line_of(lines, 'child: 다시 연결 ').split()
    t_ref = float(t_ref[-1]) if t_ref else None
    t_retry = float(t_retry[3]) if len(t_retry) > 3 else None
    before = [t for t in attempts if t_retry is None or t < t_retry]
    state = line_of(lines, 'child: 상태 ')
    check('(거절) 알림을 받음', t_ref is not None, line_of(lines, 'child: 거절 '))
    check('(거절) 저절로 다시 붙지 않음 (다시 연결 전 연결 시도 1 번)', len(before) == 1 and t_retry is not None,
          f'{len(before)} 번 / {0 if t_ref is None or t_retry is None else t_retry - t_ref:.1f} s')
    check('(거절) 알림 한 번 · 칩 "연결 거절됨" · 단추 "↻ 다시 연결" · 연결 안 됨',
          '칩=연결 거절됨' in state and '단추=↻ 다시 연결' in state and '연결=False' in state and '알림=1' in state, state)
    check('(거절) [↻ 다시 연결] 은 서버를 새로 안 띄움', '서버띄움=False' in line_of(lines, 'child: 다시 연결 '))
    check('(거절) [↻ 다시 연결] → 한 번 더 붙어 봄 → 다시 알림', len(attempts) == 2 and '알림=2' in line_of(lines, 'child: 다시 거절'),
          f'연결 시도 {len(attempts)} 번 · {line_of(lines, "child: 다시 거절")}')
    check('(거절) 거절된 채 닫아도 10 s 안에 프로세스 끝 (코드 0)', tc is not None and rc == 0, '안 끝남 → 죽임' if rc is None else
          f'{t_exit - tc:.2f} s · 코드 {rc}')
    lst.close()


if __name__ == '__main__':
    if '--child' in sys.argv:
        child()
    else:
        main_refused() if REFUSE else main_close()
        raise SystemExit(U.summary())
