"""[2026-10-27 R-1 ②] [카메라 끄기] 시험 (b) — 가짜 카메라 노드 (잠만 자는 파이썬, 명령 줄에 /cam_fixed) 로. 실제 카메라 노드는 안 건드림:
CR3A_TMP_DIR 이 시험 폴더라 PID 파일이 따로 (실제 노드의 /tmp/cr3a_cam_fixed.pid 는 안 읽고 안 씀). ROS·서버 없음.
  1) 이 GUI 가 띄운 노드 (CameraProcess.start 와 같은 Popen — 다른 세션): stop() 이 바로 끝나고 자식을 거둠 (좀비 없음), PID 파일 지움
  2) 앞 GUI 가 띄운 노드 (자식이 아님): stop() 이 바로 끝남 — /proc 상태로 (a) 없어짐 (b) 좀비 (부모가 안 거둠 — 전엔 /proc/<pid> 가 남아
     5 s 를 다 기다렸던 경우)
  3) GUI [■ 끄기] (on_cam_off): 바로 돌아오고 (QThreadPool), SIGINT 를 무시하는 노드 (5 s 뒤 SIGTERM) 동안에도 GUI 스레드가 안 멈추고
     (20 ms 타이머 간격 최대), 결과가 로그 창에, 자식을 거둠
  CR3A_GUI_PORT=47873 CR3A_TMP_DIR=<폴더> python3 tests/test_cam_stop.py   (tests/run_r1_tests.sh)
"""
import os
import subprocess
import sys
import time

import seq_util as U
from seq_util import check

U.guard_test_port()
import camera_io as ci  # noqa: E402
from common import TMP_DIR  # noqa: E402

TMP_DIR.mkdir(parents=True, exist_ok=True)
SLEEPER = 'import time\nwhile True: time.sleep(1)'
# [R-1 Thor 시험] 가짜 노드도 진짜 노드처럼 SIGINT 를 받게 처리기를 둠: bash 가 '&' 로 띄운 것 (2a) 은 SIGINT 무시 (SIG_IGN) 를 물려받고
#  파이썬은 물려받은 무시를 그대로 둬서, 5 s 뒤 SIGTERM 으로만 끝났음 (첫 Thor 시험 2a 5084 ms — 시험 탓, 끄기 코드는 맞게 SIGTERM)
DUMMY = [sys.executable, '-c', 'import signal\nsignal.signal(signal.SIGINT, signal.default_int_handler)\n' + SLEEPER, f'{ci.CAM_NS}/dummy']
DEAF = [sys.executable, '-c', 'import signal\nsignal.signal(signal.SIGINT, signal.SIG_IGN)\n' + SLEEPER, f'{ci.CAM_NS}/dummy_deaf']


def ppid(pid):
    try:
        s = open(f'/proc/{pid}/stat').read()
        return int(s[s.rfind(')') + 2:].split()[1])
    except OSError:
        return None


def timed_stop():
    t0 = time.monotonic()
    msg = ci.CameraProcess.stop()
    return time.monotonic() - t0, msg


check('PID 파일 = 시험 폴더 (실제 카메라 노드 것과 다름)', ci.CAM_PIDF.parent == TMP_DIR, str(ci.CAM_PIDF))
# 1) 이 프로세스의 자식
p = ci.CameraProcess._spawn(DUMMY)
time.sleep(0.5)
check('1 노드 도는 중 (PID 파일 + /proc 명령 줄)', ci.CameraProcess.pid() == p.pid, p.pid)
dt, msg = timed_stop()
check('1 stop() 바로 끝남', dt < 1.0, f'{dt * 1000:.0f} ms · {msg}')
check('1 자식을 거둠 (좀비 없음)', ci._proc_state(p.pid) is None, ci._proc_state(p.pid))
check('1 PID 파일 지움', not ci.CAM_PIDF.exists())
check('1 이미 꺼진 뒤 끄기', ci.CameraProcess.stop() == '이미 꺼져 있음')

# 2a) 자식이 아닌 노드 (bash 가 띄우고 끝남 → 고아)
out = subprocess.run(['bash', '-c', '"$@" >/dev/null 2>&1 </dev/null & echo $!', 'bash'] + DUMMY, capture_output=True, text=True)
pid = int(out.stdout.strip())
time.sleep(0.5)
ci.CAM_PIDF.write_text(str(pid))
check('2a 자식이 아님', ppid(pid) not in (None, os.getpid()), f'부모 {ppid(pid)}')
dt, msg = timed_stop()
check('2a stop() 바로 끝남', dt < 1.5, f'{dt * 1000:.0f} ms · {msg}')
check('2a 끝남 (/proc 없음 또는 Z)', ci._proc_state(pid) in (None, 'Z'), ci._proc_state(pid))

# 2b) 자식이 아닌 노드가 끝난 뒤 좀비로 남음 (부모가 거두지 않음) — 전엔 /proc/<pid> 가 남아 5 s 를 다 기다림
holder = subprocess.Popen([sys.executable, '-c', 'import subprocess, sys, time\n'
                           'p = subprocess.Popen(sys.argv[1:], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)\n'
                           'print(p.pid, flush=True)\ntime.sleep(60)'] + DUMMY, stdout=subprocess.PIPE, text=True)
pid = int(holder.stdout.readline())
time.sleep(0.5)
ci.CAM_PIDF.write_text(str(pid))
check('2b 자식이 아님 (손자)', ppid(pid) == holder.pid, f'부모 {ppid(pid)}')
dt, msg = timed_stop()
st = ci._proc_state(pid)
check('2b 좀비가 돼도 stop() 바로 끝남', dt < 1.5 and st == 'Z', f'{dt * 1000:.0f} ms · 상태 {st} · {msg}')
holder.kill()
holder.wait()

# 3) GUI [■ 끄기]
app = U.make_app()
from PyQt5 import QtCore  # noqa: E402
import camera_panel as cp  # noqa: E402
import cr3a_gui as G  # noqa: E402

win = G.MainWindow()
G.keep_running_on_errors()
win.show()
gaps, last = [], [time.monotonic()]


def beat():
    now = time.monotonic()
    gaps.append(now - last[0])
    last[0] = now


tm = QtCore.QTimer(interval=20, timeout=beat)


def seq():
    p = ci.CameraProcess._spawn(DEAF)
    yield ('sleep', 0.8)
    check('3 노드 도는 중', ci.CameraProcess.pid() == p.pid, p.pid)
    tm.start()
    gaps.clear()
    last[0] = t0 = time.monotonic()
    win.on_cam_off()
    dt = time.monotonic() - t0
    check('3 on_cam_off 가 바로 돌아옴 (QThreadPool)', dt < 0.05, f'{dt * 1000:.1f} ms')
    check('3 끄는 동안 단추 막음', not win.btn_cam_off.isEnabled() and win._cam_busy)
    yield ('until', lambda: not win._cam_busy, 15, '카메라 끄기 끝 (SIGINT 무시 → 5 s 뒤 SIGTERM)')
    t_done = time.monotonic() - t0
    tm.stop()
    check('3 SIGINT 무시 → SIGTERM 으로 끔', 4.5 < t_done < 8.0 and 'SIGTERM' in win.log_view.toPlainText(), f'{t_done:.2f} s')
    check('3 끄는 동안 GUI 스레드 안 멈춤 (20 ms 타이머 간격 최대)', gaps and max(gaps) < 0.25,
          f'{max(gaps) * 1000:.0f} ms / {len(gaps)} 번' if gaps else '없음')
    check('3 자식을 거둠', ci._proc_state(p.pid) is None, ci._proc_state(p.pid))
    check('3 PID 파일 지움 · 단추 다시 켬', not ci.CAM_PIDF.exists() and win.btn_cam_off.isEnabled())
    QtCore.QTimer.singleShot(0, win.close)          # (연결 안 됨 → 묻지 않고 끝)
    yield ('sleep', 5)


U.run(seq(), app)
cp.finish_jobs()
raise SystemExit(U.summary())
