"""[2026-10-27 R-1] 통합 시험용 GUI 대신 클라이언트 + 서버 띄우기 (sigterm_check·rec_trim_check). 실물 포트 47861 은 안 씀."""
import os
import subprocess
import sys
import threading
import time
from multiprocessing.connection import Client
from pathlib import Path

import numpy as np

GUI = Path(__file__).resolve().parents[1]
AUTH = b'cr3a-gui'


def start_server(port, out, *extra, log_path):
    """rt_server.py 를 자식으로 (출력은 log_path). 반환 Popen. 포트에 이미 누가 있으면 안 띄움 (남은 서버에 붙어 엉뚱한 결과가 나지 않게 —
    [2026-10-27 Thor 시험] 켜는 중 SIGTERM 시험의 남은 서버에 다음 시험이 붙었었음)."""
    import socket
    assert port != 47861, '실물 GUI 포트'
    with socket.socket() as s:
        if s.connect_ex(('127.0.0.1', port)) == 0:
            raise RuntimeError(f'포트 {port} 에 이미 서버가 있음 — 남은 서버를 먼저 끄기')
    cmd = [sys.executable, '-W', 'ignore', str(GUI / 'rt_server.py'), '--port', str(port), '--out', str(out), *extra]
    f = open(log_path, 'w')
    return subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=str(GUI), env=dict(os.environ))


class Gui:
    def __init__(self, port, timeout=300.0):
        end = time.monotonic() + timeout
        while True:
            try:
                self.c = Client(('127.0.0.1', port), authkey=AUTH)
                break
            except OSError:
                if time.monotonic() > end:
                    raise
                time.sleep(0.5)
        self.st, self.hello, self.logs, self.alive = {}, {}, [], True
        self.lock = threading.Lock()
        threading.Thread(target=self._rx, daemon=True).start()
        threading.Thread(target=self._pinger, daemon=True).start()

    def _rx(self):
        while self.alive:
            try:
                m = self.c.recv()
            except (EOFError, OSError):
                self.alive = False
                return
            t = m.get('type')
            if t == 'hello':
                self.hello.update(m)
                self.logs += m.get('logs', [])
            elif t == 'state':
                self.st = m
            elif t == 'log':
                self.logs.append(m['line'])
                print('   [서버]', m['line'][:220], flush=True)

    def _pinger(self):
        while self.alive:
            try:
                self.send({'cmd': 'ping'})
            except (OSError, ValueError):
                return
            time.sleep(0.5)

    def send(self, m):
        with self.lock:
            self.c.send(m)

    @staticmethod
    def wait(cond, timeout, step=0.05):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cond():
                return True
            time.sleep(step)
        return False

    def seen(self, text):
        return [x for x in self.logs if text in x]

    def ready(self, timeout=300.0):
        return self.wait(lambda: self.hello.get('ready') or self.seen('준비 끝 — 명령을 기다림'), timeout) and \
            self.wait(lambda: 'q' in self.st, 20.0)

    def u(self):
        return np.asarray(self.st['u'], float)
