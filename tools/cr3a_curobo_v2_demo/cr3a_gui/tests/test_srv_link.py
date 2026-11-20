"""[2026-10-27 R-1 ②] rt_server.Link.send: GUI 가 멈춰 (보내기가 막혀) 보낼 줄이 차도, 여러 스레드가 같이 넣으면서 예외가 나지 않아야 함.
전 코드 (크기 정한 Queue 가 차면 '하나 꺼내고 put_nowait') 는 두 스레드가 같이 넣으면 꺼낸 자리를 다른 스레드가 채워 queue.Full 이 났음
(제어 스레드에서 나면 serve 가 Abort 만 받아 서버가 끝남).
"""
import sys  # noqa: E402
from pathlib import Path  # noqa: E402
sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parents[2] / 'cr3a_rt_shaping')]   # rt_server · 엔진
import threading
import time

import rt_server


class BlockingConn:
    """send 가 gate 가 열릴 때까지 막히는 가짜 연결 (멈춘 GUI)."""

    def __init__(self):
        self.gate = threading.Event()
        self.got = []

    def send(self, m):
        self.gate.wait()
        self.got.append(m)

    def close(self):
        pass


def test_send_never_raises_when_full_and_shared():
    """스레드를 자주 바꿔 (switchinterval 1 µs) 경쟁 창을 드러냄 — 같은 조건에서 전 코드는 20 만 번 넣기 중 queue.Full 5085 번 (PC 3.13)."""
    old_si = sys.getswitchinterval()
    sys.setswitchinterval(1e-6)
    try:
        _run_full_and_shared()
    finally:
        sys.setswitchinterval(old_si)


def _run_full_and_shared():
    link = rt_server.Link(0)                    # 포트 0 = 빈 포트 (연결은 안 받음 — 가짜 연결을 바로 붙임)
    conn = BlockingConn()
    link.conn = conn
    errors, n_threads, n_each = [], 4, 3000

    def produce(k):
        try:
            for i in range(n_each):
                link.send({'type': 'state', 'k': k, 'i': i})
                if i % 7 == 0:
                    link.log(f'줄 {k} {i}')
        except Exception as e:                  # noqa: BLE001 — 어떤 예외든 실패
            errors.append(repr(e))

    th = [threading.Thread(target=produce, args=(k,)) for k in range(n_threads)]
    t0 = time.monotonic()
    for t in th:
        t.start()
    for t in th:
        t.join(20.0)
    assert not errors, errors[:3]
    assert time.monotonic() - t0 < 10.0         # 보내기가 막혀도 넣는 쪽은 안 기다림
    assert len(link._out) <= rt_server.OUT_MAX  # 넘친 것은 오래된 것부터 버림
    link.send({'type': 'final'})
    conn.gate.set()                             # GUI 가 다시 움직임 → 남은 줄이 나감
    deadline = time.monotonic() + 5.0
    while (link._out or not conn.got or conn.got[-1]['type'] != 'final') and time.monotonic() < deadline:
        time.sleep(0.01)
    assert not link._out
    assert 0 < len(conn.got) <= rt_server.OUT_MAX + 1      # (+1 = gate 에서 막혀 있던 하나)
    assert conn.got[-1] == {'type': 'final'}                # 가장 새 것은 남고 차례대로


def test_log_kept_without_connection():
    """연결이 없을 때: 상태는 버리고 (예외 없이), 로그 줄은 남김 (다시 붙으면 hello 로 최근 로그) — 전과 같음."""
    link = rt_server.Link(0)
    link.send({'type': 'state'})
    link.log('한 줄')
    assert list(link.logs) == ['한 줄']


# ---- [R-1 S2] GUI 쪽 제안 (a) 살아 있음 = ping 만 · (b) 두 번째 클라이언트 거절 (진짜 연결로) ----
def _client(link):
    from multiprocessing.connection import Client
    return Client(link.listener.address, authkey=rt_server.AUTH)


def _wait(cond, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_alive_only_by_ping_and_second_client_refused():
    link = rt_server.Link(0)
    a = _client(link)
    assert a.recv()['type'] == 'hello'
    assert _wait(lambda: link.conn is not None)
    first = link.conn
    t0 = link.t_msg
    time.sleep(0.05)
    a.send({'cmd': 'jog_hold', 'joint': 0, 'v_deg': 1.0})       # (조그 스레드가 보내는 명령) → 살아 있음으로 안 셈
    assert _wait(lambda: not link.cmds.empty())
    assert link.t_msg == t0 and not link.pinging
    a.send({'cmd': 'ping'})                                     # 화면 스레드의 ping → 셈
    assert _wait(lambda: link.pinging and link.t_msg > t0)
    # (b) 지금 GUI 가 살아 있는 동안 두 번째 클라이언트 → 거절 (까닭을 받고 닫힘), 지금 GUI 는 그대로
    b = _client(link)
    m = b.recv()
    assert m['type'] == 'refused' and 'GUI' in m['why']
    try:
        b.recv()
        closed = False
    except (EOFError, OSError):
        closed = True
    assert closed and link.conn is first and link._refused == 1
    assert any('거절' in x for x in link.logs)                 # 운전자 GUI 로그에도 알림
    # 지금 GUI 가 멈춤 (ping 하던 GUI 가 GUI_LOST_S 넘게 조용함) → 새 클라이언트로 바꿈
    link.t_msg = time.monotonic() - rt_server.GUI_LOST_S - 0.5
    c = _client(link)
    assert c.recv()['type'] == 'hello'
    assert _wait(lambda: link.conn is not None and link.conn is not first)
    assert any('응답이 없어 새 GUI 연결로 바꿈' in x for x in link.logs)


def test_closed_gui_then_new_client_accepted():
    """지금 GUI 가 닫히면 (연결 끊김 → gui_lost) 새 클라이언트를 받음 (거절 아님)."""
    link = rt_server.Link(0)
    a = _client(link)
    assert a.recv()['type'] == 'hello'
    assert _wait(lambda: link.conn is not None)
    a.close()
    assert _wait(lambda: link.conn is None)
    assert link.cmds.get(timeout=1.0)['cmd'] == 'gui_lost'
    b = _client(link)
    assert b.recv()['type'] == 'hello'
    assert _wait(lambda: link.conn is not None)
