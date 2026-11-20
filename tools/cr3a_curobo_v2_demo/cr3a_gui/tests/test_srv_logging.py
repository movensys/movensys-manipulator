"""[2026-10-27 R-1 ⑩] rt_server.setup_logging: 줄 모양 '[%H:%M:%S] …' 그대로, GUI (link.log) 로 같은 줄, traceback 은 출력에만,
출력 쓰기가 느려도 (멈춘 파일·파이프) 부르는 스레드는 안 기다림, 줄을 버리지 않음.
"""
import sys  # noqa: E402
from pathlib import Path  # noqa: E402
sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parents[2] / 'cr3a_rt_shaping')]   # rt_server · 엔진
import io
import re
import sys
import threading
import time

import rt_server


class FakeLink:
    def __init__(self):
        self.lines = []

    def log(self, line):
        self.lines.append(line)


class SlowOut(io.StringIO):
    """write 마다 gate 가 열릴 때까지 기다리는 출력 (막힌 stdout)."""

    def __init__(self):
        super().__init__()
        self.gate = threading.Event()

    def write(self, s):
        self.gate.wait()
        return super().write(s)


def run_with_stdout(out, fn):
    old = sys.stdout
    sys.stdout = out
    try:
        return fn()
    finally:
        sys.stdout = old


def test_format_gui_and_traceback():
    out, link = io.StringIO(), FakeLink()

    def body():
        log, listener = rt_server.setup_logging(link)
        log('서버', '시작', 3)
        try:
            raise ValueError('일부러')
        except ValueError:
            rt_server.LOGGER.error('  (traceback)', exc_info=True)
        listener.stop()
    run_with_stdout(out, body)
    text = out.getvalue()
    assert re.match(r'^\[\d\d:\d\d:\d\d\] 서버 시작 3$', text.splitlines()[0])
    assert 'Traceback (most recent call last)' in text and "ValueError: 일부러" in text
    assert link.lines == ['서버 시작 3']                 # GUI 에는 짧은 줄만


def test_slow_output_does_not_block_caller_and_keeps_all_lines():
    out, link = SlowOut(), FakeLink()
    res = {}

    def body():
        log, listener = rt_server.setup_logging(link)
        t0 = time.monotonic()
        for i in range(500):
            log(f'줄 {i}')
        res['dt'] = time.monotonic() - t0
        out.gate.set()                                  # 출력이 다시 됨
        listener.stop()                                 # 남은 줄을 다 씀
    run_with_stdout(out, body)
    assert res['dt'] < 0.5, res['dt']                    # 출력이 막혀 있는 동안에도 log 는 바로 돌아옴
    lines = out.getvalue().splitlines()
    assert len(lines) == 500 and lines[-1].endswith('줄 499')
    assert len(link.lines) == 500
