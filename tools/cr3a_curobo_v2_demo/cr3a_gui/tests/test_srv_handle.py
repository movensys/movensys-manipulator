"""[2026-10-27 R-1 ⑪] GuiController._handle (명령 → 처리 함수 표) 가 전 if/elif 와 같은 차례로 거르는지 — 상태 조합마다 어느 처리가
불리는지만 봄 (처리 함수는 가짜, 제어기는 __init__ 없이 — GPU·ROS 없음. 다만 gui_controller 가 rt_run → cuRobo 를 가져오므로 Thor 컨테이너).
전 차례: ① 조그 (Servo) 중 estop·stop·quit = Servo 멈춤 ② estop·stop·quit·servo·alarm ③ MOVES 는 R2 오류 → 서보 꺼짐 (이상적 아님) →
WMX 통신 끝 차례로 거절 ④ 나머지 명령 (모르는 명령·hello 는 무시).
"""
import collections
import sys  # noqa: E402
from pathlib import Path  # noqa: E402
sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parents[2] / 'cr3a_rt_shaping')]   # rt_server · 엔진

import pytest  # noqa: E402

gcm = pytest.importorskip('gui_controller')        # (cuRobo 가 없는 PC 에서는 건너뜀)
import rt_server as rs  # noqa: E402


class Fake(gcm.GuiController):
    """처리 함수를 불린 것만 적는 가짜 (상태는 시험이 넣음)."""

    def __init__(self, **st):
        self.calls = []
        self.sj, self.job, self.pending = None, None, collections.deque()
        self.r2_fault, self.servo_on, self.ideal, self.quit = None, True, False, False
        self.comm_deadline, self._hold_block = None, False
        for k, v in st.items():
            setattr(self, k, v)

    def _servo_halt(self, why, estop=False, soft=False):
        self.calls.append(('halt', why, estop, soft))

    def _say_once(self, text):
        self.calls.append(('say_once', text.split(' ')[0]))

    def _stop(self, why='정지'):
        self.calls.append(('stop',))

    def _set_servo(self, on):
        self.calls.append(('servo', on))

    def _clear_alarm(self):
        self.calls.append(('alarm',))

    def _gui_lost(self, why):
        self.calls.append(('gui_lost', why))

    def _servo_cmd(self, m):
        self.calls.append(('jog', m['cmd']))

    def _cmd_goto(self, m):
        self.calls.append(('goto', m['cmd']))

    def _cmd_realtime(self, m):
        self.calls.append(('realtime', m['on']))

    def _map_check(self, label, turning_on=False):
        self.calls.append(('map_check',))

    def _set_shaper(self, method, rows):
        self.calls.append(('shaper', method))

    def _set_speed(self, s):
        self.calls.append(('speed', s))


def jog():
    return rs.JogState(mode='hold', joint=True, ax=0, frame=None, name='J1', t0=0.0, t=0.0)


def run(f, m):
    try:
        f._handle(m)
        return None
    except rs.Abort as e:
        return e.kind


def test_jog_time_stop_commands_halt_servo():
    for k, why in (('estop', '비상정지'), ('stop', '정지'), ('quit', '끄기')):
        f = Fake(sj=jog(), job={'label': 'x'})
        assert run(f, {'cmd': k}) is None                       # (조그 중엔 Abort 없음)
        assert f.calls == [('halt', why, k == 'estop', False)]
        assert f.quit == (k == 'quit')


def test_stop_estop_quit_without_jog():
    assert run(Fake(), {'cmd': 'estop'}) == 'estop'
    f = Fake()
    assert run(f, {'cmd': 'stop'}) is None and f.calls == [('stop',)]
    assert run(Fake(job={'label': 'x'}), {'cmd': 'stop'}) == 'stop'
    f = Fake()
    assert run(f, {'cmd': 'quit'}) is None and f.quit and f.calls == []
    f = Fake(job={'label': 'x'})
    assert run(f, {'cmd': 'quit'}) == 'quit' and f.quit


def test_move_gates_order():
    for k in rs.MOVES:
        m = {'cmd': k}
        f = Fake(r2_fault='오류', servo_on=False, comm_deadline=0.0)       # 셋 다 → R2 오류가 먼저
        run(f, m)
        assert f.calls == [('say_once', 'R2')], (k, f.calls)
        f = Fake(servo_on=False, comm_deadline=0.0)                       # 서보 꺼짐이 통신보다 먼저
        run(f, m)
        assert f.calls == [('say_once', '서보가')], (k, f.calls)
        f = Fake(comm_deadline=0.0)                                       # (monotonic 0 = 이미 지남 → 통신 끝)
        run(f, m)
        assert f.calls == [('say_once', 'WMX')], (k, f.calls)
        f = Fake(ideal=True, servo_on=False)                              # 이상적 모드는 서보 없음 → 거르지 않음
        run(f, m)
        assert f.calls and f.calls[0][0] in ('jog', 'goto'), (k, f.calls)


def test_non_moves_not_gated():
    f = Fake(r2_fault='오류', servo_on=False, comm_deadline=0.0)
    for m, want in (({'cmd': 'servo', 'on': True}, ('servo', True)), ({'cmd': 'alarm'}, ('alarm',)),
                    ({'cmd': 'gui_lost', 'why': 'x'}, ('gui_lost', 'x')), ({'cmd': 'speed', 'scale': 0.5}, ('speed', 0.5)),
                    ({'cmd': 'shaper', 'method': 'ZV', 'rows': []}, ('shaper', 'ZV')), ({'cmd': 'realtime', 'on': True}, ('realtime', True))):
        f.calls.clear()
        run(f, m)
        assert f.calls == [want], (m, f.calls)


def test_jog_release_and_unknown():
    f = Fake(sj=jog(), r2_fault='오류')
    f._hold_block = True
    run(f, {'cmd': 'jog_release'})
    assert f.calls == [('halt', '조그 뗌', False, True)] and not f._hold_block
    for k in ('hello', 'nonsense', None):
        f = Fake()
        assert run(f, {'cmd': k}) is None and f.calls == []
