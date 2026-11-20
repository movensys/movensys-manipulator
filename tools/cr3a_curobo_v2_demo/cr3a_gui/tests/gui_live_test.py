"""GUI ↔ 서버 시험 (Thor 컨테이너, 화면 없이 QT_QPA_PLATFORM=offscreen). 버튼을 차례로 누르고 장면마다 캡처.
  CR3A_GUI_PORT=47873 python3 tests/gui_live_test.py <캡처 폴더> [대상 0 = 이상적 | 1 = WMX 시뮬레이터] [지도 1 = 실제 카메라] [순서]
[2026-10-26 2 차] WMX 시뮬레이터 대상: [▶ 시스템 시작] 이 R2 를 켜고 (안전장치) 서버를 띄움 → 같은 조작 + 이동 중 비상정지 · 서보 끄기/켜기
 (꺼진 동안 이동 거절) · 알람 해제 · 성형기 다시 바꾸기 → [■ 시스템 종료] (기록 저장 → 서보 OFF → R2 끔). 실제 로봇은 안 움직임.
[2026-10-27 R-1] test_shots/ → tests/ (경로는 이 파일 자리에서, 순서 돌리기는 seq_util). 실물 서버 포트 (47861) 로는 안 돎
 (CR3A_GUI_PORT 필요 — GUI 가 띄우는 서버도 같은 포트), 실물 시스템 (47861 서버) 이 켜져 있으면 안 돎 (R2·카메라가 섞이지 않게),
 붙은 서버가 실제 로봇이면 아무것도 안 보내고 끝. 임시 폴더 (CR3A_TMP_DIR — 서버 로그·GUI 로그) 도 /tmp 가 아니어야 함, 지도 1 (실제
 카메라) 만 /tmp (실제 카메라 노드 PID 를 봐야 함 — 아니면 [▶ 시스템 시작] 이 노드를 하나 더 띄움)."""
import sys
import time
from pathlib import Path

import numpy as np

import seq_util as U
from seq_util import GUI, log

U.guard_test_port(need_tmp=not (len(sys.argv) > 3 and sys.argv[3] == '1'))
U.refuse_if_live_server()
app = U.make_app()
import cr3a_gui as G  # noqa: E402
from common import TMP_DIR  # noqa: E402

OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else TMP_DIR / 'cr3a_gui_test'
TARGET = int(sys.argv[2]) if len(sys.argv) > 2 else 0
MAPCAM = int(sys.argv[3]) if len(sys.argv) > 3 else 0      # 1 = 지도를 실제 고정 카메라로 (짧은 순서: B → 지키기 → A)
MODE = sys.argv[4] if len(sys.argv) > 4 else ''           # 'speed' = 속도 배율 순서 (seq_speed), 'speed2' = 성형 없음·있음 × 배율 1·0.5,
#                                                           'pvat' = 정지·비상정지·일반 이동 (seq_pvat, PVAT 단계 1 보완)
RT_OFF = len(sys.argv) > 5 and sys.argv[5] == 'rtoff'    # (speed2) 실시간 회피 끔으로
AVOID_S = float(sys.argv[5]) if MODE == 'avoid' and len(sys.argv) > 5 else 90.0     # (avoid) 물체를 움직이며 반복하는 시간 s
OUT.mkdir(parents=True, exist_ok=True)
win = G.MainWindow()
G.keep_running_on_errors()
win.resize(1600, 1000)
win.show()


# 시뮬레이션 시험 자세 (rt_config TaskCfg qa_tool·qb_tool = 서버 hello qA·qB). [2026-10-27] GUI 자세 목록에서 A·B 를 뺐으므로 여기서 칸을 채움
A_DEG = (56.31, -40.67, -12.99, -29.16, 83.72, 55.72)
B_DEG = (-28.78, -42.61, -10.82, -27.41, 96.28, -26.31)


def fill(q):
    for sp, v in zip(win.sp_target, q):
        sp.setValue(v)


def st():
    return win.state or {}


def q_deg():
    s = st()
    return None if 'q' not in s else np.degrees(s['q'])


def near(target, tol=0.2):
    q = q_deg()
    return q is not None and float(np.max(np.abs(q - np.asarray(target, float)))) < tol


def still():
    return st().get('job') is None and not st().get('jog')


def shot(name):
    win.scene._sig = None
    win.scene.refresh()
    app.processEvents()
    win.grab().save(str(OUT / f'{name}.png'))
    s = st()
    log('shot', name, '| job', s.get('job'), '| mode', s.get('mode'), '| q', None if q_deg() is None else np.round(q_deg(), 2).tolist(),
        '| rt', s.get('realtime'), '| estop', s.get('estop'), '| servo', s.get('servo'), '| fault', s.get('fault'),
        '| cyc', s.get('cyc_ms'), '| lead', s.get('lead_ms'))


def mark(text):
    """서버 로그·기록에도 남게 (시험 단계 표시)."""
    log('==', text)


def seq_realcam():
    """실제 카메라 지도 (짧은 순서): 지도 화면 → 실제 팔 자세 → 지키기 5 s → 영점 → 끝.
    [2026-10-26] 1 차는 시뮬레이션용 A·B 로 갔는데 B 가 책상 위 실제 물체와 겹쳐 (MPC 지도 −68 mm) 실패 → 실제 장면에서 비어 있는 자세:
    실제 팔이 지금 있는 자리 (마지막 엔코더 읽기 — 지도에서 지워지는 자리) 와 영점 (팔을 세움)."""
    import json
    enc = json.loads((GUI / 'last_encoders.json').read_text())['q_deg']
    win.view_tabs.setCurrentIndex(0)
    shot('r01_ready_realmap')
    log('지도 칸', win.n_vox, '| 카메라', st().get('camera'), st().get('cam_fps'))
    # [2026-10-26 3 차] 받침 둘레 케이블 자리를 뺀 뒤: B (시뮬레이션용, 미리 계산 +114 mm) ↔ 영점 (+153 mm). 실제 팔 자세는 공구 옆 작은
    #  덩어리 (실제 공구에 달린 것, +49 mm) 가 경계 채우기 거리 안이라 뺌
    for name, q in (('B', list(B_DEG)), ('zero', [0.0] * 6), ('B_again', list(B_DEG))):
        mark(f'실제 카메라 지도: → {name} {[round(v, 2) for v in q]}')
        for sp, v in zip(win.sp_target, q):
            sp.setValue(v)
        win.on_goto()
        yield ('sleep', 1.5)
        shot(f'r02_moving_{name}')
        yield ('until', lambda q=q: still() and near(q), 60, f'{name} 도착')
        yield ('sleep', 5.0)
        shot(f'r03_hold_{name}')
        log('지도 칸', win.n_vox, '| 가장 가까운', st().get('clr_mm'), 'mm | 안전 검사', st().get('guard'), '| q', np.round(q_deg(), 2).tolist())
    mark('끝: 시스템 종료')
    win.on_toggle_stack()
    yield ('until', lambda: not win.connected, 90, '서버 끝')
    log('끝')


def seq_speed():
    """[2026-10-26] 속도 배율 (GUI 처음 값 0.2): B → 영점 → (움직이는 중 1.0 으로 바꾸기 = 거절, 슬라이더가 0.2 로 돌아와야) B →
    서서 1.0 으로 → 영점 → B → 0.5 로 → 영점. 다리 시간·WMX 속도/가속 ÷ 한계·끝 지연은 기록으로 (speed_check.py)."""
    log('속도 배율 GUI', win.sl_speed.value() / 100, '| 서버', st().get('speed'))
    B, Z = list(B_DEG), [0.0] * 6

    def go(name, q):
        mark(f'속도 {st().get("speed")}: → {name}')
        for sp, v in zip(win.sp_target, q):
            sp.setValue(v)
        t0 = time.time()
        win.on_goto()
        yield ('sleep', 0.5)
        yield ('until', lambda q=q: still() and near(q), 120, f'{name} 도착')
        log(f'{name} 도착 (GUI 기준 {time.time() - t0:.1f} s)')
        yield ('sleep', 1.0)
    yield from go('B', B)
    yield from go('zero', Z)
    mark('움직이는 중 속도 배율 1.0 으로 → 거절돼야')
    for sp, v in zip(win.sp_target, B):
        sp.setValue(v)
    win.on_goto()
    yield ('sleep', 0.8)
    win.sl_speed.setValue(100)
    yield ('sleep', 1.2)
    log('움직이는 중 바꾸기 뒤: 슬라이더', win.sl_speed.value() / 100, '| 서버', st().get('speed'), '| job', st().get('job'))
    yield ('until', lambda: still() and near(B), 120, 'B 도착')
    yield ('sleep', 1.0)
    win.sl_speed.setValue(100)
    yield ('sleep', 1.5)
    log('서서 바꾸기 뒤: 슬라이더', win.sl_speed.value() / 100, '| 서버', st().get('speed'))
    yield from go('zero', Z)
    yield from go('B', B)
    win.sl_speed.setValue(50)
    yield ('sleep', 1.5)
    log('서서 0.5 로: 서버', st().get('speed'))
    yield from go('zero', Z)
    shot('s9_end')
    mark('끝: 시스템 종료')
    win.on_toggle_stack()
    yield ('until', lambda: not win.connected, 90, '서버 끝')
    log('끝')


def seq_speed2():
    """[2026-10-26 사용자 "unshaped 경로로 했을 때 속도 배율이 1 -> 0.5 로 되었을 때 시간이 2배로 늘어나는지, 성형기 적용했을 때 … 입력성형기
    지연 시간을 제외한 시간이 2배가 되는지"] 성형 없음 → ETM4 8 Hz 마다 배율 1.0 · 0.5 로 B → 영점 → B. 시간은 기록으로 (speed_shape_check.py)."""
    B, Z = list(B_DEG), [0.0] * 6

    def go(name, q):
        mark(f'성형 {win.shaping.applied[0] if win.shaping.applied else "?"} · 배율 {st().get("speed")}: → {name}')
        for sp, v in zip(win.sp_target, q):
            sp.setValue(v)
        win.on_goto()
        yield ('sleep', 0.5)
        yield ('until', lambda q=q: still() and near(q), 120, f'{name} 도착')
        yield ('sleep', 1.0)

    def shaper(method, rows):
        win.shaping.set_config(method, rows, apply=False)
        win.shaping.on_apply()
        yield ('until', lambda: win.shaping.applied == win.shaping.config(), 10, f'성형기 {method} 적용')

    def speed(v):
        win.sl_speed.setValue(v)
        yield ('until', lambda: abs((st().get('speed') or 0) - v / 100) < 1e-6, 5, f'배율 {v / 100}')
    if RT_OFF:                                  # 실물 첫 시험과 같게: 실시간 회피 끔 → 경로를 시각 그대로 따라감 (rt_server._command_carrot)
        win.chk_rt.setChecked(False)
        yield ('until', lambda: st().get('realtime') is False, 5, '실시간 회피 끔')
    yield from speed(100)
    yield from shaper('Unshaped', [(True, 8.0), (False, 15.0), (False, 23.0)])
    yield from go('B', B)
    for method, rows in (('Unshaped', [(True, 8.0), (False, 15.0), (False, 23.0)]), ('ETM4', [(True, 8.0), (False, 15.0), (False, 23.0)])):
        yield from shaper(method, rows)
        for v in (100, 50):
            yield from speed(v)
            yield from go('zero', Z)
            yield from go('B', B)
    shot('t9_end')
    mark('끝: 시스템 종료')
    win.on_toggle_stack()
    yield ('until', lambda: not win.connected, 90, '서버 끝')
    log('끝')


def seq_avoid():
    """[2026-10-26 사용자 "장애물이 날아다니는 환경에서 실제로 잘 피하고, 입력성형도 잘 먹히는지 … 전체 원본 경로에 입력성형을 전체를 먹이면
    전체적으로 shaped 된 경로가 나오는지 … 1로 테스트"] 실시간 회피 켬 · 배율 1.0 · ETM4 8 Hz · 시험 물체 (moving3t) 를 움직이며 B ↔ A [반복]
    AVOID_S 초 → [정지] → 물체 치움 → 끝. 회피·전체 성형 확인은 기록으로 (tools/wmx_check.py · cr3a_rt_shaping/tools/rt_report.py · avoid_check.py)."""
    B, A = list(B_DEG), list(A_DEG)
    win.sl_speed.setValue(100)
    yield ('until', lambda: abs((st().get('speed') or 0) - 1.0) < 1e-6, 5, '배율 1.0')
    win.shaping.set_config('ETM4', [(True, 8.0), (False, 15.0), (False, 23.0)], apply=False)
    win.shaping.on_apply()
    yield ('until', lambda: win.shaping.applied == win.shaping.config(), 10, '성형기 ETM4 8 Hz')
    if not st().get('realtime'):
        win.chk_rt.setChecked(True)
        yield ('until', lambda: st().get('realtime') is True, 5, '실시간 회피 켬')
    mark('회피 시험: → B (물체 멈춤)')
    for sp, v in zip(win.sp_target, B):
        sp.setValue(v)
    win.on_goto()
    yield ('sleep', 0.5)
    yield ('until', lambda: still() and near(B), 60, 'B 도착')
    mark('시험 물체 움직이기 시작 + B ↔ A [반복]')
    win._cmd({'cmd': 'objects', 'go': True})
    for sp, v in zip(win.sp_target, A):
        sp.setValue(v)
    win.on_repeat()
    t0 = time.time()
    for k in range(int(AVOID_S // 15)):
        yield ('sleep', 15.0)
        shot(f'a{k:02d}_avoid_{int(time.time() - t0)}s')
    mark('[정지]')
    win.on_stop()
    yield ('until', lambda: still(), 30, '정지')
    win._cmd({'cmd': 'objects', 'go': False})
    yield ('sleep', 1.0)
    shot('a99_end')
    mark('끝: 시스템 종료')
    win.on_toggle_stack()
    yield ('until', lambda: not win.connected, 90, '서버 끝')
    log('끝')


def seq_pvat():
    """[2026-10-30 PVAT 단계 1 보완 — 진행 문서 12장] 지금 방식에서 정지·비상정지·일반 이동의 새 명령이 실제로 언제 엔진에 닿는지 (병합 시각).
    배율 1.0 · ETM4 8 Hz 를 처음에 한 번만 (기대 경로 = u_exec.npz 하나). 실시간 끔: → B → A, [반복] 7 s → [정지], → B,
    A 로 가다가 1.2 s 뒤 [정지], → B, A 로 가다가 0.8 s 뒤 비상정지. 실시간 켬 (물체 없음): → B, A 로 가다가 0.8 s 뒤 비상정지.
    배율 0.2: → B. 끝 [■ 시스템 종료]. 기록·분석: pvat_model/tools/pvat_gui_run.sh."""
    A, B = list(A_DEG), list(B_DEG)

    def go(name, q):
        mark(f'→ {name}')
        fill(q)
        win.on_goto()
        yield ('sleep', 0.5)
        yield ('until', lambda q=q: still() and near(q), 120, f'{name} 도착')
        yield ('sleep', 1.0)

    def go_then(name, q, after_s, action, label):
        mark(f'→ {name}, 움직이기 시작 {after_s} s 뒤 {label}')
        fill(q)
        win.on_goto()
        yield ('until', lambda: st().get('job') is not None and st().get('mode') == 'move', 20, f'{name} 이동 시작')
        yield ('sleep', after_s)
        mark(label)
        action()
        yield ('until', lambda: still(), 30, f'{label} 뒤 섬')
        yield ('sleep', 1.0)

    def realtime(on):
        win.chk_rt.setChecked(on)
        yield ('until', lambda: st().get('realtime') is on, 5, f'실시간 회피 {"켬" if on else "끔"}')

    win.sl_speed.setValue(100)
    yield ('until', lambda: abs((st().get('speed') or 0) - 1.0) < 1e-6, 5, '배율 1.0')
    win.shaping.set_config('ETM4', [(True, 8.0), (False, 15.0), (False, 23.0)], apply=False)
    win.shaping.on_apply()
    yield ('until', lambda: win.shaping.applied == win.shaping.config(), 10, '성형기 ETM4 8 Hz')
    yield from realtime(False)
    yield from go('B', B)
    yield from go('A', A)
    mark('[반복] B ↔ A 7 s')
    fill(B)
    win.on_repeat()
    yield ('sleep', 7.0)
    mark('[정지] (반복 중)')
    win.on_stop()
    yield ('until', lambda: still(), 30, '반복 정지 뒤 섬')
    yield ('sleep', 1.0)
    yield from go('B', B)
    yield from go_then('A', A, 1.2, win.on_stop, '[정지] (이동 중)')
    yield from go('B', B)
    yield from go_then('A', A, 0.8, win.on_estop, '비상정지 (실시간 끔)')
    yield from realtime(True)                   # 켜면 비상정지 풀림 (_set_realtime → _resume)
    yield from go('B', B)
    yield from go_then('A', A, 0.8, win.on_estop, '비상정지 (실시간 켬)')
    yield from realtime(True)                   # 비상정지가 실시간을 끔 → 다시 켜서 풀고 끔
    yield from realtime(False)
    win.sl_speed.setValue(20)
    yield ('until', lambda: abs((st().get('speed') or 0) - 0.2) < 1e-6, 5, '배율 0.2')
    yield from go('B', B)
    shot('p9_end')
    mark('끝: 시스템 종료')
    win.on_toggle_stack()
    yield ('until', lambda: not win.connected, 90, '서버 끝')
    log('끝')


def seq():
    yield ('sleep', 1.5)                        # 이미 떠 있는 서버가 있으면 붙음
    U.refuse_real(win)                          # (실제 로봇 서버면 여기서 끝 — 아무 명령도 안 보냄)
    if not win.connected:
        win.cb_target.setCurrentIndex(TARGET)
        win.cb_mapcam.setCurrentIndex(MAPCAM)
        mark(f'시스템 시작 (대상 {win.cb_target.currentText()}, 지도 {win.cb_mapcam.currentText()})')
        win.on_toggle_stack()                   # [▶ 시스템 시작]
        shot('00_starting')
    yield ('until', lambda: win.connected and st().get('ready'), 420, '서버 준비')
    U.refuse_real(win)
    win.scope.btn_run.setChecked(True)          # [R-1] [▶ 그래프 켜기] 를 눌러야 쌓음 (전엔 단추가 안 이어져 늘 쌓음) — 15_graph 캡처용
    shot('01_ready')
    if MODE == 'speed':
        yield from seq_speed()
        return
    if MODE == 'speed2':
        yield from seq_speed2()
        return
    if MODE == 'avoid':
        yield from seq_avoid()
        return
    if MODE == 'pvat':
        yield from seq_pvat()
        return
    if MAPCAM == 1:
        yield from seq_realcam()
        return
    # 1) 목표 이동 → B
    mark('1 목표 이동 → B')
    fill(B_DEG)
    win.on_goto()
    yield ('sleep', 1.2)
    shot('02_moving_to_B')
    yield ('until', lambda: still() and near(B_DEG), 60, 'B 도착')
    yield ('sleep', 0.8)
    shot('03_at_B')
    # 2) 입력성형 바꾸기 (서 있을 때)
    mark('2 입력성형 ETM4 8+15')
    win.shaping.set_config('ETM4', [(True, 8.0), (True, 15.0), (False, 23.0)], apply=False)
    win.shaping.on_apply()
    yield ('until', lambda: win.shaping.applied == win.shaping.config(), 10, '성형기 적용')
    shot('04_shaper_applied')
    # 3) 관절 조그 한 칸 (J1 +5°)
    mark('3 조그 J1 +5')
    q1 = q_deg().copy()
    win.cb_jstep.setCurrentText('5')
    win._jog_click('joint', 0, +1)
    q1[0] += 5.0
    yield ('until', lambda: still() and near(q1), 40, '조그 도착')
    shot('05_jog_J1_plus5')
    # 4) 누르는 동안 조그 (J2 −10°/s, 1.5 s)
    mark('4 누르는 동안 조그 J2 -10/s')
    win.rb_hold.setChecked(True)
    win.cb_jstep.setCurrentText('10')
    q2 = q_deg().copy()
    win._jog_press('joint', 1, -1)
    yield ('sleep', 1.5)
    win._jog_release()
    yield ('sleep', 1.5)
    log('누르는 동안 조그 J2 변화', round(float(q_deg()[1] - q2[1]), 2), '°')
    shot('06_hold_jog_J2')
    win.rb_step.setChecked(True)
    # 5) 움직이는 중 [정지] (성형 거친 정지): A 로 가다가 1.5 s 뒤 → 반복 4 s 뒤
    mark('5a A 로 가다가 1.5 s 뒤 정지')
    fill(A_DEG)
    win.on_goto()
    yield ('until', lambda: st().get('job') is not None and st().get('mode') == 'move', 20, '이동 시작 (정지 시험)')
    yield ('sleep', 1.2)
    win.on_stop()                               # 캡처 (~0.9 s) 보다 먼저 — 2 차 시험에서는 캡처 뒤라 이미 도착해 있었음
    shot('07_stop_pressed_while_moving')
    yield ('sleep', 2.0)
    shot('08_after_stop')
    mark('5b 반복 → 4 s 뒤 정지')
    fill(B_DEG)
    win.on_repeat()
    yield ('sleep', 4.0)
    win.on_stop()
    yield ('sleep', 2.0)
    shot('08b_repeat_stopped')
    # 6) 이동 중 비상정지
    mark('6 B 로 가다가 비상정지')
    fill(B_DEG)
    win.on_goto()
    yield ('until', lambda: st().get('job') is not None and st().get('mode') == 'move', 20, '이동 시작')
    yield ('sleep', 0.8)
    win.on_estop()
    yield ('sleep', 1.5)
    shot('09_estop_moving')
    # 7) 실시간 회피 다시 켜기 + 시험 물체
    mark('7 실시간 회피 켬 + 시험 물체 6 s')
    win.chk_rt.setChecked(True)
    yield ('sleep', 1.0)
    win._cmd({'cmd': 'objects', 'go': True})
    yield ('sleep', 6.0)
    shot('10_objects_hold')
    win._cmd({'cmd': 'objects', 'go': False})
    yield ('sleep', 1.0)
    if TARGET == 1:
        # 8) 서보 끄기 → 꺼진 동안 이동 명령 (거절돼야 함) → 서보 켜기
        mark('8 서보 끄기 / 이동 거절 / 서보 켜기')
        win.on_servo()
        yield ('until', lambda: st().get('servo') is False, 40, '서보 OFF')
        shot('11_servo_off')
        fill(A_DEG)
        win.on_goto()
        yield ('sleep', 1.5)
        log('서보 OFF 중 이동 명령 뒤 job =', st().get('job'))
        win.on_servo()
        yield ('until', lambda: st().get('servo') is True, 40, '서보 ON')
        yield ('sleep', 1.0)
        shot('12_servo_on')
        # 9) 알람 해제 (하드웨어 끔 → 켬)
        mark('9 알람 해제')
        win.on_alarm()
        yield ('sleep', 1.0)
        yield ('until', lambda: st().get('servo') is True and not st().get('fault'), 40, '알람 해제 뒤 서보 ON')
        shot('13_alarm_cleared')
    # 10) 성형기 다시 바꾸기 (ETM4 8 만) → A 로
    mark('10 입력성형 ETM4 8 → A 로')
    win.chk_rt.setChecked(False)
    yield ('sleep', 1.0)
    win.shaping.set_config('ETM4', [(True, 8.0), (False, 15.0), (False, 23.0)], apply=False)
    win.shaping.on_apply()
    yield ('until', lambda: win.shaping.applied == win.shaping.config(), 10, '성형기 적용 2')
    fill(A_DEG)
    win.on_goto()
    yield ('sleep', 1.0)
    yield ('until', lambda: still() and near(A_DEG), 60, 'A 도착')
    yield ('sleep', 0.8)
    shot('14_at_A')
    # 11) 관절 그래프
    win.view_tabs.setCurrentIndex(1)
    yield ('sleep', 1.2)
    win._t_scope = 0
    yield ('sleep', 0.6)
    shot('15_graph')
    win.view_tabs.setCurrentIndex(0)
    # 끝: [■ 시스템 종료] (기록 저장 → 서보 OFF → R2 끔)
    mark('끝: 시스템 종료')
    win.on_toggle_stack()
    yield ('until', lambda: not win.connected, 90, '서버 끝')
    log('끝')


U.run(seq(), app, on_timeout=lambda what: shot('TIMEOUT_' + what.replace(' ', '_')))
