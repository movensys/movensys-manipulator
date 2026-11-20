"""[2026-10-28 R-1 통합 시험] HEAD 전체 (GUI + 서버 + 엔진 + GPU 3D) 를 WMX 시뮬레이터로 — GUI 를 사람이 쓰듯 단추를 차례로 누름.
  ★ wmx_r2 컨테이너 안, WMX 시뮬레이터만 — Module.ini (EtherCAT disable = 1) 확인은 tests/integ_r1.sh 가 켜기 전마다 ★
  python3 tests/integ_r1.py --part A|B --out <폴더>        (tests/integ_r1.sh 가 부름 — 환경 변수: 시험 포트·임시 폴더·기록 폴더)
  A (가상 카메라)
    ① [▶ 시스템 시작] (대상 WMX 시뮬레이터) → 칩·글·파일 (GUI·서버 로그, 기록 폴더, 안전 줄)·[그래프]
    ② 누르는 동안 조그 (MoveIt Servo): 관절 J1 ± · J6 (A 자세) · 공구 끝 Z ± (A 에서 아래로 50 mm 간 자세 — A 는 팔꿈치가 거의 펴져
       Servo 특이점) → 움직인 양·뗀 뒤 서기까지. GUI 스레드 1 s 막기 → 뗌 ≤ 0.31 s, 뗌 뒤 jog_hold 없음, 로봇이 선 시각.
       빠른 누름 12 번 (뗌마다 jog_release 하나, 누른 동안에만 jog_hold)
    ③ 배율 1.0 · ETM4 8 Hz → 관절 목표 B → A, 공구 끝 Z −50 → +50 mm (도착·끝 위치)
    ④ 실시간 회피 + 시험 물체 + [반복] 60 s → [정지] → 영점 → [■ 시스템 종료] (서버 끝 코드 0)
       (성형 확인·R2 오류 = tools.wmx_check, 닿음·늦은 메시지 = cr3a_rt_shaping tools.rt_report — integ_r1.sh 가 끝난 뒤)
  B (실제 카메라 — 시뮬레이션 로봇, 팔은 마지막 엔코더 자세로 지움)
    카메라 켜기 (영상이 안 오면 initial_reset:=true 로 한 번 띄웠다 끄고 다시) → [▶ 시스템 시작] → 칩·글
    ⑤ 실시간 회피 끔 → 켬 (켜기 전 지도 점검) · [지도 점검] · 많은 지도 칸에서: 누르는 동안 조그 4 s (3D 20 Hz) 의 GUI 20 ms 타이머 간격·
       3D 한 장 시간 (paintGL·refresh)·멈춤 없이 이어짐 → GUI 1 s 막기 → 뗌 ≤ 0.31 s → [■ 시스템 종료] → [■ 끄기] (카메라 노드 끝)
  결과: PASS/FAIL 줄 + <폴더>/part_<A|B>.json (숫자), 기록 폴더 이름은 <폴더>/rec_<A|B>.txt
"""
import argparse
import json
import shutil
import signal
import subprocess
import threading
import time
from pathlib import Path

import numpy as np

import seq_util as U
from seq_util import check, log

ap = argparse.ArgumentParser()
ap.add_argument('--part', choices=['A', 'B'], required=True)
ap.add_argument('--out', required=True)
ARGS = ap.parse_args()
U.guard_test_port()
U.refuse_if_live_server()
OUT = Path(ARGS.out)
OUT.mkdir(parents=True, exist_ok=True)
app = U.make_app()
from PyQt5 import QtCore  # noqa: E402
import camera_io as ci  # noqa: E402
import camera_panel as cp  # noqa: E402
import common  # noqa: E402
import cr3a_gui as G  # noqa: E402

A_DEG = (56.31, -40.67, -12.99, -29.16, 83.72, 55.72)       # 시뮬레이션 시험 자세 (rt_config qa·qb_tool = 서버 hello qA·qB)
B_DEG = (-28.78, -42.61, -10.82, -27.41, 96.28, -26.31)
STALL_S = 1.0
RELEASE_MAX_S = 0.31                                        # 리드 기준: GUI 스레드가 멈춘 뒤 jog_release 까지
win = G.MainWindow()
G.keep_running_on_errors()
win.resize(1600, 1000)
win.show()
R = {'part': ARGS.part}
STATES = []                 # (서버 벽시계 t, q rad, jog, job) — state 메시지마다
NEARS = []                  # 지도 점검 결과 (near 메시지)
SENT = []                   # (monotonic, 명령, 보낸 스레드) — GUI 가 서버로 보낸 조그 명령 (소켓에 다 쓴 뒤)
_on_msg = win._on_msg


def on_msg(m):
    t = m.get('type')
    if t == 'state':
        STATES.append((float(m['t']), np.asarray(m['q'], float), bool(m.get('jog')), m.get('job')))
    elif t == 'near':
        NEARS.append({'ok': bool(m.get('ok')), 'n': int(len(m.get('P', []))), 'text': str(m.get('text', '')), 'w': time.time()})
    _on_msg(m)


win._on_msg = on_msg


def record_sends():
    orig = win.link.send

    def send(m):
        ok = orig(m)
        if isinstance(m, dict) and str(m.get('cmd', '')).startswith('jog'):
            SENT.append((time.monotonic(), m['cmd'], threading.current_thread().name))
        return ok
    win.link.send = send


def st():
    return win.state or {}


def q_deg():
    return np.degrees(np.asarray(st()['q'], float))


def near(target, tol=0.2):
    s = st()
    return 'q' in s and float(np.max(np.abs(np.degrees(s['q']) - np.asarray(target, float)))) < tol


def still():
    return st().get('job') is None and not st().get('jog')


def fill(q):
    for sp, v in zip(win.sp_target, q):
        sp.setValue(v)


def slog():
    return G.SERVER_LOG.read_text(encoding='utf-8', errors='replace') if G.SERVER_LOG.exists() else ''


def stop_after(t_ref, hold=0.3, tol=np.radians(0.005)):
    """t_ref (벽시계) 뒤 로봇이 선 시각까지 s — 서버 상태 (20 Hz) 에서 hold s 동안 모든 관절 변화가 tol 안이 되기 시작한 때. 못 찾으면 None."""
    S = [(t, q) for t, q, _, _ in STATES if t >= t_ref - 0.05]
    for i, (t, q) in enumerate(S):
        win_ = [q2 for t2, q2 in S[i:] if t2 <= t + hold]
        if len(win_) >= 3 and S[-1][0] >= t + hold and float(np.max(np.ptp(np.array(win_), axis=0))) < tol:
            return round(t - t_ref, 3)
    return None


def start_system(mapcam):
    """① [▶ 시스템 시작]: 대상 WMX 시뮬레이터 + 지도 카메라 (0 가상 · 1 실제) → 준비까지."""
    yield ('sleep', 1.0)
    U.refuse_real(win)
    if not check(f'{ARGS.part} 켜기 전: 시험 포트에 서버 없음', not win.connected and not U.listening(common.PORT), f'포트 {common.PORT}'):
        raise SystemExit(2)
    win.cb_target.setCurrentIndex(1)                # WMX 시뮬레이터
    win.cb_mapcam.setCurrentIndex(mapcam)
    t0 = time.time()
    win.on_toggle_stack()
    p = win.server_proc
    a = [str(x) for x in (p.args if p is not None else [])]
    ok = ('--port' in a and a[a.index('--port') + 1] == str(common.PORT) and '--robot' in a and a[a.index('--robot') + 1] == 'wmx'
          and (('--camera' in a and '--mask-q-file' in a) if mapcam else '--camera' not in a))
    if not check(f'{ARGS.part} 서버를 시험 포트·WMX 시뮬레이터·{"실제" if mapcam else "가상"} 카메라로 띄움', ok, ' '.join(a[2:12])):
        raise SystemExit(2)
    R['server_pid'] = p.pid
    yield ('until', lambda: win.connected and st().get('ready'), 900, '서버 준비 (R2 + 지도·계획기)')
    R['ready_s'] = round(time.time() - t0, 1)
    U.refuse_real(win)
    if not check(f'{ARGS.part} 서버 준비', win.connected and st().get('ready'), f'{R["ready_s"]} s'):
        raise SystemExit(2)
    record_sends()
    rec = Path(win.hello.get('out') or '')
    R['rec'] = str(rec)
    (OUT / f'rec_{ARGS.part}.txt').write_text(str(rec))
    yield ('until', lambda: time.time() - win._t_vox < 1.0, 20, '지도 칸 받음')
    yield ('sleep', 1.0)
    sl = slog()
    check(f'{ARGS.part} ① 안전 줄: Module.ini 시뮬레이터뿐 · 엔진 시뮬레이터 확인',
          'Module.ini 의 켜진 플랫폼: simu_platform.so' in sl and '시뮬레이터 확인: OK' in sl)
    check(f'{ARGS.part} ① MoveIt Servo 준비', 'MoveIt Servo 준비' in sl)
    axl = rec / 'wmx_axis_log.out'
    check(f'{ARGS.part} ① 기록 폴더 (R2 로그·WMX 축 로그 시작)', bool(win.hello.get('out')) and (rec / 'r2_launch.log').exists()
          and axl.exists() and '기록 시작' in axl.read_text(errors='replace'), str(rec))
    for h in G.LOG.handlers:
        h.flush()
    gl = common.GUI_LOG.read_text(encoding='utf-8') if common.GUI_LOG.exists() else ''
    check(f'{ARGS.part} ① 파일: GUI 로그 · 서버 로그 (시험 포트)', '서버에 연결됨' in gl and f'127.0.0.1:{common.PORT}' in sl,
          f'{common.GUI_LOG.name} {len(gl)} B · {G.SERVER_LOG.name} {len(sl)} B')


def chips_common(cam_real):
    c, s = {k: lb.text() for k, lb in win.chips.items()}, {k: lb.text() for k, lb in win.st_chips.items()}
    R.setdefault('chips', {})[ARGS.part] = {'top': c, 'cam_tab': s, 'map': win.lb_map.text()}
    check(f'{ARGS.part} ① 칩: 로봇 = WMX 시뮬레이터', c['robot'] == 'WMX 시뮬레이터', c['robot'])
    check(f'{ARGS.part} ① 칩: WMX = 시뮬레이터 · 서보 ON (· 통신)', c['wmx'].startswith('시뮬레이터 · 서보 ON'), c['wmx'])
    check(f'{ARGS.part} ① 칩: 시스템 켜짐', c['sys'] == '켜짐', c['sys'])
    if cam_real:
        fps = st().get('cam_fps') or 0.0
        check(f'{ARGS.part} ① 칩: 카메라 = 실제 (fps > 5)', c['cam'].startswith('실제') and fps > 5 and s['cam'].startswith('실제 카메라'),
              f'{c["cam"]} · {s["cam"]} · 서버 {fps} fps')
        check(f'{ARGS.part} ① 지도 글 = 실제 카메라', '실제 카메라' in win.lb_map.text(), win.lb_map.text())
    else:
        check(f'{ARGS.part} ① 칩: 카메라 = 가상 15 fps (위·카메라 탭)', c['cam'] == f'가상 {G.VCAM_FPS:g} fps'
              and s['cam'] == f'가상 카메라 {G.VCAM_FPS:g} fps', f'{c["cam"]} · {s["cam"]}')
        check(f'{ARGS.part} ① 지도 칩·글: 쌓는 중 · 가상 카메라', s['map'] == '지도 쌓는 중' and '가상 카메라' in win.lb_map.text(),
              f'{s["map"]} · {win.lb_map.text()}')


def graph_button():
    check(f'{ARGS.part} ① [그래프] 꺼짐: 안 쌓음', len(win.scope_buf) == 0, len(win.scope_buf))
    win.scope.btn_run.setChecked(True)
    yield ('sleep', 1.0)
    n_on = len(win.scope_buf)
    win.scope.btn_run.setChecked(False)
    yield ('sleep', 0.6)
    check(f'{ARGS.part} ① [그래프] 켜기 → 쌓음 · 끄기 → 멈춤', n_on >= 10 and len(win.scope_buf) == n_on, f'{n_on} 개 → {len(win.scope_buf)}')


def hold_axis(kind, axis, sign, secs, rate):
    """누르는 동안 조그 한 번: 누름 → secs → 뗌 → Servo 조그 끝. 움직인 양·뗀 뒤 선 시각."""
    if kind == 'joint':
        win.cb_jstep.setCurrentText(rate)
    else:
        win.cb_cstep.setCurrentText(rate)
    yield ('until', lambda: still(), 15, '조그 전 서 있음')
    q0, T0 = q_deg(), win.robot.tip(q_deg())
    win._jog_press(kind, axis, sign)
    yield ('sleep', secs)
    t_rel = time.time()
    win._jog_release()
    yield ('until', lambda: not st().get('jog'), 10, 'Servo 조그 끝')
    yield ('sleep', 0.5)
    q1, T1 = q_deg(), win.robot.tip(q_deg())
    name = f'{kind} {axis} {"+" if sign > 0 else "-"}'
    if kind == 'joint':
        moved = float(q1[axis] - q0[axis])
        others = float(np.max(np.abs(np.delete(q1 - q0, axis))))
        ok_move = sign * moved > 0.3 * float(rate) * secs and others < 0.5
        what = f'{moved:+.2f}° (다른 관절 최대 {others:.3f}°)'
    else:
        k = 'XYZ'.index(axis)
        moved = float((T1[k, 3] - T0[k, 3]) * 1000)
        ok_move = sign * moved > 0.3 * float(rate) * secs
        what = f'{moved:+.1f} mm'
    stop = stop_after(t_rel)
    R.setdefault('hold', []).append({'axis': name, 'secs': secs, 'rate': rate, 'moved': round(moved, 3), 'stop_after_s': stop})
    check(f'② 누르는 동안 조그 {name} {rate}/s {secs} s: 움직임', ok_move, what)
    check(f'② 조그 {name}: 뗀 뒤 1 s 안에 섬', stop is not None and stop <= 1.0, f'{stop} s')


def stall_test(tag, kind='joint', axis=0, sign=1):
    """GUI 스레드 1 s 막기 (누르는 동안 조그 중) → 뗌 ≤ 0.31 s, 뗌 뒤 jog_hold 없음, 로봇이 선 시각 (서버 상태)."""
    win.rb_hold.setChecked(True)
    win.cb_jstep.setCurrentText('10')
    yield ('until', lambda: still(), 15, '조그 전 서 있음')
    n0 = len(SENT)
    t_press = time.monotonic()
    win._jog_press(kind, axis, sign)
    yield ('sleep', 0.6)
    stall = {}

    def block():
        stall['t0'], stall['w0'] = time.monotonic(), time.time()
        time.sleep(STALL_S)
        stall['t1'] = time.monotonic()
    QtCore.QTimer.singleShot(0, block)
    yield ('until', lambda: 't1' in stall, 5, f'GUI 스레드 {STALL_S} s 막기')
    yield ('sleep', 0.8)
    new = SENT[n0:]
    t0 = stall['t0']
    holds = [t for t, c, _ in new if c == 'jog_hold']
    rel = [(t, th) for t, c, th in new if c == 'jog_release']
    before = [t for t in holds if t < t0]
    check(f'{tag} 누르는 동안 jog_hold 50 ms 마다', len(before) >= 8, f'{len(before)} 개 / {t0 - t_press:.2f} s')
    check(f'{tag} 막힌 뒤 jog_release 한 번 (조그 스레드)', len(rel) == 1 and rel[0][1] == 'hold_jog', rel)
    if rel:
        t_rel = rel[0][0]
        last = max([t for t in holds if t <= t_rel], default=t0)
        after = [t for t in holds if t > t_rel]
        R.setdefault('stall', []).append({'tag': tag, 'release_s': round(t_rel - t0, 4), 'last_hold_s': round(last - t0, 4),
                                          'robot_stop_s': stop_after(stall['w0'])})
        check(f'{tag} jog_release ≤ {RELEASE_MAX_S} s (막힌 때부터)', t_rel - t0 <= RELEASE_MAX_S,
              f'{t_rel - t0:.3f} s · 마지막 jog_hold {last - t0:+.3f} s')
        check(f'{tag} 뗌 뒤 jog_hold 없음', not after, f'{len(after)} 개')
        yield ('until', lambda: not st().get('jog'), 10, 'Servo 조그 끝')
        yield ('sleep', 0.5)
        rs = stop_after(stall['w0'])
        R['stall'][-1]['robot_stop_s'] = rs
        check(f'{tag} 로봇 (WMX 시뮬레이터) 이 막힌 지 1 s 안에 섬', rs is not None and rs <= 1.0, f'{rs} s (서버 상태 20 Hz)')
    check(f'{tag} 로그 창 알림 · 서버 연결 유지', '화면 (GUI 스레드) 이' in win.log_view.toPlainText() and win.connected)
    win._jog_release()                              # (사용자가 단추를 뗌 — 이미 뗐으므로 안 보냄)


def rapid_presses():
    yield ('until', lambda: still(), 15, '조그 전 서 있음')
    n = len(SENT)
    marks = []
    for k in range(12):
        marks.append(('press', time.monotonic()))
        win._jog_press('joint', 0, -1 if k % 2 else +1)
        yield ('sleep', 0.12 if k % 3 else 0.06)
        win._jog_release()
        marks.append(('release', time.monotonic()))
        yield ('sleep', 0.02)
    yield ('until', lambda: not st().get('jog'), 10, 'Servo 조그 끝')
    yield ('sleep', 0.8)
    new = SENT[n:]
    holds = [t for t, c, _ in new if c == 'jog_hold']
    rels = [t for t, c, _ in new if c == 'jog_release']
    P = [t for k, t in marks if k == 'press']
    Q = [t for k, t in marks if k == 'release']
    bad = [t for t in holds if not any(p <= t <= r for p, r in zip(P, Q))]
    R['rapid'] = {'holds': len(holds), 'releases': len(rels), 'outside': len(bad)}
    check('② 빠른 누름 12 번: 뗌마다 jog_release 하나 · jog_hold 는 누른 동안에만', len(rels) == 12 and not bad and len(holds) >= 12,
          R['rapid'])
    check('② 빠른 누름 뒤 로봇 섬 · 조그 스레드 하나', still() and sum(1 for th in threading.enumerate() if th.name == 'hold_jog') == 1)


def goto(name, q, timeout=90):
    fill(q)
    t0 = time.time()
    win.on_goto()
    yield ('until', lambda: st().get('job') is not None, 15, f'{name} 이동 시작')
    yield ('until', lambda: still() and near(q, 0.2), timeout, f'{name} 도착')
    R.setdefault('goto', []).append({'to': name, 's': round(time.time() - t0, 2)})
    check(f'③ 관절 목표 {name} 도착 (0.2° 안)', near(q, 0.2), f'{time.time() - t0:.1f} s')


def goto_tip(dz_mm, timeout=90):
    win._fill_xyz()
    xyz = np.array([sp.value() for sp in win.sp_xyz])
    tgt = xyz + np.array([0.0, 0.0, dz_mm])
    win.sp_xyz[2].setValue(tgt[2])
    win.chk_keep_rot.setChecked(True)
    t0 = time.time()
    win._pose_cmd('goto_pose')
    yield ('until', lambda: st().get('job') is not None, 20, f'공구 끝 Z {dz_mm:+.0f} mm 이동 시작')
    yield ('until', lambda: still(), timeout, f'공구 끝 Z {dz_mm:+.0f} mm 도착')
    T = win.robot.tip(q_deg())
    err = float(np.linalg.norm(T[:3, 3] * 1000 - tgt))
    R.setdefault('goto', []).append({'to': f'tip Z {dz_mm:+.0f}', 's': round(time.time() - t0, 2), 'err_mm': round(err, 3)})
    check(f'③ 공구 끝 목표 Z {dz_mm:+.0f} mm 도착 (1.5 mm 안 — 관절 도착 0.1° 기준)', err < 1.5, f'{err:.3f} mm · {time.time() - t0:.1f} s')


def shutdown():
    p = win.server_proc
    win.on_toggle_stack()                           # [■ 시스템 종료]
    yield ('until', lambda: not win.connected, 180, '서버 연결 끊김')
    yield ('until', lambda: p is None or p.poll() is not None, 180, '서버 프로세스 끝')
    rc = None if p is None else p.poll()
    R['server_exit'] = rc
    sl = slog()
    check(f'{ARGS.part} ⑥ 서버 끝 코드 0 · R2 끔 · Servo 끔', rc == 0 and 'R2 꺼짐' in sl and 'MoveIt Servo 끔' in sl and '서버 프로세스 끝' in sl,
          f'코드 {rc}')
    rec = Path(R.get('rec', ''))
    if rec.is_dir():
        shutil.copy(G.SERVER_LOG, rec / 'server.log')          # (tools.wmx_check 가 <기록>/server.log 를 읽음)


def seq_a():
    yield from start_system(0)
    chips_common(cam_real=False)
    check('A ① 칩: 장애물 회피 켜짐', win.chips['obst'].text() == '켜짐', win.chips['obst'].text())
    yield from graph_button()
    # ③ 목표 이동 (관절) — 배율 1.0 · ETM4 8 Hz. 조그보다 먼저: 처음 자세가 영점 (팔을 세움 — 공구 끝 위·아래 조그는 특이점) 일 수 있어 A 로
    win.sl_speed.setValue(100)
    yield ('until', lambda: abs((st().get('speed') or 0) - 1.0) < 1e-6, 10, '배율 1.0')
    win.shaping.set_config('ETM4', [(True, 8.0), (False, 15.0), (False, 23.0)], apply=False)
    win.shaping.on_apply()
    yield ('until', lambda: win.shaping.applied == win.shaping.config(), 15, '성형기 ETM4 8 Hz')
    check('③ 배율 1.0 · 성형기 ETM4 8 Hz', abs((st().get('speed') or 0) - 1.0) < 1e-6 and win.shaping.applied == win.shaping.config())
    R['q_start_deg'] = np.round(q_deg(), 2).tolist()
    for name, q in (('B', B_DEG), ('A', A_DEG)):
        yield from goto(name, q)
    # ② 누르는 동안 조그 (MoveIt Servo) — 관절은 A 자세에서
    win.rb_hold.setChecked(True)
    for kind, axis, sign, secs, rate in (('joint', 0, +1, 1.5, '10'), ('joint', 0, -1, 1.5, '10'), ('joint', 5, +1, 1.5, '10')):
        yield from hold_axis(kind, axis, sign, secs, rate)
    yield from stall_test('② GUI 1 s 막힘 (Servo):', 'joint', 0, -1)
    yield from rapid_presses()
    win.rb_step.setChecked(True)
    # ③ 공구 끝 목표 (cuRobo IK) · ② 공구 끝 조그 — [통합 시험 1 차] A 는 팔꿈치가 거의 펴져 (J3 −13°) 공구 끝 Z 조그는 Servo 특이점
    #  멈춤 ('Very close to a singularity'), 위로 50 mm 는 닿지 않음 (IK 24.7 mm 모자람 — 서버가 '안 움직임') → 아래로 먼저 (팔꿈치 굽힘,
    #  J3 −44°) 가서 그 자리에서 공구 끝 조그, 그 뒤 위로 50 mm (A 의 공구 끝 높이로 돌아감)
    yield from goto_tip(-50)
    win.rb_hold.setChecked(True)
    for kind, axis, sign, secs, rate in (('cart', 'Z', +1, 1.5, '20'), ('cart', 'Z', -1, 1.5, '20')):
        yield from hold_axis(kind, axis, sign, secs, rate)
    win.rb_step.setChecked(True)
    yield from goto_tip(+50)
    # ④ 실시간 회피 + 시험 물체 + [반복] 60 s
    if not st().get('realtime'):
        win.chk_rt.setChecked(True)
        yield ('until', lambda: st().get('realtime') is True, 20, '실시간 회피 켬')
    check('④ 실시간 회피 켜짐', st().get('realtime') is True)
    win._cmd({'cmd': 'objects', 'go': True})
    fill(B_DEG)
    n_log0 = len(slog())
    win.on_repeat()
    t0, n_state0 = time.time(), len(STATES)
    for k in range(4):
        yield ('sleep', 15.0)
        s = st()
        log(f'④ {time.time() - t0:4.0f} s: job {s.get("job")} · mode {s.get("mode")} · clr {s.get("clr_mm")} mm · 지도 칸 {win.n_vox} · '
            f'guard {s.get("guard")} · cyc {s.get("cyc_ms")} ms')
    alive = win.connected and win.server_proc is not None and win.server_proc.poll() is None
    n_states = len(STATES) - n_state0
    win.on_stop()
    yield ('until', lambda: still(), 40, '[정지] 뒤 섬')
    win._cmd({'cmd': 'objects', 'go': False})
    sl = slog()
    seg = sl[n_log0:]
    R['avoid'] = {'secs': round(time.time() - t0, 1), 'states': n_states, 'r2_error_lines': sl.count('R2 오류'),
                  'server_error_lines': sl.count('서버 오류'), 'legs_arrived': seg.count(' 도착 '),
                  'late_lines': seg.count('메시지 늦음'), 'catchup_lines': seg.count('주기 늦음')}
    check('④ 실시간 회피 60 s: 서버·연결 살아 있음 · 상태 20 Hz', alive and n_states > 900, f'상태 {n_states} 개 / {R["avoid"]["secs"]} s')
    check('④ R2 오류·서버 오류 줄 없음', R['avoid']['r2_error_lines'] == 0 and R['avoid']['server_error_lines'] == 0, R['avoid'])
    yield ('sleep', 1.0)
    yield from goto('영점', (0.0,) * 6)                # (B 의 실제 카메라 지도에서 시작 자세 — 10-26 실제 카메라 WMX 시험의 자세)
    yield from shutdown()


def camera_on():
    """실제 고정 카메라 노드 켜기 (GUI [▶ 카메라 켜기]) — 컬러 영상이 20 s 안에 안 오면 initial_reset:=true 로 한 번 띄웠다 끄고 다시."""
    win.tabs.setCurrentIndex(2)
    win.on_cam_on()
    yield ('until', lambda: win.cam_feed is not None and win.cam_feed.fps('color') > 5, 20, '카메라 영상')
    if not (win.cam_feed is not None and win.cam_feed.fps('color') > 5):
        log('카메라 영상이 안 옴 → 노드 끄고 initial_reset:=true 로 한 번 (USB 다시 붙이기)')
        R['camera_reset'] = True
        win.on_cam_off()
        yield ('until', lambda: not win._cam_busy, 20, '카메라 끔')
        y = ci.CAM_YAML.with_name('cr3a_cam_reset.yaml')
        y.write_text(ci.realsense_yaml(ci.CAM_TOPIC, ci.CAM_SERIAL) + '    initial_reset: true\n')
        with open(ci.CAM_LOG.with_name('cr3a_cam_reset.log'), 'w') as f:
            p = subprocess.Popen(ci.realsense_argv(ci.CAM_NS, y), stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        yield ('sleep', 15.0)
        p.send_signal(signal.SIGINT)
        try:
            p.wait(10)
        except subprocess.TimeoutExpired:
            p.terminate()
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
        yield ('sleep', 3.0)
        win.on_cam_on()
        yield ('until', lambda: win.cam_feed is not None and win.cam_feed.fps('color') > 5, 30, '카메라 영상 (다시 붙인 뒤)')
    fps = win.cam_feed.fps('color') if win.cam_feed is not None else 0.0
    R['camera_fps'] = round(fps, 1)
    if not check('B 카메라 영상 (컬러 > 5 fps)', fps > 5, f'{fps:.1f} fps · reset {R.get("camera_reset", False)}'):
        raise SystemExit(2)


def instrument_3d():
    """3D 한 장 시간: paintGL (GUI 스레드 CPU) · refresh (바뀐 것 넣기) — 인스턴스 함수로 감쌈 (sip 은 인스턴스 사전부터 봄)."""
    sc = win.scene
    T = {'paint': [], 'paint_t': [], 'refresh': []}
    if sc.view is None:
        return T
    view, paint0, refresh0 = sc.view, sc.view.paintGL, sc.refresh

    def paint():
        t0 = time.perf_counter()
        paint0()
        T['paint'].append(time.perf_counter() - t0)
        T['paint_t'].append(time.monotonic())

    def refresh():
        t0 = time.perf_counter()
        refresh0()
        T['refresh'].append(time.perf_counter() - t0)
    view.paintGL = paint
    sc.refresh = refresh
    return T


def stats_ms(xs):
    if not xs:
        return None
    a = np.array(xs) * 1000
    return {'n': len(a), 'median': round(float(np.median(a)), 2), 'p95': round(float(np.percentile(a, 95)), 2), 'max': round(float(a.max()), 2)}


def seq_b():
    yield from camera_on()
    yield from start_system(1)
    chips_common(cam_real=True)
    # ⑤ 켜기 전 지도 점검 (실시간 회피 끔 → 켬) · [지도 점검]
    if st().get('realtime'):
        win.chk_rt.setChecked(False)
        yield ('until', lambda: st().get('realtime') is False, 15, '실시간 회피 끔')
    yield ('until', lambda: still(), 20, '섬')
    yield ('sleep', 1.0)
    n0 = len(NEARS)
    win.chk_rt.setChecked(True)                     # 켜기 → 서버가 켜기 전 지도 점검 (1.4 s)
    yield ('until', lambda: len(NEARS) > n0, 20, '켜기 전 지도 점검 결과')
    yield ('sleep', 0.5)
    r = NEARS[n0] if len(NEARS) > n0 else None
    R['turn_on_check'] = r and {k: r[k] for k in ('ok', 'n', 'text')}
    check('⑤ 켜기 전 지도 점검이 돌고 알림 (통과면 켜짐, 걸림이면 안 켜짐)', r is not None and '실시간 회피 켜기 전 지도 점검' in r['text']
          and bool(st().get('realtime')) == r['ok'], None if r is None else f'{"통과" if r["ok"] else "걸림"} · 칸 {r["n"]} · {r["text"][:110]}')
    n1 = len(NEARS)
    win._cmd({'cmd': 'map_check'})                   # [지도 점검]
    yield ('until', lambda: len(NEARS) > n1, 20, '[지도 점검] 결과')
    r = NEARS[n1] if len(NEARS) > n1 else None
    R['map_check'] = r and {k: r[k] for k in ('ok', 'n', 'text')}
    check('⑤ [지도 점검] 결과 알림 (3D 자홍 점 = 칸)', r is not None and r['text'].startswith('지도 점검')
          and (win.scene.near is None) == (r['n'] == 0), None if r is None else f'{"통과" if r["ok"] else "걸림"} · 칸 {r["n"]}')
    if st().get('realtime'):                         # (조그·시간 재기는 지도를 멈추고 — 실시간 회피의 비키기가 끼지 않게)
        win.chk_rt.setChecked(False)
        yield ('until', lambda: st().get('realtime') is False, 15, '실시간 회피 끔')
    # ⑤ 많은 지도 칸 + 3D 20 Hz (조그로 움직이는 동안) — GUI 20 ms 타이머 간격·3D 한 장 시간·조그가 안 끊김
    T = instrument_3d()
    gaps, last = [], [time.monotonic()]

    def beat():
        now = time.monotonic()
        gaps.append(now - last[0])
        last[0] = now
    tm = QtCore.QTimer(interval=20, timeout=beat)
    win.view_tabs.setCurrentIndex(0)
    win.rb_hold.setChecked(True)
    win.cb_jstep.setCurrentText('10')
    yield ('until', lambda: still(), 15, '조그 전 서 있음')
    n_vox, n_sent = win.n_vox, len(SENT)
    for k in ('paint', 'paint_t', 'refresh'):
        T[k].clear()
    tm.start()
    last[0] = time.monotonic()
    gaps.clear()
    q0 = q_deg()
    t_meas = time.monotonic()
    win._jog_press('joint', 0, +1)
    yield ('sleep', 4.0)
    win._jog_release()
    dur = time.monotonic() - t_meas
    tm.stop()
    yield ('until', lambda: not st().get('jog'), 10, 'Servo 조그 끝')
    rel = [t for t, c, th in SENT[n_sent:] if c == 'jog_release']
    spurious = [t for t, c, th in SENT[n_sent:] if c == 'jog_release' and th == 'hold_jog']
    fr = T['paint_t']
    R['load'] = {'n_vox': n_vox, 'gui_gap_ms': stats_ms(gaps), 'paint_ms': stats_ms(T['paint']), 'refresh_ms': stats_ms(T['refresh']),
                 'frames_per_s': round(len(fr) / dur, 1) if fr else 0.0, 'moved_deg': round(float(q_deg()[0] - q0[0]), 2),
                 'gl': win.scene.view is not None}
    log('⑤ 부하:', json.dumps(R['load'], ensure_ascii=False))
    check('⑤ 지도 칸이 많음 (실제 카메라)', n_vox >= 1000, f'{n_vox} 칸')
    check('⑤ 3D = GPU (pyqtgraph) 로 그림', win.scene.view is not None and len(fr) > 0, f'{len(fr)} 장 / {dur:.1f} s')
    g = R['load']['gui_gap_ms']
    check('⑤ 3D 가 GUI 스레드를 굶기지 않음: 20 ms 타이머 간격 최대 < 150 ms', g is not None and g['max'] < 150, g)
    check('⑤ 조그 4 s 동안 화면 멈춤 뗌 없음 (사용자 뗌 하나만)', not spurious and len(rel) == 1, f'조그 스레드 뗌 {len(spurious)} · 전체 {len(rel)}')
    check('⑤ 조그로 움직임 (3D 가 20 Hz 로 따라 그림)', R['load']['moved_deg'] > 10, f'{R["load"]["moved_deg"]}°')
    yield from stall_test('⑤ GUI 1 s 막힘 (실제 카메라 지도·3D):', 'joint', 0, -1)
    win.rb_step.setChecked(True)
    yield from shutdown()
    # 카메라 노드 끄기 ([■ 끄기])
    pid, own = ci.CameraProcess.pid(), ci.CameraProcess._proc
    win.on_cam_off()
    yield ('until', lambda: not win._cam_busy, 20, '카메라 끔')
    gone = pid is None or ci._proc_state(pid) in (None, 'Z')
    R['camera_exit'] = None if own is None else own.returncode
    check('B ⑥ 카메라 노드 끔 (거둠)', gone and ci.CameraProcess.pid() is None,
          f'PID {pid} → 상태 {None if pid is None else ci._proc_state(pid)} · 끝 코드 {R["camera_exit"]}')


def finish():
    (OUT / f'part_{ARGS.part}.json').write_text(json.dumps(R, ensure_ascii=False, indent=1, default=str), encoding='utf-8')
    QtCore.QTimer.singleShot(0, win.close)          # (연결 안 됨 → 묻지 않고 끝)
    yield ('sleep', 10)


def main_seq():
    try:
        yield from (seq_a() if ARGS.part == 'A' else seq_b())
    except SystemExit:
        log('시험 중지 (앞 단계 실패)')
    except Exception as e:                          # (시험 코드 오류도 FAIL 로 남기고 정리 단계로)
        import traceback
        traceback.print_exc()
        check('시험 순서가 오류 없이 끝까지', False, repr(e))
    yield from finish()


timeouts = U.run(main_seq(), app)
p = win.server_proc
if p is not None and p.poll() is None:              # (시험이 중간에 멈춰도 이 시험이 띄운 서버는 끔 — SIGTERM → 서버가 R2·Servo 를 끄고 끝)
    log('서버가 아직 돎 → SIGTERM, 끝날 때까지 (R2 끄기 포함)')
    p.terminate()
    try:
        p.wait(240)
    except subprocess.TimeoutExpired:
        log('서버가 240 s 안에 안 끝남 — 손으로 확인 필요')
if ARGS.part == 'B' and ci.CameraProcess.pid() is not None:     # (중간에 멈췄으면 이 시험이 켠 카메라 노드도 끔 — PID 파일이 시험 폴더)
    log('카메라 노드가 아직 돎 → 끔:', ci.CameraProcess.stop())
if timeouts:
    check('시간 초과 단계 없음', False, timeouts)
cp.finish_jobs()
raise SystemExit(U.summary())
