"""[2026-10-27 R-1 리팩터링] 안전 시험 (d) + (a) — GUI 가 이상적 로봇 서버를 시험 포트로 띄워서 (화면 없이, 실제 로봇·R2·카메라 없음).
  (d) [▶ 시스템 시작] (대상 이상적·지도 가상) → 서버 --robot ideal --port <시험 포트> → 연결·준비 → 위 칩·카메라 세팅 탭 칩·지도 글이
      가상 카메라·이상적 로봇에 맞나, 거리 글 = rt_config, GUI 로그 파일이 쓰이나, 실시간 회피 끔 → 지도 칩 '지도 멈춤', 그래프 단추
  (a) 누르는 동안 조그 (J2 −10°/s) 중 GUI 스레드를 1.0 s 막음 (QTimer.singleShot(0, time.sleep(1.0))) → 조그 스레드가 막힌 지 0.4 s 안에
      jog_release 를 한 번 보내고 끝내나, 다시 누를 때까지 jog_hold 를 안 보내나, 빨리 뗐다 눌러도 스레드 하나·뗌 뒤 jog_hold 없음.
      이상적 모드 서버는 조그를 거절 (MoveIt Servo 없음) 하므로 '조그가 멈춘 시각' 은 서버로 간 명령 줄 (GUI 가 소켓에 다 쓴 시각) 에
      서버 규칙 (rt_server: jog_release = 바로 뗌, jog_hold 가 JOG_TIMEOUT_S 0.4 s 안 오면 뗌) 을 적용해 셈.
  끝: [■ 시스템 종료] → 서버 프로세스 끝 (코드 0).
  CR3A_GUI_PORT=47873 CR3A_TMP_DIR=<폴더> XDG_CONFIG_HOME=<폴더> python3 tests/test_safety_live.py   (tests/run_r1_tests.sh 가 이렇게 부름)
"""
import os
import threading
import time

import seq_util as U
from seq_util import check, log

U.guard_test_port()
app = U.make_app()
from PyQt5 import QtCore  # noqa: E402
import camera_panel as cp  # noqa: E402
import common  # noqa: E402
import cr3a_gui as G  # noqa: E402

JOG_TIMEOUT_S = 0.4         # rt_server.JOG_TIMEOUT_S (서버 규칙)
STALL_S = 1.0               # GUI 스레드를 막는 시간
win = G.MainWindow()
G.keep_running_on_errors()
win.resize(1600, 1000)
win.show()
sent = []                   # (시각 monotonic, 명령, 보낸 스레드) — GUI 가 서버로 보낸 조그 명령 (소켓에 다 쓴 뒤 시각)
server = {'proc': None}


def record_sends():
    orig = win.link.send

    def send(m):
        ok = orig(m)
        if isinstance(m, dict) and str(m.get('cmd', '')).startswith('jog'):
            sent.append((time.monotonic(), m['cmd'], threading.current_thread().name))
        return ok
    win.link.send = send


def texts(chips):
    return {k: lb.text() for k, lb in chips.items()}


def check_d():
    c, s = texts(win.chips), texts(win.st_chips)
    check('(d) 위 칩: 로봇', c['robot'] == '이상적 로봇', c['robot'])
    check('(d) 위 칩: WMX', c['wmx'] == '안 씀', c['wmx'])
    check('(d) 위 칩: 시스템', c['sys'] == '켜짐', c['sys'])
    check('(d) 위 칩: 카메라', c['cam'] == f'가상 {G.VCAM_FPS:g} fps', c['cam'])
    check('(d) 위 칩: 장애물 회피', c['obst'] == '켜짐', c['obst'])
    check('(d) 카메라 세팅 탭 칩: 카메라 (전엔 늘 초록 "가상 카메라")', s['cam'] == f'가상 카메라 {G.VCAM_FPS:g} fps', s['cam'])
    check('(d) 카메라 세팅 탭 칩: 지도', s['map'] == '지도 쌓는 중', s['map'])
    check('(d) 칩 둘만 (팔 지우기·계획이 지도 읽음 뺌)', sorted(s) == ['cam', 'map'], sorted(s))
    m = win.lb_map.text()
    check('(d) 지도 글 = 가상 카메라', f'가상 카메라 {G.VCAM_FPS:g} Hz' in m and '실제' not in m, m)
    ci = win.lb_caminfo.text()
    check('(d) 카메라 세팅 글 = 지도 가상 카메라·쌓는 중', '가상 카메라 (시뮬레이션' in ci and '쌓는 중' in ci, ci[-120:])
    check('(d) 거리 글 = rt_config 값', win.lb_dist.text() == G.DIST_TEXT and G.DIST_TEXT.startswith('피하는 거리 6 cm · 팔 지우기 4 cm · 바닥 2 cm'),
          win.lb_dist.text())
    log_txt = G.SERVER_LOG.read_text(encoding='utf-8', errors='replace') if G.SERVER_LOG.exists() else ''
    check('(d) 서버 로그 = 시험 폴더, 시험 포트로 켬', G.SERVER_LOG.parent == common.TMP_DIR and f'127.0.0.1:{common.PORT}' in log_txt,
          f'{G.SERVER_LOG} ({len(log_txt)} B)')
    for h in G.LOG.handlers:
        h.flush()
    gl = common.GUI_LOG.read_text(encoding='utf-8') if common.GUI_LOG.exists() else ''
    check('(d) GUI 로그 파일이 쓰임', '서버에 연결됨' in gl and '시스템 시작' in gl, f'{common.GUI_LOG} ({len(gl)} B)')
    view = win.log_view.toPlainText()
    check('(d) 로그 창에도 같은 줄', '서버에 연결됨' in view, view.splitlines()[-1][:80] if view else '')


def hold_stall():
    """(a) 누르는 동안 조그 + GUI 스레드 1 s 막힘."""
    win.rb_hold.setChecked(True)
    win.cb_jstep.setCurrentText('10')
    sent.clear()
    t_press = time.monotonic()
    win._jog_press('joint', 1, -1)
    yield ('sleep', 0.6)
    stall = {}

    def block():                                    # GUI 스레드에서 1 s 멈춤 (3D 그리기가 길 때와 같은 상황)
        stall['t0'] = time.monotonic()
        time.sleep(STALL_S)
        stall['t1'] = time.monotonic()
    QtCore.QTimer.singleShot(0, block)
    yield ('until', lambda: 't1' in stall, 5, f'GUI 스레드 {STALL_S} s 막기')
    yield ('sleep', 0.6)                            # 막힌 뒤에도 사용자는 단추를 누른 채 (뗌 신호 없음)
    t0 = stall['t0']
    hold_t = [t for t, c, _ in sent if c == 'jog_hold']
    rel = [(t, th) for t, c, th in sent if c == 'jog_release']
    before = [t for t in hold_t if t < t0]
    check('(a) 누르는 동안 jog_hold 를 50 ms 마다 보냄', len(before) >= 8, f'{len(before)} 개 / {t0 - t_press:.2f} s')
    check('(a) 막힌 뒤 jog_release 한 번 (조그 스레드가)', len(rel) == 1 and rel[0][1] == 'hold_jog', rel)
    if not rel:
        return
    t_rel = rel[0][0]
    last_hold = max([t for t in hold_t if t <= t_rel], default=t0)
    stop_at = min(t_rel, last_hold + JOG_TIMEOUT_S)     # 서버 규칙: 뗌이 오거나 0.4 s 동안 jog_hold 가 안 오면 섬
    check('(a) jog_release 를 막힌 지 0.4 s 안에 보냄', t_rel - t0 <= 0.4, f'{t_rel - t0:.3f} s (HOLD_STALL_S {G.HOLD_STALL_S} + 주기)')
    check('(a) 서버가 조그를 멈춘 시각 (서버 규칙) ≤ 막힌 지 0.4 s', stop_at - t0 <= 0.4,
          f'{stop_at - t0:.3f} s · 마지막 jog_hold {last_hold - t0:+.3f} s')
    after = [t for t in hold_t if t > t_rel]
    check('(a) jog_release 뒤 jog_hold 없음 (다시 누를 때까지)', not after, f'{len(after)} 개 · 막힌 동안 {STALL_S} s + 뒤 0.6 s')
    check('(a) hold_jog 비움', win.hold_jog is None)
    check('(a) 로그 창 알림', '화면 (GUI 스레드) 이' in win.log_view.toPlainText())
    check('(a) 서버 연결 유지 (1 s 막힘 — GUI 끊김 2 s 아래)', win.connected)
    n = len(sent)
    win._jog_release()                              # 사용자가 단추를 뗌 — 이미 뗐으므로 더 안 보냄
    yield ('sleep', 0.3)
    check('(a) 단추 뗌: 더 보내지 않음', len(sent) == n, sent[n:])
    # 다시 누르기 · 빨리 뗐다 누르기 (스레드 하나, 뗌 뒤 jog_hold 없음)
    marks = []
    for k in range(12):
        marks.append(('press', time.monotonic()))
        win._jog_press('joint', 1, -1 if k % 2 else +1)
        yield ('sleep', 0.12 if k % 3 else 0.06)
        win._jog_release()
        marks.append(('release', time.monotonic()))
        yield ('sleep', 0.02)
    yield ('sleep', 0.3)
    new = sent[n:]
    holds = [t for t, c, _ in new if c == 'jog_hold']
    rels = [t for t, c, _ in new if c == 'jog_release']
    presses = [t for k, t in marks if k == 'press']
    bad = [t for t in holds if not any(p <= t <= r for p, r in zip(presses, [t for k, t in marks if k == 'release']))]
    check('(a) 다시 누르면 다시 보냄', len(holds) >= 12, f'jog_hold {len(holds)} 개')
    check('(a) 뗌마다 jog_release 하나', len(rels) == 12, f'{len(rels)} 개')
    check('(a) jog_hold 는 누른 동안에만 (뗌 뒤에 끼지 않음 — 잠금 하나)', not bad, f'밖 {len(bad)} 개')
    nth = sum(1 for th in threading.enumerate() if th.name == 'hold_jog')
    check('(a) 조그 스레드 하나', nth == 1, nth)
    win.rb_step.setChecked(True)


def seq():
    yield ('sleep', 1.0)
    U.refuse_real(win)
    if not check('(d) 연결 전: 시험 포트에 서버 없음', not win.connected and not U.listening(common.PORT), f'포트 {common.PORT}'):
        return                                      # (남의 서버면 끄지 않게 — 여기서 끝)
    xdg = os.environ.get('XDG_CONFIG_HOME')
    check('(d) Qt 설정 파일 = 시험 폴더 (실물 GUI 설정을 안 건드림)', bool(xdg) and win.settings.fileName().startswith(xdg), win.settings.fileName())
    win.cb_target.setCurrentIndex(0)                # 이상적 시뮬레이션
    win.cb_mapcam.setCurrentIndex(0)                # 가상 카메라
    win.on_toggle_stack()                           # [▶ 시스템 시작]
    p = server['proc'] = win.server_proc
    a = [str(x) for x in (p.args if p is not None else [])]
    ok = '--port' in a and '--robot' in a and a[a.index('--port') + 1] == str(common.PORT) and a[a.index('--robot') + 1] == 'ideal'
    check('(d) 서버를 시험 포트·이상적 로봇·가상 카메라로 띄움', ok and '--camera' not in a, ' '.join(a[2:6]))
    if p is None:
        return
    yield ('until', lambda: win.connected and (win.state or {}).get('ready'), 420, '서버 준비')
    U.refuse_real(win)
    record_sends()
    yield ('until', lambda: time.time() - win._t_vox < 1.0, 10, '지도 칸 받음')
    yield ('sleep', 1.0)
    check_d()
    win.chk_rt.setChecked(False)                    # 실시간 회피 끔 → 지도 칩 (서버 부담도 줄임)
    yield ('until', lambda: (win.state or {}).get('realtime') is False, 15, '실시간 회피 끔')
    yield ('sleep', 0.5)
    check('(d) 실시간 회피 끔 → 지도 칩 "지도 멈춤"', win.st_chips['map'].text() == '지도 멈춤', win.st_chips['map'].text())
    check('(d) 실시간 회피 끔 → 위 칩 "지도 멈춤"', win.chips['obst'].text() == '지도 멈춤', win.chips['obst'].text())
    # 그래프 단추 ([R-1 ④] 전엔 안 이어짐)
    check('(d) 그래프 꺼짐: 안 쌓음', len(win.scope_buf) == 0, len(win.scope_buf))
    win.scope.btn_run.setChecked(True)
    yield ('sleep', 1.0)
    n_on = len(win.scope_buf)
    check('(d) [▶ 그래프 켜기]: 쌓음', n_on >= 10 and win.scope.btn_run.text() == '■ 그래프 끄기', f'{n_on} 개 · {win.scope.btn_run.text()}')
    win.scope.btn_run.setChecked(False)
    n_off = len(win.scope_buf)
    yield ('sleep', 0.6)
    check('(d) [■ 그래프 끄기]: 더 안 쌓음', len(win.scope_buf) == n_off, f'{n_off} → {len(win.scope_buf)}')
    yield from hold_stall()
    win.on_toggle_stack()                           # [■ 시스템 종료]
    yield ('until', lambda: not win.connected, 90, '서버 연결 끊김')
    yield ('until', lambda: p.poll() is not None, 60, '서버 프로세스 끝')
    check('(d) 서버 프로세스 끝 (코드 0)', p.poll() == 0, p.poll())


U.run(seq(), app)
p = server['proc']
if p is not None and p.poll() is None:              # (시험이 중간에 멈춰도 이 시험이 띄운 서버는 끔 — SIGTERM → 서버 finally)
    log('서버가 아직 돎 → SIGTERM')
    p.terminate()
    try:
        p.wait(60)
    except Exception:
        p.kill()
cp.finish_jobs()
raise SystemExit(U.summary())
