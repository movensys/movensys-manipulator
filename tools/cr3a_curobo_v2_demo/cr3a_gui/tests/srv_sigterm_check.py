"""[2026-10-27 R-1 ⑥] SIGTERM 통합 시험 (이상적 모드 — R2 없음). python3 tests/srv_sigterm_check.py <결과 폴더> [포트 47875]
  ① 목표 이동 중 SIGTERM 한 번 → [■ 시스템 종료] 와 같은 끄기: 사건 'SIGTERM 받음 → 끄기' → '끄기: 성형 거쳐 감속 정지' → '서버 끝', 기록 저장,
     마지막 명령 속도 0, 끝 코드 143
  ② 목표 이동 중 SIGTERM 두 번 → 바로 끝 (끝 코드 143), finally 가 기록은 저장
  ③ 켜는 중 (준비 전) SIGTERM → 바로 끝 (끝 코드 143)
"""
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from srv_client import Gui, start_server  # noqa: E402

ROOT = Path(sys.argv[1])
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 47875
ROOT.mkdir(parents=True, exist_ok=True)
RES = []


def check(name, ok, detail=''):
    RES.append((name, bool(ok)))
    print(('통과' if ok else '실패'), name, detail, flush=True)


def moving_goto(g):
    qA, qB = np.array(g.hello['qA']), np.array(g.hello['qB'])
    far = qB if np.max(np.abs(g.u() - qB)) > np.max(np.abs(g.u() - qA)) else qA
    u0 = g.u()
    g.send({'cmd': 'goto', 'q_deg': np.degrees(far).tolist()})
    return g.wait(lambda: g.st.get('job') is not None and np.max(np.abs(g.u() - u0)) > np.radians(3.0), 60.0)


def events(out):
    p = out / 'rt_run.json'
    return [e['text'] for e in json.loads(p.read_text())['events']] if p.exists() else None


# ① 한 번
out = ROOT / 'once'
p = start_server(PORT, out, '--robot', 'ideal', '--realtime', 'off', '--speed', '0.5', log_path=ROOT / 'once_server.log')
g = Gui(PORT)
check('① 준비', g.ready())
check('① 목표 이동 시작', moving_goto(g))
t_sig = time.monotonic()
os.kill(p.pid, signal.SIGTERM)
rc = p.wait(90)
ev = events(out) or []
i_sig = next((i for i, e in enumerate(ev) if 'SIGTERM 받음' in e), None)
i_stop = next((i for i, e in enumerate(ev) if e.startswith('끄기: 성형 거쳐 감속 정지')), None)
i_end = next((i for i, e in enumerate(ev) if e == '서버 끝'), None)
check('① 끝 코드 143', rc == 143, f'rc={rc}, {time.monotonic() - t_sig:.1f} s')
check('① 기록 저장 (rt_run.json·u_exec.npz)', (out / 'rt_run.json').exists() and (out / 'u_exec.npz').exists())
check('① 사건 차례: SIGTERM 받음 → 끄기: 성형 거쳐 감속 정지 → 서버 끝', None not in (i_sig, i_stop, i_end) and i_sig < i_stop < i_end,
      f'{i_sig} {i_stop} {i_end}: ' + ' | '.join(ev[i_sig:] if i_sig is not None else ev[-5:])[:400])
if (out / 'rt_run.json').exists():
    cy = json.loads((out / 'rt_run.json').read_text())['cycles']
    v_last = float(np.max(np.abs(cy[-1]['cmd_v']))) if cy else None
    check('① 마지막 주기 명령 속도 0 (선 뒤 끝)', v_last is not None and v_last < 1e-6, f'{v_last}')

# ② 두 번
out = ROOT / 'twice'
p = start_server(PORT, out, '--robot', 'ideal', '--realtime', 'off', '--speed', '0.5', log_path=ROOT / 'twice_server.log')
g = Gui(PORT)
check('② 준비', g.ready())
check('② 목표 이동 시작', moving_goto(g))
os.kill(p.pid, signal.SIGTERM)
time.sleep(0.01)
os.kill(p.pid, signal.SIGTERM)
rc = p.wait(90)
check('② 두 번째 신호 = 바로 끝 (끝 코드 143), 기록은 저장', rc == 143 and (out / 'rt_run.json').exists(), f'rc={rc}')

# ③ 켜는 중 — 처리기의 SystemExit 이 weakref 콜백 안에서 삼켜지면 다음 켜는 단계 (startup_check) 에서 끝 (지도·계획기 준비 ~25 s 뒤)
out = ROOT / 'startup'
p = start_server(PORT, out, '--robot', 'ideal', '--realtime', 'off', log_path=ROOT / 'startup_server.log')
time.sleep(8.0)                                 # (지도·계획기 준비 중 — 1~2 분)
t_sig = time.monotonic()
os.kill(p.pid, signal.SIGTERM)
try:
    rc = p.wait(180)
except subprocess.TimeoutExpired:               # (안 끝나면 남기지 않음 — 다음 시험이 같은 포트에 붙지 않게)
    p.kill()
    rc = f'{p.wait()} (180 s 안에 안 끝나 SIGKILL)'
slog = (ROOT / 'startup_server.log').read_text(errors='ignore')
check('③ 켜는 중 SIGTERM = 끝 (끝 코드 143, 준비 끝까지 안 감)', rc == 143 and '준비 끝 — 명령을 기다림' not in slog,
      f'rc={rc}, {time.monotonic() - t_sig:.1f} s, 삼켜짐={"Exception ignored" in slog}, 다시 봄={"켜는 중 SIGTERM 받음" in slog}')

print('결과', sum(ok for _, ok in RES), '/', len(RES))
for name, ok in RES:
    print('  ', '통과' if ok else '실패', name)
