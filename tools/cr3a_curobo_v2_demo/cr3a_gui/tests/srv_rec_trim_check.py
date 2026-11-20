"""[2026-10-27 R-1 ③] 기록 줄이기 통합 시험: --test-rec-max-s 짧게 + 실시간 회피 켬 (서 있어도 주기마다 메시지 = 세그먼트가 안 닫힘) 으로
오래 지킨 뒤 목표 이동 두 번 → 끄기. 줄이기가 돌아도 저장 (Stream.executed) 이 되고, 기록·실행된 u·(WMX) JTC 기준 기록이 안 늘었나.
  python3 tests/srv_rec_trim_check.py <결과 폴더> [ideal|wmx] [포트 47875]      (wmx = WMX 시뮬레이터만 — 서버가 wmx_r2.sh 로 확인)
"""
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from srv_client import Gui, start_server  # noqa: E402

ROOT = Path(sys.argv[1])
ROBOT = sys.argv[2] if len(sys.argv) > 2 else 'ideal'
PORT = int(sys.argv[3]) if len(sys.argv) > 3 else 47875
HOLD_S, REC_S = 40.0, 10.0
ROOT.mkdir(parents=True, exist_ok=True)
out = ROOT / f'rec_{ROBOT}'
RES = []


def check(name, ok, detail=''):
    RES.append((name, bool(ok)))
    print(('통과' if ok else '실패'), name, detail, flush=True)


p = start_server(PORT, out, '--robot', ROBOT, '--realtime', 'on', '--speed', '1.0', '--test-rec-max-s', str(REC_S),
                 log_path=ROOT / f'rec_{ROBOT}_server.log')
g = Gui(PORT)
check('준비', g.ready())
t0 = time.monotonic()
time.sleep(HOLD_S)                              # 실시간 회피 켬으로 지킴 (20 Hz 주기 — 한 세그먼트)
qA, qB = np.array(g.hello['qA']), np.array(g.hello['qB'])
for q in (qB, qA):
    n = len(g.logs)
    g.send({'cmd': 'goto', 'q_deg': np.degrees(q).tolist()})
    ok = g.wait(lambda: any('도착' in x for x in g.logs[n:]), 90.0)
    check(f'목표 이동 → {np.round(np.degrees(q), 1).tolist()}', ok)
    time.sleep(2.0)
t_run = time.monotonic() - t0
g.send({'cmd': 'quit'})
rc = p.wait(120)
check('끝 코드 0', rc == 0, f'rc={rc}')
slog = (ROOT / f'rec_{ROBOT}_server.log').read_text(errors='ignore')
check('저장 오류 없음', '기록 저장 오류' not in slog and '서버 오류로 끝남' not in slog)
rec = json.loads((out / 'rt_run.json').read_text())
u = np.load(out / 'u_exec.npz')
n_u = int(np.sum(u['n']))
print(f'실행 {t_run:.0f} s: 주기 기록 {len(rec["cycles"])} · 영상 {len(rec["frames"])} · 사건 {len(rec["events"])} · 실행된 u {n_u} 칸 '
      f'({len(u["n"])} 세그먼트) · jtc {len(rec.get("jtc") or [])}', flush=True)
# 줄이기는 5 s 마다 (주기 100·영상 100 남김) → 끝에는 그 뒤 5 s 분 (20 Hz 100 주기 · 15 fps 75 장) 까지 더 쌓였을 수 있음
check('주기 기록이 안 늘음 (≤ 300, 줄이기 없으면 ~20 Hz × 실행 시간)', len(rec['cycles']) <= 300, f'{len(rec["cycles"])} (실행 {t_run:.0f} s)')
# 실행된 u: 지금 세그먼트 조각 100 개 (5 s) + 그 뒤 5 s + 닫힌 세그먼트 하나 (다리 하나 ≤ 몇 s) — 안 줄이면 실행 시간 × 1000 칸
check('실행된 u 가 안 늘음 (< 실행 시간의 절반)', n_u < 0.5 * t_run * 1000, f'{n_u} 칸 (실행 {t_run:.0f} s = {t_run * 1000:.0f} 칸)')
if ROBOT != 'ideal':
    check('JTC 기준 기록이 안 늘음 (≤ 200 + 5 s 분)', len(rec.get('jtc') or []) < 200 + 5 * 1000, f'{len(rec.get("jtc") or [])}')
print('결과', sum(ok for _, ok in RES), '/', len(RES))
for name, ok in RES:
    print('  ', '통과' if ok else '실패', name)
