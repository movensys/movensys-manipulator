"""[2026-10-28 D-4] 깊이 거르기 서버 시험 (한 번 실행): 이상적 로봇 (WMX·R2 없음) + 실제 고정 카메라 (GUI 가 켠 노드 — 구독만) +
마지막 엔코더 자세로 팔 지우기. 실시간 회피 켬으로 RUN_S 동안 지키기 → [지도 점검] → 끄기. 영상마다 지도 처리 시간·늦음·지운 픽셀 수
(rec frames dfilt) 를 JSON 한 줄로. 거르기 켬·끔은 환경 변수 (CR3A_DEPTH_TEMPORAL·CR3A_DEPTH_BAND) 로. 실물 포트 47861 안 씀.
  python3 tests/srv_depth_filter_check.py <기록 폴더> <포트> <서버 출력> [RUN_S]"""
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from srv_client import GUI, Gui, start_server  # noqa: E402

out, port, slog = Path(sys.argv[1]), int(sys.argv[2]), Path(sys.argv[3])
RUN_S = float(sys.argv[4]) if len(sys.argv) > 4 else 40.0
p = start_server(port, out, '--robot', 'ideal', '--camera', 'real', '--mask-q-file', str(GUI / 'last_encoders.json'),
                 '--realtime', 'on', log_path=slog)
ok = False
try:
    g = Gui(port, timeout=400)
    if not Gui.wait(lambda: g.st.get('ready') or not g.alive or p.poll() is not None, 400):
        raise SystemExit('준비 안 됨')
    time.sleep(RUN_S)
    g.send({'cmd': 'map_check'})
    time.sleep(3.0)
    g.send({'cmd': 'quit'})
    ok = True
finally:
    try:
        p.wait(120)
    except Exception:
        p.terminate()
        p.wait(60)

rec = json.loads((out / 'rt_run.json').read_text())
fr = [f for f in rec['frames'] if f.get('real')]
mp = np.array([f['map_ms'] for f in fr])
lag = np.array([f['lag_ms'] for f in fr])
cy = [c for c in rec['cycles'] if c.get('cyc_ms') is not None]
df = [f['dfilt'] for f in fr if f.get('dfilt')]
pct = lambda a, q: round(float(np.percentile(a, q)), 1) if len(a) else None   # noqa: E731
print(json.dumps({
    'ok': ok, 'rc': p.returncode, 'settings': rec['settings'].get('depth_filters'), 'frames': len(fr),
    'map_ms_p50': pct(mp, 50), 'map_ms_p99': pct(mp, 99), 'lag_ms_p50': pct(lag, 50), 'lag_ms_p99': pct(lag, 99),
    'cycles': len(cy), 'cyc_ms_p99': pct(np.array([c['cyc_ms'] for c in cy]), 99),
    'late': sum('늦음' in e['text'] for e in rec['events']),
    'dfilt_tmp_p50': pct(np.array([d.get('tmp', 0) for d in df]), 50), 'dfilt_band_p50': pct(np.array([d.get('band', 0) for d in df]), 50),
    'dfilt_band_max': max((d.get('band', 0) for d in df), default=None),
    'near_mm_min': min((f['near_mm'] for f in fr if f.get('near_mm') is not None), default=None),
    'checks': [(c['label'], c['ok'], c.get('d_b_mm')) for c in rec.get('checks', [])]}, ensure_ascii=False))
