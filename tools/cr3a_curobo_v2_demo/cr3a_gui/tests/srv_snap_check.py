"""[2026-10-28 D-4] 이미지 기록 통합 시험 (한 번 실행): 서버 (--robot wmx = WMX 시뮬레이터, --camera real = GUI 가 켠 고정 카메라,
--mask-q-file = 마지막 엔코더 읽기) 를 시험 포트로 띄워 실시간 회피 켬으로 [반복] 을 RUN_S 동안 → [정지] → [지도 점검] → 끄기.
기록의 주기 시간·늦음·영상 늦음·이미지 기록 수를 JSON 한 줄로 (srv_snap_check.sh 가 기록 켬·끔을 비교). 실물 포트 47861 안 씀.
  python3 tests/srv_snap_check.py <기록 폴더> <포트> <서버 출력> [RUN_S]"""
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from srv_client import GUI, Gui, start_server  # noqa: E402

out, port, slog = Path(sys.argv[1]), int(sys.argv[2]), Path(sys.argv[3])
RUN_S = float(sys.argv[4]) if len(sys.argv) > 4 else 60.0
enc = GUI / 'last_encoders.json'
p = start_server(port, out, '--robot', 'wmx', '--camera', 'real', '--mask-q-file', str(enc), '--realtime', 'on', '--speed', '0.5',
                 log_path=slog)
ok = False
try:
    g = Gui(port, timeout=400)
    if not Gui.wait(lambda: g.st.get('ready') or not g.alive or p.poll() is not None, 400):
        raise SystemExit('준비 안 됨')
    if p.poll() is not None:
        raise SystemExit(f'서버 끝남 {p.returncode}')
    qa, qb = np.degrees(g.hello['qA']), np.degrees(g.hello['qB'])
    g.send({'cmd': 'goto', 'q_deg': qa.tolist()})
    Gui.wait(lambda: g.st.get('job') is not None, 10)
    Gui.wait(lambda: g.st.get('job') is None, 60)
    g.send({'cmd': 'repeat', 'q_deg': qb.tolist()})
    time.sleep(RUN_S)
    g.send({'cmd': 'stop'})
    Gui.wait(lambda: g.st.get('job') is None, 20)
    time.sleep(3.0)
    g.send({'cmd': 'map_check'})
    time.sleep(4.0)
    g.send({'cmd': 'quit'})
    ok = True
finally:
    try:
        p.wait(240)
    except Exception:
        p.terminate()
        p.wait(60)

rec = json.loads((out / 'rt_run.json').read_text())
cy = [c for c in rec['cycles'] if c.get('cyc_ms') is not None]
cyc = np.array([c['cyc_ms'] for c in cy])
lead = np.array([c['lead_ms'] for c in cy if c.get('lead_ms') is not None])
ev = [e['text'] for e in rec['events']]
fr = np.array([f['lag_ms'] for f in rec['frames'] if f.get('real') and f.get('lag_ms') is not None])
mp = np.array([f['map_ms'] for f in rec['frames'] if f.get('real')])
pct = lambda a, q: round(float(np.percentile(a, q)), 1) if len(a) else None   # noqa: E731
print(json.dumps({
    'ok': ok, 'rc': p.returncode, 'cycles': len(cy), 'cyc_ms_p50': pct(cyc, 50), 'cyc_ms_p99': pct(cyc, 99), 'cyc_ms_max': pct(cyc, 100),
    'lead_ms_min': pct(lead, 0), 'lead_ms_p1': pct(lead, 1),
    'late_msg': sum('메시지 늦음' in t for t in ev), 'late_cycle': sum('주기 늦음' in t for t in ev),
    'frames': len(fr), 'frame_lag_ms_p50': pct(fr, 50), 'frame_lag_ms_p99': pct(fr, 99), 'map_ms_p50': pct(mp, 50), 'map_ms_p99': pct(mp, 99),
    'snaps': len(rec.get('snaps', [])), 'snap_reasons': sorted({s['reason'] for s in rec.get('snaps', [])}),
    'snap_stats': rec.get('snap_stats'),
    # [D-4] 거르기 설정·지운 픽셀 수, near 기준 고침 확인 (55~60 mm 영상 수 · near 기록 글)
    'depth_filters': rec['settings'].get('depth_filters'),
    'dfilt_tmp_p50': pct(np.array([f['dfilt'].get('tmp', 0) for f in rec['frames'] if f.get('dfilt')]), 50),
    'dfilt_band_p50': pct(np.array([f['dfilt'].get('band', 0) for f in rec['frames'] if f.get('dfilt')]), 50),
    'frames_near_55_60': sum(1 for f in rec['frames'] if f.get('near_mm') is not None and 55 <= f['near_mm'] < 60),
    'near_snap_texts': [e['text'][:60] for e in rec['events'] if '이미지 기록' in e['text']][:3],
    'near_snaps': [s['t'] for s in rec.get('snaps', []) if s['reason'] == 'near']}, ensure_ascii=False))
