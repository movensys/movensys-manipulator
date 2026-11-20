#!/usr/bin/env python3
"""[2026-10-15] WMX 실행 폴더 비교 표 (summary.json + rt_run.json): 다리, 안전 검사, 안 보낸 메시지, 주기 계산, R2 오류, 성형 확인.
   python3 -m tools.wmx_cmp <실행 폴더> …   (cr3a_rt_shaping 에서. numpy 만 — Thor 호스트 python 으로도 됨. 잔류 진동은 넣지 않음 — 사용자가 검증, 2026-10-16)
[2026-10-27 R-1] dev/ 에서 tools/ 로 옮기고 dev/cyc_time.py (한 주기 계산 시간) 를 합침: 한 주기 계산 줄에 MPC 중앙값·계산 − MPC 도 적고,
   summary.json 이 없는 폴더 (이상적 모드·보고서 전) 는 그 줄만 (rt_run.json 의 cyc_ms: MPC 부터 보내기까지, 영상 처리 뺌)."""
import argparse
import json
from pathlib import Path

import numpy as np


def cycle_line(cy):
    """한 주기 계산 시간 (cyc_ms) 중앙 / 95 % / 최대, MPC 중앙, 계산 − MPC 중앙, 안전 검사 주기 따로 (예전 dev/cyc_time.py)."""
    cm = np.array([c['cyc_ms'] for c in cy if c.get('cyc_ms') is not None])
    gm = np.array([c['cyc_ms'] for c in cy if c.get('cyc_ms') is not None and c.get('guard') is not None])
    mp = np.array([c['mpc_ms'] for c in cy if c.get('mpc_ms') is not None])
    if not len(cm):
        return '한 주기 계산: 기록 없음 (cyc_ms — 실시간 실행만)'
    s = (f'한 주기 계산 중앙 {np.median(cm):.1f} / 95 % {np.percentile(cm, 95):.1f} / 최대 {cm.max():.1f} ms'
         f' | MPC 중앙 {np.median(mp) if len(mp) else float("nan"):.1f}'
         f' | 계산 − MPC 중앙 {np.median(cm - mp[:len(cm)]) if len(mp) == len(cm) else float("nan"):.1f}')
    if len(gm):
        s += f'  | 안전 검사 주기 중앙 {np.median(gm):.1f} / 95 % {np.percentile(gm, 95):.1f} ({len(gm)} 번)'
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('dirs', nargs='+', metavar='실행 폴더')
    a = ap.parse_args()
    for d in map(Path, a.dirs):
        r = json.loads((d / 'rt_run.json').read_text())
        cy = r['cycles']
        if not (d / 'summary.json').exists():
            print(f'== {d.name}  (summary.json 없음 — 한 주기 계산만)')
            print(f'  {cycle_line(cy)}')
            continue
        s = json.loads((d / 'summary.json').read_text(encoding='utf-8'))
        legs = [L for L in s['legs'] if L.get('duration') is not None and not L['label'].startswith('시작')]   # 시작→A 빼고
        dur = np.array([L['duration'] for L in legs])
        sc = s.get('shaping_check') or {}
        r2 = s.get('r2_log') or {}
        print(f"== {d.name}  ({(s.get('shaping') or {}).get('label', 'none')})")
        if len(dur):
            print(f"  다리 {s['legs_ok']} (시작 포함), {len(dur)} 다리 합 {dur.sum():.1f} s, 중앙 {np.median(dur):.2f} / 평균 {dur.mean():.2f} / "
                  f"최대 {dur.max():.2f} s ({len(dur)} 개, 실패 {sum(not L['ok'] for L in legs)})")
        else:
            print(f"  다리 {s['legs_ok']} (시작 포함), 끝난 다리 없음")
        print(f"  안전 검사 정지 {s['guard_brakes']}, 안 보낸 메시지 {s['skipped_msgs']}, 늦은 주기 {s.get('late_cycles')}, 주기 {len(cy)}")
        print(f'  {cycle_line(cy)}')
        print(f"  R2 오류 {r2}, 엔진 가속 최대 {max(s['engine_acc_max']):.1f}")
        if sc:
            print(f"  성형 확인 중앙 {sc['median_mrad']:.3f} / 99 % {sc['p99_mrad']:.3f} / 최대 {sc['max_mrad']:.3f} mrad (지연 {sc['delay_ms']} ms)")
        print(f"  최저 거리 mm {s['min_dist_mm']}")
        print(f"  닿은 시간 ms {s['contact_ms']}")


if __name__ == '__main__':
    main()
