#!/usr/bin/env python3
"""GUI WMX 실행 검증 — WMX 축 로그 (엔진이 1 ms 마다 쓴 명령·실제 위치) 로 GUI 조작마다 명령이 맞게 나갔나 본다.  ★ wmx_r2 컨테이너 안 ★
  cd ~/workspaces/movensys_ws/cr3a_gui && PYTHONPATH=../cr3a_rt_shaping python3 -m tools.wmx_check <기록 폴더> [서버 로그]
  → <폴더>/wmx_check.json + 표 (3D 재생 보고서는 cr3a_rt_shaping 에서 python3 -m tools.rt_report <폴더>)
  [2026-10-27 R-1] tools/ 로 옮김. 기록 읽기 (WMX 축 로그·실행된 u·전달 지연 맞춤·R2 로그 줄) 는 cr3a_rt_shaping/rt_records (numpy 만 —
  예전 rt_report 를 불러 torch·warp 까지 불러오던 것 뺌) — 그래서 PYTHONPATH 에 cr3a_rt_shaping
읽는 것: wmx_axis.csv, rt_run.json (서버 사건·주기·보낸 명령·비상정지 구간·성형기 목록), u_exec*.npz (성형기마다 실행된 u), r2_launch.log
보는 것
  ① 안전: 켤 때 엔진 플랫폼 = 시뮬레이터뿐 (서버 로그), R2 오류·CyclicBuffer 결함 (끄기 전까지)
  ② 명령 연속성: 엔진이 본 1 ms 가속 |ΔCMDVEL| ÷ 1 ms (CyclicBuffer maxAcc 가 보는 값) 관절별 최대
  ③ 성형: 성형기마다 WMX 명령 위치 = 실행된 u 전체를 그 성형기로 한 번에 컨볼루션한 것 (전달 지연 맞춘 뒤 차이). 성형기를 쓴 시각 범위
     (until = 바꾼 시각) 도 적음. [R-1 ⑮] 비상정지 감속 구간 (성형을 건너뜀) 은 뺌 — 서버가 실행된 u 를 그 스탬프에서 끊어 보통은 뺄 점이 없음
  ④ 조작별 (서버 사건 시각 기준): 이동 도착 (WMX 실제 위치 − 목표), [정지]·비상정지 (사건 → WMX 명령 속도 0 까지 시간, 그동안 최대 1 ms 가속),
     서보 끄기~켜기 (그동안 명령 위치가 그대로인지), 성형기 바꾸기 (바꾼 앞뒤 1 s 명령 가속), 서보 꺼진 동안 이동 명령 거절
  ⑤ 실시간: 늦어서 안 보낸 메시지, 보낼 때 여유 (스탬프 − 다 만든 시각), 한 주기 계산 시간
"""
import argparse
import json
import re
from pathlib import Path

import numpy as np

from rt_records import R2_PATTERNS, estop_windows, fit_delay, load_run, outside_windows, r2_max_acc, read_text, read_wmx_csv, u_exec_runs
from rt_shaping import shape_fir

V_STOP = 1e-4               # rad/s: 이보다 느리면 섬
MAX_ACC = 15.0              # R2 cyclic_max_acc 기본 (엔진이 보는 1 ms 가속 한계, wmx_r2.sh R2_ARGS 와 같음) — [R-1 ⑮] R2 로그의 maxAcc 가 있으면 그것


def first_still(t, v, t0, hold=0.05):
    """t0 뒤 명령 속도가 hold s 동안 V_STOP 아래로 이어지기 시작한 시각 (없으면 None)."""
    k = np.searchsorted(t, t0)
    still = np.abs(v[k:]).max(axis=1) < V_STOP
    n = max(1, int(round(hold / 0.001)))
    run = 0
    for i, s in enumerate(still):
        run = run + 1 if s else 0
        if run >= n:
            return float(t[k + i - n + 1])
    return None


def shaping_by_kernel(out, rec, wt, wq):
    """성형기마다: 실행된 u (u_exec*.npz) 를 그 성형기로 한 번에 컨볼루션 → WMX 명령과 차이 (움직이는 구간, 전달 지연 맞춤)."""
    res = []
    W = estop_windows(rec)
    for kinfo, segs, h, g in u_exec_runs(out, rec):
        f = kinfo['file']
        ts, Y = [], []
        for T0, b, q in segs:
            ts.append(T0 + np.arange(len(q)) * g)
            Y.append(shape_fir(q, h, before=b))
        ts, Y = np.concatenate(ts), np.concatenate(Y)
        m = (ts >= wt[0] + 0.1) & (ts <= wt[-1] - 0.2) & outside_windows(ts, W)
        ts, Y = ts[m], Y[m]
        if len(ts) < 10:
            continue
        mov = np.abs(np.gradient(Y, g, axis=0)).max(axis=1) > 0.05
        sub = np.nonzero(mov)[0][::10] if mov.sum() > 20 else np.arange(0, len(ts), 10)

        def err(L):
            W = np.stack([np.interp(ts[sub] + L, wt, wq[:, j]) for j in range(6)], 1)
            return float(np.median(np.abs(W - Y[sub]).max(axis=1)))
        # 0.5 ms 격자 → 0.1 ms → 0.01 ms (R2 버퍼를 다시 열면 지연이 0.x ms 바뀌어 0.5 ms 격자로는 속도 × 0.25 ms 만큼 차이가 남음 — 10-26 시험,
        #  배율 1.0 회피 시험: 2.4 rad/s × 0.05 ms 격자 오차 = 0.12 mrad 가 남았음 → 0.07). rt_report 와 같은 함수 (rt_records.fit_delay)
        L = fit_delay(err)
        d = np.abs(np.stack([np.interp(ts + L, wt, wq[:, j]) for j in range(6)], 1) - Y).max(axis=1) * 1000
        res.append({'label': kinfo['label'], 'file': f, 'until': kinfo['until'], 'segments': len(segs), 'impulses': int((h > 0).sum()),
                    'len_ms': round((len(h) - 1) * g * 1000, 1), 'delay_ms': round(L * 1000, 2), 'moving_s': round(float(mov.sum() * g), 2),
                    'median_mrad': round(float(np.median(d[mov])), 4) if mov.any() else None,
                    'max_mrad': round(float(d[mov].max()), 4) if mov.any() else None, 'max_all_mrad': round(float(d.max()), 4)})
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('out', help='기록 폴더 (rt_run.json · wmx_axis.csv · u_exec*.npz · r2_launch.log)')
    ap.add_argument('server_log', nargs='?', help='서버 로그 (기본 <폴더>/server.log)')
    a = ap.parse_args(argv)
    out = Path(a.out)
    slog = Path(a.server_log) if a.server_log else out / 'server.log'
    rec = load_run(out)
    w = read_wmx_csv(out / 'wmx_axis.csv')
    wt, cmd, fb, cv = w['unix'], w['cmd'], w['fb'], w['cmdvel']
    ev = rec['events']
    t_quit = next((e['t'] for e in ev if e['text'].startswith('끄기')), None) or next(
        (e['t'] for e in reversed(ev) if e['text'] == '서버 끝'), wt[-1])
    a1 = np.abs(np.diff(cv, axis=0)) / 0.001                # 엔진이 본 1 ms 가속 (rad/s²)
    ta = wt[1:]
    rep = {'folder': out.name, 'wmx_s': round(float(wt[-1] - wt[0]), 1), 'wmx_rows': int(len(wt))}

    # ① 안전·R2
    sl = read_text(slog)
    rep['safety'] = {'module_ini_simu': 'Module.ini 의 켜진 플랫폼: simu_platform.so' in sl,
                     'engine_simu_only': '시뮬레이터 확인: OK (플랫폼은 simu 뿐)' in sl,
                     'engine_platform_line': (re.findall(r'플랫폼 모듈: (.*)', sl) or [None])[0]}
    r2 = read_text(out / 'r2_launch.log')
    bad = []
    for line in r2.splitlines():
        line = re.sub(r'\x1b\[[0-9;]*m', '', line)
        mt = re.search(r'\[(WARN|ERROR|FATAL)\] \[(\d+\.\d+)\]', line)
        if mt and float(mt.group(2)) < t_quit and (mt.group(1) != 'WARN' or re.search(r'Cyclic|MaxAcc|stall|fault', line, re.I)):
            bad.append(line.split(']: ', 1)[-1][:160])
    rep['r2_errors_before_quit'] = bad[:10]
    rep['r2_counts'] = {k: len(re.findall(R2_PATTERNS[p], r2)) for k, p in (('MaxAccError', 'max_acc_error'), ('faulted', 'faulted'),
                                                                            ('push_rejected', 'push_rejected'), ('stalled', 'stalled'))}

    # ② 명령 연속성
    on = (ta >= wt[0]) & (ta <= t_quit + 5.0)
    rep['engine_acc_max'] = [round(float(x), 2) for x in a1[on].max(axis=0)]
    rep['engine_acc_limit'] = r2_max_acc(r2) or MAX_ACC
    rep['cmd_minus_fb_max_mrad'] = [round(float(x), 3) for x in (np.abs(fb - cmd).max(axis=0) * 1000)]

    # ③ 성형
    rep['shaping'] = shaping_by_kernel(out, rec, wt, cmd)

    # ④ 조작별
    ops = []
    win = rec.get('estop_windows') or []

    def acc_between(t0, t1):
        k = (ta >= t0) & (ta <= t1)
        return round(float(a1[k].max()), 2) if k.any() else None

    def moving_at(t0):
        k = np.searchsorted(wt, t0)
        return float(np.abs(cv[max(0, k - 5):k + 5]).max()) if k < len(wt) else 0.0

    for e in ev:
        t, x = e['t'], e['text']
        row = None
        if x.startswith('비상정지:') or (': 성형 건너뛰고' in x):
            ts_ = first_still(wt, cv, t)
            w_ = next((wd for wd in win if wd[0] >= t - 0.01), None)
            row = {'op': '비상정지' if x.startswith('비상정지') else x.split(':')[0], 'v_at_event': round(moving_at(t), 4),
                   'stop_after_ms': None if ts_ is None else round((ts_ - t) * 1000), 'acc_max': acc_between(t, (ts_ or t) + 0.05),
                   'planned_window': w_}
        elif '성형 거쳐 감속 정지' in x:
            ts_ = first_still(wt, cv, t)
            row = {'op': x.split(':')[0], 'v_at_event': round(moving_at(t), 4),
                   'stop_after_ms': None if ts_ is None else round((ts_ - t) * 1000), 'acc_max': acc_between(t, (ts_ or t) + 0.05)}
        elif x.startswith('입력성형 적용'):
            row = {'op': x[:60], 'acc_max_pm1s': acc_between(t - 1.0, t + 1.0), 'v_at_event': round(moving_at(t), 5)}
        elif re.search(r' 도착 [\d.]+초$', x):
            lbl = x.rsplit(' 도착', 1)[0]
            cy = min(rec['cycles'], key=lambda c: abs(c['t'] - t)) if rec['cycles'] else None
            goal = np.array(cy['goal']) if cy and cy.get('goal') else None
            k = min(np.searchsorted(wt, t + 0.5), len(wt) - 1)
            row = {'op': f'{lbl} 도착', 'time_s': float(x.rsplit(' ', 1)[-1][:-1]),
                   'fb_minus_goal_deg': None if goal is None else round(float(np.degrees(np.abs(fb[k] - goal).max())), 4)}
        elif x.startswith('서보 OFF') or x.startswith('서보 ON') or x.startswith('알람 해제 →'):
            row = {'op': x[:40], 'v_at_event': round(moving_at(t), 5)}
        elif '서보가 꺼져 있음' in x or 'R2 오류' in x:
            row = {'op': x[:60]}
        if row is not None:
            row['t'] = round(t - wt[0], 3)
            ops.append(row)
    # 서보 꺼진 구간: 명령 위치가 그대로였나
    offs = [e['t'] for e in ev if e['text'].startswith('서보 OFF')]
    ons = [e['t'] for e in ev if e['text'].startswith('서보 ON') or e['text'].startswith('알람 해제 →')]
    for t0 in offs:
        t1 = next((t for t in ons if t > t0), None)
        if t1 is None:
            continue
        k = (wt >= t0) & (wt <= t1)
        if k.any():
            ops.append({'op': '서보 꺼진 동안', 't': round(t0 - wt[0], 3), 'dur_s': round(t1 - t0, 2),
                        'cmd_drift_mrad': round(float(np.abs(cmd[k] - cmd[k][0]).max() * 1000), 4),
                        'fb_drift_mrad': round(float(np.abs(fb[k] - fb[k][0]).max() * 1000), 4)})
    rep['operations'] = ops

    # ⑤ 실시간
    cyc = [c for c in rec['cycles'] if c.get('lead_ms') is not None]
    ld = np.array([c['lead_ms'] for c in cyc]) if cyc else np.array([np.nan])
    cm = np.array([c['cyc_ms'] for c in cyc]) if cyc else np.array([np.nan])
    rep['realtime'] = {'cycles': len(cyc), 'late_events': sum(1 for e in ev if e['text'].startswith('메시지 늦음')),
                       'catchup_events': sum(1 for e in ev if e['text'].startswith('주기 늦음')),
                       'lead_ms': {'min': round(float(np.nanmin(ld)), 1), 'p1': round(float(np.nanpercentile(ld, 1)), 1),
                                   'median': round(float(np.nanmedian(ld)), 1)},
                       'cyc_ms': {'median': round(float(np.nanmedian(cm)), 1), 'p99': round(float(np.nanpercentile(cm, 99)), 1),
                                  'max': round(float(np.nanmax(cm)), 1)},
                       'skipped': sum(1 for c in rec['cycles'] if c.get('skipped')),
                       'guard': sum(1 for c in rec['cycles'] if c.get('guard'))}
    (out / 'wmx_check.json').write_text(json.dumps(rep, ensure_ascii=False, indent=1), encoding='utf-8')
    print(json.dumps(rep, ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
