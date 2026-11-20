"""[2026-10-28 R-1 통합 시험] 확인 도구 (컨테이너 안, 읽기만) — tests/integ_r1.sh 가 부름.
  python3 tests/integ_check.py ini              Module.ini: EtherCAT (ec_platform.so) disable = 1 · 켜진 플랫폼 = simu_platform.so 하나 (아니면 끝 코드 1)
  python3 tests/integ_check.py procs            남은 프로세스 (WMX 엔진·R2·Servo·카메라 노드·서버·GUI·축 로그) 와 GUI 포트 (있으면 끝 코드 1)
  python3 tests/integ_check.py summary <폴더>   항목마다 PASS/FAIL + 숫자 (part_*.json · 기록 폴더의 wmx_check.json · summary.json)
pgrep -f 대신 /proc 을 읽음 (ssh 로 부를 때 원격 셸의 명령 줄이 패턴과 맞지 않게 — 리드 규칙).
"""
import json
import os
import sys
from pathlib import Path

PATS = ('wmx3_engine', 'wmx_r2_package/', 'ros2_control_node', 'wmx_r2_control_manipulator.launch.py', 'servo_node', 'cr3a_servo.launch.py',
        'realsense2_camera_node', 'rt_server.py', 'cr3a_gui.py', 'integ_r1.py', 'wmx_axis_log')
PORTS = (47861, 47873)
SHAPING_MAX_MRAD = 1.0          # 성형 확인 (WMX 명령 − 실행된 u 의 컨볼루션) 움직이는 구간 최대 — 넘으면 FAIL (지난 검증 ≤ 0.07 mrad)


def ini():
    secs, cur = [], None
    for line in Path('/opt/wmx3/Module.ini').read_text(errors='replace').splitlines():
        s = line.strip()
        if s.startswith('['):
            cur = {'name': s.replace(' ', ''), 'dll': None, 'disable': 0}
            secs.append(cur)
        elif '=' in s and cur is not None:
            k, v = (x.strip() for x in s.split('=', 1))
            if k == 'DllName':
                cur['dll'] = v
            elif k == 'disable':
                cur['disable'] = int(v or 0)
    plat = [s for s in secs if s['name'].startswith('[Platform')]
    ec = [s for s in plat if s['dll'] == 'ec_platform.so']
    on = sorted(s['dll'] for s in plat if not s['disable'])
    ok = bool(ec) and all(s['disable'] == 1 for s in ec) and on == ['simu_platform.so']
    print(f"Module.ini: EtherCAT (ec_platform.so) disable = {[s['disable'] for s in ec]} · 켜진 플랫폼 {on} → "
          + ('OK (시뮬레이터 하나)' if ok else '✗ 시뮬레이터 하나만이 아님 — 시험 안 함'))
    return 0 if ok else 1


def _state(pid):
    try:
        s = Path(f'/proc/{pid}/stat').read_text()
        return s[s.rfind(')') + 2:][:1]
    except OSError:
        return None


def listening(port):
    for f in ('/proc/net/tcp', '/proc/net/tcp6'):
        try:
            rows = Path(f).read_text().splitlines()[1:]
        except OSError:
            continue
        for r in rows:
            p = r.split()
            if len(p) > 3 and p[3] == '0A' and int(p[1].rsplit(':', 1)[1], 16) == port:
                return True
    return False


def procs():
    me = {os.getpid(), os.getppid()}
    found = []
    for d in os.listdir('/proc'):
        if not d.isdigit() or int(d) in me:
            continue
        try:
            cmd = Path(f'/proc/{d}/cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace').strip()
        except OSError:
            continue
        if cmd and any(p in cmd for p in PATS) and _state(d) not in (None, 'Z'):
            found.append(f'{d} {cmd[:170]}')
    ports = [p for p in PORTS if listening(p)]
    for f in found:
        print('  남음:', f)
    print(f'프로세스: {"없음" if not found else f"{len(found)} 개 남음"} · 듣는 GUI 포트: {ports or "없음"}')
    return 0 if not found and not ports else 1


def _load(p):
    try:
        return json.loads(Path(p).read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return None


def summary(out):
    out = Path(out)
    rows = []

    def row(item, ok, detail):
        rows.append((item, ok, detail))
    for part in ('A', 'B'):
        pj = _load(out / f'part_{part}.json')
        rec = Path((out / f'rec_{part}.txt').read_text().strip()) if (out / f'rec_{part}.txt').exists() else None
        if pj is None:
            row(f'{part} 부분 결과', False, '없음 (시험이 안 돌았거나 중간에 끝남)')
            continue
        row(f'{part} 서버 준비·끝 코드', pj.get('server_exit') == 0, f"준비 {pj.get('ready_s')} s · 끝 코드 {pj.get('server_exit')}")
        for s in pj.get('stall', []):
            row(s['tag'] + ' 숫자', s['release_s'] <= 0.31 and (s['robot_stop_s'] or 9) <= 1.0,
                f"jog_release {s['release_s'] * 1000:.0f} ms · 마지막 jog_hold {s['last_hold_s'] * 1000:+.0f} ms · 로봇 섬 {s['robot_stop_s']} s")
        if 'load' in pj:
            L = pj['load']
            row('⑤ 많은 지도 칸 · 3D', L['gl'] and (L['gui_gap_ms'] or {}).get('max', 1e9) < 150,
                f"칸 {L['n_vox']} · GUI 20 ms 타이머 간격 {L['gui_gap_ms']} · paintGL ms {L['paint_ms']} · refresh ms {L['refresh_ms']} · "
                f"{L['frames_per_s']} 장/s")
        wc = _load(rec / 'wmx_check.json') if rec else None
        if wc is not None:
            r2 = wc.get('r2_errors_before_quit') or []
            cnt = wc.get('r2_counts') or {}
            row(f'{part} R2 오류 0 (wmx_check)', not r2 and not any(cnt.values()), f'오류 줄 {len(r2)} · {cnt}')
            row(f'{part} 안전: Module.ini·엔진 시뮬레이터 (wmx_check)', bool((wc.get('safety') or {}).get('module_ini_simu'))
                and bool((wc.get('safety') or {}).get('engine_simu_only')), wc.get('safety'))
            for k in wc.get('shaping') or []:
                row(f"{part} ③ 성형 확인 {k['label']} (명령 − 컨볼루션)", k['max_mrad'] is None or k['max_mrad'] <= SHAPING_MAX_MRAD,
                    f"움직인 {k['moving_s']} s · 중앙 {k['median_mrad']} · 최대 {k['max_mrad']} mrad · 지연 {k['delay_ms']} ms")
            rt = wc.get('realtime') or {}
            row(f'{part} ④ 실시간 (wmx_check, 참고)', True, f"주기 {rt.get('cycles')} · 늦은 메시지 {rt.get('late_events')} · 주기 늦음 "
                f"{rt.get('catchup_events')} · 여유 {rt.get('lead_ms')} · 계산 {rt.get('cyc_ms')}")
            row(f'{part} 엔진 1 ms 가속 최대 ≤ R2 한계 (wmx_check)', max(wc.get('engine_acc_max') or [0]) <= (wc.get('engine_acc_limit') or 15),
                f"{wc.get('engine_acc_max')} ≤ {wc.get('engine_acc_limit')}")
        elif rec is not None:
            row(f'{part} wmx_check', False, '결과 없음')
        sm = _load(rec / 'summary.json') if rec else None
        if sm is not None:
            c = sm.get('robot_contacts') or {}
            row(f'{part} ④ 닿음·늦은 메시지 (rt_report, 참고)', True,
                f"닿음 셈 {c.get('counted')} (로봇 탓 {c.get('robot')} · 성형 탓 {c.get('shaping')} · 물체가 와서 {c.get('object_came')} · 카메라 못 봄 "
                f"{c.get('camera_blind')}) · 안 보낸 메시지 {sm.get('late_sends')} · 늦은 주기 {sm.get('late_cycles')} · 다리 {sm.get('legs_ok')}")
    print('\n## 통합 시험 요약')
    for item, ok, detail in rows:
        print(f"{'PASS' if ok else 'FAIL'}  {item} — {detail}")
    fails = sum(1 for _, ok, _ in rows if not ok)
    print(f'요약 FAIL {fails} 개')
    return 1 if fails else 0


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else ''
    if cmd == 'ini':
        sys.exit(ini())
    if cmd == 'procs':
        sys.exit(procs())
    if cmd == 'summary' and len(sys.argv) > 2:
        sys.exit(summary(sys.argv[2]))
    sys.exit(__doc__)
