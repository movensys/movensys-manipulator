"""실행 기록 읽기 (한곳) — rt_run.py·GUI 서버 (cr3a_gui/rt_server.py) 가 남긴 결과 폴더. numpy 만 (+ rt_shaping.shape_fir).

[2026-10-27 R-1 6 단계] 보고서·분석 도구 (tools/rt_report·rt_tools·motion_quality, cr3a_gui/tools/wmx_check) 가 따로 갖던 읽기 코드를
모음: 실행된 u 풀기 (5 벌), 전달 지연 맞춤 (2 벌), R2 로그 세기 (2 벌), WMX 축 로그. GPU·cuRobo 없이 불러짐 — wmx_check 는 이것만 씀.
엔진은 이 모듈을 안 씀 (쓰는 쪽 = rt_run.Controller.save·save_u_exec, GUI 서버 save_all). 저장소 밖에서는 PYTHONPATH=<cr3a_rt_shaping>.

결과 폴더
  rt_run.json    기록: settings · cycles (MPC 주기) · frames (카메라 영상) · events · legs · newvox (지도 칸) · sent_traj (보낸 명령 점)
  u_exec.npz     실행된 성형 전 명령 u (1 ms, 세그먼트 = 다시 시작 사이): T0 · n · before · q (이어 붙인 칸) · grid_s · h (성형기 1 ms 칸)
  wmx_axis.csv   (WMX 실행) 엔진 축 로그 1 ms: SYSTEM_TIME (유닉스 시각, 100 ns) · TIME (µs) · CMDPOS · FBPOS · CMDVEL (관절 0~5)
  r2_launch.log  (WMX 실행) R2 출력
GUI 서버 기록에만 있는 것 (2026-10-26/27, cr3a_gui/gui_controller.py save_all·make_controller) — 보고서가 꼭 따라야 함 (R-1 안전 ⑮)
  u_exec_k<i>.npz       바꾸기 전 성형기 i 의 실행된 u (같은 형식). rec['kernels'] = [{'label', 'until' (바꾼 시각, 지금 것은 None), 'file'
                        (기록을 줄여 없으면 None)}] — 성형기마다 Stream 을 새로 만들어 sent_traj 의 'seg' 번호도 성형기마다 0 부터 (kernels)
  estop_windows         비상정지 감속 [(스탬프 S, 선 시각)]: 성형을 건너뛰고 한계 안에서 바로 감속 — 실행된 u 는 S 에서 끊기고 그 사이 명령은
                        sent_traj 의 감속 점뿐 (estop_windows·estop_segments)
  settings.camera       'real' = 실제 고정 카메라: 기록의 기둥·시나리오 물체는 가상이라 참값이 아님 (has_truth)
  settings.config.map.table_z   테이블 윗면 (카메라 보정) → 충돌 바닥 = max(table_z, 0) (floor_z, 엔진 rt_world.World.floor_z 와 같은 규칙)
"""
import csv
import json
import re
from collections import namedtuple
from pathlib import Path

import numpy as np

from rt_shaping import shape_fir

JOINTS = [f'joint{i}' for i in range(1, 7)]
# R2 로그에서 세는 줄 (보고서 summary.json r2_log·wmx_check r2_counts)
R2_PATTERNS = {'max_acc_error': r'MaxAccError', 'push_rejected': r'Cyclic push rejected', 'faulted': r'Cyclic buffer faulted',
               'stalled': r'stalled'}


def load_run(d) -> dict:
    """결과 폴더의 rt_run.json."""
    return json.loads((Path(d) / 'rt_run.json').read_text())


def read_text(path) -> str:
    """로그 파일 글 (없으면 빈 글)."""
    p = Path(path)
    return p.read_text(errors='ignore') if p.exists() else ''


def read_wmx_csv(path) -> dict:
    """WMX 축 로그 → {'unix' 유닉스 시각 s, 'tus' 처음부터 s, 'cmd'·'fb'·'cmdvel' [n, 6]}. 줄 길이가 머리와 다른 줄 (끊긴 끝 줄) 은 뺌."""
    with open(path) as f:
        rd = csv.reader(f)
        head = next(rd)
        rows = [row for row in rd if len(row) == len(head)]
    a = np.array(rows, dtype=float)
    col = {h: i for i, h in enumerate(head)}
    g = lambda p: np.stack([a[:, col[f'{p}{j}']] for j in range(6)], axis=1)  # noqa: E731
    return {'unix': a[:, col['SYSTEM_TIME']] * 1e-7, 'tus': (a[:, col['TIME']] - a[0, col['TIME']]) * 1e-6,
            'cmd': g('CMDPOS'), 'fb': g('FBPOS'), 'cmdvel': g('CMDVEL')}


def fit_delay(err) -> float:
    """전달 지연 맞춤: err(L) 이 가장 작은 L (s) 을 0.5 ms 격자 (0 ~ 80 ms) → 0.1 ms → 0.01 ms 로. [2026-10-26] 0.5 ms 격자만 쓰면 격자 오차 ×
    관절 속도만큼 차이가 남음 (배율 1.0 회피 시험: 2.4 rad/s × 0.26 ms → 0.63 mrad 를 성형 차이로 보고했음. 0.01 ms 로 0.07 mrad)."""
    L = min(np.arange(0.0, 0.0801, 0.0005), key=err)
    L = min(np.arange(L - 0.0005, L + 0.00051, 0.0001), key=err)
    return float(min(np.arange(L - 0.0001, L + 0.000101, 0.00001), key=err))


def r2_counts(text: str) -> dict:
    """R2 로그의 결함 줄 수 {이름: 수} (R2_PATTERNS)."""
    return {k: len(re.findall(p, text)) for k, p in R2_PATTERNS.items()}


def r2_max_acc(text: str):
    """R2 로그에 찍힌 CyclicBuffer maxAcc (엔진이 보는 1 ms 가속 한계, rad/s²). 없으면 None."""
    m = re.search(r'maxAcc ([\d.]+)', text)
    return float(m.group(1)) if m else None


# ---------------- 장면 ----------------
def scene(st) -> list:
    """기록의 장면 → [(이름, 크기, 경유점 또는 None, 정지 중심 또는 None)]: 기둥 (정지) + 시나리오 물체 (경유점, 시각 = t − t_go)."""
    return [(st['pillar'][0], st['pillar'][2], None, st['pillar'][1])] + \
           [(o['name'], o['dims'], o['wps'], None) for o in st['obstacles']]


def moving_on(t, t_go, t_end=None) -> bool:
    """움직이는 시험 물체가 시각 t 에 장면에 있나: GO 뒤 ~ 치우기 전 (t_end = GUI 실행에서 물체를 치운 시각, [2026-10-26])."""
    return t_go is not None and t >= t_go and (t_end is None or t < t_end)


# ---------------- 실행된 u (u_exec.npz) ----------------
def u_exec_segments(path):
    """u_exec*.npz → ([(T0, before, q [n, J])] 세그먼트마다, h, grid_s)."""
    z = np.load(path)
    q = z['q']
    out, o = [], 0
    for T0, n, b in zip(z['T0'], z['n'], z['before']):
        out.append((T0, b, q[o:o + int(n)]))
        o += int(n)
    return out, z['h'], float(z['grid_s'])


# ---------------- GUI 서버 기록: 성형기 여러 개 · 비상정지 감속 · 실제 카메라 · 테이블 높이 ----------------
def floor_z(rec) -> float:
    """충돌 바닥 높이 (m, 받침 원점 기준) = max(테이블 윗면, 0) — 엔진 rt_world.World.floor_z 와 같은 규칙 (계획기·MPC·안전 검사가 쓴 바닥).
    테이블 윗면 = 기록 settings.config.map.table_z (GUI 서버가 실제 카메라일 때 카메라 보정 값을 넣음). 없으면 (옛 기록·시뮬레이션) 0."""
    tz = ((((rec.get('settings') or {}).get('config') or {}).get('map') or {}).get('table_z')) or 0.0
    return max(float(tz), 0.0)


def has_truth(rec) -> bool:
    """기록의 기둥·시나리오 물체 (settings.pillar·obstacles) 가 실제 장면인가. 가상 카메라 (rt_run·GUI --camera virtual) 는 그 상자들을 렌더해
    지도를 쌓으므로 참값. 실제 카메라 (GUI --camera real → settings.camera 'real') 는 실제 장면을 보므로 기록의 상자는 참값이 아님 → 물체
    거리·닿음은 셀 수 없음 (참값은 바닥 = 테이블 높이뿐)."""
    return (rec.get('settings') or {}).get('camera', 'virtual') != 'real'


def is_gui(rec) -> bool:
    """GUI 서버 기록인가 (save_all 이 rec['gui'] 를 넣음) — 목표는 사용자가 고름 (A·B 고정 아님)."""
    return 'gui' in rec


def kernels(rec) -> list:
    """성형기마다 실행된 u 파일과 그 성형기를 쓴 시각 범위 [{'label', 'file', 'until', 't0', 't1'}] (쓴 차례). t0 ~ t1 = 앞 성형기를 바꾼
    시각 ~ 이 성형기를 바꾼 시각 (처음 −∞, 지금 것 +∞). rt_run 기록은 성형기 하나 = u_exec.npz."""
    ks = rec.get('kernels') or [{'label': ((rec.get('settings') or {}).get('shaping') or {}).get('label'), 'until': None, 'file': 'u_exec.npz'}]
    out, t0 = [], -np.inf
    for k in ks:
        t1 = np.inf if k.get('until') is None else float(k['until'])
        out.append({'label': k.get('label'), 'file': k.get('file'), 'until': k.get('until'), 't0': t0, 't1': t1})
        t0 = t1
    return out


def u_exec_runs(d, rec) -> list:
    """성형기마다 [(kernels 의 항목, 세그먼트들 [(T0, before, q)], h, grid_s)] — 파일이 있는 것만 (쓴 차례)."""
    out = []
    for k in kernels(rec):
        f = k['file']
        if f and (Path(d) / f).exists():
            out.append((k,) + u_exec_segments(Path(d) / f))
    return out


def estop_windows(rec) -> list:
    """(GUI 서버) 비상정지 감속 구간 [(스탬프 S, 선 시각)] — 성형을 건너뛰고 한계 안에서 바로 감속한 명령."""
    return [(float(a), float(b)) for a, b in (rec.get('estop_windows') or [])]


def outside_windows(t, windows) -> np.ndarray:
    """시각 t [n] 중 구간들 [(a, b)] 밖인 것 (bool [n])."""
    t = np.asarray(t, float)
    m = np.ones(len(t), bool)
    for a, b in windows:
        m &= ~((t >= a) & (t <= b))
    return m


# 실행된 명령 한 세그먼트 (1 ms): 시각 t [n], 성형 전 u [n, J], 성형된 y [n, J] (= WMX 로 간 명령), 성형기 h·grid_s·이름,
#  estop = 비상정지 감속 구간 (u = y = 보낸 감속 점, 성형 없음)
Seg = namedtuple('Seg', 't u y h g label estop')


def estop_segments(rec, grid_s=0.001) -> list:
    """비상정지 감속 구간의 명령 = 보낸 감속 점 (sent_traj, 5 ms + 마지막 점) 을 grid_s 로 이은 것 → [Seg (u = y, h = [1])]. 실행된 u 에
    없는 구간이라 닿음 판정·보고서 3D 의 u 가 이 동안을 건너뛰지 않게."""
    st, W, out = rec.get('sent_traj'), estop_windows(rec), []
    if not st or not W:
        return out
    t, q = np.asarray(st['t'], float), np.asarray(st['q'], float)
    o = np.argsort(t, kind='stable')
    t, q = t[o], q[o]
    for a, b in W:
        m = (t >= a - 1e-6) & (t <= b + 1e-6)
        if m.sum() < 2:
            continue
        ta, qa = t[m], q[m]
        keep = np.r_[True, np.diff(ta) > 1e-9]
        ta, qa = ta[keep], qa[keep]
        tt = ta[0] + np.arange(int(np.floor((ta[-1] - ta[0]) / grid_s + 1e-9)) + 1) * grid_s
        qq = np.stack([np.interp(tt, ta, qa[:, j]) for j in range(qa.shape[1])], 1)
        out.append(Seg(tt, qq, qq.copy(), np.array([1.0]), grid_s, '비상정지 감속 (성형 없음)', True))
    return out


def executed_uy(d, rec=None, estop=True) -> list:
    """실행된 명령 [Seg] (1 ms) — 성형기마다 (kernels) 그 파일의 세그먼트를 그 성형기 h 로 한 번에 컨볼루션 (y = rt_shaping.shape_fir).
    이상적 모드는 다시 계획하는 동안 시계를 건너뛰어 앞 세그먼트 꼬리와 다음 세그먼트 시각이 겹칠 수 있음 → 다음 세그먼트 시작 전까지만
    (성형기 파일 차례 = 시간 차례). estop = 비상정지 감속 구간도 (estop_segments, 맨 뒤에)."""
    rec = load_run(d) if rec is None else rec
    runs = [(T0, b, q, np.asarray(h, float), g, k['label']) for k, segs, h, g in u_exec_runs(d, rec) for T0, b, q in segs]
    T0s = [float(r[0]) for r in runs]
    out = []
    for i, (_, b, q, h, g, lab) in enumerate(runs):
        q = np.asarray(q, float)
        y = shape_fir(q, h, before=b)
        t = T0s[i] + np.arange(len(q)) * g
        m = t < (T0s[i + 1] if i + 1 < len(T0s) else np.inf) - 1e-9
        out.append(Seg(t[m], q[m], y[m], h, g, lab, False))
    if estop:
        out += estop_segments(rec, out[0].g if out else 0.001)
    return out


def u_series(d, rec):
    """보고서 3D 의 성형 전 명령 u: (시각 [n], u [n, J]) 시각 차례 — 모든 성형기의 실행된 u (자르지 않음) + 비상정지 감속 구간 (보낸 감속 점).
    실행된 u 가 없으면 None."""
    runs = u_exec_runs(d, rec)
    if not runs:
        return None
    ts = [float(T0) + np.arange(len(q)) * g for _, segs, _, g in runs for T0, _, q in segs]
    qs = [q for _, segs, _, _ in runs for _, _, q in segs]
    for s in estop_segments(rec, runs[-1][3]):
        ts.append(s.t)
        qs.append(s.u)
    t, q = np.concatenate(ts), np.concatenate(qs)
    o = np.argsort(t, kind='stable')
    return t[o], q[o]
