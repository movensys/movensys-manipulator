"""움직이는 물체 시나리오. 물체 = (이름, 크기 xyz (m), 경유점 [(시각 s, x, y, z)]) — 시각은 GO(물체 출발) 기준,
경유점 사이는 등속 직선, 마지막 경유점 뒤에는 그 자리. 처음 자리는 지도 밖 (x 0.95 또는 y ±0.9 또는 z 0.98) 이라 배경에 안 들어감.
좌표는 로봇 받침 기준. 참고: A·B 손끝 (0.42, ±0.34, 0.20), 기둥 x 0.37~0.47 · |y| < 0.05 · z < 0.30, 바닥 z = 0.

  moving5    — [2026-10-13 사용자 요청] 5개, 3D 대각선 경유점 고리를 실행 끝까지 반복 (A↔B 사이 팔이 지나가는 공간을 가로지름,
               목표 자세는 막지 않음: 사전 점검 A·B 자세 로봇과 최소 79 mm)
  moving3t   — [2026-10-16 사용자 결정] 끝단 공구 모델용 3개 (moving5 에서 손3·손5 뺌, 상자2·상자4 경로를 목표 자세에서 8.5 cm 밖으로)
  none       — 물체 없음
  goal_*     — 도착 자리 막기 (아래 함수 설명, --legs 4)
  hold_*     — [2026-10-16] 지키기 시험: A 에서 지키는 로봇에 물체가 천천히 다가와 머묾 (--legs 0 --set task.end_hold_s=33, HOLD_SIDE 설명)
새 시나리오는 SCENARIOS 에 함수로 더한다. 기둥·바닥 겹침, 목표·이동 경로와의 거리는 `python3 -m tools.rt_tools scenario <이름>` 으로 미리 본다.
"""
import numpy as np


def obstacle_pose(wps, t):
    """경유점 [(시각, x, y, z)] 의 시각 t 위치."""
    if t <= wps[0][0]:
        return list(wps[0][1:])
    for (t0, *p0), (t1, *p1) in zip(wps, wps[1:]):
        if t <= t1:
            a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            return [p0[k] + a * (p1[k] - p0[k]) for k in range(3)]
    return list(wps[-1][1:])


def loop(p0, entry_v, pts, speeds, wait=0.3, until=150.0, delay=0.0, scale=1.0):
    """p0 에서 entry_v 로 들어와 경유점 고리 pts 를 until 초까지 반복 (구간마다 speeds[k], 경유점마다 wait 초 멈춤).
    delay = 들어오기 전 p0 (지도 밖) 에서 기다리는 시간, scale = 속도 배율 (시나리오 변형용)."""
    t, out, cur, first = 0.0, [(0.0, *p0)], np.array(p0, float), True
    if delay > 0:
        t = delay
        out.append((t, *p0))
    entry_v, speeds = entry_v * scale, [s * scale for s in speeds]
    while t < until:
        for k, tgt in enumerate(pts):
            tgt = np.array(tgt, float)
            sp = entry_v if first else speeds[k % len(speeds)]
            first = False
            t += float(np.linalg.norm(tgt - cur)) / sp
            out.append((t, *tgt))
            if wait > 0:
                t += wait
                out.append((t, *tgt))
            cur = tgt
    return out


def moving5(seed=None):
    """seed 없음 = 2026-10-13 시나리오 그대로. seed 가 있으면 물체마다 들어오는 시각 (0~6 s 늦춤) 과 속도 (×0.8~1.2) 를 바꾼 변형
    (같은 seed = 같은 변형) — 한 번 실행으로는 우연히 마주치는 상황에 따라 결과가 크게 달라서 여러 변형으로 비교하려고."""
    rng = None if seed is None else np.random.default_rng(int(seed))

    def loop_(*a, **kw):
        if rng is not None:
            kw.update(delay=float(rng.uniform(0.0, 6.0)), scale=float(rng.uniform(0.8, 1.2)))
        return loop(*a, **kw)
    return [
        ('손1', [0.08, 0.08, 0.08], loop_([0.95, 0.30, 0.60], 0.10, [[0.58, 0.22, 0.48], [0.45, -0.10, 0.62], [0.62, -0.32, 0.42],
                                                                 [0.52, 0.05, 0.36]], [0.08, 0.10, 0.07, 0.09])),
        ('상자2', [0.12, 0.12, 0.12], loop_([0.15, -0.90, 0.55], 0.08, [[0.18, -0.55, 0.52], [0.50, -0.52, 0.62], [0.42, -0.12, 0.50],
                                                                  [0.12, -0.38, 0.56]], [0.07, 0.06, 0.08, 0.07])),
        ('손3', [0.06, 0.06, 0.06], loop_([0.30, 0.20, 0.98], 0.08, [[0.30, 0.20, 0.58], [0.48, 0.02, 0.66], [0.28, -0.20, 0.55],
                                                                 [0.40, 0.10, 0.45]], [0.06, 0.07, 0.06, 0.08])),
        ('상자4', [0.10, 0.10, 0.10], loop_([0.35, 0.90, 0.55], 0.08, [[0.30, 0.56, 0.52], [0.56, 0.46, 0.60], [0.42, 0.13, 0.50],
                                                                  [0.16, 0.44, 0.56]], [0.08, 0.06, 0.07, 0.08])),
        ('손5', [0.07, 0.07, 0.07], loop_([0.95, -0.15, 0.25], 0.12, [[0.76, -0.05, 0.28], [0.70, 0.32, 0.36], [0.76, -0.30, 0.30],
                                                                 [0.60, 0.00, 0.50]], [0.12, 0.10, 0.12, 0.09])),
    ]


def moving3t(seed=None):
    """[2026-10-16 사용자 결정 "경로도 고치고, 장애물도 2개 줄이자"] 끝단 공구 모델 ('tool') 용. 공구를 붙이면 A·B 에서 팔이 높아져
    moving5 그대로는 목표 자세를 막음 (상자2 ↔ B −55 mm, 상자4 ↔ A −49 mm, 손3 ↔ A·B −2·−16 mm; rt_tools scenario).
    손3·손5 를 빼고, 상자2·상자4 의 고리 경유점을 A·B 자세 로봇과 8.5 cm 이상 떨어지게 옮김 (archive/robot_tool/make_moving3t.py: 가장 가까운
    표본의 구간 양 끝 경유점을 로봇 → 상자 방향으로 밀기, 옮긴 거리 상자2 2.5·5.8·24.2·14.8 cm, 상자4 2.1·9.3·21.3·16.3 cm — 대부분 위로).
    손1 은 그대로 (A 78 / B 72 mm). 들어오는 점·속도·멈춤은 moving5 와 같음. seed 변형도 moving5 와 같은 방식."""
    rng = None if seed is None else np.random.default_rng(int(seed))

    def loop_(*a, **kw):
        if rng is not None:
            kw.update(delay=float(rng.uniform(0.0, 6.0)), scale=float(rng.uniform(0.8, 1.2)))
        return loop(*a, **kw)
    return [
        ('손1', [0.08, 0.08, 0.08], loop_([0.95, 0.30, 0.60], 0.10, [[0.58, 0.22, 0.48], [0.45, -0.10, 0.62], [0.62, -0.32, 0.42],
                                                                 [0.52, 0.05, 0.36]], [0.08, 0.10, 0.07, 0.09])),
        ('상자2', [0.12, 0.12, 0.12], loop_([0.15, -0.90, 0.55], 0.08, [[0.175, -0.569, 0.536], [0.547, -0.519, 0.654], [0.526, 0.027, 0.66],
                                                                  [0.055, -0.30, 0.665]], [0.07, 0.06, 0.08, 0.07])),
        ('상자4', [0.10, 0.10, 0.10], loop_([0.35, 0.90, 0.55], 0.08, [[0.305, 0.576, 0.533], [0.622, 0.413, 0.651], [0.398, -0.021, 0.649],
                                                                  [0.083, 0.389, 0.694]], [0.08, 0.06, 0.07, 0.08])),
    ]


A_TOOL, B_TOOL = (0.42, 0.34, 0.20), (0.42, -0.34, 0.20)     # A·B 자세의 손끝 (rt_config.TaskCfg.qa·qb; 공구 모델은 그리퍼 끝 = 같은 자리)


def visit(p_out, p_in, t_in, t_out=None, speed=0.3):
    """지도 밖 p_out 에서 직선으로 들어와 시각 t_in 에 p_in 에 닿고, t_out 에 같은 길로 나감 (None = 끝까지 머묾)."""
    d = float(np.linalg.norm(np.subtract(p_in, p_out)))
    w = [(0.0, *p_out), (max(t_in - d / speed, 0.0), *p_out), (t_in, *p_in)]
    if t_out is not None:
        w += [(t_out, *p_in), (t_out + d / speed, *p_out)]
    return w


# 도착 자리 막기 [2026-10-13 사용자 요청 "도착 지점에 장애물이 있으면 어떻게 행동하나"]. 다리 4 개로 돌림 (--legs 4):
#   GO 직후 1. A→B (~2.5 s 에 B 도착) → 1 s 쉼 → 2. B→A (~3.5 s 에 계획) → 3. A→B → 4. B→A
def goal_wait():
    """로봇이 B 에 있을 때 상자가 A 손끝 자리를 차지 (2.6 s) → 8 s 뒤 나감: 2. B→A 의 계획이 막혔다가 풀리는 경우."""
    return [('상자A', [0.10, 0.10, 0.10], visit((0.95, 0.34, 0.20), A_TOOL, 2.6, 10.6))]


def goal_stay():
    """상자가 A 손끝 자리를 차지하고 안 나감: 2. B→A·4. B→A 가 끝내 못 가는 경우."""
    return [('상자A', [0.10, 0.10, 0.10], visit((0.95, 0.34, 0.20), A_TOOL, 2.6))]


def goal_arrive():
    """로봇이 B 로 가는 중에 상자가 B 손끝 자리로 들어옴 (1.5 s) → 6 s 뒤 나감: 움직이는 중에 목표가 막히는 경우."""
    return [('상자B', [0.10, 0.10, 0.10], visit((0.95, -0.34, 0.20), B_TOOL, 1.5, 7.5))]


def goal_near():
    """A 손끝 바로 아래 바닥에 상자 (윗면 z 0.12 = 손끝 8 cm 아래, 닿지는 않음) 를 두고 안 치움: 목표 자세가 반응 거리 안인 경우."""
    return [('상자A', [0.12, 0.12, 0.12], visit((0.95, 0.34, 0.06), (0.42, 0.34, 0.06), 2.6))]


def come(p_out, p_in, t_in, stay, v=0.08):
    """지도 밖 p_out 에서 속도 v 로 들어와 t_in 에 p_in, stay 초 머문 뒤 같은 길로 나감."""
    d = float(np.linalg.norm(np.subtract(p_in, p_out)))
    t0 = max(t_in - d / v, 0.0)
    return [(0.0, *p_out), (t0, *p_out), (t_in, *p_in), (t_in + stay, *p_in), (t_in + stay + d / v, *p_out)]


# [2026-10-16 사용자 "가만히 있을 때 장애물이 다가오더라도 … 슬금슬금 움직여서 피해야" → 평가에 넣음 (작업기록 5-21)] 지키기 시험:
#  로봇은 A 에 도착해 그 자리를 지키고 (--legs 0, 끝 지키기 task.end_hold_s = 마지막 경유 + 2 s), 물체가 하나씩 A 자세 로봇을 향해 8 cm/s 로
#  들어와 (그대로 서 있으면 닿는 자리: 공구 모델 A 자세와 손S −27 mm, 상자T −39 mm — archive/dev/hold_approach.py check) 3 s 머물다 나감.
#  손S = +x 쪽에서 그리퍼로, 상자T = 위에서 손목으로. 돌림: archive/dev/hold_approach.py run (이상적) / RT_ARGS 로 tools/rt_demo.sh (WMX)
HOLD_SIDE = ('손S', [0.08, 0.08, 0.08], come((0.95, 0.34, 0.30), (0.445, 0.34, 0.30), 8.0, 3.0))
HOLD_TOP = ('상자T', [0.10, 0.10, 0.10], come((0.42, 0.30, 0.98), (0.42, 0.30, 0.50), 22.0, 3.0))


def get_scenario(name):
    """이름 → 물체 목록. '이름:숫자' 는 변형 (moving5:3 = seed 3)."""
    base, _, seed = name.partition(':')
    if base not in SCENARIOS:
        raise KeyError(f'시나리오 {name}: {sorted(SCENARIOS)} (moving5 는 moving5:숫자 변형도)')
    return SCENARIOS[base](int(seed)) if seed else SCENARIOS[base]()


SCENARIOS = {
    'moving5': moving5,
    'moving3t': moving3t,
    'none': lambda: [],
    'goal_wait': goal_wait,
    'goal_stay': goal_stay,
    'goal_arrive': goal_arrive,
    'goal_near': goal_near,
    'hold_side': lambda: [HOLD_SIDE],
    'hold_top': lambda: [HOLD_TOP],
    'hold_both': lambda: [HOLD_SIDE, HOLD_TOP],
}
