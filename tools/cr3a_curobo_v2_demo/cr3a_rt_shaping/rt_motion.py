"""로봇 명령 궤적을 만드는 순수 계산 (numpy, GPU·ROS 없음). 관절마다 독립, 모두 속도·가속·저크 한계 안.

  track_target  — 고정 목표점(MPC 상태)을 한계 안에서 쫓는 궤적 (Mirobot CommandLimiter 와 같은 생각)
  clamp_final   — 마지막 접근에서 목표점이 목표를 넘지 않게
  brake         — 지금 상태에서 한계 안에서 가장 빨리 멈추는 궤적 (안전 검사의 정지)
  braking_tail  — 메시지 끝에 붙이는 멈춤 꼬리 (다음 메시지가 안 오면 JTC 가 이걸 따라 멈춤)

왜 저크 한계인가 (2026-10-13 WMX 실측): 가속을 계단으로 바꾸면 R2 의 1 ms 변환이 계단을 키워 CyclicBuffer maxAcc 에
걸렸다 (감속 −7.78 → 0 계단이 10.4 로). 저크 500 rad/s³ 는 그 계산에서 고른 값 (rt_config.MotionCfg 설명).
"""
import math

import numpy as np


def acc_bounds(v, vmax, amax, jmax):
    """속도 한계에 저크 한계로 부드럽게 닿는 가속 범위: v 가 vmax 에 가까우면 가속 상한이 sqrt(2 j (vmax − v)) 로 줄어듦."""
    hi = np.minimum(amax, np.sqrt(2.0 * jmax * np.maximum(vmax - v, 0.0)))
    lo = -np.minimum(amax, np.sqrt(2.0 * jmax * np.maximum(vmax + v, 0.0)))
    return lo, hi


def stop_speed(d, a, j):
    """거리 d 안에 멈출 수 있는 가장 큰 속도 (가속 0 에서 시작, 감속을 저크 j 로 a 까지 키웠다가 다시 0 으로):
    멈춤 거리 = v²/(2a) + v·a/(2j) 를 v 로 푼 것. d ≤ a³/j² 면 감속이 a 까지 못 가는 삼각형: d = v^1.5 / √j."""
    d0 = a ** 3 / j ** 2
    return np.where(d <= d0, np.cbrt(d * d * j), -a * a / (2.0 * j) + np.sqrt(a ** 4 / (4.0 * j * j) + 2.0 * a * d))


WALL_DECEL = 0.8       # 관절 위치 한계 앞에서 멈출 때 계획하는 감속 비율 (아래 track_target 설명: 0.8 이면 고정 목표 지나침 0.05° 이하)
WALL_INSET = 1e-3      # 위치 한계에서 이만큼 (rad, 0.06°) 안쪽을 벽으로: 저크 한계 추종기의 남는 지나침 (≤ 0.05°) 몫


def stop_dist(v, a, j):
    """속도 v 에서 감속 a·저크 j 로 멈추는 거리 (stop_speed 의 역, 가속 0 에서 시작)."""
    v = np.abs(v)
    return np.where(v <= a * a / j, v * np.sqrt(v / j), v * v / (2.0 * a) + v * a / (2.0 * j))


def track_target(q0, v0, a0, target, n, h, vmax, amax, jmax, cycle, sub=5, stop_decel=1.0, qlim=None, fine=False):
    """고정 목표점을 쫓는 궤적의 n 점 (간격 h, 첫 점 = h 뒤) 의 (q, v, a) [n, 관절].
    원하는 속도 = 남은 거리를 한 주기(cycle)에 가는 속도를, 그 거리에서 저크 한계로 멈출 수 있는 속도 (stop_speed) 와 vmax 로
    자름 → 원하는 가속 = (원하는 속도 − 지금 속도) / (cycle/4) 를 가속·속도·저크 범위로 자름. 점 사이 sub 단계로 적분.
    stop_decel < 1 = 멈춤 속도를 감속 한계 × stop_decel 로 계산 (감속 여유: 위 1차 추종의 늦음을 메워 목표를 거의 안 지나침).
    qlim = (아래, 위) 관절 위치 한계: 목표점을 그 안으로 자르고, 한계까지 남은 거리에서 멈출 수 있는 속도 이하로만 움직임 (벽).
    [2026-10-14] MPC 는 위치 한계를 비용으로만 다뤄 당근이 관절 3 한계 (±154.9°) 를 111° 까지 넘었고 명령도 따라가 23° 넘었다
    (이상적 모드 변형 시나리오, 팔이 아래로 꺾인 자세 → 구 모델이 메시를 못 덮어 바닥 −31 mm).
    [2026-10-13] 멈출 수 있는 속도를 예전엔 √(2·a·e) (저크 무시) 로 잡아 감속으로 바꾸는 저크 시간만큼 늦게 서서 목표를 지나쳤다.
    fine = True 면 적분 단계마다 (h/sub, 1 ms) 의 (q, v, a) [n·sub, 관절] 도 돌려줌 (입력성형용, 값은 같은 계산).
    [2026-10-15 빠르게] 관절마다 파이썬 실수로 계산 (관절이 서로 독립). 관절 6 개짜리 numpy 배열은 계산보다 부르는 비용이 커서
    주기마다 4.5 ms → 1.1 ms (Thor). 값은 예전 numpy 계산과 비트까지 같음 (tests/test_motion.py 의 _track_np 와 비교): 식의 순서를
    그대로 두고, amax (cuRobo float32) 에 걸린 상수는 numpy 로 같은 식·같은 형 (numpy 1.26: 파이썬 실수 × float32 배열 = float32) 으로 미리 계산."""
    q, v, a = np.array(q0, float), np.array(v0, float), np.array(a0, float)
    tgt = np.asarray(target, float)
    J = q.size
    if qlim is not None:
        plo, phi = np.asarray(qlim[0], float) + WALL_INSET, np.asarray(qlim[1], float) - WALL_INSET
        tgt = np.clip(tgt, plo, phi)
        # 이번 n 점 동안 어느 관절도 벽에 못 닿으면 (지금 자리 ± 최고 속도로 n·h 동안 + 그 속도에서 멈추는 거리) 벽 계산을 건너뜀 (시간 절약:
        #  Thor 에서 주기마다 ~3.6 ms)
        reach = np.abs(v) * n * h + 0.5 * amax * (n * h) ** 2 + stop_dist(np.minimum(np.abs(v) + amax * n * h, vmax), WALL_DECEL * amax, jmax)
        if np.all((q - reach > plo) & (q + reach < phi)):
            qlim = None
    hh = h / sub
    den = max(cycle / 4.0, hh)
    jm = _per_joint(jmax, J)
    ks = _stop_consts(stop_decel * amax, jmax, J)                 # 멈춤 속도 (stop_speed) 상수 — 예전과 같은 인자 그대로
    kw = _stop_consts(WALL_DECEL * amax, jmax, J) if qlim is not None else None
    vm, am = _per_joint(vmax, J), _per_joint(amax, J)
    two_j, jh = 2.0 * jm, jm * hh
    Q, V, A = np.zeros((n, J)), np.zeros((n, J)), np.zeros((n, J))
    F = np.zeros((n * sub, 3, J)) if fine else None
    for i in range(J):
        qi, vi, ai, ti = float(q[i]), float(v[i]), float(a[i]), float(tgt[i])
        vmi, ami, ji, tji, jhi, ki = float(vm[i]), float(am[i]), float(jm[i]), float(two_j[i]), float(jh[i]), ks[i]
        if kw is not None:
            pli, phii, kwi = float(plo[i]), float(phi[i]), kw[i]
        m = 0
        for k in range(n):
            for _ in range(sub):
                e = ti - qi
                vs = _stop_speed1(abs(e), ki, ji)
                vd = _clip(_clip(e / cycle, -vs, vs), -vmi, vmi)
                if kw is not None:                                   # 위치 한계 = 벽 (한계 밖에 있으면 안쪽으로만)
                    dl, dh = qi - pli, phii - qi
                    vd = _clip(vd, -_stop_speed1(dl if dl > 0.0 else 0.0, kwi, ji), _stop_speed1(dh if dh > 0.0 else 0.0, kwi, ji))
                ad = _clip((vd - vi) / den, -ami, ami)
                x = vmi - vi                                         # acc_bounds
                hi = math.sqrt(tji * (x if x > 0.0 else 0.0))
                hi = ami if ami < hi else hi
                x = vmi + vi
                lo = math.sqrt(tji * (x if x > 0.0 else 0.0))
                lo = -(ami if ami < lo else lo)
                an = _clip(_clip(ad, lo, hi), ai - jhi, ai + jhi)
                qi = qi + vi * hh + 0.5 * an * hh * hh
                vi = _clip(vi + an * hh, -vmi, vmi)
                ai = an
                if fine:
                    F[m, 0, i], F[m, 1, i], F[m, 2, i] = qi, vi, ai
                m += 1
            Q[k, i], V[k, i], A[k, i] = qi, vi, ai
    if fine:
        return Q, V, A, F
    return Q, V, A


def _clip(x, lo, hi):
    """np.clip 의 실수 하나 판 (같은 비교: 작으면 lo, 크면 hi)."""
    x = x if x > lo else lo
    return x if x < hi else hi


def _per_joint(x, J):
    """관절별 값 (스칼라·배열 모두) → float64 [J] (float32 는 numpy 가 섞을 때와 같은 값으로)."""
    return np.broadcast_to(np.asarray(x), (J,)).astype(float)


def _stop_consts(a, j, J):
    """stop_speed(d, a, j) 에서 d 와 무관한 부분을 같은 식·같은 형으로 (a 가 float32 면 a·a, a³, a⁴ 도 float32 — numpy 판과 같은 값)
    → 관절마다 (d0, c1, c2, c3): v = cbrt(d·d·j) (d ≤ d0), 아니면 c1 + sqrt(c2 + c3·d)."""
    a, j = np.asarray(a), np.asarray(j)
    d0 = a ** 3 / j ** 2
    c1 = -a * a / (2.0 * j)
    c2 = a ** 4 / (4.0 * j * j)
    c3 = 2.0 * a
    return [tuple(float(np.broadcast_to(x, (J,))[i]) for x in (d0, c1, c2, c3)) for i in range(J)]


def _stop_speed1(d, k, j):
    """stop_speed 의 관절 하나 판 (k = _stop_consts 한 줄)."""
    return math.cbrt(d * d * j) if d <= k[0] else k[1] + math.sqrt(k[2] + k[3] * d)


def clamp_final(target, q, goal, eps=1e-4):
    """마지막 접근: 관절마다 당근(target)이 목표를 넘어가는 쪽이면 목표로, 지금 명령 q 가 이미 목표에 있으면 (eps rad 안) 목표 그대로.
    목표에서 물러나는 쪽은 그대로 둔다. 반환 (고친 목표점, 실제로 바꾼 관절이 있나)."""
    target, q, goal = (np.asarray(x, float) for x in (target, q, goal))
    gap = goal - q
    fix = (np.abs(gap) < eps) | ((target - goal) * np.sign(gap) > 0)
    return np.where(fix, goal, target), bool(np.any(fix & (target != goal)))


def brake(q, v, a, ts, amax, jmax, h=0.001):
    """상태 (q, v, a) 에서 가장 빨리 멈추는 궤적의 시각 ts (오름차순, > 0) 상태 (q, v, a) [n, 관절] 와 멈춘 시각 (못 멈추면 None).
    감속 −sign(v)·amax 를 저크 한계로 따라가다, 남은 속도가 a²/(2j) 이하가 되면 가속을 저크 한계로 0 까지 풂 (끝에서 가속도 0).
    시각은 정수 단계로 센다: h 씩 더해 비교하면 float32 점 간격(0.0250000004)과 어긋나 70 ms 부터 1 ms 늦게 찍혀
    위치가 한 점에서 튀었다 (2026-10-13 WMX 실측: 유한차분 가속 14 rad/s²)."""
    q, v, a = np.array(q, float), np.array(v, float), np.array(a, float)
    out, k, t_stop = [], 0, None
    for tt in ts:
        k_end = int(round(tt / h))
        while t_stop is None and k < k_end:
            release = (a * v < 0) & (np.abs(v) <= a * a / (2.0 * jmax) + jmax * h * h)   # 지금 가속을 0 으로 풀면 딱 멈춤
            at = np.where(release, 0.0, -np.sign(v) * amax)
            an = np.clip(at, a - jmax * h, a + jmax * h)
            q = q + v * h + 0.5 * an * h * h
            v = v + an * h
            a = an
            k += 1
            done = (np.abs(v) < 2.0 * jmax * h * h) & (np.abs(a) <= 2.0 * jmax * h)   # 멈춤 (저크 두 단계 안: 이산화 떨림 크기)
            v, a = np.where(done, 0.0, v), np.where(done, 0.0, a)
            if np.all(done):
                t_stop = k * h
        out.append((q.copy(), v.copy(), a.copy()))
    Q, V, A = (np.array([o[i] for o in out]) for i in range(3))
    return Q, V, A, t_stop


def brake_fine(q, v, a, amax, jmax, h=0.001, t_max=5.0):
    """brake 와 같은 정지를 h 마다 멈출 때까지: (q, v, a) [n, 3, 관절] (n 번째 = 멈춘 상태) 와 멈춘 시각 n·h. 입력성형용 1 ms 칸
    (같은 계산이라 brake 의 같은 시각 값과 같음).
    [2026-10-15 빠르게] 관절마다 파이썬 실수로 (track_target 설명과 같은 까닭, 2.9 ms → 0.4 ms). 먼저 선 관절은 그 자리 (속도·가속 0)
    로 채움 — 예전 계산도 선 관절은 다음 단계에 그대로 서 있음 (가속 −0 → 위치 그대로). 값은 비트까지 같음 (tests/test_motion.py)."""
    q, v, a = np.array(q, float), np.array(v, float), np.array(a, float)
    J = q.size
    am, jm = _per_joint(amax, J), _per_joint(jmax, J)
    two_j, rel, jh = 2.0 * jm, jm * h * h, jm * h
    dv, da = 2.0 * jm * h * h, 2.0 * jm * h
    kmax = int(round(t_max / h))
    runs = []
    for i in range(J):
        qi, vi, ai = float(q[i]), float(v[i]), float(a[i])
        ami, tji, rli, jhi, dvi, dai = float(am[i]), float(two_j[i]), float(rel[i]), float(jh[i]), float(dv[i]), float(da[i])
        rq, rv, ra = [], [], []
        for _ in range(kmax):
            if ai * vi < 0 and abs(vi) <= ai * ai / tji + rli:      # 지금 가속을 0 으로 풀면 딱 멈춤
                at = 0.0
            else:
                at = -(1.0 if vi > 0 else (-1.0 if vi < 0 else 0.0)) * ami
            an = _clip(at, ai - jhi, ai + jhi)
            qi = qi + vi * h + 0.5 * an * h * h
            vi = vi + an * h
            ai = an
            stop = abs(vi) < dvi and abs(ai) <= dai
            if stop:
                vi = ai = 0.0
            rq.append(qi)
            rv.append(vi)
            ra.append(ai)
            if stop:
                break
        else:
            raise RuntimeError(f'정지가 {t_max} 초 안에 안 끝남')
        runs.append((rq, rv, ra))
    n = max(len(r[0]) for r in runs)
    out = np.zeros((n, 3, J))
    for i, (rq, rv, ra) in enumerate(runs):
        m = len(rq)
        out[:m, 0, i], out[:m, 1, i], out[:m, 2, i] = rq, rv, ra
        out[m:, 0, i] = rq[-1]
    return out, n * h


def braking_tail(t0, q, v, a, amax, jmax, dt=0.01, t_max=5.0):
    """시각 t0 의 상태에서 멈추는 꼬리 [(시각, q, v, a)], dt 간격, 마지막 점 = 멈춘 상태 (속도·가속 0)."""
    grid = [j * dt for j in range(1, int(round(t_max / dt)) + 1)]
    Q, V, A, t_stop = brake(q, v, a, grid, amax, jmax)       # 멈추면 그 뒤는 멈춘 상태 그대로
    if t_stop is None:
        raise RuntimeError(f'멈춤 꼬리가 {t_max} 초 안에 안 멈춤')
    ts = sorted(set(round(x, 6) for x in [tt for tt in grid if tt < t_stop - 1e-9] + [max(t_stop, dt)]))
    n = len(ts)                                               # 마지막 점 = 멈춘 상태 (격자 다음 점 = 같은 값)
    out = [(t0 + tt, Q[i], V[i], A[i]) for i, tt in enumerate(ts[:n])]
    out[-1] = (out[-1][0], out[-1][1], np.zeros_like(out[-1][1]), np.zeros_like(out[-1][1]))
    return out
