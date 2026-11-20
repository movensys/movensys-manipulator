#!/usr/bin/env python3
"""고정 카메라 위치 계산 (카메라 보정) — 로봇 받침 (base_link) ← 깊이 카메라 광학 좌표. 미로봇 GUI camera_fit.py 를 CR3A 로 옮김.

  python3 camera_fit.py <촬영 폴더> [--with-gripper]      (tools/cam_capture.py 가 찍은 폴더 + 그 순간의 엔코더 관절각 encoders.json)
  → <폴더>/camera_pose.json (카메라 위치·점수·판정), overlay.png (컬러 영상 위에 로봇 모델), fit_points.png (깊이 점 ↔ 모델 거리)

방법 (미로봇과 같음, ICP 와 같은 생각): 깊이 점을 후보 카메라 위치로 받침 좌표로 옮겨, 로봇 3D 모델 (찍은 순간의 관절각) 표면 근처의
점은 표면에 붙게 + 받침 둘레 테이블 윗면 점은 높이 0 에 오게 강건 최소제곱으로 고친다 (넓게 고르고 점점 좁힘). 로봇은 안 움직인다.
[CR3A 에서 바뀐 것 2026-10-26]
- 관절각 = 실제 로봇 엔코더 (R2 일반 노드로 통신만, 서보 OFF — wmx_r2.sh encoders). 미로봇은 스텝 수를 썼음.
- 깊이 = D455 깊이 원본 (image_rect_raw, 왜곡 없는 핀홀) → 카메라 위치 = 깊이 광학 좌표. 컬러 겹쳐 보기는 카메라가 알려 준 깊이 → 컬러
  외부 값으로.
- 시작 값을 자동으로 (미로봇은 사진에서 점 4 개 클릭): ① 테이블 평면 (RANSAC — 받침 바닥 = 테이블 윗면) 으로 카메라 높이·기울기,
  ② 위에서 내려다본 높이 지도를 로봇 모델 높이 지도와 맞춰 받침 위치·방향 (5° 마다), ③ 좋은 후보 몇 개 + 골짜기 방향 시작 값에서 맞추기
  → 로봇 점이 가장 많이 붙는 답 (미로봇 auto 와 같은 잣대).
- 모델 = CR3A + 공구 (cr3a_model). 그리퍼는 손가락 벌림이 실제와 다를 수 있어 기본은 뺌 (팔·브래킷·손목 카메라).
[2026-10-27 디버깅 D-2, 사용자 "받침 아래 구조물이 URDF 에 없음 … 카메라로 바닥면도 보정"] 테이블 높이도 잼: 전엔 테이블 윗면 점을 z = 0
  (받침 원점) 으로 당겨 맞췄음 → 실제 받침이 URDF 보다 몇 mm 높거나 아래에 판이 있으면 로봇 점과 테이블이 서로 당겨 '테이블 높이 안 맞음'
  (10-27 실물 3.02 mm > 3 mm). 이제 테이블 점은 자기 평균에서 벗어난 만큼만 (평평함) 맞춤 → 테이블은 카메라 기울기만 잡고 높이는 로봇 점이
  정함. 테이블 높이 (table_z_mm, 받침 원점 기준) 는 맞춘 뒤 테이블 점의 가운데값으로 재서 결과에 남김 → 등록 (camera_pose.json 의
  table_z_m) → 서버가 지도 바닥 띠·충돌 바닥에 씀. 테이블 점은 받침 둘레에서 높이 ±TABLE_SEARCH_M 안의 가운데값 둘레로 고름.
  [D-2 독립 검토] 테이블 점이 TABLE_MIN_PTS 보다 적으면 테이블 높이를 안 잼 (None — 0 mm 와 구분), 받침 원점보다 TABLE_UP_MAX_MM 넘게
  높으면 '나쁨' (서버가 안 씀), 고르기 기준 3 → 4 mm (PLANE_FLAT_MM).
"""
import argparse
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

import cr3a_model as cm
from common import LINK_TO_OPTICAL, link_from_optical  # noqa: F401 ([R-1] camera_link → 광학 좌표는 common 한곳 — 이름은 그대로 cf.LINK_TO_OPTICAL)

ARM_LINKS = ('base_link', 'Link1', 'Link2', 'Link3', 'Link4', 'Link5', 'Link6', 'tool_bracket', 'tool_camera')
SURF_STEP_M = 0.003             # 모델 표면 점 간격
DEPTH_STEP = 2                  # 깊이 점은 2 픽셀마다
Z_RANGE_M = (0.2, 2.5)          # 쓸 깊이
PLANE_TOL_M = 0.02              # 테이블 윗면 점: 테이블 높이 (받침 둘레 점의 가운데값) 에서 이 안
PLANE_R_OUT_M = 0.30            # 테이블 윗면 점: 받침 축에서 이 안 (받침 둘레)
TABLE_SEARCH_M = 0.06           # [D-2] 테이블 높이를 찾는 범위: 받침 원점 ± 이만큼 (받침 아래 판·URDF 높이 차 몇 cm 까지)
TABLE_MIN_PTS = 500             # [D-2 검토] 테이블 점이 이보다 적으면 테이블 높이를 안 잼 (가려짐 — 몇 점의 가운데값을 높이로 쓰지 않게)
TABLE_UP_MAX_MM = 15.0          # [D-2 검토] 테이블이 받침 원점보다 이만큼 넘게 높으면 '나쁨' — 서버도 안 씀 (rt_config FLOOR_REACT_TOP_MAX_M
                                #  3.5 cm − 바닥 반응 2 cm: 바닥을 더 올리면 받침·Link1 충돌 구가 늘 바닥 반응 거리 안 → 계획이 모두 실패)
TABLE_DOWN_MAX_MM = 25.0        # [D-2] 테이블이 받침 원점보다 이만큼 넘게 낮으면 '나쁨' (받침 아래 구조물·로봇 모델 확인)
PLANE_FLAT_MM = 4.0             # [D-2 검토] 테이블 고르기 기준 (가운데값에서 벗어난 크기의 가운데값). 3 → 4 mm: 10-26·10-27 보정 자료 3 장이
                                #  D455 깊이 잡음만으로 2.2~2.9 mm 라 3 mm 는 좋은 보정도 자세에 따라 '나쁨' 이 됨
HMAP_RES_M = 0.01               # 높이 지도 칸
HMAP_HALF_M = 0.65              # 모델 높이 지도 반 폭 (CR3A 팔 길이 + 공구)
YAW_STEP_DEG = 5.0
N_CANDIDATES = 6                # 높이 지도에서 고른 시작 값 수
VALLEY_DEG = (-9, -6, -3, 3, 6, 9)      # 가장 좋은 후보 둘레로 카메라를 로봇 둘레 세로축으로 돌린 시작 값 (미로봇 auto 의 골짜기)
COLORS = {'base_link': (90, 90, 90), 'Link1': (40, 160, 255), 'Link2': (60, 220, 60), 'Link3': (255, 120, 40),
          'Link4': (200, 60, 220), 'Link5': (40, 220, 220), 'Link6': (255, 60, 120), 'tool_bracket': (40, 140, 220),
          'tool_camera': (220, 200, 40), 'tool_gripper': (160, 160, 255)}      # BGR


# ---------------- 모델 ----------------
class RobotSurface:
    """로봇 메시 표면에 고르게 뿌린 점 (링크 좌표). at(q) = 받침 좌표 점 + 링크 이름."""

    def __init__(self, robot: cm.Robot, links=ARM_LINKS, step=SURF_STEP_M, seed=0):
        rng = np.random.default_rng(seed)
        self.robot, self.local = robot, {}
        for name, P, F, _ in robot.parts:
            if name not in links:
                continue
            tri = P[F]
            area = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
            idx = np.repeat(np.arange(len(tri)), rng.poisson(area / step ** 2))
            s, r2 = np.sqrt(rng.random(len(idx))), rng.random(len(idx))
            pts = (1 - s)[:, None] * tri[idx, 0] + (s * (1 - r2))[:, None] * tri[idx, 1] + (s * r2)[:, None] * tri[idx, 2]
            self.local[name] = np.vstack([pts, P])
        base = self.local['base_link']
        self.base_r = float(np.hypot(base[:, 0], base[:, 1])[base[:, 2] < 0.05].max())     # 받침 바닥 반지름

    def at(self, q_deg):
        T = self.robot.fk(q_deg)
        pts = [p @ T[n][:3, :3].T + T[n][:3, 3] for n, p in self.local.items()]
        names = np.concatenate([[n] * len(p) for n, p in zip(self.local, pts)])
        return np.vstack(pts), names


def depth_points(depth_mm, K, step=DEPTH_STEP, z_range=Z_RANGE_M):
    """깊이 영상 (mm) → 광학 좌표 점 [n, 3] (m)."""
    d = depth_mm.astype(np.float32) / 1000.0
    v, u = np.mgrid[0:d.shape[0]:step, 0:d.shape[1]:step]
    z = d[v, u]
    ok = (z > z_range[0]) & (z < z_range[1])
    u, v, z = u[ok], v[ok], z[ok]
    return np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], 1)


def apply(T, P):
    return P @ T[:3, :3].T + T[:3, 3]


def pose_xyzrpy(T):
    """4x4 → (x, y, z mm, Rx, Ry, Rz °: 고정 축 x→y→z = ROS RPY)."""
    return [round(float(v) * 1000, 2) for v in T[:3, 3]] + [round(a, 3) for a in cm.rpy_from_matrix(T[:3, :3])]


def pose_diff(Ta, Tb):
    """두 자세 차이 (위치 mm, 방향 °)."""
    c = (np.trace(Ta[:3, :3].T @ Tb[:3, :3]) - 1) / 2
    return float(np.linalg.norm(Ta[:3, 3] - Tb[:3, 3]) * 1000), math.degrees(math.acos(max(-1.0, min(1.0, c))))


def same_answer(Ta, Tb, mm=10.0, deg=1.5):
    d, a = pose_diff(Ta, Tb)
    return d < mm and a < deg


# ---------------- 맞추기 ----------------
class PoseFitter:
    """한 순간의 자료: 깊이 점 (광학 좌표) + 그 순간 관절각의 로봇 모델 표면 점. 카메라 위치 T = 받침 ← 깊이 광학 좌표."""

    def __init__(self, depth_mm, K, q_deg, surf: RobotSurface):
        self.pts = depth_points(depth_mm, K)
        self.model, self.names = surf.at(q_deg)
        self.tree = cKDTree(self.model)
        self.r_in = surf.base_r + 0.015

    def select(self, T, near):
        """로봇 모델에서 near 안의 점 (로봇으로 보는 점) + 받침 둘레 테이블 윗면 점 (높이 ±TABLE_SEARCH_M 안 점의 가운데값 ±PLANE_TOL_M —
        [D-2] 테이블이 받침 원점 z = 0 이 아니어도 고름)."""
        B = apply(T, self.pts)
        dist, _ = self.tree.query(B, distance_upper_bound=near)
        r = np.hypot(B[:, 0], B[:, 1])
        ring = (r > self.r_in) & (r < PLANE_R_OUT_M)
        cand = ring & ~np.isfinite(dist) & (np.abs(B[:, 2]) < TABLE_SEARCH_M)     # 테이블 높이는 모델에서 먼 받침 둘레 점으로
        zt = float(np.median(B[cand, 2])) if cand.sum() > 50 else 0.0
        robot = np.isfinite(dist) & (B[:, 2] > zt + 0.01)
        plane = ring & (np.abs(B[:, 2] - zt) < PLANE_TOL_M) & ~robot
        return self.pts[robot], self.pts[plane]

    @staticmethod
    def _make(T0, v):
        T = T0.copy()
        T[:3, :3] = Rotation.from_rotvec(v[3:]).as_matrix() @ T0[:3, :3]
        T[:3, 3] = T0[:3, 3] + v[:3]
        return T

    def fit(self, T0, max_move_m=0.25, max_rot_deg=20.0):
        """T0 에서 맞추기 (이 자료 하나로 fit_joint)."""
        return fit_joint([self], T0, max_move_m, max_rot_deg)

    def score(self, T, near=0.012):
        """같은 잣대 (미로봇과 같음): 로봇 점 수 (모델 12 mm 안), 그 거리 중앙값, 테이블 윗면 점 수, 고르기 (테이블 점이 그 가운데값에서
        벗어난 크기의 가운데값 — [D-2] 전엔 z = 0 에서 벗어난 크기), [D-2] 테이블 높이 (받침 원점 기준, 테이블 점의 가운데값 —
        테이블 점이 TABLE_MIN_PTS 보다 적으면 None = 안 잼)."""
        R, Pl = self.select(T, near)
        dr = self.tree.query(apply(T, R))[0] if len(R) else np.array([np.inf])
        zp = apply(T, Pl)[:, 2] if len(Pl) else np.array([np.nan])
        zt = float(np.nanmedian(zp)) if len(Pl) else None
        return {'robot': int(len(R)), 'median_mm': round(float(np.median(dr) * 1000), 2), 'plane': int(len(Pl)),
                'plane_mm': round(float(np.nanmedian(np.abs(zp - zt)) * 1000), 2) if len(Pl) else None,
                'table_z_mm': round(zt * 1000, 2) if len(Pl) >= TABLE_MIN_PTS else None}

    @staticmethod
    def verdict(s, agree=None):
        """판정 (미로봇 기준을 CR3A 크기에 맞춤: 로봇이 커서 점이 많음). 테이블 높이는 판정 글에 같이 적음."""
        why = []
        if agree is not None and agree < 2:
            why.append('이 답에 닿은 시작 값이 하나뿐 — 다른 답일 수 있음')
        if s['robot'] < 1500:
            why.append(f'로봇 점이 적음 ({s["robot"]} < 1500)')
        if s['median_mm'] > 3.0:
            why.append(f'로봇 점이 모델에서 멂 (중앙값 {s["median_mm"]:.1f} mm > 3 mm)')
        if s['plane'] > 100 and s['plane_mm'] is not None and s['plane_mm'] > PLANE_FLAT_MM:
            why.append(f'테이블 윗면이 고르지 않거나 받침과 기울어짐 ({s["plane_mm"]:.1f} mm > {PLANE_FLAT_MM:.0f} mm)')
        zt = s.get('table_z_mm')
        if zt is not None and zt > TABLE_UP_MAX_MM:
            why.append(f'테이블이 받침 원점보다 {zt:.0f} mm 높음 ({TABLE_UP_MAX_MM:.0f} mm 넘음 — 서버가 안 씀. 받침 아래 구조물·로봇 모델 확인)')
        elif zt is not None and zt < -TABLE_DOWN_MAX_MM:
            why.append(f'테이블이 받침 원점보다 {-zt:.0f} mm 낮음 ({TABLE_DOWN_MAX_MM:.0f} mm 넘음 — 받침 아래 구조물·로봇 모델 확인)')
        tail = f' · 테이블 높이 안 잼 (테이블 점 {s["plane"]} < {TABLE_MIN_PTS})' if zt is None else f' · 테이블 높이 {zt:+.1f} mm'
        return not why, (' / '.join(why) or '좋음') + tail


def fit_joint(fitters, T0, max_move_m=0.25, max_rot_deg=20.0):
    """카메라 위치 하나로 여러 자료 (로봇 자세마다 PoseFitter 하나) 의 깊이 점이 모두 모델에 붙게: 넓게 고르고 → 맞추고 → 다시 고르기
    (8 → 4 → 2 → 1.2 cm). 회전은 카메라 자리 둘레 (받침 축 기준 회전 벡터). 자료가 하나면 PoseFitter.fit.
    [D-2] 테이블 점 항 = 높이에서 (모든 자료의) 평균 높이를 뺀 값 — 테이블이 평평하고 받침과 평행하게만 (기울기), 높이는 로봇 점이 정함."""
    x = np.zeros(6)
    lim = np.array([max_move_m] * 3 + [math.radians(max_rot_deg)] * 3)
    for near in (0.08, 0.04, 0.02, 0.012):
        sel = [(f, *f.select(PoseFitter._make(T0, x), near)) for f in fitters]
        if sum(len(R) for _, R, _ in sel) < 30:
            break

        def res(v):
            T = PoseFitter._make(T0, v)
            out, zs = [], []
            for f, R, Pl in sel:
                out.append(f.tree.query(apply(T, R))[0])
                zs.append(apply(T, Pl)[:, 2])
            z = np.concatenate(zs)
            return np.concatenate(out + ([z - z.mean()] if len(z) else []))
        x = least_squares(res, np.clip(x, -lim + 1e-9, lim - 1e-9), bounds=(-lim, lim), loss='cauchy', f_scale=0.005,
                          diff_step=1e-4, x_scale=[0.01] * 6, max_nfev=200).x
    return PoseFitter._make(T0, x)


def mean_pose(Ts):
    """카메라 위치 여러 개의 단순 평균: 위치는 평균, 방향은 회전 평균 (각도 숫자를 따로 평균하면 ±180° 근처에서 틀림)."""
    out = np.eye(4)
    out[:3, :3] = Rotation.from_matrix([T[:3, :3] for T in Ts]).mean().as_matrix()
    out[:3, 3] = np.mean([T[:3, 3] for T in Ts], 0)
    return out


def fit_many(fitters, Ts, max_starts=5, log=print, should_stop=None):
    """[미로봇 2026-10-06 fit_many 와 같음] 여러 자세에서 따로 구한 카메라 위치 (Ts, fitters 와 같은 순서) 를 하나로.
    자세 하나로는 골짜기 방향 (옆 이동 ↔ 좌우 돌림) 이 잘 안 정해져 자세마다 답이 몇 cm·몇 ° 갈림 (미로봇 2~3 cm·3°). 숫자만 평균하면
    잘 정해진 자세와 덜 정해진 자세가 같은 몫 → 단순 평균에서 시작해 모든 자세의 자료를 함께 맞춤 (그 방향을 잘 잡는 자세가 더 큰 몫).
    시작 값 = 평균 + 평균에서 먼 순서로 서로 다른 자세별 답 (max_starts 개까지). 모든 자세 합쳐 로봇 점이 가장 많이 붙는 답.
    [2026-10-27 R-1] should_stop() 이 참이면 다음 시작 값 전에 멈춤 (GUI 를 끌 때 — camera_panel.CANCEL, 판정 '중간에 멈춤').
    반환 dict: T, scores (자세별), agree (같은 답으로 모인 시작 값 수), tried, mean (단순 평균), mean_scores"""
    avg = mean_pose(Ts)
    starts = [avg]
    for T in sorted(Ts, key=lambda T: -(pose_diff(T, avg)[0] / 10 + pose_diff(T, avg)[1] / 1.5)):
        if len(starts) < max_starts and not any(same_answer(T, s) for s in starts):
            starts.append(T)
    results = []
    for i, s in enumerate(starts):
        if should_stop is not None and should_stop():
            break
        T = fit_joint(fitters, s)
        results.append(([f.score(T) for f in fitters], T))
        log(f'  여러 자세 함께 맞추기 {i + 1}/{len(starts)}: 로봇 점 {sum(sc["robot"] for sc in results[-1][0])}')
    out = {'T': None, 'scores': None, 'agree': 0, 'tried': len(results), 'starts': len(starts), 'mean': avg,
           'mean_scores': [f.score(avg) for f in fitters]}
    if results:
        results.sort(key=lambda r: (-sum(s['robot'] for s in r[0]), float(np.mean([s['median_mm'] for s in r[0]]))))
        out['scores'], out['T'] = results[0]
        out['agree'] = sum(1 for _, T in results if same_answer(T, out['T']))
    return out


def verdict_many(scores, agree, tried, starts):
    """fit_many 판정: 자세마다 PoseFitter.verdict + 시작 값들이 같은 답으로 모였나. → (좋음?, 이유)"""
    why = []
    if tried < starts:
        why.append(f'중간에 멈춤 — 시작 값 {starts} 개 중 {tried} 개만')
    elif tried >= 2 and agree < 2:
        why.append('시작 값들이 서로 다른 답으로 갈림 — 튀는 자세를 지우고 다시')
    for i, s in enumerate(scores or []):
        ok, w = PoseFitter.verdict(s)
        if not ok:
            why.append(f'자세 {i + 1}: {w}')
    # [D-2 검토] 테이블 높이 = 자세별 값의 가운데값 (camera_panel [등록] 이 저장하는 값과 같음)
    tz = [s['table_z_mm'] for s in scores or [] if s.get('table_z_mm') is not None]
    tail = f' · 테이블 높이 {float(np.median(tz)):+.1f} mm' if tz else ' · 테이블 높이 안 잼'
    return not why, (' / '.join(why) or '좋음') + tail


def refine(fitter, T0, log=print):
    """이미 아는 카메라 위치 (등록된 것·앞 자세의 답) 에서 시작해 이 자세만 빠르게: T0 + 골짜기 시작 값 → 가장 좋은 답.
    처음 자세는 calibrate (자동 시작 값, ~100 s), 다음 자세부터는 이것 (~20 s). 반환 calibrate 와 같은 dict."""
    results = []
    for T in [T0] + [around_robot(T0, d) for d in VALLEY_DEG]:
        Tf = fitter.fit(T)
        results.append((fitter.score(Tf), Tf))
    results.sort(key=lambda r: (-r[0]['robot'], r[0]['median_mm']))
    s, T = results[0]
    agree = sum(1 for _, t in results if same_answer(t, T))
    ok, why = PoseFitter.verdict(s, agree)
    log(f'맞추기 (아는 위치에서): 로봇 점 {s["robot"]}, 중앙값 {s["median_mm"]} mm, 같은 답 {agree}/{len(results)} → {why}')
    return {'T': T, 'score': s, 'agree': agree, 'tried': len(results), 'ok': ok, 'why': why}


# ---------------- 자동 시작 값 ----------------
def table_plane(P, iters=1500, tol=0.01, seed=0):
    """테이블 평면 (RANSAC): 영상에서 위를 보는 (광학 y 가 아래라 법선 y < −0.3) 가장 큰 평면. → (법선 n, d) n·p + d = 0, 카메라 쪽 d > 0."""
    rng = np.random.default_rng(seed)
    S = P[rng.choice(len(P), min(len(P), 30000), replace=False)]
    best = (0, None, None)
    for _ in range(iters):
        a, b, c = S[rng.choice(len(S), 3, replace=False)]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        if n @ a > 0:
            n = -n                                  # 카메라 (원점) 쪽을 보게
        if n[1] > -0.3:
            continue
        d = -float(n @ a)
        cnt = int((np.abs(S @ n + d) < tol).sum())
        if cnt > best[0]:
            best = (cnt, n, d)
    _, n, d = best
    inl = P[np.abs(P @ n + d) < tol]
    c = inl.mean(0)
    n2 = np.linalg.svd(inl - c)[2][2]
    n2 = n2 if n2 @ n > 0 else -n2
    return n2, -float(n2 @ c), len(inl)


def plane_frame(n, d):
    """평면 좌표 F (카메라 좌표 기준 4x4): 원점 = 카메라 중심을 평면에 내린 발, z = 법선 (위), x = 카메라 앞 방향을 평면에 내린 것."""
    o = -d * n
    x = np.array([0.0, 0.0, 1.0]) - n[2] * n
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, np.cross(n, x), n])
    T[:3, 3] = o
    return T


def height_map(P, x0, y0, nx, ny, res=HMAP_RES_M, zmin=0.02, zmax=0.8):
    """점 (평면 좌표) → 위에서 본 높이 지도 [ny, nx] (칸마다 가장 높은 점, 없으면 0)."""
    k = (P[:, 2] > zmin) & (P[:, 2] < zmax)
    ix = np.floor((P[k, 0] - x0) / res).astype(int)
    iy = np.floor((P[k, 1] - y0) / res).astype(int)
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    H = np.zeros((ny, nx), np.float32)
    np.maximum.at(H, (iy[ok], ix[ok]), P[k, 2][ok].astype(np.float32))
    return H


def auto_starts(fitter: PoseFitter, log=print):
    """시작 값 후보 [(높이 지도 점수, T)] — 테이블 평면 + 높이 지도 맞추기. T = 받침 ← 깊이 광학 좌표."""
    n, d, n_in = table_plane(fitter.pts)
    T_CF = plane_frame(n, d)
    PF = apply(np.linalg.inv(T_CF), fitter.pts)
    log(f'테이블 평면: 점 {n_in} 개, 카메라 높이 {d * 1000:.0f} mm, 기울기 {math.degrees(math.acos(min(1.0, abs(n[2])))):.1f}°')
    x0, y0 = float(PF[:, 0].min()) - HMAP_HALF_M, float(PF[:, 1].min()) - HMAP_HALF_M
    nx = int((PF[:, 0].max() - x0 + HMAP_HALF_M) / HMAP_RES_M) + 1
    ny = int((PF[:, 1].max() - y0 + HMAP_HALF_M) / HMAP_RES_M) + 1
    D = cv2.GaussianBlur(height_map(PF, x0, y0, nx, ny), (0, 0), 1.5)
    m = int(round(HMAP_HALF_M / HMAP_RES_M))
    found = []
    for yaw in np.arange(0.0, 360.0, YAW_STEP_DEG):
        Rz = Rotation.from_euler('z', yaw, degrees=True).as_matrix()
        M = height_map(fitter.model @ Rz.T, -HMAP_HALF_M, -HMAP_HALF_M, 2 * m + 1, 2 * m + 1)
        M = cv2.GaussianBlur(M, (0, 0), 1.5)
        r = cv2.matchTemplate(D, M, cv2.TM_CCORR_NORMED)
        _, val, _, (cx, cy) = cv2.minMaxLoc(r)
        bx, by = x0 + (cx + m + 0.5) * HMAP_RES_M, y0 + (cy + m + 0.5) * HMAP_RES_M     # 받침 원점 (평면 좌표)
        T_FB = np.eye(4)
        T_FB[:3, :3] = Rz
        T_FB[:3, 3] = [bx, by, 0.0]
        found.append((float(val), yaw, np.linalg.inv(T_CF @ T_FB)))
    found.sort(key=lambda f: -f[0])
    out = []
    for val, yaw, T in found:
        if not any(pose_diff(T, t)[0] < 100 and pose_diff(T, t)[1] < 15 for _, t in out):
            out.append((val, T))
        if len(out) >= N_CANDIDATES:
            break
    log('높이 지도 후보 (점수, 받침 방향 °): ' + ', '.join(f'{v:.3f}' for v, _ in out))
    return out


def around_robot(T, deg, center=(0.0, 0.0, 0.25)):
    """카메라를 로봇 가운데를 지나는 세로축 둘레로 deg 돌린 위치 (사진 속 로봇 자리는 거의 그대로 — 골짜기 시작 값)."""
    c = np.eye(4)
    c[:3, 3] = center
    Rz = np.eye(4)
    Rz[:3, :3] = Rotation.from_euler('z', deg, degrees=True).as_matrix()
    return c @ Rz @ np.linalg.inv(c) @ T


def calibrate(fitter: PoseFitter, log=print, should_stop=None):
    """자동 보정: 높이 지도 후보 + 가장 좋은 후보 둘레 골짜기 시작 값 → 맞추기 → 로봇 점이 가장 많이 붙는 답.
    [2026-10-27 R-1] should_stop() 이 참이면 다음 후보 전에 멈춤 (GUI 를 끌 때 — camera_panel.CANCEL. 하나도 못 했으면 IndexError).
    반환 dict: T, score, agree (같은 답으로 모인 시작 값 수), tried, ok, why."""
    t0 = time.time()
    cands = auto_starts(fitter, log)
    results = []
    for i, (_, T0) in enumerate(cands):
        if should_stop is not None and should_stop():
            break
        T = fitter.fit(T0)
        results.append((fitter.score(T), T))
        log(f'  후보 {i + 1}/{len(cands)}: 로봇 점 {results[-1][0]["robot"]}, 중앙값 {results[-1][0]["median_mm"]} mm')
    results.sort(key=lambda r: (-r[0]['robot'], r[0]['median_mm']))
    best = results[0][1]
    for deg in VALLEY_DEG:
        if should_stop is not None and should_stop():
            break
        T = fitter.fit(around_robot(best, deg))
        results.append((fitter.score(T), T))
    results.sort(key=lambda r: (-r[0]['robot'], r[0]['median_mm']))
    s, T = results[0]
    agree = sum(1 for _, t in results if same_answer(t, T))
    ok, why = PoseFitter.verdict(s, agree)
    log(f'보정 {time.time() - t0:.1f} s: 로봇 점 {s["robot"]}, 중앙값 {s["median_mm"]} mm, 테이블 {s["plane"]} 점 '
        f'고르기 {s["plane_mm"]} mm, 같은 답 {agree}/{len(results)} → {why}')
    return {'T': T, 'score': s, 'agree': agree, 'tried': len(results), 'ok': ok, 'why': why}


# ---------------- 확인 그림 ----------------
def project(T_color_base, P, K, D=None):
    """받침 좌표 점 → 컬러 픽셀 (u, v) 와 깊이 z."""
    C = apply(T_color_base, P)
    z = C[:, 2]
    uv, _ = cv2.projectPoints(C.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, None if D is None else np.asarray(D, float))
    return uv.reshape(-1, 2), z


def overlay(color, T_color_base, K, D, model, names, alpha=0.45):
    """컬러 영상 위에 로봇 모델 점 (링크마다 색, 가까운 점이 앞)."""
    uv, z = project(T_color_base, model, K, D)
    h, w = color.shape[:2]
    ok = (z > 0.05) & (uv[:, 0] >= 0) & (uv[:, 0] < w - 1) & (uv[:, 1] >= 0) & (uv[:, 1] < h - 1)
    order = np.argsort(-z[ok])                      # 먼 점부터 → 같은 픽셀은 가까운 점이 남음
    u = uv[ok][order].round().astype(int)
    col = np.array([COLORS.get(n, (200, 200, 200)) for n in np.asarray(names)[ok][order]], np.uint8)
    lay = np.zeros_like(color)
    mask = np.zeros((h, w), np.uint8)
    lay[u[:, 1], u[:, 0]] = col
    mask[u[:, 1], u[:, 0]] = 1
    mask = cv2.dilate(mask, np.ones((2, 2), np.uint8)).astype(bool)
    lay = cv2.dilate(lay, np.ones((2, 2), np.uint8))
    out = color.copy()
    out[mask] = (alpha * lay[mask] + (1 - alpha) * color[mask]).astype(np.uint8)
    return out


def residual_image(fitter: PoseFitter, T, depth_shape, K):
    """깊이 점마다 로봇 모델까지 거리 (로봇 근처 3 cm 안만 색: 초록 0 mm → 빨강 15 mm+)."""
    B = apply(T, fitter.pts)
    dist, _ = fitter.tree.query(B, distance_upper_bound=0.03)
    h, w = depth_shape
    img = np.full((h, w, 3), 30, np.uint8)
    P = fitter.pts
    u = np.clip((P[:, 0] * K[0, 0] / P[:, 2] + K[0, 2]).round().astype(int), 0, w - 1)
    v = np.clip((P[:, 1] * K[1, 1] / P[:, 2] + K[1, 2]).round().astype(int), 0, h - 1)
    img[v, u] = (90, 90, 90)
    k = np.isfinite(dist)
    t = np.clip(dist[k] / 0.015, 0, 1)
    img[v[k], u[k]] = np.stack([np.zeros_like(t), (1 - t) * 255, t * 255], 1).astype(np.uint8)
    img = cv2.dilate(img, np.ones((2, 2), np.uint8))
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('folder')
    ap.add_argument('--with-gripper', action='store_true', help='그리퍼 (손가락 95 mm 벌림) 도 모델에 넣음')
    a = ap.parse_args()
    f = Path(a.folder)
    info = json.loads((f / 'info.json').read_text())
    enc = json.loads((f / 'encoders.json').read_text())
    depth = np.load(f / 'depth_raw.npy')
    color = cv2.imread(str(f / 'color.png'))
    Kd = np.array(info['depth']['K'], float).reshape(3, 3)
    Kc = np.array(info['color']['K'], float).reshape(3, 3)
    T_cd = np.array(info['T_color_depth'], float)
    links = ARM_LINKS + (('tool_gripper',) if a.with_gripper else ())
    robot = cm.Robot()
    surf = RobotSurface(robot, links)
    fitter = PoseFitter(depth, Kd, enc['q_deg'], surf)
    print(f'관절각 (엔코더) {enc["q_deg"]}, 깊이 점 {len(fitter.pts)}, 모델 점 {len(fitter.model)} ({", ".join(surf.local)})')
    res = calibrate(fitter)
    T = res['T']
    T_link = link_from_optical(T)                   # 받침 ← camera_link
    T_color = T @ np.linalg.inv(T_cd)
    out = {'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'serial': info['serial'], 'q_deg': enc['q_deg'], 'links': list(surf.local),
           'T_base_depth': T.tolist(), 'T_base_color': T_color.tolist(),
           'camera_link_xyzrpy': pose_xyzrpy(T_link), 'depth_optical_xyzrpy': pose_xyzrpy(T),
           'depth_K': Kd.tolist(), 'depth_wh': [info['depth']['w'], info['depth']['h']],
           'score': res['score'], 'agree': res['agree'], 'tried': res['tried'], 'ok': res['ok'], 'why': res['why'],
           'table_z_m': None if res['score'].get('table_z_mm') is None else round(res['score']['table_z_mm'] / 1000, 4)}
    (f / 'camera_pose.json').write_text(json.dumps(out, indent=1, ensure_ascii=False))
    full, full_names = RobotSurface(robot, ARM_LINKS + ('tool_gripper',)).at(enc['q_deg'])
    cv2.imwrite(str(f / 'overlay.png'), overlay(color, np.linalg.inv(T_color), Kc, info['color']['D'], full, full_names))
    cv2.imwrite(str(f / 'fit_points.png'), residual_image(fitter, T, depth.shape, Kd))
    print('camera_link (받침 기준) x y z mm, Rx Ry Rz °:', out['camera_link_xyzrpy'])
    print('판정:', res['why'])


if __name__ == '__main__':
    main()
