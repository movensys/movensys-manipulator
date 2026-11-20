#!/usr/bin/env python3
"""[2026-10-28 디버깅 D-4, 사용자 "장애물이 감지되었다고 판단되었을 때 이미지를 촬영하도록 로그를 추가할 수 있을까?"] 이미지 기록.

까닭: 실물 실행 (gui_real_20261028_131249, 13:34) 에서 아무것도 없는데 7 번 비킴. 기록을 보면 그때마다 지도 칸이 로봇 자기 어깨·위팔·
  팔꿈치 17~50 mm 에 있었고 경계 채우기로 MPC 지도 거리가 −1~−40 mm. 멈춘 자세에서는 보정·팔 지우기가 맞음 → 어떤 자세·움직이는 중에만
  생김. 그 순간의 영상이 없어서 무엇이 잡혔는지 (팔 자신·케이블·실제 물체) 모름 → 판단한 순간 앞뒤 영상을 남김.

(실제 카메라만) 서버가 '로봇 가까이 물체' 로 판단한 순간 (트리거 — gui_controller):
  yield       "막혀 서 있는데 물체가 다가옴 → 그 자리에서 비킴"
  replan      "… → 멈추지 않고 다시 계획" (경로 앞 막힘 등)       goalblocked  "목표 자리가 막힘"
  guard       안전 검사 감속 정지 (brake) 가 새로 시작
  near        실시간 회피 중 MPC 지도 거리 (영상 시각 관절, 경계 채우기 포함 — rec frames near_mm) < MPC 반응 거리 (act_new) − 5 mm (SNAP_NEAR_MARGIN_M)
              — 들어갈 때, 계속 안이면 SNAP_NEAR_HOLD_S 마다
  mapcheck    켜기 전 지도 점검·[지도 점검] 걸림
→ <기록 폴더>/snap/<HHMMSS.mmm>_<까닭>/ 에:
  depth_XX.png   트리거 SNAP_PRE_S 전 ~ SNAP_POST_S 뒤 깊이 원본 (16 비트 mm — 깜빡임 거르기·팔 지우기 전)
  color_XX.jpg   깊이마다 가장 가까운 컬러
  frame_XX.json  영상마다: 찍힌 시각·받은 늦음·팔 지우기 관절 (q_mask)·지도에 쌓였으면 그 기록 (q_ctl·lag_ms·near_mm…)·팔 옆
                 SNAP_NEAR_M 안 살아남은 점 (받침 기준 위치·가장 가까운 링크·팔 겉면까지 거리 mm, 가까운 차례 SNAP_PTS_MAX 개)
  overlay.jpg    트리거 직전 영상: 컬러 위에 팔 지우기 점 파랑, 살아남은 팔 옆 점 빨강 (링크 이름·최소 거리), 그 순간 지도에서
                 로봇 겉면 5.5 cm 안 칸 노랑 (경계 채우기가 걸리는 칸 — GuiController._near_now)
  snap.json      까닭·글·시각·관절·그 순간 지도 칸 (칸마다 링크·거리)·영상 목록·시간
  '팔 거리' = cuRobo 충돌 구 겉면까지 (팔 지우기 RobotSegmenter 와 같은 식: 구 겉면 거리 < mask 면 지움). 지도에서 빼는 점 (바닥 띠·받침
  자리·받침 둘레 케이블·외따로 뜬 조각) 은 빨강에서 뺌 (frame JSON 에 수만).
제어 주기를 늦추지 않게 — 2026-10-28 Thor 시험 (WMX 시뮬레이터 + 실제 카메라, 늦은 메시지 수): 분석을 서버 안 스레드에서 → 157 → 769
  (78 % 가 쓰는 동안), 분석을 따로 프로세스로 옮겨도 서버가 컬러를 날 것으로 받기만 해도 597 (컬러 안 받으면 304) → 그래서:
  서버 안: 트리거는 적어만 두고 (잠금 하나), 제어 스레드의 poll 이 주기마다 한 가지 무거운 일만 (그 순간 지도 칸 한 번 · 넘길 때 구 계산
    한 번 — GPU 몇 ms), 쓰기 스레드는 깊이 PNG·작업 파일만 (cv2·파일 쓰기 — GIL 을 놓음) 쓰고 일꾼에게 폴더 이름 한 줄.
  일꾼 프로세스 (python3 snap_log.py --worker --ros-color, nice SNAP_NICE, 서버가 켜고 끔): 컬러 토픽을 날 것으로 받아 최근 SNAP_COLOR_N
    장을 둠 → 요청이 오면 깊이마다 받은 시각이 가장 가까운 컬러를 바로 꺼내 두고 → 풀기·분석·그림 → 결과 한 줄 (서버가 로그로).
  쓰기 줄 SNAP_QUEUE (차면 버리고 알림). 간격 SNAP_GAP_S 에 한 번, 실행마다 SNAP_MAX 번까지.
시험용 환경 변수 (서버): CR3A_SNAP=0 → 끔, CR3A_SNAP_NEAR_MM=<mm> → near 기준을 바꿈 (기본 act_new).
"""
import json
import math
import os
import queue
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

SNAP_PRE_S = 0.5            # 트리거 앞 영상 (s)
SNAP_POST_S = 0.5           # 트리거 뒤 영상 (s)
SNAP_WAIT_S = 0.15          # 뒤 끝 영상이 받기까지 더 기다림 (15 fps 한 장 + 전달)
SNAP_GAP_S = 1.0            # 기록 간격 최소 (s)
SNAP_MAX = 150              # 실행마다 최대 기록 수 (한 번 ~5 MB)
SNAP_QUEUE = 3              # 쓰기 줄 크기 — 차면 버림 (제어 스레드가 안 기다리게)
SNAP_NEAR_HOLD_S = 5.0      # near: 계속 가까운 채면 이 간격에 한 번 (반응 거리 밖으로 나갔다 들어오면 바로)
SNAP_PTS_MAX = 2000         # 영상 JSON 의 점 수 최대 (가까운 차례)
SNAP_VOX_MAX = 3000         # snap.json 의 지도 칸 수 최대
SNAP_JPG_Q = 90
SNAP_PNG_LEVEL = 1          # PNG 압축 (1 = 빠름)
SNAP_NICE = 15              # 분석 프로세스 우선순위 낮춤
SNAP_CHUNK = 20000          # 점-구 거리 계산 묶음 (메모리)
SNAP_FRAME_PAUSE_S = 0.03   # 분석 프로세스: 영상 사이 쉼 (CPU·메모리 대역 몰아 쓰지 않게)
SNAP_COLOR_N = 75           # 일꾼이 두는 최근 컬러 수 (15 fps 의 5 s — 요청이 트리거 ~0.8 s 뒤에 옴)
SNAP_CLOSE_S = 20.0         # 끝낼 때 남은 분석을 기다리는 시간 (R2 끄기 전 — 보통 몇 s)

# 사건 글 → 까닭 (rt_run·gui_controller 의 event 글)
EVENT_REASONS = (('막혀 서 있는데 물체가 다가옴', 'yield'), ('멈추지 않고 다시 계획', 'replan'), ('목표 자리가 막힘', 'goalblocked'),
                 ('지도 점검 걸림', 'mapcheck'))


def reason_of(text):
    """사건 글 → 이미지 기록 까닭 (없으면 None)."""
    for pat, r in EVENT_REASONS:
        if pat in text:
            return r
    return None


SNAP_NEAR_MARGIN_M = 0.005     # near 기준 = act_new − 이것. [2026-10-28 실물 gui_real_20261028_150253] 아무것도 없을 때 MPC 지도 거리가
                               #  59.7 mm (= act_new 근처에서 잘린 값) 라 '< 60 mm' 로 5 s 마다 걸렸음 (151126·151128·151135)


def env_near_m(act_new_m):
    """near 기준 (m): act_new − SNAP_NEAR_MARGIN_M. CR3A_SNAP_NEAR_MM (시험용) 이 있으면 그것."""
    v = os.environ.get('CR3A_SNAP_NEAR_MM')
    default = float(act_new_m) - SNAP_NEAR_MARGIN_M
    try:
        return float(v) / 1000.0 if v else default
    except ValueError:
        return default


def enabled_by_env():
    return os.environ.get('CR3A_SNAP', '1') != '0'


# [2026-10-28 Thor 시험] 일꾼이 따로 프로세스 (nice) 여도 늦은 메시지 10 → 421: OpenBLAS (행렬곱)·OpenCV 가 코어 14 개 모두에 스레드를
#  띄워 (OpenBLAS 는 기다리며 돎) 서버 프로세스와 겹침 → 일꾼은 스레드 1 개씩
SNAP_THREAD_ENV = {'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1', 'NUMEXPR_NUM_THREADS': '1'}


def worker_env():
    return dict(os.environ, **SNAP_THREAD_ENV)


def _one_thread():
    """(일꾼·--analyze) 수학·영상 라이브러리 스레드 1 개 (환경 변수는 numpy 를 가져오기 전에만 들음 — worker_env 가 넣음)."""
    try:
        import cv2
        cv2.setNumThreads(1)
    except ImportError:
        pass


# ---------------- 분석 (순수 numpy — 단위 시험) ----------------
def sphere_dist(P, c, r, chunk=SNAP_CHUNK):
    """점 [n, 3] 과 구 (중심 [S, 3]·반지름 [S]) 겉면까지 최소 거리 [n] 와 그 구 번호 [n] (float32)."""
    n = P.shape[0]
    d = np.empty(n, np.float32)
    k = np.empty(n, np.int64)
    c = np.asarray(c, np.float32)
    r = np.asarray(r, np.float32)
    for i in range(0, n, chunk):
        D = np.sqrt(((P[i:i + chunk, None, :] - c[None]) ** 2).sum(-1)) - r[None]
        k[i:i + chunk] = D.argmin(1)
        d[i:i + chunk] = D[np.arange(D.shape[0]), k[i:i + chunk]]
    return d, k


def sphere_dist_near(P, c, r, reach):
    """sphere_dist 와 같은 값이지만 어느 구 겉면에서도 reach 밖인 점은 inf (번호 −1) — 점 KD 트리로 구마다 둘레 점만 계산.
    [2026-10-28 Thor] 모든 점 × 모든 구 (점 ~10 만 × 구 ~60) 는 한 장 ~120 ms 에 메모리를 많이 써 (Thor 는 GPU 와 같은 메모리) 서버 제어
    주기의 늦은 메시지가 늘었음 → 둘레 점만."""
    n = P.shape[0]
    d = np.full(n, np.inf, np.float32)
    k = np.full(n, -1, np.int64)
    if n == 0:
        return d, k
    try:
        from scipy.spatial import cKDTree
    except ImportError:
        d, k = sphere_dist(P, c, r)
        far = d >= reach
        d[far], k[far] = np.inf, -1
        return d, k
    tree = cKDTree(P)
    c = np.asarray(c, np.float32)
    r = np.asarray(r, np.float32)
    for s, idx in enumerate(tree.query_ball_point(c, r + reach)):
        if not idx:
            continue
        idx = np.asarray(idx)
        ds = np.sqrt(((P[idx] - c[s]) ** 2).sum(1)) - r[s]
        better = ds < d[idx]
        d[idx[better]] = ds[better]
        k[idx[better]] = s
    return d, k


def candidate_pixels(shape, K, T_base_depth, c, r, reach):
    """구마다 겉면 + reach 안 점이 찍힐 수 있는 깊이 영상 네모 (보수적) 의 합 → bool [h, w]. 카메라 뒤·가까운 구는 영상 전체."""
    h, w = shape
    M = np.zeros((h, w), bool)
    Ti = np.linalg.inv(np.asarray(T_base_depth, float))
    pc = np.asarray(c, float) @ Ti[:3, :3].T + Ti[:3, 3]
    fx, fy, cx, cy = K[0][0], K[1][1], K[0][2], K[1][2]
    for (x, y, z), rr in zip(pc, np.asarray(r, float) + reach):
        zn = z - rr
        if zn < 0.05:
            M[:] = True
            return M
        u, v, ru, rv = cx + fx * x / z, cy + fy * y / z, fx * rr / zn, fy * rr / zn
        u0, u1 = int(max(0, math.floor(u - ru - 1))), int(min(w, math.ceil(u + ru + 2)))
        v0, v1 = int(max(0, math.floor(v - rv - 1))), int(min(h, math.ceil(v + rv + 2)))
        if u0 < u1 and v0 < v1:
            M[v0:v1, u0:u1] = True
    return M


def analyze(depth_mm, K, T_base_depth, c, r, *, mask_m, near_m, floor_z=-math.inf, base_exclude=None, base_ring=None,
            speck_fn=None):
    """깊이 한 장 + 그때 팔 (구 c [S, 3], r [S] — 반지름 > 0 만) → 팔 지우기 점·팔 옆 살아남은 점.
    반환 dict: masked_px [m, 2] (v, u), masked_P [m, 3], near_px [k, 2], near_P [k, 3] (받침 m), near_d [k] (m), near_s [k] (구 번호),
    n: {valid, cand, masked, near, excluded_near (지도에서 빼는 점 중 팔 옆), speck}."""
    c = np.asarray(c, np.float32).reshape(-1, 3)
    r = np.asarray(r, np.float32).reshape(-1)
    reach = max(mask_m, near_m)
    cand = candidate_pixels(depth_mm.shape, K, T_base_depth, c, r, reach) & (depth_mm > 0)
    v, u = np.nonzero(cand)
    z = depth_mm[v, u].astype(np.float32) * 0.001
    fx, fy, cx, cy = K[0][0], K[1][1], K[0][2], K[1][2]
    T = np.asarray(T_base_depth, np.float32)
    P = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], 1).astype(np.float32) @ T[:3, :3].T + T[:3, 3]
    out = {'n': {'valid': int(np.count_nonzero(depth_mm)), 'cand': int(len(v))}}
    excl = np.zeros(len(v), bool)
    if speck_fn is not None:
        d2, n_sp = speck_fn(depth_mm)
        excl |= d2[v, u] == 0
        out['n']['speck'] = int(n_sp)
    excl |= P[:, 2] <= floor_z
    if base_exclude is not None:
        x0, y0, z0, x1, y1, z1 = base_exclude
        excl |= (P[:, 0] > x0) & (P[:, 0] < x1) & (P[:, 1] > y0) & (P[:, 1] < y1) & (P[:, 2] > z0) & (P[:, 2] < z1)
    if base_ring is not None:
        rr, zz = base_ring
        excl |= (P[:, 0] ** 2 + P[:, 1] ** 2 < rr * rr) & (P[:, 2] < zz)
    d, k = sphere_dist_near(P, c, r, reach)                 # (reach 밖 = inf — 지우기·빨강 모두 아님)
    m = d < mask_m                                          # cuRobo RobotSegmenter: 구 겉면 거리 < 문턱 → 지움
    nr = (~m) & (d < near_m)
    keep = nr & ~excl
    out['masked_px'] = np.stack([v[m], u[m]], 1)
    out['masked_P'] = P[m]
    order = np.argsort(d[keep], kind='stable')
    out['near_px'] = np.stack([v[keep], u[keep]], 1)[order]
    out['near_P'] = P[keep][order]
    out['near_d'] = d[keep][order]
    out['near_s'] = k[keep][order]
    out['n'].update(masked=int(m.sum()), near=int(keep.sum()), excluded_near=int((nr & excl).sum()))
    return out


def project(P_base, T_base_color, Kc, Dc=None):
    """받침 기준 점 [n, 3] → 컬러 영상 픽셀 [n, 2] (u, v) 와 카메라 앞 [n]."""
    import cv2
    P = np.asarray(P_base, np.float64).reshape(-1, 3)
    if len(P) == 0:
        return np.zeros((0, 2)), np.zeros(0, bool)
    Ti = np.linalg.inv(np.asarray(T_base_color, float))
    pc = P @ Ti[:3, :3].T + Ti[:3, 3]
    front = pc[:, 2] > 0.05
    uv, _ = cv2.projectPoints(pc, np.zeros(3), np.zeros(3), np.asarray(Kc, float),
                              np.asarray(Dc if Dc is not None else np.zeros(5), float))
    return uv.reshape(-1, 2), front


def overlay(color_bgr, a, T_base_color, Kc, Dc, link_names, sph_link, vox=None, title=''):
    """컬러 위 그림: 팔 지우기 점 파랑 (반투명), 팔 옆 살아남은 점 빨강, 링크마다 이름·최소 거리, 지도 칸 노랑 (vox [n, 3])."""
    import cv2
    img = color_bgr.copy()
    h, w = img.shape[:2]

    def layer(P, dil):
        L = np.zeros((h, w), np.uint8)
        uv, front = project(P, T_base_color, Kc, Dc)
        if len(uv):
            uu, vv = np.round(uv[:, 0]).astype(int), np.round(uv[:, 1]).astype(int)
            ok = front & (uu >= 0) & (uu < w) & (vv >= 0) & (vv < h)
            L[vv[ok], uu[ok]] = 255
            if dil > 1:
                L = cv2.dilate(L, np.ones((dil, dil), np.uint8))
        return L > 0, uv
    blue, _ = layer(a['masked_P'], 3)
    img[blue] = (0.5 * img[blue] + 0.5 * np.array([255, 80, 0])).astype(np.uint8)
    if vox is not None and len(vox):
        uv_v, fr = project(vox, T_base_color, Kc, Dc)
        for (x, y), f in zip(uv_v, fr):
            if f and 0 <= x < w and 0 <= y < h:
                cv2.rectangle(img, (int(x) - 3, int(y) - 3), (int(x) + 3, int(y) + 3), (0, 255, 255), 1)
    red, uv_r = layer(a['near_P'], 5)                  # (빨강을 노랑 위에 — 원본 점이 더 중요)
    img[red] = (0, 0, 255)
    if len(a['near_P']):
        sl = np.asarray(sph_link)[a['near_s']]
        links = np.array([link_names[int(s)] if int(s) < len(link_names) else '?' for s in sl])
        for ln in np.unique(links):                     # 링크마다 한 글 (구 여러 개가 한 링크)
            sel = links == ln
            x, y = np.median(uv_r[sel], 0)
            if not (np.isfinite(x) and np.isfinite(y)):
                continue
            txt = f'{ln} {a["near_d"][sel].min() * 1000:.0f}mm n={int(sel.sum())}'
            pos = (int(np.clip(x + 8, 0, w - 160)), int(np.clip(y, 14, h - 4)))
            cv2.putText(img, txt, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, txt, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    for i, line in enumerate([title, 'blue = arm mask, red = kept points near arm, yellow = map voxels near arm (trigger)']):
        cv2.putText(img, line, (6, 16 + 16 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, line, (6, 16 + 16 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def _r(x, n=4):
    return np.round(np.asarray(x, float), n).tolist()


def drop_specks(depth_mm, px=50, diff_mm=40):
    """rt_server.drop_specks 와 같음 (분석 프로세스는 rt_server 를 안 가져옴 — 값은 작업 파일로 받음)."""
    import cv2
    d = np.minimum(depth_mm, 32767).astype(np.int16)
    cv2.filterSpeckles(d, 0, px, diff_mm)
    gone = (d == 0) & (depth_mm > 0)
    return np.where(gone, 0, depth_mm).astype(depth_mm.dtype), int(gone.sum())


def decode_ros_color(raw):
    """컬러 sensor_msgs/Image 날 것 (bytes) → (찍힌 시각, BGR)."""
    import cv2
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image
    m = deserialize_message(raw, Image)
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
    img = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a.copy()
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, img


# ---------------- 분석 (일꾼 · python3 snap_log.py --analyze <폴더>) ----------------
def analyze_dir(d):
    """작업 파일 (_job.json · _job.npz · depth_XX.png · _raw/color_XX.*) → frame_XX.json · color_XX.jpg · overlay.jpg · snap.json.
    반환 요약 dict (서버가 로그 한 줄로)."""
    import cv2
    d = Path(d)
    t_w = time.perf_counter()
    job = json.loads((d / '_job.json').read_text(encoding='utf-8'))
    z = np.load(d / '_job.npz')
    p, cam, cfg = job['p'], job['cam'], job['cfg']
    cam = {k: (np.array(v, float) if v is not None and k != 'Dc' else v) for k, v in cam.items()}
    names = cfg['link_names']
    speck = (lambda D: drop_specks(D, *cfg['speck'])) if cfg.get('speck') else None
    # 컬러: 풀어서 찍힌 시각 차례로
    cols = []
    extra = d / '_colors.json'                      # (일꾼이 꺼내 둔 ROS 컬러)
    clist = job['colors'] + (json.loads(extra.read_text(encoding='utf-8')) if extra.exists() else [])
    for cf in clist:
        f = d / '_raw' / cf['file']
        try:
            if cf['fmt'] == 'ros':
                st, img = decode_ros_color(f.read_bytes())
            else:
                img = cv2.imread(str(f))
                st = cf['stamp']
        except Exception:
            continue
        cols.append([st, cf['recv'], img])
    cols.sort(key=lambda x: x[0])
    for j, c in enumerate(cols):
        cv2.imwrite(str(d / f'color_{j:02d}.jpg'), c[2], [cv2.IMWRITE_JPEG_QUALITY, SNAP_JPG_Q])
    cst = np.array([c[0] for c in cols]) if cols else np.zeros(0)
    C = z['C'] if 'C' in z else None
    R = z['R'] if 'R' in z else None
    Q = z['Q'] if 'Q' in z else np.zeros((0, 6))
    frames_meta, ana = [], []
    for i, fr in enumerate(job['frames']):
        depth = cv2.imread(str(d / fr['depth']), cv2.IMREAD_UNCHANGED)
        stamp = fr['stamp']
        j = int(np.argmin(np.abs(cst - stamp))) if len(cst) else None
        meta = {'i': i, 'n': fr['n'], 'stamp': round(stamp, 4), 'dt_trigger_ms': round((stamp - p['t']) * 1000, 1),
                'recv_lag_ms': round((fr['recv'] - stamp) * 1000, 1), 'depth': fr['depth'],
                'color': None if j is None else f'color_{j:02d}.jpg', 'color_dt_ms': None if j is None else round((cst[j] - stamp) * 1000, 1),
                'q_mask_deg': _r(np.degrees(Q[i]), 3) if i < len(Q) else None, 'map': job['infos'][i]}
        a = None
        if C is not None and depth is not None:
            keep = R[i] > 0
            sidx = np.nonzero(keep)[0]
            a = analyze(depth, cam['K'], cam['T_base_depth'], C[i][keep], R[i][keep], mask_m=cfg['mask_m'], near_m=cfg['near_m'],
                        floor_z=cfg['floor_z'], base_exclude=cfg['base_exclude'], base_ring=cfg['base_ring'], speck_fn=speck)
            a['sph'] = sidx
            s_all = sidx[a['near_s']]
            by = {}
            for s in np.unique(s_all):
                sel = s_all == s
                ln = names[int(s)] if int(s) < len(names) else '?'
                b = by.setdefault(ln, [0, math.inf])
                b[0] += int(sel.sum())
                b[1] = min(b[1], float(a['near_d'][sel].min()))
            n = min(SNAP_PTS_MAX, len(a['near_d']))
            meta.update(counts=a['n'], min_d_mm=round(float(a['near_d'][0]) * 1000, 1) if len(a['near_d']) else None,
                        near_by_link={k: {'n': v[0], 'min_mm': round(v[1] * 1000, 1)} for k, v in sorted(by.items())},
                        near_pts_cols=['x', 'y', 'z', 'd_mm', 'link'],
                        near_pts=[[*_r(P, 4), round(float(dd) * 1000, 1), names[int(s)] if int(s) < len(names) else '?']
                                  for P, dd, s in zip(a['near_P'][:n], a['near_d'][:n], s_all[:n])])
        (d / f'frame_{i:02d}.json').write_text(json.dumps(meta, ensure_ascii=False), encoding='utf-8')
        frames_meta.append({k: meta.get(k) for k in ('i', 'stamp', 'dt_trigger_ms', 'recv_lag_ms', 'color', 'color_dt_ms', 'min_d_mm',
                                                      'near_by_link')} | {'n_near': (meta.get('counts') or {}).get('near')})
        ana.append(a)
        time.sleep(SNAP_FRAME_PAUSE_S)
    # 그 순간 지도 칸 (칸마다 링크·거리 — 그 순간 관절의 구 = C 마지막)
    near = dict(job['near'] or {})
    vox = z['B'] if 'B' in z else np.zeros((0, 3))
    vox_rows = []
    if len(vox) and C is not None and job.get('near_q'):
        keep = R[-1] > 0
        dv, kv = sphere_dist(vox.astype(np.float32), C[-1][keep], R[-1][keep])
        sidx = np.nonzero(keep)[0]
        for P, dd, kk in sorted(zip(vox, dv, kv), key=lambda x: x[1])[:SNAP_VOX_MAX]:
            s = int(sidx[kk])
            vox_rows.append([*_r(P, 3), round(float(dd) * 1000, 1), names[s] if s < len(names) else '?'])
    # 겹친 그림: 트리거 직전 (없으면 가장 가까운) 영상
    ov = None
    if job['frames'] and len(cols):
        st = np.array([f['stamp'] for f in job['frames']])
        before = np.nonzero(st <= p['info'].get('t_frame', p['t']) + 1e-6)[0]
        i = int(before[-1]) if len(before) else int(np.argmin(np.abs(st - p['t'])))
        a = ana[i]
        if a is not None:
            j = int(np.argmin(np.abs(cst - st[i])))
            title = (f'{p["reason"]} {job["name"][:10]}  depth #{i} ({(st[i] - p["t"]) * 1000:+.0f} ms)  color dt '
                     f'{(cst[j] - st[i]) * 1000:+.0f} ms  near {a["n"]["near"]} pts'
                     + (f' min {a["near_d"][0] * 1000:.0f} mm' if len(a['near_d']) else ''))
            img = overlay(cols[j][2], a, cam['T_base_color'], cam['Kc'], cam.get('Dc'), names, a['sph'], vox=vox, title=title)
            cv2.imwrite(str(d / 'overlay.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, SNAP_JPG_Q])
            ov = i
    summ = {'reason': p['reason'], 'text': p['text'], 't': round(p['t'], 4),
            'time': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(p['t'])) + f'.{int((p["t"] % 1) * 1000):03d}',
            'info': p['info'], 'state': job['state'], 'mask_mm': cfg['mask_m'] * 1000, 'near_mm': cfg['near_m'] * 1000,
            'map_near': near | {'q_deg': job.get('near_q'), 'n_vox': int(len(vox))}, 'near_ms': p.get('near_ms'),
            'map_vox_cols': ['x', 'y', 'z', 'd_mm', 'link'], 'map_vox': vox_rows, 'overlay_frame': ov,
            'n_depth': len(job['frames']), 'n_color': len(cols), 'frames': frames_meta,
            'dump_s': job.get('dump_s'), 'analyze_s': round(time.perf_counter() - t_w, 2)}
    (d / 'snap.json').write_text(json.dumps(summ, ensure_ascii=False, indent=1, default=float), encoding='utf-8')
    for f in (d / '_raw').glob('*'):
        f.unlink()
    (d / '_raw').rmdir()
    if extra.exists():
        extra.unlink()
    mins = [(m['min_d_mm'], ln) for m in frames_meta if m['min_d_mm'] is not None
            for ln, v in (m['near_by_link'] or {}).items() if v['min_mm'] == m['min_d_mm']]
    best = min(mins) if mins else None
    return {'n_depth': len(job['frames']), 'n_color': len(cols), 'best': best, 'n_vox': int(len(vox)),
            'vox0': vox_rows[0][3:] if vox_rows else None, 'analyze_s': summ['analyze_s'], 'overlay': ov is not None}


# ---------------- 서버 쪽 기록기 ----------------
class SnapLog:
    """트리거 → (제어 스레드 poll) 그 순간 지도 칸 · 앞뒤 깊이 모으기 → (쓰기 스레드) 깊이 PNG·작업 파일 → (일꾼 프로세스) 컬러·분석·
    그림. 모든 시각은 벽시계 (깊이 헤더와 같음).
    feed: depth_between(t0, t1) → [(번호, 찍힌 시각, 받은 시각, 깊이)], color_src = 'ros' (일꾼이 ROS 컬러 토픽을 받음 —
          rt_server.DepthFeed) 또는 'feed' (시험: color_between(r0, r1) → [(받은 시각, (찍힌 시각, BGR))]).
    q_fn(시각) → 그 영상의 팔 지우기 관절 · spheres_fn(Q [n, 6]) → (중심 [n, S, 3], 반지름 [n, S]) numpy ·
    frame_info_fn(시각들) → [지도 기록 dict | None] · near_fn() → 그 순간 지도 칸 dict (q, raw, mpc, B, d_b, link) · state_fn() → dict.
    speck = (SPECK_PX, SPECK_DIFF_MM) 또는 None."""

    def __init__(self, feed, root, cam, *, mask_m, near_m, floor_z, base_exclude, base_ring, link_names, q_fn, spheres_fn,
                 frame_info_fn=None, near_fn=None, state_fn=None, speck=None, log=print, rec_list=None):
        self.feed, self.root = feed, Path(root)
        self.cam = {k: (None if v is None else np.asarray(v, float).tolist()) for k, v in cam.items()}
        self.cfg = {'mask_m': float(mask_m), 'near_m': float(near_m), 'floor_z': float(floor_z),
                    'base_exclude': None if base_exclude is None else list(base_exclude),
                    'base_ring': None if base_ring is None else list(base_ring), 'link_names': list(link_names),
                    'speck': None if speck is None else list(speck)}
        self.near_m = float(near_m)
        self.q_fn, self.spheres_fn, self.frame_info_fn, self.near_fn, self.state_fn = q_fn, spheres_fn, frame_info_fn, near_fn, state_fn
        self.log = log
        self.rec_list = rec_list                            # ctl.rec['snaps'] — 넘길 때 요약 한 줄 (제어 스레드)
        self._lock = threading.Lock()
        self._pending = []
        self._t_last = -math.inf
        self.n_taken = self.n_dropped = self.n_written = 0
        self._cap_said = False
        self._q = queue.Queue(maxsize=SNAP_QUEUE)
        self.poll_ms = []                                   # 제어 스레드에서 쓴 시간 (ms, 일이 있던 poll 만)
        self._sent = 0                                      # 일꾼에게 보낸 수 (끝낼 때 결과를 다 받았나)
        self._done = threading.Condition()
        self._n_res = 0
        self._ros = getattr(feed, 'color_src', 'ros') == 'ros'
        self._worker = subprocess.Popen(
            [sys.executable, '-u', '-W', 'ignore', str(Path(__file__).resolve()), '--worker'] + (['--ros-color'] if self._ros else []),
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, start_new_session=True, env=worker_env())
        threading.Thread(target=self._reader, daemon=True, name='snap_reader').start()
        self._th = threading.Thread(target=self._writer, daemon=True, name='snap_writer')
        self._th.start()

    # ---- 제어·다른 스레드 ----
    def trigger(self, reason, text='', t=None, **info):
        """판단한 순간 (t = 벽시계, 기본 지금). 간격·횟수 안이면 적어 두고 True. 어느 스레드든 (잠금 하나·계산 없음)."""
        t = time.time() if t is None else float(t)
        with self._lock:
            if self.n_taken >= SNAP_MAX:
                if not self._cap_said:
                    self._cap_said = True
                    self.log(f'이미지 기록: 이번 실행 {SNAP_MAX} 번 다 씀 — 더 안 남김')
                return False
            if t - self._t_last < SNAP_GAP_S:
                return False
            self._t_last = t
            self.n_taken += 1
            self._pending.append({'t': t, 'reason': reason, 'text': text, 'info': info, 'near': None, 'near_ms': None})
        return True

    def poll(self, final=False):
        """(제어 스레드) 주기마다: 무거운 일 하나만 — 그 순간 지도 칸 (near_fn) 또는 뒤 영상이 다 온 기록 넘기기. final = 끝 (다 넘김)."""
        with self._lock:
            if not self._pending:
                return
            p = self._pending[0]
        t0 = time.perf_counter()
        try:
            if p['near'] is None and self.near_fn is not None:
                try:
                    p['near'] = self.near_fn()
                except Exception as e:                      # (기록 실패로 제어를 멈추지 않음)
                    p['near'] = {'err': repr(e)}
                p['near_ms'] = round((time.perf_counter() - t0) * 1000, 2)
                if not final:
                    return
            if final or time.time() >= p['t'] + SNAP_POST_S + SNAP_WAIT_S:
                with self._lock:
                    self._pending.pop(0)
                self._handoff(p)
        finally:
            self.poll_ms.append(round((time.perf_counter() - t0) * 1000, 2))
            del self.poll_ms[:-2000]

    def _handoff(self, p):
        t = p['t']
        frames = self.feed.depth_between(t - SNAP_PRE_S, t + SNAP_POST_S)
        cands = [] if self._ros else self.feed.color_between(t - SNAP_PRE_S - 0.3, t + SNAP_POST_S + 0.5)
        colors, seen = [], set()                            # 깊이마다 받은 시각이 가장 가까운 컬러 (겹치면 한 번)
        if cands:
            rc = np.array([c[0] for c in cands])
            for f in frames:
                j = int(np.argmin(np.abs(rc - f[2])))
                if j not in seen:
                    seen.add(j)
                    colors.append(cands[j])
        Q = [np.asarray(self.q_fn(f[1]), float) for f in frames]
        near = dict(p['near'] or {})
        nq = near.pop('q', None)
        if nq is not None:
            Q.append(np.asarray(nq, float))
        C = R = None
        if Q:
            try:
                C, R = self.spheres_fn(np.stack(Q))
            except Exception as e:
                self.log(f'이미지 기록: 구 계산 실패 {e!r}')
        infos = self.frame_info_fn([f[1] for f in frames]) if self.frame_info_fn else [None] * len(frames)
        st = self.state_fn() if self.state_fn else {}
        name = time.strftime('%H%M%S', time.localtime(t)) + f'.{int((t % 1) * 1000):03d}_{p["reason"]}'
        B = np.asarray(near.pop('B', np.zeros((0, 3))), float).reshape(-1, 3)
        job = {'p': {k: p[k] for k in ('t', 'reason', 'text', 'info', 'near_ms')}, 'name': name, 'frames': frames, 'colors': colors,
               'Q': np.array(Q).reshape(-1, 6), 'C': C, 'R': R, 'B': B, 'infos': infos, 'state': st, 'near': near,
               'near_q': None if nq is None else np.degrees(np.asarray(nq, float)).round(3).tolist()}
        if self.rec_list is not None:
            self.rec_list.append({'t': round(t, 3), 'reason': p['reason'], 'dir': f'snap/{name}', 'n_depth': len(frames)})
        try:
            self._q.put_nowait(job)
        except queue.Full:
            self.n_dropped += 1
            self.log(f'이미지 기록: 쓰기가 밀려 {name} 을 버림 (버린 수 {self.n_dropped})')

    def stats(self):
        """기록 요약 (rt_run.json snap_stats): 받은·쓴·버린 수, 제어 스레드 poll 시간 (ms)."""
        p = np.asarray(self.poll_ms, float)
        return {'taken': self.n_taken, 'written': self.n_written, 'dropped': self.n_dropped, 'polls_busy': int(len(p)),
                'poll_ms_p50': round(float(np.percentile(p, 50)), 2) if len(p) else None,
                'poll_ms_p99': round(float(np.percentile(p, 99)), 2) if len(p) else None,
                'poll_ms_max': round(float(p.max()), 2) if len(p) else None}

    def close(self, timeout=SNAP_CLOSE_S):
        """끝: 남은 트리거를 지금 있는 영상으로 넘기고 쓰기·분석을 기다린 뒤 일꾼을 끔 (제어 스레드 — 저장 전)."""
        while True:
            with self._lock:
                if not self._pending:
                    break
            self.poll(final=True)
        t_end = time.monotonic() + timeout
        while self._q.unfinished_tasks and time.monotonic() < t_end:
            time.sleep(0.05)
        try:
            self._q.put(None, timeout=1.0)
        except queue.Full:
            pass
        with self._done:
            self._done.wait_for(lambda: self._n_res >= self._sent or self._worker.poll() is not None,
                                max(0.0, t_end - time.monotonic()))
        if self._n_res < self._sent:
            self.log(f'이미지 기록: 끝낼 때 분석 {self._sent - self._n_res} 개를 못 기다림 (깊이 원본·작업 파일은 남음 — '
                     f'python3 snap_log.py --analyze <폴더> 로 다시)')
        try:
            self._worker.stdin.close()                      # 일꾼: 입력 끝 → 남은 일 뒤 끝
            self._worker.wait(5.0)
        except (OSError, subprocess.TimeoutExpired):
            self._worker.kill()

    # ---- 쓰기 스레드 (파일만) → 분석 프로세스 ----
    def _writer(self):
        while True:
            job = self._q.get()
            try:
                if job is None:
                    return
                frames = self._dump(job)
                self._worker.stdin.write(json.dumps({'dir': str(self.root / job['name']),
                                                     'depth_recv': [f['recv'] for f in frames]}) + '\n')
                self._worker.stdin.flush()
                self._sent += 1
            except Exception as e:
                self.log(f'이미지 기록 쓰기 오류 ({job["name"] if job else "?"}): {e!r}')
            finally:
                self._q.task_done()

    def _dump(self, job):
        """깊이 PNG·컬러 날 것·작업 파일 (cv2·파일 쓰기 — 대부분 GIL 을 놓음)."""
        import cv2
        t0 = time.perf_counter()
        d = self.root / job['name']
        (d / '_raw').mkdir(parents=True, exist_ok=True)
        frames = []
        for i, (n, stamp, recv, depth) in enumerate(job['frames']):
            cv2.imwrite(str(d / f'depth_{i:02d}.png'), depth, [cv2.IMWRITE_PNG_COMPRESSION, SNAP_PNG_LEVEL])
            frames.append({'n': int(n), 'stamp': float(stamp), 'recv': float(recv), 'depth': f'depth_{i:02d}.png'})
        colors = []                                         # (시험 feed 만 — ROS 컬러는 일꾼이 꺼냄)
        for j, (recv, raw) in enumerate(job['colors']):
            f = f'fcolor_{j:02d}.png'
            cv2.imwrite(str(d / '_raw' / f), raw[1])
            colors.append({'file': f, 'fmt': 'img', 'recv': float(recv), 'stamp': float(raw[0])})
        arrs = {'Q': job['Q'], 'B': job['B']}
        if job['C'] is not None:
            arrs.update(C=np.asarray(job['C'], np.float32), R=np.asarray(job['R'], np.float32))
        np.savez(d / '_job.npz', **arrs)
        meta = {k: job[k] for k in ('p', 'name', 'infos', 'state', 'near_q')} | {
            'frames': frames, 'colors': colors, 'cam': self.cam, 'cfg': self.cfg, 'dump_s': round(time.perf_counter() - t0, 3),
            'near': {k: v for k, v in job['near'].items() if isinstance(v, (int, float, str, type(None)))}}
        (d / '_job.json').write_text(json.dumps(meta, ensure_ascii=False, default=float), encoding='utf-8')
        return frames

    def _reader(self):
        """일꾼 결과 한 줄마다 → 로그 한 줄."""
        for line in self._worker.stdout:
            try:
                s = json.loads(line)
            except ValueError:
                continue
            if 'ready' in s:
                self.log(f'이미지 기록 일꾼 준비 (컬러: {"ROS 토픽 받는 중" if s["ready"] else "시험 · 또는 ROS 못 켬"})')
                continue
            name = Path(s.get('dir', '?')).name
            if 'err' in s:
                self.log(f'이미지 기록 분석 실패 ({name}): {s["err"][-300:]} — 깊이 원본·작업 파일은 남음')
            else:
                self.n_written += 1
                best, v0 = s.get('best'), s.get('vox0')
                self.log(f'이미지 기록 [{name.split("_", 1)[-1]}] snap/{name}: 깊이 {s["n_depth"]} 장·컬러 {s["n_color"]} 장, 팔 옆 '
                         f'{self.near_m * 100:.0f} cm 안 살아남은 점 ' + (f'최소 {best[0]:.0f} mm ({best[1]})' if best else '없음')
                         + f', 그 순간 지도 칸 {s["n_vox"]} 개' + (f' (가장 가까운 {v0[0]:.0f} mm {v0[1]})' if v0 else '')
                         + f' — 분석 {s["analyze_s"]:.1f} s')
            with self._done:
                self._n_res += 1
                self._done.notify_all()
        with self._done:
            self._done.notify_all()


class ColorRing:
    """(일꾼) 컬러 토픽을 날 것 (bytes) 으로 받아 최근 SNAP_COLOR_N 장 (받은 시각, 날 것)."""

    def __init__(self):
        import collections
        self.ring = collections.deque(maxlen=SNAP_COLOR_N)
        self.lock = threading.Lock()
        self.ok = threading.Event()
        threading.Thread(target=self._spin, daemon=True, name='color_ring').start()

    def _spin(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from camera_io import CAM_TOPIC
        ctx = Context()
        rclpy.init(context=ctx)
        node = Node('cr3a_snap_color', context=ctx)
        node.create_subscription(Image, f'{CAM_TOPIC}/color/image_raw', self._on, qos_profile_sensor_data, raw=True)
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        self.ok.set()
        try:
            ex.spin()
        except Exception:
            pass

    def _on(self, data):
        with self.lock:
            self.ring.append((time.time(), data))

    def pick(self, recvs):
        """깊이 받은 시각마다 받은 시각이 가장 가까운 컬러 (겹치면 한 번) [(받은 시각, 날 것)]."""
        with self.lock:
            cands = list(self.ring)
        if not cands:
            return []
        rc = np.array([c[0] for c in cands])
        out, seen = [], set()
        for r in recvs:
            j = int(np.argmin(np.abs(rc - r)))
            if j not in seen:
                seen.add(j)
                out.append(cands[j])
        return out


def _worker(ros_color):
    """일꾼: 입력 줄 {'dir', 'depth_recv'} 마다 (ROS 면) 컬러를 바로 꺼내 _raw 에 두고 → 분석 스레드가 analyze_dir → 결과 한 줄.
    입력이 닫히면 남은 분석 뒤 끝."""
    try:
        os.nice(SNAP_NICE)
    except OSError:
        pass
    _one_thread()
    out_lock = threading.Lock()

    def say(obj):
        with out_lock:
            print(json.dumps(obj, ensure_ascii=False, default=float), flush=True)
    ring = None
    if ros_color:
        try:
            ring = ColorRing()
            ring.ok.wait(20.0)
        except Exception:
            ring = None
    say({'ready': bool(ring is not None and ring.ok.is_set())})
    q = queue.Queue()

    def work():
        while True:
            d = q.get()
            if d is None:
                return
            try:
                say({'dir': d} | analyze_dir(d))
            except Exception as e:
                import traceback
                say({'dir': d, 'err': f'{e!r} {traceback.format_exc()[-800:]}'})
    th = threading.Thread(target=work, name='analyze')
    th.start()
    for line in sys.stdin:
        try:
            req = json.loads(line)
        except ValueError:
            continue
        d = Path(req['dir'])
        if ring is not None:
            try:
                cols = []
                (d / '_raw').mkdir(parents=True, exist_ok=True)
                for j, (recv, raw) in enumerate(ring.pick(req.get('depth_recv') or [])):
                    f = f'color_{j:02d}.bin'
                    (d / '_raw' / f).write_bytes(bytes(raw))
                    cols.append({'file': f, 'fmt': 'ros', 'recv': float(recv)})
                (d / '_colors.json').write_text(json.dumps(cols), encoding='utf-8')
            except Exception as e:                      # (컬러 없이 분석 — 결과 줄은 하나만)
                print(f'컬러 꺼내기 실패 {d}: {e!r}', file=sys.stderr)
        q.put(str(d))
    q.put(None)
    th.join()
    sys.stdout.flush()
    os._exit(0)                                         # (rclpy 받기 스레드를 기다리지 않음)


def _main():
    if len(sys.argv) >= 2 and sys.argv[1] == '--worker':
        sys.path.insert(0, str(Path(__file__).resolve().parent))      # camera_io
        _worker('--ros-color' in sys.argv[2:])
        return 0
    if len(sys.argv) == 3 and sys.argv[1] == '--analyze':
        try:
            os.nice(SNAP_NICE)
        except OSError:
            pass
        _one_thread()
        print(json.dumps(analyze_dir(sys.argv[2]), ensure_ascii=False, default=float))
        return 0
    print(__doc__)
    return 2


if __name__ == '__main__':
    sys.exit(_main())
