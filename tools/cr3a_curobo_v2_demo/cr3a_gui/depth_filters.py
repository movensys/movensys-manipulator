#!/usr/bin/env python3
"""[2026-10-28 디버깅 D-4] 실제 카메라 깊이 거르기 — 지도에 쌓기 전 (gui_controller._real_frame), 실제 카메라만.

원인 (작업기록 D-4, 이미지 기록 gui_real_20261028_150253/snap): D455 스테레오 가림 띠. 깊이는 왼쪽 카메라 기준이라 가까운 물체 (로봇)
  옆에 오른쪽 카메라가 못 보는 띠 (폭 = 기준선 × fx × (1/z_로봇 − 1/z_배경) 픽셀) 가 생기고, 보통은 깊이 없음 (0) 인데 가끔 로봇 깊이로
  채워짐 → 그 점이 팔 지우기 (구 겉면 4 cm) 바로 밖이라 지도에 들어가고, 경계 채우기가 MPC 에 '닿음' 으로 보여 줌 (허공 비킴).
(B) 가림 띠 지우기 shadow_band: 로봇 픽셀 (팔 지우기 픽셀) 의 띠 쪽 옆 기준선·fx/z_로봇 픽셀 (배경이 무한히 먼 경우 = 가장 넓은 띠, 3D
  로 옆 9.5 cm) 안에서 깊이가 그 로봇 픽셀 깊이 ± tol 인 픽셀을 지움.
  ⚠️ 사각지대: 로봇 옆 (띠 쪽) 9.5 cm 안에 로봇과 같은 깊이 (± tol) 로 있는 실제 물체도 지움 (다른 쪽·더 가깝거나 먼 물체는 그대로).
  behind (B+, CR3A_DEPTH_BAND=behind): 로봇 깊이 − tol 보다 뒤 모두 — 151116 (띠 안 섬이 옆 Link1 깊이로 채워짐) 까지 지우지만 사각지대가
  '로봇 왼쪽 띠 폭 안의 로봇 뒤 모든 것' 으로 넓어짐 (가짜 물체 시험: 로봇 뒤를 옆으로 지나는 물체 픽셀 25 % 만 남음).
  behind_adaptive (기본, 2026-10-28 사용자 "b+ 방법을 넣어줘" + lead 적응 폭): B+ 인데 띠 폭을 그 줄 배경 거리로 (shadow_band 설명) —
  사진 13 개 지도 점검 걸림 B+ 와 같이 0 (A' 4/5 와 함께), 로봇 뒤 물체 (가림 지킴 가짜 물체) 남은 픽셀: 서 있음 46 → 90 %, 다가옴
  0.5 m/s 53 → 79 %, 옆으로 0.5 m/s 28 → 34 %, 지도에 잡히는 장은 거르기 없음과 같음. 실제 점 (로봇 15 cm 밖) 96.6~98.7 % 남음.
(A) 시간 확인 temporal_keep: 지난 n 장 중 k 장 이상에서 깊이가 있고 (tol 이 있으면 그 픽셀 최근 가운데값 ± tol 안) 지금도 그렇다면
  남김. tol = None (A', 기본) 이면 '깊이 있음' 만 셈 → 늦음은 깊이가 없던 자리에 나타난 물체만 (k − 1) 장 (15 fps: × 67 ms), 배경 깊이가
  있던 자리 (책상·벽 앞) 는 늦음 없음. ⚠️ tol = 20 mm (가운데값 비교) 는 쓰지 말 것: 0.5 m/s 로 다가오는 물체 픽셀이 0.2 % 만 남고
  (깊이가 한 장에 33 mm 씩 바뀜) 옆으로 1 m/s 는 0 % — 2026-10-28 tools/d4_band_eval.py delay. GPU 시간도 6.4 ms (A' 0.2 ms).
모든 함수는 torch (GPU·CPU 같음), 깊이 = m (0 = 없음) [높이, 너비].
"""
import collections
import os

import torch

BASELINE_M = 0.095          # D455 스테레오 기준선
BAND_SIDE = 'left'          # 가림 띠가 로봇의 어느 쪽 (깊이 영상 기준) — 2026-10-28 이미지 기록에서 잼 (tools/d4_band_eval.py side)
BAND_TOL_M = 0.03           # 띠 안 픽셀 깊이가 로봇 깊이 ± 이것이면 지움
BAND_MODE = 'behind_adaptive_2d'   # BAND_MODES (아래) — 2026-10-28 사용자 "b+ 방법을 넣어줘" + lead 적응 폭, 그 뒤 사용자 "위아래 30줄 기준 … 실물에 반영해 줘" (2D 기준 — 실물 164808·164815·164906 헛방 3 번 모두 지움, 실제 점 98.3 % 그대로 — 작업기록 D-4)
TEMPORAL_KN = (4, 5)        # (k, n): 지난 n 장 중 k 장 (tools/d4_band_eval.py: 4/5 가 3/4 보다 151116 유령을 더 지움, 늦음은 배경 깊이가 없던 자리만 +67 ms)
TEMPORAL_TOL_M = None       # None = 깊이 있음만 셈 (A' — 위 설명)


BAND_PAD_PX = 3             # (적응 폭) 계산한 띠 폭에 더하는 여유: 픽셀 + 로봇 거리에서 BAND_PAD_M (스테레오 정합 창·가장자리 잡음)
BAND_PAD_M = 0.01
BAND_BG_SPAN_PX = 80        # (적응 폭) 가장 넓은 띠 밖에서 배경을 찾는 거리 — 이 안에 깊이 있는 배경이 없으면 가장 넓은 띠 그대로
BAND_REF_ROWS = 30          # (2D 기준) 위아래 이만큼 줄 안의 로봇 깊이 최소를 기준으로 (tools/d4_band_eval.py — 15·30·45 비교)
BAND_REF_BUCKETS_PX = (48, 64, 96, 128)     # (2D 기준) 오른쪽으로 보는 폭 (가장 넓은 띠를 이 중 같거나 큰 것으로 올림 — 풀링 몇 번만)


def _robot_min_rows(rd, rows):
    """rd [h, w] (로봇 픽셀 깊이, 나머지 inf) → 픽셀마다 줄 y−rows..y+rows 안의 최소 (세로 최소 풀링)."""
    import torch.nn.functional as F
    m = F.max_pool2d(F.pad(-rd[None, None], (0, 0, rows, rows), value=-float('inf')), kernel_size=(2 * rows + 1, 1), stride=1)
    return -m[0, 0]


def _min_right(v, reach):
    """v [h, w] → 픽셀마다 열 x..x+reach 안의 최소 (가로 최소 풀링). 세로 → 가로 차례로 하면 네모 안 최소 (세로는 한 번만)."""
    import torch.nn.functional as F
    m = F.max_pool2d(F.pad(-v[None, None], (0, reach), value=-float('inf')), kernel_size=(1, reach + 1), stride=1)
    return -m[0, 0]


def shadow_band(depth, robot, fx, side=BAND_SIDE, baseline=BASELINE_M, tol_m=BAND_TOL_M, behind=False, adaptive=False, ref_rows=0):
    """지울 픽셀 bool [h, w]: 로봇 아닌 픽셀 중, 그 줄에서 side 쪽으로 가장 가까운 로봇 픽셀까지 ≤ 띠 폭이고 깊이가 z_로봇 ± tol_m
    (behind = True 면 z_로봇 − tol_m 이상 = 로봇과 같거나 뒤 — 띠 안 깊이는 무엇이든 지어낸 값일 수 있음: 2026-10-28 151116 은 띠 안
    섬이 가림 로봇 (600 mm) 이 아니라 옆 Link1 깊이 (868 mm) 로 채워짐). side = 'left' (띠가 로봇 왼쪽 — 가장 가까운 로봇 픽셀을 오른쪽
    에서 찾음) · 'right' · 'both'.
    띠 폭: 기본 = 가장 넓은 띠 baseline·fx/z_로봇 (배경이 무한히 멂). adaptive = True (2026-10-28 lead 안 — 로봇 바로 뒤 실제 물체가 B+
    에 지워지지 않게): 가장 넓은 띠 바깥 (로봇에서 먼 쪽) 으로 BAND_BG_SPAN_PX 안의 가장 가까운 깊이 있는 로봇 아닌 픽셀을 배경 z_bg 로
    보고 (로봇 깊이 − tol_m 보다 뒤일 때만) 폭 = baseline·fx·(1/z_로봇 − 1/z_bg) + BAND_PAD_PX + BAND_PAD_M·fx/z_로봇 (가장 넓은 띠 넘지
    않음). 띠 안의 섬은 배경 찾기에서 건너뜀 (가장 넓은 띠 밖에서부터 찾으므로). 배경이 없거나 로봇보다 앞이면 가장 넓은 띠.
    ref_rows > 0 (2D 기준, 2026-10-28 lead 안 — 실물 164906·164808): 지우는 깊이 기준을 같은 줄 가장자리 깊이 대신 위아래 ref_rows 줄 ·
    오른쪽 가장 넓은 띠 안의 로봇 깊이 최소로 (그 줄 가장자리보다 가까우면). 띠가 옆 줄의 더 가까운 로봇 부분 깊이로 채워져 (그리퍼 앞
    섬 540~556 mm, 같은 줄 가장자리 600~640 mm) B+ 가 남기던 것까지 지움. 사각지대: 띠 안에서 그 둘레 가장 가까운 로봇 부분 깊이 −
    tol 보다 뒤인 실제 물체 (그보다 앞 물체는 남음)."""
    if side == 'both':
        return (shadow_band(depth, robot, fx, 'left', baseline, tol_m, behind, adaptive, ref_rows)
                | shadow_band(depth, robot, fx, 'right', baseline, tol_m, behind, adaptive, ref_rows))
    if side == 'right':     # 오른쪽 띠 = 좌우를 뒤집은 영상의 왼쪽 띠
        return torch.flip(shadow_band(torch.flip(depth, [1]), torch.flip(robot, [1]), fx, 'left', baseline, tol_m, behind, adaptive,
                                      ref_rows), [1])
    h, w = depth.shape
    col = torch.arange(w, device=depth.device).expand(h, w)
    # 띠는 로봇 왼쪽 → 픽셀마다 오른쪽 (같거나 큰 열) 의 가장 가까운 로봇 열 = 그 로봇 덩어리의 왼쪽 가장자리
    idx = torch.where(robot, col, torch.full_like(col, w))
    nr = torch.flip(torch.cummin(torch.flip(idx, [1]), dim=1).values, [1])
    has = nr < w
    nr_c = nr.clamp(0, w - 1)
    zr = torch.gather(depth, 1, nr_c)                       # 그 로봇 픽셀 깊이
    dist = (nr_c - col).float()
    width = baseline * fx / zr.clamp(min=1e-3)              # 가장 넓은 띠 (배경 무한)
    zref = zr
    if ref_rows > 0:                                        # 2D 기준: 둘레 로봇 깊이 최소 (오른쪽 가장 넓은 띠 · 위아래 ref_rows 줄)
        rv = _robot_min_rows(torch.where(robot & (depth > 0), depth, torch.full_like(depth, float('inf'))), int(ref_rows))
        reach = torch.ceil(width)
        lo = 0.0
        for k, b in enumerate(BAND_REF_BUCKETS_PX):
            last = k == len(BAND_REF_BUCKETS_PX) - 1
            sel = (reach > lo) & ((reach <= b) | last)
            zref = torch.where(sel, torch.minimum(zref, _min_right(rv, b)), zref)
            lo = float(b)
    if adaptive:
        bgc = (depth > 0) & ~robot                          # 배경 후보: 깊이 있는 로봇 아닌 픽셀
        lv = torch.cummax(torch.where(bgc, col, torch.full_like(col, -1)), dim=1).values    # 그 열 또는 왼쪽의 가장 가까운 후보
        s = nr_c - torch.ceil(width).long() - 1             # 가장 넓은 띠 바로 바깥 열
        c = torch.gather(lv, 1, s.clamp(0, w - 1))
        ok = (s >= 0) & (c >= 0) & (s - c <= BAND_BG_SPAN_PX)
        zb = torch.gather(depth, 1, c.clamp(0, w - 1))
        ok = ok & (zb >= zr - tol_m)
        w_ad = baseline * fx * (1.0 / zr.clamp(min=1e-3) - 1.0 / zb.clamp(min=1e-3)).clamp(min=0.0) \
            + BAND_PAD_PX + BAND_PAD_M * fx / zr.clamp(min=1e-3)
        width = torch.where(ok, torch.minimum(w_ad, width), width)
    same = (depth >= zref - tol_m) if behind else ((depth - zref).abs() <= tol_m)
    return has & ~robot & (depth > 0) & (zr > 0) & (dist <= width) & same


def temporal_keep(stack, k, tol_m=TEMPORAL_TOL_M):
    """stack [n, h, w] (마지막 = 지금 장, 0 = 깊이 없음·빠진 장) → 지금 장에서 남길 픽셀 bool [h, w].
    tol_m = None: 지금 깊이가 있고 n 장 중 k 장 이상 깊이가 있음. tol_m (m): 거기에 더해 그 픽셀의 n 장 가운데값 ± tol_m 안인 장으로 세고
    지금 장도 그 안."""
    valid = stack > 0
    cur = stack[-1]
    if tol_m is None:
        return valid[-1] & (valid.sum(0) >= k)
    med = torch.nanmedian(torch.where(valid, stack, torch.full_like(stack, float('nan'))), dim=0).values
    cons = valid & ((stack - med).abs() <= tol_m)
    return cons[-1] & (cons.sum(0) >= k) & (cur > 0)


class TemporalFilter:
    """(서버) 카메라 영상 번호로 지난 n 장을 GPU 에 둠. 빠진 번호 (서버가 건너뛴 영상) 는 push 로 채우고, 끝내 없으면 '깊이 없음' 장."""

    def __init__(self, k, n, tol_m=TEMPORAL_TOL_M):
        assert 1 <= k <= n
        self.k, self.n, self.tol_m = int(k), int(n), tol_m
        self.hist = collections.OrderedDict()               # 번호 → 깊이 (m, GPU)

    def push(self, num, depth):
        self.hist[int(num)] = depth
        for key in [key for key in self.hist if key <= int(num) - self.n]:
            del self.hist[key]

    def has(self, num):
        return int(num) in self.hist

    def keep(self, num, depth):
        """번호 num 의 깊이 (이미 push 했어도 됨) → 남길 픽셀 bool."""
        self.push(num, depth)
        z = torch.zeros_like(depth)
        stack = torch.stack([self.hist.get(m, z) for m in range(int(num) - self.n + 1, int(num) + 1)])
        return temporal_keep(stack, self.k, self.tol_m)


def robot_mask_from_spheres(depth, K, T_base_depth, c, r, mask_m, chunk=20000):
    """(시험·분석) 깊이 픽셀 중 구 (중심 c [S, 3], 반지름 r [S] > 0) 겉면에서 mask_m 안 = 팔 지우기 픽셀 bool [h, w]
    (cuRobo RobotSegmenter 와 같은 식). 서버는 segmenter 의 것을 씀."""
    h, w = depth.shape
    dev = depth.device
    v, u = torch.nonzero(depth > 0, as_tuple=True)
    z = depth[v, u]
    K = torch.as_tensor(K, dtype=torch.float32, device=dev)
    T = torch.as_tensor(T_base_depth, dtype=torch.float32, device=dev)
    pc = torch.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], 1)
    P = pc @ T[:3, :3].T + T[:3, 3]
    c = torch.as_tensor(c, dtype=torch.float32, device=dev)
    r = torch.as_tensor(r, dtype=torch.float32, device=dev)
    d = torch.empty(len(P), device=dev)
    for i in range(0, len(P), chunk):                    # (cdist 'donot_use_mm' 은 Thor 에서 50~80 배 느림 — R-1 교훈)
        d[i:i + chunk] = (((P[i:i + chunk, None, :] - c[None]) ** 2).sum(-1).sqrt() - r).min(1).values
    out = torch.zeros((h, w), dtype=torch.bool, device=dev)
    out[v[d < mask_m], u[d < mask_m]] = True
    return out


BAND_MODES = {'same': (False, False, 0), 'behind': (True, False, 0), 'behind_adaptive': (True, True, 0),
              'behind_adaptive_2d': (True, True, None)}     # 이름: (behind, adaptive, ref_rows — None = BAND_REF_ROWS·CR3A_DEPTH_BAND_ROWS)


def settings_from_env():
    """서버 설정: CR3A_DEPTH_TEMPORAL = 'k,n' (기본 TEMPORAL_KN) · '0' 끔, CR3A_DEPTH_TEMPORAL_TOL_MM = mm (기본 TEMPORAL_TOL_M — 없음 =
    깊이 있음만), CR3A_DEPTH_BAND = same (로봇 깊이 ± BAND_TOL_M) · behind (B+: 로봇 깊이 − BAND_TOL_M 보다 뒤 모두, 가장 넓은 띠) ·
    behind_adaptive (B+ 를 배경에 맞춘 띠 폭으로) · behind_adaptive_2d (거기에 2D 기준 — CR3A_DEPTH_BAND_ROWS 줄, 기본
    BAND_REF_ROWS) · 0 끔 (기본 BAND_MODE), CR3A_DEPTH_BAND_SIDE = left|right|both.
    반환 (temporal (k, n, tol) | None, band (side, tol, 모드 이름) | None)."""
    t = os.environ.get('CR3A_DEPTH_TEMPORAL', f'{TEMPORAL_KN[0]},{TEMPORAL_KN[1]}').strip()
    temporal = None
    if t not in ('', '0'):
        k, n = (int(x) for x in t.split(','))
        tol = os.environ.get('CR3A_DEPTH_TEMPORAL_TOL_MM')
        temporal = (k, n, (float(tol) / 1000.0 if tol else TEMPORAL_TOL_M))
    mode = os.environ.get('CR3A_DEPTH_BAND', BAND_MODE).strip()
    band = None
    if mode not in ('', '0'):
        assert mode in BAND_MODES, f'CR3A_DEPTH_BAND = {" | ".join(BAND_MODES)} | 0 (받은 값 {mode})'
        band = (os.environ.get('CR3A_DEPTH_BAND_SIDE', BAND_SIDE).strip(), BAND_TOL_M, mode)
    return temporal, band


def band_kw(mode):
    """모드 이름 → shadow_band 인자 {behind, adaptive, ref_rows} (2D 줄 수는 CR3A_DEPTH_BAND_ROWS 가 있으면 그것)."""
    behind, adaptive, rows = BAND_MODES[mode]
    if rows is None:
        rows = int(os.environ.get('CR3A_DEPTH_BAND_ROWS', BAND_REF_ROWS))
    return {'behind': behind, 'adaptive': adaptive, 'ref_rows': rows}


def band_lateral_m(baseline=BASELINE_M):
    """가림 띠 지우기가 로봇 옆으로 지우는 최대 폭 (3D, m) = 기준선 (픽셀 폭 × z / fx)."""
    return baseline

