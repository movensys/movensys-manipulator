#!/usr/bin/env python3
"""[만듦 2026-10-16] 끝단 공구 (브래킷 + D455 + 그리퍼) 를 '몇 개의 큰 구' 로 빈틈없이 덮는 충돌 구 (사용자 결정 2026-10-16:
"그냥 그리퍼를 덮은 몇개의 큰 구로 만들면 안되냐"). 팔 링크 구 (cr3a_sim/cr3a_curobo.yml 의 60 개) 는 그대로.

실행 (PC 또는 Thor, numpy·scipy 만):  python cover_spheres.py [구 개수 ...]   → tool_cover_spheres.json (기본 개수 = DEFAULT_K)
방법: 세 부품의 표면 표본 점 (Link6 좌표, 정확한 메시) 을 k-means 로 K 묶음 → 묶음마다 최소 포함 구 (Bădoiu–Clarkson 반복) →
      점을 '반지름이 가장 덜 늘어나는 구' 로 다시 나누며 반복 (덮개 다듬기). 마지막 반지름 = 맡은 점까지 최대 거리 + COVER_PAD
      → 모든 표면 점이 어떤 구 안 (덮개 밖 0). 구 → 부품 이름 = 그 구가 맡은 점이 가장 많은 부품.
덮개 두께 (보수적인 정도) = 구 합집합 겉면 점 ↔ 실제 표면 가장 가까운 거리 (중앙·95 %·최대).
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.cluster.vq import kmeans2
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from make_tool_meshes import read_stl, sample  # noqa: E402

PARTS = ['tool_bracket', 'tool_camera', 'tool_gripper']
DEFAULT_K = 6
COVER_PAD = 0.001           # 표본 점 사이 빈틈 (표본 간격 ~1 mm) 을 덮는 여유 (m)


def min_sphere(P, iters=400):
    """점들의 최소 포함 구 근사 (Bădoiu–Clarkson): 중심을 가장 먼 점 쪽으로 1/(i+1) 씩."""
    c = P.mean(0)
    for i in range(iters):
        far = P[np.argmax(np.linalg.norm(P - c, axis=1))]
        c = c + (far - c) / (i + 2)
    return c, float(np.linalg.norm(P - c, axis=1).max())


def cover(P, K, seed=0, rounds=12):
    best = None
    for s in range(8):                                    # k-means 시작점 여러 번 → 가장 작은 '반지름 세제곱 합' (부피)
        _, lab = kmeans2(P, K, minit='++', seed=seed + s)
        for _ in range(rounds):
            C, R = [], []
            for k in range(K):
                Q = P[lab == k]
                if len(Q) == 0:
                    continue
                c, r = min_sphere(Q)
                C.append(c)
                R.append(r)
            C, R = np.array(C), np.array(R)
            # 점 → 반지름이 가장 덜 늘어나는 구 (이미 안이면 0)
            D = np.linalg.norm(P[:, None, :] - C[None], axis=-1)
            grow = np.maximum(D - R[None], 0) + 1e-9 * D
            new = np.argmin(grow, axis=1)
            if np.array_equal(new, lab):
                break
            lab = new
        C, R = [], []
        for k in np.unique(lab):
            c, r = min_sphere(P[lab == k])
            C.append(c)
            R.append(r)
        C, R = np.array(C), np.array(R)
        D = np.linalg.norm(P[:, None, :] - C[None], axis=-1)
        lab = np.argmin(D - R[None], axis=1)              # 덮는 구 중 하나로 (이미 덮였으면 아무 구)
        R = np.array([max(R[k], np.linalg.norm(P[lab == k] - C[k], axis=1).max() if (lab == k).any() else 0) for k in range(len(C))])
        vol = float((R ** 3).sum())
        if best is None or vol < best[0]:
            best = (vol, C, R, lab)
    return best[1], best[2] + COVER_PAD, best[3]


def thickness(C, R, Psurf, n_per=4000, seed=1):
    """구 합집합 겉면 (다른 구 안이 아닌 부분) 점 → 실제 표면까지 거리 (m)"""
    rng = np.random.default_rng(seed)
    S = []
    for c, r in zip(C, R):
        v = rng.normal(size=(n_per, 3))
        v /= np.linalg.norm(v, axis=1, keepdims=True)
        S.append(c + r * v)
    S = np.concatenate(S)
    inside_other = np.min(np.linalg.norm(S[:, None, :] - C[None], axis=-1) - R[None], axis=1) < -1e-6
    S = S[~inside_other]
    return cKDTree(Psurf).query(S)[0]


def main():
    Ks = [int(a) for a in sys.argv[1:]] or [DEFAULT_K]
    pts, owner = [], []
    for i, ln in enumerate(PARTS):
        T = read_stl(HERE / 'tool' / 'meshes' / f'{ln}.STL')
        area = 0.5 * np.linalg.norm(np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]), axis=1).sum()
        n = int(area / 1e-6)                              # 1 mm² 에 점 하나
        pts.append(sample(T, n, 30 + i))
        owner.append(np.full(n, i))
    P, owner = np.concatenate(pts), np.concatenate(owner)
    Pd = P[np.random.default_rng(5).choice(len(P), min(len(P), 40000), replace=False)]   # 맞출 때는 일부, 확인은 전부
    print(f'표면 점 {len(P)} 개 (확인), 맞춤 {len(Pd)} 개')
    out = {}
    for K in Ks:
        C, R, _ = cover(Pd, K)
        Dall = np.min(np.linalg.norm(P[:, None, :] - C[None], axis=-1) - R[None], axis=1)
        # 확인에서 밖으로 나간 점이 있으면 그 구들 반지름을 늘림 (모든 점 덮기)
        if (Dall > 0).any():
            k = np.argmin(np.linalg.norm(P[Dall > 0][:, None, :] - C[None], axis=-1) - R[None], axis=1)
            for kk in np.unique(k):
                R[kk] = max(R[kk], float(np.linalg.norm(P[Dall > 0][k == kk] - C[kk], axis=1).max()) + COVER_PAD)
        Dall = np.min(np.linalg.norm(P[:, None, :] - C[None], axis=-1) - R[None], axis=1)
        lab = np.argmin(np.linalg.norm(P[:, None, :] - C[None], axis=-1) - R[None], axis=1)
        th = thickness(C, R, P)
        sph = []
        for k in range(len(C)):
            own = np.bincount(owner[lab == k], minlength=3)
            sph.append({'link': PARTS[int(np.argmax(own))], 'center': [round(float(v), 5) for v in C[k]], 'radius': round(float(R[k]), 5)})
        out[K] = {'spheres': sph, 'outside_max_mm': round(float(max(Dall.max(), 0)) * 1e3, 3),
                  'thick_med_mm': round(float(np.median(th)) * 1e3, 1), 'thick_p95_mm': round(float(np.quantile(th, .95)) * 1e3, 1),
                  'thick_max_mm': round(float(th.max()) * 1e3, 1), 'r_max_mm': round(float(R.max()) * 1e3, 1),
                  'tip_z_mm': round(float(np.max(C[:, 2] + R)) * 1e3, 1),
                  'x_mm': [round(float(np.min(C[:, 0] - R)) * 1e3, 1), round(float(np.max(C[:, 0] + R)) * 1e3, 1)],
                  'y_mm': [round(float(np.min(C[:, 1] - R)) * 1e3, 1), round(float(np.max(C[:, 1] + R)) * 1e3, 1)]}
        o = out[K]
        print(f'구 {K:2d} 개: 반지름 최대 {o["r_max_mm"]:.0f} mm, 덮개 밖 표면 {o["outside_max_mm"]:.3f} mm, '
              f'덮개 두께 (구 겉면 ↔ 실제 표면) 중앙 {o["thick_med_mm"]:.0f} / 95% {o["thick_p95_mm"]:.0f} / 최대 {o["thick_max_mm"]:.0f} mm, '
              f'덮개 끝 z {o["tip_z_mm"]:.0f} mm (실제 192.5), x {o["x_mm"]} y {o["y_mm"]}')
    (HERE / 'tool_cover_spheres.json').write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding='utf-8')


if __name__ == '__main__':
    main()
