#!/usr/bin/env python3
"""[만듦 2026-10-16] 사용자 CAD (SolidWorks 0_IIFES Demo.SLDASM: Dobot CR3 + 브래킷 robot_braket.step + RealSense D455 +
DH-Robotics AG-160-95 그리퍼) → Dobot URDF 의 Link6 좌표 공구 메시 3 개 + cr3_robot_tool.urdf.

실행 (PC, numpy·scipy 만):  py -3.11 make_tool_meshes.py
입력  cad/cad_raw.npz — cad/sw_extract.py 가 실행 중인 SolidWorks 에서 읽은 것 (부품마다 표시용 삼각형
      (부품 좌표) + 어셈블리 변환 행렬. 변환 규약 p_asm = p_part·R + t 은 부품 상자 (GetBox) 와 맞춰 확인)
      (옛 추출 = 고체만 + 따로 읽은 J6 곡면 몸체 j6b_tri.npy·sw_j6.py 는 archive/robot_tool/cad/ 에 보관)
      dobot_rviz/ — Thor cr3a_sim/dobot_rviz 사본 (고치지 않음)
하는 일
  1) CAD 로봇 자세 찾기: 받침~Link4 는 링크마다 6 자유도 ICP (축 회전 24 개 시작점) — 관절 사슬 검사 (앞 링크 → 다음 링크가 URDF
     관절 원점 + z 축 회전인지) 로 CAD 기구학 = URDF 확인. Link5·Link6 은 앞 링크 자세에서 관절각만 (1 자유도) 맞춤.
     확인: 그 Link6 좌표에서 CAD J6 플랜지 면이 z = 0 (URDF J6 메시 면) 에 오는지 (cad_J6_zmax_mm).
     [2026-10-16 확인] 링크마다 처음부터 6 자유도로 맞추면 원통 모양인 J6 플랜지가 엉뚱한 각도에서 멈춤 (중앙 1.35 mm, 138.6° 돌아감;
     사슬 맞춤은 0.67 mm). 받침부터 관절 1~6 을 모두 사슬로만 맞추면 받침 기울기 오차 (0.1° 수준) 가 손목까지 쌓여 J6 이 156° 로 틀림.
  2) 브래킷·카메라·그리퍼 삼각형을 그 Link6 좌표로 옮김 (플랜지 면 = z 0, 공구 쪽 = +z).
  3) 메시 정리: 꼭짓점 묶기 (CLUSTER_M 격자, 삼각형 수 줄이기) → 긴 모서리 나누기 (모서리 ≤ EDGE_M). 나누는 까닭: 닿음 판정
     (tools/rt_geom.RobotMesh) 이 메시 꼭짓점 ↔ 상자 거리라서, 브래킷 판 같은 넓은 면에 꼭짓점이 없으면 닿음을 놓침.
     (Dobot 메시는 모서리 95 % 14~24 mm, 최대 38~98 mm — 그대로 둠)
  4) 원래 표면 ↔ 정리한 메시 거리 확인, STL·URDF·tool_fit.json 저장. 보고서 표시용은 따로 가볍게 (tool/meshes/view/).
"""
import json
import math
import time
import xml.etree.ElementTree as ET
from itertools import permutations, product
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
URDF_IN = HERE / 'dobot_rviz' / 'urdf' / 'cr3_robot.urdf'
MESH_IN = HERE / 'dobot_rviz' / 'meshes' / 'cr3'
URDF_OUT = HERE / 'cr3_robot_tool.urdf'
MESH_OUT = HERE / 'tool' / 'meshes'
CLUSTER_M = 0.001       # 꼭짓점 묶기 격자 (m)
EDGE_M = 0.006          # 나눈 뒤 모서리 최대 (m)
VIEW_CLUSTER_M = 0.0015     # 보고서 3D 표시용 가벼운 메시 (tool/meshes/view/, 나누지 않음) — rt_report 가 있으면 이것을 넣음
GROUPS = [('CR3_Robot-1/CR3-BASE-1', 'base_link'), ('CR3_Robot-1/CR3-J1_ASM-1/', 'Link1'), ('CR3_Robot-1/CR3-J2_ASM-1/', 'Link2'),
          ('CR3_Robot-1/CR3-J3_ASM-1/', 'Link3'), ('CR3_Robot-1/CR3-J4_ASM-1/', 'Link4'), ('CR3_Robot-1/CR3-J5_ASM-1/', 'Link5'),
          ('CR3_Robot-1/CR3-J6_ASM-1/', 'Link6'), ('D455_Solid-1', 'camera'), ('AG-160-95-W-S_DH-Robotics-.step-1/', 'gripper'),
          ('robot_braket.step-1', 'bracket')]
TOOL = [('tool_bracket', 'bracket', '0.85 0.55 0.20 1'), ('tool_camera', 'camera', '0.23 0.44 0.77 1'),     # 보고서 3D 색: 주황·파랑·진회색
        ('tool_gripper', 'gripper', '0.30 0.32 0.36 1')]
LINKS = ['base_link', 'Link1', 'Link2', 'Link3', 'Link4', 'Link5', 'Link6']


# ---------------------------------------------------------------- 입출력
def read_stl(path):
    b = Path(path).read_bytes()
    n = int(np.frombuffer(b, np.uint32, 1, 80)[0])
    dt = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    assert 84 + n * 50 == len(b), f'binary STL 아님: {path}'
    return np.frombuffer(b, dt, n, 84)['v'].astype(np.float64)


def write_stl(path, V, F, name):
    t = V[F].astype(np.float32)
    nrm = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = np.where(ln > 0, nrm / np.maximum(ln, 1e-30), 0).astype(np.float32)
    dt = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    a = np.zeros(len(t), dt)
    a['n'] = nrm
    a['v'] = t
    with open(path, 'wb') as f:
        f.write(name.encode().ljust(80, b' ')[:80])
        f.write(np.uint32(len(t)).tobytes())
        f.write(a.tobytes())


def A_of(xf):
    """SolidWorks MathTransform ArrayData (행 벡터 규약 p_asm = p_part R + t) → 열 벡터 4x4"""
    A = np.eye(4)
    A[:3, :3] = np.array(xf[:9]).reshape(3, 3).T
    A[:3, 3] = xf[9:12]
    return A


def load_cad():
    d = np.load(HERE / 'cad' / 'cad_raw.npz')
    meta = json.loads(str(d['meta']))
    groups, parts = {}, []
    for m in sorted(meta, key=lambda x: x['name']):      # 이름 순 (SolidWorks 를 다시 열면 부품 순서가 바뀌어 표본 점이 달라짐)
        if m['n_children']:
            continue
        T = d[f"tri{m['i']}"].reshape(-1, 3, 3).astype(np.float64)
        if len(T) == 0 and m['name'].endswith('CR3-J6_CR3_2__CR30001_1_2_2_1_1-1'):   # (옛 추출 = 고체만) J6 곡면 몸체는 따로 읽은 것
            T = np.load(HERE.parent / 'archive' / 'robot_tool' / 'cad' / 'j6b_tri.npy').reshape(-1, 3, 3).astype(np.float64)
        A = A_of(m['xf'])
        g = next(g for pre, g in GROUPS if m['name'].startswith(pre))
        W = T @ A[:3, :3].T + A[:3, 3]
        groups.setdefault(g, []).append(W)
        parts.append((g, m['name'], W))
    return {g: np.concatenate(v) for g, v in groups.items()}, parts


# ---------------------------------------------------------------- URDF 기구학
def rpy_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rot_axis(axis, q):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K
    return T


def urdf_joints():
    root = ET.parse(URDF_IN).getroot()
    out = {}
    for j in root.findall('joint'):
        o = j.find('origin')
        T = np.eye(4)
        if o is not None:
            T[:3, :3] = rpy_R(*[float(v) for v in o.get('rpy', '0 0 0').split()])
            T[:3, 3] = [float(v) for v in o.get('xyz', '0 0 0').split()]
        ax = j.find('axis')
        out[j.find('child').get('link')] = {'name': j.get('name'), 'T': T,
                                            'axis': [float(v) for v in ax.get('xyz').split()] if ax is not None else None}
    meshes = {ln.get('name'): read_stl(MESH_IN / Path(ln.find('visual/geometry/mesh').get('filename')).name)
              for ln in root.findall('link') if ln.find('visual/geometry/mesh') is not None}
    return out, meshes


# ---------------------------------------------------------------- 맞추기
def sample(tris, n, seed):
    rng = np.random.default_rng(seed)
    a = 0.5 * np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1)
    idx = rng.choice(len(tris), n, p=a / a.sum())
    u, v = rng.random(n), rng.random(n)
    f = u + v > 1
    u[f], v[f] = 1 - u[f], 1 - v[f]
    t = tris[idx]
    return t[:, 0] + u[:, None] * (t[:, 1] - t[:, 0]) + v[:, None] * (t[:, 2] - t[:, 0])


def kabsch(P, Q):
    cp, cq = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((P - cp).T @ (Q - cq))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    T = np.eye(4)
    T[:3, :3] = Vt.T @ D @ U.T
    T[:3, 3] = cq - T[:3, :3] @ cp
    return T


def icp(src, tree, T0, iters=80, trim=0.9):
    T = T0.copy()
    for _ in range(iters):
        P = src @ T[:3, :3].T + T[:3, 3]
        dd, ii = tree.query(P)
        k = dd <= np.quantile(dd, trim)
        dT = kabsch(P[k], tree.data[ii[k]])
        T = dT @ T
        if np.abs(dT[:3, 3]).max() < 1e-7 and np.abs(dT[:3, :3] - np.eye(3)).max() < 1e-8:
            break
    return T, tree.query(src @ T[:3, :3].T + T[:3, 3])[0]


def fit_base(src, tree):
    cs, ct = src.mean(0), tree.data.mean(0)
    best = None
    for perm in permutations(range(3)):
        for sg in product((1, -1), repeat=3):
            R = np.zeros((3, 3))
            for i, p in enumerate(perm):
                R[i, p] = sg[i]
            if np.linalg.det(R) < 0:
                continue
            T0 = np.eye(4)
            T0[:3, :3] = R
            T0[:3, 3] = ct - R @ cs
            T, dd = icp(src, tree, T0)
            if best is None or np.median(dd) < np.median(best[1]):
                best = (T, dd)
    return best


def fit_joint(Tpar, j, src, tree):
    """부모 링크 자세 Tpar 에서 관절 j 의 각만 바꿔 맞춤: 1° 격자 → 0.2·0.05·0.01·0.002° 다듬기. 점수 = URDF 점 → CAD 점 거리 중앙"""
    def score(q):
        M = Tpar @ j['T'] @ rot_axis(j['axis'], q)
        return np.median(tree.query(src @ M[:3, :3].T + M[:3, 3])[0])
    grid = np.radians(np.arange(-180, 180, 1.0))
    s = np.array([score(q) for q in grid])
    q = grid[int(np.argmin(s))]
    for step in (0.2, 0.05, 0.01, 0.002):
        cand = q + np.radians(np.arange(-5, 6) * step)
        q = cand[int(np.argmin([score(c) for c in cand]))]
    M = Tpar @ j['T'] @ rot_axis(j['axis'], q)
    order = np.argsort(s)
    second = next(s[i] for i in order if abs(((grid[i] - q + math.pi) % (2 * math.pi)) - math.pi) > math.radians(10))
    return q, M, tree.query(src @ M[:3, :3].T + M[:3, 3])[0], float(second)


# ---------------------------------------------------------------- 메시 정리
def cluster(T, g):
    """삼각형 묶음 [N,3,3] → 격자 g 칸마다 꼭짓점 하나 (칸 안 평균), 찌그러진·겹친 삼각형 뺌"""
    V = T.reshape(-1, 3)
    _, inv = np.unique(np.floor(V / g).astype(np.int64), axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    n = inv.max() + 1
    Vc = np.zeros((n, 3))
    np.add.at(Vc, inv, V)
    Vc /= np.bincount(inv, minlength=n)[:, None]
    F = inv.reshape(-1, 3)
    F = F[(F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])]
    _, keep = np.unique(np.sort(F, axis=1), axis=0, return_index=True)
    return compact(Vc, F[np.sort(keep)])


def compact(V, F):
    used, F2 = np.unique(F, return_inverse=True)
    return V[used], F2.reshape(-1, 3)


def refine(V, F, emax):
    """긴 모서리 (> emax) 를 가운데서 나눔. 표시는 모서리마다 (길이로 정함) 라 이웃 삼각형이 같은 점으로 나뉨 (T 자 틈 없음).
    삼각형마다 표시된 모서리 수 (1·2·3) 대로 2·3·4 개로 나누고, 꼭짓점 순서 (면 방향) 유지. 모두 emax 이하가 될 때까지 반복."""
    for _ in range(30):
        E = np.stack([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]], axis=1)
        mark = np.linalg.norm(V[E[..., 0]] - V[E[..., 1]], axis=-1) > emax
        if not mark.any():
            break
        uk, inv = np.unique(np.sort(E[mark], axis=1), axis=0, return_inverse=True)
        base = len(V)
        V = np.vstack([V, 0.5 * (V[uk[:, 0]] + V[uk[:, 1]])])
        M = -np.ones(F.shape, np.int64)
        M[mark] = base + inv.reshape(-1)
        cnt = mark.sum(1)
        out = [F[cnt == 0]]
        for k in range(3):
            a, b, c = F[:, k], F[:, (k + 1) % 3], F[:, (k + 2) % 3]
            s1 = (cnt == 1) & mark[:, k]
            m = M[:, k]
            out += [np.stack([a, m, c], 1)[s1], np.stack([m, b, c], 1)[s1]]
            s2 = (cnt == 2) & ~mark[:, k]                       # 모서리 k (a-b) 만 안 나눔
            m1, m2 = M[:, (k + 1) % 3], M[:, (k + 2) % 3]       # b-c, c-a 의 가운데
            out += [np.stack([m1, c, m2], 1)[s2], np.stack([a, b, m1], 1)[s2], np.stack([a, m1, m2], 1)[s2]]
        s3 = cnt == 3
        a, b, c = F[s3, 0], F[s3, 1], F[s3, 2]
        mab, mbc, mca = M[s3, 0], M[s3, 1], M[s3, 2]
        out += [np.stack([a, mab, mca], 1), np.stack([mab, b, mbc], 1), np.stack([mca, mbc, c], 1), np.stack([mab, mbc, mca], 1)]
        F = np.concatenate(out)
    return V, F


def surf_err(T_ref, V, F):
    """원래 표면 표본 → 정리한 메시 (촘촘한 표본) 거리 (m): 중앙, 99 %, 최대"""
    P = sample(T_ref, 60000, 11)
    Q = sample(V[F], 600000, 12)
    d = cKDTree(Q).query(P)[0]
    return float(np.median(d)), float(np.quantile(d, 0.99)), float(d.max())


# ---------------------------------------------------------------- URDF 쓰기
def write_urdf():
    s = URDF_IN.read_text(encoding='utf-8')
    add = ['', '  <!-- [2026-10-16] 끝단 공구 (사용자 CAD 0_IIFES Demo.SLDASM, robot_tool/make_tool_meshes.py). 메시는 Link6 좌표',
           '       (플랜지 면 z = 0, 공구 쪽 +z). 그리퍼 AG-160-95 는 CAD 그대로 완전히 벌림 (손가락 간격 95 mm) - 사용자 결정 -->']
    add += ['       cuRobo 충돌 구는 메시로 맞추지 않고 공구 전체를 덮는 큰 구 (cover_spheres.py, 사용자 결정 2026-10-16) -->']
    add[-2] = add[-2].replace(' -->', '')
    for link, g, rgba in TOOL:
        fn = f'package://tool/meshes/{link}.STL'
        add += [f'  <link name="{link}">',
                f'    <visual><origin xyz="0 0 0" rpy="0 0 0" /><geometry><mesh filename="{fn}" /></geometry>'
                f'<material name=""><color rgba="{rgba}" /></material></visual>',
                f'    <collision><origin xyz="0 0 0" rpy="0 0 0" /><geometry><mesh filename="{fn}" /></geometry></collision>',
                '  </link>',
                f'  <joint name="{link}_joint" type="fixed"><origin xyz="0 0 0" rpy="0 0 0" /><parent link="Link6" />'
                f'<child link="{link}" /></joint>']
    i = s.rindex('</robot>')
    URDF_OUT.write_text(s[:i] + '\n'.join(add) + '\n' + s[i:], encoding='utf-8')


def main():
    t0 = time.time()
    cad, parts = load_cad()
    J, um = urdf_joints()
    rec = {'made': time.strftime('%Y-%m-%d %H:%M'), 'cad': '0_IIFES Demo.SLDASM (C:\\Users\\JohnJunhoHeo\\Desktop\\Robot)', 'links': {}}
    trees = {ln: cKDTree(sample(cad[ln], 40000, 1)) for ln in LINKS}
    srcs = {ln: sample(um[ln], 6000, 2) for ln in LINKS}
    T, q = {}, []
    for ln in LINKS[:5]:                                   # 받침 ~ Link4: 6 자유도
        T[ln], dd = fit_base(srcs[ln], trees[ln])
        rec['links'][ln] = {'fit': '6dof', 'med_mm': round(float(np.median(dd)) * 1e3, 3), 'p95_mm': round(float(np.quantile(dd, .95)) * 1e3, 3)}
        print(f"{ln}: 6 자유도 URDF→CAD 중앙 {np.median(dd) * 1e3:.3f} / 95% {np.quantile(dd, .95) * 1e3:.2f} mm")
    for k, ln in enumerate(LINKS[1:5], 1):                 # 관절 사슬 검사
        j = J[ln]
        Rel = np.linalg.inv(j['T']) @ np.linalg.inv(T[LINKS[k - 1]]) @ T[ln]
        qk = math.atan2(Rel[1, 0], Rel[0, 0])
        q.append(qk)
        rec['links'][ln].update(q_deg=round(math.degrees(qk), 3), chain_dxyz_mm=np.round(Rel[:3, 3] * 1e3, 3).tolist(),
                                chain_tilt_deg=round(math.degrees(math.acos(np.clip(Rel[2, 2], -1, 1))), 3))
        print(f"  {j['name']}: q = {math.degrees(qk):8.3f}°, 사슬 검사 남는 이동 {np.round(Rel[:3, 3] * 1e3, 2)} mm, "
              f"z 축 기울기 {rec['links'][ln]['chain_tilt_deg']:.3f}°")
    for k, ln in ((5, 'Link5'), (6, 'Link6')):             # 관절각만
        qk, T[ln], dd, second = fit_joint(T[LINKS[k - 1]], J[ln], srcs[ln], trees[ln])
        q.append(qk)
        rec['links'][ln] = {'fit': '1dof', 'q_deg': round(math.degrees(qk), 3), 'med_mm': round(float(np.median(dd)) * 1e3, 3),
                            'p95_mm': round(float(np.quantile(dd, .95)) * 1e3, 3), 'next_best_med_mm': round(second * 1e3, 3)}
        print(f"{ln}: 관절각만 q = {math.degrees(qk):8.3f}°  URDF→CAD 중앙 {np.median(dd) * 1e3:.3f} / 95% {np.quantile(dd, .95) * 1e3:.2f} mm"
              f" (10° 밖 다음 후보 중앙 {second * 1e3:.3f} mm)")
    # (비교만) Link6 을 그 자리에서 6 자유도로 다듬어 보기 — [2026-10-16] 중앙 0.67 → 1.39 mm 로 나빠지고 플랜지 면이 1.9 mm 어긋나서
    #  (ICP 는 잘라 낸 제곱 평균을 줄여, URDF 에 없는 CAD J6 곡면 부품 쪽으로 끌림) 쓰지 않음. 쓰는 것 = 관절각만 맞춘 자세
    Tr, dd = icp(srcs['Link6'], trees['Link6'], T['Link6'])
    D = np.linalg.inv(T['Link6']) @ Tr
    rec['links']['Link6'].update(refine_unused_med_mm=round(float(np.median(dd)) * 1e3, 3),
                                 refine_unused_move_mm=np.round(D[:3, 3] * 1e3, 3).tolist(),
                                 refine_unused_rot_deg=round(math.degrees(math.acos(np.clip((np.trace(D[:3, :3]) - 1) / 2, -1, 1))), 3))
    print(f"(비교만) Link6 6 자유도 다듬기: 중앙 {np.median(dd) * 1e3:.3f} mm, 움직인 양 {np.round(D[:3, 3] * 1e3, 2)} mm, "
          f"{rec['links']['Link6']['refine_unused_rot_deg']:.3f}° → 안 씀")
    inv6 = np.linalg.inv(T['Link6'])
    P6 = cad['Link6'].reshape(-1, 3) @ inv6[:3, :3].T + inv6[:3, 3]
    rec['cad_J6_zmax_mm'] = round(float(P6[:, 2].max()) * 1e3, 3)          # 플랜지 면 (URDF z = 0) 과 맞는지
    rec['q_cad_deg'] = [round(math.degrees(v), 3) for v in q]
    rec['T_link6_in_cad_asm'] = np.round(T['Link6'], 7).tolist()
    print('CAD J6 플랜지 면 z 최대 (mm, 0 이어야):', rec['cad_J6_zmax_mm'])
    MESH_OUT.mkdir(parents=True, exist_ok=True)
    rec['tool'] = {}
    for link, g, _ in TOOL:
        Tl = cad[g] @ inv6[:3, :3].T + inv6[:3, 3]
        V, F = cluster(Tl, CLUSTER_M)
        n_c = len(F)
        V, F = refine(V, F, EDGE_M)
        e = np.linalg.norm(V[F[:, [1, 2, 0]]] - V[F], axis=-1)
        med, p99, mx = surf_err(Tl, V, F)
        write_stl(MESH_OUT / f'{link}.STL', V, F, link)
        Vv, Fv = cluster(Tl, VIEW_CLUSTER_M)
        (MESH_OUT / 'view').mkdir(exist_ok=True)
        write_stl(MESH_OUT / 'view' / f'{link}.STL', Vv, Fv, link + ' view')
        vmed, vp99, vmx = surf_err(Tl, Vv, Fv)
        lo, hi = V.min(0), V.max(0)
        rec['tool'][link] = {'tri_cad': len(Tl), 'tri_cluster': n_c, 'tri': len(F), 'verts': len(V),
                             'edge_max_mm': round(float(e.max()) * 1e3, 2), 'err_med_mm': round(med * 1e3, 3),
                             'err_p99_mm': round(p99 * 1e3, 3), 'err_max_mm': round(mx * 1e3, 3),
                             'view_tri': len(Fv), 'view_err_p99_mm': round(vp99 * 1e3, 3), 'view_err_max_mm': round(vmx * 1e3, 3),
                             'min_mm': np.round(lo * 1e3, 1).tolist(), 'max_mm': np.round(hi * 1e3, 1).tolist()}
        print(f'{link}: 삼각형 CAD {len(Tl)} → 묶음 {n_c} → 나눔 {len(F)} (꼭짓점 {len(V)}, 모서리 최대 {e.max() * 1e3:.1f} mm), '
              f'원래 표면과 거리 중앙 {med * 1e3:.3f} / 99% {p99 * 1e3:.3f} / 최대 {mx * 1e3:.3f} mm, 범위 mm {np.round(lo * 1e3, 1)} ~ {np.round(hi * 1e3, 1)}'
              f' | 표시용 {len(Fv)} 삼각형 (99% {vp99 * 1e3:.2f} / 최대 {vmx * 1e3:.2f} mm)')
    tips = []
    for g, nm, W in parts:
        if 'FINGERTIP' in nm:
            P = W.reshape(-1, 3) @ inv6[:3, :3].T + inv6[:3, 3]
            tips.append({'part': nm.split('/')[-1], 'center_mm': np.round(P.mean(0) * 1e3, 2).tolist(),
                         'zmax_mm': round(float(P[:, 2].max()) * 1e3, 2), 'x_inner_mm': round(float(np.abs(P[:, 0]).min()) * 1e3, 2)})
    rec['fingertips'] = tips
    rec['tip_z_m'] = round(max(t['zmax_mm'] for t in tips) / 1e3, 4)
    print('손가락 끝:', tips, ' 그리퍼 끝 z =', rec['tip_z_m'], 'm')
    write_urdf()
    (HERE / 'tool_fit.json').write_text(json.dumps(rec, ensure_ascii=False, indent=1), encoding='utf-8')
    print(f'저장: {URDF_OUT.name}, tool/meshes/*.STL, tool_fit.json ({time.time() - t0:.0f} s)')


if __name__ == '__main__':
    main()
