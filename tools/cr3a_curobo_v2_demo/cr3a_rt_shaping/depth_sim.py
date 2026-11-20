#!/usr/bin/env python3
# [만듦 2026-10-13] CR3A 합성 깊이 영상 (카메라 1대 — 사용자 결정: 실제 로봇도 RealSense 1대) — warp 광선 추적. Mapper 시험에서 실제 카메라 대신 씀
#   로봇: Dobot 공식 URDF(cr3a_sim/dobot_rviz)의 STL 메시를 관절값으로 FK 해서 그림. 충돌 공이 아니라 실제 모양이라서,
#         [2026-10-16] 로봇 모델은 rt_model (기본 'tool' = 끝단 브래킷·D455·그리퍼 메시까지 그림, 'bare' = Dobot URDF 만)
#         팔 지우기(공 표면 + 5 cm)가 실제 팔을 다 덮는지가 결과에 드러난다 (Mirobot 시뮬레이션은 공으로 그려서 이 차이가 0 이었음)
#   장면: 바닥 (z = 0 평면) + 장애물 상자들 (매 장 자세·크기를 바꿀 수 있음, 수식으로 광선 검사)
#   카메라: movensys-manipulator feature/dynamic-obstacle-avoidance 의 두 카메라 중 0번 (camera_nvblox.launch.py 기본값,
#           world_manipulator → camera_nvblox_N_link, RealSense camera_link 규약: x 앞, z 위) 를 로봇 받침 좌표로 바꿈.
#           world_manipulator → Link0 = (-0.03, 0.275, 0.05), yaw -1.57 (movensys_manipulator.xacro robot_joint).
#           Dobot base_link 와 movensys Link0 은 같은 받침 좌표로 봄 (가정).
#   깊이: 광학 z 축 거리 (RealSense 깊이와 같은 뜻), 640x480 (R2 realsense_nvblox_N.yaml), fx = fy = 385 (D435 계열 가정)
#   잡음: 정규분포 σ = 2 mm (Mirobot 시뮬레이션 mapper_probe.render_depth 와 같음). depth_max 밖이나 맞은 곳 없음 = 0 (깊이 없음)
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import torch
import trimesh
import warp as wp

import rt_model

HERE = Path(__file__).resolve().parent
SIM = HERE.parent / 'cr3a_sim'
ASSET = SIM                                   # package://dobot_rviz/... → SIM/dobot_rviz/...
JOINTS = [f'joint{i}' for i in range(1, 7)]
WIDTH, HEIGHT, FX, FY = 640, 480, 385.0, 385.0
CX, CY = (WIDTH - 1) / 2, (HEIGHT - 1) / 2

# movensys-manipulator feature/dynamic-obstacle-avoidance (e9f16e0, 2026-09-16) camera_nvblox.launch.py 기본값의 0 번 카메라.
#  [2026-10-27 R-1] 1 번 카메라 (0.417, −0.255, 0.879, 0.074, 0.867, 2.402) 는 안 써서 뺌 (카메라 1 대 — 사용자 결정 2026-10-13)
R2_CAMERAS_WORLD = [  # x, y, z, roll, pitch, yaw (world_manipulator → camera_nvblox_N_link)
    (-0.402, -0.149, 0.947, -0.076, 1.017, 0.696),
]
WORLD_TO_BASE = (-0.03, 0.275, 0.05, 0.0, 0.0, -1.57)   # movensys_manipulator.xacro: table(=world) → Link0


def rpy_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def T_of(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_R(*rpy)
    T[:3, 3] = xyz
    return T


def R_to_wxyz(R):
    q = trimesh.transformations.quaternion_from_matrix(np.block([[R, np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]]))
    return [float(x) for x in q]              # trimesh: wxyz


# camera_link (x 앞, y 왼쪽, z 위) → 광학 좌표 (z 앞, x 오른쪽, y 아래)
LINK_TO_OPTICAL = rpy_R(-math.pi / 2, 0.0, -math.pi / 2)


def r2_camera_poses_base():
    """R2 카메라 (R2_CAMERAS_WORLD) 의 광학 좌표 자세 (로봇 받침 기준) → [(4x4)]"""
    Twb = T_of(WORLD_TO_BASE[:3], WORLD_TO_BASE[3:])
    out = []
    for c in R2_CAMERAS_WORLD:
        Twc = T_of(c[:3], c[3:])
        Tbc = np.linalg.inv(Twb) @ Twc
        Tbc[:3, :3] = Tbc[:3, :3] @ LINK_TO_OPTICAL
        out.append(Tbc)
    return out


def pose7(T):
    """4x4 → [x y z qw qx qy qz] (CameraMap.set_camera 형식)"""
    return [float(v) for v in T[:3, 3]] + R_to_wxyz(T[:3, :3])


def intrinsics():
    return [[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]]


class UrdfMeshes:
    """URDF 의 visual 메시 + FK (회전 관절, 고정 관절만). model = rt_model 이름·Model (None = 환경 변수 CR3A_MODEL 또는 기본) —
    urdf 를 직접 주면 그것 (asset 도 같이)"""

    def __init__(self, urdf=None, asset=None, device='cuda', model=None):
        if urdf is None:
            m = model if isinstance(model, rt_model.Model) else rt_model.get(model)
            urdf, asset = m.urdf, m.asset
        self.urdf, self.asset = Path(urdf), Path(asset if asset is not None else ASSET)
        asset = self.asset
        root = ET.parse(urdf).getroot()
        self.device = device
        self.links = {}
        for ln in root.findall('link'):
            vis = ln.find('visual')
            if vis is None or vis.find('geometry/mesh') is None:
                continue
            o = vis.find('origin')
            Tv = T_of([float(v) for v in o.get('xyz', '0 0 0').split()], [float(v) for v in o.get('rpy', '0 0 0').split()]) \
                if o is not None else np.eye(4)
            m = vis.find('geometry/mesh')
            fn = m.get('filename').replace('package://', '')
            sc = [float(v) for v in m.get('scale', '1 1 1').split()]
            mesh = trimesh.load(asset / fn, force='mesh')
            v = np.asarray(mesh.vertices) * np.array(sc)
            v = (Tv[:3, :3] @ v.T).T + Tv[:3, 3]
            self.links[ln.get('name')] = (torch.tensor(v, dtype=torch.float32, device=device),
                                          np.asarray(mesh.faces, dtype=np.int32))
        self.joints = {}
        for j in root.findall('joint'):
            o = j.find('origin')
            To = T_of([float(v) for v in o.get('xyz', '0 0 0').split()], [float(v) for v in o.get('rpy', '0 0 0').split()]) \
                if o is not None else np.eye(4)
            ax = j.find('axis')
            self.joints[j.find('child').get('link')] = dict(
                name=j.get('name'), parent=j.find('parent').get('link'), T=To, type=j.get('type'),
                axis=np.array([float(v) for v in ax.get('xyz').split()]) if ax is not None else np.zeros(3))
        children = set(self.joints)
        parents = {d['parent'] for d in self.joints.values()}
        roots = parents - children
        assert len(roots) == 1, f'URDF 뿌리 링크가 하나가 아님: {roots}'
        self.root = roots.pop()

    def link_T(self, q):
        """q: {관절 이름: rad} → {링크: 4x4 (받침 = 뿌리 링크 기준)}"""
        out = {self.root: np.eye(4)}

        def get(link):
            if link in out:
                return out[link]
            j = self.joints[link]
            T = get(j['parent']) @ j['T']
            if j['type'] in ('revolute', 'continuous'):
                a = j['axis'] / np.linalg.norm(j['axis'])
                R = trimesh.transformations.rotation_matrix(q.get(j['name'], 0.0), a)
                T = T @ R
            out[link] = T
            return T
        for ln in self.links:
            get(ln)
        return out


@wp.kernel
def _raycast_links(meshes: wp.array(dtype=wp.uint64), inv_R: wp.array(dtype=wp.mat33), inv_t: wp.array(dtype=wp.vec3),
                   box_c: wp.array(dtype=wp.vec3), box_Rinv: wp.array(dtype=wp.mat33), box_h: wp.array(dtype=wp.vec3),
                   n_boxes: int, cam_pos: wp.array(dtype=wp.vec3), cam_rot: wp.array(dtype=wp.mat33),
                   fx: float, fy: float, cx: float, cy: float, dmax: float, out: wp.array3d(dtype=float)):
    """[2026-10-13 고침] 링크마다 고정 메시(자기 좌표) + 광선을 링크 좌표로 옮겨 검사. 상자·바닥(z = 0)은 수식.
    (첫 판은 링크 메시를 한 메시에 합쳐 매 장 꼭짓점만 옮기고 refit → 원점에 겹친 채 만든 BVH 가 엉켜 2대 113 ms/장)"""
    c, v, u = wp.tid()
    d = wp.vec3((float(u) - cx) / fx, (float(v) - cy) / fy, 1.0)
    n = wp.length(d)
    o = cam_pos[c]
    dw = cam_rot[c] * (d / n)
    best = dmax * n
    for i in range(meshes.shape[0]):
        ol = inv_R[i] * o + inv_t[i]
        dl = inv_R[i] * dw
        q = wp.mesh_query_ray(meshes[i], ol, dl, best)
        if q.result:
            best = q.t
    for b in range(n_boxes):                      # 상자: 상자 좌표에서 slab
        ob = box_Rinv[b] * (o - box_c[b])
        db = box_Rinv[b] * dw
        h = box_h[b]
        t0 = float(0.0)
        t1 = best
        hit = int(1)
        for k in range(3):
            if wp.abs(db[k]) < 1.0e-9:
                if ob[k] < -h[k] or ob[k] > h[k]:
                    hit = 0
            else:
                ta = (-h[k] - ob[k]) / db[k]
                tb = (h[k] - ob[k]) / db[k]
                t0 = wp.max(t0, wp.min(ta, tb))
                t1 = wp.min(t1, wp.max(ta, tb))
        if hit == 1 and t0 <= t1 and t0 > 0.0:
            best = t0
    if dw[2] < -1.0e-9:                           # 바닥 z = 0
        tf = -o[2] / dw[2]
        if tf < best:
            best = tf
    if best < dmax * n:
        out[c, v, u] = best / n
    else:
        out[c, v, u] = 0.0


class DepthSim:
    """depth = sim.render(q, boxes) → torch [카메라 수, 높이, 너비] (m, GPU). boxes: [(중심 xyz, 크기 xyz, wxyz)]
    [2026-10-13 사용자 결정] 카메라는 1대 (실제 로봇도 RealSense 1대): 기본 = R2 브랜치 카메라 0번 자세 (cam_T 로 바꿀 수 있음)"""

    def __init__(self, cam_T=None, depth_max=2.0, noise_m=0.002, seed=0, device='cuda', model=None):
        wp.init()
        self.device = device
        self.robot = UrdfMeshes(device=device, model=model)
        self.cam_T = cam_T if cam_T is not None else r2_camera_poses_base()
        self.depth_max, self.noise_m = depth_max, noise_m
        self.gen = torch.Generator(device=device).manual_seed(seed)
        self.link_names = list(self.robot.links)
        self._meshes = []                         # 링크 좌표의 고정 메시 (BVH 한 번만)
        for ln in self.link_names:
            v, f = self.robot.links[ln]
            self._meshes.append(wp.Mesh(points=wp.from_torch(v.contiguous(), dtype=wp.vec3),
                                        indices=wp.array(f.reshape(-1), dtype=wp.int32, device=device)))
        self.mesh_ids = wp.array([m.id for m in self._meshes], dtype=wp.uint64, device=device)
        self.out = wp.zeros((len(self.cam_T), HEIGHT, WIDTH), dtype=float, device=device)
        self.set_cameras(self.cam_T)

    def set_cameras(self, cam_T):
        self.cam_T = cam_T
        self.cam_pos = wp.array([wp.vec3(*T[:3, 3]) for T in cam_T], dtype=wp.vec3, device=self.device)
        self.cam_rot = wp.array([wp.mat33(*T[:3, :3].reshape(-1)) for T in cam_T], dtype=wp.mat33, device=self.device)
        if self.out.shape[0] != len(cam_T):
            self.out = wp.zeros((len(cam_T), HEIGHT, WIDTH), dtype=float, device=self.device)

    def render(self, q, boxes=(), noise=True):
        Ts = self.robot.link_T(dict(zip(JOINTS, q)))
        invR = [Ts[ln][:3, :3].T for ln in self.link_names]
        invt = [-(R @ Ts[ln][:3, 3]) for R, ln in zip(invR, self.link_names)]
        inv_R = wp.array([wp.mat33(*R.reshape(-1)) for R in invR], dtype=wp.mat33, device=self.device)
        inv_t = wp.array([wp.vec3(*t) for t in invt], dtype=wp.vec3, device=self.device)
        bc = [wp.vec3(*b[0]) for b in boxes] or [wp.vec3(0.0, 0.0, -10.0)]
        bR = [wp.mat33(*trimesh.transformations.quaternion_matrix(b[2])[:3, :3].T.reshape(-1)) for b in boxes] \
            or [wp.mat33(*np.eye(3).reshape(-1))]
        bh = [wp.vec3(*(0.5 * np.asarray(b[1], float))) for b in boxes] or [wp.vec3(0.0, 0.0, 0.0)]
        wp.launch(_raycast_links, dim=(len(self.cam_T), HEIGHT, WIDTH),
                  inputs=[self.mesh_ids, inv_R, inv_t,
                          wp.array(bc, dtype=wp.vec3, device=self.device),
                          wp.array(bR, dtype=wp.mat33, device=self.device),
                          wp.array(bh, dtype=wp.vec3, device=self.device), len(boxes) if boxes else 0,
                          self.cam_pos, self.cam_rot, FX, FY, CX, CY, self.depth_max, self.out],
                  device=self.device)
        d = wp.to_torch(self.out).clone()
        if noise and self.noise_m > 0:
            hit = d > 0
            d = torch.where(hit, d + self.noise_m * torch.randn(d.shape, generator=self.gen, device=self.device), d)
        return d


if __name__ == '__main__':
    # 점검: 카메라 깊이 영상을 PNG 로 (HOME 자세 + 상자 하나)
    import sys
    import time
    sim = DepthSim()
    home = [0.0, 0.6072, -1.7223, -0.2949, 1.6134, 0.0]
    box = ([0.42, 0.0, 0.15], [0.10, 0.10, 0.30], [1, 0, 0, 0])
    for k, T in enumerate(sim.cam_T):
        print(f'카메라 {k}: 위치(받침 기준) {np.round(T[:3, 3], 3)}, 광축 {np.round(T[:3, 2], 3)}')
    d = sim.render(home, [box])
    torch.cuda.synchronize()
    t0 = time.perf_counter()
    for _ in range(20):
        d = sim.render(home, [box])
    torch.cuda.synchronize()
    print(f'렌더 {len(sim.cam_T)}대 x {WIDTH}x{HEIGHT}: {(time.perf_counter() - t0) / 20 * 1000:.2f} ms/장, '
          f'깊이 있는 픽셀 {[round(float((x > 0).float().mean()), 3) for x in d]}')
    if len(sys.argv) > 1:
        out = Path(sys.argv[1])
        for k in range(d.shape[0]):
            img = (np.clip(d[k].cpu().numpy() / 2.0, 0, 1) * 255).astype(np.uint8)
            try:
                from PIL import Image
                Image.fromarray(img).save(out / f'depth_cam{k}.png')
            except ImportError:
                np.save(out / f'depth_cam{k}.npy', d[k].cpu().numpy())
        print('저장:', out)
