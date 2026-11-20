"""실제 형상 거리 (한곳) — 로봇 메시 꼭짓점 (실행 기록 모델의 URDF 메시, depth_sim.UrdfMeshes) ↔ 축 정렬 상자·바닥. 닿음 = 이 거리 < 0.

[2026-10-27 R-1 6 단계] rt_report.MeshDist (자세 하나) 와 rt_tools.BatchMesh (자세 여러 개) 를 한 클래스로 — 계산은 각각 예전 그대로:
  자세 하나 = UrdfMeshes.link_T (trimesh 회전) → 링크마다 float32 행렬곱, 여러 개 = 같은 회전을 로드리게스로 한꺼번에 → 묶음 행렬곱.
  (둘은 끝자리 float 가 다를 수 있어 쓰던 쪽은 쓰던 길 그대로 — 닿음 판정 경계 0.01 mm 를 바꾸지 않게)
바닥 거리: 받침 (base_link) · 1 번 링크 (받침 위에서 z 축으로만 돎) 꼭짓점은 뺀 가장 낮은 z − 바닥 높이 floor_z (기록의 충돌 바닥 =
  rt_records.floor_z — 엔진이 쓴 바닥 max(table_z, 0). [2026-10-27 R-1 ⑮] 예전엔 늘 z = 0 이라 테이블을 올린 실제 카메라 기록에서 틀림).
device: 기본 = CUDA 가 있으면 'cuda' (Thor), 없으면 'cpu'.
"""
import numpy as np
import torch

import depth_sim as ds
import rt_model

JOINTS = [f'joint{i}' for i in range(1, 7)]
FLOOR_SKIP = ('base_link', 'Link1')     # 바닥 거리에서 빼는 링크 (바닥에 붙어 있거나 바닥과 높이가 안 바뀜)


class RobotMesh:
    """모델 (rt_model 이름·Model, None = 환경 변수 CR3A_MODEL 또는 기본) 의 메시 꼭짓점·거리."""

    def __init__(self, model=None, device=None):
        m = model if isinstance(model, rt_model.Model) else rt_model.get(model)
        self.m, self.model = m, m.name
        self.device = device or ('cuda' if torch.cuda.is_available() else 'cpu')
        self.torch = torch
        self.r = ds.UrdfMeshes(device=self.device, model=m)
        self.names = list(self.r.links)
        self.V = [self.r.links[ln][0] for ln in self.names]
        self.floor_mask = torch.cat([torch.full((len(v),), ln not in FLOOR_SKIP, dtype=torch.bool, device=self.device)
                                     for ln, v in zip(self.names, self.V)])
        self.jidx = {f'joint{i + 1}': i for i in range(6)}

    # ---- 자세 하나 (예전 rt_report.MeshDist) ----
    def points(self, q):
        """관절 q [6] → 꼭짓점 [N, 3]."""
        Ts = self.r.link_T(dict(zip(JOINTS, q)))
        out = []
        for ln, v in zip(self.names, self.V):
            T = torch.tensor(Ts[ln], dtype=torch.float32, device=self.device)
            out.append(v @ T[:3, :3].T + T[:3, 3])
        return torch.cat(out)

    def floor_dist(self, p, floor_z=0.0):
        """꼭짓점 [N, 3] 의 바닥 (z = floor_z) 거리 (m)."""
        return float(p[self.floor_mask, 2].min()) - floor_z

    def box_dist(self, p, center, dims):
        """꼭짓점 [N, 3] ↔ 축 정렬 상자 (중심, 크기) 최소 거리 (m, 안쪽 음수)."""
        c = torch.tensor(center, dtype=torch.float32, device=self.device)
        h = torch.tensor(dims, dtype=torch.float32, device=self.device) / 2
        d = (p - c).abs() - h
        outside = torch.clamp(d, min=0).norm(dim=-1)
        inside = torch.clamp(d.max(dim=-1).values, max=0)
        return float((outside + inside).min())

    def tip(self, q):
        """손끝 위치 [3] (m) = Link6 원점에서 공구 축 (+z) 으로 모델의 tip_m (bare 0 = 플랜지 면, tool = 그리퍼 끝)."""
        return (self.r.link_T(dict(zip(JOINTS, q)))['Link6'] @ np.array([0.0, 0.0, self.m.tip_m, 1.0]))[:3]

    # ---- 자세 여러 개 (예전 rt_tools.BatchMesh) ----
    def _link_T(self, Q):
        """Q [B, 6] → {링크: [B, 4, 4]} — UrdfMeshes.link_T 를 여러 자세로 (같은 회전: trimesh rotation_matrix = 로드리게스)."""
        B = len(Q)
        out = {self.r.root: np.repeat(np.eye(4)[None], B, 0)}

        def rot(axis, ang):
            a = axis / np.linalg.norm(axis)
            K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            T = np.repeat(np.eye(4)[None], B, 0)
            T[:, :3, :3] = np.eye(3)[None] + np.sin(ang)[:, None, None] * K[None] + (1 - np.cos(ang))[:, None, None] * (K @ K)[None]
            return T

        def get(link):
            if link in out:
                return out[link]
            j = self.r.joints[link]
            T = get(j['parent']) @ j['T']
            if j['type'] in ('revolute', 'continuous'):
                ang = Q[:, self.jidx[j['name']]] if j['name'] in self.jidx else np.zeros(B)
                T = T @ rot(j['axis'], ang)
            out[link] = T
            return T
        for ln in self.names:
            get(ln)
        return out

    def points_batch(self, Q):
        """[B, 6] → 꼭짓점 [B, N, 3]."""
        Ts = self._link_T(np.asarray(Q, float))
        out = []
        for ln, v in zip(self.names, self.V):
            T = torch.tensor(Ts[ln], dtype=torch.float32, device=self.device)
            out.append(v[None] @ T[:, :3, :3].transpose(1, 2) + T[:, None, :3, 3])
        return torch.cat(out, dim=1)

    def box_d(self, P, C, dims):
        """꼭짓점 [B, N, 3] ↔ 축 정렬 상자 (중심 C [B, 3] 또는 [3], 크기 [3]) 최소 거리 [B] (m, 안쪽 음수)."""
        c = torch.as_tensor(np.asarray(C, np.float32), device=self.device).view(-1, 1, 3)
        h = torch.tensor(dims, dtype=torch.float32, device=self.device) / 2
        d = (P - c).abs() - h
        return (torch.clamp(d, min=0).norm(dim=-1) + torch.clamp(d.max(dim=-1).values, max=0)).min(dim=1).values.cpu().numpy()

    def floor_d(self, P, floor_z=0.0):
        """꼭짓점 [B, N, 3] 의 바닥 (z = floor_z) 거리 [B] (m)."""
        d = P[:, self.floor_mask, 2].min(dim=1).values.cpu().numpy()
        return d - floor_z if floor_z else d


_CACHE = {}


def robot_mesh(m) -> RobotMesh:
    """모델별 RobotMesh (한 번만 만듦 — 메시 읽기가 몇 초)."""
    name = m.name if isinstance(m, rt_model.Model) else rt_model.get(m).name
    if name not in _CACHE:
        _CACHE[name] = RobotMesh(m)
    return _CACHE[name]
