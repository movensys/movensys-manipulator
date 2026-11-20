#!/usr/bin/env python3
"""CR3A + 끝단 공구 형상 — URDF 읽기, 정기구학, STL 읽기 (numpy 만). GUI 3D 뷰가 씀.

형상 = 작업 사본 cr3a_rt_shaping/robot_tool (rt_model 'tool': Dobot CR3A URDF + 사용자 CAD 공구 — 브래킷·D455·AG-160-95 그리퍼).
좌표 = 로봇 받침 (base_link): x 앞, y 왼쪽, z 위, 원점 = 받침 바닥 가운데. 길이 m, 각도는 함수마다 표시.
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
RT = HERE.parent / 'cr3a_rt_shaping'           # 작업 사본 (입력성형·실시간 회피 코드)
TOOLDIR = RT / 'robot_tool'
URDF = TOOLDIR / 'cr3_robot_tool.urdf'
TIP_M = 0.1925                                  # 공구 끝 = Link6 원점에서 +z (그리퍼 손가락 끝, rt_model 'tool')
JOINTS = tuple(f'joint{i}' for i in range(1, 7))
DECIMATE_M = 0.0025                             # 3D 뷰용 메시 줄이기: 이 크기 칸 안의 꼭짓점을 하나로 (그림 속도)
POSES = {                                       # 자세 목록에 늘 있는 자세 (°, 지울 수 없음). 사용자가 저장한 자세는 cr3a_gui POSES_FILE.
    '영점 (모두 0°)': (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),    # [2026-10-27 사용자 "하드코딩 된 자세 두개는 지우고"] 시뮬레이션 자세 A·B 뺌
}


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


def rot_axis(axis, a):
    """축 (단위 벡터) 둘레로 a rad 도는 4x4."""
    x, y, z = axis
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    T = np.eye(4)
    T[:3, :3] = [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                 [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                 [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]
    return T


def read_stl(path):
    """STL (이진) → 삼각형 (n, 3, 3) float64."""
    data = Path(path).read_bytes()
    n = int.from_bytes(data[80:84], 'little')
    if 84 + 50 * n != len(data):
        raise ValueError(f'{path}: 이진 STL 이 아님')
    rec = np.frombuffer(data, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]), count=n, offset=84)
    return rec['v'].astype(np.float64)


def decimate(tri, cell):
    """꼭짓점 묶기로 메시 줄이기: cell 칸 안의 꼭짓점을 평균 하나로, 찌그러진·겹친 삼각형은 뺌. → (V, F)."""
    V = tri.reshape(-1, 3)
    _, inv = np.unique(np.floor(V / cell).astype(np.int64), axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    cnt = np.bincount(inv)
    P = np.zeros((len(cnt), 3))
    np.add.at(P, inv, V)
    P /= cnt[:, None]
    F = inv.reshape(-1, 3)
    F = F[(F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])]
    _, keep = np.unique(np.sort(F, axis=1), axis=0, return_index=True)
    return P, F[np.sort(keep)]


class Robot:
    """URDF 의 관절 사슬 + 링크마다 메시 (3D 뷰용으로 줄인 것)."""

    def __init__(self, urdf=URDF, cell=DECIMATE_M):
        root = ET.parse(urdf).getroot()
        self.joints = []                     # (이름, 종류, 부모, 자식, 원점 4x4, 축)
        for j in root.findall('joint'):
            o = j.find('origin')
            T0 = T_of([float(v) for v in o.get('xyz', '0 0 0').split()],
                      [float(v) for v in o.get('rpy', '0 0 0').split()]) if o is not None else np.eye(4)
            a = j.find('axis')
            axis = np.array([float(v) for v in a.get('xyz').split()]) if a is not None else None
            self.joints.append((j.get('name'), j.get('type'), j.find('parent').get('link'), j.find('child').get('link'),
                                T0, axis))
        self.limits_deg = []
        for name in JOINTS:
            lim = root.find(f"joint[@name='{name}']/limit")
            self.limits_deg.append((math.degrees(float(lim.get('lower'))), math.degrees(float(lim.get('upper')))))
        self.parts = []                      # (링크, 꼭짓점 (m, 링크 좌표), 면, RGB 0~255)
        for ln in root.findall('link'):
            v = ln.find('visual')
            if v is None:
                continue
            o = v.find('origin')
            Tv = T_of([float(x) for x in o.get('xyz', '0 0 0').split()],
                      [float(x) for x in o.get('rpy', '0 0 0').split()]) if o is not None else np.eye(4)
            tri = read_stl(self._mesh_path(v.find('geometry/mesh').get('filename')))
            tri = tri @ Tv[:3, :3].T + Tv[:3, 3]
            P, F = decimate(tri, cell)
            c = v.find('material/color')
            rgb = [float(x) * 255 for x in c.get('rgba').split()[:3]] if c is not None else [200, 200, 200]
            self.parts.append((ln.get('name'), P, F, np.array(rgb)))

    @staticmethod
    def _mesh_path(fn):
        rel = fn.replace('package://', '')
        p = TOOLDIR / rel
        view = p.parent / 'view' / p.name            # 공구는 보고서용 가벼운 메시가 있으면 그것
        return view if view.exists() else p

    def fk(self, q_deg):
        """관절각 (°, 6 개) → {링크: 4x4 (받침 기준)}."""
        q = dict(zip(JOINTS, np.radians(np.asarray(q_deg, float))))
        T = {'dummy_link': np.eye(4), 'base_link': np.eye(4)}
        for name, kind, parent, child, T0, axis in self.joints:
            Tc = T[parent] @ T0
            if kind in ('revolute', 'continuous'):
                Tc = Tc @ rot_axis(axis, q[name])
            T[child] = Tc
        return T

    def tip(self, q_deg):
        """공구 끝 (그리퍼 손가락 끝) 4x4. [2026-10-27 R-1] 아무도 안 넘기던 fk 인자는 뺌."""
        return self.fk(q_deg)['Link6'] @ T_of([0, 0, TIP_M], [0, 0, 0])


def rpy_from_matrix(R):
    """회전 → (Rx, Ry, Rz) ° — 고정 축 x→y→z 순서 (R = Rz·Ry·Rx)."""
    sy = -R[2, 0]
    ry = math.asin(max(-1.0, min(1.0, sy)))
    if abs(sy) < 0.99999:
        rx, rz = math.atan2(R[2, 1], R[2, 2]), math.atan2(R[1, 0], R[0, 0])
    else:
        rx, rz = math.atan2(-R[1, 2], R[1, 1]), 0.0
    return tuple(math.degrees(a) for a in (rx, ry, rz))
