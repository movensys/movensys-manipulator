#!/usr/bin/env python3
"""GUI 의 3D 뷰 — CR3A + 끝단 공구, 바닥, 장애물 칸, 고정 카메라, 목표 자세, 계획 경로.
[2026-10-27 R-1] GPU 로 그림 (pyqtgraph.opengl + PyOpenGL). 전에는 numpy + OpenCV 로 삼각형을 직접 칠함 (한 장 ~0.1 s, 장애물 칸이
  많으면 몇 배 — 그래서 GUI 가 3D 를 4 Hz 로 줄여 그렸음). 이제 메시는 처음 한 번만 GPU 에 올리고, 자세가 바뀌면 링크 변환 (4x4) 만,
  지도가 바뀌면 칸 가운데·색만 올림. refresh() 는 바뀐 것만 넣고 다시 그리기를 부탁한 뒤 바로 돌아옴 (그리기는 Qt 가 다음 화면 갱신 때).
  pyqtgraph·PyOpenGL 이 없거나 OpenGL 문맥을 못 만들면 3D 뷰 자리에 까닭을 글로 보여 줌 (GUI 는 계속 돎. 소프트웨어로 대신 그리지 않음).

그리는 것
  - 로봇: URDF 메시 (링크 색 = URDF, 공구 = 브래킷 주황 · D455 파랑 · 그리퍼 진회색). 밝기로 입체감 + 어두운 테두리
  - 바닥 (테이블 윗면 floor_z) + 10 cm 격자, 받침 축 (x 빨강, y 초록, z 파랑)
  - 장애물: 카메라 지도의 물체 칸 (높이 색: 파랑 = 낮음 → 빨강 = 높음)
  - 목표 자세: 반투명 초록 로봇 / 계획 경로: 공구 끝이 지나가는 주황 선 / 목표 공구 끝: 노란 십자 + 방향 축
  - 성형 전 명령 (켜면): 반투명 보라 로봇 — 입력성형 전에 보낸 명령 자세 (실제 로봇은 성형기 지연만큼 늦게 따라감)
  - 고정 카메라: 시야 사각뿔 (D455 깊이 시야 87° x 58°)
  - 지도 점검 (로봇 겉면 5.5 cm 안 지도 칸): 자홍 점
  가림: 로봇 · 장애물 칸 · 반투명 로봇은 깊이로 서로 가림 (단 실제 로봇은 전처럼 반투명 로봇보다 늘 위, 실제 로봇의 바깥
  윤곽선은 전처럼 장애물 칸 위에도). 바닥 · 격자 · 받침 축은 늘 밑에. 지도 점검 점 → 경로 · 목표 · 카메라는 전처럼 맨 위에.
보기: 왼쪽 끌기 = 회전, 휠 = 확대/축소, 더블클릭 = 처음 시점. [카메라 시점] = 고정 카메라가 보는 대로.
"""
import ctypes
import math
import sys

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

import cr3a_model as cm

try:                    # [2026-10-27 R-1] GPU 그리기 — 없으면 3D 뷰 자리에 까닭을 보여 줌 (GUI 는 돎)
    from OpenGL import GL
    import pyqtgraph.opengl as gl
    from pyqtgraph import Transform3D
    from pyqtgraph.opengl import shaders as pg_shaders
    from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem
    GL_ERR = None
except Exception as e:  # 무엇이 빠졌든 (모듈 없음, libGL 없음 …) 글로 보여 줌
    GL = gl = None
    GL_ERR = (f'pyqtgraph · PyOpenGL 을 불러오지 못함 ({type(e).__name__}: {e})\n'
              'pip install --user --break-system-packages pyqtgraph PyOpenGL')

PRESETS = {'대각': (-50.0, 24.0, 2.05), '앞': (0.0, 6.0, 2.05), '옆': (-90.0, 6.0, 2.05), '위': (-90.0, 88.0, 2.4)}
ORBIT_TARGET = np.array([0.20, 0.0, 0.30])
ORBIT_FOV_V = 38.0                      # 둘러보기 세로 시야 (°)
BG_TOP, BG_BOTTOM = (46, 52, 60), (24, 28, 33)
FLOOR_RGB = (52, 60, 68)
FLOOR_XY = (-0.5, 0.9, -0.7, 0.7)       # 바닥 x0, x1, y0, y1 (m, 10 cm 격자)
GRID_RGB, GRID0_RGB = (82, 92, 102), (120, 130, 140)          # 격자 / 원점을 지나는 격자
AXIS_RGB = ((255, 72, 72), (72, 208, 72), (72, 136, 255))     # x 빨강, y 초록, z 파랑
EDGE_RGB = (18, 20, 24)                 # 로봇 테두리
GHOST_RGB = (63, 185, 80)
U_RGB = (190, 120, 255)
PATH_RGB = (255, 150, 50)
TARGET_RGB = (255, 224, 40)
CAM_RGB = (57, 197, 207)
NEAR_RGB = (255, 60, 255)
CAM_FOV = (87.0, 58.0)
LIGHT = np.array([0.35, -0.45, 0.82]) / np.linalg.norm([0.35, -0.45, 0.82])
# [2026-10-27 R-1] GPU 그리기 설정
CLIP_M = (0.05, 30.0)                   # 그리는 깊이 범위 (렌즈 앞 m) — 전처럼 5 cm 보다 가까운 것은 안 그림
MSAA = 4                                # 가장자리 매끄럽게 (전: 1.5 배로 그려 줄임)
EDGE_PX = 2.5                           # 로봇 테두리 = 뒷면 선 굵기 (실루엣 밖으로 반쯤 나옴)
XRAY_PX = 4.0                           # 실제 로봇 바깥 윤곽선 (칸 위에도 그림) 굵기 — 실루엣 밖 ~2 px, 전 윤곽선과 비슷
LINE_PX, THIN_PX = 2.5, 1.3             # 경로·축·목표 선 / 카메라 시야 선 (px)
CV_TO_GL = np.diag([1.0, -1.0, -1.0, 1.0])  # 렌즈 좌표 (OpenCV: x 오른쪽 y 아래 z 앞) → OpenGL 눈 좌표 (x 오른쪽 y 위 z 뒤)

# 로봇: 링크 메시를 한 VBO 에 (꼭짓점마다 링크 번호) → 링크 변환은 uniform 배열로 한 번에. 밝기 식은 전 그림과 같음
# (밝기 = 0.22 + 0.50·정면 + 0.36·빛, 가장 어두운 면 = 색 × u_dark). 전에는 10 단계로 끊었고 이제는 매끄럽게.
_ROBOT_VS = '''
uniform mat4 u_proj;
uniform mat4 u_mv[NLINK];
uniform vec4 u_col[NLINK];
attribute vec3 a_pos;
attribute vec3 a_nrm;
attribute float a_link;
varying vec3 v_pos;
varying vec3 v_nrm;
varying vec4 v_col;
void main() {
    int k = int(a_link + 0.5);
    vec4 p = u_mv[k] * vec4(a_pos, 1.0);
    v_pos = p.xyz;
    v_nrm = (u_mv[k] * vec4(a_nrm, 0.0)).xyz;
    v_col = u_col[k];
    gl_Position = u_proj * p;
}
'''
_ROBOT_FS = '''
#ifdef GL_ES
precision mediump float;
#endif
uniform vec3 u_light;
uniform float u_dark;
uniform vec4 u_edge;
varying vec3 v_pos;
varying vec3 v_nrm;
varying vec4 v_col;
void main() {
    if (u_edge.a > 0.0) {
        gl_FragColor = u_edge;
    } else {
        vec3 n = normalize(v_nrm);
        float facing = abs(dot(n, normalize(v_pos)));
        float key = max(dot(n, u_light), 0.0);
        float shade = 0.22 + 0.50 * facing + 0.36 * key;
        float b = u_dark + (1.1 - u_dark) * clamp(shade / 1.08, 0.0, 1.0);
        gl_FragColor = vec4(min(v_col.rgb * b, vec3(1.0)), v_col.a);
    }
}
'''
# 장애물 칸: 정육면체 하나 (36 꼭짓점) 를 칸마다 되풀이 (instancing) — 지도가 바뀌면 칸 가운데·색 (칸마다 24 B) 만 올림
_VOX_VS = '''
uniform mat4 u_mvp;
uniform float u_size;
attribute vec4 a_corner;
attribute vec3 a_center;
attribute vec3 a_color;
varying vec3 v_color;
void main() {
    v_color = a_color * a_corner.w;
    gl_Position = u_mvp * vec4(a_center + a_corner.xyz * u_size, 1.0);
}
'''
_VOX_FS = '''
#ifdef GL_ES
precision mediump float;
#endif
varying vec3 v_color;
void main() {
    gl_FragColor = vec4(v_color, 1.0);
}
'''
_BG_VS = '''
attribute vec4 a_position;
attribute vec4 a_color;
varying vec4 v_color;
void main() {
    v_color = a_color;
    gl_Position = vec4(a_position.xy, 0.0, 1.0);
}
'''
_BG_FS = '''
#ifdef GL_ES
precision mediump float;
#endif
varying vec4 v_color;
void main() {
    gl_FragColor = v_color;
}
'''


def height_color(z):
    """높이 (m) → RGB: 파랑 (바닥) → 초록 → 노랑 → 빨강 (0.6 m)."""
    t = np.clip(np.asarray(z) / 0.6, 0, 1)[..., None]
    stops = np.array([[60, 110, 255], [40, 200, 200], [90, 210, 90], [250, 210, 60], [240, 80, 60]], float)
    x = t * (len(stops) - 1)
    i = np.minimum(x.astype(int), len(stops) - 2)
    f = x - i
    return stops[i[..., 0]] * (1 - f) + stops[i[..., 0] + 1] * f


def _rgba(rgb, a=1.0):
    return tuple(float(c) / 255.0 for c in rgb) + (float(a),)


def _cm(M):
    """행 우선 4x4 (여러 개면 n x 4 x 4) → glUniformMatrix4fv (transpose 없이) 용 열 우선 float32."""
    return np.ascontiguousarray(np.swapaxes(np.asarray(M, np.float32), -1, -2))


def _gl_opts(depth=True, write=True, blend=False):
    """pyqtgraph 물체의 GL 상태. 0.14 는 물체 사이에 상태를 되돌리지 않으므로 깊이 쓰기·비교도 늘 적어 둠."""
    o = {GL.GL_DEPTH_TEST: depth, GL.GL_BLEND: blend, GL.GL_CULL_FACE: False,
         'glDepthMask': (GL.GL_TRUE if write else GL.GL_FALSE,), 'glDepthFunc': (GL.GL_LEQUAL,)}
    if blend:
        o['glBlendFuncSeparate'] = (GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA, GL.GL_ONE, GL.GL_ONE_MINUS_SRC_ALPHA)
    return o


def _program(vs, fs, attribs, uniforms):
    """셰이더 프로그램 (지금 GL 문맥) → (번호, {uniform: 위치}). attribs 는 0 번부터 차례로 (0 번 = 늘 배열로 주는 것)."""
    from OpenGL.GL import shaders
    prog = shaders.compileProgram(shaders.compileShader(vs, GL.GL_VERTEX_SHADER),
                                  shaders.compileShader(fs, GL.GL_FRAGMENT_SHADER))
    for i, name in enumerate(attribs):
        GL.glBindAttribLocation(prog, i, name)
    GL.glLinkProgram(prog)
    loc = {u: GL.glGetUniformLocation(prog, u) for u in uniforms}
    missing = [u for u, k in loc.items() if k < 0]
    if missing:
        raise RuntimeError(f'셰이더에 uniform 없음: {missing}')
    return prog, loc


def _unit_cube():
    """한 변 1 정육면체 (가운데 원점) 삼각형 12 개 → (36, 4) float32: 꼭짓점 xyz + 면 밝기.
    면 밝기는 전 그림과 같음 (윗·아랫면 1.0, x 면 0.78, y 면 0.62). 바깥에서 볼 때 반시계 (뒷면 빼기)."""
    rows = []
    for a, shade in ((0, 0.78), (1, 0.62), (2, 1.0)):
        b, c = (a + 1) % 3, (a + 2) % 3                     # e_b x e_c = e_a
        for s in (0.5, -0.5):
            quad = []
            for d1, d2 in ((-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)):
                p = [0.0, 0.0, 0.0]
                p[a], p[b], p[c] = s, d1, d2
                quad.append(p + [shade])
            if s < 0:
                quad = quad[::-1]
            rows += [quad[i] for i in (0, 1, 2, 0, 2, 3)]
    return np.array(rows, np.float32)


def _show(item, on):
    if item.visible() != on:
        item.setVisible(on)


def _report(item, what, e):
    """GPU 물체 그리기 실패 → 3D 뷰 자리에 까닭 (pyqtgraph 는 물체 오류를 찍기만 하고 넘어가서, 반쪽 그림이 되지 않게)."""
    v = item.view()
    if v is not None:
        v.owner._fail(f'{what} 그리기 실패: {type(e).__name__}: {e}')


def _gl_problem():
    """OpenGL 을 쓸 수 있는지 미리 봄 → 문제 글 / None. QOpenGLWidget 를 창에 넣은 뒤 문맥이 안 되면 창 전체 그리기가
    깨질 수 있어서 (Qt 5 는 GL 위젯이 한 번 들어온 창을 계속 GL 로 합성), 넣기 전에 따로 만들어 봄."""
    if GL_ERR:
        return GL_ERR
    ctx = QtGui.QOpenGLContext()
    if not ctx.create():
        return 'OpenGL 문맥을 만들지 못함 (DISPLAY · 그래픽 드라이버 확인)'
    if ctx.isOpenGLES():
        return 'OpenGL ES 문맥만 됨 — 데스크톱 OpenGL 3.3 이상이 필요함'
    surf = QtGui.QOffscreenSurface()
    surf.setFormat(ctx.format())
    surf.create()
    if not ctx.makeCurrent(surf):
        return 'OpenGL 문맥을 쓸 수 없음 (makeCurrent 실패)'
    try:
        ver = GL.glGetString(GL.GL_VERSION).decode(errors='replace')
        major, minor = (int(x) for x in ver.split()[0].split('.')[:2])
        if (major, minor) < (3, 3) or not (bool(GL.glDrawArraysInstanced) and bool(GL.glVertexAttribDivisor)):
            return f'OpenGL 3.3 이상이 필요함 (지금 {ver})'
    except Exception as e:
        return f'OpenGL 확인 실패: {type(e).__name__}: {e}'
    finally:
        ctx.doneCurrent()
    return None


class _RobotMesh:
    """로봇 메시 한 벌 → VBO 하나 (꼭짓점마다 x y z, 면 법선, 링크 번호 — float32 7 개) + 셰이더.
    실제·목표·성형 전 로봇 셋이 같이 씀 (GPU 에 한 번만 올림). 법선 = 면마다 (전 그림처럼 평평한 면)."""

    def __init__(self, robot):
        rows = []
        for k, (_, P, F, _) in enumerate(robot.parts):
            W = P[F]
            n = np.cross(W[:, 1] - W[:, 0], W[:, 2] - W[:, 0])
            ln = np.linalg.norm(n, axis=1)
            ok = ln > 1e-12                                 # 넓이 0 인 삼각형 빼기 (법선이 없음)
            V = np.empty((int(ok.sum()), 3, 7), np.float32)
            V[..., :3] = W[ok]
            V[..., 3:6] = (n[ok] / ln[ok, None])[:, None, :]
            V[..., 6] = k
            rows.append(V.reshape(-1, 7))
        self.data = np.ascontiguousarray(np.concatenate(rows))
        self.n_link = len(robot.parts)
        self.prog = self.loc = self.vbo = None

    def ensure(self):
        if self.prog is None:
            self.prog, self.loc = _program(_ROBOT_VS.replace('NLINK', str(self.n_link)), _ROBOT_FS, ('a_pos', 'a_nrm', 'a_link'),
                                           ('u_proj', 'u_mv', 'u_col', 'u_light', 'u_dark', 'u_edge'))
            self.vbo = GL.glGenBuffers(1)
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.vbo)
            GL.glBufferData(GL.GL_ARRAY_BUFFER, self.data.nbytes, self.data, GL.GL_STATIC_DRAW)
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)


class _Robot(GLGraphicsItem if gl else object):
    """로봇 한 벌 — 그릴 때마다 링크 변환 (n x 4x4) 만 바꿈. alpha < 1 = 반투명 (목표·성형 전 명령)."""

    def __init__(self, mesh, colors, dark, edge, alpha=1.0):
        super().__init__()
        self.mesh, self.dark, self.alpha = mesh, float(dark), float(alpha)
        self.col = np.array([_rgba(c, alpha) for c in colors], np.float32)
        self.edge = np.array(_rgba(edge), np.float32)
        self.T = None                       # 링크마다 받침 ← 링크 4x4 (n, 4, 4)

    def set_pose(self, T):
        self.T = T
        self.update()

    def paint(self):
        if self.T is None:
            return
        try:
            self._paint()
        except Exception as e:
            _report(self, '로봇', e)

    def _paint(self):
        m, v = self.mesh, self.view()
        m.ensure()
        loc, n = m.loc, len(m.data)
        GL.glUseProgram(m.prog)
        try:
            GL.glUniformMatrix4fv(loc['u_proj'], 1, GL.GL_FALSE, _cm(v.P))
            GL.glUniformMatrix4fv(loc['u_mv'], m.n_link, GL.GL_FALSE, _cm(v.V @ self.T))
            GL.glUniform4fv(loc['u_col'], m.n_link, self.col)
            GL.glUniform3f(loc['u_light'], *(v.V[:3, :3] @ LIGHT))        # 빛 (받침 좌표) → 눈 좌표
            GL.glUniform1f(loc['u_dark'], self.dark)
            GL.glUniform4f(loc['u_edge'], 0.0, 0.0, 0.0, 0.0)
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, m.vbo)
            for i, (size, off) in enumerate(((3, 0), (3, 12), (1, 24))):
                GL.glEnableVertexAttribArray(i)
                GL.glVertexAttribPointer(i, size, GL.GL_FLOAT, GL.GL_FALSE, 28, ctypes.c_void_p(off))
            GL.glEnable(GL.GL_DEPTH_TEST)
            GL.glEnable(GL.GL_CULL_FACE)                                   # 메시는 모두 바깥 방향 면 → 뒷면 빼기 (전 그림과 같음)
            GL.glCullFace(GL.GL_BACK)
            # 스텐실 비트 1 = 실제 로봇이 보이는 화소 (+ 바깥 윤곽선): 반투명 로봇은 거기 안 그림 → 전 그림처럼 실제 로봇이 늘 위
            #  (같은 자세면 깊이가 똑같아 LEQUAL 로 반투명 색이 실제 로봇을 덮음 — 목표에 서 있으면 늘 그런 자세. 받침은 늘 같음).
            #  비트 2 = 실제 로봇 실루엣 전체 (가려진 곳 포함) — 바깥 윤곽선을 장애물 칸 위에도 그리는 데 씀 (아래 테두리 2)
            GL.glEnable(GL.GL_STENCIL_TEST)
            real = self.alpha >= 1.0
            if real:
                GL.glStencilMask(0x01)
                GL.glStencilFunc(GL.GL_ALWAYS, 0x01, 0x01)
                GL.glStencilOp(GL.GL_KEEP, GL.GL_KEEP, GL.GL_REPLACE)
                GL.glDisable(GL.GL_BLEND)
                GL.glDepthMask(GL.GL_TRUE)
                GL.glDepthFunc(GL.GL_LESS)
                GL.glDrawArrays(GL.GL_TRIANGLES, 0, n)
            else:
                GL.glStencilMask(0x00)
                GL.glStencilFunc(GL.GL_NOTEQUAL, 0x01, 0x01)
                GL.glStencilOp(GL.GL_KEEP, GL.GL_KEEP, GL.GL_KEEP)
                # 반투명: 1) 깊이만 → 2) 가장 앞 면만 한 번 섞음 (전 그림처럼 겹친 면이 더 진해지지 않음)
                GL.glColorMask(GL.GL_FALSE, GL.GL_FALSE, GL.GL_FALSE, GL.GL_FALSE)
                GL.glDepthMask(GL.GL_TRUE)
                GL.glDepthFunc(GL.GL_LESS)
                GL.glDrawArrays(GL.GL_TRIANGLES, 0, n)
                GL.glColorMask(GL.GL_TRUE, GL.GL_TRUE, GL.GL_TRUE, GL.GL_TRUE)
                GL.glDepthMask(GL.GL_FALSE)
                GL.glDepthFunc(GL.GL_LEQUAL)
                GL.glEnable(GL.GL_BLEND)
                GL.glBlendFuncSeparate(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA, GL.GL_ONE, GL.GL_ONE_MINUS_SRC_ALPHA)
                GL.glDrawArrays(GL.GL_TRIANGLES, 0, n)
                GL.glDisable(GL.GL_BLEND)
            # 테두리 1: 뒷면을 선으로 그림 (깊이 비교) — 보이는 실루엣 밖으로 반쯤 나오고, 안쪽 경계 (팔이 받침 앞 등) 도 보임
            GL.glDepthMask(GL.GL_FALSE)
            GL.glDepthFunc(GL.GL_LEQUAL)
            GL.glCullFace(GL.GL_FRONT)
            GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_LINE)
            GL.glLineWidth(EDGE_PX)
            GL.glUniform4f(loc['u_edge'], *self.edge)
            GL.glDrawArrays(GL.GL_TRIANGLES, 0, n)
            if real:
                # 테두리 2 (전 그림처럼 바깥 윤곽선은 장애물 칸 위에도 — 칸이 빽빽해도 로봇 자리가 보임):
                #  실루엣 전체를 비트 2 로 칠한 뒤, 깊이 비교 없이 그 밖으로 나온 뒷면 선만 그림 (그린 곳은 비트 1)
                GL.glDisable(GL.GL_DEPTH_TEST)
                GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_FILL)
                GL.glCullFace(GL.GL_BACK)
                GL.glColorMask(GL.GL_FALSE, GL.GL_FALSE, GL.GL_FALSE, GL.GL_FALSE)
                GL.glStencilMask(0x02)
                GL.glStencilFunc(GL.GL_ALWAYS, 0x02, 0x02)
                GL.glStencilOp(GL.GL_REPLACE, GL.GL_REPLACE, GL.GL_REPLACE)
                GL.glDrawArrays(GL.GL_TRIANGLES, 0, n)
                GL.glColorMask(GL.GL_TRUE, GL.GL_TRUE, GL.GL_TRUE, GL.GL_TRUE)
                GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_LINE)
                GL.glLineWidth(XRAY_PX)
                GL.glCullFace(GL.GL_FRONT)
                GL.glStencilMask(0x01)
                GL.glStencilFunc(GL.GL_NOTEQUAL, 0x03, 0x02)               # 비트 2 = 0 (실루엣 밖) 만. 쓰는 값 = 3 & 1 = 1
                GL.glStencilOp(GL.GL_KEEP, GL.GL_KEEP, GL.GL_REPLACE)
                GL.glDrawArrays(GL.GL_TRIANGLES, 0, n)
        finally:
            GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_FILL)
            GL.glLineWidth(1.0)
            GL.glCullFace(GL.GL_BACK)
            GL.glDisable(GL.GL_CULL_FACE)
            GL.glStencilMask(0xFF)
            GL.glDisable(GL.GL_STENCIL_TEST)
            GL.glEnable(GL.GL_DEPTH_TEST)
            GL.glDisable(GL.GL_BLEND)
            GL.glColorMask(GL.GL_TRUE, GL.GL_TRUE, GL.GL_TRUE, GL.GL_TRUE)
            GL.glDepthMask(GL.GL_TRUE)
            GL.glDepthFunc(GL.GL_LESS)
            for i in range(3):
                GL.glDisableVertexAttribArray(i)
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)
            GL.glUseProgram(0)


class _Voxels(GLGraphicsItem if gl else object):
    """장애물 칸 (정육면체, 높이 색). set() 은 칸 가운데·색 배열만 만들고, GPU 로 올리기는 다음에 그릴 때."""

    def __init__(self):
        super().__init__()
        self.prog = self.loc = self.vbo = None
        self.pending, self.n, self.size = None, 0, 0.01

    def set(self, C, size):
        C = np.asarray(C, np.float32).reshape(-1, 3)
        inst = np.empty((len(C), 6), np.float32)
        inst[:, :3] = C
        inst[:, 3:] = height_color(C[:, 2]) / 255.0
        self.pending, self.size = inst, float(size)
        self.update()

    def paint(self):
        try:
            self._paint()
        except Exception as e:
            _report(self, '장애물 칸', e)

    def _paint(self):
        if self.prog is None:
            self.prog, self.loc = _program(_VOX_VS, _VOX_FS, ('a_corner', 'a_center', 'a_color'), ('u_mvp', 'u_size'))
            self.vbo = GL.glGenBuffers(2)
            cube = _unit_cube()
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.vbo[0])
            GL.glBufferData(GL.GL_ARRAY_BUFFER, cube.nbytes, cube, GL.GL_STATIC_DRAW)
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)
        if self.pending is not None:
            self.n, data, self.pending = len(self.pending), self.pending, None
            if self.n:
                GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.vbo[1])
                GL.glBufferData(GL.GL_ARRAY_BUFFER, data.nbytes, data, GL.GL_DYNAMIC_DRAW)
                GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)
        if not self.n:
            return
        v = self.view()
        GL.glUseProgram(self.prog)
        try:
            GL.glUniformMatrix4fv(self.loc['u_mvp'], 1, GL.GL_FALSE, _cm(v.P @ v.V))
            GL.glUniform1f(self.loc['u_size'], self.size * 1.002)          # 이웃 칸 사이 머리카락 틈이 안 보이게 아주 조금 크게
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.vbo[0])
            GL.glEnableVertexAttribArray(0)
            GL.glVertexAttribPointer(0, 4, GL.GL_FLOAT, GL.GL_FALSE, 16, None)
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.vbo[1])
            for i, off in ((1, 0), (2, 12)):
                GL.glEnableVertexAttribArray(i)
                GL.glVertexAttribPointer(i, 3, GL.GL_FLOAT, GL.GL_FALSE, 24, ctypes.c_void_p(off))
                GL.glVertexAttribDivisor(i, 1)
            GL.glEnable(GL.GL_DEPTH_TEST)
            GL.glDepthMask(GL.GL_TRUE)
            GL.glDepthFunc(GL.GL_LESS)
            GL.glDisable(GL.GL_BLEND)
            GL.glEnable(GL.GL_CULL_FACE)
            GL.glCullFace(GL.GL_BACK)
            GL.glDrawArraysInstanced(GL.GL_TRIANGLES, 0, 36, self.n)
        finally:
            for i in (1, 2):
                GL.glVertexAttribDivisor(i, 0)                            # 기본 VAO 를 pyqtgraph 물체와 같이 쓰므로 되돌림
            for i in range(3):
                GL.glDisableVertexAttribArray(i)
            GL.glDisable(GL.GL_CULL_FACE)
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)
            GL.glUseProgram(0)


class _View(gl.GLViewWidget if gl else QtWidgets.QWidget):
    """pyqtgraph 3D 위젯 — 시점·투영은 Scene3D._camera() (전 그림과 같은 식), 마우스는 Scene3D 로 넘김."""

    def __init__(self, owner):
        super().__init__()
        self.owner = owner
        fmt = QtGui.QSurfaceFormat()
        fmt.setVersion(2, 1)                # pyqtgraph 가 보는 판. 실제로는 드라이버 최고 판 (Thor = 4.6 호환) — offscreen 플랫폼은 요청한 판을 그대로 알려 줌
        fmt.setDepthBufferSize(24)
        fmt.setStencilBufferSize(8)         # 실제 로봇 화소 표시 (반투명 로봇이 덮지 않게). QOpenGLWidget 의 FBO 는 깊이+스텐실
        fmt.setSamples(MSAA)
        self.setFormat(fmt)
        self.setFocusPolicy(QtCore.Qt.NoFocus)          # 키보드는 GUI 가 (전 3D 뷰처럼 초점을 받지 않음)
        self.setBackgroundColor(BG_BOTTOM)
        self.V, self.P = np.eye(4), np.eye(4)            # 이번 그림의 시점 (받침 → 눈) · 투영 (행 우선) — 직접 그리는 물체가 씀

    def viewMatrix(self):
        T, _ = self.owner._camera(self.width(), self.height())
        self.V = CV_TO_GL @ T
        return QtGui.QMatrix4x4(*self.V.ravel())

    def projectionMatrix(self, region, viewport):
        """카메라 행렬 K (렌즈 가운데 = 화면 가운데) 그대로의 원뿔. pyqtgraph 의 fov 는 가로 시야라 쓰지 않음."""
        x0, y0, w, h = viewport
        _, K = self.owner._camera(w, h)
        rx, ry, rw, rh = region
        n, f = CLIP_M
        m = QtGui.QMatrix4x4()
        m.frustum(n * (rx - x0 - K[0, 2]) / K[0, 0], n * (rx + rw - x0 - K[0, 2]) / K[0, 0],
                  n * (ry - y0 - (h - K[1, 2])) / K[1, 1], n * (ry + rh - y0 - (h - K[1, 2])) / K[1, 1], n, f)
        self.P = np.array(m.data(), float).reshape(4, 4).T
        return m

    def initializeGL(self):
        try:
            super().initializeGL()
        except Exception as e:              # PyQt 는 가상 함수 예외로 프로그램을 끝낼 수 있어 여기서 막음
            self.owner._fail(f'OpenGL 준비 실패: {type(e).__name__}: {e}')

    def paintGL(self):
        try:
            GL.glDepthMask(GL.GL_TRUE)      # glClear 가 깊이·색을 다 지우게 (앞 그림의 물체가 끈 채 두었을 수 있음)
            GL.glColorMask(GL.GL_TRUE, GL.GL_TRUE, GL.GL_TRUE, GL.GL_TRUE)
            GL.glStencilMask(0xFF)          # 스텐실은 pyqtgraph 가 안 지워서 여기서
            GL.glClearStencil(0)
            GL.glClear(GL.GL_STENCIL_BUFFER_BIT)
            super().paintGL()
        except Exception as e:
            self.owner._fail(f'그리기 실패: {type(e).__name__}: {e}')

    def mousePressEvent(self, ev):
        self.owner.mousePressEvent(ev)

    def mouseMoveEvent(self, ev):
        self.owner.mouseMoveEvent(ev)

    def mouseReleaseEvent(self, ev):
        self.owner.mouseReleaseEvent(ev)

    def mouseDoubleClickEvent(self, ev):
        self.owner.mouseDoubleClickEvent(ev)

    def wheelEvent(self, ev):
        self.owner.wheelEvent(ev)

    def keyPressEvent(self, ev):
        ev.ignore()

    def keyReleaseEvent(self, ev):
        ev.ignore()


class _Overlay(QtWidgets.QWidget):
    """3D 뷰 위 글 (안내 + 범례). 투명, 마우스는 아래 3D 뷰로. 글이 바뀔 때만 다시 그림 (GL 그림마다가 아님)."""

    def __init__(self, owner):
        super().__init__(owner)
        self.owner = owner
        self.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)
        self.setAttribute(QtCore.Qt.WA_NoSystemBackground)
        self.text = ('', [])

    def paintEvent(self, _):
        note, items = self.text
        p = QtGui.QPainter(self)
        p.setPen(QtGui.QColor(210, 215, 222))
        p.drawText(10, 18, note)
        y = self.height() - 10
        for color, text in reversed(items):
            p.fillRect(10, y - 10, 11, 11, QtGui.QColor(color))
            p.setPen(QtGui.QColor(210, 215, 222))
            p.drawText(27, y, text)
            y -= 17
        p.end()


class Scene3D(QtWidgets.QWidget):
    def __init__(self, robot=None):
        super().__init__()
        self.setMinimumSize(480, 360)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.robot = robot or cm.Robot()
        self.az, self.el, self.dist = PRESETS['대각']
        self.mode = 'orbit'                 # 'orbit' / 'camera'
        self.flags = {'obst': True, 'cam': True, 'plan': True, 'u': False}
        self.q = None                       # 지금 관절각 (°, 엔코더)
        self.ghost = None                   # 목표 자세 (°)
        self.u_q = None                     # 성형 전 명령 자세 (°)
        self.path = None                    # 계획 경로 (관절각 ° 목록)
        self.target_T = None                # 목표 공구 끝 4x4
        self.cam_T = None                   # 받침 ← 고정 카메라 광학 좌표 (z 앞, x 오른쪽, y 아래) 4x4
        self.obst = None                    # (칸 가운데 N x 3 m, 칸 크기 m)
        self.near = None                    # [2026-10-27 D-1 ㄴ] 지도 점검: 로봇 겉면 5.5 cm 안 지도 칸 가운데 N x 3 m (로봇에 안 가리게 위에 그림)
        self.floor_z = 0.0                  # [2026-10-27 D-2] 테이블 윗면 높이 (받침 원점 기준 m, 카메라 보정이 잰 값) — 바닥 격자를 이 높이에
        self.note = ''
        # [2026-10-27 R-1] _sig = 지난 refresh() 때 넣은 것들 (바뀐 것만 다시 넣음). None 으로 두면 다음 refresh() 가 모두 다시 넣음
        self._sig, self._drag = None, None
        self.view = self._overlay = self._label = None
        self._it, self._failed = {}, False
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        why = _gl_problem()
        if why:
            self._show_error(why)
            return
        try:
            self._build(lay)
        except Exception as e:
            self._show_error(f'3D 뷰 만들기 실패: {type(e).__name__}: {e}')

    # ---------- 보기 ----------
    def set_preset(self, name):
        self.mode = 'orbit'
        self.az, self.el, self.dist = PRESETS[name]
        self.refresh()

    def set_camera_mode(self):
        if self.cam_T is not None:
            self.mode = 'camera'
            self.refresh()

    def set_flag(self, key, on):
        self.flags[key] = on
        self.refresh()

    def mousePressEvent(self, ev):
        if ev.button() == QtCore.Qt.LeftButton:
            self._drag = (ev.pos(), self.az, self.el)

    def mouseMoveEvent(self, ev):
        if self._drag is not None:
            p0, az0, el0 = self._drag
            self.mode = 'orbit'
            self.az = az0 - (ev.pos().x() - p0.x()) * 0.4
            self.el = float(np.clip(el0 + (ev.pos().y() - p0.y()) * 0.4, -5.0, 89.0))
            self.refresh()

    def mouseReleaseEvent(self, ev):
        self._drag = None
        self.refresh()

    def mouseDoubleClickEvent(self, ev):
        self.set_preset('대각')

    def wheelEvent(self, ev):
        if self.mode == 'orbit':
            self.dist = float(np.clip(self.dist * 0.9 ** (ev.angleDelta().y() / 120.0), 0.6, 4.0))
            self.refresh()

    def resizeEvent(self, ev):
        if self._overlay is not None:
            self._overlay.setGeometry(self.rect())
        self.refresh()

    def showEvent(self, ev):
        super().showEvent(ev)
        if self.view is not None:
            QtCore.QTimer.singleShot(0, self._check_gl)

    # ---------- 그리기 ----------
    def refresh(self):
        """[2026-10-27 R-1] 바뀐 것만 GPU 물체에 넣고 다시 그리기를 부탁함 — 막지 않음 (그리기는 Qt 가 다음 화면 갱신 때).
        전에는 여기서 한 장을 다 그렸음. _sig = None 이면 모두 다시 넣음 (시험 코드가 씀)."""
        if self.view is None:
            return
        try:
            self._sync()
        except Exception as e:              # 물체 갱신이 깨져도 GUI 는 살리고 까닭을 보여 줌
            self._fail(f'3D 장면 갱신 실패: {type(e).__name__}: {e}')
            return
        self.view.update()
        text = self._legend()
        if text != self._overlay.text:
            self._overlay.text = text
            self._overlay.update()

    def _build(self, lay):
        v = self.view = _View(self)
        lay.addWidget(v)
        it = self._it

        def add(key, item, order, visible=True):
            item.setDepthValue(order)       # 그리는 차례 (작은 것 먼저): 배경 → 바닥 → 칸 → 로봇 → 반투명 로봇 → 맨 위 (가림 없음)
            item.setVisible(visible)
            v.addItem(item)
            it[key] = item

        quad = np.array([[0, 1, 2], [0, 2, 3]])
        x0, x1, y0, y1 = FLOOR_XY
        bg = pg_shaders.ShaderProgram(f'cr3a_bg_{id(self)}', [pg_shaders.VertexShader(_BG_VS), pg_shaders.FragmentShader(_BG_FS)])
        add('bg', gl.GLMeshItem(vertexes=np.array([[-1, -1, 0], [1, -1, 0], [1, 1, 0], [-1, 1, 0]], float), faces=quad,
                                vertexColors=np.array([_rgba(BG_BOTTOM)] * 2 + [_rgba(BG_TOP)] * 2), smooth=True,
                                computeNormals=False, shader=bg, glOptions=_gl_opts(depth=False, write=False)), -100)
        add('floor', gl.GLMeshItem(vertexes=np.array([[x0, y0, 0], [x1, y0, 0], [x1, y1, 0], [x0, y1, 0]], float), faces=quad,
                                   color=_rgba(FLOOR_RGB), smooth=False, computeNormals=False, glOptions=_gl_opts(write=False)), -90)
        P, C = [], []
        for x in np.arange(x0, x1 + 1e-6, 0.1):
            P += [(x, y0, 0.0), (x, y1, 0.0)]
            C += [_rgba(GRID0_RGB if abs(x) < 1e-6 else GRID_RGB)] * 2
        for y in np.arange(y0, y1 + 1e-6, 0.1):
            P += [(x0, y, 0.0), (x1, y, 0.0)]
            C += [_rgba(GRID0_RGB if abs(y) < 1e-6 else GRID_RGB)] * 2
        # 바닥·격자·받침 축은 깊이를 쓰지 않음 → 뒤에 그리는 로봇·칸이 늘 위 (전 그림처럼. 테이블 높이 > 0 이면 받침 밑이 바닥 아래)
        add('grid', gl.GLLinePlotItem(pos=np.array(P), color=np.array(C), mode='lines', width=1.0, glOptions=_gl_opts(write=False)), -80)
        P, C = np.zeros((6, 3)), []
        for i, col in enumerate(AXIS_RGB):
            P[2 * i + 1, i] = 0.15
            C += [_rgba(col)] * 2
        add('axes', gl.GLLinePlotItem(pos=P, color=np.array(C), mode='lines', width=LINE_PX, glOptions=_gl_opts(write=False)), -70)
        # 칸을 로봇보다 먼저: 로봇 테두리 (깊이 안 씀) 가 뒤의 칸에 덮이지 않고, 스텐실 1 = 실제로 보이는 로봇 화소만
        mesh, n = _RobotMesh(self.robot), len(self.robot.parts)
        add('vox', _Voxels(), 0, False)
        add('robot', _Robot(mesh, [c for *_, c in self.robot.parts], 0.18, EDGE_RGB), 1)
        add('ghost', _Robot(mesh, [GHOST_RGB] * n, 0.62, np.multiply(GHOST_RGB, 0.8), alpha=0.35), 10, False)
        add('u', _Robot(mesh, [U_RGB] * n, 0.62, np.multiply(U_RGB, 0.8), alpha=0.40), 11, False)
        top = _gl_opts(depth=False)
        add('path', gl.GLLinePlotItem(mode='line_strip', color=_rgba(PATH_RGB), width=LINE_PX, glOptions=top), 20, False)
        add('path_end', gl.GLScatterPlotItem(pos=np.zeros((2, 3)), color=np.array([_rgba((255, 255, 255)), _rgba(PATH_RGB)]),
                                             size=np.array([11.0, 13.0]), pxMode=True, glOptions=top), 21, False)
        add('target', gl.GLLinePlotItem(mode='lines', width=LINE_PX, glOptions=top), 22, False)
        add('cam', gl.GLLinePlotItem(mode='lines', color=_rgba(CAM_RGB), width=THIN_PX, glOptions=top), 23, False)
        add('cam_dot', gl.GLScatterPlotItem(pos=np.zeros((1, 3)), color=_rgba(CAM_RGB), size=16.0, pxMode=True, glOptions=top), 24, False)
        # 지도 점검 점: 로봇·칸 위, 경로·목표·카메라 아래 (전 그림 차례)
        add('near_ring', gl.GLScatterPlotItem(pos=np.zeros((1, 3)), color=_rgba((255, 255, 255)), size=12.5, pxMode=True,
                                              glOptions=top), 18, False)
        add('near', gl.GLScatterPlotItem(pos=np.zeros((1, 3)), color=_rgba(NEAR_RGB), size=10.5, pxMode=True, glOptions=top), 19, False)
        self._overlay = _Overlay(self)
        self._overlay.raise_()

    def _sync(self):
        """Scene3D 값 → GPU 물체 (바뀐 것만)."""
        it, f = self._it, self.flags
        last = self._sig if isinstance(self._sig, dict) else {}
        sig = {}

        def changed(key, val, by_id=False):
            """지난번과 다르면 True. by_id = 같은 물체인지로 (경로·지도처럼 GUI 가 새 배열을 넣는 것 — 물체를 쥐고 있어 id 가 다시 쓰이지 않음)."""
            sig[key] = val
            if key not in last:
                return True
            return last[key] is not val if by_id else last[key] != val

        q = np.zeros(6) if self.q is None else np.asarray(self.q, float)
        if changed('q', tuple(np.round(q, 3))):
            it['robot'].set_pose(self._links(q))
        on = self.ghost is not None
        if on and changed('ghost', tuple(np.round(np.asarray(self.ghost, float), 3))):
            it['ghost'].set_pose(self._links(self.ghost))
        _show(it['ghost'], on)
        on = bool(f['u']) and self.u_q is not None
        if on and changed('u', tuple(np.round(np.asarray(self.u_q, float), 3))):
            it['u'].set_pose(self._links(self.u_q))
        _show(it['u'], on)
        on = bool(f['obst']) and self.obst is not None and len(self.obst[0]) > 0
        if on and changed('obst', self.obst, by_id=True):
            it['vox'].set(*self.obst)
        _show(it['vox'], on)
        on = self.near is not None and len(self.near) > 0
        if on and changed('near', self.near, by_id=True):
            P = np.asarray(self.near, float).reshape(-1, 3)
            it['near_ring'].setData(pos=P)
            it['near'].setData(pos=P)
        _show(it['near_ring'], on)
        _show(it['near'], on)
        on = bool(f['plan']) and self.path is not None and len(self.path) > 1
        if on and changed('path', self.path, by_id=True):
            F = np.array([self.robot.tip(qq)[:3, 3] for qq in self.path])
            it['path'].setData(pos=F)
            it['path_end'].setData(pos=F[[0, -1]])
        _show(it['path'], on)
        _show(it['path_end'], on)
        on = bool(f['cam']) and self.cam_T is not None and self.mode == 'orbit'
        if on and changed('cam', self.cam_T, by_id=True):
            seg, eye = self._frustum()
            it['cam'].setData(pos=seg)
            it['cam_dot'].setData(pos=eye[None])
        _show(it['cam'], on)
        _show(it['cam_dot'], on)
        z = float(self.floor_z or 0.0)
        if changed('floor', round(z, 5)):
            tr = Transform3D()
            tr.translate(0.0, 0.0, z)
            it['floor'].setTransform(tr)
            it['grid'].setTransform(tr)
        on = self.target_T is not None
        if on and changed('target', (tuple(np.round(np.asarray(self.target_T)[:3].ravel(), 5)), self._view_key())):
            P, C = self._target_lines()
            it['target'].setData(pos=P, color=C)
        _show(it['target'], on)
        self._sig = sig

    def _links(self, q):
        """관절각 (°) → 링크마다 받침 ← 링크 4x4 (robot.parts 차례)."""
        fk = self.robot.fk(q)
        return np.stack([fk[name] for name, *_ in self.robot.parts])

    def _view_key(self):
        cam = tuple(np.round(self.cam_T[:3].ravel(), 5)) if self.mode == 'camera' and self.cam_T is not None else None
        return self.mode, round(self.az, 3), round(self.el, 3), round(self.dist, 4), self.view.width(), self.view.height(), cam

    def _camera(self, w, h):
        """→ (T 렌즈←받침 4x4, K 3x3) — 전 그림과 같은 식. 렌즈 좌표 = OpenCV 식 (x 오른쪽, y 아래, z 앞).
        카메라 시점 = 고정 카메라 자세 + 가로 시야 87° (D455 깊이), 둘러보기 = 세로 시야 38°."""
        w, h = max(w, 1), max(h, 1)
        if self.mode == 'camera' and self.cam_T is not None:
            T = np.linalg.inv(self.cam_T)
            f = w / 2 / math.tan(math.radians(CAM_FOV[0]) / 2)
            return T, np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1.0]])
        az, el = math.radians(self.az), math.radians(self.el)
        eye = ORBIT_TARGET + self.dist * np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
        f = ORBIT_TARGET - eye
        f /= np.linalg.norm(f)
        r = np.cross(f, [0, 0, 1.0])
        r /= np.linalg.norm(r)
        d = np.cross(r, f)
        R = np.stack([r, -d, f])
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, -R @ eye
        fy = h / 2 / math.tan(math.radians(ORBIT_FOV_V) / 2)
        return T, np.array([[fy, 0, w / 2], [0, fy, h / 2], [0, 0, 1.0]])

    def _frustum(self):
        """고정 카메라 시야 사각뿔 (16 cm) → (선분 끝점 16 x 3, 렌즈 위치)."""
        L = 0.16
        hx, hy = math.tan(math.radians(CAM_FOV[0] / 2)) * L, math.tan(math.radians(CAM_FOV[1] / 2)) * L
        local = np.array([[0, 0, 0], [hx, hy, L], [-hx, hy, L], [-hx, -hy, L], [hx, -hy, L]], float)   # 광학 좌표 (z 앞)
        P = local @ self.cam_T[:3, :3].T + self.cam_T[:3, 3]
        seg = []
        for i in range(1, 5):
            seg += [P[0], P[i], P[i], P[i % 4 + 1]]
        return np.array(seg), P[0]

    def _target_lines(self):
        """목표 공구 끝: 노란 십자 (화면에 나란히, 반지름 12 px) + 방향 축 6 cm → (선분 끝점, 색 RGBA)."""
        T0 = np.asarray(self.target_T, float)
        p0, R = T0[:3, 3], T0[:3, :3]
        T, K = self._camera(self.view.width(), self.view.height())
        z = max(float(T[2, :3] @ p0 + T[2, 3]), CLIP_M[0])
        r = 12.0 * z / K[0, 0]
        P = [p0 - r * T[0, :3], p0 + r * T[0, :3], p0 - r * T[1, :3], p0 + r * T[1, :3]]      # 렌즈 x·y 축 (받침 좌표)
        C = [_rgba(TARGET_RGB)] * 4
        for axis, col in enumerate(AXIS_RGB):
            P += [p0, p0 + 0.06 * R[:, axis]]
            C += [_rgba(col)] * 2
        return np.array(P), np.array(C)

    def _legend(self):
        """(안내 글, [(색, 범례 글)]) — 전 그림과 같은 글."""
        items = [('#c9d1e9', '로봇 = 실제 위치 (엔코더)')]
        if self.flags['u'] and self.u_q is not None:
            items.append(('#be78ff', '성형 전 명령'))
        if self.flags['obst'] and self.obst is not None:
            items.append(('#58a6ff', f'장애물 {len(self.obst[0]):,} 칸'))
        if self.near is not None and len(self.near):
            items.append(('#ff3cff', f'로봇 5.5 cm 안 지도 점 {len(self.near)} 칸 (지도 점검)'))
        if self.ghost is not None or self.target_T is not None:
            items.append(('#3fb950', '목표'))
        if self.flags['plan'] and self.path is not None:
            items.append(('#ff9632', '계획 경로 (공구 끝)'))
        if self.flags['cam'] and self.cam_T is not None:
            items.append(('#39c5cf', '고정 카메라'))
        note = self.note or ('카메라 시점 (고정 카메라)' if self.mode == 'camera' else '3D 뷰: 드래그 = 회전, 휠 = 확대, 더블클릭 = 처음 시점')
        return note, items

    # ---------- GL 이 안 될 때 ----------
    def _check_gl(self):
        v = self.view
        if v is not None and v.isVisible() and v.width() > 0 and v.height() > 0 and not v.isValid():
            self._fail('OpenGL 문맥을 만들지 못함 (DISPLAY · 그래픽 드라이버 확인)')

    def _fail(self, why):
        """GL 콜백 안에서도 부를 수 있어서 위젯 바꾸기는 다음 이벤트 때."""
        if not self._failed:
            self._failed = True
            QtCore.QTimer.singleShot(0, lambda: self._show_error(why))

    def _show_error(self, why):
        """3D 뷰 자리에 까닭을 글로 (GUI 는 계속 돎)."""
        if self._label is not None:
            return
        print(f'[scene3d] 3D 뷰를 쓸 수 없음: {why}', file=sys.stderr, flush=True)
        for w in (self.view, self._overlay):
            if w is not None:
                w.hide()
                w.deleteLater()
        self.view = self._overlay = None
        self._it = {}
        self._label = QtWidgets.QLabel('3D 뷰를 쓸 수 없음\n\n' + why)
        self._label.setAlignment(QtCore.Qt.AlignCenter)
        self._label.setWordWrap(True)
        self._label.setStyleSheet('background:#181c21; color:#d2d7de; padding:12px;')
        self.layout().addWidget(self._label)
