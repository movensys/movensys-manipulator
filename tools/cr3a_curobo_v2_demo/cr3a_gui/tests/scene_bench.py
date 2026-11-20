#!/usr/bin/env python3
"""[2026-10-27 R-1] 3D 뷰 (scene3d.py) 시험 · 속도 재기 — 새 GPU 그리기 vs 전 OpenCV 그리기 (같은 장면, 같은 크기 900 x 600).

  python3 -B scene_bench.py --out <폴더> [--old <전 scene3d.py>] [--gui <cr3a_model.py 폴더>] [--scene <scene3d.py 폴더>]
                            [--campose <camera_pose.json>] [--frames 40] [--fail noimport]

장면: 로봇 (경로 위 자세), 목표 자세 (반투명), 성형 전 명령 (켬), 계획 경로 120 점 + 목표 축, 고정 카메라 (등록한 보정 T_base_depth),
  지도 점검 점 50 개 (로봇 겉면 3~5 cm 밖), 바닥 = -2.3 mm, 장애물 = 없음 / 2 천 / 2 만 칸 (작업 공간 안 1 cm 칸, 무작위·seed 고정).
잼 (경우마다): refresh() = GUI 스레드가 쓰는 시간 (전 그림은 이것이 한 장 다 그리기),
  새 그림은 + paintGL + glFinish (GPU 까지 다 그린 시간), refresh + repaint (Qt 합성까지 한 번 갱신, CPU 시간), 지도 바꾸기 (refresh + 첫 그림).
캡처: <out>/new_<시점>_<경우>.png · old_… · 나란히 cmp_… (왼쪽 전, 오른쪽 새), 숫자 = <out>/bench.json
그래픽: QT_QPA_PLATFORM=offscreen + DISPLAY=:1 이면 창 없이 GPU (GLX), DISPLAY=:1 만이면 화면에 창. DISPLAY 가 없으면 GL 없음 → 글 (실패 시험).
Thor 에서는 python3 -B 로 (movensys_ws 에 __pycache__ 를 쓰지 않게).
"""
import argparse
import importlib.util
import json
import os
import statistics
import sys
import time
from pathlib import Path

sys.dont_write_bytecode = True
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
A = np.array((56.31, -40.67, -12.99, -29.16, 83.72, 55.72))     # (GUI 시안 경로의 두 끝)
B = np.array((-28.78, -42.61, -10.82, -27.41, 96.28, -26.31))
BUMP = np.array((0.0, 20.0, 15.0, 0.0, 0.0, 0.0))
FLOOR_Z = -0.0023
SIZE = (900, 600)
# 카메라 보정 파일이 없을 때 (PC) — 2026-10-27 등록한 T_base_depth (Thor movensys_ws/cr3a_gui/camera_pose.json, 소수 4 자리)
T_BASE_DEPTH = [[0.5829, 0.5250, -0.6201, 0.5236], [0.8125, -0.3788, 0.4431, -0.4295],
                [-0.0023, -0.7621, -0.6474, 0.8259], [0.0, 0.0, 0.0, 1.0]]
CASES = (('robot', 0), ('2k', 2000), ('20k', 20000))


def parse():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True)
    ap.add_argument('--old', default='', help='전 scene3d.py (나란히 비교·속도). 없으면 새 것만')
    ap.add_argument('--gui', default=str(HERE.parent), help='cr3a_model.py 가 있는 폴더')
    ap.add_argument('--scene', default='', help='새 scene3d.py 가 있는 폴더 (기본 = --gui)')
    ap.add_argument('--campose', default='/home/admin/workspaces/movensys_ws/cr3a_gui/camera_pose.json')
    ap.add_argument('--frames', type=int, default=40, help='경우마다 새 그림 자세 갱신 횟수')
    ap.add_argument('--old-s', type=float, default=4.0, help='경우마다 전 그림을 재는 최대 시간 (s)')
    ap.add_argument('--fail', default='', choices=('', 'noimport'), help='noimport = pyqtgraph 를 못 불러온 척 (글이 나와야)')
    return ap.parse_args()


def ms(xs):
    return round(statistics.median(xs) * 1000, 2) if xs else None


def voxels(rng, n):
    """작업 공간 (x -0.3~0.8, y -0.6~0.6, z 0~0.6 m) 안 1 cm 칸 가운데 n 개 (겹침 없음)."""
    if n == 0:
        return np.zeros((0, 3), np.float32)
    P = rng.uniform((-0.3, -0.6, 0.0), (0.8, 0.6, 0.6), (int(n * 1.3) + 100, 3))
    P = np.unique((np.floor(P / 0.01) + 0.5) * 0.01, axis=0)
    rng.shuffle(P)
    return P[:n].astype(np.float32)


def near_points(robot, q, rng, n=50):
    """로봇 겉면 꼭짓점에서 3~5 cm 밖 점 n 개 (지도 점검 결과처럼)."""
    fk = robot.fk(q)
    W = np.concatenate([P @ fk[name][:3, :3].T + fk[name][:3, 3] for name, P, _, _ in robot.parts])
    W = W[rng.choice(len(W), n, replace=False)]
    d = rng.normal(size=(n, 3))
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    return (W + d * rng.uniform(0.03, 0.05, (n, 1))).astype(np.float32)


def set_state(sc, st, obst):
    sc.q, sc.u_q, sc.ghost, sc.path = st['q'], st['u'], st['ghost'], st['path']
    sc.target_T, sc.cam_T, sc.near, sc.floor_z = st['target_T'], st['cam_T'], st['near'], FLOOR_Z
    sc.obst = (obst, 0.01) if len(obst) else None
    sc.flags['u'] = True


def gl_render(sc, GL):
    """새 그림 한 장: paintGL + glFinish (GPU 까지) 시간 s."""
    v = sc.view
    v.makeCurrent()
    t0 = time.perf_counter()
    v.paintGL()
    GL.glFinish()
    dt = time.perf_counter() - t0
    v.doneCurrent()
    return dt


def bench_new(sc, st, app, GL, frames):
    """자세 갱신 frames 번: refresh (CPU) / paintGL+glFinish / refresh+repaint (CPU 시간, Qt 합성 포함)."""
    t_ref, t_gpu, t_cpu = [], [], []
    path = st['path']
    for i in range(frames):
        k = 20 + (i * 3) % 80
        sc.q, sc.u_q = path[k], path[k + 6]
        t0 = time.perf_counter()
        sc.refresh()
        t_ref.append(time.perf_counter() - t0)
        t_gpu.append(gl_render(sc, GL))
        sc.q = path[k + 1]
        c0 = time.thread_time()
        sc.refresh()
        sc.view.repaint()
        t_cpu.append(time.thread_time() - c0)
        app.processEvents()
    return {'refresh_ms': ms(t_ref), 'paint_finish_ms': ms(t_gpu), 'refresh_repaint_cpu_ms': ms(t_cpu), 'n': frames}


def bench_obst(sc, obst, GL, reps=5):
    """지도 바꾸기: 같은 수의 칸을 섞어 새 배열로 → refresh (CPU) + 다음 그림 (올리기 포함)."""
    rng = np.random.default_rng(7)
    t_ref, t_draw = [], []
    for _ in range(reps):
        P = obst.copy()
        rng.shuffle(P)
        sc.obst = (P, 0.01)
        t0 = time.perf_counter()
        sc.refresh()
        t_ref.append(time.perf_counter() - t0)
        t_draw.append(gl_render(sc, GL))
    return {'obst_refresh_ms': ms(t_ref), 'obst_first_paint_ms': ms(t_draw)}


def bench_old(sc, st, app, max_s, frames):
    """전 그림: refresh() 가 한 장을 다 그림 (CPU). max_s 넘으면 그만."""
    t_ref = []
    path = st['path']
    t_end = time.perf_counter() + max_s
    for i in range(frames):
        k = 20 + (i * 3) % 80
        sc.q, sc.u_q = path[k], path[k + 6]
        t0 = time.perf_counter()
        sc.refresh()
        sc.repaint()
        t_ref.append(time.perf_counter() - t0)
        app.processEvents()
        if time.perf_counter() > t_end:
            break
    return {'refresh_ms': ms(t_ref), 'n': len(t_ref)}


def proj_check(sc, old, P):
    """같은 점 P (N x 3, 받침) 를 새 GL 행렬 (투영 · 시점, 마지막 그림 것) 과 전 그림 식 (_view · _proj) 으로 화면에 → 최대 차이 px."""
    w, h = sc.view.width(), sc.view.height()
    X = np.c_[P, np.ones(len(P))] @ (sc.view.P @ sc.view.V).T
    with np.errstate(divide='ignore', invalid='ignore'):      # 렌즈 뒤 점 (카메라 시점의 카메라 자리) 은 둘 다 NaN → 뺌
        ndc = X[:, :3] / X[:, 3:4]
        u, v = (ndc[:, 0] + 1) / 2 * w, (1 - ndc[:, 1]) / 2 * h
        T, K = old._view(w, h)
        uo, vo, _ = old._proj(P, T, K)
        d = np.hypot(u - uo, v - vo)
    return round(float(np.max(d[np.isfinite(d)])), 4)


def save_cmp(QtGui, old_img, new_img, path, title):
    """나란히 (왼쪽 전, 오른쪽 새) PNG."""
    w, h = new_img.width(), new_img.height()
    img = QtGui.QImage(2 * w + 8, h + 24, QtGui.QImage.Format_RGB32)
    img.fill(QtGui.QColor(255, 255, 255))
    p = QtGui.QPainter(img)
    p.drawImage(0, 24, old_img)
    p.drawImage(w + 8, 24, new_img)
    p.setPen(QtGui.QColor(0, 0, 0))
    p.drawText(6, 17, f'전 (OpenCV) — {title}')
    p.drawText(w + 14, 17, f'새 (GPU, pyqtgraph.opengl) — {title}')
    p.end()
    img.save(str(path))


def main():
    a = parse()
    gui = Path(a.gui)
    sys.path[:0] = [str(Path(a.scene or gui)), str(gui)]
    if a.fail == 'noimport':
        sys.modules['pyqtgraph'] = None          # import pyqtgraph → ImportError
    from PyQt5 import QtGui, QtWidgets
    app = QtWidgets.QApplication(sys.argv[:1])
    app.setStyle('Fusion')
    if 'NanumGothic' in QtGui.QFontDatabase().families():
        app.setFont(QtGui.QFont('NanumGothic', 9))
    import cr3a_model as cm
    import scene3d as S
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    robot = cm.Robot()
    info = {'platform': app.platformName(), 'display': os.environ.get('DISPLAY'), 'scene3d': S.__file__}
    print('플랫폼', info, flush=True)

    sc = S.Scene3D(robot)
    sc.resize(*SIZE)
    sc.show()
    for _ in range(20):
        app.processEvents()
        time.sleep(0.02)
    if sc.view is None:                          # GL 없음 → 글 (실패 시험)
        name = f'fail_{a.fail or "nogl"}.png'
        sc.grab().save(str(out / name))
        print('3D 뷰 대신 글:', sc._label.text().replace('\n', ' | ') if sc._label else None, '→', out / name)
        return
    if a.fail:
        print('실패 시험인데 3D 뷰가 만들어짐', file=sys.stderr)
        sys.exit(1)
    from OpenGL import GL
    sc.view.makeCurrent()
    info['gl'] = f"{GL.glGetString(GL.GL_RENDERER).decode()} / {GL.glGetString(GL.GL_VERSION).decode()}"
    info['samples'] = int(GL.glGetIntegerv(GL.GL_SAMPLES))    # 위젯 FBO 의 MSAA (format() 은 문맥 것이라 0 으로 보임)
    sc.view.doneCurrent()
    print('GL', info['gl'], 'MSAA', info['samples'], flush=True)

    rng = np.random.default_rng(1)
    s = np.linspace(0, 1, 120)[:, None]
    path = A + (B - A) * s + np.sin(np.pi * s) * BUMP
    try:
        cam_T = np.array(json.loads(Path(a.campose).read_text())['T_base_depth'], float)
        info['cam_T'] = a.campose
    except (OSError, KeyError, ValueError):
        cam_T = np.array(T_BASE_DEPTH, float)
        info['cam_T'] = 'T_BASE_DEPTH (내장)'
    st = {'q': path[40], 'u': path[46], 'ghost': B, 'path': path, 'target_T': robot.tip(B), 'cam_T': cam_T,
          'near': near_points(robot, path[40], rng)}
    vox = {name: voxels(np.random.default_rng(2), n) for name, n in CASES}

    old = None
    if a.old:
        spec = importlib.util.spec_from_file_location('scene3d_old', a.old)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        old = mod.Scene3D(robot)
        old.resize(*SIZE)
        old.show()
        app.processEvents()

    res = {'info': info, 'new': {}, 'old': {}}
    for mode in ('orbit', 'camera'):
        for case, _ in CASES:
            key = f'{mode}_{case}'
            for w in (sc, old):
                if w is None:
                    continue
                set_state(w, st, vox[case])
                if mode == 'camera':
                    w.set_camera_mode()
                else:
                    w.set_preset('대각')
                w._sig = None
                w.refresh()
            app.processEvents()
            r = bench_new(sc, st, app, GL, a.frames)
            if case != 'robot':
                r.update(bench_obst(sc, vox[case], GL))
            res['new'][key] = r
            if old is not None:
                res['old'][key] = bench_old(old, st, app, a.old_s, a.frames)
            # 캡처: 처음 자세로
            imgs = {}
            for tag, w in (('new', sc), ('old', old)):
                if w is None:
                    continue
                set_state(w, st, vox[case])
                w.refresh()
                app.processEvents()
                imgs[tag] = w.grab().toImage()
                imgs[tag].save(str(out / f'{tag}_{key}.png'))
            if old is not None:
                save_cmp(QtGui, imgs['old'], imgs['new'], out / f'cmp_{key}.png', key)
                gl_render(sc, GL)               # 시점 행렬을 지금 자세 그림 것으로
                pts = np.array([st['target_T'][:3, 3], cam_T[:3, 3], (0.0, 0.0, 0.0), (0.4, -0.3, 0.0), (0.2, 0.3, 0.5)])
                res['new'][key]['proj_vs_old_px'] = proj_check(sc, old, pts)
            print(key, '새', res['new'][key], '| 전', res['old'].get(key), flush=True)
    # 다른 시점 (2 천 칸, 그림만)
    for name in ('앞', '옆', '위'):
        imgs = {}
        for tag, w in (('new', sc), ('old', old)):
            if w is None:
                continue
            set_state(w, st, vox['2k'])
            w.set_preset(name)
            app.processEvents()
            imgs[tag] = w.grab().toImage()
            imgs[tag].save(str(out / f'{tag}_preset_{name}.png'))
        if old is not None:
            save_cmp(QtGui, imgs['old'], imgs['new'], out / f'cmp_preset_{name}.png', f'{name} (2 천 칸)')
    # GL 만 (글 없이) 한 장 — grabFramebuffer
    sc.set_preset('대각')
    app.processEvents()
    sc.view.grabFramebuffer().save(str(out / 'new_framebuffer_only.png'))
    (out / 'bench.json').write_text(json.dumps(res, ensure_ascii=False, indent=1))
    print('저장:', out, flush=True)


if __name__ == '__main__':
    main()
