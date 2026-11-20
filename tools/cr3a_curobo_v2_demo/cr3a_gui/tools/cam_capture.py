#!/usr/bin/env python3
"""RealSense 한 장 찍기 (realsense2_camera 노드를 잠깐 띄워 컬러·깊이·내부 값 저장).  ★ wmx_r2 컨테이너 안에서 ★

  cd ~/workspaces/movensys_ws/cr3a_gui && python3 -m tools.cam_capture <시리얼> <폴더> [장 수=5]   ([2026-10-27 R-1] tools/ 로 옮김 — camera_io 를 불러옴)
저장: color.png, depth.npy (컬러에 맞춘 깊이, uint16 mm), depth_raw.npy (깊이 카메라 그대로 = 왜곡 없는 핀홀), depth_vis.png,
     info.json (컬러·깊이 내부 값 K, 깊이 → 컬러 외부 값 T_color_depth). 깊이는 [장 수] 장의 가운데값 (튀는 점 줄임).
[2026-10-26] 사용자가 전체를 보는 고정 카메라를 하나 더 달았음 (D455 412243062648). 손목 D455 (412243062189) 는 지금 안 씀.
[2026-10-27 R-1] 노드 설정·실행 줄·영상 바꾸기·깊이 가운데값은 camera_io (GUI 카메라와 같은 코드).
"""
import argparse
import ctypes
import json
import signal
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from realsense2_camera_msgs.msg import Extrinsics
from sensor_msgs.msg import CameraInfo, Image

from camera_io import depth_median, realsense_argv, realsense_yaml
from camera_io import ros_image_to_np as to_np

NS = '/cam_cap'


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('serial', help='RealSense 시리얼 (예: 고정 카메라 412243062648)')
    ap.add_argument('out', help='저장 폴더')
    ap.add_argument('n', nargs='?', type=int, default=5, help='깊이 가운데값을 낼 장 수 (기본 5)')
    a = ap.parse_args()
    serial, out, n = a.serial, Path(a.out), a.n
    out.mkdir(parents=True, exist_ok=True)
    params = out / 'cam_params.yaml'
    params.write_text(realsense_yaml(f'{NS}/camera', serial, 640, 480, 15))
    # [2026-10-26] timeout 이 이 스크립트를 먼저 끝내 카메라 노드가 고아로 남아, 없는 시리얼을 4 s 마다 찾으며 다른 D455 를 계속
    #  리셋했음 (USB 끊김 반복) → 이 프로세스가 끝나면 노드도 SIGTERM 을 받게 (PR_SET_PDEATHSIG), SIGTERM 도 finally 로
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    libc = ctypes.CDLL('libc.so.6', use_errno=True)
    with open(out / 'camera_node.log', 'w') as log:         # (자식만 씀 — 띄운 뒤 닫음)
        proc = subprocess.Popen(realsense_argv(NS, params), stdout=log, stderr=subprocess.STDOUT,
                                preexec_fn=lambda: libc.prctl(1, signal.SIGTERM))
    rclpy.init()
    node = Node('cam_capture')
    got = {'color': [], 'depth': [], 'raw': [], 'cinfo': None, 'dinfo': None, 'ext': None}
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Extrinsics, f'{NS}/camera/extrinsics/depth_to_color', lambda m: got.__setitem__('ext', m), latched)
    node.create_subscription(Image, f'{NS}/camera/color/image_raw', lambda m: got['color'].append(to_np(m)), qos_profile_sensor_data)
    node.create_subscription(Image, f'{NS}/camera/aligned_depth_to_color/image_raw', lambda m: got['depth'].append(to_np(m)),
                             qos_profile_sensor_data)
    node.create_subscription(Image, f'{NS}/camera/depth/image_rect_raw', lambda m: got['raw'].append(to_np(m)), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, f'{NS}/camera/color/camera_info', lambda m: got.__setitem__('cinfo', m), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, f'{NS}/camera/depth/camera_info', lambda m: got.__setitem__('dinfo', m), qos_profile_sensor_data)
    t0 = time.time()
    try:
        # 처음 몇 장은 자동 노출이 자리 잡는 중이라 버림 (2 s 뒤부터 모음)
        while time.time() - t0 < 40:
            rclpy.spin_once(node, timeout_sec=0.1)
            if proc.poll() is not None:
                raise SystemExit(f'카메라 노드가 끝남 (코드 {proc.returncode}) — {out / "camera_node.log"}')
            if time.time() - t0 < 2.0 or not (got['color'] and got['depth']):
                if time.time() - t0 < 2.0:
                    for k in ('color', 'depth', 'raw'):
                        got[k].clear()
                continue
            if min(len(got['color']), len(got['depth']), len(got['raw'])) >= n and got['cinfo'] and got['dinfo'] and got['ext']:
                break
        else:
            raise SystemExit('40 s 안에 영상을 다 못 받음')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        proc.send_signal(2)
        try:
            proc.wait(10)
        except subprocess.TimeoutExpired:
            proc.kill()
    color = got['color'][-1]
    depth = depth_median(got['depth'][-n:])
    raw = depth_median(got['raw'][-n:])
    cv2.imwrite(str(out / 'color.png'), color)
    np.save(out / 'depth.npy', depth)
    np.save(out / 'depth_raw.npy', raw)
    vis = cv2.applyColorMap(cv2.convertScaleAbs(np.clip(depth, 0, 3000), alpha=255 / 3000), cv2.COLORMAP_TURBO)
    vis[depth == 0] = 0
    cv2.imwrite(str(out / 'depth_vis.png'), vis)
    ci, di, ex = got['cinfo'], got['dinfo'], got['ext']
    # 깊이 → 컬러 (librealsense rs2_extrinsics: rotation = 열 우선 3x3, p_color = R·p_depth + t)
    T_cd = np.eye(4)
    T_cd[:3, :3] = np.array(ex.rotation, float).reshape(3, 3).T
    T_cd[:3, 3] = ex.translation
    info = {'serial': serial, 'stamp': time.time(), 'n': n, 'T_color_depth': T_cd.tolist(),
            'color': {'w': ci.width, 'h': ci.height, 'K': list(ci.k), 'D': list(ci.d), 'model': ci.distortion_model, 'frame': ci.header.frame_id},
            'depth': {'w': di.width, 'h': di.height, 'K': list(di.k), 'D': list(di.d), 'model': di.distortion_model, 'frame': di.header.frame_id},
            'valid_ratio': round(float((depth > 0).mean()), 3)}
    (out / 'info.json').write_text(json.dumps(info, indent=1))
    print(f'저장 {out}: 컬러 {color.shape}, 깊이 {depth.shape} (쓸 수 있는 점 {info["valid_ratio"] * 100:.0f} %)')


if __name__ == '__main__':
    main()
