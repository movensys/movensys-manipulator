"""[2026-11-02 디버깅 D-7 시험] 가짜 고정 카메라 깊이: 카메라 광축에 수직인 평면 하나 (모든 픽셀 같은 깊이) 를 15 fps 로 서버가 받는 토픽
({camera_io.CAM_TOPIC}/depth/image_rect_raw) 에 냄. SIGTERM·SIGINT 로 끝.  ★ 시험 ROS 도메인에서만 (실물 카메라 토픽과 섞이지 않게) ★
  ROS_DOMAIN_ID=<시험 도메인> python3 tests/fake_depth_pub.py <깊이 mm> [너비 높이]
"""
import os
import signal
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from camera_io import CAM_TOPIC  # noqa: E402

LIVE_DOMAINS = ('', '55', '57', '70')       # 실물 GUI·카메라 (70)·동료 컨테이너 — 여기에는 안 냄
FPS = 15.0

dom = os.environ.get('ROS_DOMAIN_ID', '')
if dom in LIVE_DOMAINS:
    sys.exit(f'ROS_DOMAIN_ID={dom!r} 는 실물이 쓰는 도메인 — 시험 도메인으로')
depth_mm = int(sys.argv[1])
w, h = (int(sys.argv[2]), int(sys.argv[3])) if len(sys.argv) > 3 else (640, 480)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

stop = []
for s in (signal.SIGTERM, signal.SIGINT):
    signal.signal(s, lambda *_: stop.append(1))
rclpy.init()
node = Node('d7_fake_depth')
pub = node.create_publisher(Image, f'{CAM_TOPIC}/depth/image_rect_raw', qos_profile_sensor_data)
m = Image()
m.height, m.width, m.encoding, m.is_bigendian, m.step = h, w, '16UC1', 0, w * 2
m.header.frame_id = 'cam_fixed_depth_optical_frame'
m.data = np.full((h, w), depth_mm, np.uint16).tobytes()
print(f'가짜 깊이 {depth_mm} mm {w}x{h} {FPS:g} fps → {CAM_TOPIC}/depth/image_rect_raw (도메인 {dom})', flush=True)
n, t_next = 0, time.monotonic()
while not stop:
    m.header.stamp = node.get_clock().now().to_msg()
    pub.publish(m)
    n += 1
    t_next += 1.0 / FPS
    time.sleep(max(0.0, t_next - time.monotonic()))
print(f'끝 ({n} 장)', flush=True)
node.destroy_node()
rclpy.shutdown()
