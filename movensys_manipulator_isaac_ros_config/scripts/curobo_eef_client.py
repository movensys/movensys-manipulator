#!/usr/bin/env python3
"""Send end-effector targets [x, y, z, roll, pitch, yaw] to curobo_node, in order."""
import math
import sys

from geometry_msgs.msg import PoseStamped
from movensys_manipulator_isaac_ros_config.action import MoveEef
import rclpy
from rclpy.action import ActionClient


def _pose(frame, v):
    cr, sr = math.cos(v[3] / 2), math.sin(v[3] / 2)
    cp, sp = math.cos(v[4] / 2), math.sin(v[4] / 2)
    cy, sy = math.cos(v[5] / 2), math.sin(v[5] / 2)
    m = PoseStamped()
    m.header.frame_id = frame
    m.pose.position.x, m.pose.position.y, m.pose.position.z = v[:3]
    m.pose.orientation.w = cr * cp * cy + sr * sp * sy
    m.pose.orientation.x = sr * cp * cy - cr * sp * sy
    m.pose.orientation.y = cr * sp * cy + sr * cp * sy
    m.pose.orientation.z = cr * cp * sy - sr * sp * cy
    return m


def main():
    rclpy.init()
    node = rclpy.create_node('curobo_eef_client')
    action_name = node.declare_parameter('action_name', '/curobo/move_eef').value
    frame = node.declare_parameter('frame_id', 'world_manipulator').value
    flat = list(node.declare_parameter('target_poses', [0.0] * 6).value)
    targets = [flat[i:i + 6] for i in range(0, len(flat) - 5, 6)]
    client = ActionClient(node, MoveEef, action_name)
    node.get_logger().info(f'Waiting for {action_name}...')
    client.wait_for_server()
    ok = True
    for i, v in enumerate(targets):
        node.get_logger().info(f'Target {i}: {v}')
        fut = client.send_goal_async(MoveEef.Goal(target=_pose(frame, v)))
        rclpy.spin_until_future_complete(node, fut)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(node, rf)
        r = rf.result().result
        (node.get_logger().info if r.success else node.get_logger().error)(
            f'Target {i}: {r.message}, error {r.position_error * 1000:.1f} mm, '
            f'{r.replans} replans')
        if not r.success:
            ok = False
            break
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
