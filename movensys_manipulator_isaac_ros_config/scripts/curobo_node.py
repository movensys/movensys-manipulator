#!/usr/bin/env python3
"""cuRoboV2 end-effector controller.

MoveEef action (target pose) -> IK + global plan -> MPC follows the path at rate_hz, avoiding
camera obstacles (depth -> robot mask -> ESDF) -> jerk-limited JointTrajectory ->
dynamic_trajectory_controller. The path is checked against the live ESDF and replanned when
blocked, when the MPC is pushed off it, or when progress stalls. RViz: drag the goal marker,
PLAN, EXECUTE.
"""
import math
import os
import threading
import time
import types

from builtin_interfaces.msg import Duration, Time
from geometry_msgs.msg import Point, Pose as PoseMsg
from interactive_markers import InteractiveMarkerServer, MenuHandler
from movensys_manipulator_isaac_ros_config.action import MoveEef
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState as JointStateMsg
import tf2_ros
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import (InteractiveMarker, InteractiveMarkerControl,
                                    InteractiveMarkerFeedback, Marker)

WALL_DECEL = 0.8
WALL_INSET = 1e-3


def _clip(x, lo, hi):
    x = x if x > lo else lo
    return x if x < hi else hi


def _per_joint(x, n):
    return np.broadcast_to(np.asarray(x, float), (n,)).astype(float)


def _stop_consts(a, j):
    d0 = a ** 3 / j ** 2
    return d0, -a * a / (2.0 * j), a ** 4 / (4.0 * j * j), 2.0 * a


def _stop_speed(d, k, j):
    return math.cbrt(d * d * j) if d <= k[0] else k[1] + math.sqrt(k[2] + k[3] * d)


def _stop_distance(v, a, j):
    v = np.abs(v)
    return np.where(v <= a * a / j, v * np.sqrt(v / j), v * v / (2.0 * a) + v * a / (2.0 * j))


def _track_target(q0, v0, a0, target, n, h, vmax, amax, jmax, cycle, sub=5,
                  stop_decel=1.0, qlim=None):
    q, v, a = np.array(q0, float), np.array(v0, float), np.array(a0, float)
    tgt = np.asarray(target, float)
    nj = q.size
    vm, am, jm = _per_joint(vmax, nj), _per_joint(amax, nj), _per_joint(jmax, nj)
    lo_lim = hi_lim = None
    if qlim is not None:
        lo_lim = np.asarray(qlim[0], float) + WALL_INSET
        hi_lim = np.asarray(qlim[1], float) - WALL_INSET
        tgt = np.clip(tgt, lo_lim, hi_lim)
    hh = h / sub
    den = max(cycle / 4.0, hh)
    out_q, out_v, out_a = np.zeros((n, nj)), np.zeros((n, nj)), np.zeros((n, nj))
    for i in range(nj):
        qi, vi, ai, ti = float(q[i]), float(v[i]), float(a[i]), float(tgt[i])
        vmi, ami, ji = float(vm[i]), float(am[i]), float(jm[i])
        ks = _stop_consts(stop_decel * ami, ji)
        kw = _stop_consts(WALL_DECEL * ami, ji)
        for k in range(n):
            for _ in range(sub):
                e = ti - qi
                vs = _stop_speed(abs(e), ks, ji)
                vd = _clip(_clip(e / cycle, -vs, vs), -vmi, vmi)
                if lo_lim is not None:
                    dl, dh = qi - float(lo_lim[i]), float(hi_lim[i]) - qi
                    vd = _clip(vd, -_stop_speed(max(dl, 0.0), kw, ji),
                               _stop_speed(max(dh, 0.0), kw, ji))
                ad = _clip((vd - vi) / den, -ami, ami)
                hi = min(ami, math.sqrt(2.0 * ji * max(vmi - vi, 0.0)))
                lo = -min(ami, math.sqrt(2.0 * ji * max(vmi + vi, 0.0)))
                an = _clip(_clip(ad, lo, hi), ai - ji * hh, ai + ji * hh)
                qi = qi + vi * hh + 0.5 * an * hh * hh
                vi = _clip(vi + an * hh, -vmi, vmi)
                ai = an
            out_q[k, i], out_v[k, i], out_a[k, i] = qi, vi, ai
    return out_q, out_v, out_a


def _stopping_tail(q, v, a, h, vmax, amax, jmax, max_s=3.0, qlim=None, stop_decel=0.8):
    q, v, a = np.asarray(q, float), np.asarray(v, float), np.asarray(a, float)
    nj = q.size
    am, jm = _per_joint(amax, nj), _per_joint(jmax, nj)
    hold = q + np.sign(v) * _stop_distance(v, stop_decel * am, jm)
    chunk = max(1, int(round(0.1 / h)))
    parts = []
    for _ in range(int(np.ceil(max_s / (chunk * h)))):
        part = _track_target(q, v, a, hold, chunk, h, vmax, amax, jmax, cycle=4.0 * h,
                             stop_decel=stop_decel, qlim=qlim)
        parts.append(part)
        q, v, a = part[0][-1], part[1][-1], part[2][-1]
        if np.max(np.abs(v)) < 1e-4 and np.max(np.abs(a)) < 1e-3:
            break
    return tuple(np.vstack([p[k] for p in parts]) for k in range(3))


def _duration(t):
    sec = int(np.floor(t))
    return Duration(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1000000000)


def _stamp(t):
    sec = int(np.floor(t))
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1000000000)


def _quat_to_matrix(qw, qx, qy, qz):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ])


def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw]


def _compose(a, b):
    p = np.asarray(a[:3]) + _quat_to_matrix(*a[3:]) @ np.asarray(b[:3])
    return p.tolist() + _quat_mul(a[3:], b[3:])


def _quat_angle(a, b):
    d = abs(float(np.dot(np.asarray(a), np.asarray(b))))
    return 2.0 * math.acos(min(1.0, d))


class Curobo(Node):

    def __init__(self):
        super().__init__('curobo')
        p = self.declare_parameter
        default_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_names = list(p('joint_names', default_joints).value)
        self.urdf_path = p('urdf_path', '').value
        self.xrdf_path = p('xrdf_path', '').value
        self.base_frame = p('base_frame', 'world_manipulator').value

        self.action_name = p('action_name', '/curobo/move_eef').value
        self.output_topic = p('output_topic',
                              '/dynamic_trajectory_controller/joint_trajectory').value
        joint_state_topic = p('joint_state_topic', '/joint_states').value
        depth_topic = p('depth_topic', '/image_nvblox/depth').value
        camera_info_topic = p('camera_info_topic', '/image_nvblox/camera_info').value
        obstacles_topic = p('obstacles_topic', '/curobo/obstacles').value
        goal_js_topic = p('goal_joint_state_topic', '/curobo/goal_joint_states').value
        status_topic = p('goal_status_topic', '/curobo/goal_status').value
        path_topic = p('planned_path_topic', '/curobo/planned_path').value
        self.im_namespace = p('interactive_marker_namespace', 'curobo_goal').value

        self.rate_hz = float(p('rate_hz', 20.0).value)
        self.interpolation_steps = int(p('interpolation_steps', 4).value)
        self.warm_iters = int(p('warm_iters', 100).value)
        self.cold_iters = int(p('cold_iters', 150).value)
        self.activation = float(p('collision_activation_distance', 0.06).value)
        self.floor_activation = float(p('floor_activation_distance', 0.02).value)
        self.diverge_rad = float(p('diverge_rad', 1.0).value)
        self.lookahead_rad = float(p('path_lookahead_rad', 0.1).value)
        self.max_lead_rad = float(p('path_max_lead_rad', 0.4).value)
        self.resync_rad = float(p('resync_rad', 0.2).value)

        self.integrate_rate_hz = float(p('integrate_depth_rate_hz', 15.0).value)
        self.esdf_rate_hz = float(p('update_esdf_rate_hz', 10.0).value)
        self.segment_distance = float(p('segment_distance', 0.04).value)
        self.map_center = list(p('map_center', [-0.03, 0.075, 0.43]).value)
        self.map_size = list(p('map_size', [1.3, 1.3, 0.8]).value)
        self.esdf_voxel = float(p('esdf_voxel_size', 0.01).value)
        self.tsdf_voxel = float(p('tsdf_voxel_size', 0.005).value)
        self.truncation = float(p('truncation_distance', 0.08).value)
        self.depth_min = float(p('depth_min', 0.1).value)
        self.depth_max = float(p('depth_max', 2.0).value)
        self.decay = float(p('decay', 0.7).value)
        self.floor_z = float(p('floor_z', 0.0).value)
        self.floor_band = float(p('floor_band', 0.015).value)
        self.base_exclude = list(p('base_exclude', [0.0] * 6).value)
        self.warmup_frames = int(p('warmup_frames', 15).value)
        self.depth_scale = float(p('depth_scale_16u', 0.001).value)
        self.marker_period_s = float(p('marker_period_s', 0.5).value)

        self.jerk = float(p('jerk', 500.0).value)
        self.stop_decel = float(p('stop_decel', 0.8).value)
        self.speed_scale = float(p('speed_scale', 1.0).value)
        self.step_s = float(p('step_s', 0.005).value)
        self.cycles = int(p('cycles', 2).value)
        self.lead_s = float(p('lead_s', 0.05).value)
        self.late_margin_s = float(p('late_margin_s', 0.01).value)
        self.arrive_tol = float(p('arrive_tolerance', 0.002).value)

        self.ik_num_seeds = int(p('ik_num_seeds', 32).value)
        self.plan_activation = float(p('plan_collision_activation_distance', 0.03).value)
        self.plan_dt = float(p('plan_interpolation_dt', 0.02).value)
        self.exec_lookahead = float(p('path_follow_lookahead_rad', 0.25).value)
        self.goal_timeout = float(p('goal_timeout_s', 60.0).value)
        self.plan_timeout = float(p('plan_timeout_s', 10.0).value)
        self.replan_period = float(p('replan_period_s', 0.3).value)
        self.check_period = 1.0 / float(p('path_check_rate_hz', 10.0).value)
        self.path_clearance = float(p('path_clearance', 0.01).value)
        self.replan_deviation = float(p('replan_deviation_rad', 0.3).value)
        self.stall_timeout = float(p('stall_timeout_s', 2.0).value)
        self.ui_rate_hz = float(p('ui_rate_hz', 10.0).value)
        self.marker_scale = float(p('goal_marker_scale', 0.2).value)
        self.dt = 1.0 / self.rate_hz

        self._load_curobo()
        self.robot = self._robot_config()

        self.js = None
        self.js_full = None
        self.depth_msg = None
        self.depth_seq = 0
        self.integrated_seq = 0
        self.info = None
        self.cam = None
        self.frames = 0
        self.grid = None
        self.mpc = None
        self.ik = None
        self.plan = None
        self.stamp = 0.0
        self.active = False
        self.exec = None
        self.request = None
        self.req_lock = threading.Lock()
        self.live_follow = False
        self.goal = None
        self.goal_pose = None
        self.goal_pose_dirty = False
        self.goal_set = False
        self.goal_q = None
        self.goal_ok = False
        self.path_plan = None
        self.status = ('', 'info')
        self.last_esdf = 0.0
        self.last_marker = 0.0
        self.cycle_times = []
        self.late = 0

        self.action_node = rclpy.create_node('curobo_action', use_global_arguments=False)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(JointStateMsg, joint_state_topic, self._on_joint_state, 10)
        self.create_subscription(Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, camera_info_topic, self._on_info,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(JointTrajectory, self.output_topic, 10)
        self.marker_pub = self.create_publisher(Marker, obstacles_topic, 1)
        self.goal_js_pub = self.create_publisher(JointStateMsg, goal_js_topic, 1)
        self.status_pub = self.create_publisher(Marker, status_topic, 1)
        self.path_pub = self.create_publisher(Marker, path_topic, 1)
        self.create_timer(1.0 / self.integrate_rate_hz, self._map_cycle)
        self.create_timer(1.0 / self.ui_rate_hz, self._ui_cycle)
        self.create_timer(self.dt, self._cycle)
        self.get_logger().info(f'{self.rate_hz:.0f} Hz MPC -> {self.output_topic}')

    # ------------------------------------------------------------------ setup

    def _load_curobo(self):
        import torch
        from curobo._src.robot.loader.util import load_robot_yaml
        from curobo._src.types.robot import RobotCfg
        from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
        from curobo.kinematics import Kinematics
        from curobo.model_predictive_control import (ModelPredictiveControl,
                                                     ModelPredictiveControlCfg)
        from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
        from curobo.perception import FilterDepth, Mapper, MapperCfg, RobotSegmenter
        from curobo.scene import Cuboid, Scene, VoxelGrid
        from curobo.types import CameraObservation, ContentPath, GoalToolPose, JointState, Pose
        self.torch = torch
        self.device = torch.device('cuda:0')
        self.cu = {
            'load_robot_yaml': load_robot_yaml, 'MPC': ModelPredictiveControl,
            'MPCCfg': ModelPredictiveControlCfg, 'FilterDepth': FilterDepth, 'Mapper': Mapper,
            'MapperCfg': MapperCfg, 'RobotSegmenter': RobotSegmenter, 'Cuboid': Cuboid,
            'Scene': Scene, 'VoxelGrid': VoxelGrid, 'CameraObservation': CameraObservation,
            'ContentPath': ContentPath, 'JointState': JointState, 'Pose': Pose,
            'RobotCfg': RobotCfg, 'Kinematics': Kinematics, 'IK': InverseKinematics,
            'IKCfg': InverseKinematicsCfg, 'GoalToolPose': GoalToolPose,
            'MotionPlanner': MotionPlanner, 'MotionPlannerCfg': MotionPlannerCfg,
        }

    def _robot_config(self):
        content = self.cu['ContentPath'](
            robot_xrdf_absolute_path=self.xrdf_path, robot_urdf_absolute_path=self.urdf_path,
            robot_asset_absolute_path=os.path.dirname(self.xrdf_path))
        return self.cu['load_robot_yaml'](content)

    def _on_joint_state(self, msg):
        try:
            idx = [list(msg.name).index(n) for n in self.joint_names]
        except ValueError:
            return
        self.js = np.array([msg.position[i] for i in idx], float)
        self.js_full = (list(msg.name), list(msg.position))

    def _on_depth(self, msg):
        self.depth_msg = msg
        self.depth_seq += 1

    def _on_info(self, msg):
        if self.info is None:
            self.info = msg

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _joint_state(self, q, v=None, a=None):
        t = self.torch
        pos = t.as_tensor(np.asarray(q, np.float32).reshape(1, -1), device=self.device)
        st = self.cu['JointState'].from_position(pos, joint_names=list(self.joint_names))
        st.velocity = t.zeros_like(pos) if v is None else t.as_tensor(
            np.asarray(v, np.float32).reshape(1, -1), device=self.device)
        st.acceleration = t.zeros_like(pos) if a is None else t.as_tensor(
            np.asarray(a, np.float32).reshape(1, -1), device=self.device)
        return st

    # ------------------------------------------------------------------ map

    def _map_cycle(self):
        if self.js is None or self.info is None or self.depth_msg is None:
            return
        if self.cam is None:
            self._setup_camera()
        self._integrate()
        now = time.monotonic()
        if self.frames >= self.warmup_frames and now - self.last_esdf >= 1.0 / self.esdf_rate_hz:
            self.last_esdf = now
            self._esdf()
            self._publish_obstacles()

    def _setup_camera(self):
        t = self.torch
        h, w = self.info.height, self.info.width
        k = np.asarray(self.info.k, np.float32).reshape(3, 3)
        self.cam = {'h': h, 'w': w, 'k': t.as_tensor(k, device=self.device).view(1, 3, 3),
                    'frame': self.info.header.frame_id}
        self.kin = self.cu['Kinematics'](self.cu['RobotCfg'].create(self.robot).kinematics)
        self.segmenter = self.cu['RobotSegmenter'](
            self.kin, distance_threshold=self.segment_distance, use_cuda_graph=False,
            ops_dtype=t.float32)
        self.segmenter.update_camera_projection(self.cu['CameraObservation'](
            depth_image=t.zeros((1, h, w), device=self.device), intrinsics=self.cam['k'],
            depth_to_meter=1.0))
        self.filter = self.cu['FilterDepth'](
            image_shape=(h, w), depth_minimum_distance=self.depth_min,
            depth_maximum_distance=self.depth_max, flying_pixel_threshold=0.5,
            bilateral_kernel_size=5)
        n = [1 + int(np.floor(s / self.esdf_voxel + 1e-6)) for s in self.map_size]
        self.grid_shape = n
        self.mapper = self.cu['Mapper'](self.cu['MapperCfg'](
            extent_meters_xyz=tuple(s + 0.2 for s in self.map_size),
            voxel_size=self.tsdf_voxel, esdf_voxel_size=self.esdf_voxel,
            extent_esdf_meters_xyz=tuple((c - 0.5) * self.esdf_voxel for c in n),
            grid_center=t.tensor(self.map_center, dtype=t.float32),
            truncation_distance=self.truncation,
            depth_minimum_distance=self.depth_min, depth_maximum_distance=self.depth_max,
            decay_factor=self.decay, minimum_tsdf_weight=0.1, num_cameras=1,
            image_height=h, image_width=w, device=str(self.device)))
        vv, uu = t.meshgrid(t.arange(h, device=self.device, dtype=t.float32),
                            t.arange(w, device=self.device, dtype=t.float32), indexing='ij')
        self.rays = t.stack([(uu - float(k[0, 2])) / float(k[0, 0]),
                             (vv - float(k[1, 2])) / float(k[1, 1]), t.ones_like(uu)], dim=-1)
        self.rgb = t.zeros((1, h, w, 3), dtype=t.uint8, device=self.device)
        self.memory = t.zeros((1, h, w), device=self.device)
        self.get_logger().info(f'Camera {w}x{h} in frame {self.cam["frame"]}, map {n} voxels')

    def _lookup(self, frame):
        if not frame or frame == self.base_frame:
            return [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame, rclpy.time.Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'No transform {self.base_frame} <- {frame}: {e}',
                                   throttle_duration_sec=2.0)
            return None
        tr, r = tf.transform.translation, tf.transform.rotation
        return [tr.x, tr.y, tr.z, r.w, r.x, r.y, r.z]

    def _depth_tensor(self, msg):
        h, w = self.cam['h'], self.cam['w']
        if msg.encoding in ('16UC1', 'mono16'):
            raw = np.frombuffer(msg.data, np.uint16).reshape(h, msg.step // 2)[:, :w]
            depth = raw.astype(np.float32) * self.depth_scale
        elif msg.encoding == '32FC1':
            depth = np.frombuffer(msg.data, np.float32).reshape(h, msg.step // 4)[:, :w]
        else:
            self.get_logger().error(f'Unsupported depth encoding {msg.encoding}',
                                    throttle_duration_sec=5.0)
            return None
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        return self.torch.as_tensor(np.ascontiguousarray(depth), device=self.device).view(1, h, w)

    def _integrate(self):
        if self.depth_seq == self.integrated_seq:
            return
        self.integrated_seq = self.depth_seq
        pose_list = self._lookup(self.cam['frame'])
        depth = self._depth_tensor(self.depth_msg) if pose_list is not None else None
        if depth is None:
            return
        pose = self.cu['Pose'].from_list(pose_list)
        obs = self.cu['CameraObservation']
        _, world = self.segmenter.get_robot_mask_from_active_js(
            obs(depth_image=depth.clone(), pose=pose), self._joint_state(self.js))
        seen = world > 0
        self.memory = self.torch.where(seen, world, self.memory)
        world = self.torch.where((depth > 0) & ~seen, self.memory, world)
        filtered, _ = self.filter(world.contiguous())
        rot = self.torch.as_tensor(_quat_to_matrix(*pose_list[3:]), dtype=self.torch.float32,
                                   device=self.device)
        pts = (self.rays @ rot.T) * filtered[0].unsqueeze(-1) + self.torch.as_tensor(
            pose_list[:3], dtype=self.torch.float32, device=self.device)
        keep = pts[..., 2] > self.floor_z + self.floor_band
        x0, y0, z0, x1, y1, z1 = self.base_exclude
        if x1 > x0 and y1 > y0 and z1 > z0:
            inside = ((pts[..., 0] > x0) & (pts[..., 0] < x1) & (pts[..., 1] > y0)
                      & (pts[..., 1] < y1) & (pts[..., 2] > z0) & (pts[..., 2] < z1))
            keep = keep & ~inside
        self.mapper.integrate(obs(depth_image=filtered * keep.unsqueeze(0), rgb_image=self.rgb,
                                  pose=pose, intrinsics=self.cam['k']))
        self.frames += 1

    def _esdf(self):
        full = self.mapper.compute_esdf()
        if self.grid is None:
            self.grid = self.cu['VoxelGrid'](
                name='camera_esdf', pose=list(full.pose), dims=list(full.dims),
                voxel_size=full.voxel_size, feature_tensor=full.feature_tensor.clone())
        else:
            self.grid.feature_tensor.copy_(full.feature_tensor)

    def _publish_obstacles(self):
        now = time.monotonic()
        if (now - self.last_marker < self.marker_period_s
                or self.marker_pub.get_subscription_count() == 0):
            return
        self.last_marker = now
        nx, ny, nz = self.grid_shape
        values = self.grid.feature_tensor.view(-1)[:nx * ny * nz].view(nx, ny, nz).float()
        idx = (values < 0.5 * self.esdf_voxel).nonzero()
        if idx.shape[0] > 20000:
            idx = idx[:: idx.shape[0] // 20000 + 1]
        half = (np.asarray(self.grid_shape) - 1) * self.esdf_voxel / 2
        pts = idx.cpu().numpy() * self.esdf_voxel + (np.asarray(self.grid.pose[:3]) - half)
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'obstacles', 0, Marker.CUBE_LIST, Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = self.esdf_voxel
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.3, 0.1, 0.8
        m.pose.orientation.w = 1.0
        m.points = [Point(x=float(q[0]), y=float(q[1]), z=float(q[2])) for q in pts]
        self.marker_pub.publish(m)

    # ------------------------------------------------------------------ solvers

    def _create_solvers(self):
        floor = self.cu['Cuboid'](
            name='floor', dims=[3.0, 3.0, 0.02],
            pose=[0.0, 0.0, self.floor_z - 0.01 - (self.activation - self.floor_activation),
                  1.0, 0.0, 0.0, 0.0])
        self.scene = self.cu['Scene'](cuboid=[floor], voxel=[self.grid])
        self.mpc = self.cu['MPC'](self.cu['MPCCfg'].create(
            robot=self.robot, scene_model=self.scene,
            use_cuda_graph=True, optimization_dt=self.dt,
            interpolation_steps=self.interpolation_steps,
            optimizer_collision_activation_distance=self.activation,
            warm_start_optimization_num_iters=self.warm_iters,
            cold_start_optimization_num_iters=self.cold_iters))
        if list(self.mpc.joint_names) != self.joint_names:
            raise RuntimeError(f'cuRobo joints {self.mpc.joint_names} differ from joint_names')
        lim = self.mpc.kinematics.get_joint_limits()
        k = [list(lim.joint_names).index(n) for n in self.joint_names]
        self.vmax = lim.velocity[1].cpu().numpy()[k].astype(float) * self.speed_scale
        self.amax = lim.acceleration[1].cpu().numpy()[k].astype(float) * self.speed_scale
        self.qlim = (lim.position[0].cpu().numpy()[k].astype(float),
                     lim.position[1].cpu().numpy()[k].astype(float))
        state = self._joint_state(self.js)
        self.mpc.setup(state)
        self.mpc.disable_tool_pose_tracking()
        self.mpc.enable_joint_position_tracking()
        self.mpc_state = state
        self.path = [self.js.copy()]
        self._set_goal(self.js.copy())
        self.mpc.optimize_action_sequence(self.mpc_state)

        self.ik = self.cu['IK'](self.cu['IKCfg'].create(
            robot=self.robot, scene_model=self.scene, num_seeds=self.ik_num_seeds,
            optimizer_collision_activation_distance=self.plan_activation, use_cuda_graph=True))
        self.tool_frame = self.ik.tool_frames[0]
        self.planner = self.cu['MotionPlanner'](self.cu['MotionPlannerCfg'].create(
            robot=self.robot, scene_model=self.scene,
            optimizer_collision_activation_distance=self.plan_activation,
            interpolation_dt=self.plan_dt, use_cuda_graph=True))
        self.get_logger().info('Warming up IK and planner (CUDA graph capture, ~1 min)...')
        for _ in range(2):
            self._ik_solve(self._tool_pose(self.js), self.js)
        self.planner.warmup(enable_graph=True, num_warmup_iterations=2)
        self._path_valid(np.tile(self.js, (3, 1)))
        self._create_ui()
        self.action_server = ActionServer(
            self.action_node, MoveEef, self.action_name, self._on_action,
            cancel_callback=lambda gh: CancelResponse.ACCEPT)
        self.get_logger().info(f'cuRobo ready, action {self.action_name}')

    def _tool_pose(self, q):
        pose = self.kin.compute_kinematics(self._joint_state(q)).tool_poses.to_dict()[
            self.tool_frame]
        return pose.position.view(-1).cpu().tolist() + pose.quaternion.view(-1).cpu().tolist()

    def _ik_solve(self, pose, seed_q):
        t = self.torch
        gp = self.cu['GoalToolPose'](
            tool_frames=[self.tool_frame],
            position=t.tensor(pose[:3], device=self.device, dtype=t.float32).view(1, 1, 1, 1, 3),
            quaternion=t.tensor(pose[3:], device=self.device,
                                dtype=t.float32).view(1, 1, 1, 1, 4))
        seed = t.as_tensor(np.asarray(seed_q, np.float32), device=self.device).view(1, 1, -1)
        r = self.ik.solve_pose(gp, current_state=self._joint_state(seed_q), seed_config=seed)
        ok = bool(r.success.view(-1)[0].item())
        return ok, r.solution.view(-1, len(self.joint_names))[0].cpu().numpy().astype(float)

    def _plan_path(self, start_q, goal_q):
        r = self.planner.plan_cspace(self._joint_state(goal_q), self._joint_state(start_q))
        if r is None or not bool(r.success.any().item()):
            self.plan_fail = getattr(r, 'status', None) if r is not None else 'no graph path'
            return None
        js = r.get_interpolated_plan()
        names = list(js.joint_names) if js.joint_names else self.joint_names
        cols = [names.index(n) for n in self.joint_names]
        return js.position.view(-1, js.position.shape[-1])[:, cols].cpu().numpy().astype(float)

    def _path_valid(self, path):
        t = self.torch
        q = t.as_tensor(np.asarray(path, np.float32), device=self.device).view(
            -1, len(self.joint_names))
        sph = self.kin.compute_kinematics(self.cu['JointState'].from_position(
            q, joint_names=self.joint_names)).robot_spheres.view(q.shape[0], -1, 4)
        n = t.tensor(self.grid_shape, device=self.device)
        origin = t.tensor(self.grid.pose[:3], device=self.device, dtype=t.float32) - \
            (n - 1) * self.esdf_voxel / 2
        idx = t.round((sph[..., :3] - origin) / self.esdf_voxel).long()
        inside = ((idx >= 0) & (idx < n)).all(-1) & (sph[..., 3] > 0)
        if not bool(inside.any()):
            return True
        nx, ny, nz = self.grid_shape
        esdf = self.grid.feature_tensor.view(-1)[:nx * ny * nz].view(nx, ny, nz)
        i = idx[inside]
        dist = esdf[i[:, 0], i[:, 1], i[:, 2]].float() - sph[..., 3][inside]
        return bool((dist >= self.path_clearance).all().item())

    # ------------------------------------------------------------------ execution

    def _set_goal(self, q, label=None):
        self.goal = np.asarray(q, float)
        self.mpc.update_goal_state(self._joint_state(self.goal))
        if label:
            self.get_logger().info(f'{label}: {np.round(np.degrees(self.goal), 1).tolist()} deg')

    def _activate(self):
        if not self.active:
            self._restart_mpc(self.js)
            self.plan = None
            self.active = True

    def _submit(self, req):
        with self.req_lock:
            old, self.request = self.request, req
        if old is not None:
            old.result = self._result(False, 'preempted', old)
            old.done.set()

    def _begin(self, req):
        if self.exec is not None:
            self._finish(False, 'preempted')
        if self.live_follow:
            self._toggle_follow()
        now = self._now()
        self.exec = {
            'req': req, 'pose': req.pose, 'goal_q': req.goal_q, 'path': req.path, 'idx': 0,
            't0': now, 'last_try': -1e9, 'last_check': now, 'replans': 0, 'planned': False,
            'progress_idx': 0, 'progress_t': now, 'state': 'PLANNING', 'pos_err': float('nan')}
        if req.path is not None:
            self.exec.update(planned=True, state='MOVING')
            self._draw_path(req.path)
            self._activate()
        self.get_logger().info(
            f'New goal {np.round(req.pose[:3], 3).tolist()} ({req.source})')

    def _replan(self, q):
        e = self.exec
        seed = e['goal_q'] if e['goal_q'] is not None else q
        ok, gq = self._ik_solve(e['pose'], seed)
        if not ok:
            ok, gq = self._ik_solve(e['pose'], q)
        if not ok:
            self.get_logger().warn('Replan: no IK solution for target', throttle_duration_sec=2.0)
            return False
        path = self._plan_path(q, gq)
        if path is None and e['path'] is not None:
            wp, i = e['path'], e['idx']
            for k in (15, 40):
                j = max(i - k, 0)
                tail = self._plan_path(wp[j], gq)
                if tail is not None:
                    path = np.vstack([wp[j:i + 1][::-1], tail])
                    break
                if j == 0:
                    break
        if path is None:
            self.get_logger().warn(f'Replan failed: {self.plan_fail}, start clear: '
                                   f'{self._path_valid(q[None])}', throttle_duration_sec=2.0)
            return False
        if e['planned']:
            e['replans'] += 1
            self.get_logger().info(f'Replanned ({e["replans"]})')
        e.update(goal_q=gq, path=path, idx=0, planned=True, state='MOVING',
                 progress_idx=0, progress_t=self._now())
        self.goal_q, self.goal_ok = gq, True
        self._draw_path(path)
        return True

    def _exec_update(self, s0):
        e = self.exec
        q, v, _ = s0
        now = self._now()
        if e['req'].cancel:
            return self._finish(False, 'canceled')
        if now - e['t0'] > self.goal_timeout:
            return self._finish(False, f'timeout after {self.goal_timeout:.0f} s')
        if e['state'] in ('PLANNING', 'REPLANNING', 'BLOCKED'):
            if not e['planned'] and now - e['t0'] > self.plan_timeout:
                return self._finish(False, 'no collision-free plan (unreachable or blocked)')
            if now - e['last_try'] >= self.replan_period:
                e['last_try'] = now
                if self._replan(q):
                    self._activate()
                elif e['planned']:
                    if e['state'] != 'BLOCKED':
                        self.get_logger().warn('Path blocked, holding and retrying')
                    e['state'] = 'BLOCKED'
                    self._set_goal(q)
            if e['state'] != 'MOVING':
                return

        wp = e['path']
        window = wp[e['idx']:e['idx'] + 50]
        dist = np.max(np.abs(window - q), axis=1)
        e['idx'] += int(np.argmin(dist))
        if e['idx'] > e['progress_idx']:
            e['progress_idx'], e['progress_t'] = e['idx'], now
        j = e['idx']
        while j + 1 < len(wp) and np.max(np.abs(wp[j + 1] - q)) <= self.exec_lookahead:
            j += 1
        if not np.array_equal(wp[j], self.goal):
            self._set_goal(wp[j])
        last = j == len(wp) - 1

        if now - e['last_check'] >= self.check_period:
            e['last_check'] = now
            e['pos_err'] = float(np.linalg.norm(
                np.asarray(self._tool_pose(q)[:3]) - np.asarray(e['pose'][:3])))
            reason = None
            if not self._path_valid(wp[e['idx']:]):
                reason = 'obstacle on path'
            elif float(np.min(dist)) > self.replan_deviation:
                reason = 'pushed off path'
            elif not last and now - e['progress_t'] > self.stall_timeout:
                reason = 'no progress'
            if reason:
                self.get_logger().info(f'Replanning: {reason}')
                e['state'], e['last_try'] = 'REPLANNING', -1e9
                return

        if last and np.max(np.abs(q - wp[-1])) < self.arrive_tol and np.max(np.abs(v)) < 0.01:
            self._finish(True, 'reached')

    def _result(self, success, message, req, q=None):
        res = MoveEef.Result()
        res.success, res.message = success, message
        if q is not None and req.pose is not None:
            pose = self._tool_pose(q)
            res.position_error = float(np.linalg.norm(np.asarray(pose[:3]) -
                                                      np.asarray(req.pose[:3])))
            res.orientation_error = _quat_angle(pose[3:], req.pose[3:])
        return res

    def _finish(self, success, message):
        e, self.exec = self.exec, None
        req = e['req']
        res = self._result(success, message, req, self.js)
        res.replans = e['replans']
        req.result = res
        req.done.set()
        if not success and self.active:
            self._set_goal(self.plan[2][-1].copy() if self.plan is not None else self.js)
        self._clear_path()
        self._set_status(f'Goal {message}', 'ok' if success else 'error')
        log = self.get_logger().info if success else self.get_logger().warn
        log(f'Goal {message}: position error {res.position_error * 1000:.1f} mm, '
            f'{res.replans} replans')

    def _on_action(self, gh):
        t = gh.request.target
        frame = self._lookup(t.header.frame_id)
        req = types.SimpleNamespace(pose=None, goal_q=None, path=None, source='action',
                                    cancel=False, done=threading.Event(), result=None)
        if frame is None:
            gh.abort()
            return MoveEef.Result(message=f'no transform to {t.header.frame_id}')
        p, o = t.pose.position, t.pose.orientation
        req.pose = _compose(frame, [p.x, p.y, p.z, o.w, o.x, o.y, o.z])
        self._submit(req)
        fb = MoveEef.Feedback()
        while not req.done.wait(0.1):
            if gh.is_cancel_requested:
                req.cancel = True
            e = self.exec
            if e is not None and e['req'] is req:
                fb.state, fb.replans = e['state'], e['replans']
                fb.position_error = e['pos_err']
                gh.publish_feedback(fb)
        if req.cancel and gh.is_cancel_requested:
            gh.canceled()
        elif req.result.success:
            gh.succeed()
        else:
            gh.abort()
        return req.result

    # ------------------------------------------------------------------ control loop

    def _restart_mpc(self, q):
        state = self._joint_state(q)
        self.mpc.reset_robot(state)
        self.mpc_state = state
        self.path = [np.asarray(q, float).copy()]

    def _mpc_step(self, q):
        while len(self.path) > 1 and np.max(np.abs(self.path[0] - q)) < self.lookahead_rad:
            self.path.pop(0)
        if np.max(np.abs(self.path[0] - q)) > self.resync_rad + self.lookahead_rad:
            self._restart_mpc(q)
        r = self.mpc.optimize_action_sequence(self.mpc_state)
        seq = r.action_sequence
        if seq is None or seq.position.shape[1] == 0:
            return self.path[0]
        positions = seq.position[0].cpu().numpy().astype(float)
        if not np.all(np.isfinite(positions)):
            self.get_logger().warn('MPC returned a non-finite state, restarting',
                                   throttle_duration_sec=1.0)
            self._restart_mpc(q)
            return q
        feasible = True if r.success is None else bool(r.success.reshape(-1)[0].item())
        lead = np.max(np.abs(self.mpc_state.position.reshape(-1).cpu().numpy() - q))
        diverged = not feasible and np.max(np.abs(positions[-1] - q)) > self.diverge_rad
        if lead < self.max_lead_rad and not diverged:
            nxt = self.cu['JointState'].from_position(seq.position[:, -1, :].clone(),
                                                      joint_names=self.joint_names)
            nxt.velocity = seq.velocity[:, -1, :].clone()
            nxt.acceleration = seq.acceleration[:, -1, :].clone()
            self.mpc_state = nxt
            self.path.extend(positions)
        return self.path[0]

    def _plan_state(self, t):
        start, ts, q, v, a = self.plan
        i = int(np.clip(round((t - start) / self.step_s), 0, len(ts) - 1))
        return q[i].copy(), v[i].copy(), a[i].copy()

    def _command_state(self, now):
        if self.plan is None or self.stamp + self.dt - now < self.late_margin_s:
            stamp = now + self.lead_s
            if self.plan is not None:
                self.late += 1
                return stamp, self._plan_state(stamp)
            return stamp, (self.js.copy(), np.zeros_like(self.js), np.zeros_like(self.js))
        stamp = self.stamp + self.dt
        return stamp, self._plan_state(stamp)

    def _cycle(self):
        cycle_start = time.perf_counter()
        if self.mpc is None:
            if self.grid is not None and self.js is not None:
                self._create_solvers()
            return
        with self.req_lock:
            req, self.request = self.request, None
        if req is not None:
            self._begin(req)
        now = self._now()
        stamp, s0 = self._command_state(now)
        if self.exec is not None:
            self._exec_update(s0)
        elif self.active and not self.live_follow and \
                np.max(np.abs(s0[0] - self.goal)) < self.arrive_tol and \
                np.max(np.abs(s0[1])) < 0.01:
            self.active = False
        if not self.active:
            self.plan = None
            return
        self.stamp = stamp
        target = self._mpc_step(s0[0])
        n = int(round(self.cycles * self.dt / self.step_s))
        q, v, a = _track_target(*s0, target, n, self.step_s, self.vmax, self.amax, self.jerk,
                                self.dt, stop_decel=self.stop_decel, qlim=self.qlim)
        tq, tv, ta = _stopping_tail(q[-1], v[-1], a[-1], self.step_s, self.vmax, self.amax,
                                    self.jerk, qlim=self.qlim)
        q = np.vstack([s0[0][None], q, tq])
        v = np.vstack([s0[1][None], v, tv])
        a = np.vstack([s0[2][None], a, ta])
        ts = np.arange(len(q)) * self.step_s
        self.plan = (stamp, ts, q, v, a)
        self._publish(stamp, ts, q, v, a)
        self._timing(time.perf_counter() - cycle_start)

    def _timing(self, cycle_s):
        self.cycle_times.append(cycle_s)
        if len(self.cycle_times) >= int(10.0 * self.rate_hz):
            ms = np.asarray(self.cycle_times) * 1000.0
            self.get_logger().info(
                f'Cycle {ms.mean():.1f} ms mean, {ms.max():.1f} ms max '
                f'(period {self.dt * 1000:.0f} ms), {self.late} late restarts')
            self.cycle_times, self.late = [], 0

    def _publish(self, stamp, ts, q, v, a):
        msg = JointTrajectory()
        msg.header.stamp = _stamp(stamp)
        msg.header.frame_id = self.base_frame
        msg.joint_names = list(self.joint_names)
        for i in range(0, len(ts), 2):
            pt = JointTrajectoryPoint()
            pt.positions = q[i].tolist()
            pt.velocities = v[i].tolist()
            pt.accelerations = a[i].tolist()
            pt.time_from_start = _duration(ts[i])
            msg.points.append(pt)
        self.pub.publish(msg)

    # ------------------------------------------------------------------ RViz UI

    def _create_ui(self):
        self.im_server = InteractiveMarkerServer(self, self.im_namespace)
        self.menu = MenuHandler()
        self.menu.insert('Plan', callback=lambda fb: self._plan_preview())
        self.menu.insert('Execute', callback=lambda fb: self._execute_preview())
        self.menu.insert('Stop', callback=lambda fb: self._stop())
        self.menu.insert('Reset goal to current', callback=lambda fb: self._reset_goal())
        self.follow_entry = self.menu.insert('Live follow',
                                             callback=lambda fb: self._toggle_follow())
        self.menu.setCheckState(self.follow_entry, MenuHandler.UNCHECKED)
        pose = self._pose_msg(self._tool_pose(self.js))
        im = InteractiveMarker()
        im.header.frame_id = self.base_frame
        im.name = 'goal'
        im.description = f'{self.tool_frame} goal'
        im.scale = self.marker_scale
        im.pose = pose
        body = InteractiveMarkerControl()
        body.always_visible = True
        body.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        sphere = Marker(type=Marker.SPHERE)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.25 * self.marker_scale
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.6, 0.0, 0.8
        body.markers.append(sphere)
        im.controls.append(body)
        for axis, (w, x, y, z) in (('x', (1.0, 1.0, 0.0, 0.0)), ('y', (1.0, 0.0, 0.0, 1.0)),
                                   ('z', (1.0, 0.0, 1.0, 0.0))):
            for mode, prefix in ((InteractiveMarkerControl.MOVE_AXIS, 'move_'),
                                 (InteractiveMarkerControl.ROTATE_AXIS, 'rotate_')):
                c = InteractiveMarkerControl()
                c.name = prefix + axis
                c.orientation.w, c.orientation.x = w, x
                c.orientation.y, c.orientation.z = y, z
                c.interaction_mode = mode
                im.controls.append(c)
        self.im_server.insert(im, feedback_callback=self._on_marker)
        self.menu.apply(self.im_server, 'goal')
        self.im_server.insert(self._buttons_marker(pose), feedback_callback=self._on_button)
        self.im_server.applyChanges()
        self.goal_pose = self._pose_list(pose)
        self.goal_q, self.goal_ok = self.js.copy(), True
        self._set_status('Drag the goal, then PLAN', 'info')

    def _buttons_marker(self, goal_pose):
        im = InteractiveMarker()
        im.header.frame_id = self.base_frame
        im.name = 'buttons'
        im.scale = self.marker_scale
        im.pose = self._buttons_pose(goal_pose)
        for i, (name, rgb) in enumerate((('plan', (0.2, 0.5, 1.0)), ('execute', (0.1, 0.8, 0.2)))):
            c = InteractiveMarkerControl()
            c.name = name
            c.interaction_mode = InteractiveMarkerControl.BUTTON
            c.always_visible = True
            box = Marker(type=Marker.CUBE)
            box.pose.position.y = (i - 0.5) * 0.11
            box.scale.x, box.scale.y, box.scale.z = 0.01, 0.10, 0.035
            box.color.r, box.color.g, box.color.b, box.color.a = *rgb, 0.9
            text = Marker(type=Marker.TEXT_VIEW_FACING)
            text.pose.position.y = box.pose.position.y
            text.pose.position.z = 0.003
            text.scale.z = 0.022
            text.text = name.upper()
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            c.markers.extend([box, text])
            im.controls.append(c)
        return im

    @staticmethod
    def _buttons_pose(goal_pose):
        m = PoseMsg()
        m.position.x, m.position.y = goal_pose.position.x, goal_pose.position.y
        m.position.z = goal_pose.position.z + 0.2
        m.orientation.w = 1.0
        return m

    @staticmethod
    def _pose_msg(p):
        m = PoseMsg()
        m.position.x, m.position.y, m.position.z = p[:3]
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = p[3:]
        return m

    @staticmethod
    def _pose_list(m):
        return [m.position.x, m.position.y, m.position.z,
                m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z]

    def _on_marker(self, fb):
        if fb.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.goal_pose = self._pose_list(fb.pose)
            self.goal_pose_dirty = True
            self.goal_set = True
            self.im_server.setPose('buttons', self._buttons_pose(fb.pose))
            self.im_server.applyChanges()

    def _on_button(self, fb):
        if fb.event_type != InteractiveMarkerFeedback.BUTTON_CLICK:
            return
        if fb.control_name == 'plan':
            self._plan_preview()
        elif fb.control_name == 'execute':
            self._execute_preview()

    def _ui_cycle(self):
        if self.ik is None:
            return
        if self.exec is not None:
            e = self.exec
            self.goal_pose = e['pose']
            if e['goal_q'] is not None:
                self.goal_q = e['goal_q']
            text = {'PLANNING': 'Planning...', 'MOVING': 'Moving',
                    'REPLANNING': 'Replanning...', 'BLOCKED': 'Blocked - retrying'}[e['state']]
            if e['replans']:
                text += f' ({e["replans"]} replans)'
            self._set_status(text, 'error' if e['state'] == 'BLOCKED' else 'busy')
            pose = self._pose_msg(e['pose'])
            self.im_server.setPose('goal', pose)
            self.im_server.setPose('buttons', self._buttons_pose(pose))
            self.im_server.applyChanges()
            self.goal_set = True
        elif not self.goal_set and not self.active:
            self.goal_q, self.goal_ok = self.js.copy(), True
            pose = self._pose_msg(self._tool_pose(self.js))
            self.goal_pose = self._pose_list(pose)
            self.im_server.setPose('goal', pose)
            self.im_server.setPose('buttons', self._buttons_pose(pose))
            self.im_server.applyChanges()
        elif self.goal_pose_dirty:
            self.goal_pose_dirty = False
            ok, q = self._ik_solve(self.goal_pose, self.goal_q if self.goal_q is not None
                                   else self.js)
            if ok:
                self.goal_q = q
            self.goal_ok = ok
            if self.live_follow:
                if ok:
                    self._set_goal(q)
                self._set_status('Live follow' if ok else 'No IK solution',
                                 'ok' if ok else 'error')
            else:
                self._clear_path()
                self._set_status('Goal OK - click PLAN' if ok else
                                 'No IK solution (unreachable or in collision)',
                                 'info' if ok else 'error')
        self._publish_goal_preview()

    def _set_status(self, text, level):
        self.status = (text, level)

    def _publish_goal_preview(self):
        if self.goal_q is not None and self.js_full is not None:
            names, pos = self.js_full
            pos = list(pos)
            for i, n in enumerate(self.joint_names):
                pos[names.index(n)] = float(self.goal_q[i])
            msg = JointStateMsg()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name, msg.position = names, pos
            self.goal_js_pub.publish(msg)
        text, level = self.status
        m = Marker()
        m.header.frame_id = self.base_frame
        m.ns, m.id, m.type, m.action = 'goal_status', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        x, y, z = self.goal_pose[:3]
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z + 0.27
        m.pose.orientation.w = 1.0
        m.scale.z = 0.03
        m.text = text
        m.color.r, m.color.g, m.color.b = {'ok': (0.2, 1.0, 0.2), 'error': (1.0, 0.2, 0.2),
                                           'busy': (1.0, 0.8, 0.1)}.get(level, (1.0, 1.0, 1.0))
        m.color.a = 1.0
        self.status_pub.publish(m)

    def _draw_path(self, path):
        q = self.torch.as_tensor(np.asarray(path, np.float32), device=self.device)
        pts = self.kin.compute_kinematics(self.cu['JointState'].from_position(
            q, joint_names=self.joint_names)).tool_poses.to_dict()[self.tool_frame]
        m = Marker()
        m.header.frame_id = self.base_frame
        m.ns, m.id, m.type, m.action = 'planned_path', 0, Marker.LINE_STRIP, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.006
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.3, 1.0
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                    for p in pts.position.view(-1, 3).cpu().numpy()]
        self.path_pub.publish(m)

    def _clear_path(self):
        self.path_plan = None
        m = Marker()
        m.header.frame_id = self.base_frame
        m.ns, m.id, m.action = 'planned_path', 0, Marker.DELETE
        self.path_pub.publish(m)

    def _plan_preview(self):
        if self.active:
            self._set_status('Robot is moving - Stop first', 'error')
            return
        if not self.goal_ok:
            self._set_status('No IK solution - move the goal', 'error')
            return
        path = self._plan_path(self.js, self.goal_q)
        if path is None:
            self._clear_path()
            self._set_status('Plan failed - move the goal', 'error')
            return
        self.path_plan, self.path_plan_start = path, self.js.copy()
        self._draw_path(path)
        self._set_status(f'Plan OK ({(len(path) - 1) * self.plan_dt:.1f} s) - click EXECUTE',
                         'ok')

    def _execute_preview(self):
        if self.path_plan is None:
            self._set_status('No plan - click PLAN first', 'error')
            return
        if self.active:
            self._set_status('Robot is moving - Stop first', 'error')
            return
        if np.max(np.abs(self.js - self.path_plan_start)) > 0.05:
            self._clear_path()
            self._set_status('Robot moved since planning - PLAN again', 'error')
            return
        self._submit(types.SimpleNamespace(
            pose=list(self.goal_pose), goal_q=self.goal_q.copy(), path=self.path_plan,
            source='rviz', cancel=False, done=threading.Event(), result=None))

    def _stop(self):
        if self.live_follow:
            self._toggle_follow()
        if self.exec is not None:
            self.exec['req'].cancel = True
        elif self.active:
            self._set_goal(self.plan[2][-1].copy() if self.plan is not None else self.js)
        self._set_status('Stopped', 'info')

    def _reset_goal(self):
        self.goal_set = False
        self._clear_path()
        self._set_status('Drag the goal, then PLAN', 'info')

    def _toggle_follow(self):
        self.live_follow = not self.live_follow
        self.menu.setCheckState(self.follow_entry, MenuHandler.CHECKED if self.live_follow
                                else MenuHandler.UNCHECKED)
        self.menu.reApply(self.im_server)
        self.im_server.applyChanges()
        if self.live_follow:
            if self.exec is not None:
                self.exec['req'].cancel = True
            self._clear_path()
            self._activate()
            if self.goal_ok:
                self._set_goal(self.goal_q)
            self._set_status('Live follow', 'ok')


def main(args=None):
    rclpy.init(args=args)
    node = Curobo()
    action_executor = SingleThreadedExecutor()
    action_executor.add_node(node.action_node)
    threading.Thread(target=action_executor.spin, daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        action_executor.shutdown()
        node.action_node.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
