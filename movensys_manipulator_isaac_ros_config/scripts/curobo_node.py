#!/usr/bin/env python3
"""cuRoboV2 end-effector controller.

MoveEef action (target pose) -> IK + global plan -> MPC follows the path at rate_hz, avoiding
camera obstacles (depth -> robot mask -> ESDF) -> jerk-limited JointTrajectory ->
dynamic_trajectory_controller. The path is checked against the live ESDF and replanned when
blocked, when the MPC is pushed off it, or when progress stalls. IK + global planning run on a
worker thread so the control loop keeps its period. RViz: drag the goal marker, PLAN, EXECUTE.
"""
import collections
from concurrent.futures import ThreadPoolExecutor
import math
import os
import threading
import time
import types
import xml.etree.ElementTree as ET

from builtin_interfaces.msg import Duration, Time
from geometry_msgs.msg import Point, Pose as PoseMsg
from interactive_markers import InteractiveMarkerServer
from movensys_manipulator_isaac_ros_config.action import MoveEef
from movensys_manipulator_isaac_ros_config.msg import PlannerStatus
from movensys_manipulator_isaac_ros_config.srv import PlanningCommand
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState as JointStateMsg
import tf2_ros
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import (InteractiveMarker, InteractiveMarkerControl,
                                    InteractiveMarkerFeedback, Marker)

WALL_DECEL = 0.8
CURRENT, MARKER = '<current>', '<marker>'
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
        start_js_topic = p('start_joint_state_topic', '/curobo/start_joint_states').value
        status_topic = p('planner_status_topic', '/curobo/planner_status').value
        command_service = p('command_service', '/curobo/command').value
        srdf_path = p('srdf_path', '').value
        self.planning_group = p('planning_group', '').value
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
        self.memory_margin = float(p('memory_margin', 0.03).value)
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
        self.settle_tol = float(p('settle_position_tolerance', 0.005).value)
        # Idle obstacle avoidance (<= 0 disables): start the MPC below guard, stop above release.
        self.guard_dist = float(p('idle_guard_distance', self.activation - 0.01).value)
        self.guard_release = max(float(p('idle_guard_release', self.activation).value),
                                 self.guard_dist)
        self.guarding = False
        if self.segment_distance >= self.activation:
            self.get_logger().warn(
                f'segment_distance {self.segment_distance} >= collision_activation_distance '
                f'{self.activation}: obstacles closer than that to the arm are masked as robot, '
                f'so the MPC (and idle guard) cannot react to them')
        if self.plan_activation < self.activation:
            self.get_logger().warn(
                f'plan_collision_activation_distance {self.plan_activation} < MPC '
                f'collision_activation_distance {self.activation}: IK/planner may pick goals the '
                f'MPC will not reach')
        self.ui_rate_hz = float(p('ui_rate_hz', 10.0).value)
        self.marker_scale = float(p('goal_marker_scale', 0.2).value)
        self.dt = 1.0 / self.rate_hz

        self._load_curobo()
        self.robot = self._robot_config()

        self.js = None
        self.js_full = None
        self.js_hist = collections.deque(maxlen=200)
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
        self.status = ('', 'info')
        self.start_name = CURRENT
        self.goal_name = CURRENT
        self.preview = None
        self.ui_cmds = collections.deque()
        self.named = self._load_named_states(srdf_path)
        self.last_esdf = 0.0
        self.last_marker = 0.0
        self.cycle_times = []
        self.late = 0
        # IK + planner are shared by the replan worker and the UI; MPC stays on the control loop.
        self.solver_lock = threading.Lock()
        self.replan_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='curobo_replan')
        self.replan_job = None

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
        self.start_js_pub = self.create_publisher(JointStateMsg, start_js_topic, 1)
        self.status_pub = self.create_publisher(
            PlannerStatus, status_topic,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(PlanningCommand, command_service, self._on_command)
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
        # cuRobo records CUDA graphs lazily (GraphExecutor), which can happen on the replan worker.
        # In torch's default 'global' capture mode a blocking CUDA call from any other thread (map,
        # MPC) aborts that capture; 'thread_local' only checks the capturing thread.
        if not getattr(torch.cuda.graph, '_thread_local_capture', False):
            base = torch.cuda.graph

            class _ThreadLocalCapture(base):
                _thread_local_capture = True

                def __init__(self, cuda_graph, *args, **kwargs):
                    kwargs.setdefault('capture_error_mode', 'thread_local')
                    super().__init__(cuda_graph, *args, **kwargs)

            torch.cuda.graph = _ThreadLocalCapture
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
        self.js_hist.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, self.js))

    def _js_at(self, stamp):
        t = stamp.sec + stamp.nanosec * 1e-9
        if t <= 0.0 or not self.js_hist:
            return self.js
        return min(self.js_hist, key=lambda h: abs(h[0] - t))[1]

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
            obs(depth_image=depth.clone(), pose=pose),
            self._joint_state(self._js_at(self.depth_msg.header.stamp)))
        seen = world > 0
        self.memory = self.torch.where(seen, world, self.memory)
        # Memory fills robot-masked pixels with the last surface seen there, which is only valid
        # while the robot is in front of it. If the ray now reaches the robot (e.g. the base)
        # beyond the remembered depth, that surface has moved away: forget it, or it is
        # re-integrated every frame and the vacated space never clears.
        stale = ~seen & (depth > self.memory + self.memory_margin)
        self.memory = self.torch.where(stale, self.torch.zeros_like(self.memory), self.memory)
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
        return bool(self._path_clear(path).all())

    def _path_clear(self, path):
        return self._clearance(path) >= self.path_clearance

    def _clearance(self, path):
        """Min distance (m) from the robot spheres to the camera ESDF for each config in path."""
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
        nx, ny, nz = self.grid_shape
        esdf = self.grid.feature_tensor.view(-1)[:nx * ny * nz].view(nx, ny, nz)
        dist = t.full(inside.shape, 1e3, device=self.device)
        i = idx[inside]
        dist[inside] = esdf[i[:, 0], i[:, 1], i[:, 2]].float() - sph[..., 3][inside]
        return dist.min(-1).values.cpu().numpy()

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
        self.guarding = False
        now = self._now()
        self.exec = {
            'req': req, 'pose': req.pose, 'goal_q': req.goal_q, 'path': req.path, 'idx': 0,
            't0': now, 'last_try': -1e9, 'last_check': now, 'replans': 0, 'planned': False,
            'progress_idx': 0, 'progress_t': now, 'state': 'PLANNING', 'pos_err': float('nan')}
        if req.path is not None:
            self.exec.update(planned=True, state='MOVING', clear0=self._path_clear(req.path))
            self._draw_path(req.path)
            self._activate()
        self.get_logger().info(
            f'New goal {np.round(req.pose[:3], 3).tolist()} ({req.source})')

    def _replan_idle(self):
        return self.replan_job is None or self.replan_job.done()

    def _replan_compute(self, q, pose, goal_q, wp, i):
        """Worker thread: IK + global plan. Uses only the IK and planner solvers."""
        with self.solver_lock:
            ok, gq = self._ik_solve(pose, goal_q if goal_q is not None else q)
            if not ok:
                ok, gq = self._ik_solve(pose, q)
            if not ok:
                return {'q': q, 'gq': None, 'path': None, 'fail': None}
            path = self._plan_path(q, gq)
            if path is None and wp is not None:
                for k in (15, 40):
                    j = max(i - k, 0)
                    tail = self._plan_path(wp[j], gq)
                    if tail is not None:
                        path = np.vstack([wp[j:i + 1][::-1], tail])
                        break
                    if j == 0:
                        break
            return {'q': q, 'gq': gq, 'path': path, 'fail': getattr(self, 'plan_fail', None)}

    def _apply_replan(self, job):
        """Control loop: adopt a finished worker result for the current goal."""
        e = self.exec
        try:
            res = job.result()
        except Exception as ex:  # keep the control loop alive if a solve throws
            self.get_logger().error(f'Replan error: {ex!r}', throttle_duration_sec=2.0)
            return False
        q, gq, path = res['q'], res['gq'], res['path']
        if gq is None:
            self.get_logger().warn('Replan: no IK solution for target', throttle_duration_sec=2.0)
            return False
        if path is None:
            self.get_logger().warn(f'Replan failed: {res["fail"]}, start clear: '
                                   f'{self._path_valid(q[None])}', throttle_duration_sec=2.0)
            return False
        if e['planned']:
            e['replans'] += 1
            self.get_logger().info(f'Replanned ({e["replans"]})')
        e.update(goal_q=gq, path=path, idx=0, planned=True, state='MOVING',
                 progress_idx=0, progress_t=self._now(), clear0=self._path_clear(path))
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
            job = e.get('job')
            if job is not None and job.done():
                e['job'] = None
                if self._apply_replan(job):
                    self._activate()
                else:
                    e['last_try'] = now
                    if e['planned']:
                        if e['state'] != 'BLOCKED':
                            self.get_logger().warn('Path blocked, holding and retrying')
                        e['state'] = 'BLOCKED'
                        self._set_goal(q)
            elif job is None and now - e['last_try'] >= self.replan_period and \
                    self._replan_idle():
                # Plan from where the arm is being held (the MPC goal) so the new path starts
                # where the arm will actually be when the result arrives.
                start = self.goal.copy() if self.active else np.asarray(q, float).copy()
                e['last_try'] = now
                e['job'] = self.replan_job = self.replan_pool.submit(
                    self._replan_compute, start, list(e['pose']), e['goal_q'], e['path'],
                    e['idx'])
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
            i = e['idx']
            if ((~self._path_clear(wp[i:])) & e['clear0'][i:]).any():
                reason = 'obstacle on path'
            elif float(np.min(dist)) > self.replan_deviation:
                reason = 'pushed off path'
            elif not last and now - e['progress_t'] > self.stall_timeout:
                reason = 'no progress'
            if reason:
                self.get_logger().info(f'Replanning: {reason}')
                e['state'], e['last_try'] = 'REPLANNING', -1e9
                # Hold at the current stopping point while the worker plans.
                self._set_goal(self.plan[2][-1].copy() if self.plan is not None else q)
                return

        if not (last and np.max(np.abs(v)) < 0.01):
            e['settle_t'] = None
        elif np.max(np.abs(q - wp[-1])) < self.arrive_tol:
            self._finish(True, 'reached')
        elif e.get('settle_t') is None:
            e['settle_t'] = now
        elif now - e['settle_t'] > self.stall_timeout:
            # Stopped short of the final waypoint (typically the MPC collision cost holding the
            # arm off a nearby voxel): finish instead of waiting for goal_timeout_s.
            err = float(np.linalg.norm(np.asarray(self._tool_pose(q)[:3]) -
                                       np.asarray(e['pose'][:3])))
            if err <= self.settle_tol:
                self._finish(True, f'reached (settled {err * 1000:.1f} mm off)')
            else:
                self._finish(False, f'stalled {err * 1000:.1f} mm from goal')

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

    def _idle_update(self, s0):
        """No goal: hold the pose. Run the MPC (goal = hold pose) whenever an obstacle comes
        within idle_guard_distance of the arm, so its collision cost pushes the arm away; it
        returns to the hold pose and goes idle once clearance is back above idle_guard_release."""
        q, v, _ = s0
        at_hold = np.max(np.abs(q - self.goal)) < self.arrive_tol and np.max(np.abs(v)) < 0.01
        if self.guard_dist <= 0.0:
            if self.active and at_hold:
                self.active = False
            return
        clear = float(self._clearance(q[None])[0])
        if not self.active:
            if clear < self.guard_dist:
                hold = self.goal if np.max(np.abs(self.goal - q)) < 0.05 else q
                self._activate()
                self._set_goal(np.array(hold, float))
                self.guarding = True
                self.get_logger().warn(f'Obstacle {clear * 1000:.0f} mm from idle arm, avoiding',
                                       throttle_duration_sec=1.0)
        elif at_hold and clear >= self.guard_release:
            self.active = False
            if self.guarding:
                self.guarding = False
                self.get_logger().info(f'Obstacle cleared ({clear * 1000:.0f} mm), holding')

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
        elif not self.live_follow:
            self._idle_update(s0)
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

    # ------------------------------------------------------------------ RViz UI
    # The cuRobo RViz panel (CuroboPanel) sends PlanningCommand requests and shows
    # PlannerStatus; the 6-DOF interactive marker sets the <marker> goal.

    def _load_named_states(self, path):
        """SRDF group_states that set every planning joint -> {name: joint positions}."""
        named = {}
        if not path:
            return named
        try:
            root = ET.parse(path).getroot()
        except (OSError, ET.ParseError) as e:
            self.get_logger().warn(f'Cannot read SRDF {path}: {e}')
            return named
        for gs in root.iter('group_state'):
            if self.planning_group and gs.get('group') != self.planning_group:
                continue
            vals = {j.get('name'): float(j.get('value')) for j in gs.iter('joint')}
            if all(n in vals for n in self.joint_names):
                named[gs.get('name')] = np.array([vals[n] for n in self.joint_names])
        self.get_logger().info(f'Named states from SRDF: {list(named)}')
        return named

    def _create_ui(self):
        self.im_server = InteractiveMarkerServer(self, self.im_namespace)
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
        self.im_server.applyChanges()
        self.goal_pose = self._pose_list(pose)
        self.goal_q, self.goal_ok = self.js.copy(), True
        self._set_status('Drag the goal marker or pick a goal state, then Plan', 'info')

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

    def _set_marker(self, pose_list):
        self.im_server.setPose('goal', self._pose_msg(pose_list))
        self.im_server.applyChanges()

    def _on_marker(self, fb):
        if fb.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.goal_pose = self._pose_list(fb.pose)
            self.goal_pose_dirty = True
            self.goal_set = True
            self.goal_name = MARKER

    def _on_command(self, req, res):
        R = PlanningCommand.Request
        names = {
            R.SET_START: [CURRENT] + list(self.named),
            R.SET_GOAL: [MARKER, CURRENT] + list(self.named),
            R.LIVE_FOLLOW: ['on', 'off'],
        }
        if req.command not in (R.PLAN, R.EXECUTE, R.PLAN_EXECUTE, R.STOP) and \
                req.command not in names:
            res.success, res.message = False, f'Unknown command "{req.command}"'
        elif req.command in names and req.name not in names[req.command]:
            res.success, res.message = False, f'Unknown {req.command} name "{req.name}"'
        else:
            self.ui_cmds.append((req.command, req.name))
            res.success, res.message = True, 'queued'
        return res

    def _ui_command(self, cmd, name):
        R = PlanningCommand.Request
        if cmd == R.SET_START:
            self.start_name = name
            self._clear_path()
            self._set_status(f'Start state {name}', 'info')
        elif cmd == R.SET_GOAL:
            self._clear_path()
            self.goal_name = name
            if name == CURRENT:
                self.goal_set = False
                self._set_status('Goal state <current>', 'info')
            elif name == MARKER:
                self.goal_set, self.goal_pose_dirty = True, True
            else:
                q = self.named[name].copy()
                self.goal_q, self.goal_ok, self.goal_set = q, True, True
                self.goal_pose_dirty = False
                self.goal_pose = self._tool_pose(q)
                self._set_marker(self.goal_pose)
                if self.live_follow:
                    self._set_goal(q)
                self._set_status(f'Goal state {name} - click Plan', 'info')
        elif cmd == R.PLAN:
            self._start_preview(execute=False)
        elif cmd == R.PLAN_EXECUTE:
            self._start_preview(execute=True)
        elif cmd == R.EXECUTE:
            self._execute_preview()
        elif cmd == R.STOP:
            self._stop()
        elif cmd == R.LIVE_FOLLOW and (name == 'on') != self.live_follow:
            self._toggle_follow()

    def _start_q(self):
        return self.js.copy() if self.start_name == CURRENT else self.named[self.start_name].copy()

    def _ui_cycle(self):
        if self.ik is None:
            return
        while self.ui_cmds:
            self._ui_command(*self.ui_cmds.popleft())
        self._poll_preview()
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
            self._set_marker(e['pose'])
            self.goal_set = True
        elif not self.goal_set and not self.active:
            self.goal_q, self.goal_ok = self.js.copy(), True
            self.goal_pose = self._tool_pose(self.js)
            self._set_marker(self.goal_pose)
        elif self.goal_pose_dirty and self.solver_lock.acquire(blocking=False):
            try:
                self.goal_pose_dirty = False
                ok, q = self._ik_solve(self.goal_pose, self.goal_q if self.goal_q is not None
                                       else self.js)
            finally:
                self.solver_lock.release()
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
                self._set_status('Goal OK - click Plan' if ok else
                                 'No IK solution (unreachable or in collision)',
                                 'info' if ok else 'error')
        self._publish_state(self.goal_js_pub, self.goal_q)
        self._publish_state(self.start_js_pub, self._start_q())
        self._publish_status()

    def _set_status(self, text, level):
        self.status = (text, level)

    def _publish_state(self, pub, q):
        if q is None or self.js_full is None:
            return
        names, pos = self.js_full
        pos = list(pos)
        for i, n in enumerate(self.joint_names):
            pos[names.index(n)] = float(q[i])
        msg = JointStateMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position = names, pos
        pub.publish(msg)

    def _can_execute(self):
        pv = self.preview
        return (pv is not None and pv['path'] is not None and self.exec is None and
                not self.active and np.max(np.abs(self.js - pv['start'])) <= 0.05)

    def _publish_status(self):
        text, level = self.status
        pv = self.preview
        m = PlannerStatus()
        m.text = text
        m.level = {'ok': PlannerStatus.LEVEL_OK, 'busy': PlannerStatus.LEVEL_BUSY,
                   'error': PlannerStatus.LEVEL_ERROR}.get(level, PlannerStatus.LEVEL_INFO)
        m.start_states = [CURRENT] + list(self.named)
        m.goal_states = [MARKER, CURRENT] + list(self.named)
        m.start_state, m.goal_state = self.start_name, self.goal_name
        m.goal_ok = bool(self.goal_ok)
        m.planning = pv is not None and pv['path'] is None
        m.has_plan = pv is not None and pv['path'] is not None
        m.can_execute = bool(self._can_execute())
        m.moving = self.exec is not None or (self.active and not self.live_follow)
        m.live_follow = self.live_follow
        m.plan_duration = (len(pv['path']) - 1) * self.plan_dt if m.has_plan else 0.0
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
        self.preview = None
        m = Marker()
        m.header.frame_id = self.base_frame
        m.ns, m.id, m.action = 'planned_path', 0, Marker.DELETE
        self.path_pub.publish(m)

    def _preview_compute(self, start, goal):
        """Worker thread: global plan for the panel's Plan button."""
        with self.solver_lock:
            path = self._plan_path(start, goal)
            return path, getattr(self, 'plan_fail', None)

    def _start_preview(self, execute):
        if self.exec is not None or self.active:
            return self._set_status('Robot is moving - Stop first', 'error')
        if not self.goal_ok:
            return self._set_status('No IK solution for the goal - move it', 'error')
        if execute and self.start_name != CURRENT:
            return self._set_status('Plan & Execute needs start state <current>', 'error')
        if not self._replan_idle():
            return self._set_status('Planner busy - try again', 'error')
        start, goal = self._start_q(), np.asarray(self.goal_q, float).copy()
        self._clear_path()
        job = self.replan_job = self.replan_pool.submit(self._preview_compute, start, goal)
        self.preview = {'job': job, 'start': start, 'goal': goal, 'pose': list(self.goal_pose),
                        'execute': execute, 'path': None}
        self._set_status('Planning...', 'busy')

    def _poll_preview(self):
        pv = self.preview
        if pv is None or pv['path'] is not None or not pv['job'].done():
            return
        try:
            path, fail = pv['job'].result()
        except Exception as ex:  # keep the UI alive if a solve throws
            path, fail = None, repr(ex)
        if path is None:
            self.preview = None
            return self._set_status(f'Plan failed: {fail}', 'error')
        pv['path'], pv['job'] = path, None
        self._draw_path(path)
        if pv['execute']:
            return self._execute_preview()
        dur = (len(path) - 1) * self.plan_dt
        if self._can_execute():
            self._set_status(f'Plan OK ({dur:.1f} s) - click Execute', 'ok')
        else:
            self._set_status(f'Plan OK ({dur:.1f} s) - preview from {self.start_name}', 'ok')

    def _execute_preview(self):
        pv = self.preview
        if pv is None or pv['path'] is None:
            return self._set_status('No plan - click Plan first', 'error')
        if self.exec is not None or self.active:
            return self._set_status('Robot is moving - Stop first', 'error')
        if not self._can_execute():
            return self._set_status('Plan does not start at the current state - '
                                    'Plan again from <current>', 'error')
        self._submit(types.SimpleNamespace(
            pose=pv['pose'], goal_q=pv['goal'].copy(), path=pv['path'],
            source='rviz', cancel=False, done=threading.Event(), result=None))
        self.preview = None
        self._set_status('Executing', 'busy')

    def _stop(self):
        if self.live_follow:
            self._toggle_follow()
        if self.preview is not None and self.preview['path'] is None:
            self.preview = None
        if self.exec is not None:
            self.exec['req'].cancel = True
        elif self.active:
            self._set_goal(self.plan[2][-1].copy() if self.plan is not None else self.js)
        self._set_status('Stopped', 'info')

    def _toggle_follow(self):
        self.live_follow = not self.live_follow
        if self.live_follow:
            if self.exec is not None:
                self.exec['req'].cancel = True
            self._clear_path()
            self._activate()
            if self.goal_ok:
                self._set_goal(self.goal_q)
            self._set_status('Live follow', 'ok')
        else:
            self._set_status('Live follow off', 'info')


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
        node.replan_pool.shutdown(wait=False, cancel_futures=True)
        action_executor.shutdown()
        node.action_node.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
