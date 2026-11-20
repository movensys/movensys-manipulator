#!/usr/bin/env python3
"""Obstacle avoidance with cuRoboV2 and the WMX R2 dynamic_trajectory_controller.

One camera -> cuRoboV2 Mapper (robot masked out of the depth image) -> ESDF -> cuRoboV2 MPC
(rate_hz) chasing joint targets -> command bounded by velocity, acceleration and jerk ->
JointTrajectory stamped with the time of its first point -> dynamic_trajectory_controller.

Based on the CR3A demo in feature/cr3a-curobo-v2-realtime-avoidance (cr3a_rt_shaping), without
its global planner, input shaping, GUI and recording.
"""
import os
import time

from builtin_interfaces.msg import Duration, Time
from geometry_msgs.msg import Point
from movensys_manipulator_isaac_ros_config.curobo_motion import stopping_tail, track_target
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState as JointStateMsg
import tf2_ros
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker


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


class Curobo(Node):

    def __init__(self):
        super().__init__('curobo')
        p = self.declare_parameter
        default_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_names = list(p('joint_names', default_joints).value)
        self.urdf_path = p('urdf_path', '').value
        self.xrdf_path = p('xrdf_path', '').value
        flat = list(p('targets', [0.0]).value)
        nj = len(self.joint_names)
        if len(flat) < nj or len(flat) % nj:
            raise ValueError(f'targets must hold a multiple of {nj} joint values')
        self.targets = np.asarray(flat, float).reshape(-1, nj)
        self.output_topic = p('output_topic',
                              '/dynamic_trajectory_controller/joint_trajectory').value
        self.base_frame = p('base_frame', 'world_manipulator').value
        self.rate_hz = float(p('rate_hz', 20.0).value)
        self.interpolation_steps = int(p('interpolation_steps', 4).value)
        self.warm_iters = int(p('warm_iters', 100).value)
        self.cold_iters = int(p('cold_iters', 150).value)
        self.activation = float(p('collision_activation_distance', 0.06).value)
        self.floor_activation = float(p('floor_activation_distance', 0.02).value)
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
        self.jerk = float(p('jerk', 500.0).value)
        self.stop_decel = float(p('stop_decel', 0.8).value)
        self.speed_scale = float(p('speed_scale', 1.0).value)
        self.step_s = float(p('step_s', 0.005).value)
        self.cycles = int(p('cycles', 2).value)
        self.lead_s = float(p('lead_s', 0.05).value)
        self.late_margin_s = float(p('late_margin_s', 0.01).value)
        self.diverge_rad = float(p('diverge_rad', 1.0).value)
        self.lookahead_rad = float(p('path_lookahead_rad', 0.1).value)
        self.max_lead_rad = float(p('path_max_lead_rad', 0.4).value)
        self.resync_rad = float(p('resync_rad', 0.2).value)
        self.arrive_tol = float(p('arrive_tolerance', 0.002).value)
        self.hold_at_goal_s = float(p('hold_at_goal_s', 1.0).value)
        self.leg_timeout_s = float(p('leg_timeout_s', 25.0).value)
        self.marker_period_s = float(p('marker_period_s', 0.5).value)
        self.depth_scale = float(p('depth_scale_16u', 0.001).value)
        self.dt = 1.0 / self.rate_hz

        self._load_curobo()
        self.robot = self._robot_config()

        self.js = None
        self.depth_msg = None
        self.depth_seq = 0
        self.integrated_seq = 0
        self.info = None
        self.cam = None
        self.frames = 0
        self.mpc = None
        self.grid = None
        self.plan = None
        self.goal_index = 0
        self.leg_start = None
        self.arrived_since = None
        self.last_marker = 0.0
        self.cycle_times = []
        self.late = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(JointStateMsg, p('joint_state_topic', '/joint_states').value,
                                 self._on_joint_state, 10)
        self.create_subscription(Image, p('depth_topic', '/image_nvblox/depth').value,
                                 self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo,
                                 p('camera_info_topic', '/image_nvblox/depth/camera_info').value,
                                 self._on_info, qos_profile_sensor_data)
        self.pub = self.create_publisher(JointTrajectory, self.output_topic, 10)
        self.marker_pub = self.create_publisher(Marker, '~/obstacles', 1)
        self.timer = self.create_timer(self.dt, self._cycle)
        self.get_logger().info(
            f'{len(self.targets)} targets, {self.rate_hz:.0f} Hz MPC -> {self.output_topic}')

    def _load_curobo(self):
        import torch
        from curobo._src.robot.loader.util import load_robot_yaml
        from curobo._src.types.robot import RobotCfg
        from curobo.kinematics import Kinematics
        from curobo.model_predictive_control import (ModelPredictiveControl,
                                                     ModelPredictiveControlCfg)
        from curobo.perception import FilterDepth, Mapper, MapperCfg, RobotSegmenter
        from curobo.scene import Cuboid, Scene, VoxelGrid
        from curobo.types import CameraObservation, ContentPath, JointState, Pose
        self.torch = torch
        self.device = torch.device('cuda:0')
        self.cu = {
            'load_robot_yaml': load_robot_yaml, 'MPC': ModelPredictiveControl,
            'MPCCfg': ModelPredictiveControlCfg, 'FilterDepth': FilterDepth, 'Mapper': Mapper,
            'MapperCfg': MapperCfg, 'RobotSegmenter': RobotSegmenter, 'Cuboid': Cuboid,
            'Scene': Scene, 'VoxelGrid': VoxelGrid, 'CameraObservation': CameraObservation,
            'ContentPath': ContentPath, 'JointState': JointState, 'Pose': Pose,
            'RobotCfg': RobotCfg, 'Kinematics': Kinematics,
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

    def _setup_camera(self):
        t = self.torch
        h, w = self.info.height, self.info.width
        k = np.asarray(self.info.k, np.float32).reshape(3, 3)
        self.cam = {'h': h, 'w': w, 'k': t.as_tensor(k, device=self.device).view(1, 3, 3),
                    'frame': self.info.header.frame_id}
        kinematics = self.cu['Kinematics'](self.cu['RobotCfg'].create(self.robot).kinematics)
        self.segmenter = self.cu['RobotSegmenter'](
            kinematics, distance_threshold=self.segment_distance, use_cuda_graph=False,
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

    def _camera_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.cam['frame'],
                                                 rclpy.time.Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'No camera transform yet: {e}',
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
        if self.depth_seq == self.integrated_seq or self.js is None:
            return
        self.integrated_seq = self.depth_seq
        pose_list = self._camera_pose()
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
        return self.grid

    def _create_mpc(self):
        floor = self.cu['Cuboid'](
            name='floor', dims=[3.0, 3.0, 0.02],
            pose=[0.0, 0.0, self.floor_z - 0.01 - (self.activation - self.floor_activation),
                  1.0, 0.0, 0.0, 0.0])
        cfg = self.cu['MPCCfg'].create(
            robot=self.robot, scene_model=self.cu['Scene'](cuboid=[floor], voxel=[self.grid]),
            use_cuda_graph=True, optimization_dt=self.dt,
            interpolation_steps=self.interpolation_steps,
            optimizer_collision_activation_distance=self.activation,
            warm_start_optimization_num_iters=self.warm_iters,
            cold_start_optimization_num_iters=self.cold_iters)
        self.mpc = self.cu['MPC'](cfg)
        names = list(self.mpc.joint_names)
        if names != self.joint_names:
            raise RuntimeError(f'cuRobo joints {names} differ from joint_names')
        lim = self.mpc.kinematics.get_joint_limits()
        k = [list(lim.joint_names).index(n) for n in names]
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
        self._set_goal(self.targets[self.goal_index])
        self.mpc.optimize_action_sequence(self.mpc_state)
        self.get_logger().info('cuRobo MPC ready')

    def _set_goal(self, q):
        self.goal = np.asarray(q, float)
        self.mpc.update_goal_state(self._joint_state(self.goal))
        self.leg_start = self._now()
        self.arrived_since = None
        self.get_logger().info(
            f'Target {self.goal_index}: {np.round(np.degrees(self.goal), 1).tolist()} deg')

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

    def _mission(self, s0):
        q, v, _ = s0
        now = self._now()
        if np.max(np.abs(q - self.goal)) < self.arrive_tol and np.max(np.abs(v)) < 0.01:
            if self.arrived_since is None:
                self.arrived_since = now
            elif now - self.arrived_since > self.hold_at_goal_s:
                self.goal_index = (self.goal_index + 1) % len(self.targets)
                self._set_goal(self.targets[self.goal_index])
        else:
            self.arrived_since = None
            if now - self.leg_start > self.leg_timeout_s:
                self.get_logger().warn(f'Target {self.goal_index} not reached in '
                                       f'{self.leg_timeout_s:.0f} s, next target')
                self.goal_index = (self.goal_index + 1) % len(self.targets)
                self._set_goal(self.targets[self.goal_index])

    def _cycle(self):
        cycle_start = time.perf_counter()
        now = self._now()
        if self.js is None or self.info is None:
            return
        if self.cam is None:
            self._setup_camera()
        self._integrate()
        if self.frames < self.warmup_frames:
            return
        grid = self._esdf()
        if self.mpc is None:
            self._create_mpc()
            return
        self._publish_obstacles(grid)

        if self.plan is None or self.stamp + self.dt - now < self.late_margin_s:
            stamp = now + self.lead_s
            s0 = self._plan_state(stamp) if self.plan is not None else (
                self.js.copy(), np.zeros_like(self.js), np.zeros_like(self.js))
            if self.plan is not None:
                self.late += 1
        else:
            stamp = self.stamp + self.dt
            s0 = self._plan_state(stamp)
        self.stamp = stamp

        self._mission(s0)
        target = self._mpc_step(s0[0])
        n = int(round(self.cycles * self.dt / self.step_s))
        q, v, a = track_target(*s0, target, n, self.step_s, self.vmax, self.amax, self.jerk,
                               self.dt, stop_decel=self.stop_decel, qlim=self.qlim)
        tq, tv, ta = stopping_tail(q[-1], v[-1], a[-1], self.step_s, self.vmax, self.amax,
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

    def _publish_obstacles(self, grid):
        now = time.monotonic()
        if (now - self.last_marker < self.marker_period_s
                or self.marker_pub.get_subscription_count() == 0):
            return
        self.last_marker = now
        nx, ny, nz = self.grid_shape
        values = grid.feature_tensor.view(-1)[:nx * ny * nz].view(nx, ny, nz).float()
        idx = (values < 0.5 * self.esdf_voxel).nonzero()
        if idx.shape[0] > 20000:
            idx = idx[:: idx.shape[0] // 20000 + 1]
        origin = np.asarray(grid.pose[:3]) - (np.asarray(self.grid_shape) - 1) * self.esdf_voxel / 2
        pts = idx.cpu().numpy() * self.esdf_voxel + origin
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'obstacles', 0, Marker.CUBE_LIST, Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = self.esdf_voxel
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.3, 0.1, 0.8
        m.pose.orientation.w = 1.0
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts]
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = Curobo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
