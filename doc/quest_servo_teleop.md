# Quest controller → MoveIt Servo teleoperation (Gazebo)

Drive the arm end-effector with a **Meta Quest right controller** through MoveIt 2
Servo POSE mode, using a **relative clutch** (right grip = deadman). v1 is arm
motion only (no gripper).

```
quest_pose_node (teleop container)                 quest_servo_teleop (manipulator container)
  /quest_pose_publisher/controller_pose_right ───►  latest controller pose
  /quest_pose_publisher/joy_right (grip=clutch) ─►  engage/disengage
                                                     │  relative clutch mapping
                                                     ▼
                                    /servo_node/pose_target_cmds ──► moveit_servo::ServoNode
                                                                       ▼
                                              sim_bridge → gz_ros2_control → robot
```

## Prerequisites
- Both containers share `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
  (host networking; use the Jazzy / `isaac-ros_4.1` track on both so the Servo
  message hashes match).
- `MANIPULATOR_MODEL=dobot_cr3a`.
- Build the workspace after adding these files: `colcon build --symlink-install`.

## Bring-up

**1 — Quest pose + buttons (teleop container).** Connect the Quest via the Isaac
Teleop web client, then:
```bash
ros2 run quest_pose_publisher quest_pose_node --ros-args \
    -p hand:=right -p pose_source:=grip -p cloudxr_accept_eula:=true
```
Verify from the manipulator container:
```bash
ros2 topic hz /quest_pose_publisher/controller_pose_right   # ~60 Hz
ros2 topic echo /quest_pose_publisher/joy_right             # axes[3]=grip
```

**2 — Gazebo + controllers.**
```bash
ros2 launch movensys_manipulator_description gazebo_trajectory_simulation.launch.py
```

**3 — MoveIt (move_group + servo_node + RViz) and the sim bridge.**
```bash
ros2 launch movensys_manipulator_moveit_config moveit.launch.py use_sim_time:=true rsp:=false
ros2 launch movensys_manipulator_moveit_config sim_bridge.launch.py use_sim_time:=true
```

**4 — Quest servo teleop.**
```bash
ros2 launch movensys_manipulator_moveit_config quest_servo_teleop.launch.py use_sim_time:=true
```
It switches Servo to POSE mode automatically (`/servo_node/switch_command_type`).

## Operating
- **Hold the right grip** to engage. The EEF follows your hand's motion since the
  moment you gripped, scaled by `position_scale` (default 0.5).
- **Release** to freeze (Servo halts on command timeout). Reposition your hand and
  grip again to continue — the target re-anchors, so there is no jump.

## Constrained motion (DOF modes)

The clutch delta is multiplied by a per-axis gain `[tx, ty, tz, rx, ry, rz]`
before it reaches the anchor, so a gain of `0` pins that axis at its anchored
value and anything between `0` and `1` damps it. Named presets:

| mode | tx ty tz | rx ry rz | use |
|---|---|---|---|
| `full` | 1 1 1 | 1 1 1 | 6-DOF (default) |
| `translation` | 1 1 1 | 0 0 0 | move without re-orienting the tool |
| `rotation` | 0 0 0 | 1 1 1 | re-orient in place |
| `planar` | 1 1 0 | 0 0 1 | tabletop: XY + yaw |
| `vertical` | 0 0 1 | 0 0 0 | straight up/down |
| `yaw` / `pitch` / `roll` | 0 0 0 | one axis | single-axis wrist work |
| `custom` | `custom_dof_gain` | | anything else, incl. fractional gains |

Switch at runtime:
```bash
ros2 param set /quest_servo_teleop motion_mode translation
ros2 topic echo /quest_servo_teleop/active_mode     # latched, for a UI
```
**Every switch re-anchors the clutch**, so the arm does not jump when a mask
drops an axis the operator had already moved. If the re-anchor cannot read TF,
the clutch is dropped instead of jumping; the next joy message re-engages.

`mask_frame` picks the axes the mask is expressed in — `base` (the robot base)
or `tool` (the EEF axes *as anchored at engage*, for approach-axis work like
insertion). It is deliberately the anchored tool orientation, not the live one:
a live frame would rotate the constraint surface out from under the operator.

To switch modes from inside the headset, set `mode_cycle_button` to an index
into `Joy.buttons` = `[primary, secondary, thumbstick, menu]`; a rising edge
steps through `mode_cycle_list`. It defaults to `-1` (off) so the face buttons
stay free for the gripper.

Rotation masking goes through the rotation-vector (log map) representation
rather than an Euler decomposition, so it is continuous everywhere instead of
gimbal-locking at pitch ±90°. One consequence worth knowing: because rotations
do not commute, masking a *composite* delta to one axis gives the projection of
its rotation vector, not the corresponding Euler factor — a 30° roll followed by
a 40° yaw masks to 39.1° of yaw, not 40°.

## Tuning (`config/dobot_cr3a/quest_servo_teleop.yaml`)
- `align_yaw_deg` — rotate operator "forward" onto robot **+X**. Set this first:
  engage, push your hand forward, and adjust until the EEF moves along +X.
- `position_scale` — hand-to-EEF gain. Raise once direction is correct.
- `orientation_scale` — `1.0` = 1:1 wrist rotation; lower to damp it.
- `motion_mode` / `mask_frame` / `custom_dof_gain` — see **Constrained motion**.
- `max_target_step` — per-cycle target clamp (glitch guard).
- Servo speed/collision limits live in `config/dobot_cr3a/servo.yaml`
  (`scale.linear/rotational`, singularity thresholds, `check_collisions`).

## Notes
- `sim_bridge` publishes `/moveit2_trajectory/execution_active` (latched) on both the
  Gazebo and Isaac Sim paths, so streaming pauses and POSE mode re-anchors around
  move_group plans. The node only subscribes to it and does not require it. If that
  post-trajectory re-anchor cannot read TF the clutch is dropped rather than left on
  the stale anchor, which would snap the arm back to its pre-plan pose.
- All TF reads are non-blocking. They run on executor threads, so a retry loop there
  would stall the pose stream past Servo's `incoming_command_timeout` and trip a
  halt/resume jerk. The stream timer also runs in its own callback group under a
  multi-threaded executor, so no other callback can delay a command.
- Masking the *target* does not guarantee the arm holds a pinned axis exactly:
  Servo's IK, singularity damping and collision slowdown can leave residual
  motion there. This is a teleop mapping, not a hard constraint.
- Clutch source is the analog grip (`joy_right` axes[3], `squeeze_value`,
  thresholded at 0.6). Switch to a digital button by setting
  `enable_button_index` ≥ 0 in the yaml.
