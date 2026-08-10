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
                                            gazebo_bridge → gz_ros2_control → robot
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
ros2 launch movensys_manipulator_moveit_config sim_bridge.launch.py simulator:=gazebo use_sim_time:=true
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

## Tuning (`config/dobot_cr3a/quest_servo_teleop.yaml`)
- `align_yaw_deg` — rotate operator "forward" onto robot **+X**. Set this first:
  engage, push your hand forward, and adjust until the EEF moves along +X.
- `position_scale` — hand-to-EEF gain. Raise once direction is correct.
- `orientation_scale` — `1.0` = 1:1 wrist rotation; lower to damp it.
- `max_target_step` — per-cycle target clamp (glitch guard).
- Servo speed/collision limits live in `config/dobot_cr3a/servo.yaml`
  (`scale.linear/rotational`, singularity thresholds, `check_collisions`).

## Notes
- The `gazebo_bridge` does not publish `/moveit2_trajectory/execution_active`; the node
  subscribes to it for forward-compat with the Isaac Sim bridge (which pauses
  streaming and re-anchors around move_group plans) but does not require it.
- Clutch source is the analog grip (`joy_right` axes[3], `squeeze_value`,
  thresholded at 0.6). Switch to a digital button by setting
  `enable_button_index` ≥ 0 in the yaml.
