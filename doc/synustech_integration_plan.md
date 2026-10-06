# Synustech Arm Integration Plan

Status as of 2026-10-06. Branch `feature/add-synuctech-robot-support`.

This document records how the Synustech 6-axis arm is being added as a third
manipulator model next to the Dobot CR3A and CR5A, what has been implemented,
and what is still open. Only the arm is integrated. The mobile base, lidars,
IMU, and differential drive from the vendor package are out of scope, and so is
Isaac cuMotion support.

## 1. Background

### Source material

Everything vendor-provided lives in the git-ignored `ref_files/` directory:

| Item | Content | Used |
|---|---|---|
| `ref_files/synustech_description/` | Gazebo Classic package for the full mobile manipulator | Arm meshes, joint origins, inertials |
| `ref_files/synustech_description/urdf/assets/j0..j6.stl` | Arm link meshes, millimetres, each in its own joint frame | Yes, renamed `Link0..Link6.STL` |
| `ref_files/synustech_description/urdf/manipulator.xacro` | Arm links with **all joints fixed** | Origins and inertials only |
| `ref_files/wmx_parameters_20260305.xml` | WMX3 axis parameter export of the whole robot | Reference for the `wmx-ros2` side |
| DAE meshes, zip, mobile base, wheels, lidars, EKF, diff drive | Not used | No |

### How the repo selects a robot

Every launch file reads `MANIPULATOR_MODEL` and resolves
`movensys_manipulator_description/urdf/<model>/` and
`movensys_manipulator_moveit_config/config/<model>/`. Nothing else is model
specific as long as the model keeps these names, which are hard-coded in the
client yaml, teleop nodes, sim bridge, and WMX configs:

- links `Link0` to `Link6`, joints `joint1` to `joint6`
- root link `world_manipulator`
- planning group `movensys_manipulator_arm`
- controller `movensys_manipulator_arm_controller`

The Synustech model follows this convention, so no C++ or launch code changed.
The gripper-less Dobot CR5A was used as the template.

### Geometry derived from the reference

Forward kinematics of the fixed reference chain, relative to the arm mount:

| Frame | Position x, y, z (m) | Role |
|---|---|---|
| Link0 | 0, 0, 0 | base, 141 mm tall |
| Link1 | 0, 0, 0.23 | joint1, vertical |
| Link2 | 0, 0.175, 0.23 | shoulder, lateral offset |
| Link3 | 0, 0.175, 0.73 | elbow, upper arm 0.5 m |
| Link4 | 0.46, 0.175, 0.73 | forearm 0.46 m |
| Link5 | 0.46, 0.175, 0.58 | wrist, 0.15 m drop |
| Link6 | 0.46, 0.075, 0.58 | flange, tool along local +y, 55 mm |

Reach from the shoulder is about 0.96 m. Total arm mass from the CAD inertials
is 61.4 kg, with a 25.9 kg upper arm.

The meshes are authored with local **−y pointing up**, unlike the Dobot
`Link0` which has +z up. The mount joint therefore carries a −π/2 roll in
addition to the CR3A position and yaw.

### Findings from the WMX parameter export

- The export covers the whole mobile manipulator. WMX axes 0 and 1 are the
  drive wheels in velocity mode. The six arm joints are WMX axes **2 to 7**.
- Arm axis gear ratio is 524288 / 360, meaning one axis unit is one degree of
  the 19-bit motor encoder with **no reducer folded in**. The Dobot files use
  encoder counts × reducer / 2π, so one unit is one joint radian. The ROS nodes
  send joint radians unconverted, so the Synustech parameter file for
  `wmx-ros2` must be changed to numerator 524288 × reducer and denominator 2π
  per joint, and `EStopDec` rescaled to match.
- Polarity on arm axes 2 to 7 is −1, −1, +1, −1, −1, −1.
- `AbsoluteEncoderHomeOffset` is nonzero on all six arm axes, so the arm has
  been zeroed in WMX. The physical pose that zero corresponds to is not
  documented.
- Soft limits, following error checks, limit switches, and the E-stop signal
  are all disabled. Torque limit 300 percent, max motor speed 3000 rpm.
- The export is from an older WMX3 build and lacks the closed-loop PID, max
  speed unit, and torque ramp tags present in the Dobot files.

## 2. What has been done

All items below are implemented in the working tree and validated. Nothing is
committed yet.

### Description package

New directory `movensys_manipulator_description/urdf/synustech/`:

| File | Content |
|---|---|
| `synustech.xacro` | Macro `synustech(joint_vel, joint_effort)`. Seven links with reference inertials, STL visual and collision meshes at scale 0.001. Six `revolute` joints with reference origins written as `${pi/2}` and `${pi}`, proposed axes, placeholder limits ±3.14 rad, damping and friction, `implicitSpringDamper` tags. |
| `movensys_manipulator.xacro` | Root `world_manipulator`, CR3A stage, `robot_joint` with the CR3A offset `xyz -0.03 0.275 0.05`, `rpy -π/2 0 -1.57`, then the arm macro. No gripper. |
| `stage.xacro` | Copied from CR3A. Table mesh and Jetson box. Table STL is referenced from `urdf/dobot_cr3a/assets/` to avoid duplicating it. |
| `control.xacro`, `transmission.xacro`, `movensys_manipulator.gazebo.xacro` | Copied from CR5A, paths changed to `synustech`. Joints `joint1` to `joint6`, no picker joints. |
| `control.yaml` | Gazebo PID gains raised to roughly ten times the Dobot values for the heavier arm. Marked TODO, to be tuned in Gazebo. |
| `movensys_manipulator.urdf` | Static URDF generated with xacro, for parity with the other models. |
| `assets/Link0.STL` to `Link6.STL` | Reference meshes `j0` to `j6`, renamed. 2.4 MB total. |

Proposed joint axes, inferred from mesh geometry and marked TODO in the xacro:

| Joint | Parent to child | Origin | Axis (local) | Direction in world at zero |
|---|---|---|---|---|
| joint1 | Link0 to Link1 | `0 -0.23 0` | `0 -1 0` | vertical |
| joint2 | Link1 to Link2 | `0 0 0.175`, rpy `-π/2 0 -π/2` | `0 -1 0` | horizontal, shoulder pitch |
| joint3 | Link2 to Link3 | `0.5 0 0`, rpy `0 -π/2 0` | `0 -1 0` | horizontal, elbow pitch |
| joint4 | Link3 to Link4 | `0.46 0 0`, rpy `0 π 0` | `0 0 1` | vertical, wrist roll |
| joint5 | Link4 to Link5 | `0 0 -0.15`, rpy `π/2 0 π` | `1 0 0` | along forearm, wrist pitch |
| joint6 | Link5 to Link6 | `0 0 0.1`, rpy `-π/2 0 0` | `0 1 0` | flange axis |

### MoveIt config

New directory `movensys_manipulator_moveit_config/config/synustech/`, twelve
files copied from CR5A:

| File | Change |
|---|---|
| `movensys_manipulator.urdf.xacro` | Includes `urdf/synustech/movensys_manipulator.xacro` |
| `movensys_manipulator.srdf` | Chain `Link0` to `Link6`. Group states `initial`, `zero`, `test` all zero for now. Collision matrix rebuilt for `table`, `jetson_thor`, `Link0` to `Link6`. |
| `joint_limits.yaml` | CR5A values kept as placeholders, 1.0 rad/s and 1.0 rad/s². TODO comment added. |
| `initial_positions.yaml` | All zero. TODO comment added. |
| `trajectory.yaml` | Waypoints moved into this arm's workspace, about 0.45 m in front of the base at 0.6 m height, tool pointing down. |
| `kinematics.yaml`, `moveit_controllers.yaml`, `moveit2_client.yaml`, `sim_bridge.yaml`, `servo.yaml`, `pilz_cartesian_limits.yaml`, `movensys_manipulator_arm.ros2_control.xacro` | Unchanged from CR5A. Frames `world_manipulator` and `Link6` and all topics already match. |

### Repo wiring

- `synustech` added to both `for model in` loops in
  `.github/workflows/test-description.yml`.
- `synustech` added to the supported model lists in `README.md` and
  `doc/1_setup.md`, and to the description package row in the README table.
- `ref_files/` stays git-ignored. The meshes were copied into the description
  package.

### Validation performed

- `xacro` and `check_urdf` pass on `movensys_manipulator.xacro`,
  `movensys_manipulator.gazebo.xacro`, and the MoveIt
  `movensys_manipulator.urdf.xacro`.
- Forward kinematics on the generated URDF confirms the arm stands upright,
  joint1 is vertical, joints 2 and 3 are parallel and horizontal, and joints 4,
  5, 6 are mutually perpendicular.
- `MoveItConfigsBuilder` loads the synustech config the same way
  `moveit.launch.py` does. All yaml files parse.
- `movensys_manipulator_rviz.launch.py` runs with `MANIPULATOR_MODEL=synustech`
  from a scratch build and publishes every link. The robot is visible in RViz.
- Gazebo was not tested. No Gazebo install is available on the development
  host, so that step has to run in the container.

### Gotcha found and fixed

The description launch files pass the xacro output directly as the
`robot_description` parameter, and launch_ros tries to parse it as YAML. Any
colon followed by a space in the URDF, including inside XML comments, makes
launch fail with "Unable to parse the value of parameter robot_description as
yaml". The Dobot files happen to contain none. All comments in the Synustech
xacros avoid `": "`. Check with `grep -c ": " movensys_manipulator.urdf`,
which must print 0.

## 3. Remaining items

### Inputs needed from outside the repo

| # | Item | Blocks | Source |
|---|---|---|---|
| 1 | Joint axis directions and signs for all six joints | URDF correctness, everything downstream | Verify the proposed axes against the real arm in RViz, or the datasheet |
| 2 | Joint position limits | URDF `limit`, `joint_limits.yaml`, servo margins | Datasheet |
| 3 | Joint velocity and acceleration limits | `joint_limits.yaml`, servo, WMX `max_joint_velocity` | Datasheet |
| 4 | Reducer ratio per joint | WMX parameter file gear ratios, real and HIL modes | Datasheet, drive part numbers, or measured by jogging a known angle |
| 5 | Physical pose at WMX encoder zero | URDF zero pose, or `AbsoluteEncoderHomeOffset` | Whoever calibrated the arm for the 2026-03-05 export |
| 6 | Whether the EtherCAT chain keeps the base drives on axes 0 and 1 | `joint_axes` mapping | Cell wiring |

### Work remaining in this repo

1. **Axis verification in RViz.** Launch
   `movensys_manipulator_rviz.launch.py` with `MANIPULATOR_MODEL=synustech`,
   move each slider, compare against the real arm, and flip signs in
   `synustech.xacro` where needed. Remove the TODO markers once done.
2. **Fill in limits.** Replace the ±3.14 placeholders in `synustech.xacro`
   and the CR5A values in `joint_limits.yaml` with datasheet numbers.
3. **Record group states.** Set `initial` and `test` in the SRDF and
   `initial_positions.yaml` to real poses, and update `trajectory.yaml`
   waypoints accordingly.
4. **Collision matrix check.** Confirm in RViz that `Link1` against `Link3`
   and `Link2` against `Link4` really never touch across the joint range;
   otherwise remove those `Never` rows.
5. **Full build in the container** with `colcon build`. The current install
   is a plain copy from August and does not contain the new folders.
6. **Gazebo tuning.** Run `gazebo_trajectory_simulation.launch.py` and tune
   the PID gains in `control.yaml` until the arm holds pose without
   oscillation.
7. **MoveIt execution test.** `moveit.launch.py` with the sim bridge, then
   `trajectory_test.launch.py`. Confirm OMPL and Pilz plan and execute.
8. **Commit** the description, MoveIt config, workflow, README, and doc
   changes, and open the PR.

### Work remaining in `wmx-ros2` (separate repo and PR)

Blocked on items 4 and 5 above.

1. `wmx_r2_control/urdf/synustech.wmx.urdf.xacro`, including
   `urdf/synustech/movensys_manipulator.xacro` from this repo.
2. `wmx_r2_control/urdf/synustech.wmx.ros2_control.xacro` with axis params
   `2` to `7`.
3. `wmx_r2_control/config/synustech_controllers.yaml` and a launch file,
   copied from the CR5A ones.
4. `wmx_r2_package/config/synustech_manipulator_config.yaml` with
   `joint_axes: [2, 3, 4, 5, 6, 7]` in all four controller blocks.
5. `wmx_r2_package/config/synustech_wmx_parameters.xml` from
   `ref_files/wmx_parameters_20260305.xml`, with arm axes 2 to 7 set to
   numerator 524288 × reducer and denominator 2π, `EStopDec` rescaled, soft
   limits enabled, and `AbsoluteEncoderHomeOffset` checked for units after the
   gear ratio change.
6. First import with the `ImportAndSetAll` error structs inspected before
   servo-on, since the export comes from an older WMX3 build.
7. HIL run, then real run with `vel_scale` and `acc_scale` in
   `moveit2_client.yaml` set to 0.1.

## 4. Decisions taken

- Directory and model name is `synustech`, matching the vendor package name.
  The branch name spells it `synuctech`.
- No gripper and no end effector group, as for the CR5A.
- No `movensys_manipulator.xrdf` and no Isaac launch changes. cuMotion is out
  of scope for this model.
- The CR3A stage and mount offset are reused so the arm can be compared with
  the CR3A in the same cell layout.
- The mesh authored pose is the URDF zero pose until the encoder zero pose is
  known. Reconciliation will be done through the WMX home offset, not by
  rotating the URDF frames.
