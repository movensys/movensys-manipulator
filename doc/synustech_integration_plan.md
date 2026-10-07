# Synustech Arm Integration Plan

Status as of 2026-10-07. Branch `feature/add-synuctech-robot-support`.

This document records how the Synustech arm is being added as a third
manipulator model next to the Dobot CR3A and CR5A, what has been implemented,
and what is still open. Only the arm is integrated. The mobile base, lidars,
IMU, and differential drive from the vendor package are out of scope, and so is
Isaac cuMotion support.

## 1. Background

### The robot

The arm is a **SIASUN DUCO GCR30-1100** collaborative robot, supplied through
Synustech. Nameplate: rated load 30 kg, weight 64 kg, arm length 1335 mm,
IP54, manufactured 2025-04. Data below comes from the DUCO hardware manual
v4.2 (`ref_files/Docu_related_material/duco-hardware-v4.2-en.pdf`,
GCR30-1100 chapter 2.16, printed pages 239 to 251) and the GCR series
brochure.

| Joint | Range | Max velocity |
|---|---|---|
| J1 base | ±360° | 120°/s |
| J2 shoulder | ±360° | 120°/s |
| J3 elbow | ±160° | 180°/s |
| J4, J5, J6 wrist | ±360° | 225°/s |

Dimension drawing: base to J2 axis 235 mm, upper arm 496 mm, forearm
459.5 mm, J4 axis to flange centre 144.5 mm, lateral offset base axis to
wrist 179.3 mm, wrist to flange face 121 mm. These sum to the 1335 mm arm
length on the nameplate. Repeatability ±0.05 mm. Rated load CoG offset
127.9 mm lateral, 148.8 mm axial. Packing posture J3 155°, J4 25°, others 0.

Frame convention (manual Fig. 3-12): zero pose is the arm pointing straight
up. Z1 vertical, Z2, Z3, Z4 horizontal and parallel, Z5 vertical, Z6
horizontal along the tool. The shoulder offset and the tool flange point to
DUCO +Y0. Positive rotation is right-hand about these axes (Fig. 3-13).

### Source material

Everything vendor-provided lives in the git-ignored `ref_files/` directory:

| Item | Content | Used |
|---|---|---|
| `synustech_description/urdf/assets/j0..j6.stl` | Arm link meshes, millimetres, each in its own joint frame | Yes, renamed `Link0..Link6.STL` |
| `synustech_description/urdf/manipulator.xacro` | Arm links with **all joints fixed**, rounded link lengths | Joint origins (corrected) and inertials |
| `wmx_parameters_20260305.xml` | WMX3 axis parameter export of the whole robot | Reference for the `wmx-ros2` side |
| `Docu_related_material/` | DUCO hardware manual, brochure, nameplate photo | Limits, velocities, dimensions, frame convention |
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

### Mesh geometry findings

- The meshes are authored with local **−y pointing up**, unlike the Dobot
  `Link0` which has +z up. The mount joint therefore carries a −π/2 roll in
  addition to the CR3A position and yaw.
- Every joint module in the meshes is a cylinder whose axis is the link's
  local y. All six URDF joint axes are therefore local y.
- The vendor xacro was authored in a bent pose (upper arm vertical, forearm
  horizontal, wrist rotated). The fixed pitch and roll of joints 3 to 5 were
  changed so the URDF zero is the DUCO straight-up zero. The meshes are
  unchanged.
- The vendor link lengths were rounded (230, 500, 460, 150, 175, 100 mm).
  The URDF uses the manual values. The 121 mm flange offset also makes the
  Link5 and Link6 meshes meet exactly where the vendor's 100 mm left a 20 mm
  overlap. Other joints may show seams of up to 5 mm.

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
  been zeroed in WMX. Whether that zero is the DUCO straight-up pose is not
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
| `synustech.xacro` | Macro `synustech(joint_vel, joint_effort)`. Seven links with reference inertials, STL visual and collision meshes at scale 0.001. Six `revolute` joints, all axes `0 -1 0`, manual link lengths, DUCO zero pose, manual ranges and per-joint velocity caps, damping and friction, `implicitSpringDamper` tags. |
| `movensys_manipulator.xacro` | Root `world_manipulator`, CR3A stage, `robot_joint` with the CR3A offset `xyz -0.03 0.275 0.05`, `rpy -π/2 0 -1.57`, then the arm macro with `joint_vel 4.0` so the datasheet caps apply. No gripper. |
| `stage.xacro` | Copied from CR3A. Table mesh and Jetson box. Table STL is referenced from `urdf/dobot_cr3a/assets/`. |
| `control.xacro`, `transmission.xacro`, `movensys_manipulator.gazebo.xacro` | Copied from CR5A, paths changed to `synustech`. Joints `joint1` to `joint6`, no picker joints. |
| `control.yaml` | Gazebo PID gains raised to roughly ten times the Dobot values for the heavier arm. Marked TODO, to be tuned in Gazebo. |
| `movensys_manipulator.urdf` | Static URDF generated with xacro, for parity with the other models. |
| `assets/Link0.STL` to `Link6.STL` | Reference meshes `j0` to `j6`, renamed. 2.4 MB total. |

Joint definitions as implemented:

| Joint | Parent to child | Origin xyz (m) | Origin rpy | Axis | DUCO axis at zero |
|---|---|---|---|---|---|
| joint1 | Link0 to Link1 | `0 -0.235 0` | `0 0 0` | `0 -1 0` | Z1, up |
| joint2 | Link1 to Link2 | `0 0 0.1793` | `-π/2 0 -π/2` | `0 -1 0` | Z2, +Y0 |
| joint3 | Link2 to Link3 | `0.496 0 0` | `0 0 0` | `0 -1 0` | Z3, +Y0 |
| joint4 | Link3 to Link4 | `0.4595 0 0` | `0 -π/2 0` | `0 -1 0` | Z4, +Y0 |
| joint5 | Link4 to Link5 | `0 0 -0.1445` | `π/2 0 0` | `0 -1 0` | Z5, up |
| joint6 | Link5 to Link6 | `0 0 0.121` | `-π/2 0 0` | `0 -1 0` | Z6, +Y0 |

The `Link6` origin is the flange face. The tool points along `Link6` −y.

### MoveIt config

New directory `movensys_manipulator_moveit_config/config/synustech/`, twelve
files copied from CR5A:

| File | Change |
|---|---|
| `movensys_manipulator.urdf.xacro` | Includes `urdf/synustech/movensys_manipulator.xacro` |
| `movensys_manipulator.srdf` | Chain `Link0` to `Link6`. Group states `initial`, `zero`, `test` all zero for now, which is the DUCO straight-up pose. Collision matrix rebuilt for `table`, `jetson_thor`, `Link0` to `Link6`. |
| `joint_limits.yaml` | Datasheet velocities 2.094, 2.094, 3.142, 3.927, 3.927, 3.927 rad/s. Acceleration 2.0 rad/s² placeholder. |
| `initial_positions.yaml` | All zero. |
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
- Forward kinematics on the generated URDF at zero gives the flange centre at
  Y0 300.3 mm, Z0 1335 mm in DUCO base coordinates, and every joint axis
  pointing as in manual Fig. 3-12.
- `MoveItConfigsBuilder` loads the synustech config the same way
  `moveit.launch.py` does. All yaml files parse.
- `movensys_manipulator_rviz.launch.py` runs with `MANIPULATOR_MODEL=synustech`
  from a scratch build and publishes every link. The robot is visible in RViz
  and follows the joint state sliders.
- Gazebo with the sim bridge and `moveit.launch.py` runs with
  `MANIPULATOR_MODEL=synustech` in the container. Each joint was moved in
  the positive direction and the rotation sense matches the DUCO manual
  convention (Fig. 3-13, right-hand about the axes of Fig. 3-12). The URDF
  `axis` signs are therefore correct relative to the manual. Whether the WMX
  polarity on axes 2 to 7 produces the same sense on the real arm is still
  open.

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
| 1 | Confirmation that the real arm's positive direction matches the manual | WMX polarity on axes 2 to 7 | Jog the real arm one joint at a time and compare with RViz. The URDF side is already verified against the manual in simulation. |
| 2 | Reducer ratio per joint | WMX parameter file gear ratios, real and HIL modes | Datasheet, drive part numbers, or measured by jogging a known angle |
| 3 | Physical pose at WMX encoder zero | `AbsoluteEncoderHomeOffset`, or an offset in the URDF | Whoever calibrated the arm for the 2026-03-05 export. Expected to be the DUCO straight-up zero. |
| 4 | Joint acceleration limits | `joint_limits.yaml`, servo | Not published by DUCO. Tune on hardware. |
| 5 | Whether the EtherCAT chain keeps the base drives on axes 0 and 1 | `joint_axes` mapping | Cell wiring |

### Work remaining in this repo

1. **Sign verification.** Done against the manual on 2026-10-07 in Gazebo
   with the sim bridge and MoveIt: every joint rotates in the DUCO positive
   sense. Remaining is the real-arm check, which belongs to the WMX polarity
   work in `wmx-ros2` rather than to `synustech.xacro`. Mesh seams at each
   joint still need a visual check.
2. **Record group states.** Done in simulation on 2026-10-07. `initial`
   (SRDF and `initial_positions.yaml`) is J1 0.041, J2 -0.071, J3 1.618,
   J4 0.015, J5 -1.562, J6 1.511 rad. `test` is the same pose with J6 at
   0.096 rad. Both should be re-checked once the arm has been run through
   WMX.
3. **Collision matrix check.** Confirm in RViz that `Link1` against `Link3`
   and `Link2` against `Link4` really never touch across the joint range;
   otherwise remove those `Never` rows.
4. **Full build in the container** with `colcon build`. The current install
   is a plain copy from August and does not contain the new folders.
5. **Gazebo tuning.** Run `gazebo_trajectory_simulation.launch.py` and tune
   the PID gains in `control.yaml` until the arm holds pose without
   oscillation.
6. **MoveIt execution test.** `moveit.launch.py` with the sim bridge, then
   `trajectory_test.launch.py`. Confirm OMPL and Pilz plan and execute.
7. **Commit** the description, MoveIt config, workflow, README, and doc
   changes, and open the PR.

### Work remaining in `wmx-ros2` (separate repo and PR)

Blocked on items 2 and 3 above.

1. `wmx_r2_control/urdf/synustech.wmx.urdf.xacro`, including
   `urdf/synustech/movensys_manipulator.xacro` from this repo.
2. `wmx_r2_control/urdf/synustech.wmx.ros2_control.xacro` with axis params
   `2` to `7`.
3. `wmx_r2_control/config/synustech_controllers.yaml` and a launch file,
   copied from the CR5A ones.
4. `wmx_r2_package/config/synustech_manipulator_config.yaml` with
   `joint_axes: [2, 3, 4, 5, 6, 7]` in all four controller blocks, and
   `max_joint_velocity` set from the datasheet.
5. `wmx_r2_package/config/synustech_wmx_parameters.xml` from
   `ref_files/wmx_parameters_20260305.xml`, with arm axes 2 to 7 set to
   numerator 524288 × reducer and denominator 2π, `EStopDec` rescaled, soft
   limits enabled at the manual ranges, and `AbsoluteEncoderHomeOffset`
   checked for units after the gear ratio change.
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
- URDF zero is the DUCO straight-up zero, not the vendor mesh pose, so the
  manual's joint ranges apply directly and the WMX zero is expected to match.
- Link lengths follow the manual drawing rather than the vendor meshes.
  Kinematic accuracy against the real robot was preferred over seamless
  meshes.
- `Link6` is the tool frame with the flange face at its origin, as on the
  Dobots. No separate `tcp` link.
