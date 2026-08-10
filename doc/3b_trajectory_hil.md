# Trajectory Planning 
## Execution Procedure

### Step 1a: Open Isaac Sim
`~/workspaces/movensys-simulation/<MANIPULATOR_MODEL>/3b_trajectory_hil.usd`

### Step 1b: Open Gazebo
```
mros ros2 launch movensys_manipulator_description gazebo_trajectory_hil.launch.py
```




### Step 2: Run wmx-r2 for manipulator
check `~/workspaces/movensys_ws/src/wmx-r2/doc/launch_<MANIPULATOR_MODEL>_manipulator.md` 
set `use_sim_time:=true`





### Step 3a: Launch MoveIt2's OMPL + API
```
mros ros2 launch movensys_manipulator_moveit_config moveit.launch.py use_sim_time:=true
```
add `rsp:=false` if use gazebo (step 1b) or ros2_control.

### Step 3b: Launch cuMotion + API
```
mros ros2 launch movensys_manipulator_isaac_ros_config isaac_cumotion.launch.py use_sim_time:=true
```
add `rsp:=false` if use gazebo (step 1b) or ros2_control.





### Step 4 (optional): Drive the end effector with the keyboard
```
mros ros2 run movensys_manipulator_moveit_config keyboard_teleop
```
Pick a mode first (`j` / `t` / `p`), then jog:

| Key            | Action                                                     |
|----------------|------------------------------------------------------------|
| `j`            | **JOINT** mode — keys `1`…`6` jog joint 1…6                 |
| `t`            | **TWIST** mode — Cartesian EEF jog                          |
| `p`            | **POSE** mode — nudge an absolute EEF target pose          |
| `↑` / `↓`      | X (+ / −)  — twist jog, or pose-target nudge                |
| `←` / `→`      | Y (− / +)  — twist jog, or pose-target nudge                |
| `.` / `;`      | Z (− / +)  — twist jog, or pose-target nudge                |
| `1` … `6`      | Joint jog for joint 1 … 6            (JOINT mode)           |
| `w` / `e`      | Frame for TWIST jog **and** POSE nudge = base (`world_manipulator`) / eef (`Link6`) |
| `r`            | Reverse jog direction (twist / joint)                      |
| `q`            | Quit                                                       |




### Step 5 (optional): Execute Trajectory Test
```
mros ros2 launch movensys_manipulator_moveit_config trajectory_test.launch.py use_sim_time:=true
```

### Step 6 (optional): Execute Coverage Test
```
mros ros2 launch movensys_manipulator_moveit_config coverage_pose.launch.py use_sim_time:=true
```