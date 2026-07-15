# Nvblox Obstacle Avoidance
## Execution Procedure

### Step 1: Open Isaac Sim
`~/workspaces/movensys-simulation/<MANIPULATOR_MODEL>/7b_apriltag_obstacle_avoidance_hil.usd`






### Step 2: Run wmx-ros2 for manipulator
check `~/workspaces/movensys_ws/src/wmx-ros2/doc/launch_<MANIPULATOR_MODEL>_manipulator.md` 
set `use_sim_time:=true`






### Step 3: Launch cuMotion + NvBlox
```
mros ros2 launch movensys_manipulator_isaac_ros_config isaac_cumotion_nvblox.launch.py use_sim_time:=true
```
add `rsp:=false` if use ros2_control.





### Step 4: Launch Isaac Apriltag
```
mros ros2 launch movensys_manipulator_isaac_ros_config isaac_apriltag.launch.py use_sim_time:=true
```






### Step 5: AprilTag + Obstacle Avoidance
```
mros ros2 launch movensys_manipulator_moveit_config apriltag_pick_and_place.launch.py use_sim_time:=true target_spawn:=false
```
