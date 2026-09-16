# Nvblox Obstacle Avoidance (Real)
## Execution Procedure

### Step 1: Open Isaac Sim
`~/workspaces/movensys-simulation/<MANIPULATOR_MODEL>/5c_obstacle_avoidance_real.usd`






### Step 2: Run wmx-r2 for manipulator
check `~/workspaces/movensys_ws/src/wmx-r2/doc/launch_manipulator.md` 





### Step 3: Launch cuMotion + NvBlox
```
mros ros2 launch movensys_manipulator_isaac_ros_config isaac_cumotion_nvblox.launch.py
```
add `rsp:=false` if use ros2_control.







### Step 4: Obstacle Avoidance
```
mros ros2 launch movensys_manipulator_moveit_config obstacle_avoidance.launch.py
```




### Step 5: Tuning Nvblox camera (optional)
```
mros ros2 launch movensys_manipulator_perception camera_transform_tuning.launch.py
    parent_frame:=world_manipulator
    child_frame:=camera_nvblox_link
```