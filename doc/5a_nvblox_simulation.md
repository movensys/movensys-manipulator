# Nvblox Obstacle Avoidance
## Execution Procedure

### Step 1: Open Isaac Sim
`~/workspaces/movensys-simulation/<MANIPULATOR_MODEL>/5a_obstacle_avoidance_simulation.usd`






### Step 2: Run simulator bridge
```
mros ros2 launch movensys_manipulator_moveit_config sim_bridge.launch.py use_sim_time:=true
```






### Step 3: Launch cuMotion + NvBlox
```
mros ros2 launch movensys_manipulator_isaac_ros_config isaac_cumotion_nvblox.launch.py use_sim_time:=true


```







### Step 4: Obstacle Avoidance
```
mros ros2 launch movensys_manipulator_moveit_config obstacle_avoidance.launch.py use_sim_time:=true
```





### Step 5: Tuning Nvblox camera (optional)
```
mros ros2 launch movensys_manipulator_perception camera_transform_tuning.launch.py use_sim_time:=true 
    parent_frame:=world_manipulator
    child_frame:=camera_nvblox_color_optical_frame
```