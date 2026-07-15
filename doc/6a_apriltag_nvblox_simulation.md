# Nvblox Obstacle Avoidance
## Execution Procedure

### Step 1: Open Isaac Sim
`~/workspaces/movensys-simulation/<MANIPULATOR_MODEL>/6a_apriltag_obstacle_avoidance_simulation.usd`






### Step 2: Run simulator bridge
```
mros ros2 launch movensys_manipulator_moveit_config sim_bridge.launch.py simulator:=isaacsim use_sim_time:=true 
```






### Step 3: Launch cuMotion + NvBlox
```
mros ros2 launch movensys_manipulator_isaac_ros_config isaac_cumotion_nvblox.launch.py use_sim_time:=true
```





### Step 4: Launch Isaac Apriltag
```
mros ros2 launch movensys_manipulator_isaac_ros_config isaac_apriltag.launch.py use_sim_time:=true
```






### Step 5: AprilTag + Obstacle Avoidance
```
mros ros2 launch movensys_manipulator_moveit_config apriltag_pick_and_place.launch.py use_sim_time:=true target_spawn:=false
```
