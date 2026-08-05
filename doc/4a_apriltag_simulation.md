# AprilTag Pick and Place
## Execution Procedure

### Step 1: Open Isaac Sim
`~/workspaces/movensys-simulation/<MANIPULATOR_MODEL>/4a_apriltag_pick_and_place_simulation.usd`






### Step 2: Run simulator bridge
```
mros ros2 launch movensys_manipulator_moveit_config sim_bridge.launch.py use_sim_time:=true
```





### Step 3a: Launch MoveIt2's OMPL + OpenCV Apriltag
```
mros ros2 launch movensys_manipulator_perception apriltag_detector.launch.py use_sim_time:=true

mros ros2 launch movensys_manipulator_moveit_config moveit.launch.py use_sim_time:=true
```

### Step 3b: Launch cuMotion + Isaac Apriltag
```
mros ros2 launch movensys_manipulator_isaac_ros_config isaac_cumotion_apriltag.launch.py use_sim_time:=true


```







### Step 4: AprilTag Pick and Place
```
mros ros2 launch movensys_manipulator_moveit_config apriltag_pick_and_place.launch.py use_sim_time:=true target_spawn:=false
```
