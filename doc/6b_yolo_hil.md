# YOLO Pick-and-Place
## Execution Procedure

### Step 1: Open Isaac Sim
`~/workspaces/movensys-simulation/<MANIPULATOR_MODEL>/8b_yolo_pick_and_place_hil.usd`








### Step 2: Run wmx-r2 for manipulator
check `~/workspaces/movensys_ws/src/wmx-r2/doc/launch_manipulator.md` 
set `use_sim_time:=true`






### Step 3a: Launch MoveIt2's OMPL + API
```
mros ros2 launch movensys_manipulator_moveit_config moveit.launch.py use_sim_time:=true
```
add `rsp:=false` if use ros2_control.

### Step 3b: Launch cuMotion + API
```
mros ros2 launch movensys_manipulator_isaac_ros_config isaac_cumotion.launch.py use_sim_time:=true
```
add `rsp:=false` if use ros2_control.






### Step 4: Launch Yolo detector
```
mros ros2 launch movensys_manipulator_perception yolo_dice_and_cube_detector.launch.py use_sim_time:=true
```





### Step 5: Execute YOLO pick-and-place
```
mros ros2 launch movensys_manipulator_moveit_config yolo_pick_and_place.launch.py use_sim_time:=true
```

#### Debug YOLO result (Optional)
```
ros2 run rqt_image_view rqt_image_view /yolo_dice_detector/debug_image
ros2 run rqt_image_view rqt_image_view /yolo_cube_detector/debug_image
```



