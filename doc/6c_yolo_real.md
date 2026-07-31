# YOLO Pick-and-Place (Real Hardware)
## Execution Procedure

### Step 1: Open Isaac Sim
`~/workspaces/movensys-simulation/<MANIPULATOR_MODEL>/8c_yolo_pick_and_place_real.usd`





### Step 2: Run wmx-r2 for manipulator
check `~/workspaces/movensys_ws/src/wmx-r2/doc/launch_<MANIPULATOR_MODEL>_manipulator.md`




### Step 3a: Launch MoveIt2's OMPL + API
```
mros ros2 launch movensys_manipulator_moveit_config moveit.launch.py
```
add `rsp:=false` if use ros2_control.

### Step 3b: Launch cuMotion + API
```
mros ros2 launch movensys_manipulator_isaac_ros_config isaac_cumotion.launch.py
```
add `rsp:=false` if use ros2_control.



### Step 4: Launch the YOLO cube detector
```
mros ros2 launch movensys_manipulator_perception yolo_cube_detector.launch.py
```












#### Debug YOLO result (Optional)
```bash
ros2 run rqt_image_view rqt_image_view /yolo_cube_detector/debug_image
```


### Step 6: Execute YOLO pick-and-place (Optional)
```
mros ros2 launch movensys_manipulator_moveit_config yolo_pick_and_place.launch.py
```
Loads pick/drop poses from `config/<MANIPULATOR_MODEL>/yolo_trajectory.yaml`.
The node moves to the scan pose, visually servos over each cube in
`cube_classes` order, picks it up, and places it in the per-class drop box.
