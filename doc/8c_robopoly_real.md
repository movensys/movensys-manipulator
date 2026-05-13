# Running Robopoly Game
## Execution Procedure

### Step 1: Open Isaac Sim
`~/workspaces/movensys-simulation/dobot_cr3a/6a_robopoly_real.usd`

### Step 2: Run wmx-ros2 for manipulator
check `~/workspaces/movensys_ws/src/wmx-ros2/doc/launch_<MANIPULATOR_MODEL>_manipulator.md`

### Step 3: Launch MoveIt2's OMPL + API
```
mros ros2 launch movensys_manipulator_moveit_config moveit.launch.py use_sim_time:=true
```

### Step 4: Run YOLO for cube detection
```bash
cd ~/workspaces/movensys_ws/src/movensys-manpulator/movensys_manipulator_perception
mros ros2 launch movensys_manipulator_perception yolo_cube_detector.launch.py
```

### Step 5: Run YOLO for dice detection
```bash
cd ~/workspaces/movensys_ws/src/movensys-manpulator/movensys_manipulator_perception
mros ros2 launch movensys_manipulator_perception yolo_dice_detector.launch.py
```

### Step 6: Run YOLO debugger (Optional)
```bash
ros2 run rqt_image_view rqt_image_view /yolo_dice_detector/debug_image
ros2 run rqt_image_view rqt_image_view /yolo_cube_detector/debug_image
```

### Step 7: Running movensys_vlm
```
cd ~/workspaces/movensys-intelligence/movensys_vlm/docker
docker compose down
docker compose build
docker compose up
```

### Step 8: Running movensys_robopoly
```
cd ~/workspaces/movensys-intelligence/movensys_sample/movensys_robopoly
python3 -m uvicorn main:app --host 0.0.0.0 --port 7999
```

### Step 9: Play the robopoly game
1. Click `Toggle is_YOLO` and check `is_YOLO` is set to ON.
2. Click `Reset game` and `Roll dice`. Enjoy the game.