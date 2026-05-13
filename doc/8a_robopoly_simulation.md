# Running Robopoly Game
## Execution Procedure

### Step 1: Open Isaac Sim
`~/workspaces/movensys-simulation/dobot_cr3a/6a_robopoly_simulation.usd`


### Step 2: Run simulator bridge
```
mros ros2 launch movensys_manipulator_moveit_config sim_bridge.launch.py simulator:=isaacsim use_sim_time:=true 
```

### Step 3: Launch MoveIt2's OMPL + API
```
mros ros2 launch movensys_manipulator_moveit_config moveit.launch.py use_sim_time:=true
```

### Step 4: Running movensys_vlm
```
cd ~/workspaces/movensys-intelligence/movensys_vlm/docker
docker compose down
docker compose build
docker compose up
```

### Step 5: Running movensys_robopoly
```
cd ~/workspaces/movensys-intelligence/movensys_sample/movensys_robopoly
python3 -m uvicorn main:app --host 0.0.0.0 --port 7999
```

### Step 6: Enjoy the robopoly game
1. Click `Toggle is_YOLO` and check `is_YOLO` is set to OFF.
2. Click Reset game and play the game.