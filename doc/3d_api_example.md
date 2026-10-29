# API Node Examples

## Publisher
### EEF pose (base frame)
```
mros ros2 topic echo /wmx/moveit2/eef_pose
```

### EEF orientation as roll/pitch/yaw (rad)
```
mros ros2 topic echo /wmx/moveit2/eef_rpy
```

### joint states 
```
mros ros2 topic echo /joint_states
```

## Subscriber
### Servo tool-frame pose target
```
mros ros2 topic pub --once /wmx/servo_node/tool_pose geometry_msgs/msg/PoseStamped \
        '"{pose: {position: {x: 0.0, y: 0.0, z: 0.01}, orientation: {x: 0.1305, y: 0.0, z: 0.0, w: 0.9914}}}"'
```

### Jog joints (JOINT_JOG, rad/s per joint):
```
mros ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType '"{command_type: 0}"'
mros ros2 topic pub --rate 50 /servo_node/delta_joint_cmds control_msgs/msg/JointJog \
        '"{joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], velocities: [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]}"'
```

## Jog the EEF (TWIST, `header.frame_id` selects the frame — `world_manipulator` or `Link6`):
```
mros ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType '"{command_type: 1}"'
mros ros2 topic pub --rate 50 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped \
        '"{header: {frame_id: world_manipulator}, twist: {linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"'
```

## Track an absolute EEF target (POSE, **base frame only**):
```
mros ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType '"{command_type: 2}"'
mros ros2 topic pub --rate 50 /servo_node/pose_target_cmds geometry_msgs/msg/PoseStamped \
        '"{header: {frame_id: world_manipulator}, pose: {position: {x: -0.158, y: -0.071, z: 0.346}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}"'
```

## Service
### Get EEF pose 
```
mros ros2 service call /wmx/moveit2/get_eef_pose movensys_manipulator_moveit_config/srv/GetEefPose '"{}"'
```

### Gripper service
```
mros ros2 service call /wmx/set_gripper std_srvs/srv/SetBool '"{data: true}"'               
```

### Get tf for specific transform 
```
mros ros2 run tf2_ros tf2_echo world_manipulator Link6
```

### Absolute cartesian move (base frame)
```
mros ros2 service call /wmx/moveit2/absolute_base_eef_cartesian \
        movensys_manipulator_moveit_config/srv/MovePose \
        '"{pos: [-0.158, -0.071, 0.346], ori: [3.14159265, 0.0, -3.14159265]}"'
```

### Relative cartesian move (base frame)
```
mros ros2 service call /wmx/moveit2/relative_base_eef_cartesian \
        movensys_manipulator_moveit_config/srv/MovePose \
        '"{pos: [0.05, 0.0, 0.0], ori: [0.0, 0.0, 0.0]}"'
```

### Relative cartesian move (tool frame)
```
mros ros2 service call /wmx/moveit2/relative_tool_eef_cartesian \
        movensys_manipulator_moveit_config/srv/MovePose \
        '"{pos: [0.05, 0.0, 0.0], ori: [0.0, 0.0, 0.0]}"'
```

### Absolute joint-space move (pose target, base frame) 
```
mros ros2 service call /wmx/moveit2/absolute_base_eef_joint_movement \
        movensys_manipulator_moveit_config/srv/MovePose \
        '"{pos: [-0.005, -0.071, 0.346], ori: [3.14159265, 0.0, -3.14159265]}"'
```

### Joint movement (absolute) 
```
mros ros2 service call /wmx/moveit2/joint_movement \
        movensys_manipulator_moveit_config/srv/MoveJoints \
        '"{joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], joint_values: [0.0, 0.0, -1.57, 0.0, 1.57, 0.0]}"'
```

### Joint movement (relative / increment) 
```
mros ros2 service call /wmx/moveit2/relative_joint_movement \
        movensys_manipulator_moveit_config/srv/MoveJoints \
        '"{joint_names: [joint1, joint2], joint_values: [0.2, 0.2]}"'
```