# 1. Note For Isaac-ROS only
please follow this isaac-ros setup first: 
        https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/index.html
        https://nvidia-isaac-ros.github.io/v/release-4.1/getting_started/index.html
 
# 2. Docker setup
```
cd ~/workspaces/movensys_ws/src/movensys-manipulator/docker
docker compose -f ${MOVENSYS_ROS_VERSION}.yaml -f movensys_manipulator.${CPU_ARCH}.yaml down
docker compose -f ${MOVENSYS_ROS_VERSION}.yaml -f movensys_manipulator.${CPU_ARCH}.yaml build
docker compose -f ${MOVENSYS_ROS_VERSION}.yaml -f movensys_manipulator.${CPU_ARCH}.yaml up -d
```

# 3. Checking Docker
```
docker logs movensys_manipulator_container -f
```

# 4. Get inside Docker
```
mros
```

# 5. Checking URDF 
```
mros ros2 launch movensys_manipulator_description movensys_manipulator_rviz.launch.py
```
