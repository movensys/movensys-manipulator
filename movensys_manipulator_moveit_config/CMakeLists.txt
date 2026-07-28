cmake_minimum_required(VERSION 3.8)
project(movensys_manipulator_moveit_config)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# MoveIt Servo's ServoCommandType service only exists on Jazzy and newer; it is
# absent on Humble. Guard the servo teleop executables so the package still
# builds on distros that lack it. moveit_msgs_DIR points at share/moveit_msgs/cmake.
if(EXISTS "${moveit_msgs_DIR}/../srv/ServoCommandType.srv")
  set(HAVE_SERVO_COMMAND_TYPE TRUE)
else()
  message(STATUS "moveit_msgs/srv/ServoCommandType not found; "
    "skipping servo teleop executables (keyboard_teleop, quest_servo_teleop)")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MovePose.srv"
  "srv/MoveJoints.srv"
  "srv/GetEefPose.srv"
)

# MoveIt2 Client Library
add_library(moveit2_client_lib SHARED
  src/moveit2_client.cpp
)
ament_target_dependencies(moveit2_client_lib
  rclcpp
  std_srvs
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
)
target_include_directories(moveit2_client_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

install(DIRECTORY
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include
)

add_executable(isaacsim_bridge src/isaacsim_bridge.cpp)
ament_target_dependencies(isaacsim_bridge rclcpp rclcpp_action control_msgs trajectory_msgs sensor_msgs std_msgs std_srvs)

add_executable(gazebo_bridge src/gazebo_bridge.cpp)
ament_target_dependencies(gazebo_bridge rclcpp rclcpp_action control_msgs trajectory_msgs sensor_msgs std_srvs)

if(HAVE_SERVO_COMMAND_TYPE)
  add_executable(keyboard_teleop src/keyboard_teleop.cpp)
  ament_target_dependencies(keyboard_teleop rclcpp geometry_msgs control_msgs moveit_msgs std_msgs tf2 tf2_ros)

  add_executable(quest_servo_teleop src/quest_servo_teleop.cpp)
  ament_target_dependencies(quest_servo_teleop rclcpp geometry_msgs sensor_msgs moveit_msgs std_msgs tf2 tf2_ros)
endif()

add_executable(trajectory_test src/trajectory_test.cpp)
target_link_libraries(trajectory_test moveit2_client_lib)
ament_target_dependencies(trajectory_test
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  std_srvs
)

add_executable(coverage_pose src/coverage_pose.cpp)
target_link_libraries(coverage_pose moveit2_client_lib)
ament_target_dependencies(coverage_pose
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  std_srvs
)

add_executable(obstacle_avoidance_cpp src/obstacle_avoidance.cpp)
target_link_libraries(obstacle_avoidance_cpp moveit2_client_lib)
ament_target_dependencies(obstacle_avoidance_cpp
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  std_srvs
)

add_executable(apriltag_pick_and_place src/apriltag_pick_and_place.cpp)
target_link_libraries(apriltag_pick_and_place moveit2_client_lib)
ament_target_dependencies(apriltag_pick_and_place
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  std_srvs
)

add_executable(yolo_pick_and_place_cpp src/yolo_pick_and_place.cpp)
target_link_libraries(yolo_pick_and_place_cpp moveit2_client_lib)
ament_target_dependencies(yolo_pick_and_place_cpp
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  std_srvs
)

add_executable(moveit2_api src/moveit2_api.cpp)
target_link_libraries(moveit2_api moveit2_client_lib)
ament_target_dependencies(moveit2_api
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  std_srvs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(moveit2_api ${cpp_typesupport_target})

install(TARGETS
  trajectory_test
  coverage_pose
  obstacle_avoidance_cpp
  apriltag_pick_and_place
  moveit2_api
  yolo_pick_and_place_cpp
  DESTINATION lib/${PROJECT_NAME}
)

if(HAVE_SERVO_COMMAND_TYPE)
  install(TARGETS
    keyboard_teleop
    quest_servo_teleop
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(TARGETS moveit2_client_lib isaacsim_bridge gazebo_bridge
  EXPORT export_moveit2_client_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_moveit2_client_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_srvs
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  Eigen3
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_srv_interfaces test/test_srv_interfaces.cpp)
  if(TARGET test_srv_interfaces)
    rosidl_get_typesupport_target(test_cpp_typesupport_target
      ${PROJECT_NAME} rosidl_typesupport_cpp)
    target_link_libraries(test_srv_interfaces "${test_cpp_typesupport_target}")
  endif()
endif()

ament_package()
