import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    MediaPipe hand -> MoveIt Servo POSE teleop (relative clutch).

    Same executable as quest_servo_teleop.launch.py; only the pose source and
    the operator->robot frame differ, and both are parameters. See
    config/<model>/hand_servo_teleop.yaml.

    Assumes moveit.launch.py (move_group + servo_node + RViz) and the sim
    bridge (sim_bridge.launch.py) are already running, and that the perception
    node is publishing /hand_pose and /hand_joy:

        python -m perception.ros_node --ros-args -p use_sim_time:=true
        python -m perception.ros_node --replay session.csv --loop \
            --ros-args -p use_sim_time:=true

    use_sim_time must match on both sides. The pose carries a stamp, the
    teleop node will time it out (phase 6), and a wall-clock stamp compared
    against Gazebo's /clock is off by about 1.7e9 seconds in a direction no
    timeout test catches.
    """
    manipulator_model = os.environ.get("MANIPULATOR_MODEL", "dobot_cr3a")
    params_file = os.path.join(
        get_package_share_directory("movensys_manipulator_moveit_config"),
        "config",
        manipulator_model,
        "hand_servo_teleop.yaml",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time (true for Gazebo/Isaac Sim)",
        ),
        Node(
            package="movensys_manipulator_moveit_config",
            executable="quest_servo_teleop",
            # Renamed so this and the Quest teleop can never collide in the
            # graph. The params file keys on /** precisely so that renaming
            # the node cannot silently drop every parameter to its code
            # default -- which for quest_axis_map is the identity "x,y,z".
            # Phase 4's runtime commands address this name:
            #   ros2 param set /hand_servo_teleop align_rpy_deg "[0,45,180]"
            name="hand_servo_teleop",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
    ])
