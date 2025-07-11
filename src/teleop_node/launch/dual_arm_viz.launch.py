from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    # Find the package share directory
    dual_arm_config = FindPackageShare("moveit_resources").find("moveit_resources")
    config_path = os.path.join(dual_arm_config, "dual_arm_panda_moveit_config", "config")
    config_path1 = os.path.join(dual_arm_config, "panda_moveit_config", "config")

    # Build robot_description using xacro
    robot_description_content = Command([
        "xacro ",
        os.path.join(config_path, "panda.urdf.xacro")
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Load SRDF file content
    srdf_path = os.path.join(config_path1, "panda_arm_hand.srdf")
    with open(srdf_path, "r") as srdf_file:
        srdf_text = srdf_file.read()

    # Build parameters dictionary
    robot_parameters = {
        "robot_description": robot_description,
        "robot_description_semantic": srdf_text,
    }

    return LaunchDescription([
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen"
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_parameters]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(config_path, "../launch/moveit.rviz")]
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[robot_parameters]
        )
    ])

