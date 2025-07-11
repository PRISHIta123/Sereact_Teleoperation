from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    dual_arm_config = "/home/prishita/sereact_ws/src/moveit_resources/dual_arm_panda_moveit_config/config"

    # Load URDF
    robot_description_content = Command([
        "xacro ",
        os.path.join(dual_arm_config, "panda.urdf.xacro")
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Load SRDF properly
    srdf_path = os.path.join(dual_arm_config, "dual_panda.srdf")
    with open(srdf_path, "r") as srdf_file:
        srdf_text = srdf_file.read()

    # ✅ THIS is critical — wrap the string in a ParameterValue:
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(srdf_text, value_type=str)
    }

    robot_parameters = {}
    robot_parameters.update(robot_description)
    robot_parameters.update(robot_description_semantic)

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
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[robot_parameters]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(dual_arm_config, "../launch/moveit.rviz")],
            parameters=[robot_parameters]
        )
    ])

