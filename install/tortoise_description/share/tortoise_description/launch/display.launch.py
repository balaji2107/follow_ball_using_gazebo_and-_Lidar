import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import TextSubstitution
from pathlib import Path

def generate_launch_description():

    # use_sim_time = True

    urdf_file = os.path.join(
        get_package_share_directory('tortoisebot_gazebo'),'urdf','tortoisebot.urdf'
    )

    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    # robot_description_config = TextSubstitution(text=Path(urdf_file).read_text())

    robot_description = {"robot_description": robot_desc}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen"
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
