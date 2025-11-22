import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Paths
    world_file = os.path.join(
        get_package_share_directory('tortoisebot_gazebo'),
        'worlds',
        'empty_world.world'
    )
    # world_file = 'empty1.sdf'
    
    urdf_file = os.path.join(
        get_package_share_directory('tortoisebot_gazebo'),
        'urdf',
        'tortoisebot.urdf'
    )

    bridge_yaml = os.path.join(
        get_package_share_directory('tortoisebot_gazebo'),
        'config',
        'bridge.yaml'
    )

    pkg_ros_gz_sim=get_package_share_directory('ros_gz_sim')

    robot_description = {"robot_description": open(urdf_file).read()}


     # 1️⃣ Start Gazebo
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        PathJoinSubstitution([pkg_ros_gz_sim,'launch','gz_sim.launch.py'])
        ),
        
        launch_arguments={'gz_args': world_file,'on_exit_shutdown':'true'}.items(),

    )

    # 2️⃣ Publish robot state (URDF and TF)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen"
    )

    # 3️⃣ Spawn robot entity in Gazebo
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'tortoisebot','-z','1'],
        output='screen'
    )
    
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_cmd_vel',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )

    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_lidar',
        parameters=[{'config_file': bridge_yaml}],
        # arguments=[
        #     '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
        # ],
        output='screen'
    )

   

    return LaunchDescription([
        start_gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        bridge_cmd_vel, 
        bridge_lidar
        # start_display
    ])
