import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_name = 'track_bot_sim'

    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'model.urdf.xacro')
    rviz_file = os.path.join(get_package_share_directory(pkg_name), 'rviz', 'track_bot.rviz')
    world_file = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'test_world.world')

    robot_description_raw = xacro.process_file(xacro_file).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_file}.items(),
        )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'track_bot'],
        output='screen')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    odom_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        arguments = ['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    world_publisher = Node(
        package=pkg_name,
        executable='world_publisher',
        name='world_publisher',
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        spawn_entity,
        # odom_publisher,
        world_publisher,
        rviz2
    ])
