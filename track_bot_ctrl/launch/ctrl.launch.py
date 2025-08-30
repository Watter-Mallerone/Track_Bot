from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    wheel_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wheel_control",
            "--controller-manager", "/controller_manager",
            "--param-file", PathJoinSubstitution([
                get_package_share_directory("track_bot_ctrl"),
                "config", "ctrl.yaml"
            ]),
        ],
        output="screen",
    )

    wheel_keyboard = Node(
        package="track_bot_ctrl",
        executable="keyboard_ctrl.py",
        name="keyboard_ctrl",
        output="screen"
    )

    # Run the node
    return LaunchDescription([
        # joint_state_broadcaster_spawner,
        wheel_control_spawner,
        wheel_keyboard
    ])
