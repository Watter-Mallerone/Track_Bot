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

    wheel_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wheel_control",
            "--controller-manager", "/controller_manager",
            "--param-file", PathJoinSubstitution([
                get_package_share_directory("track_bot_ctrl"),
                "config", "ctrl.yaml"      # ← 여기에 너의 controllers.yaml
            ]),
        ],
        output="screen",
    )

    # --- [추가] 키보드 teleop 노드 ---
    wheel_keyboard = Node(
        package="track_bot_ctrl",                # ← 본인 패키지명
        executable="keyboard_ctrl.py",         # ← scripts/ 에 넣은 파일
        name="keyboard_ctrl",
        output="screen"
    )

    # Run the node
    return LaunchDescription([
        joint_state_broadcaster_spawner,
        wheel_velocity_controller_spawner,
        wheel_keyboard
    ])
