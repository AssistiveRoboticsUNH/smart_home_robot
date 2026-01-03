import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # --- Simulator Launch ---
    tb3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "tb3_simulation_launch.py",
            )
        ),
        launch_arguments={"headless": "False"}.items(),
    )

    # --- Define Nodes ---
    display_node = Node(
        package="shr_display",
        executable="display_node",
        name="display_node",
        output="screen",
    )

    play_video_node_cmd = Node(
        package="smart_home_pytree",
        executable="play_video",
        name="play_video",
        output="screen",
    )

    human_voice_interaction = Node(
        package="shr_human_interaction",
        executable="human_interaction_node",
        name="human_interaction_node",
        output="screen",
    )

    mock_dock_undock_cmd = Node(
        package="smart_home_pytree",
        executable="mock_dock_undock",
        name="mock_dock_undock",
        output="screen",
    )

    # waits for subscriber
    # ros2 topic pub --once -w 1 /charging std_msgs/msg/Bool "{data: True}"
    # --qos-durability transient_local --qos-reliability reliable
    charging_pub = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            "-w",
            "1",
            "/charging",
            "std_msgs/msg/Bool",
            "{data: True}",
            "--qos-durability",
            "transient_local",
            "--qos-reliability",
            "reliable",
        ],
        output="screen",
    )

    # --- Return one LaunchDescription with everything included ---
    return LaunchDescription(
        [
            tb3_launch,
            display_node,
            # play_video_node_cmd,
            human_voice_interaction,
            mock_dock_undock_cmd,
            charging_pub,
        ]
    )
