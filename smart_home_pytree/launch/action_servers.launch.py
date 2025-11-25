import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # docking_launch_file_path = os.path.join(
    #     get_package_share_directory('shr_docking'),
    #     'launch',
    #     'docking_action_servers.launch.py'
    # )

    # docking_launch_cmd = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(docking_launch_file_path)
    #     )
    
    play_video_node_cmd = Node(
        package='smart_home_pytree',
        executable='play_video',
        name='play_video',
        output='screen')

    # make_call_node_cmd = Node(
    #     package='smart_home_pytree',
    #     executable='make_call_action',
    #     name='make_call_action',
    #     output='screen')

    undock_cmd = Node(
        package='smart_home_pytree',
        executable='undocking',
        name='undocking',
        output='screen')

    dock_cmd = Node(
        package='smart_home_pytree',
        executable='docking',
        name='docking',
        output='screen')

    # question_response_action_cmd = Node(
    #     package='convros_bot',
    #     executable='question_response_action',
    #     name='question_response_action',
    #     output='screen'
    # )
    
    ld = LaunchDescription()

    ld.add_action(play_video_node_cmd)
    # ld.add_action(make_call_node_cmd)
    # ld.add_action(question_response_action_cmd)

    ld.add_action(undock_cmd)
    ld.add_action(dock_cmd)

    return ld
