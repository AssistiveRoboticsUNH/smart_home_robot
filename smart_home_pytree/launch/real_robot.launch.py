from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    ld = LaunchDescription()

    charge_monitor = Node(
        package='smart_home_pytree',
        executable='charge_monitor',
        name='charge_monitor',
        output='screen'
    )

    discord_logger = Node(
        package='simple_logger',
        executable='simple_logger_discord',
        name='logger_discord',
        output='screen'
    )

    display_node = Node(
        package='shr_display',
        executable='display_node',
        name='display_node',
        output='screen'
    )

    play_video_node_cmd = Node(
        package='smart_home_pytree',
        executable='play_video',
        name='play_video',
        output='screen')
    
    human_voice_interaction = Node(
        package='shr_human_interaction',
        executable='human_interaction_node',
        name='human_interaction_node',
        output='screen')

    ld.add_action(charge_monitor)
    ld.add_action(discord_logger)
    ld.add_action(display_node)
    ld.add_action(play_video_node_cmd)
    ld.add_action(human_voice_interaction)

    return ld
