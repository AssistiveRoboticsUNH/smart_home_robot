from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    ld = LaunchDescription()

    smartthings_node_plug = Node(
        package='smart_home_pytree',
        executable='smart_plug',
        output='screen'
    )

    discord_logger = Node(
        package='simple_logger',
        executable='simple_logger_discord',
        output='screen'
    )

    display_node = Node(
        package='shr_display',
        executable='display_node',
        output='screen'
    )


    ld.add_action(smartthings_node_plug)

    ld.add_action(discord_logger)
    ld.add_action(display_node)

    return ld
