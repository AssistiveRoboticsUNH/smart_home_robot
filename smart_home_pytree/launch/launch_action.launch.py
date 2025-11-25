from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from smart_home_pytree.trees.move_to_tree import MoveToLocationTree,required_actions_

from launch.conditions import IfCondition

def generate_launch_description():
    # --- Launch arguments ---
    run_actions = LaunchConfiguration('run_actions', default='true')
 
    # Get required action nodes (these are already Node objects), 
    action_servers = required_actions_()
    
    # Create nodes with conditions
    action_nodes = []
    for package, executables in action_servers.items():
        for executable in executables:
            action_nodes.append(
                Node(
                    package=package,
                    executable=executable,
                    name=executable,
                    output='screen',
                    emulate_tty=True,
                    condition=IfCondition(run_actions)
                )
            )


    # --- Launch Description ---
    return LaunchDescription([
        DeclareLaunchArgument('run_actions', default_value='true'),
        *action_nodes,
    ])


## to run
# ros2 launch smart_home_pytree move_to_tree_with_sim.launch.py \
#     x:=-1.2 y:=0.8 qz:=0.707 qw:=0.707

# ros2 launch smart_home_pytree move_to_tree_with_sim.launch.py run_simulator:=true run_actions:=true
