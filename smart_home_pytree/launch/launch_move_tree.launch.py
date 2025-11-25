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
    run_simulator = LaunchConfiguration('run_simulator', default='false')
    run_actions = LaunchConfiguration('run_actions', default='false')
    run_continuous = LaunchConfiguration('run_continuous', default='false')
    location = LaunchConfiguration('location', default='living_room')
   
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
            
    # --- Simulator Launch ---
    tb3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'tb3_simulation_launch.py'
            )
        ),
        launch_arguments={'headless': 'False'}.items(),
        condition=IfCondition(run_simulator)  # <-- use IfCondition, not lambda
    )

    # --- Move-To Tree Node ---
    move_to_tree_node = Node(
        package='smart_home_pytree',
        executable='move_to_tree',
        name='move_to_location_tree',
        output='screen',
        arguments=[
            '--run_simulator', run_simulator,
            '--run_actions', run_actions,
            '--run_continuous', run_continuous,
            '--location', location,
        ],
    )

    # --- Launch Description ---
    return LaunchDescription([
        DeclareLaunchArgument('run_simulator', default_value='false'),
        DeclareLaunchArgument('run_actions', default_value='false'),
        DeclareLaunchArgument('run_continuous', default_value='false'),
        DeclareLaunchArgument('location', default_value='location'),
        tb3_launch,
        *action_nodes,
        move_to_tree_node
    ])


## to run
# ros2 launch smart_home_pytree move_to_tree_with_sim.launch.py location:=living_room

# ros2 launch smart_home_pytree launch_move_tree.launch.py run_simulator:=true run_actions:=true run_continuous:=true
