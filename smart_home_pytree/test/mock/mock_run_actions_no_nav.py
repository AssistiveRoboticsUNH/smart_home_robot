from nav2_msgs.action import NavigateToPose
import rclpy

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
import threading
from mock_action_server import BaseMockActionServer
from smart_home_pytree.robot_interface import RobotInterface

try:
    # ROS 2 Jazzy / Rolling (Standard)
    from nav2_msgs.action import DockRobot, UndockRobot
except ImportError:
    # ROS 2 Humble (Requires 'ros-humble-opennav-docking-msgs')
    from opennav_docking_msgs.action import DockRobot, UndockRobot

    
def main():
   
    if not rclpy.ok():
        ## just for safety
        try:
            rclpy.init(args=None)
            rclpy_initialized_here = True
        except RuntimeError:
            # ROS2 already initialized somewhere else
            rclpy_initialized_here = False
    else:
        rclpy_initialized_here = False
    
    robot_interface = RobotInterface()
    
    mock_dock_server = BaseMockActionServer(
        action_name='/dock_robot',
        action_type=DockRobot,
        result_cls=DockRobot.Result,
        succeed=True,
        wait_time=10.0 
    )

    mock_undock_server = BaseMockActionServer(
        action_name='/undock_robot',
        action_type=UndockRobot,
        result_cls=UndockRobot.Result,
        succeed=True,
        wait_time=10.0 
    )
    
    executor = MultiThreadedExecutor()
    executor.add_node(mock_dock_server)
    executor.add_node(mock_undock_server)
    
    robot_interface.state.update('charging', False)
    robot_interface.state.update('person_location', 'living_room')
    robot_interface.state.update('robot_location', 'kitchen')

   
    def on_docking_trigger(action_name):
        print(f"[Callback] Moving triggered ({action_name}), setting new location")
        robot_interface.state.update('charging', True)
        
    def on_undocking_trigger(action_name):
        print(f"[Callback] Moving triggered ({action_name}), setting new location")
        robot_interface.state.update('charging', False)
    
    mock_dock_server.set_on_trigger(on_docking_trigger)
    mock_undock_server.set_on_trigger(on_undocking_trigger)
    
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # --- Cleanup ---
        executor.shutdown(); 
        # executor_thread.join()
        mock_dock_server.destroy_node()
        mock_undock_server.destroy_node()
        
        if rclpy_initialized_here:
            rclpy.shutdown()


if __name__ == "__main__":
    main()
