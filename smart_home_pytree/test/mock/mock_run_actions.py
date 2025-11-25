from nav2_msgs.action import NavigateToPose
import rclpy

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from shr_msgs.action import DockingRequest
import threading
from mock_action_server import BaseMockActionServer
from smart_home_pytree.robot_interface import RobotInterface

# class DelayedMockActionServer(BaseMockActionServer):
#     def execute_callback(self, goal_handle):
#         self.get_logger().info('Delaying response...')
#         self.create_timer(2.0, lambda: None)  # simulate delay
#         return super().execute_callback(goal_handle)
    
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
    
    mock_nav_server = BaseMockActionServer(
        action_name='/navigate_to_pose',
        action_type=NavigateToPose,
        result_cls=NavigateToPose.Result,
        succeed=True,
        wait_time=0.5
    )
    
    mock_dock_server = BaseMockActionServer(
        action_name='/docking',
        action_type=DockingRequest,
        result_cls=DockingRequest.Result,
        succeed=True,
        wait_time=1.0 
    )

    mock_undock_server = BaseMockActionServer(
        action_name='/undocking',
        action_type=DockingRequest,
        result_cls=DockingRequest.Result,
        succeed=True,
        wait_time=1.0 
    )
    
    executor = MultiThreadedExecutor()
    executor.add_node(mock_nav_server)
    executor.add_node(mock_dock_server)
    executor.add_node(mock_undock_server)
    
    robot_interface.state.update('charging', False)
    robot_interface.state.update('person_location', 'living_room')
    robot_interface.state.update('robot_location', 'kitchen')

    def on_moving_trigger(action_name):
        print(f"[Callback] Moving triggered ({action_name}), setting new location")
        robot_interface.state.update('robot_location', 'living_room')

    def on_docking_trigger(action_name):
        print(f"[Callback] Moving triggered ({action_name}), setting new location")
        robot_interface.state.update('charging', True)
        
    def on_undocking_trigger(action_name):
        print(f"[Callback] Moving triggered ({action_name}), setting new location")
        robot_interface.state.update('charging', False)
    
    mock_nav_server.set_on_trigger(on_moving_trigger)
    mock_dock_server.set_on_trigger(on_docking_trigger)
    mock_undock_server.set_on_trigger(on_undocking_trigger)
    # # Function to spin executor
    # def spin_executor():
    #     executor.spin()

    # # Start spinning in a separate thread
    # executor_thread = threading.Thread(target=spin_executor, daemon=True)
    # executor_thread.start()
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # --- Cleanup ---
        executor.shutdown(); 
        # executor_thread.join()
        mock_nav_server.destroy_node()
        mock_dock_server.destroy_node()
        mock_undock_server.destroy_node()
        
        if rclpy_initialized_here:
            rclpy.shutdown()


if __name__ == "__main__":
    main()
