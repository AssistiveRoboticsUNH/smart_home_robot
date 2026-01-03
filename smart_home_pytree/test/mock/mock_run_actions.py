import rclpy
from geometry_msgs.msg import PoseStamped
from mock_action_server import BaseMockActionServer
from nav2_msgs.action import NavigateToPose
from rclpy.executors import MultiThreadedExecutor
from smart_home_pytree.robot_interface import RobotInterface

from shr_msgs.action import PlayVideoRequest

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

    mock_nav_server = BaseMockActionServer(
        action_name="/navigate_to_pose",
        action_type=NavigateToPose,
        result_cls=NavigateToPose.Result,
        succeed=True,
        wait_time=0.5,
    )

    mock_dock_server = BaseMockActionServer(
        action_name="/dock_robot",
        action_type=DockRobot,
        result_cls=DockRobot.Result,
        succeed=True,
        wait_time=1.0,
    )

    mock_undock_server = BaseMockActionServer(
        action_name="/undock_robot",
        action_type=UndockRobot,
        result_cls=UndockRobot.Result,
        succeed=True,
        wait_time=1.0,
    )

    mock_play_video = BaseMockActionServer(
        action_name="/play_video",
        action_type=PlayVideoRequest,
        result_cls=PlayVideoRequest.Result,
        succeed=True,
        wait_time=5.0,
    )

    executor = MultiThreadedExecutor()
    executor.add_node(mock_nav_server)
    executor.add_node(mock_dock_server)
    executor.add_node(mock_undock_server)
    executor.add_node(mock_play_video)

    robot_interface.state.update("charging", False)
    robot_interface.state.update("person_location", "living_room")
    robot_interface.state.update("robot_location", "kitchen")

    def on_moving_trigger(action_name):
        print(f"[Callback] Moving triggered ({action_name}), setting new location")
        robot_interface.state.update("robot_location", "living_room")

    def on_docking_trigger(action_name):
        print(f"[Callback] Moving triggered ({action_name}), setting new location")
        robot_interface.state.update("charging", True)

    def on_undocking_trigger(action_name):
        print(f"[Callback] Moving triggered ({action_name}), setting new location")
        robot_interface.state.update("charging", False)

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
        executor.shutdown()
        # executor_thread.join()
        mock_nav_server.destroy_node()
        mock_dock_server.destroy_node()
        mock_undock_server.destroy_node()
        mock_play_video.destroy_node()

        if rclpy_initialized_here:
            rclpy.shutdown()


if __name__ == "__main__":
    main()
