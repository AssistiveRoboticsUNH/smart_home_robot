import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
import py_trees
from smart_home_pytree.trees.move_to_tree import MoveToLocationTree

# Replace these with your actual action types
from shr_msgs.action import DockingRequest
from nav2_msgs.action import NavigateToPose
from ..mock.mock_action_server import BaseMockActionServer
import time
from smart_home_pytree.robot_interface import RobotInterface

### 

# CAREFUL NOT TO RUN TOPICS AND ACTIONS IN THE BACKGROUND 

###
# --- Global references for teardown ---
robot_interface = None

def setup_module(module):
    print('\nsetup_module()')
    
    # global executor, executor_thread, mock_nav_server, mock_undock_server
    global robot_interface
    
    if not rclpy.ok():
        rclpy.init(args=None)
    
    
    
    robot_interface = RobotInterface()
    
def teardown_module(module):
    print('teardown_module()')
    
    global robot_interface
    
    robot_interface.shutdown()
    rclpy.shutdown()
    
def setup_function(function):
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.storage.clear()
    print('\nsetup_function()')

def teardown_function(function):
    print('\nteardown_function()')

        
def test_move_to_location_tree_success():
    print('-  test_move_to_location_tree_success()')
    """Test that MoveToLocationTree returns SUCCESS when navigation and Undock succeeds."""
    # robot interface
    mock_nav_server = BaseMockActionServer(
        action_name='/navigate_to_pose',
        action_type=NavigateToPose,
        result_cls=NavigateToPose.Result,
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
    executor.add_node(mock_undock_server)
    
    # Function to spin executor
    def spin_executor():
        executor.spin()

    # Start spinning in a separate thread
    executor_thread = threading.Thread(target=spin_executor, daemon=True)
    executor_thread.start()
    
    print("start wait")
    time.sleep(2.0)  # give server time to start
    print("stop wait")
    
    robot_interface.state.update('charging', False)
    
    # Create the MoveTo tree
    move_tree = MoveToLocationTree(
        node_name="test_move_to_tree",
        location="kitchen",
        location_key="person_location",
        robot_interface = robot_interface,
    )

    move_tree.setup()

    # Run the tree once until done and get final status
    final_status = move_tree.run_until_done()
    print("final_status", final_status)

    assert final_status == py_trees.common.Status.SUCCESS, f"Expected SUCCESS but got {final_status}"

    # shutdown
    executor.shutdown();executor_thread.join()
    mock_nav_server.destroy_node();mock_undock_server.destroy_node()
    move_tree.cleanup()
    
def test_move_to_location_tree_navigation_fail():
    print('-  test_move_to_location_tree_navigation_fail()')
    """Navigation fails → FAILURE"""
    # rclpy.init()
    if not rclpy.ok():
        rclpy.init(args=None)

    mock_nav_server = BaseMockActionServer(
        action_name='/navigate_to_pose',
        action_type=NavigateToPose,
        result_cls=NavigateToPose.Result,
        succeed=False,  # NAV FAIL
        wait_time=0.5
    )
    mock_undock_server = BaseMockActionServer(
        action_name='/undocking',
        action_type=DockingRequest,
        result_cls=DockingRequest.Result,
        succeed=True,
        wait_time=0.5
    )

    executor = MultiThreadedExecutor()
    executor.add_node(mock_nav_server)
    executor.add_node(mock_undock_server)

    # Function to spin executor
    def spin_executor():
        executor.spin()
       
    # Start spinning in a separate thread
    executor_thread = threading.Thread(target=spin_executor, daemon=True)
    executor_thread.start() 
    
    print("start wait")
    time.sleep(2.0)  # give server time to start
    print("stop wait")
    
    robot_interface.state.update('charging', False)
 
    move_tree = MoveToLocationTree(
        node_name="test_nav_fail",
        location="kitchen",
        location_key="person_location",
        robot_interface = robot_interface,
    )

    move_tree.setup()

    result = move_tree.run_until_done()
    assert result == py_trees.common.Status.FAILURE, f"Expected FAILURE but got {result}"

    executor.shutdown(); executor_thread.join()
    mock_nav_server.destroy_node(); mock_undock_server.destroy_node()
    move_tree.cleanup()

def test_move_to_location_tree_charging_undock_fail():
    print('-  test_move_to_location_tree_charging_undock_fail()')
    """Robot charging but Undock fails → FAILURE"""
    
    if not rclpy.ok():
        rclpy.init(args=None)
            
    mock_nav_server = BaseMockActionServer(
        action_name='/navigate_to_pose',
        action_type=NavigateToPose,
        result_cls=NavigateToPose.Result,
        succeed=True,
        wait_time=0.5
    )
    mock_undock_server = BaseMockActionServer(
        action_name='/undocking',
        action_type=DockingRequest,
        result_cls=DockingRequest.Result,
        succeed=False,  # UNDOCK FAIL
        wait_time=0.5
    )

    executor = MultiThreadedExecutor()
    executor.add_node(mock_nav_server)
    executor.add_node(mock_undock_server)

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    time.sleep(1.0)
    
    robot_interface.state.update('charging', True)

    move_tree = MoveToLocationTree(
        node_name="test_undock_fail",
        location="kitchen",
        location_key="person_location",
        robot_interface=robot_interface,
    )

    move_tree.setup()

    result = move_tree.run_until_done()
    print("result: ", result)
    assert result == py_trees.common.Status.FAILURE, f"Expected FAILURE but got {result}"

    executor.shutdown(); thread.join()
    mock_nav_server.destroy_node(); mock_undock_server.destroy_node()
    move_tree.cleanup()

def test_move_to_location_tree_uncharging_undock_fail():
    print('-  test_move_to_location_tree_uncharging_undock_fail()')
    """Robot not charging but Undock fails → SUCCESS"""
    """If it fails it means undocking was run when it shouldnt have"""
    # rclpy.init()
    if not rclpy.ok():
        rclpy.init(args=None)

    mock_nav_server = BaseMockActionServer(
        action_name='/navigate_to_pose',
        action_type=NavigateToPose,
        result_cls=NavigateToPose.Result,
        succeed=True,
        wait_time=0.5
    )
    mock_undock_server = BaseMockActionServer(
        action_name='/undocking',
        action_type=DockingRequest,
        result_cls=DockingRequest.Result,
        succeed=False,  # UNDOCK FAIL
        wait_time=0.5
    )

    executor = MultiThreadedExecutor()
    executor.add_node(mock_nav_server)
    executor.add_node(mock_undock_server)

    # Function to spin executor
    def spin_executor():
        executor.spin()

    # Start spinning in a separate thread
    executor_thread = threading.Thread(target=spin_executor, daemon=True)
    executor_thread.start()
    
    print("start wait")
    time.sleep(2.0)  # give server time to start
    print("stop wait")
    
    robot_interface.state.update('charging', False)

    move_tree = MoveToLocationTree(
        node_name="test_undock_fail",
        location="kitchen",
        location_key="person_location",
        robot_interface=robot_interface,
    )
    
    move_tree.setup()

    result = move_tree.run_until_done()
    print("result: ", result)
    assert result == py_trees.common.Status.SUCCESS, f"Expected SUCCESS but got {result}"

    executor.shutdown(); executor_thread.join()
    mock_nav_server.destroy_node(); mock_undock_server.destroy_node()
    move_tree.cleanup()
    
# if __name__ == "__main__":
#     # test_move_to_location_tree_success()
#     test_move_to_location_tree_navigation_fail()
    
    
# to run : python3 -m test.unit.test_move_to_tree in /home/olagh48652/smart_home_pytree_ws/src/smart_home_pytree
# all tests
# python3 -m  pytest test/unit/test_move_to_tree.py -vv
# colcon test --packages-select smart_home_pytree

## doesnt work currenly
# pytest test/unit -vv from smart_home_pytree_ws/src/smart_home_pytree


# python3 -m pytest -s -v test_pytest.py 