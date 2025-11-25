
## functions that start with test will be run 

import pytest

import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
import py_trees
from smart_home_pytree.trees.move_to_person_location import MoveToPersonLocationTree

# Replace these with your actual action types
from shr_msgs.action import DockingRequest
from nav2_msgs.action import NavigateToPose
from ..mock.mock_action_server import BaseMockActionServer
import time
from smart_home_pytree.robot_interface import RobotInterface
import threading

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


def test_move_to_person_success_diff_loc():
    
    # Setup mock navigation server
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
    robot_interface.state.update('person_location', 'living_room')
    robot_interface.state.update('robot_location', 'kitchen')
    
    move_tree = MoveToPersonLocationTree(
        node_name="move_to_person_test",
        robot_interface=robot_interface,
        num_attempts=3
    )
    move_tree.setup()

    def on_moving_trigger(action_name):
        print(f"[Callback] Moving triggered ({action_name}), setting new location")
        robot_interface.state.update('robot_location', 'living_room')

    mock_nav_server.set_on_trigger(on_moving_trigger)
    
    
    runner_thread = threading.Thread(target=move_tree.run_until_done, daemon=True)
    runner_thread.start()
    runner_thread.join(timeout=100)
    
    if runner_thread.is_alive():
        pytest.fail("Tree did not finish — possible hang")
        
    assert move_tree.final_status == py_trees.common.Status.SUCCESS
    assert mock_nav_server.triggered
    assert not mock_undock_server.triggered
    
    robot_final_loc = robot_interface.state.get('robot_location')
    person_final_loc = robot_interface.state.get('person_location')
    assert robot_final_loc == person_final_loc, f"Expected robot and person to be in same location, got robot={robot_final_loc}, person={person_final_loc}"

    # Cleanup
    executor.shutdown(); executor_thread.join()
    mock_nav_server.destroy_node()
    mock_dock_server.destroy_node()
    mock_undock_server.destroy_node()
    move_tree.cleanup()
        
        
def test_move_to_person_success_same_loc():
    
    # Setup mock navigation server
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
    robot_interface.state.update('person_location', 'living_room')
    robot_interface.state.update('robot_location', 'living_room')
    
    move_tree = MoveToPersonLocationTree(
        node_name="move_to_person_test",
        robot_interface=robot_interface,
        num_attempts=3
    )
    move_tree.setup()

    
    runner_thread = threading.Thread(target=move_tree.run_until_done, daemon=True)
    runner_thread.start()
    runner_thread.join(timeout=100)
    
    if runner_thread.is_alive():
        pytest.fail("Tree did not finish — possible hang")
        
    assert move_tree.final_status == py_trees.common.Status.SUCCESS
    assert not mock_nav_server.triggered
    assert not mock_undock_server.triggered

    robot_final_loc = robot_interface.state.get('robot_location')
    person_final_loc = robot_interface.state.get('person_location')
    assert robot_final_loc == person_final_loc, f"Expected robot and person to be in same location, got robot={robot_final_loc}, person={person_final_loc}"

    # Cleanup
    executor.shutdown(); executor_thread.join()
    mock_nav_server.destroy_node()
    mock_dock_server.destroy_node()
    mock_undock_server.destroy_node()
    move_tree.cleanup()
        

def test_move_to_person_nav_fails():
    # Setup mock navigation server to fail
    mock_nav_server = BaseMockActionServer(
        action_name='/navigate_to_pose',
        action_type=NavigateToPose,
        result_cls=NavigateToPose.Result,
        succeed=False,
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

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    robot_interface.state.update('person_location', 'living_room')
    robot_interface.state.update('robot_location', 'kitchen')

    move_tree = MoveToPersonLocationTree(
        node_name="move_to_person_test_nav_fail",
        robot_interface=robot_interface,
        num_attempts=3
    )
    move_tree.setup()

    runner_thread = threading.Thread(target=move_tree.run_until_done, daemon=True)
    runner_thread.start()
    runner_thread.join(timeout=50)

    if runner_thread.is_alive():
        pytest.fail("Tree did not finish — possible hang")

    # Assertions
    assert move_tree.final_status == py_trees.common.Status.FAILURE
    robot_final_loc = robot_interface.state.get('robot_location')
    person_final_loc = robot_interface.state.get('person_location')
    assert robot_final_loc != person_final_loc
    
    executor.shutdown(); executor_thread.join()
    mock_nav_server.destroy_node(); mock_undock_server.destroy_node(); mock_undock_server.destroy_node()
    move_tree.cleanup()

def test_move_to_person_person_moves():
    mock_nav_server = BaseMockActionServer(
        action_name='/navigate_to_pose',
        action_type=NavigateToPose,
        result_cls=NavigateToPose.Result,
        succeed=True,
        wait_time=0.5
    )

    executor = MultiThreadedExecutor()
    executor.add_node(mock_nav_server)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    robot_interface.state.update('person_location', 'living_room')
    robot_interface.state.update('robot_location', 'kitchen')

    move_tree = MoveToPersonLocationTree(
        node_name="move_to_person_test_person_moves",
        robot_interface=robot_interface,
        num_attempts=1 
    )
    move_tree.setup()

    # Person moves during movement
    def on_moving_trigger(action_name):
        robot_interface.state.update('person_location', 'bedroom')
        robot_interface.state.update('robot_location', 'living_room')

    mock_nav_server.set_on_trigger(on_moving_trigger)

    runner_thread = threading.Thread(target=move_tree.run_until_done, daemon=True)
    runner_thread.start()
    runner_thread.join(timeout=50)

    if runner_thread.is_alive():
        pytest.fail("Tree did not finish — possible hang")

    assert move_tree.final_status == py_trees.common.Status.FAILURE
    robot_final_loc = robot_interface.state.get('robot_location')
    person_final_loc = robot_interface.state.get('person_location')
    assert robot_final_loc != person_final_loc

    executor.shutdown(); executor_thread.join()
    mock_nav_server.destroy_node()
    move_tree.cleanup()

# if __name__ == "__main__":
#     test_charge_robot_tree_2_success()

# ~/smart_home_pytree_ws/src/smart_home_pytree: run  python3 -m  pytest test/unit/test_move_to_person_location_tree.py -vv
