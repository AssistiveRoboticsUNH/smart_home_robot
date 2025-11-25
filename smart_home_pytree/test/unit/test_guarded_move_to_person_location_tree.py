
## functions that start with test will be run 

import pytest

import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
import py_trees
from smart_home_pytree.trees.guarded_move_to_person_location import GuardedMoveToPersonLocationTree

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
    print('\nsetup_function()')
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.storage.clear()

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
    
    move_tree = GuardedMoveToPersonLocationTree(
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
    
    move_tree = GuardedMoveToPersonLocationTree(
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

    move_tree = GuardedMoveToPersonLocationTree(
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
    
    # Cleanup
    executor.shutdown(); executor_thread.join()
    mock_nav_server.destroy_node()
    mock_dock_server.destroy_node()
    mock_undock_server.destroy_node()
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

    move_tree = GuardedMoveToPersonLocationTree(
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
    
    # Cleanup
    # Cleanup
    executor.shutdown(); executor_thread.join()
    mock_nav_server.destroy_node()
    move_tree.cleanup()
    
def test_guarded_move_aborts_when_person_moves():
    """Verify that GuardedMoveToPersonLocationTree stops immediately if the person moves mid-navigation."""
    
    # Mock navigation action server
    mock_nav_server = BaseMockActionServer(
        action_name='/navigate_to_pose',
        action_type=NavigateToPose,
        result_cls=NavigateToPose.Result,
        succeed=True,        # Would normally succeed if uninterrupted
        wait_time=10.0        # Long enough for guard to interrupt before success
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

    # Initial conditions: robot and person start in different locations
    robot_interface.state.update('person_location', 'living_room')
    robot_interface.state.update('robot_location', 'kitchen')

    move_tree = GuardedMoveToPersonLocationTree(
        node_name="move_to_person_guard_abort",
        robot_interface=robot_interface,
        num_attempts=1  # No retries
    )
    move_tree.setup()

    # Simulate person moving away while the robot is navigating
    def on_moving_trigger(action_name):
        print("[Callback] Navigation started, person moves away!")
        robot_interface.state.update('person_location', 'bedroom')  # triggers guard to abort
        # Do NOT update robot_location — the robot never finishes moving

    mock_nav_server.set_on_trigger(on_moving_trigger)

    # Run the tree
    runner_thread = threading.Thread(target=move_tree.run_until_done, daemon=True)
    runner_thread.start()
    runner_thread.join(timeout=50)

    for _ in range(20):
        if mock_nav_server.result_status is not None:
            break
        time.sleep(0.1)
    
    if runner_thread.is_alive():
        pytest.fail("Tree did not finish — possible hang")

    # === Assertions ===
    assert mock_nav_server.result_status == "canceled", "Expected navigation to be cancelled"
    assert mock_nav_server.triggered, "Expected navigation to start"
    assert move_tree.final_status == py_trees.common.Status.FAILURE, "Tree should fail when guard condition breaks"
    assert mock_nav_server.completed==False, "Expected navigation to not be completed"
    robot_final_loc = robot_interface.state.get('robot_location')
    person_final_loc = robot_interface.state.get('person_location')

    # The robot never caught up to the new person location
    assert robot_final_loc != person_final_loc, (
        f"Expected robot to stop early (different locations), "
        f"got robot={robot_final_loc}, person={person_final_loc}"
    )

    # Cleanup
    executor.shutdown(); executor_thread.join()
    mock_nav_server.destroy_node()
    mock_dock_server.destroy_node()
    mock_undock_server.destroy_node()
    move_tree.cleanup()



# ~/smart_home_pytree_ws/src/smart_home_pytree: run  python3 -m  pytest test/unit/test_guarded_move_to_person_location_tree.py -vv
# python3 -m pytest test/unit/test_guarded_move_to_person_location_tree.py::test_guarded_move_aborts_when_person_moves -vv

