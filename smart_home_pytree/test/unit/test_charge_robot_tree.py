
## functions that start with test will be run 

import pytest

import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
import py_trees
from smart_home_pytree.trees.charge_robot_tree import ChargeRobotTree

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
    
    """Reset the py_trees blackboard before each test automatically."""
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.storage.clear()

def teardown_function(function):
    print('\nteardown_function()')

        
def test_charge_robot_tree_success():
    print('-  test_charge_robot_tree_success()')
    """Test that ChargeRobotTree returns SUCCESS when not chargign but then switches navigation and dock succeeds."""
    
    # robot interface
    mock_nav_server = BaseMockActionServer(
        action_name='/navigate_to_pose',
        action_type=NavigateToPose,
        result_cls=NavigateToPose.Result,
        succeed=True,
        wait_time=1.0 
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
    
    # Create the chargerobot tree
    charge_robot_tree = ChargeRobotTree(
        node_name="test_charge_robot_tree",
        num_attempts=5,
        robot_interface = robot_interface,
    )

    charge_robot_tree.setup()

    # Register callback to simulate charger connection during docking
    def on_docking_trigger(action_name):
        print(f"[Callback] Docking triggered ({action_name}), setting charging=True")
        robot_interface.state.update('charging', True)

    mock_dock_server.set_on_trigger(on_docking_trigger)
    
    runner_thread = threading.Thread(target=charge_robot_tree.run_until_done, daemon=True)
    runner_thread.start()
    runner_thread.join(timeout=50)
        
    if runner_thread.is_alive():
        pytest.fail("Tree did not finish within 50 s — possible hang or infinite loop.")
  
    final_status =  charge_robot_tree.final_status
    print("final_status:", final_status)

    # --- Assertions ---
    assert mock_nav_server.triggered, "Expected navigation to be triggered"
    assert mock_dock_server.triggered, "Expected docking to be triggered"
    assert mock_nav_server.complete_time and mock_dock_server.trigger_time, "Missing timing info"
    assert mock_nav_server.complete_time < mock_dock_server.trigger_time, "Docking should start after navigation"
    assert final_status == py_trees.common.Status.SUCCESS, f"Expected SUCCESS but got {final_status}"
    assert robot_interface.state.get('charging') is True, "Robot should end charging"
    assert mock_dock_server.result_status == "succeeded"
    
    # Cleanup
    executor.shutdown()
    executor_thread.join()
    mock_nav_server.destroy_node()
    mock_dock_server.destroy_node()
    mock_undock_server.destroy_node()
    charge_robot_tree.cleanup()


def test_charge_robot_tree_charging_true():
    """Test that if robot is already charging, nothing triggers."""
    
    # Mock servers
    mock_nav_server = BaseMockActionServer(
        action_name='/navigate_to_pose',
        action_type=NavigateToPose,
        result_cls=NavigateToPose.Result,
        succeed=True,
        wait_time=1.0
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
    
    # Spin executor in background
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    robot_interface.state.update('charging', True)  # already charging
    
    # Tree setup
    charge_robot_tree = ChargeRobotTree(
        node_name="test_charge_robot_tree_already_charging",
        num_attempts=5,
        robot_interface=robot_interface
    )
    charge_robot_tree.setup()
    
    # Run tree
    runner_thread = threading.Thread(target=charge_robot_tree.run_until_done, daemon=True)
    runner_thread.start()
    runner_thread.join(timeout=50)
    
    if runner_thread.is_alive():
        pytest.fail("Tree did not finish — possible hang or infinite loop.")
    
    final_status = charge_robot_tree.final_status
    print("final_status:", final_status)
    
    # Assertions
    assert not mock_nav_server.triggered, "Navigation should NOT be triggered if charging=True"
    assert not mock_dock_server.triggered, "Docking should NOT be triggered if charging=True"
    assert final_status == py_trees.common.Status.SUCCESS, f"Expected SUCCESS but got {final_status}"
    assert robot_interface.state.get('charging') is True, "Robot should not be charging"
    
    # Cleanup
    executor.shutdown()
    executor_thread.join()
    mock_nav_server.destroy_node()
    mock_dock_server.destroy_node()
    mock_undock_server.destroy_node()
    charge_robot_tree.cleanup()
    
def test_charge_robot_tree_nav_fails():
    """Test that ChargeRobotTree returns FAILURE if navigation fails."""
    
    # Mock servers
    mock_nav_server = BaseMockActionServer(
        action_name='/navigate_to_pose',
        action_type=NavigateToPose,
        result_cls=NavigateToPose.Result,
        succeed=False,  # navigation fails
        wait_time=1.0
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
    
    # Spin executor in background
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    robot_interface.state.update('charging', False)
    
    # Tree setup
    charge_robot_tree = ChargeRobotTree(
        node_name="test_charge_robot_tree_nav_fail",
        num_attempts=5,
        robot_interface=robot_interface
    )
    charge_robot_tree.setup()
    
    # Run tree
    runner_thread = threading.Thread(target=charge_robot_tree.run_until_done, daemon=True)
    runner_thread.start()
    runner_thread.join(timeout=100)
    
    if runner_thread.is_alive():
        pytest.fail("Tree did not finish — possible hang or infinite loop.")
    
    final_status = charge_robot_tree.final_status
    print("final_status:", final_status)
    
    # Assertions
    assert mock_nav_server.triggered, "Navigation should have been triggered"
    assert mock_nav_server.result_status == "failure"
    assert not mock_dock_server.triggered, "Docking should not be triggered if navigation fails"
    assert final_status == py_trees.common.Status.FAILURE, f"Expected FAILURE but got {final_status}"
    assert robot_interface.state.get('charging') is False, "Robot should not be charging"

    # Cleanup
    executor.shutdown()
    executor_thread.join()
    mock_nav_server.destroy_node()
    mock_dock_server.destroy_node()
    mock_undock_server.destroy_node()
    charge_robot_tree.cleanup()

def test_charge_robot_tree_docking_fails():
    """Test that ChargeRobotTree returns FAILURE if docking fails."""
    
    # Mock servers
    mock_nav_server = BaseMockActionServer(
        action_name='/navigate_to_pose',
        action_type=NavigateToPose,
        result_cls=NavigateToPose.Result,
        succeed=True,  
        wait_time=1.0
    )
    
    mock_dock_server = BaseMockActionServer(
        action_name='/docking',
        action_type=DockingRequest,
        result_cls=DockingRequest.Result,
        succeed=False,
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
    
    # Spin executor in background
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    robot_interface.state.update('charging', False)
    
    # Tree setup
    charge_robot_tree = ChargeRobotTree(
        node_name="test_charge_robot_tree_docking_fail",
        num_attempts=5,
        robot_interface=robot_interface
    )
    charge_robot_tree.setup()
    
    # Run tree
    runner_thread = threading.Thread(target=charge_robot_tree.run_until_done, daemon=True)
    runner_thread.start()
    runner_thread.join(timeout=100)
    
    if runner_thread.is_alive():
        pytest.fail("Tree did not finish — possible hang or infinite loop.")
    
    final_status = charge_robot_tree.final_status
    print("final_status:", final_status)
    
    # Assertions
    assert mock_nav_server.triggered, "Navigation should have been triggered"
    assert mock_nav_server.result_status == "succeeded"
    assert mock_dock_server.triggered, "Docking should have been triggered"
    assert mock_dock_server.result_status == "failure"
    assert final_status == py_trees.common.Status.FAILURE, f"Expected FAILURE but got {final_status}"
    assert robot_interface.state.get('charging') is False, "Robot should not be charging"
    
    # Cleanup
    executor.shutdown()
    executor_thread.join()
    mock_nav_server.destroy_node()
    mock_dock_server.destroy_node()
    mock_undock_server.destroy_node()
    charge_robot_tree.cleanup()



# ~/smart_home_pytree_ws/src/smart_home_pytree: run  python3 -m  pytest test/unit/test_charge_robot_tree.py -vv
