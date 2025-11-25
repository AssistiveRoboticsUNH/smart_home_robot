
## functions that start with test will be run 

import pytest

import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
import py_trees
from smart_home_pytree.trees.read_script_tree import ReadScriptTree
from smart_home_pytree.registry import load_locations_to_blackboard, load_protocols_to_bb

# Replace these with your actual action types
from nav2_msgs.action import NavigateToPose
from shr_msgs.action import DockingRequest
from ..mock.mock_action_server import BaseMockActionServer
import time
from smart_home_pytree.robot_interface import RobotInterface
import threading
import os

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
    
    yaml_file_path = os.getenv("house_yaml_path", None) 
    load_protocols_to_bb(yaml_file_path)
    print('\nsetup_function()')

def teardown_function(function):
    print('\nteardown_function()')

def test_read_script_only_runs_in_same_location():
    blackboard = py_trees.blackboard.Blackboard()
    
    
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
    protocol_name = "medicine_am"
    
    print("start wait")
    time.sleep(2.0)  # give server time to start
    print("stop wait")
    
    robot_interface.state.update('charging', False)
    robot_interface.state.update('person_location', 'living_room')
    robot_interface.state.update('robot_location', 'kitchen')
    
    
    text = "first_text"
    wait_time_key = "wait_time_between_reminders"
  
    tree_runner = ReadScriptTree(
        node_name="read_script_test",
        robot_interface=robot_interface,
        protocol_name=protocol_name,
        text_number=text,
        wait_time_key=wait_time_key
    )
    
    tree_runner.setup()

    def on_moving_trigger(action_name):
        print(f"[Callback] Moving triggered ({action_name}), setting new location")
        robot_interface.state.update('robot_location', 'living_room')

    def on_docking_trigger(action_name):
        print(f"[Callback] Moving triggered ({action_name}), setting new location")
        robot_interface.state.update('charging', True)
        
    mock_nav_server.set_on_trigger(on_moving_trigger)
    mock_dock_server.set_on_trigger(on_docking_trigger)
    
    runner_thread = threading.Thread(target=tree_runner.run_until_done, daemon=True)
    runner_thread.start()
    runner_thread.join(timeout=100)
    
    ## since charging is false
    assert not mock_undock_server.triggered
    assert mock_nav_server.triggered, "Expected navigation to be triggered"
    assert mock_dock_server.triggered, "Expected docking to be triggered"
    
    assert robot_interface.state.get('charging') is True, "Robot should not be charging"
    
    ## assert all keys corresponding to protocol passed
    found_protocol = False
    expected_keys_to_be_true = [f"{text}_done", f"{wait_time_key}_done" ]
    
    for key, value in blackboard.storage.items():
        print("key:", key)

        if key == f"/{protocol_name}_done":
            found_protocol = True            
            assert isinstance(value, dict), f"[FAILED] Blackboard value for '{key}' is not a dict (got {type(value).__name__})"
            for sub_key, sub_value in value.items():
                
                if sub_key in expected_keys_to_be_true: 
                    assert sub_value is True, f"[FAILED] Blackboard key '{sub_key}' has value '{sub_value}', expected True"

    # Fail if the protocol key wasn't found at all
    assert found_protocol, f"[FAILED] Blackboard did not contain '/{protocol_name}_done'"
    assert tree_runner.final_status == py_trees.common.Status.SUCCESS
    
    executor.shutdown(); executor_thread.join()
    mock_nav_server.destroy_node()
    mock_dock_server.destroy_node()
    mock_undock_server.destroy_node()
    tree_runner.cleanup()
    
def test_read_script_shouldnt_run_in_diff_location():
    blackboard = py_trees.blackboard.Blackboard()

    
    # Setup mock navigation server
    mock_nav_server = BaseMockActionServer(
        action_name='/navigate_to_pose',
        action_type=NavigateToPose,
        result_cls=NavigateToPose.Result,
        succeed=False,
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
    protocol_name = "medicine_am"
    
    print("start wait")
    time.sleep(2.0)  # give server time to start
    print("stop wait")
    
    robot_interface.state.update('charging', False)
    robot_interface.state.update('person_location', 'living_room')
    robot_interface.state.update('robot_location', 'kitchen')
    
    
    text = "first_text"
    wait_time_key = "wait_time_between_reminders"
  
    tree_runner = ReadScriptTree(
        node_name="read_script_test",
        robot_interface=robot_interface,
        protocol_name=protocol_name,
        text_number=text,
        wait_time_key=wait_time_key
    )
    
    tree_runner.setup()

    def on_docking_trigger(action_name):
        print(f"[Callback] Moving triggered ({action_name}), setting new location")
        robot_interface.state.update('charging', True)
        
    mock_dock_server.set_on_trigger(on_docking_trigger)
    
    runner_thread = threading.Thread(target=tree_runner.run_until_done, daemon=True)
    runner_thread.start()
    runner_thread.join(timeout=100)
    
    ## since charging is false
    assert not mock_undock_server.triggered
    assert mock_nav_server.triggered, "Expected navigation to be triggered"
    # assert mock_dock_server.result_status == "succeeded"

    assert not mock_dock_server.triggered, "Expected docking not to be triggered"
    # assert robot_interface.state.get('charging') is True, "Robot should not be charging"
    
    ## assert all keys corresponding to protocol passed
    found_protocol = False
    expected_keys_to_be_true = [f"{text}_done", f"{wait_time_key}_done" ]
    
    for key, value in blackboard.storage.items():
        print("key:", key)

        if key == f"/{protocol_name}_done":
            found_protocol = True            
            assert isinstance(value, dict), f"[FAILED] Blackboard value for '{key}' is not a dict (got {type(value).__name__})"
            for sub_key, sub_value in value.items():
                
                if sub_key in expected_keys_to_be_true: 
                    assert sub_value is False, f"[FAILED] Blackboard key '{sub_key}' has value '{sub_value}', expected False"

    # Fail if the protocol key wasn't found at all
    assert found_protocol, f"[FAILED] Blackboard did not contain '/{protocol_name}_done'"
    
    robot_final_loc = robot_interface.state.get('robot_location')
    person_final_loc = robot_interface.state.get('person_location')
    assert robot_final_loc != person_final_loc, f"Expected robot and person not to be in same location, got robot={robot_final_loc}, person={person_final_loc}"            
     
    assert tree_runner.final_status == py_trees.common.Status.FAILURE
    
    executor.shutdown(); executor_thread.join()
    mock_nav_server.destroy_node()
    mock_dock_server.destroy_node()
    mock_undock_server.destroy_node()
    tree_runner.cleanup()
     
# ~/smart_home_pytree_ws/src/smart_home_pytree: run  python3 -m  pytest test/unit/test_read_script_tree.py -vv
