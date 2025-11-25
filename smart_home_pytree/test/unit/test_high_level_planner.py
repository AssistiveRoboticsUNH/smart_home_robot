
## functions that start with test will be run 

import pytest

import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
import py_trees
from smart_home_pytree.trees.two_reminder_protocol import TwoReminderProtocolTree
from smart_home_pytree.registry import load_locations_to_blackboard, load_protocols_to_bb
from smart_home_pytree.protocol_orchestrator import ProtocolOrchestrator

# Replace these with your actual action types
from nav2_msgs.action import NavigateToPose
from shr_msgs.action import DockingRequest
from ..mock.mock_action_server import BaseMockActionServer
import time
from smart_home_pytree.robot_interface import RobotInterface
import threading
import os

import signal

# pytest fixture that temporarily disables Python’s signal handling inside your test environment
@pytest.fixture(autouse=True)
def disable_signal_calls(monkeypatch):
    monkeypatch.setattr(signal, "signal", lambda *args, **kwargs: None)
    
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
    
###
# TODO make sure it always uses the yaml file in house_info regardlesss of env variable
    ## medicine_am : time: {'from': '10:00', 'to': '11:00'}
    ## medicine_pm : time: {'from': '22:00', 'to': '23:00'}
    ## coffee : time: {'from': '09:00', 'to': '13:00'}
###

### HELPER FUNCTIONS
def run_orchestrator_until_all_completed(orch, protocol_names, timeout=100):
    """
    Run the orchestrator loop in a thread until *all* given protocols complete
    or until the timeout expires.
    """
    loop_thread = threading.Thread(target=orch.orchestrator_loop, daemon=True)
    loop_thread.start()

    deadline = time.time() + timeout
    remaining = set(protocol_names)

    while time.time() < deadline and remaining:
        completed = set(orch.trigger_monitor.completed_protocols)
        newly_done = [p for p in remaining if p in completed]

        for p in newly_done:
            print(f"[TestHelper] Protocol completed: {p}")
            remaining.remove(p)

        if not remaining:
            break

        time.sleep(0.5)

    orch.shutdown()
    loop_thread.join(timeout=3)

    if remaining:
        raise AssertionError(f"[FAILED] Protocols not completed in time: {sorted(remaining)}")
    else:
        print("[TestHelper] All protocols completed successfully.")


def assert_protocol_completed(orch, expected_satisfied):
    """Check that only expected protocols are satisfied before running."""
    satisfied = orch.trigger_monitor.get_satisfied()
    assert satisfied == expected_satisfied, (
        f"[FAILED] Expected {expected_satisfied}, got {satisfied}"
    )


def assert_blackboard_protocol_done(blackboard, protocol_name):
    """Check that the blackboard marks all reminders as done for a given protocol."""
    found_protocol = False
    key = f"/{protocol_name}_done"

    for k, v in blackboard.storage.items():
        if k == key:
            found_protocol = True
            assert isinstance(v, dict), f"[FAILED] Blackboard value for '{k}' is not a dict"
            for sub_key, sub_value in v.items():
                assert sub_value is True, (
                    f"[FAILED] Blackboard key '{sub_key}' has value '{sub_value}', expected True"
                )

    assert found_protocol, f"[FAILED] Blackboard did not contain '{key}'"


def test_protocol_orchestrator_one_protocol_true():
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
    
    print("start wait")
    time.sleep(2.0)  # give server time to start
    print("stop wait")
    
    robot_interface.state.update('charging', False)
    robot_interface.state.update('person_location', 'living_room')
    robot_interface.state.update('robot_location', 'kitchen')
  
    def on_moving_trigger(action_name):
        print(f"[Callback] Moving triggered ({action_name}), setting new location")
        robot_interface.state.update('robot_location', 'living_room')

    def on_docking_trigger(action_name):
        print(f"[Callback] Moving triggered ({action_name}), setting new location")
        robot_interface.state.update('robot_location', 'home')
        robot_interface.state.update('charging', True)
        
    def on_undocking_trigger(action_name):
        print(f"[Callback] Moving triggered ({action_name}), setting new location")
        robot_interface.state.update('charging', False)
        
    mock_nav_server.set_on_trigger(on_moving_trigger)
    mock_dock_server.set_on_trigger(on_docking_trigger)
    mock_undock_server.set_on_trigger(on_undocking_trigger)
    
    
    robot_interface.state.update('coffee', False)
    robot_interface.state.update('coffee_pot', False)
    orch = ProtocolOrchestrator(robot_interface=robot_interface, test_time="10:30")
    
    time.sleep(2.5)

    # # only satisfied should only be medicine_am
    satisfied = orch.trigger_monitor.get_satisfied()
    print("Satisfied protocols:", satisfied)

    expected = [("TwoReminderProtocol.medicine_am", 2)]
    assert satisfied == expected, f"Expected only medicine_am to be satisfied, got {satisfied}"


    assert_protocol_completed(orch, expected)
    run_orchestrator_until_all_completed(orch, ["TwoReminderProtocol.medicine_am"])
    assert_blackboard_protocol_done(blackboard, "medicine_am")
    
    executor.shutdown(); executor_thread.join()
    mock_nav_server.destroy_node()
    mock_dock_server.destroy_node()
    mock_undock_server.destroy_node()
 
def test_protocol_orchestrator_two_protocol_true():
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
    
    print("start wait")
    time.sleep(2.0)  # give server time to start
    print("stop wait")
    
    robot_interface.state.update('charging', False)
    robot_interface.state.update('person_location', 'living_room')
    robot_interface.state.update('robot_location', 'kitchen')
  
    def on_moving_trigger(action_name):
        print(f"[Callback] Moving triggered ({action_name}), setting new location")
        robot_interface.state.update('robot_location', 'living_room')

    def on_docking_trigger(action_name):
        print(f"[Callback] Moving triggered ({action_name}), setting new location")
        robot_interface.state.update('charging', True)
        robot_interface.state.update('robot_location', 'home')
        
    def on_undocking_trigger(action_name):
        print(f"[Callback] Moving triggered ({action_name}), setting new location")
        robot_interface.state.update('charging', False)
        
    mock_nav_server.set_on_trigger(on_moving_trigger)
    mock_dock_server.set_on_trigger(on_docking_trigger)
    mock_undock_server.set_on_trigger(on_undocking_trigger)
    
    
    robot_interface.state.update('coffee', True)
    robot_interface.state.update('coffee_pot', True)
    orch = ProtocolOrchestrator(robot_interface=robot_interface, test_time="10:30")
    
    executed_protocols = []
    
    # save original method
    original_start_protocol = orch.start_protocol

    # define wrapper to capture order + timestamps
    def wrapped_start_protocol(protocol_name, *args, **kwargs):
        print(f"[TestHook] Starting protocol: {protocol_name}")
        executed_protocols.append(protocol_name)
        return original_start_protocol(protocol_name, *args, **kwargs)

    # patch the orchestrator's start_protocol with our wrapper
    orch.start_protocol = wrapped_start_protocol
    
    
    time.sleep(2.5)
    
    # # only satisfied should only be medicine_am
    satisfied = orch.trigger_monitor.get_satisfied()
    print("Satisfied protocols:", satisfied)

    expected = [("TwoReminderProtocol.medicine_am", 2),("CoffeeProtocol.coffee", 3)]
    assert satisfied == expected, f"Expected medicine_am and coffee to be satisfied, got {satisfied}"

    assert_protocol_completed(orch, expected)
    run_orchestrator_until_all_completed(orch, ["TwoReminderProtocol.medicine_am", "CoffeeProtocol.coffee"])
    assert_blackboard_protocol_done(blackboard, "medicine_am")
    assert_blackboard_protocol_done(blackboard, "coffee")
    
    
    # expected order by priority (smaller number = higher priority)
    expected_priority_order = [p[0] for p in sorted(satisfied, key=lambda x: x[1])]

    assert [p[0] for p in executed_protocols] == expected_priority_order, (
    f"[FAILED] Expected execution order {expected_priority_order}, "
    f"but got {[p[0] for p in executed_protocols]}"
    )
    
    executor.shutdown(); executor_thread.join()
    mock_nav_server.destroy_node()
    mock_dock_server.destroy_node()
    mock_undock_server.destroy_node()
         
# ~/smart_home_pytree_ws/src/smart_home_pytree: run  python3 -m  pytest test/unit/test_two_reminder_protocol_tree.py -vv
