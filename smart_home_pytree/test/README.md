# Smart Home PyTree – Unit Tests Overview

This directory contains unit tests for the **Smart Home PyTree** project.

TODO: update tests to pick up if behavior was run by the protocol more than once programatically

## Directory Layout

```
test/
├── action_server_tests/        # Placeholder for ROS2 action server integration tests
├── mock/                       # Mock utilities for ROS2 nodes and action servers
├── unit/                       # Core behavior tree and robot interaction tests
│   ├── test_charge_robot_tree.py
│   ├── test_get_person_location_behavior.py
│   ├── test_guarded_move_to_person_location_tree.py
│   ├── test_move_to_person_location_tree.py
│   └── test_move_to_tree.py
└── gui_for_testing.py          # Optional GUI launcher for testing purposes but not with testing scripts
```

---

## About the Mock Action Server

The `BaseMockActionServer` class simulates a ROS2 action server for testing without real hardware.  
It allows you to mock any action (ex: navigation, docking, and undocking).

### Key Parameters

| Argument | Type | Description |
|-----------|------|-------------|
| `action_name` | `str` | Name of the ROS2 action topic (e.g., `/navigate_to_pose`) |
| `action_type` | `Action` | ROS2 action type (e.g., `NavigateToPose`) |
| `result_cls` | `Result` | Result message type |
| `succeed` | `bool` | If `True`, action completes successfully |
| `cancel` | `bool` | If `True`, simulates cancellation |
| `wait_time` | `float` | Duration before completing action |

### Example Usage

```python
mock_nav_server = BaseMockActionServer(
    action_name='/navigate_to_pose',
    action_type=NavigateToPose,
    result_cls=NavigateToPose.Result,
    succeed=True,
    wait_time=1.0
)
```

You can also attach a **trigger callback**:

```python
mock_nav_server.set_on_trigger(lambda name: print(f"{name} triggered"))
```

Use this when you want to change the state after a mock action server is triggered.

---

## Available Tests

Each test focuses on testing a specific tree or behavior. 

**Important**: Do not run live ROS topics during testing, the RobotInterface automatically updates its state from active topics, which can overwrite test configurations.

 - test_charge_robot_tree.py  
 - test_get_person_location_behavior.py  
 - test_guarded_move_to_person_location_tree.py  
 - test_move_to_person_location_tree.py  
 - test_move_to_tree.py
 - test_read_script_tree.py

---

## Running in Simulation
note you will need the turtlebot package installed

Run the launch script
```
ros2 launch smart_home_pytree sim_robot.launch.py
```

then in another terminal run the orchestrator

Prerequisites: Before running, you must set an environment variable pointing to your configuration file. By default, the script looks for house_yaml_path.
```bash
export house_yaml_path="/absolute/path/to/your/config.yaml"
```
Basic Execution:
```
python3 protocol_orchestrator.py 
```

Testing Specific Times: Run with time override for 2:30 PM and debug mode
```
python3 protocol_orchestrator.py --test_time "14:30" --debug
```

| Argument | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `--test_time` | `str` | `""` | Optional time override in `HH:MM` format. If provided, the system ignores the wall clock and uses this static time. |
| `--debug` | `flag` | `False` | Enable detailed debug output to the console. |
| `--env_yaml_file_name` | `str` | `"house_yaml_path"` | The **name** of the environment variable to read the YAML path from. Change this if you use a different env var name. |

## Running the Tests

NOTE: HUGE REFACTOR HAPPENED TO THE CODE, TESTS HAVENT BEEN UPDATED YET 
From the package root `~/smart_home_pytree_ws/src/smart_home_pytree`:

To run all the tests at once:

```bash
python3 -m pytest test/unit -vv
```
But currently this doesnt work correctly due to ros startup and teardown. working on a fix
 
To run one test at a time:
```bash
python3 -m pytest test/unit/test_move_to_person_location_tree.py -vv
```

### Running a Specific Test Function

```bash
python3 -m pytest test/unit/test_file.py::function_name -vv
```
Example:
```bash
python3 -m pytest test/unit/test_guarded_move_to_person_location_tree.py::test_guarded_move_aborts_when_person_moves -vv
```

### Display Print and Log Output

Use `-s` to disable pytest’s output capture and show all console logs:

```bash
python3 -m pytest test/unit/test_move_to_person_location_tree.py -vv -s
```
---

## Notes

- Initialize ROS2 (rclpy) once in setup_module() and shut it down cleanly in teardown_module(). Don't initialize within a function. 
- Always close executors and destroy nodes at the end of each test to prevent resource leaks.
- Use `pytest -vv -s` for detailed, real-time debugging output. Failures are 
- All test files and functions must start with test_ to be automatically detected by pytest.
