# Smart Home Pytree

This project is being developed at and for the University of New Hampshire’s [Cognitive Assistive Robotics Lab](https://github.com/AssistiveRoboticsUNH) under NIH support.  


## Overview

### `Directory Structure`

```
smart_home_pytree
├── config
│   ├── house_info.yaml
│   └── Readme.md
├── doc
│   ├── charge_robot_tree.png
│   ├── moveto_tree.png
│   └── move_to_with_gaurd.webm
├── launch
│   ├── launch_action.launch.py
│   └── launch_move_tree.launch.py
├── package.xml
├── README.md
├── resource
│   └── smart_home_pytree
├── robot_actions
│   ├── docking.py
│   ├── generic_action_server.py
│   ├── __init__.py
│   └── undocking.py
├── setup.cfg
├── setup.py
└── smart_home_pytree
    ├── behaviors
    │   ├── action_behaviors
    │   ├── check_robot_state_key.py
    │   ├── get_person_location.py
    │   ├── logging_behavior.py
    │   ├── move_to_behavior.py
    │   └── robot_person_same_location.py
    ├── __init__.py
    ├── registry.py
    ├── robot_interface.py
    └── trees
        ├── base_tree_runner.py
        ├── charge_robot_tree.py
        ├── guarded_move_to_person_location.py
        ├── move_to_person_location.py
        ├── move_to_tree.py
        ├── Readme.md
        └── two_reminder_protocol.py
```

### `Config directory: `
This should contain a yaml file (`config/house_info.yaml`). This file should contain data required for the different protocols.  
It will vary for each home but should include:
- (a) Locations (`x`, `y`, `quat`) of different landmarks in the environment.  
- (b) Protocol names and their corresponding identifiers (e.g., `medicine_am`) along with information needed for the protocols.

---

### `robot_actions directory: `
contains the action servers needed for the robot:

#### `generic_action_server.py`
Defines the base class for action servers.  
It raises an error if the derived class does not implement `execute_callback`.  

Currently available action servers:
mock docking:
mock undocking: 

TODO:
undocking: actually send undock
docking: actually dock the robot
question answer


---
### `smart_home_pytree directory: `

This directory contains two main folders: **`trees`** and **`behaviors`**.

**General Files:**
- **`robot_interface.py`** – A singleton class that runs in its own thread when initialized. It includes data about the robot and environment that it gets from the topics.
- **`registry.py`** – Reads data from the YAML file and registers it into the blackboard.

---
#### `smart_home_pytree/trees: `

Contains the trees listed in available trees section in addition to the base class:

#### `base_tree_runner.py`
Defines the base class for all behavior trees.  
It raises an error if the derived class does not implement `create_tree()`.  

**Main functions:**
- **`create_tree()`** – Must be implemented in derived classes.  
- **`setup()`** – Calls `create_tree()` and runs the tree in an executor thread.  
- **`run_until_done()`** – Runs the tree until it finishes with either `SUCCESS` or `FAILURE`.  
- **`run_continuous()`** – Runs the tree continuously until the user stops it.  
- **`cleanup()`** – Ensures a clean shutdown of all nodes, executors, and subprocesses.
- **`stop()`** – Stops the tree.

**Helper functions:**
- **`required_actions()`** – Should be overridden in the derived class.  
- **`required_topics()`** – Should be overridden in the derived class.  
- **`describe_requirements()`** – Lists the requirements specified in `required_actions` and `required_topics`.

## Available Trees
These trees can be combined to build higher-level protocols.  
All of them inherit from `base_tree_runner.py`.

### 1. `move_to_tree`
Receives a location to move to and checks if the location is initialized in the blackboard. Before initiating movement, the tree checks whether the robot is currently charging.  
If it is, the robot will undock before moving.

---

### 2. `charge_robot_tree`
Responsible for managing the robot charging process.  
Takes the number of retries (`num_attempts`) as input.  

**Behavior:**
- If the robot is already charging, it exits.
- Otherwise, it moves the robot to the home position, docks, and checks if charging was successful.  
- Retries up to `num_attempts` times, logging failures if they occur.

---

### 3. `move_to_person_location`
Retrieves the person’s location and moves the robot there.  
When the robot reaches the target, it verifies if the robot and person are at the same location.  
If not, it retries up to `num_attempts` times.

---

### 4. `guarded_move_to_person_location`
An extension of `move_to_person_location`.  
If the person’s location changes while the robot is moving, the tree halts movement and redirects to the new location.

---

### 5. `two_reminder_protocol`
Implements the two-reminder protocol.  
Sequence:
1. Go to the person’s location and read the first reminder.  
2. Return to the docking station and wait.  
3. Go to the person’s location again to deliver the second reminder.

Includes:
- **`load_protocol_info_from_bb()`** – Loads protocol-specific information from the blackboard.

## Running a Tree

Navigate to the `smart_home_pytree/trees` folder.  
Each script includes a help function that lists the available options.

**Example:**

```bash
python3 two_reminder_protocol.py --help
```

**Output:**

```
usage: two_reminder_protocol.py [-h] [--run_continuous RUN_CONTINUOUS]
                                [--num_attempts NUM_ATTEMPTS]
                                [--protocol_name PROTOCOL_NAME]

Two Reminder Protocol Tree

Handles the Two Reminder Protocol:
1. Retries up to num_attempts times if needed.

options:
  -h, --help            Show this help message and exit.
  --run_continuous RUN_CONTINUOUS
                        Run tree continuously (default: False)
  --num_attempts NUM_ATTEMPTS
                        Retry attempts (default: 3)
  --protocol_name PROTOCOL_NAME
                        Name of the protocol to run (e.g., medicine_am)
```

> **Note:** The required action servers and topics must be running before executing any tree.

for testing you can use a simple GUI for testing topics is available in:  
`test/gui_for_testing.py`  

### For Fast Debugging
You can use the test_actions.py. the script creates action servers that give yes after some time. you can adjust the time in the script too. to run that use command in the ws directory: python3 src/smart_home_pytree/test/mock/mock_run_actions.py 
This runs the followign three action servers docking, undocking and navigate_to_pose

for the topics use the gui_for_testing.py mentioned above.

## Saving Protocol Info into BlackBoard

Each protocol is uniquely identified by its name. When loaded, its configuration is stored on the Blackboard, using the protocol’s name as the key and its associated data (from the YAML) as the value.

Accessing Protocol Data
1. Get a Blackboard instance:
```blackboard = py_trees.blackboard.Blackboard()```

2. Retrieve your protocol’s data:
Set protocol_name to the unique name defined in your YAML (e.g., medicine_am):

  ```protocol_info = blackboard.get(protocol_name)```

3. Use the stored values:

For example, if your YAML defines:

medicine_am:
  low_level:
    first_text: "please take your morning medicine"
    second_text: "This is the second reminder to take your morning medicine"
    wait_time_between_reminders: 5

You can access any field like this:
```text = protocol_info["first_text"]```


## Video of Running the Tree

[running_move_to_tree.webm](https://github.com/user-attachments/assets/70da4dc0-a3cc-47f9-82a3-99c17cc8f576)

## For Testing

Refer to the README.md file in the test folder

## Relevant Documentation

For more details, see:
[py_trees_ros documentation](https://py-trees-ros.readthedocs.io/en/devel/)
[py_trees documentation](https://py-trees.readthedocs.io/en/devel/)
[py_trees_ros_tutorials documentation](https://py-trees-ros-tutorials.readthedocs.io/en/devel/)
