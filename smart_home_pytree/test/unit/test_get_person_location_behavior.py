import pytest
import py_trees
from smart_home_pytree.behaviors.get_person_location import GetPersonLocation
from smart_home_pytree.registry import load_locations_to_blackboard
import os

class RobotInterface_:
    def __init__(self, state):
        self.state = state

# def setup_module(module):
    
#     print('\nsetup_module()')
#     """Reset the py_trees blackboard before each test automatically."""
    
#     # global blackboard
#     global blackboard
    
#     blackboard = py_trees.blackboard.Blackboard()
#     blackboard.storage.clear()
        
#     """Fixture to load the house locations into the blackboard."""
#     # yaml_file_path = "/home/olagh48652/smart_home_pytree_ws/src/smart_home_pytree/config/house_info.yaml"
#     yaml_file_path = os.getenv("house_yaml_path", None) 
#     load_locations_to_blackboard(yaml_file_path)
#     blackboard = py_trees.blackboard.Blackboard()
    
def setup_function(function):
    print('\nsetup_function()')
    global blackboard
    
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.storage.clear()
    yaml_file_path = os.getenv("house_yaml_path", None) 
    load_locations_to_blackboard(yaml_file_path)    
        
def test_person_location_valid():
    """Test when the person location is a valid room."""
    target_location = blackboard.get("locations")
    loc = "kitchen"

    # Simulate blackboard and state
    blackboard.set("person_location", loc)
    state = {"person_location": loc}
    robot_interface = RobotInterface_(state)

    # Run GetPersonLocation
    get_person_loc = GetPersonLocation(robot_interface)
    status = get_person_loc.update()

    # Expected behavior
    assert loc in target_location
    assert status == py_trees.common.Status.SUCCESS


def test_person_location_not_registered():
    """Test when the location is not in the registered house locations."""
    loc = "garage"  # Not in YAML
    state = {"person_location": loc}

    robot_interface = RobotInterface_(state)
    
    blackboard.set("person_location", loc)
    get_person_loc = GetPersonLocation(robot_interface)
    status = get_person_loc.update()

    assert status == py_trees.common.Status.FAILURE


def test_person_location_none():
    """Test when the person location is None."""
    state = {"person_location": None}
    robot_interface = RobotInterface_(state)
    
    get_person_loc = GetPersonLocation(robot_interface)
    status = get_person_loc.update()

    assert status == py_trees.common.Status.FAILURE


def test_person_location_missing_key():
    """Test when the state dictionary does not contain the key 'person_location'."""
    state = {}  # Missing key
    robot_interface = RobotInterface_(state)
    
    get_person_loc = GetPersonLocation(robot_interface)
    status = get_person_loc.update()

    assert status == py_trees.common.Status.FAILURE


# folder: /home/olagh48652/smart_home_pytree_ws/src/smart_home_pytree/test
# python3 -m pytest -v unit/test_get_person_location_behavior.py
# =========================================================================================== test session starts ============================================================================================
# platform linux -- Python 3.10.12, pytest-6.2.5, py-1.11.0, pluggy-0.13.0 -- /usr/bin/python3
# cachedir: .pytest_cache
# rootdir: /home/olagh48652/smart_home_pytree_ws/src/smart_home_pytree
# plugins: launch-testing-ros-0.19.7, launch-pytest-1.0.5, launch-testing-1.0.5, ament-xmllint-0.12.10, ament-pep257-0.12.10, ament-mypy-0.12.10, ament-flake8-0.12.10, ament-copyright-0.12.10, ament-lint-0.12.10, langsmith-0.4.27, colcon-core-0.20.1, anyio-4.11.0, cov-3.0.0, mock-3.6.1, timeout-2.1.0, repeat-0.9.1, rerunfailures-10.2, typeguard-4.3.0
# collected 4 items                                                                                                                                                                                          

# unit/test_get_person_location_behavior.py::test_person_location_valid PASSED                                                                                                                         [ 25%]
# unit/test_get_person_location_behavior.py::test_person_location_not_registered PASSED                                                                                                                [ 50%]
# unit/test_get_person_location_behavior.py::test_person_location_none PASSED                                                                                                                          [ 75%]
# unit/test_get_person_location_behavior.py::test_person_location_missing_key PASSED                                                                                                                   [100%]

# ============================================================================================ 4 passed in 0.49s =============================================================================================
# olagh48652@olagh-Legion-5-15IAH7H:~/smart_home_pytree_ws/src/smart_home_pytree/test$ 
