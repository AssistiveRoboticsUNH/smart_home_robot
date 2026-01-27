#!/usr/bin/env python3
import os

import py_trees
import py_trees_ros
import yaml

from smart_home_pytree.utils import BlackboardLogger


def update_protocol_config(protocol_name: str, key_to_update: str, new_value: str):
    """
    Updates a specific configuration value for a protocol on the Blackboard
    without resetting the execution state (done flags).
    """
    blackboard = py_trees.blackboard.Blackboard()
    
    # 1. Check if the protocol exists
    if not blackboard.exists(protocol_name):
        print(f"[Update] Error: Protocol '{protocol_name}' not found on Blackboard.")
        return False

    # 2. Get the CURRENT configuration dictionary
    #    (This contains 'video_path', 'text', etc.)
    current_config = blackboard.get(protocol_name)

    if not isinstance(current_config, dict):
        print(f"[Update] Error: Blackboard key '{protocol_name}' is not a dictionary.")
        return False

    # 3. Log the change for safety
    old_value = current_config.get(key_to_update, "NOT_SET")
    print(f"[Update] Changing {protocol_name}[{key_to_update}]: '{old_value}' -> '{new_value}'")

    # 4. Update the dictionary in place
    current_config[key_to_update] = new_value

    # 5. Write it back to Blackboard (updates the reference)
    blackboard.set(protocol_name, current_config)
    
    return True

def load_locations_to_blackboard(yaml_path: str, debug: bool = False):
    """
    Load location data from a YAML file and register it to the py_trees blackboard.
    """

    # Get blackboard
    blackboard = py_trees.blackboard.Blackboard()

    try:
        blackboard.get("initialized")
        return blackboard
    except BaseException:
        pass
    # ------------------------

    # Load YAML
    with open(yaml_path, "r") as file:
        data = yaml.safe_load(file)

    if "locations" not in data:
        raise KeyError("YAML file must contain a 'locations' field.")

    locations = data["locations"]

    # Register to blackboard
    blackboard.set("locations", locations)

    if debug:
        print("Registered the following locations to the blackboard:")
    for name, loc in locations.items():
        if debug:
            print(f"  {name}: {loc}")

    # Set flag (singleton-style marker)
    blackboard.set("initialized", True)
    if debug:
        print("[Blackboard] Registered 'locations' once only.")

    return blackboard


def load_protocols_to_bb(yaml_path: str, debug: bool = False):
    """
    Load protocol related data from a YAML file and register it to the py_trees blackboard.
    also sets done flags to false for each action in the protocol.
    """
    # Get blackboard
    blackboard = py_trees.blackboard.Blackboard()

    # Load YAML
    with open(yaml_path, "r") as file:
        data = yaml.safe_load(file)

    # Ensure structure is correct
    if "protocols" not in data:
        raise KeyError("YAML must contain protocols -> TwoReminderProtocol structure.")

    protocols = data["protocols"]

    for protocol_type in protocols.keys():
        if debug:
            print("Protocol type: ", protocol_type)
        for protocol_name in protocols[protocol_type].keys():
            if debug:
                print("protocol name : ", protocol_name)
            protocol_dict = {}
            protocol_dict_done = {}

            # low_level = protocols[protocol_type][protocol_name]["low_level"]
            # 1. Get the specific protocol data
            protocol_data = protocols[protocol_type][protocol_name]

            # 2. Try to get 'low_level'. If missing, default to None.
            low_level = protocol_data.get("low_level")

            # 3. If it exists but is empty in YAML (None), make it an empty dict
            if low_level is None:
                if debug:
                    print("low_level is none for protocol : ", protocol_name)
                continue

            for key, value in low_level.items():
                if debug:
                    print(key, value)
                protocol_dict[key] = value

                if key.startswith("type_"):
                    # Skip creating *_done entry
                    continue

                if key.startswith("number_of_protocols"):
                    # Skip creating *_done entry
                    continue

                if "exercise" in protocol_name:
                    if key != "get_confirmation":
                        continue
                protocol_dict_done[f"{key}_done"] = False

            blackboard.set(protocol_name, protocol_dict)

            if "exercise" in protocol_name:
                if key != "get_confirmation":
                    continue

            blackboard.set(f"{protocol_name}_done", protocol_dict_done)

    if debug:
        for key, value in blackboard.storage.items():
            print(f"{key} : {value}")

    return blackboard
        
if __name__ == "__main__":

    yaml_file_path = os.getenv("house_yaml_path", None)
    print("yaml_file_path", yaml_file_path)
    bb = load_locations_to_blackboard(yaml_file_path)
    print("bb: ", bb)

    print("\n--- Blackboard raw storage ---")
    print(py_trees.blackboard.Blackboard.storage)

    target_location_name = "kitchen"
    target_location = bb.get("locations")[target_location_name]

    x = target_location["x"]
    y = target_location["y"]
    quat_vals = target_location["quat"]

    print("x: ", x, " y: ", y, " quat: ", quat_vals)

    load_protocols_to_bb(yaml_file_path)
