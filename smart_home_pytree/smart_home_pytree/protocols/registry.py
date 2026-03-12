#!/usr/bin/env python3
import os

import py_trees

from smart_home_pytree.protocols.loader import load_house_config_yaml
from smart_home_pytree.protocols.schema import RUN_TREE_SPECS


def update_protocol_config(protocol_name: str, key_to_update: str, new_value: str, debug: bool = False) -> bool:
    """
    Updates a specific configuration value for a protocol on the Blackboard
    without resetting the execution state (done flags).
    """
    blackboard = py_trees.blackboard.Blackboard()
    
    # 1. Check if the protocol exists
    if not blackboard.exists(protocol_name):
        if debug:
            print(f"[Update] Error: Protocol '{protocol_name}' not found on Blackboard.")
        return False

    # 2. Get the CURRENT configuration dictionary
    #    (This contains 'video_path', 'text', etc.)
    current_config = blackboard.get(protocol_name)

    if not isinstance(current_config, dict):
        if debug:
            print(f"[Update] Error: Blackboard key '{protocol_name}' is not a dictionary.")
        return False

    # 3. Log the change for safety
    old_value = current_config.get(key_to_update, "NOT_SET")
    
    if debug:
        print(f"[Update] Changing {protocol_name}[{key_to_update}]: '{old_value}' -> '{new_value}'")

    # 4. Update the dictionary in place
    current_config[key_to_update] = new_value

    # 5. Write it back to Blackboard (updates the reference)
    blackboard.set(protocol_name, current_config)
    
    return True

def load_locations_to_blackboard(yaml_path: str, debug: bool = False, force: bool = False):
    """
    Load location data from a YAML file and register it to the py_trees blackboard.
    """

    # Get blackboard
    blackboard = py_trees.blackboard.Blackboard()

    if not force:
        try:
            blackboard.get("initialized")
            return blackboard
        except BaseException:
            pass
    # ------------------------

    # Load YAML
    data = load_house_config_yaml(yaml_path)

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


def load_protocols_to_bb(yaml_path: str, debug: bool = False, force: bool = False):
    """
    Load protocol related data from a YAML file and register it to the py_trees blackboard.
    also sets done flags to false for each action in the protocol.
    """
    # Get blackboard
    blackboard = py_trees.blackboard.Blackboard()

    if force and blackboard.exists("_protocol_registry_names"):
        prior_names = blackboard.get("_protocol_registry_names") or []
        for name in prior_names:
            if blackboard.exists(name):
                blackboard.unset(name)
            done_key = f"{name}_done"
            if blackboard.exists(done_key):
                blackboard.unset(done_key)

    # Load YAML
    data = load_house_config_yaml(yaml_path)

    # Ensure structure is correct
    if "protocols" not in data:
        raise KeyError("YAML must contain a top-level 'protocols' mapping.")

    protocols = data["protocols"]

    for protocol_name, protocol_data in protocols.items():
        if debug:
            print("protocol name : ", protocol_name)

        runner = protocol_data.get("runner", "GenericProtocol")
        protocol_dict = {}
        protocol_dict_done = {}

        # Generic protocols store action.steps and synthesize blackboard keys for existing trees.
        if runner == "GenericProtocol":
            action_config = protocol_data.get("action", {}) or {}
            steps = action_config.get("steps", []) or []
            protocol_dict["steps"] = steps

            def _add_executable_step(prefix: str, action_step: dict):
                tree_name = action_step.get("tree_name")
                protocol_dict_done[f"{prefix}_done"] = False

                spec = RUN_TREE_SPECS.get(tree_name, {})
                payload_param = spec.get("blackboard_value_param")
                tree_params = action_step.get("tree_params") or {}
                if payload_param:
                    protocol_dict[prefix] = tree_params.get(payload_param)

            def _add_step_wait(wait_key: str, next_step_after, has_following_step: bool):
                if has_following_step and next_step_after not in (None, 0, 0.0, "", "0"):
                    protocol_dict[wait_key] = next_step_after
                    protocol_dict_done[f"{wait_key}_done"] = False

            def _synth_branch_steps(branch_prefix: str, branch_steps: list):
                for b_idx, b_step in enumerate(branch_steps, start=1):
                    if not isinstance(b_step, dict):
                        continue
                    step_key = f"{branch_prefix}_step_{b_idx}"
                    wait_key = f"{branch_prefix}_wait_{b_idx}"
                    _add_executable_step(step_key, b_step)
                    _add_step_wait(
                        wait_key=wait_key,
                        next_step_after=b_step.get("next_step_after", 0),
                        has_following_step=(b_idx < len(branch_steps)),
                    )

            for idx, step in enumerate(steps, start=1):
                if not isinstance(step, dict):
                    continue

                step_key = f"step_{idx}"
                confirm_key = f"confirm_{idx}"
                wait_key = f"wait_{idx}"

                if "confirmation" in step:
                    confirmation = step.get("confirmation") or {}
                    protocol_dict[confirm_key] = confirmation.get("question")
                    protocol_dict_done[f"{confirm_key}_done"] = False
                    _synth_branch_steps(f"yes_{idx}", confirmation.get("on_yes") or [])
                    _synth_branch_steps(f"no_{idx}", confirmation.get("on_no") or [])
                else:
                    _add_executable_step(step_key, step)

                _add_step_wait(
                    wait_key=wait_key,
                    next_step_after=step.get("next_step_after", 0),
                    has_following_step=(idx < len(steps)),
                )

            blackboard.set(protocol_name, protocol_dict)
            blackboard.set(f"{protocol_name}_done", protocol_dict_done)
            continue

        # Legacy/special protocols keep low_level schema.
        low_level = protocol_data.get("low_level")
        if low_level is None:
            if debug:
                print("low_level is none for protocol : ", protocol_name)
            continue

        for key, value in low_level.items():
            if debug:
                print(key, value)
            protocol_dict[key] = value

            if key.startswith("type_"):
                continue
            if key.startswith("number_of_protocols"):
                continue

            if "exercise" in protocol_name and key != "get_confirmation":
                continue
            protocol_dict_done[f"{key}_done"] = False

        blackboard.set(protocol_name, protocol_dict)
        blackboard.set(f"{protocol_name}_done", protocol_dict_done)

    blackboard.set("_protocol_registry_names", sorted(protocols.keys()))

    if debug:
        for key, value in blackboard.storage.items():
            print(f"{key} : {value}")

    return blackboard
        
if __name__ == "__main__":
    from smart_home_pytree.utils import get_house_yaml_path

    yaml_file_path = get_house_yaml_path()
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
