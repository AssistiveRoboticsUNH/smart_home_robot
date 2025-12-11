#!/usr/bin/env python3
import yaml
import py_trees
import py_trees_ros

def load_locations_to_blackboard(yaml_path: str):
    """
    Load location data from a YAML file and register it to the py_trees blackboard.
    """
    
    # Get blackboard
    blackboard = py_trees.blackboard.Blackboard()
    
    try:
        blackboard.get('initialized')
        return blackboard
    except:
        pass
    # ------------------------
    
    # Load YAML
    with open(yaml_path, 'r') as file:
        data = yaml.safe_load(file)

    if "locations" not in data:
        raise KeyError("YAML file must contain a 'locations' field.")

    locations = data["locations"]
    
    # Register to blackboard
    blackboard.set("locations", locations)

    print("Registered the following locations to the blackboard:")
    for name, loc in locations.items():
        print(f"  {name}: {loc}")

    # Set flag (singleton-style marker)
    blackboard.set("initialized", True) 
    print("[Blackboard] Registered 'locations' once only.")
    
    return blackboard


def load_protocols_to_bb(yaml_path: str):  
    """
    Load protocol related data from a YAML file and register it to the py_trees blackboard.
    also sets done flags to false for each action in the protocol.
    """
    # Get blackboard
    blackboard = py_trees.blackboard.Blackboard()
    
    # Load YAML
    with open(yaml_path, 'r') as file:
        data = yaml.safe_load(file)
        
       # Ensure structure is correct
    if "protocols" not in data:
        raise KeyError("YAML must contain protocols -> TwoReminderProtocol structure.")
    
    protocols = data["protocols"]
    
    for protocol_type in protocols.keys():
        print("protocol type: ", protocol_type)
        for protocol_name in protocols[protocol_type].keys():
            print("protocol name : ", protocol_name)
            protocol_dict = {}
            protocol_dict_done = {}
            
            low_level = protocols[protocol_type][protocol_name]["low_level"]
            for key, value in low_level.items():
                print(key, value)
                protocol_dict[key] = value
                
                if key.startswith("type_"):
                # Skip creating *_done entry
                    continue
                
                if key.startswith("number_of_protocols"):
                # Skip creating *_done entry
                    continue
                
                if "exercise" in protocol_name:
                    print("protocol_name")
                    continue
                
                protocol_dict_done[f"{key}_done"] = False
                    
            blackboard.set(protocol_name, protocol_dict)
            
            if "exercise" in protocol_name:
                    print("protocol_name")
                    continue
            blackboard.set(f"{protocol_name}_done", protocol_dict_done)
    
    for key, value in blackboard.storage.items():
        print(f"{key} : {value}")
    

    
    return blackboard
    

import os
if __name__ == "__main__":
    # yaml_file_path = "/home/olagh48652/smart_home_pytree_ws/src/smart_home_pytree/config/house_info.yaml"
    
    yaml_file_path = os.getenv("house_yaml_path", None) 
    print("yaml_file_path", yaml_file_path)
    bb = load_locations_to_blackboard(yaml_file_path)
    print("bb: ",bb)
    
    print("\n--- Blackboard raw storage ---")
    print(py_trees.blackboard.Blackboard.storage)
    
    target_location_name = "kitchen"
    target_location = bb.get("locations")[target_location_name]

    x = target_location["x"]
    y = target_location["y"]
    quat_vals = target_location["quat"]

    print("x: ", x, " y: ", y, " quat: ", quat_vals)
    
    
    load_protocols_to_bb(yaml_file_path)