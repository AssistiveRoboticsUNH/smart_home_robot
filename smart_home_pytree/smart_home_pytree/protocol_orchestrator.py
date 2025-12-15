import os
import os
import yaml
import time
import random
import threading
from datetime import datetime
import rclpy
# from smart_home_pytree.trees.two_reminder_protocol import TwoReminderProtocolTree
from smart_home_pytree.trees.charge_robot_tree import ChargeRobotTree
from smart_home_pytree.robot_interface import get_robot_interface
# from smart_home_pytree.trees.coffee_reminder_protocol import CoffeeReminderProtocolTree
# from smart_home_pytree.trees.move_away_protocol import MoveAwayProtocolTree
import py_trees        
from smart_home_pytree.registry import load_protocols_to_bb
import re
import importlib

#  TRIGGER MONITOR 
class TriggerMonitor:
    def __init__(self, robot_interface, yaml_path=None):
        
        self.robot_interface = robot_interface
        
        if yaml_path is None:
            yaml_path = os.getenv("house_yaml_path")
        # yaml_path = "/home/olagh48652/smart_home_pytree_ws/src/smart_home_pytree/config/house_info copy.yaml"
        
        with open(yaml_path, "r") as f:
            self.protocols_yaml = yaml.safe_load(f)

        self.current_satisfied_protocols = []  # [(protocol_name, priority)]
        self.lock = threading.Lock()
        self.stop_flag = False
        self.completed_protocols = set()  # track completed ones
        
        self.protocols_to_reset = set()  # track ones that need resetting
        # self.protocols_to_reset should include (protocl_name, time it fines, after how much time to reset)
        
         # Dynamically collect event keys from YAML
        self.event_keys = self._extract_event_keys()
        print(f"[TriggerMonitor] Loaded event keys from YAML: {self.event_keys}")
        
        self.blackboard = py_trees.blackboard.Blackboard()

    # -------- Helper functions to reset protocols --------
    def reset_specific_protocol_dones(self, protocol_name: str):
        """
        Reset the <protocol_name>_done dictionary in the py_trees blackboard.
        """
        key = f"{protocol_name}_done"

        # Ensure it exists
        value = self.blackboard.get(key)
        if value is None:
            print(f"[reset] No such protocol done key: {key}")
            return

        if not isinstance(value, dict):
            print(f"[reset] Key '{key}' exists but is not a dictionary.")
            return

        # Reset each entry to False
        reset_dict = {sub_key: False for sub_key in value.keys()}
        self.blackboard.set(key, reset_dict)

        print(f"[reset] Reset {key} → {reset_dict}")

    def reset_all_protocol_dones(self):
        """
        Reset all *_done dictionaries in the py_trees blackboard.
        """
        for key, value in list(self.blackboard.storage.items()):
            # Look only for keys ending with "_done"
            if key.endswith("_done") and isinstance(value, dict):
                reset_dict = {sub_key: False for sub_key in value.keys()}
                self.blackboard.set(key, reset_dict)    
                print(f"Reset {key} → {reset_dict}")
    
    def set_complete_specific_protocol(self, protocol_name: str):
        """
        Set the <protocol_name>_done dictionary in the py_trees blackboard to true.
        """
        key = f"{protocol_name}_done"

        # Ensure it exists
        value = self.blackboard.get(key)
        if value is None:
            print(f"[reset] No such protocol done key: {key}")
            return

        if not isinstance(value, dict):
            print(f"[reset] Key '{key}' exists but is not a dictionary.")
            return

        # Set each entry to True
        reset_dict = {sub_key: True for sub_key in value.keys()}
        self.blackboard.set(key, reset_dict)

        print(f"[reset] Reset {key} → {reset_dict}")

  
    # -------- Helper functions to get protocols to activate --------
    def _extract_event_keys(self):
        """Collect all event topic names used in the YAML protocols."""
        event_keys = set()
        for _, protocol_data in self.protocols_yaml.get("protocols", {}).items():
            for _, sub_data in protocol_data.items():
                high_level_subdata = sub_data["high_level"]
                reqs = high_level_subdata.get("requirements", {})
                event_reqs = reqs.get("event", [])
                for ev in event_reqs:
                    if "state" in ev:
                        event_keys.add(ev["state"])
        return sorted(event_keys)
    
    def _collect_current_events(self):
        """Build a dict of current event states from RobotState."""
        current_events = {}
        for key in self.event_keys:
            val = self.robot_interface.state.get(key)
            print("val")
            print("key: ", key, "value: ", val)
            if val is None:
                print(f"[TriggerMonitor] Topic '{key}' not publishing or None → treating as False")
                current_events[key] = False
            else:
                current_events[key] = val
        return current_events

    def check_day_requirement(self, day_req, current_day=None):
        if not day_req or len(day_req) == 0:
            return True
        
        if current_day is None:
            current_day = datetime.now().strftime("%A")
        
        if current_day.lower() not in [d.lower() for d in day_req]:
            return False
       
        return True
    
    def check_time_requirement(self, time_req, current_time=None):
        if not time_req:
            return True
        if current_time is None:
            current_time = datetime.now().strftime("%H:%M")
            
        fmt = "%H:%M"
        t_from = datetime.strptime(time_req['from'], fmt)
        t_to = datetime.strptime(time_req['to'], fmt)
        t_now = datetime.strptime(current_time, fmt)
        
        return t_from <= t_now <= t_to

    def check_event_requirement(self, event_reqs, current_events):
        if not event_reqs:
            return True
        for ev in event_reqs:
            topic, val = ev['state'], ev['value']
            if topic not in current_events or current_events[topic] != val:
                return False
        return True

    def satisfied_protocols(self, current_events, current_day=None, current_time=None):
        satisfied = []
        for protocol_name, protocol_data in self.protocols_yaml['protocols'].items():
            # print("protocol_name: ", protocol_name)
            for sub_name, sub_data in protocol_data.items():
                # print("sub_name: ", sub_name)
                high_level_subdata = sub_data["high_level"]
                full_name = f"{protocol_name}.{sub_name}"
                if full_name in self.completed_protocols:  # skip completed ones
                    continue
                
                reqs = high_level_subdata.get('requirements', {})
                event_reqs = reqs.get('event', [])
                time_req = reqs.get('time', {})
                day_req = reqs.get('day', [])
                # print("$$$$$ time_req: ", time_req)
                
                if self.check_day_requirement(day_req, current_day) and \
                    self.check_event_requirement(event_reqs, current_events) and \
                    self.check_time_requirement(time_req, current_time):
                       
                    priority = high_level_subdata.get('priority', 1)
                    satisfied.append((full_name, priority))
        satisfied.sort(key=lambda x: x[1])
        return satisfied

    def start_monitor(self, current_time: str = None):
        last_day = datetime.now().strftime("%Y-%m-%d")
        
        while not self.stop_flag:
            
            # Check for daily reset
            today = datetime.now().strftime("%Y-%m-%d")
            if today != last_day:
                print("[TriggerMonitor] New day → resetting protocol done flags.")
                self.reset_all_protocol_dones()
                last_day = today
            
            # resets periodic protocols in reset array
            self.check_and_reset_protocols()
            
            current_events = self._collect_current_events()
            new_satisfied = self.satisfied_protocols(current_events, current_time=current_time)
            
            print("**************** new_satisfied: ", new_satisfied)
            
            with self.lock:
                self.current_satisfied_protocols = new_satisfied

            time.sleep(2)  # small delay to avoid busy loop

    def parse_reset_pattern(self, reset_pattern):
        """
        Convert a reset_pattern dict into total seconds.
        Safe for missing fields.
        Examples:
            {"hours": 1, "minutes": 30} → 5400
            {"minutes": 30}             → 1800
            {"hours": 1}                → 3600
            None or {}                  → None
        """
        if not reset_pattern:
            return None

        hours = reset_pattern.get("hours", 0)
        minutes = reset_pattern.get("minutes", 0)

        # Validate types
        if not isinstance(hours, (int, float)) or not isinstance(minutes, (int, float)):
            print(f"[reset_pattern] Invalid format: {reset_pattern}")
            return None

        total_seconds = int(hours * 3600 + minutes * 60)

        return total_seconds if total_seconds > 0 else None

    def mark_completed(self, protocol_name):
        """Mark a protocol as successfully completed to avoid retriggering."""
        with self.lock:
            print(f"[TriggerMonitor] Marking {protocol_name} as completed.")
            
            
            # ---- Split the name correctly ----
            try:
                main, sub = protocol_name.split(".")
            except ValueError:
                print(f"[ERROR] Invalid protocol name format: {protocol_name}")
                return
            
            # ---- Look up YAML entry safely ----
            try:
                sub_data = self.protocols_yaml['protocols'][main][sub]
            except KeyError:
                print(f"[ERROR] Protocol not found in YAML: {protocol_name}")
                return
        
            high_level = sub_data["high_level"]
            reset_pattern = high_level.get('reset_pattern', None)
            
            if reset_pattern is None:
                print(f"[TriggerMonitor] No reset pattern for {protocol_name}")
                return

            pattern_type = reset_pattern.get("type", "periodic")  # default to periodic if not given
            # INSTANT -> DONT ADD IT TO COMPLETED
            print("pattern_type: ", pattern_type)
            if pattern_type == "instant":
                return
            
            self.completed_protocols.add(protocol_name)
            
            # PERIODIC 
            if pattern_type != "periodic":
                return
            
            timestamp = datetime.now()
            
            reset_seconds = self.parse_reset_pattern(reset_pattern)
            print("protocol_name: ", protocol_name, " timestamp: ", timestamp, " reset_pattern: ", reset_seconds)
            self.protocols_to_reset.add(
                (protocol_name, timestamp, reset_seconds)
            )
            print(f"[TriggerMonitor] Scheduled reset for {protocol_name} in {reset_pattern}")

    def check_and_reset_protocols(self):
        """Check all protocols scheduled for auto-reset and reset those whose timer expired."""
        now = datetime.now()
        to_remove = []

        for (name, finished_time, reset_seconds) in list(self.protocols_to_reset):
            if (now - finished_time).total_seconds() >= reset_seconds:
                print(f"[TriggerMonitor] Auto-resetting protocol: {name}")

                sub_name = name.split(".")[-1]
                self.reset_specific_protocol_dones(sub_name)
                
                if name in self.completed_protocols:   
                    self.completed_protocols.remove(name)
                else:
                    print(f"[TriggerMonitor] Warning protocol {name} not in completed_protocols")

                to_remove.append((name, finished_time, reset_seconds))

        # Remove after iteration (safe)
        for entry in to_remove:
            print(f"[reset] ************ protocol to reset : {entry}")
            self.protocols_to_reset.remove(entry)
        
    def get_satisfied(self):
        with self.lock:
            return list(self.current_satisfied_protocols)

    def stop_monitor(self):
        self.stop_flag = True

    
# PROTOCOL ORCHESTRATOR 
class ProtocolOrchestrator:
    def __init__(self, robot_interface = None, test_time=None):
        # rclpy.init()
        self.rclpy_initialized_here = False

        if not rclpy.ok():
            ## just for safety
            try:
                rclpy.init(args=None)
                self.rclpy_initialized_here = True
                print(" self.rclpy_initialized_here should be true: ", self.rclpy_initialized_here)
            except RuntimeError:
                # ROS2 already initialized somewhere else
                self.rclpy_initialized_here = False
                print(" self.rclpy_initialized_here should be false: ", self.rclpy_initialized_here)

        
        if  robot_interface is None:
            print("initialize robot interface")
            self.robot_interface=get_robot_interface()
        else:
            print("using robot interface provided")
            self.robot_interface = robot_interface
        
        self.trigger_monitor = TriggerMonitor(self.robot_interface)
        
        ## need to pass argument
        # self.monitor_thread = threading.Thread(target=self.trigger_monitor.start_monitor, daemon=True)
        self.monitor_thread = threading.Thread(
            target=self.trigger_monitor.start_monitor,
            kwargs={"current_time": test_time},  # can be None or e.g. "10:30"
            daemon=True
        )
        
        self.monitor_thread.start()
    
        self.running_tree = None
        self.running_thread = None
        self.lock = threading.Lock()
        self.stop_flag = False

        ## define keys that has to be observed for the tree to work
        self.state_ready = False
        self.required_state_keys = ["charging"]

    def _state_is_ready(self):
        for key in self.required_state_keys:
            if self.robot_interface.state.get(key) is None:
                return False
        return True

       
    def orchestrator_loop(self, mock: bool = False):
        """Main loop that manages protocol execution."""
        while not self.stop_flag:
            # --- cleanup if running thread finished ---
            if self.running_thread and not self.running_thread.is_alive():
                with self.lock:
                    print(f"[Orchestrator] Protocol {self.running_tree['name']} completed.")
                    self.running_tree = None
                    self.running_thread = None

            print("self.completed_protocols: ", self.trigger_monitor.completed_protocols)
            satisfied = self.trigger_monitor.get_satisfied()
            
            print("########### satisfied: ", satisfied)
            next_protocol = min(satisfied, key=lambda x: x[1], default=None)
            print("****************next_protocol: ", next_protocol)
            if not next_protocol:
                if self.running_tree:
                    ## if something is running this means it shouldnt run anymore and needs to be stopped
                    ## for now let it continue
                    time.sleep(3)
                    continue
                
                else:
                    ## Check if the robot is charging
                    # charging = self.robot_interface.state.get("charging",None)
                    if not self._state_is_ready():
                        print("[Orchestrator] Waiting for robot state to initialize...")
                        time.sleep(1)
                        continue
        
                    # if charging is None:
                    #     print(f"[TriggerMonitor] Topic '{charging}' not publishing or None → treating as False")
                    #     charging = False
                        
                    charging = self.robot_interface.state.get("charging")
                    if not charging:
                        # CHARGE THE ROBOT
                        # self.start_protocol(("ChargeRobotTree",100))
                        if mock:
                            self.start_protocol_mock(("ChargeRobotTree",100))
                        else:
                            self.start_protocol(("ChargeRobotTree",100))
                    

            if not self.running_tree:
                if mock:
                    self.start_protocol_mock(next_protocol)
                else:
                    self.start_protocol(next_protocol)
            else:
                if next_protocol is None:
                    time.sleep(1)
                    continue
                if next_protocol[1] < self.running_tree["priority"]:
                    print(f"[Orchestrator] Higher-priority protocol detected: {next_protocol}")
                    self.stop_protocol()
                    self.start_protocol(next_protocol)

            time.sleep(3) ## more than start_monitor that updated self.satisfied

    def _run_protocol(self, tree_runner, class_protocol_name, priority):
        """Run a protocol tree and clean up when done."""
        try:
            tree_runner.run_until_done()
        finally:
            print("tree_runner.final_status", tree_runner.final_status)
            if (tree_runner.final_status ==  py_trees.common.Status.SUCCESS):
                
                if "ChargeRobotTree" not in class_protocol_name:
                    self.trigger_monitor.mark_completed(class_protocol_name)
                
            # tree_runner.final_status == py_trees.common.Status.SUCCESS

            with self.lock:
                time.sleep(3)
                print(f"[Orchestrator] Finished: {class_protocol_name}")
                tree = self.running_tree["tree"]
                tree.nodes_cleanup()
                self.running_tree = None
                self.running_thread = None

    

    def camel_to_snake(self, name: str) -> str:
        """Convert CamelCase to snake_case."""
        s1 = re.sub(r'(.)([A-Z][a-z]+)', r'\1_\2', name)
        s2 = re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', s1)
        return s2.lower()

    def start_protocol_mock(self, protocol_tuple):
        """Start the protocol in its own thread."""
        if protocol_tuple is None: 
            print("[Orchestrator] Warning: Tried to start None protocol — skipping.")
            return
        
        print("protocol_tuple: ", protocol_tuple)
        protocol_name, priority = protocol_tuple
        sub_name = protocol_name.split(".")[-1]
        print(f"[Orchestrator] Starting: {protocol_name} (priority {priority})")
        print("sub_name", sub_name)
        print("protocol_name mega", protocol_name.split(".")[0])
        
        
        if "TwoReminderProtocol" in protocol_name:
            print(f"[Orchestrator]  two_reminder_protocol_tree for {protocol_name}")
            self.trigger_monitor.set_complete_specific_protocol(sub_name)
            self.trigger_monitor.mark_completed(protocol_name)
            
        elif "CoffeeProtocol" in protocol_name:
            print(f"[Orchestrator]  coffee_protocol_tree for {protocol_name}")
            self.trigger_monitor.set_complete_specific_protocol(sub_name)
            self.trigger_monitor.mark_completed(protocol_name)
        elif "MoveAwayProtocol" in protocol_name:
            self.trigger_monitor.set_complete_specific_protocol(sub_name)
            self.trigger_monitor.mark_completed(protocol_name)
        elif "ChargeRobotTree" in protocol_name:
            print(f"[Orchestrator]  charge_robot_tree for {protocol_name}")
        else:
            print(f"[Orchestrator] No matching tree for {protocol_name}")
            return


    def start_protocol(self, protocol_tuple):
        """Start the protocol in its own thread."""
        if protocol_tuple is None: 
            print("[Orchestrator] Warning: Tried to start None protocol — skipping.")
            return
        
        print("protocol_tuple: ", protocol_tuple)
        class_protocol_name, priority = protocol_tuple
        
        print(f"[Orchestrator] Starting: {class_protocol_name} (priority {priority})")

             
        if "ChargeRobotTree" in class_protocol_name:
            protocol_name = "" ## charge robot doesnt take a protocol name cause it doesnt depend on it
            tree_runner = ChargeRobotTree(node_name="charge_robot_tree",robot_interface=self.robot_interface)
        else:
            ### load based on name where name is
            ## import tree dynamically
            
            tree_class_name = class_protocol_name.split(".")[0] ## example MoveAwayProtocolTree
            snake_case_class_name = self.camel_to_snake(tree_class_name)  ## turns it to snake case
            protocol_name = class_protocol_name.split(".")[-1] ## unique for a protocol ex: medicine_am
            
            print("**** tree_class_name : ", tree_class_name)
            print("**** snake_case_class_name : ", snake_case_class_name)
            print("**** protocol name: ", protocol_name)
            #  Dynamically import module & class
            try:
                module = importlib.import_module(f"smart_home_pytree.trees.{snake_case_class_name}")
                tree_class = getattr(module, f"{tree_class_name}Tree") ## gets the class from the file in module
            except Exception as e:
                print(f"[Orchestrator] Failed to load tree '{tree_class_name}Tree' from module smart_home_pytree.trees.{snake_case_class_name} : {e}")
                return 

            # Instantiate the tree
            tree_runner = tree_class(
                node_name=snake_case_class_name,
                protocol_name=protocol_name,
                robot_interface=self.robot_interface
            )

        tree_runner.setup()
        thread = threading.Thread(
            target=self._run_protocol,
            args=(tree_runner, class_protocol_name, priority),
            daemon=True,
        )
        thread.start()

        with self.lock:
            self.running_tree = {"name": protocol_name, "priority": priority, "tree": tree_runner}
            self.running_thread = thread

    def stop_protocol(self):
        """Stop the currently running protocol."""
        with self.lock:
            if not self.running_tree:
                return
            name = self.running_tree["name"]
            print(f"[Orchestrator] Stopping: {name}")
            tree = self.running_tree["tree"]

        tree.stop_tree()
        self.running_thread.join(timeout=5)
        tree.cleanup()

        with self.lock:
            self.running_tree = None
            self.running_thread = None

    def shutdown(self):
        """Gracefully stop everything."""
        print("[Orchestrator] Shutting down...")
        self.stop_flag = True
        self.trigger_monitor.stop_monitor()
        self.monitor_thread.join(timeout=10)

        if self.running_tree:
            self.stop_protocol()

        if self.rclpy_initialized_here:
            print("[Orchestrator] rclpy Shutdown.")
            rclpy.shutdown()
        print("[Orchestrator] Shutdown complete.")


if __name__ == "__main__":
    import time
    yaml_file_path = os.getenv("house_yaml_path", None)
    blackboard = py_trees.blackboard.Blackboard()
    load_protocols_to_bb(yaml_file_path)
    # For testing:
    orch = ProtocolOrchestrator(test_time="15:30")
    # orch = ProtocolOrchestrator()
    # For live use:
    # orch = ProtocolOrchestrator()

    # # Mock data: pretend these are current satisfied protocols
    # orch.trigger_monitor.get_satisfied = lambda: [
    #     ("TwoReminderProtocol.medicine_am", 1)
    # ]

    try:
        orch.orchestrator_loop(mock=False)
    except KeyboardInterrupt:
        print("Shutting down orchestrator...")
        orch.shutdown()
        
        
        
"""
ros2 topic pub /display_rx std_msgs/msg/String "data: 'exercise_requested'" 
ros2 topic pub /display_rx std_msgs/msg/String "data: 'exercise_stop'" 

"""