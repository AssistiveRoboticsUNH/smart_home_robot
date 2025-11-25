import os
import os
import yaml
import time
import random
import threading
from datetime import datetime
import rclpy
from smart_home_pytree.trees.two_reminder_protocol import TwoReminderProtocolTree
from smart_home_pytree.trees.charge_robot_tree import ChargeRobotTree
from smart_home_pytree.robot_interface import get_robot_interface
from smart_home_pytree.trees.coffee_reminder_protocol import CoffeeReminderProtocolTree


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
        
         # Dynamically collect event keys from YAML
        self.event_keys = self._extract_event_keys()
        print(f"[TriggerMonitor] Loaded event keys from YAML: {self.event_keys}")

    def _extract_event_keys(self):
        """Collect all event topic names used in the YAML protocols."""
        event_keys = set()
        for _, protocol_data in self.protocols_yaml.get("protocols", {}).items():
            for _, sub_data in protocol_data.items():
                high_level_subdata = sub_data["high_level"]
                reqs = high_level_subdata.get("requirements", {})
                event_reqs = reqs.get("event", [])
                for ev in event_reqs:
                    if "topic" in ev:
                        event_keys.add(ev["topic"])
        return sorted(event_keys)
    
    def _collect_current_events(self):
        """Build a dict of current event states from RobotState."""
        current_events = {}
        for key in self.event_keys:
            key = key.lstrip("/") 
            val = self.robot_interface.state.get(key)
            print("key: ", key, "value: ", val)
            if val is None:
                print(f"[TriggerMonitor] Topic '{key}' not publishing or None → treating as False")
                current_events[key] = False
            else:
                current_events[key] = val
        return current_events

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
            topic, val = ev['topic'].lstrip("/"), ev['value']
            if topic not in current_events or current_events[topic] != val:
                return False
        return True

    def satisfied_protocols(self, current_events, current_time=None):
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
                
                # print("$$$$$ time_req: ", time_req)
                
                if self.check_event_requirement(event_reqs, current_events) and \
                   self.check_time_requirement(time_req, current_time):
                    priority = high_level_subdata.get('priority', 1)
                    satisfied.append((full_name, priority))
        satisfied.sort(key=lambda x: x[1])
        return satisfied

    def start_monitor(self, current_time: str = None):
        while not self.stop_flag:
         
            current_events = self._collect_current_events()
            new_satisfied = self.satisfied_protocols(current_events, current_time=current_time)
            
            print("**************** new_satisfied: ", new_satisfied)
            
            with self.lock:
                self.current_satisfied_protocols = new_satisfied

            time.sleep(2)  # small delay to avoid busy loop

    def mark_completed(self, protocol_name):
        """Mark a protocol as successfully completed to avoid retriggering."""
        with self.lock:
            print(f"[TriggerMonitor] Marking {protocol_name} as completed.")
            self.completed_protocols.add(protocol_name)
            
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
                print(" self.rclpy_initialized_here shoul;d be true: ", self.rclpy_initialized_here)
            except RuntimeError:
                # ROS2 already initialized somewhere else
                self.rclpy_initialized_here = False
                print(" self.rclpy_initialized_here shoul;d be false: ", self.rclpy_initialized_here)

        
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
       
    def orchestrator_loop(self):
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
                    charging = self.robot_interface.state.get("charging",None)
        
                    if charging is None:
                        print(f"[TriggerMonitor] Topic '{charging}' not publishing or None → treating as False")
                        charging = False
                        
                    if not charging:
                        # CHARGE THE ROBOT
                        self.start_protocol(("ChargeRobotTree",100))
                    

            if not self.running_tree:
                self.start_protocol(next_protocol)
            else:
                if next_protocol is None:
                    time.sleep(1)
                    continue
                if next_protocol[1] < self.running_tree["priority"]:
                    print(f"[Orchestrator] Higher-priority protocol detected: {next_protocol}")
                    self.stop_protocol()
                    self.start_protocol(next_protocol)

            time.sleep(1)

    def _run_protocol(self, tree_runner, protocol_name, priority):
        """Run a protocol tree and clean up when done."""
        try:
            tree_runner.run_until_done()
        finally:
            print("tree_runner.final_status", tree_runner.final_status)
            if (tree_runner.final_status ==  py_trees.common.Status.SUCCESS):
                
                if "ChargeRobotTree" not in protocol_name:
                    self.trigger_monitor.mark_completed(protocol_name)
                
            # tree_runner.final_status == py_trees.common.Status.SUCCESS

            with self.lock:
                time.sleep(3)
                print(f"[Orchestrator] Finished: {protocol_name}")
                tree = self.running_tree["tree"]
                tree.nodes_cleanup()
                self.running_tree = None
                self.running_thread = None


    def start_protocol(self, protocol_tuple):
        """Start the protocol in its own thread."""
        if protocol_tuple is None: 
            print("[Orchestrator] Warning: Tried to start None protocol — skipping.")
            return
        
        print("protocol_tuple: ", protocol_tuple)
        protocol_name, priority = protocol_tuple
        sub_name = protocol_name.split(".")[-1]
        print(f"[Orchestrator] Starting: {protocol_name} (priority {priority})")

        # Choose the correct tree type
        print("**** sub_name: ", sub_name)
        if "TwoReminderProtocol" in protocol_name:
            tree_runner = TwoReminderProtocolTree(
                node_name="two_reminder_protocol_tree", protocol_name=sub_name, robot_interface=self.robot_interface 
            )
        elif "CoffeeProtocol" in protocol_name:
            tree_runner = CoffeeReminderProtocolTree(
                node_name="coffee_protocol_tree", protocol_name=sub_name, robot_interface=self.robot_interface 
            )
        elif "ChargeRobotTree" in protocol_name:
            tree_runner = ChargeRobotTree(node_name="charge_robot_tree",robot_interface=self.robot_interface)
        else:
            print(f"[Orchestrator] No matching tree for {protocol_name}")
            return

        tree_runner.setup()
        thread = threading.Thread(
            target=self._run_protocol,
            args=(tree_runner, protocol_name, priority),
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

        
import py_trees        
from smart_home_pytree.registry import load_protocols_to_bb
if __name__ == "__main__":
    import time
    yaml_file_path = os.getenv("house_yaml_path", None)
    blackboard = py_trees.blackboard.Blackboard()
    load_protocols_to_bb(yaml_file_path)
    # For testing:
    orch = ProtocolOrchestrator(test_time="10:30")

    # For live use:
    # orch = ProtocolOrchestrator()

    # # Mock data: pretend these are current satisfied protocols
    # orch.trigger_monitor.get_satisfied = lambda: [
    #     ("TwoReminderProtocol.medicine_am", 1)
    # ]

    try:
        orch.orchestrator_loop()
    except KeyboardInterrupt:
        print("Shutting down orchestrator...")
        orch.shutdown()
