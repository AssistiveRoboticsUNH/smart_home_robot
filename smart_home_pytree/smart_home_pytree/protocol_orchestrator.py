import os
import os
import yaml
import time
import random
import threading

import rclpy
from smart_home_pytree.trees.charge_robot_tree import ChargeRobotTree
from smart_home_pytree.robot_interface import get_robot_interface
import py_trees        
from smart_home_pytree.registry import load_protocols_to_bb
import re
import importlib
from datetime import datetime, timedelta
from smart_home_pytree.trigger_monitor import TriggerMonitor
    
# PROTOCOL ORCHESTRATOR 
class ProtocolOrchestrator:
    def __init__(self, robot_interface = None, test_time=None, debug=False):
        """_summary_

        Args:
            robot_interface (_type_, optional): _description_. Defaults to None.
            test_time (_type_, optional): _description_. Defaults to None.
            debug (bool, optional): _description_. Defaults to False.
            
            required_state_keys define what topics need to exist before the protocl should start
        """
        # rclpy.init()
        self.rclpy_initialized_here = False
        self.debug = debug
        
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
            self.robot_interface=get_robot_interface()
        else:
            self.robot_interface = robot_interface
        
        self.trigger_monitor = TriggerMonitor(self.robot_interface)
        
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
        
        self.last_satisfied = None

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

            # print("self.completed_protocols: ", self.trigger_monitor.completed_protocols)
            satisfied = self.trigger_monitor.get_satisfied()
                        
            if self.debug and satisfied != self.last_satisfied:
                print("[Orchestrator] Satisfied: ", satisfied)
                self.last_satisfied = satisfied
                
            next_protocol = min(satisfied, key=lambda x: x[1], default=None)
            # print("****************next_protocol: ", next_protocol)
            if not next_protocol:
                if self.running_tree:
                    ## if something is running this means it shouldnt run anymore and needs to be stopped
                    ## for now let it continue
                    time.sleep(3)
                    continue
                
                else:
                    if not self._state_is_ready():
                        print("[Orchestrator] Waiting for robot state to initialize...")
                        print("[Orchestrator] ********************** Waiting for robot state to initialize...")
                        time.sleep(1)
                        continue
                        
                    charging = self.robot_interface.state.get("charging")
                    if not charging:
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
                    print(f"************** mark_completed *************")
                    self.trigger_monitor.mark_completed(class_protocol_name)
            # else:
            #     ## wait sends false to make sure that it was polled and added to marked_completed
            #     self.trigger_monitor.collect_wait_requests()
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
            if self.debug: 
                print("[Orchestrator] Warning: Tried to start None protocol — skipping.")
            return
        
        class_protocol_name, priority = protocol_tuple
        
        if self.debug: 
            print(f"[Orchestrator] Starting: {class_protocol_name} (priority {priority})")

             
        if "ChargeRobotTree" in class_protocol_name:
            protocol_name = "" ## charge robot doesnt take a protocol name cause it doesnt depend on it
            tree_runner = ChargeRobotTree(node_name="charge_robot_tree",robot_interface=self.robot_interface)
        else:
            ## import tree dynamically
            tree_class_name = class_protocol_name.split(".")[0] ## example MoveAwayProtocolTree
            snake_case_class_name = self.camel_to_snake(tree_class_name)  ## turns it to snake case
            protocol_name = class_protocol_name.split(".")[-1] ## unique for a protocol ex: medicine_am
            
            if self.debug:
                print("**** tree_class_name : ", tree_class_name)
                print("**** snake_case_class_name : ", snake_case_class_name)
                print("**** protocol name: ", protocol_name)
                
            #  Dynamically import module & class
            try:
                module = importlib.import_module(f"smart_home_pytree.protocols.{snake_case_class_name}")
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
    orch = ProtocolOrchestrator(test_time="10:35")
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