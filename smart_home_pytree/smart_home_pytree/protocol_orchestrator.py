import os
import time
import threading
import rclpy
from smart_home_pytree.robot_interface import get_robot_interface
import py_trees        
from smart_home_pytree.registry import load_protocols_to_bb
import re
import importlib
from smart_home_pytree.trigger_monitor import TriggerMonitor
from smart_home_pytree.human_interface import HumanInterface

  
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
            try:
                rclpy.init(args=None)
                self.rclpy_initialized_here = True
                print(" self.rclpy_initialized_here should be true: ", self.rclpy_initialized_here)
            except RuntimeError:
                self.rclpy_initialized_here = False
                print(" self.rclpy_initialized_here should be false: ", self.rclpy_initialized_here)

        # Setup Events FIRST
        self.orchestrator_wakeup = threading.Event()
        self.human_interrupt_event = threading.Event()
        
        self.running_tree = None
        self.running_thread = None
        self.lock = threading.Lock()
        self.stop_flag = False
        self.state_ready = False
        self.required_state_keys = ["charging"]
        self.last_satisfied = None
        
        # Setup HumanInterface (CRITICAL: Create BEFORE Robot Interface spins)
        self.human_interface_node = HumanInterface(
            human_interrupt_event=self.human_interrupt_event,
            orchestrator_wakeup=self.orchestrator_wakeup,
            robot_interface=None # Pass None for now to prevent dependency loop
        )

        # 4. Setup RobotInterface (Starts the background thread)
        if robot_interface is None:
            self.robot_interface = get_robot_interface()
        else:
            self.robot_interface = robot_interface
            
        # Patch the Link
        self.human_interface_node.robot_interface = self.robot_interface
        
        # Setup TriggerMonitor
        self.trigger_monitor = TriggerMonitor(
            self.robot_interface, 
            wake_event=self.orchestrator_wakeup 
        )
        
        # Start Threads
        self.monitor_thread = threading.Thread(
            target=self.trigger_monitor.start_monitor,
            kwargs={"current_time": test_time},
            daemon=True
        )
        self.monitor_thread.start()
        
        self.ros_spin_thread = threading.Thread(
            target=self._spin_human_interface,
            daemon=True
        )
        self.ros_spin_thread.start()
        
    # TODO: use multi thread executor but robot interface need to stop spinning itself
    # Create one executor for EVERYTHING
    # self.executor = MultiThreadedExecutor()
    # self.executor.add_node(self.human_interface_node)
    # self.executor.add_node(self.robot_interface) # Assuming it inherits form Node
    
    # # Run the executor in a background thread
    # self.ros_spin_thread = threading.Thread(
    #     target=self.executor.spin, 
    #     daemon=True
    # )
    # self.ros_spin_thread.start()
    
    def _spin_human_interface(self):
        """Runs the ROS node for Human Interface."""
        print("[Orchestrator] Starting Human Interface Listener Thread...")
        try:
            rclpy.spin(self.human_interface_node)
        except Exception as e:
            print(f"[Orchestrator] Human Interface Thread Error: {e}")
               
    def _state_is_ready(self):
        for key in self.required_state_keys:
            if self.robot_interface.state.get(key) is None:
                return False
        return True

    def orchestrator_loop(self, mock: bool = False):
        if self.debug:
            print("[Orchestrator] Starting orchestrator loop")

        while not self.stop_flag:
            
            if not self._state_is_ready():
                ## this is supposed to prevent orchestartor from starting until prerequisites are up
                if self.debug:
                    print("[Orchestrator] State not ready, waiting...")
                time.sleep(2)
                continue

            # Block until something changes
            if self.debug:
                print("[Orchestrator] Waiting for wakeup event...")
                
            self.orchestrator_wakeup.wait()
            self.orchestrator_wakeup.clear()

            if self.debug:
                print("[Orchestrator] Wakeup received")
            
            # --- Human interrupt has absolute priority ---
            if self.human_interrupt_event.is_set():
                if self.debug:
                    print("[Orchestrator] Human interrupt active")

                if self.running_tree:
                    if self.debug:
                        print(f"[Orchestrator] Preempting running protocol: {self.running_tree['name']}")
                    self.stop_protocol()

                # Block here until human clears the flag. Use the wakeup event to wake us when they say "IDLE".
                self.orchestrator_wakeup.wait()
                self.orchestrator_wakeup.clear()
                continue # Restart loop immediately

            # --- Cleanup finished protocol ---
            if self.running_thread and not self.running_thread.is_alive():
                if self.debug:
                    print("[Orchestrator] Detected finished protocol thread. Cleaning it up")
                with self.lock:
                    self.running_tree = None
                    self.running_thread = None

            satisfied = self.trigger_monitor.get_satisfied()
            if self.debug and satisfied != self.last_satisfied:
                print("[Orchestrator] Satisfied: ", satisfied)
                self.last_satisfied = satisfied
    
            next_protocol = min(satisfied, key=lambda x: x[1], default=None)

            if not next_protocol:
                if self.debug:
                    print("[Orchestrator] No satisfied protocols")
                continue

            if not self.running_tree:
                if self.debug:
                    print(f"[Orchestrator] Starting protocol {next_protocol}")
                self.start_protocol(next_protocol)
            else:
                if next_protocol[1] < self.running_tree["priority"]:
                    if self.debug:
                        print(f"[Orchestrator] Running  higher-priority protocol {next_protocol}")
                    self.stop_protocol()
                    self.start_protocol(next_protocol)

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
  

            with self.lock:
                self.orchestrator_wakeup.set()
                # time.sleep(3) ## updated mark_completed to update the current satisfied protocols
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
        
        ## using charging as protocl now for polling  
        # if "ChargeRobotTree" in class_protocol_name:
        #     protocol_name = "" ## charge robot doesnt take a protocol name cause it doesnt depend on it
        #     tree_runner = ChargeRobotTree(node_name="charge_robot_tree",robot_interface=self.robot_interface)
        # else:
        
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
        
        if hasattr(self, 'human_interface_node'):
            self.human_interface_node.destroy_node()

        if self.running_tree:
            self.stop_protocol()

        if self.rclpy_initialized_here:
            print("[Orchestrator] rclpy Shutdown.")
            rclpy.shutdown()
            
        # Since ROS is down, spin() has returned, and the thread is ready to exit.
        # if self.ros_spin_thread.is_alive():
        #     self.ros_spin_thread.join() # will die automatically since deamon is set to true and rlcpy .shutdown is triggered
            
        print("[Orchestrator] Shutdown complete.")


if __name__ == "__main__":
    import time
    yaml_file_path = os.getenv("house_yaml_path", None)
    blackboard = py_trees.blackboard.Blackboard()
    load_protocols_to_bb(yaml_file_path)

    # For testing:
    orch = ProtocolOrchestrator(test_time="9:30", debug=True)
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