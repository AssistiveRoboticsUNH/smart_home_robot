import argparse
import importlib
import os
import re
import threading
import time
from datetime import datetime

import py_trees
import rclpy
from rclpy.executors import MultiThreadedExecutor

from smart_home_pytree.human_interface import HumanInterface
from smart_home_pytree.registry import load_protocols_to_bb, load_locations_to_blackboard
from smart_home_pytree.robot_interface import RobotInterface
from smart_home_pytree.trigger_monitor import TriggerMonitor
from smart_home_pytree.utils import BlackboardLogger


class ProtocolOrchestrator:
    # pylint: disable=too-many-instance-attributes
    """
    Manages the lifecycle of Behavior Tree protocols based on triggers and events.

    This class runs a main event loop that monitors triggers, handles human
    interruptions, and manages the execution threads of specific protocol trees.
    """

    def __init__(self, robot_interface=None, test_time: str = "", debug=False):
        """
        Initialize the Orchestrator.

        Args:
            robot_interface (object, optional): Pre-existing robot interface.
                Defaults to None (will create new one).
            test_time (str, optional): Simulated time for testing (e.g., "09:00").
                Defaults to "".
            debug (bool, optional): Enable verbose logging. Defaults to False.
        """
        self.rclpy_initialized_here = False
        self.debug = debug

        if not rclpy.ok():
            try:
                rclpy.init(args=None)
                self.rclpy_initialized_here = True
                print(
                    " self.rclpy_initialized_here should be true: ",
                    self.rclpy_initialized_here,
                )
            except RuntimeError:
                self.rclpy_initialized_here = False
                print(
                    " self.rclpy_initialized_here should be false: ",
                    self.rclpy_initialized_here,
                )
        
        
        # Events
        self.orchestrator_wakeup = threading.Event()
        self.human_interrupt_event = threading.Event()

        self.running_tree = None
        self.running_thread = None
        self.lock = threading.Lock()
        self.stop_flag = False
        self.state_ready = False
        self.required_state_keys = ["charging"]
        self.last_satisfied = None

        self.robot_interface = robot_interface or RobotInterface()
        
        # Setup logger
        blackboard = py_trees.blackboard.Blackboard()

        # 3. Create  Custom Logger
        # If robot_interface is a node, it uses it. If not, it uses print.
        custom_logger = BlackboardLogger(node=self.robot_interface, debug_mode=debug)

        # 4. SAVE IT GLOBALLY
        # Saved globally so 'logger' is available to every single Python file in project
        blackboard.set("logger", custom_logger)
        self.bb_logger = blackboard.get("logger")
        
        self.bb_logger.notify_discord("[Orchestrator] Orchestrator initilized")          

        # Setup HumanInterface 
        self.human_interface_node = HumanInterface(
            human_interrupt_event=self.human_interrupt_event,
            orchestrator_wakeup=self.orchestrator_wakeup,
            robot_interface=self.robot_interface,
        )
        
        # Setup TriggerMonitor
        self.trigger_monitor = TriggerMonitor(
            self.robot_interface,
            wake_event=self.orchestrator_wakeup,
            test_time=test_time,
        )

        # ---- SINGLE executor for EVERYTHING ----
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.robot_interface)
        self.executor.add_node(self.human_interface_node)

        # ---- ONE spin thread ----
        self.ros_spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.ros_spin_thread.start()

        # Non-ROS threads
        self.monitor_thread = threading.Thread(
            target=self.trigger_monitor.start_monitor, daemon=True
        )
        self.monitor_thread.start()

    def _state_is_ready(self):
        for key in self.required_state_keys:
            if self.robot_interface.state.get(key) is None:
                return False
        return True

    def orchestrator_loop(self):
        """
        Reactive Event Loop.
        Blocks until an event occurs (Human Voice or Trigger Change), then acts.
        """
        self.bb_logger.notify_discord("[Orchestrator] Starting Orchestrator loop")          

        while not self.stop_flag:
            # if self.debug:
            #     print("[Orchestrator] while of orchestrator_loop...")
            self.bb_logger.debug("[Orchestrator] In the while of orchestrator_loop...") 

            # 1. Gatekeeper: Doesn't do anything if robot isn't ready
            if not self._state_is_ready():
                self.bb_logger.info("[Orchestrator]  Robot state not ready. Waiting...")    
                time.sleep(2)
                continue

            # 2. THE BLOCK: Wait here until HumanInterface or TriggerMonitor wakes the event
            self.orchestrator_wakeup.wait()
            self.orchestrator_wakeup.clear()

            if self.stop_flag:
                self.bb_logger.notify_discord("[Orchestrator] stop_flag was triggered") 
                break

            # 3. PRIORITY 1: Human Interruption
            #    If the interrupt flag is set, we stop everything and trap execution
            #    here until the human releases us.
            if self.human_interrupt_event.is_set():
                self.bb_logger.debug("[Orchestrator] Human interruption is set")
                self._handle_human_interrupt()
                # Once we return from _handle_human_interrupt, the flag is cleared.
                # We 'continue' to restart the loop and see what protocols are now valid.
                continue

            # 4. PRIORITY 2: Protocol Management
            self._reconcile_protocols()

    def _handle_human_interrupt(self):
        """
        Stops the running protocol and blocks until human clears the interrupt.
        """
        self.bb_logger.debug("[Orchestrator] Human Interrupt Detected! Pausing system.")
        
        # 1. Immediately kill any running tasks
        if self.running_tree:
            self.bb_logger.notify_discord(f"[Orchestrator] Preempting {self.running_tree['name']} due to Human.") 
            
            self.stop_protocol()

        # 2. Trap the orchestrator here.
        #    We loop/wait until the human_interrupt_event is CLEARED by the voice node.
        #    The voice node triggers 'orchestrator_wakeup' when it clears the flag,
        #    breaking this wait.
        while not self.stop_flag and self.human_interrupt_event.is_set():
            self.bb_logger.debug(
                    "[Orchestrator] System paused. Waiting for human idle.")
            self.orchestrator_wakeup.wait()
            self.orchestrator_wakeup.clear()

        self.bb_logger.notify_discord(
            "[Orchestrator] Human released control. Resuming operations."
        )
     
    def _reconcile_protocols(self):
        """
        Compares currently running tree against satisfied triggers
        to decide if we should Start, Stop, or Swap tasks.
        """

        if self.stop_flag:
            self.bb_logger.debug(
                "[Orchestrator] Stop flag set. Skipping reconciliation."
            )
            return

        # A. Cleanup: If a thread finished naturally, clear our memory of it
        if self.running_thread and not self.running_thread.is_alive():
            if self.debug:
                self.bb_logger.info(
                    "[Orchestrator] Thread finished naturally. Cleaning up."
                )
            with self.lock:
                self.running_tree = None
                self.running_thread = None

        # B. Get Candidates
        satisfied = self.trigger_monitor.get_satisfied()
        self.bb_logger.debug(
                    f"[Orchestrator] satisfied protocols {satisfied}."
                )
        # Sort by priority (lowest number = highest priority)
        # satisfied is list of tuples: [("Name", priority_int), ...]
        best_candidate = min(satisfied, key=lambda x: x[1], default=None)

        if not best_candidate:
            self.bb_logger.debug(
                "[Orchestrator] No protocols satisfied."
            )
            return

        with self.lock:
            current_run = self.running_tree

        # C. Decision Logic
        if not current_run:
            # Case 1: Nothing running -> Start the best one
            self.bb_logger.debug(
                f"[Orchestrator] Idle. Starting {best_candidate}"
            )
            self.start_protocol(best_candidate)

        else:
            # Case 2: Something running -> Check for Preemption
            self.bb_logger.debug(
                "[Orchestrator]  Something is running."
            )
         

            current_priority = current_run["priority"]
            new_priority = best_candidate[1]

            if new_priority < current_priority:
                self.bb_logger.debug(
                    f"[Orchestrator] Preempting Priority {current_priority} for Priority {new_priority}"
                    )
                self.stop_protocol()
                self.start_protocol(best_candidate)

    def _run_protocol(self, tree_runner, class_protocol_name):
        """Run a protocol tree and clean up when done."""
        try:
            tree_runner.run_until_done()
        finally:
            self.bb_logger.info(f"tree_runner.final_status {tree_runner.final_status}")
            if tree_runner.final_status == py_trees.common.Status.SUCCESS:
                self.bb_logger.info(
                    f"{class_protocol_name} is marked completed" 
                )
                self.trigger_monitor.mark_completed(class_protocol_name)

            with self.lock:
                self.bb_logger.info(f"[Orchestrator] Finished: {class_protocol_name}")
                # tree = self.running_tree["tree"]
                # tree.nodes_cleanup() ## script does on its own
                self.running_tree = None
                self.running_thread = None

    def camel_to_snake(self, name: str) -> str:
        """Convert CamelCase to snake_case."""
        s1 = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
        s2 = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s1)
        return s2.lower()

    def start_protocol(self, protocol_tuple):
        """Start the protocol in its own thread."""
        if protocol_tuple is None:
            self.bb_logger.debug(
                    "[Orchestrator] Warning: Tried to start None protocol — skipping. HARMLESS"
                )
            return

        class_protocol_name, priority = protocol_tuple

        self.bb_logger.debug(
                f"[Orchestrator] Starting: {class_protocol_name} (priority {priority})"
            )

        # import tree dynamically
        tree_class_name = class_protocol_name.split(".")[
            0
        ]  # example MoveAwayProtocolTree
        snake_case_class_name = self.camel_to_snake(
            tree_class_name
        )  # turns it to snake case
        protocol_name = class_protocol_name.split(".")[
            -1
        ]  # unique for a protocol ex: medicine_am

        self.bb_logger.debug(
            f"**** tree_class_name : {tree_class_name}\n"
            f"**** snake_case_class_name : {snake_case_class_name}\n"
            f"**** protocol name : {protocol_name}"
        )

        #  Dynamically import module & class
        try:
            module = importlib.import_module(
                f"smart_home_pytree.protocols.{snake_case_class_name}"
            )
            # gets the class from the file in module
            tree_class = getattr(module, f"{tree_class_name}Tree")
        except Exception as e:
            self.bb_logger.info(
                f"[Orchestrator] Failed to load tree '{tree_class_name}Tree' from module smart_home_pytree.trees.{snake_case_class_name} : {e}"
            )
            return

        # Instantiate the tree
        tree_runner = tree_class(
            node_name=snake_case_class_name,
            protocol_name=protocol_name,
            robot_interface=self.robot_interface,
            executor=self.executor,
        )

        tree_runner.setup()
        thread = threading.Thread(
            target=self._run_protocol,
            args=(tree_runner, class_protocol_name),
            daemon=True,
        )
        thread.start()

        with self.lock:
            self.running_tree = {
                "name": protocol_name,
                "priority": priority,
                "tree": tree_runner,
            }
            self.running_thread = thread

    def stop_protocol(self):
        """Stop the currently running protocol."""
        with self.lock:
            if not self.running_tree:
                return
            name = self.running_tree["name"]
            self.bb_logger.info(f"[Orchestrator] Stopping: {name}")
            tree = self.running_tree["tree"]

        tree.stop_tree()
        self.running_thread.join(timeout=5)
        # tree.cleanup() ## cleans up on its own with stop tree

        with self.lock:
            self.running_tree = None
            self.running_thread = None

    def shutdown(self):
        """Gracefully stop everything (ROS-safe)."""
        self.bb_logger.info("[Orchestrator] Shutting down...")
        

        # 1. Stop orchestrator logic
        self.stop_flag = True
        self.orchestrator_wakeup.set()

        # 2. Stop non-ROS threads
        if self.trigger_monitor:
            self.trigger_monitor.stop_monitor()

        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=10)

        if self.running_tree:
            self.stop_protocol()

        # 3. STOP EXECUTOR FIRST (CRITICAL)
        if hasattr(self, "executor") and self.executor:
            self.bb_logger.debug("[Orchestrator] Shutting down executor...")
            self.executor.shutdown()

        # 4. Destroy nodes AFTER executor stops
        if self.human_interface_node:
            self.human_interface_node.shutdown()
            self.human_interface_node.destroy_node()

        if self.robot_interface:
            self.robot_interface.destroy_node()

        # 5. Join spin thread
        if self.ros_spin_thread and self.ros_spin_thread.is_alive():
            self.ros_spin_thread.join(timeout=5)

        # 6. Shutdown rclpy ONCE
        if rclpy.ok() and self.rclpy_initialized_here:
            self.bb_logger.debug("[Orchestrator] rclpy shutdown")
            rclpy.shutdown()

        self.bb_logger.notify_discord("[Orchestrator] Shutdown complete.")


def validate_time_arg(time_str: str) -> str:
    """
    Validate HH:MM (24-hour) time format.

    Returns:
        The original string if valid.

    Raises:
        argparse.ArgumentTypeError if invalid.
    """
    if time_str == "":
        return time_str
    try:
        datetime.strptime(time_str, "%H:%M")
        return time_str
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"Invalid time format '{time_str}'. Expected HH:MM (24-hour), e.g. 10:30."
        )


def main():
    """Main function to run the Protocol Orchestrator."""
    parser = argparse.ArgumentParser(
        description=""" Protocol Orchestrator that runs the behavior tree based on the requirements

        """,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "--test_time",
        type=validate_time_arg,
        default="",
        help="Optional protocol time (string) override in HH:MM format (e.g., 10:30).",
    )

    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug output.",
    )

    parser.add_argument(
        "--env_yaml_file_name",
        type=str,
        default="house_yaml_path",
        help=(
            "Name of the environment variable that stores the YAML file path "
            "(default: house_yaml_path)."
        ),
    )

    args, _ = parser.parse_known_args()
    test_time = args.test_time

    # Resolve YAML file path from environment variable
    yaml_file_path = os.getenv(args.env_yaml_file_name)
    if yaml_file_path is None:
        raise RuntimeError(
            f"Environment variable '{args.env_yaml_file_name}' is not set."
        )
    # blackboard = py_trees.blackboard.Blackboard()
    print("Loading location and protocol to bb")
    
    load_protocols_to_bb(yaml_file_path, debug=False)
    load_locations_to_blackboard(yaml_file_path)

    if test_time:
        print(f"[INFO] Using overridden time: {test_time}")
    else:
        print("[INFO] Using system time")

    # For testing:
    orch = ProtocolOrchestrator(test_time=test_time, debug=args.debug)

    try:
        print("ORCHI LOOP")
        orch.orchestrator_loop()
    except KeyboardInterrupt:
        print("[Main] KeyboardInterrupt received.")
    finally:
        orch.shutdown()


if __name__ == "__main__":
    main()


# ros2 topic pub /display_rx std_msgs/msg/String "data: 'exercise_requested'"
# ros2 topic pub /display_rx std_msgs/msg/String "data: 'exercise_stop'"


# ros2 run smart_home_pytree protocol_orchestrator -- --debug --test_time 10:30 --env_yaml_file_name house_yaml_path
# ros2 run rqt_console rqt_console
