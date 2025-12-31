#!/usr/bin/env python3

import py_trees
import py_trees_ros.trees
import py_trees.console as console
import rclpy
from py_trees import display
from smart_home_pytree.registry import load_locations_to_blackboard
from smart_home_pytree.robot_interface import get_robot_interface
import time
import os


class BaseTreeRunner:
    """
    Base class for running behavior trees in a modular way.
    Subclasses must override `create_tree()`.

    kwargs are for variables that are not related to the BaseTreeRunner but to he ones inheriting from it.
    """

    def __init__(self, node_name: str, robot_interface=None, **kwargs):

        self.node_name = node_name
        self.debug = kwargs.get("debug", False)

        # ticking the behavior tree at regular intervals
        self.timer_period = kwargs.get("timer_period", 1.0)

        self.executor_ = None
        self.tree = None
        self.root = None
        self.kwargs = kwargs   # store extra args for flexibility
        self.tb3_process = None
        self.launch_service = None

        yaml_file_path = os.getenv("house_yaml_path", None)

        self.nodes_cleanup_done = False
        load_locations_to_blackboard(yaml_file_path)
        self.rclpy_initialized_here = None
        self._stop_tree = False
        self.robot_interface_initialized_here = None

        if robot_interface is None:
            print("Initialize robot interface")
            self.robot_interface = get_robot_interface()
            self.robot_interface_initialized_here = True
        else:
            if self.debug:
                print("NOTICE: USING PROVIDED ROBOT INTERFACE")
            self.robot_interface = robot_interface
            self.robot_interface_initialized_here = False

    def required_actions(self) -> dict:
        """
        Subclasses can override this to define required actions.
        Should return a dict in the form:
            { "package_name": ["action1", "action2", ...] }
        """
        return {}

    def required_topics(self) -> list:
        """
        Subclasses can override this to define required topics.
        Should return a list of topic names (strings).
        """
        return []

    def describe_requirements(self):
        """
        Prints required topics and actions defined in required_actions and required_topics functions.
        """
        console.loginfo(console.bold + f"[{self.node_name}] Required resources:" + console.reset)

        topics = self.required_topics()
        actions = self.required_actions()

        # --- Print topics ---
        if topics:
            console.loginfo("  • Topics:")
            for t in topics:
                console.loginfo(f"     - {t}")
        else:
            console.logwarn("  • No specific topics required.")

        # --- Print actions ---
        if actions:
            console.loginfo("  • Actions:")
            for pkg, nodes in actions.items():
                for node in nodes:
                    console.loginfo(f"     - {pkg}/{node}")
        else:
            console.logwarn("  • No specific actions required.")

    def create_tree(self) -> py_trees.behaviour.Behaviour:
        """
        Override this in subclasses to define the specific behavior tree.
        """
        raise NotImplementedError("Subclasses must implement create_tree()")

    def setup(self):
        """Initialize ROS2, executor, and build the behavior tree."""
        self.describe_requirements()

        if not rclpy.ok():
            # just for safety
            try:
                rclpy.init(args=None)
                self.rclpy_initialized_here = True
            except RuntimeError:
                # ROS2 already initialized somewhere else
                self.rclpy_initialized_here = False
        else:
            self.rclpy_initialized_here = False

        if self.debug:
            print("######### SETUP ##################")
            print("BASEE TREE self.robot_interface", self.robot_interface)
            print("BASEE TREE: ", self.node_name, " self id:", id(self))

        # self.executor_ = rclpy.executors.MultiThreadedExecutor()
        self.executor_ = rclpy.executors.SingleThreadedExecutor()

        # Build the tree
        self.root = self.create_tree()
        self.tree = py_trees_ros.trees.BehaviourTree(
            root=self.root,
            unicode_tree_debug=True
        )

        try:
            self.tree.setup(node_name=self.node_name, timeout=15.0)
        except py_trees_ros.exceptions.TimedOutError as e:
            console.logerror(f"Tree setup failed: {e}")
            self.cleanup(exit_code=1)
        except KeyboardInterrupt:
            console.logerror("Tree setup interrupted.")
            self.cleanup(exit_code=1)

        self.executor_.add_node(self.tree.node)

    def run_until_done(self):
        """Run until the tree finishes with SUCCESS or FAILURE. Uses tree.root.tick_once """

        # make sure to overwrite stop flag
        self._stop_tree = False
        self.final_status = py_trees.common.Status.FAILURE  # initialize

        def tick_tree_until_done(timer):
            try:

                self.tree.root.tick_once()

                if self.debug:
                    print("=" * 25 + " TREE STATE " + "=" * 25)
                    print(display.unicode_tree(root=self.tree.root, show_status=True))
                    print("\n")

            except Exception as e:
                import traceback
                print(" Exception during tick:")
                traceback.print_exc()
                self.cleanup(exit_code=1)
                return

            current_status = self.tree.root.status
            if self.tree.root.status in [
                py_trees.common.Status.SUCCESS,
                py_trees.common.Status.FAILURE
            ]:
                color = console.green if current_status == py_trees.common.Status.SUCCESS else console.red

                console.loginfo(
                    color
                    + f"Tree finished with status: {current_status}"
                    + console.reset
                )

                self.final_status = current_status
                timer.cancel()
                self._stop_tree = True

        # ticking the behavior tree at regular intervals
        timer_period = self.timer_period
        tree_timer = self.tree.node.create_timer(
            timer_period,
            lambda: tick_tree_until_done(tree_timer)
        )

        try:
            while not self._stop_tree:
                self.executor_.spin_once(timeout_sec=0.1)

            if self.debug:
                print("tree_timer.is_canceled(): ", tree_timer.is_canceled())

            if not tree_timer.is_canceled():

                if self.debug:
                    print("Cancelling timer after stop signal")
                    print("failing the tree")

                self.final_status = py_trees.common.Status.INVALID
                self.tree.root.stop(self.final_status)
                tree_timer.cancel()
                self.nodes_cleanup()

            print("Stop spinning")
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            console.logwarn("Executor interrupted.")
        finally:
            return self.final_status

    def stop_tree(self):
        """Signal to stop the tree execution."""
        self._stop_tree = True

    def run_continuous(self):
        """Run the tree infinetly until user stops. Uses tree.tick_tock """
        self.tree.tick_tock(period_ms=1000.0)
        try:
            self.executor_.spin()
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            console.logwarn("Executor interrupted.")
        finally:
            self.cleanup()

    def nodes_cleanup(self):
        """Clean up the tree node."""
        try:

            if self.tree:
                if self.debug:
                    print("self.tree shutdown ")
                self.tree.shutdown()

            if self.robot_interface and self.robot_interface_initialized_here:
                if self.debug:
                    print("self.robot_interface shutdown ")
                self.robot_interface.shutdown()

            if self.executor_:
                if self.debug:
                    print("self.executor_ shutdown ")
                self.executor_.shutdown()

            self.nodes_cleanup_done = True
        except Exception as e:
            print(f"Node cleanup failed: {e}")

    def cleanup(self, exit_code=0):
        """Clean shutdown of all nodes, executors, and subprocesses."""
        try:
            if not self.nodes_cleanup_done:
                if self.debug:
                    print("Cleaning up nodes ")
                self.nodes_cleanup()

            # Shutdown ROS2
            if self.rclpy_initialized_here:
                if self.debug:
                    print("ROS2 shutdown ")
                rclpy.try_shutdown()

        except Exception as e:
            print(f"Cleanup failed: {e}")
