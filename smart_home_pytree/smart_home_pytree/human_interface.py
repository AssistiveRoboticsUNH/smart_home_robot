import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HumanInterface(Node):
    """
    ROS 2 Node that translates voice commands into system interrupts and robot state updates.

    This node monitors voice status to pause the Orchestrator during human interaction
    and parses specific user intents to update the robot's shared state. It includes
    a safety timeout to ensure the system doesn't remain paused indefinitely.
    """
    def __init__(self, human_interrupt_event: threading.Event, orchestrator_wakeup: threading.Event,
                 node_name: str = 'voice_trigger', robot_interface=None, debug:bool = True):
        """
        Initialize the HumanInterface node.

        Args:
            human_interrupt_event (threading.Event): Shared event to signal human interruption.
            orchestrator_wakeup (threading.Event): Shared event to wake the main loop.
            node_name (str, optional): ROS node name. Defaults to 'voice_trigger'.
            robot_interface (object, optional): Interface to update robot state. Defaults to None.
        """
        super().__init__(node_name)

        # This allows the timer and the subscriptions to run in parallel
        self.human_interrupt_event = human_interrupt_event
        self.orchestrator_wakeup = orchestrator_wakeup
        self.robot_interface = robot_interface
        self.debug = debug

        # Timer handle for the safety timeout
        self.timeout_timer = None
        self.timeout_duration_sec = 120.0  # 2 minutes
       
        # Voice Subscriptions
        self.create_subscription(
            String,
            '/voice/status',
            self.voice_status_callback,
            10
        )

        # in the voice_user_callback, implmeemnt the logic in how you want to set
        # it based on the input of the user
        self.create_subscription(
            String,
            '/voice/user',
            self.voice_user_callback,
            10
        )

    def _start_safety_timer(self):
        """Starts a native Python thread timer."""
        self._cancel_safety_timer()
        
        # This creates a new thread that sleeps for 120s then runs the function
        self.timeout_timer = threading.Timer(
            self.timeout_duration_sec, 
            self._safety_timeout_callback
        )
        self.timeout_timer.start()
        self.get_logger().info(f"[HumanInterface] Native Safety Timer started ({self.timeout_duration_sec}s)")
        if self.debug:
            print(f"[HumanInterface] Native Safety Timer started ({self.timeout_duration_sec}s)")
    
    def _cancel_safety_timer(self):
        """Cancels the native timer."""
        if self.timeout_timer is not None:
            self.timeout_timer.cancel()
            self.timeout_timer = None

    def _safety_timeout_callback(self):
        """Fires from a separate native thread."""
        self.get_logger().error("[HumanInterface] TIMEOUT! System resuming automatically.")
        if self.debug:
            print(f"[HumanInterface] Native Safety Timer started ({self.timeout_duration_sec}s)")
        
        if self.human_interrupt_event.is_set():
            self.human_interrupt_event.clear()
            self.orchestrator_wakeup.set()
            if self.debug:
                print( "[HumanInterface] orchestrator_wakeup SET")
                print( "[HumanInterface] human_interrupt_event clear")
            
    def voice_status_callback(self, msg: String):
        """
        Handles status updates from the voice engine (Waking vs Idle).

        Args:
            msg (String): The status message (e.g., "WAKEWORD_TRIGGER", "IDLE").
        """
        wakeup_triggered = "WAKEWORD_TRIGGER"
        done_waking = "IDLE"

        if msg.data == wakeup_triggered:
            # Only log and set if we aren't already interrupted
            if not self.human_interrupt_event.is_set():
                self.get_logger().warn("[HumanInterface] WAKEWORD DETECTED! Pausing Orchestrator.")
                self.human_interrupt_event.set()
                self.orchestrator_wakeup.set()

                if self.debug:
                    print( "[HumanInterface] orchestrator_wakeup SET")
                    print( "[HumanInterface] human_interrupt_event SET")


                # Start safety timer in case we never get an IDLE within timeout
                self._start_safety_timer()
                
        elif msg.data == done_waking:
            if self.human_interrupt_event.is_set():
                self.get_logger().info("[HumanInterface] Voice IDLE. Resuming Orchestrator.")
                if self.debug:
                    print( "[HumanInterface] Voice IDLE. Resuming Orchestrator.")

                self.human_interrupt_event.clear()
                self.orchestrator_wakeup.set()
                if self.debug:
                    print("[HumanInterface] orchestrator_wakeup SET")
                    print("[HumanInterface] human_interrupt_event clear")
                
                # Interaction successful, we don't need the fallback timer anymore
                self._cancel_safety_timer()

        self.get_logger().info(f"[VOICE STATUS] {msg.data}")

    ## move is hard to pick up todo fix
    def voice_user_callback(self, msg: String):
        """
        Parses natural language intents and updates the robot's shared state.

        Args:
            msg (String): The user's transcribed text.
        """
        text = msg.data.lower()
        self.get_logger().info(f"[VOICE USER] '{text}'")

        if "home" in text:
            self.get_logger().info(
                "[DECISION] User said 'home', set move_away=True, position='home'"
            )

            if self.debug:
                print( "[DECISION] User said 'home', set move_away=True, position='home'")
        
            self.robot_interface.state.update('move_away', True) ## to trigger the protocol, positoin decides the location
            self.robot_interface.state.update('position', "home")

        elif "away" in text:
            self.get_logger().info(
                "[DECISION] User said 'away' → set move_away=True, position='away'"
            )

            if self.debug:
                print( "[DECISION] User said 'away' → set move_away=True, position='away'")

            self.robot_interface.state.update('move_away', True)
            self.robot_interface.state.update('position', "away")

    def shutdown(self):
        """Cleanup method to be called by the Orchestrator on exit."""
        self.get_logger().info("Shutting down HumanInterface...")
        self._cancel_safety_timer()
        # Additional cleanup if needed
        self.destroy_node()

def main(args=None):
    """Main function to run the HumanInterface node."""
    rclpy.init(args=args)

    # Fix: Create the events required by the constructor
    human_event = threading.Event()
    wakeup_event = threading.Event()

    node = HumanInterface(human_interrupt_event=human_event, orchestrator_wakeup=wakeup_event)    

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
