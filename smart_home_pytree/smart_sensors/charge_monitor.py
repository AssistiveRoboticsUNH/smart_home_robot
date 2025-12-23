import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool

# Importing based on your provided snippet
import stretch_body_ii.robot.robot_client as rc

class ChargingMonitor(Node):
    def __init__(self):
        super().__init__('charging_monitor')

        # 1. Initialize Robot Connection
        self.get_logger().info('Connecting to Robot Client...')
        self.robot = rc.RobotClient()
        if not self.robot.startup():
            self.get_logger().error('Failed to connect to Robot Client')
            exit(1)

        # 2. Define QoS Profile (Non-Volatile / Latched)
        # TRANSIENT_LOCAL means new subscribers get the last message immediately
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # 3. Create Publisher
        self.publisher_ = self.create_publisher(Bool, '/charging', self.qos_profile)

        # 4. State tracking
        self.last_charging_state = None
        
        # 5. Timer to check status (Check every 0.5 seconds)
        self.timer = self.create_timer(0.5, self.check_charging_status)
        self.get_logger().info('Charging Monitor Node Started.')

    def check_charging_status(self):
        try:
            # This pulls the latest data
            self.robot.pull_status() 
            
            # Access the Pimu status
            pimu=self.robot.status["pimu"]
            is_charging = pimu['charger_is_charging']

            # Check if state has changed
            if is_charging != self.last_charging_state:
                msg = Bool()
                msg.data = bool(is_charging)
                
                self.publisher_.publish(msg)
                
                state_str = "CHARGING" if is_charging else "NOT CHARGING"
                self.get_logger().info(f'State Change Detected: {state_str}')
                
                # Update tracker
                self.last_charging_state = is_charging

        except KeyError:
            self.get_logger().warn("Could not find 'charger_is_charging' in robot status.")
        except Exception as e:
            self.get_logger().error(f"Error reading status: {e}")

    def destroy_node(self):
        # Clean shutdown of robot connection
        self.robot.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ChargingMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()