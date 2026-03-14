"""Pimu Monitor — publishes hardware telemetry from the Stretch pimu.

Topics published (all TRANSIENT_LOCAL / RELIABLE / depth 1):
  /charging          (Bool)    — charger_is_charging  (change-only)
  /battery_percent   (Int32)   — battery_soc          (every poll)
  /voltage_36v       (Float32) — voltage_36v0         (every poll)
  /voltage_12v       (Float32) — voltage_12v0         (every poll)
  /over_tilt         (Bool)    — over_tilt_alert       (change-only)
"""

import rclpy
import stretch_body_ii.robot.robot_client as rc
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32


class PimuMonitor(Node):
    def __init__(self):
        super().__init__("pimu_monitor")

        # ---- Robot Client ----
        self.get_logger().info("Connecting to Robot Client...")
        self.robot = rc.RobotClient()
        if not self.robot.startup():
            self.get_logger().error("Failed to connect to Robot Client")
            exit(1)

        # ---- QoS (latched — late joiners get last value) ----
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ---- Publishers ----
        self.pub_charging = self.create_publisher(Bool, "/charging", self.qos)
        self.pub_battery = self.create_publisher(Int32, "/battery_percent", self.qos)
        self.pub_v36 = self.create_publisher(Float32, "/voltage_36v", self.qos)
        self.pub_v12 = self.create_publisher(Float32, "/voltage_12v", self.qos)
        self.pub_tilt = self.create_publisher(Bool, "/over_tilt", self.qos)

        # ---- Change-tracking (only for change-only topics) ----
        self.last_charging: bool | None = None
        self.last_over_tilt: bool | None = None

        # ---- Timer (poll every 0.5 s) ----
        self.timer = self.create_timer(0.5, self._poll)
        self.get_logger().info("Pimu Monitor Started — publishing 5 topics.")

    # ------------------------------------------------------------------
    def _poll(self):
        try:
            self.robot.pull_status()
            pimu = self.robot.status["pimu"]
        except KeyError:
            self.get_logger().warn("Could not read pimu from robot status.")
            return
        except Exception as e:
            self.get_logger().error(f"Error reading robot status: {e}")
            return

        # ---- charging (change-only) ----
        is_charging = bool(pimu.get("charger_is_charging", False))
        if is_charging != self.last_charging:
            msg = Bool()
            msg.data = is_charging
            self.pub_charging.publish(msg)
            state_str = "CHARGING" if is_charging else "NOT CHARGING"
            self.get_logger().info(f"Charging state change: {state_str}")
            self.last_charging = is_charging

        # ---- battery_percent (every poll) ----
        battery_soc = int(pimu.get("battery_soc", 0))
        bat_msg = Int32()
        bat_msg.data = battery_soc
        self.pub_battery.publish(bat_msg)

        if battery_soc < 30:
            self.get_logger().warn(
                f"weblog=WARNING!! BATTERY PERCENTAGE AT {battery_soc}%"
            )

        # ---- voltage_36v (every poll) ----
        v36_msg = Float32()
        v36_msg.data = float(pimu.get("voltage_36v0", 0.0))
        self.pub_v36.publish(v36_msg)

        # ---- voltage_12v (every poll) ----
        v12_msg = Float32()
        v12_msg.data = float(pimu.get("voltage_12v0", 0.0))
        self.pub_v12.publish(v12_msg)

        # ---- over_tilt (change-only) ----
        over_tilt = bool(pimu.get("over_tilt_alert", False))
        if over_tilt != self.last_over_tilt:
            tilt_msg = Bool()
            tilt_msg.data = over_tilt
            self.pub_tilt.publish(tilt_msg)
            if over_tilt:
                self.get_logger().error("weblog=OVER TILT ALERT! Robot may have tipped over.")
            self.last_over_tilt = over_tilt

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.robot.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PimuMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
