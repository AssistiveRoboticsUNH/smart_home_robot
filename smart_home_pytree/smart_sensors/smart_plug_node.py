import asyncio
import os
import threading
import time

import rclpy
from kasa import SmartPlug
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32


class SmartPlugPublisher(Node):
    def __init__(self, smartthings_response, update_period):
        super().__init__("smart_plug_node")

        # 1. Define QoS Profile (Latched / Transient Local)
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # 2. Create Publisher with Bool type
        self.charging_publisher = self.create_publisher(
            Bool, "charging", self.qos_profile
        )

        self.timer = self.create_timer(update_period, self.timer_callback)
        self.smartplug_response = smartthings_response

        # 3. Variable to track the previous state (init to None)
        self.last_powered_state = None

    def timer_callback(self):
        if self.smartplug_response.updated:
            current_state = self.smartplug_response.powered  # This is now a boolean

            # 4. Publish only on Flip (State Change) or first run
            if (
                self.last_powered_state is None
                or current_state != self.last_powered_state
            ):
                msg = Bool()
                msg.data = current_state
                self.charging_publisher.publish(msg)

                self.get_logger().info(f"Plug State Changed: {current_state}")

                # Update the tracker
                self.last_powered_state = current_state


class SmartPlugResponse:
    def __init__(self, update_period):
        self.updated = False
        self.smart_plug = SmartPlug(os.environ.get("plug_ip"))

        self.powered = 0

        self.update_period = update_period

    async def read_device(self):
        while True:
            start = time.time()
            try:
                await self.smart_plug.update()
                if self.smart_plug.emeter_realtime.power > 15:
                    self.powered = True
                else:
                    self.powered = False
                self.updated = True
            except Exception as e:
                print(f"[SmartPlug] Failed to update: {e}")
                self.updated = False
                self.powered = False  # optional: set to 0 if unreachable

            end = time.time()
            sleep_duration = self.update_period - (end - start)
            if sleep_duration > 0:
                await asyncio.sleep(sleep_duration)
            else:
                await asyncio.sleep(0.1)  # fallback delay if processing is slow


def main(args=None):
    update_period = 1  # 1 sec
    smartplug_response = SmartPlugResponse(update_period)
    x = threading.Thread(target=asyncio.run, args=(smartplug_response.read_device(),))
    x.start()

    rclpy.init(args=args)

    minimal_publisher = SmartPlugPublisher(smartplug_response, update_period)

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
