import asyncio
from kasa import SmartPlug

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import os

class SmartPlugPublisher(Node):

    def __init__(self, smartthings_response, update_period):
        super().__init__('smart_pulg')
        self.charging_publisher = self.create_publisher(Int32, 'charging', 10)

        self.timer = self.create_timer(update_period, self.timer_callback)
        self.smartplug_response = smartthings_response

    def timer_callback(self):
        if self.smartplug_response.updated:
            msg = Int32()
            msg.data = self.smartplug_response.powered
            self.charging_publisher.publish(msg)


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
                    self.powered = 1
                else:
                    self.powered = 0
                self.updated = True
            except Exception as e:
                print(f"[SmartPlug] Failed to update: {e}")
                self.updated = False
                self.powered = 0  # optional: set to 0 if unreachable
            
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


if __name__ == '__main__':
    main()