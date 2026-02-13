import aiohttp
import asyncio
import threading
import time
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Int32
import requests

class HomeSeerPublisher(Node):
    ## update ip, name of main door sensor and id for devices
    def __init__(self):
        super().__init__('homeseer_publisher')
        self.publisher_bathroom_sensor = self.create_publisher(Bool, 'bathroom_sensor', 10)
        self.publisher_main_sensor_door = self.create_publisher(Bool, 'UPDATE', 10)
        update_period = 5 # sec
        
        self.timer = self.create_timer(update_period, self.timer_callback)
        self.url = 'http://192.168.50.77/json?request=getstatus&ref='

    def timer_callback(self):
        # Make the request to get the JSON data just add ref number
        msg_bool = Bool()

        bedroom_door_sensor_response = requests.get(self.url+"316")
        # Parse the JSON content
        data = bedroom_door_sensor_response.json()
        # Find the value associated with the "ref"
        devices = data.get("Devices", [])
        for device in devices:
            if device.get("ref") == 316:
                value = device.get("value")
                if value ==22: # door is open
                    msg_bool.data = True
                else:
                    msg_bool.data = False
                self.publisher_bedroom_sensor_door.publish(msg_bool)

                print(f'The value is: {value}')


        motion_sensor_response = requests.get(self.url+"320")
        # Parse the JSON content
        data = motion_sensor_response.json()
        # Find the value associated with the "ref"
        
        devices = data.get("Devices", [])
        for device in devices:
            if device.get("ref") == 320:
                value = device.get("value")
                if value ==7: # motion detected
                    msg_bool.data = True
                else:
                    msg_bool.data = False
                self.publisher_pills_motion_sensor.publish(msg_bool)

                print(f'The value is: {value}')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = HomeSeerPublisher()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()