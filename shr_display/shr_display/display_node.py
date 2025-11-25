import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .protocol_manager import ProtocolManager
from .zmq_interface import ZmqInterface

import json, yaml
import pathlib
import time
import subprocess, signal, os

class DisplayNode(Node):
    def __init__(self):
        # Load default parameters from config file
        config_file = self.get_default_config_path()
        super().__init__('shr_display')

        with open(config_file, 'r') as f:
            yaml_params = yaml.safe_load(f)

        params = yaml_params.get('shr_display', {}).get('ros__parameters', {})

        # declare each parameter with the default value from yaml
        for key, value in params.items():
            self.declare_parameter(key, value)

        # Parameters
        tx_port = self.get_parameter('zmq_tx_port').value
        rx_port = self.get_parameter('zmq_rx_port').value
        interval = self.get_parameter('protocol_publish_interval_sec').value
        intersection_file_path = self.get_parameter('intersection_file_path').value
        webapp_file_path = self.get_parameter('webapp_file_path').value
        alive_log_interval = self.get_parameter('alive_log_interval_sec').value

        # Load JSON routines
        self.protocol_manager = ProtocolManager(
            str(self.get_package_share_directory() / 'config' / 'protocol_routines.json'), intersection_file_path
        )

        self.zmq = ZmqInterface(tx_port, rx_port)
        time.sleep(1) # Allow 1 second for zmq connection to establish

        # ROS topics
        self.tx_sub = self.create_subscription(String, 'display_tx', self.forward_to_app, 10)
        self.rx_pub = self.create_publisher(String, 'display_rx', 10)

        # Timers
        self.create_timer(interval, self.publish_protocols)
        self.create_timer(0.5, self.poll_app_responses)
        self.create_timer(alive_log_interval, self.periodic_alive_log)

        self.display_alive = False

        # Publish once immediately
        self.publish_protocols()

        deadline = time.time() + 2
        while time.time() < deadline:
            try:
                response = self.zmq.receive()
                if response:
                    self.display_alive = True
                    # print(response)
                    break
            except self.zmq.Again:
                time.sleep(0.05)
        

        msg = "Display app is ALIVE" if self.display_alive else "Display app NOT reachable"
        self.get_logger().info(msg)

        self.start_flask_app(webapp_file_path)
    
    def forward_to_app(self, msg: String):
        """Forward anything being published on display_tx to the ZMQ PUB socket."""
        self.zmq.send(msg.data)

    def get_default_config_path(self):
        from ament_index_python.packages import get_package_share_directory
        pkg_path = pathlib.Path(get_package_share_directory('shr_display'))
        return pkg_path / 'config' / 'params.yaml'

    def get_package_share_directory(self):
        from ament_index_python.packages import get_package_share_directory
        return pathlib.Path(get_package_share_directory('shr_display'))

    def publish_protocols(self):
        self.protocol_manager.update_protocol_json()
        msg_string = self.protocol_manager.get_as_string()
        self.zmq.send(msg_string)

    def poll_app_responses(self):
        response = self.zmq.receive()
        if response:
            self.display_alive = True
            self.rx_pub.publish(String(data=response))
            # update JSON if protocol_confirm
            try:
                data = json.loads(response)
                if data.get("type") == "protocol_confirm":
                    self.protocol_manager.update_confirmed(
                        data["protocol"],
                        bool(data["confirmed"])
                    )
            except Exception:
                pass

    def periodic_alive_log(self):
        if not self.display_alive:
            self.get_logger().warn("Display app NOT reachable")

    def start_flask_app(self, webapp_file_path):
        """Start the external Flask app as a subprocess."""
        flask_cmd = ["python3", webapp_file_path]
        self.flask_process = subprocess.Popen(flask_cmd)
        self.get_logger().info(f"Flask app started (PID {self.flask_process.pid})")

    def destroy_node(self):
        """Override destroy_node so we can also terminate the Flask app."""
        if hasattr(self, "flask_process"):
            try:
                self.get_logger().info("Terminating Flask app...")
                os.killpg(self.flask_process.pid, signal.SIGTERM)
                try:
                    self.flask_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.get_logger().warn("Flask didn’t exit; killing...")
                    os.killpg(self.flask_process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DisplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()