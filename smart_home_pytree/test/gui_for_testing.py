#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QComboBox, QPushButton, QLabel
from PyQt5.QtCore import QTimer
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Define some example locations
LOCATIONS = ["dining", "living_room", "bedroom", "home"]

class RobotControlGUI(Node, QWidget):
    def __init__(self):
        rclpy.init(args=None)
        Node.__init__(self, "robot_control_gui")
        QWidget.__init__(self)
        
        self.setWindowTitle("Robot Control GUI")
        self.setGeometry(200, 200, 300, 200)
        
        # --- QoS Profile (Matched for Latching) ---
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # --- Publishers ---
        self.robot_pub = self.create_publisher(String, 'robot_location', 10)
        self.person_pub = self.create_publisher(String, 'person_location', 10)
        self.charging_pub = self.create_publisher(Bool, 'charging', self.qos_profile)
        
        # --- Subscriber ---
        # We subscribe to the same topic to keep the GUI in sync with the robot
        self.charging_sub = self.create_subscription(
            Bool, 
            'charging', 
            self.charging_callback, 
            self.qos_profile
        )
        
        # --- GUI Layout ---
        layout = QVBoxLayout()
        
        # Robot Location
        layout.addWidget(QLabel("Robot Location:"))
        self.robot_combo = QComboBox()
        self.robot_combo.addItems(LOCATIONS)
        layout.addWidget(self.robot_combo)
        
        # Person Location
        layout.addWidget(QLabel("Person Location:"))
        self.person_combo = QComboBox()
        self.person_combo.addItems(LOCATIONS)
        layout.addWidget(self.person_combo)
        
        # Charging Button
        layout.addWidget(QLabel("Charging State:"))
        self.charging_button = QPushButton("False")
        self.charging_button.setCheckable(True)
        self.charging_button.clicked.connect(self.user_toggled_charging)
        layout.addWidget(self.charging_button)
        
        # Set layout
        self.setLayout(layout)
        
        # --- State Management ---
        self.charging_state = False
        self.manual_override = False # prevents feedback loops while clicking
        
        # --- Timer Loop ---
        # This acts as our "main loop" for both GUI updates and ROS processing
        self.timer = QTimer()
        self.timer.timeout.connect(self.timer_spin)
        self.timer.start(100)  # Run every 100 ms

    def timer_spin(self):
        """
        This function runs 10 times a second. 
        It processes incoming ROS messages AND publishes current GUI state.
        """
        # 1. Process incoming ROS messages (Subscription callbacks fire here)
        rclpy.spin_once(self, timeout_sec=0)
        
        # 2. Publish current state
        self.publish_topics()

    def charging_callback(self, msg):
        """Called when an external node updates the charging status."""
        # Only update GUI if the value is actually different to avoid flicker
        if self.charging_state != msg.data:
            self.charging_state = msg.data
            self.update_button_visuals()

    def user_toggled_charging(self):
        """Called when the USER clicks the button."""
        # Update internal state based on button
        self.charging_state = self.charging_button.isChecked()
        self.update_button_visuals()
        
        # Immediately publish the new state so the system reacts fast
        self.publish_topics()

    def update_button_visuals(self):
        """Syncs the button text/checked state with self.charging_state"""
        # Block signals briefly so setting 'Checked' doesn't trigger 'clicked' again
        self.charging_button.blockSignals(True)
        self.charging_button.setChecked(self.charging_state)
        self.charging_button.setText(str(self.charging_state))
        self.charging_button.blockSignals(False)

    def publish_topics(self):
        robot_msg = String()
        person_msg = String()
        charging_msg = Bool()
        
        robot_msg.data = self.robot_combo.currentText()
        person_msg.data = self.person_combo.currentText()
        charging_msg.data = bool(self.charging_state)
        
        self.robot_pub.publish(robot_msg)
        self.person_pub.publish(person_msg)
        self.charging_pub.publish(charging_msg)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui_node = RobotControlGUI()
    gui_node.show()
    
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()