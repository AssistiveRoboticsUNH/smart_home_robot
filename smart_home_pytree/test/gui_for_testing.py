#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QComboBox, QPushButton, QLabel, QHBoxLayout
from PyQt5.QtCore import QTimer
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

# Define some example locations (replace with your YAML-loaded locations if needed)
LOCATIONS = ["kitchen", "living_room", "bedroom", "home"]

class RobotControlGUI(Node, QWidget):
    def __init__(self):
        rclpy.init(args=None)
        Node.__init__(self, "robot_control_gui")
        QWidget.__init__(self)
        
        self.setWindowTitle("Robot Control GUI")
        self.setGeometry(200, 200, 300, 200)
        
        # Publishers
        self.robot_pub = self.create_publisher(String, 'robot_location', 10)
        self.person_pub = self.create_publisher(String, 'person_location', 10)
        self.charging_pub = self.create_publisher(Bool, 'charging', 10)
        
        # GUI Layout
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
        self.charging_button.clicked.connect(self.toggle_charging)
        layout.addWidget(self.charging_button)
        
        # Set layout
        self.setLayout(layout)
        
        # Timer to continuously publish
        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_topics)
        self.timer.start(200)  # Publish every 200 ms
        
        # Current states
        self.charging_state = False
        
    def toggle_charging(self):
        self.charging_state = self.charging_button.isChecked()
        self.charging_button.setText(str(self.charging_state))
        
    def publish_topics(self):
        robot_msg = String()
        person_msg = String()
        charging_msg = Bool()
        
        robot_msg.data = self.robot_combo.currentText()
        person_msg.data = self.person_combo.currentText()
        charging_msg.data = self.charging_state
        
        self.robot_pub.publish(robot_msg)
        self.person_pub.publish(person_msg)
        self.charging_pub.publish(charging_msg)
        
        # Optional: print for debug
        # print(f"Robot: {robot_msg.data}, Person: {person_msg.data}, Charging: {charging_msg.data}")
        

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui_node = RobotControlGUI()
    gui_node.show()
    
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()
