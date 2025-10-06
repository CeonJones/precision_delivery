#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial
import re
import numpy as np

class Feedback(Node):
    def __init__(self):
        super().__init__('servo_feedback')
        # open serial connection to Arduino
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.pub = self.create_publisher(Float64, 'servo_feedback', 10)

        # calibration values from your sweep
        self.raw_min = 110   # example: ADC reading at minPulse
        self.raw_max = 427   # example: ADC reading at maxPulse
        self.angle_min = 0.0
        self.angle_max = 180.0

        self.get_logger().info("Servo feedback node started. Reading from /dev/ttyUSB0.")
        self.timer = self.create_timer(0.02, self.read_serial)  # 50 Hz

        # regex to parse "us = #### | raw = ###"
        self.pattern = re.compile(r"us\s*=\s*(\d+)\s*\|\s*raw\s*=\s*(\d+)")

    def read_serial(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        match = self.pattern.search(line)
        if match:
            raw_value = int(match.group(2))
            angle_deg = self.map_raw_to_angle(raw_value)
            msg = Float64()
            msg.data = np.deg2rad(angle_deg)
            self.pub.publish(msg)
            self.get_logger().info(f"raw={raw_value} → {angle_deg:.2f}°")

    def map_raw_to_angle(self, raw):
        # clamp and map ADC to angle
        raw = np.clip(raw, self.raw_min, self.raw_max)
        norm = (raw - self.raw_min) / (self.raw_max - self.raw_min)
        angle = self.angle_min + norm * (self.angle_max - self.angle_min)
        return angle
    
def main(args=None):
    rclpy.init(args=args)
    feedback_node = Feedback()
    while rclpy.ok():
        try:
            rclpy.spin_once(feedback_node)
        except KeyboardInterrupt:
            break
        feedback_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()