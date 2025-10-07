#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from mavros_msgs.srv import CommandLong
import numpy as np
import time

class SignalSubscriber(Node):
    def __init__(self):
        super().__init__('signal_subscriber')

        # parameters
        self.declare_parameter('min_angle_deg', -90.0)
        self.declare_parameter('max_angle_deg',  90.0)
        self.declare_parameter('min_pwm', 1000)
        self.declare_parameter('max_pwm', 2200)
        self.declare_parameter('servo_start_index', 1)  # map channel 0 -> SERVO1 by default
        self.declare_parameter('cmd_rate_hz', 20.0)     # rate-limit CommandLong

        self.min_angle: float = (self.get_parameter('min_angle_deg').value)
        self.max_angle: float = (self.get_parameter('max_angle_deg').value)
        self.min_pwm: int = (self.get_parameter('min_pwm').value)
        self.max_pwm: int = (self.get_parameter('max_pwm').value)
        self.servo_base: int = (self.get_parameter('servo_start_index').value)
        self.period: float = (1.0 / float(self.get_parameter('cmd_rate_hz').value))
        self._next_send_ts = 0.0
        
        self.subscription = self.create_subscription(
            Float64MultiArray, 'servo_n', self.listener_callback, 10)

        
        # Service client for MAVLink servo commands
        self.servo_client = self.create_client(CommandLong, 'mavros/cmd/command')
        if not self.servo_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('mavros/cmd/command service not ready, will retry')
        
        self.msg_count = 0 # Message counter for logging
        self.get_logger().info("Single signal subscriber initialized. (CommandLong)")
        self.get_logger().info("Listening to servo_n topic")
        self.get_logger().info("Using mavros/cmd/command service for servo control")
        

    def listener_callback(self, msg: Float64MultiArray):
        # Increment message counter
        now = time.monotonic()
        if now < self._next_send_ts:
            return  # Rate limit
        self._next_send_ts = now + self.period
        self.msg_count += 1
        values = np.asarray(msg.data, dtype=np.float64)
        
        # convert incoming array to numpy array for easier handling
        values: np.ndarray = np.array(msg.data, dtype=np.float64)
        
        # Log message reception
        self.get_logger().info(f"Received message #{self.msg_count} with {len(values)} servo values")
        
        # Send individual servo commands for each channel
        for i, radian_value in enumerate(values):
            pwm_value = self.radian_to_pwm(radian_value)
            
            # Create MAVLink servo command request
            request = CommandLong.Request()
            request.command = 183  # MAV_CMD_DO_SET_SERVO
            request.param1 = float(self.servo_base + i)  # Servo number (1-8 for PixRacer)
            request.param2 = float(pwm_value)  # PWM value in microseconds
            request.param3 = request.param4 = request.param5 = request.param6 = request.param7 = 0.0
            request.broadcast = False  # Send to specific vehicle
            self.servo_client.call_async(request)
            
            
            # Log the command (less frequently to avoid spam)
            if self.msg_count % 10 == 0:  # Log every 10th message
                self.get_logger().info(f"Sent {len(values)} cmd(s). Example ch{self.servo_base}: {pwm_value}us")

    # --- Helper functions for conversions servo control ---  
    def radian_to_pwm(self, radian_value: float) -> int:

        """
        Convert angle to radians to PWM microseconds
        Default:
            For right now with SG90 servo: -90 to +90 degrees maps to 1000 to 2000 microseconds
            For other servos, adjust min_angle, max_angle, min_pwm, max_pwm accordingly
        """
        degree_value = np.rad2deg(radian_value)

        # Normalize degree value to [0, 1]
        alpha_norm = (degree_value - self.min_angle) / (self.max_angle - self.min_angle)

        # Map alpha_norm to PWM range
        pwm_value = int(np.clip(self.min_pwm + alpha_norm * (self.max_pwm - self.min_pwm), self.min_pwm, self.max_pwm))
        return pwm_value

def main(args=None):
    rclpy.init(args=args)
    signal_subscriber = SignalSubscriber()

    try:
        rclpy.spin(signal_subscriber)
    except KeyboardInterrupt:
        pass

    signal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()