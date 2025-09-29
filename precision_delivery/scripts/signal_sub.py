#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from mavros_msgs.msg import OverrideRCIn
import numpy as np

class SignalSubscriber(Node):
    def __init__(self):
        super().__init__('signal_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'servo_n',
            self.listener_callback,
            10
        )

        # Publisher to MAVROS RC override topic
        self.rc_pub = self.create_publisher(OverrideRCIn, 'mavros/rc/override', 10)
        
        self.get_logger().info("Servo signal subscriber initialized.")
        self.get_logger().info("Listening to servo_n topic")
        self.get_logger().info("Publishing to mavros/rc/override topic")

    def listener_callback(self, msg: Float64MultiArray):
        # convert incoming array to numpy array for easier handling
        values: np.ndarray = np.array(msg.data, dtype=np.float64)

        # Prepare RC override message
        rc_msg = OverrideRCIn()
        rc_msg.channels = [0]*18  # Initialize all channels to 0

        # loop through each available servo channel and print its value
        for i, radian_value in enumerate(values):
            pmw_value = self.radian_to_pwm(radian_value)
            rc_msg.channels[i] = pmw_value
            self.get_logger().info(f"Servo {i}: {radian_value:.4f} rad -> {pmw_value} PWM")

        # Publish the RC override message
        self.rc_pub.publish(rc_msg)

    # --- Helper functions for conversions servo control ---  
    def radian_to_pwm(self, 
                        radian_value: float,
                        min_angle: float = -90.0,
                        max_angle: float = 90.0,
                        min_pwm: int = 1000,
                        max_pwm: int = 2000) -> int:

        """
        Convert angle to radians to PWM microseconds
        Default:
            For right now with SG90 servo: -90 to +90 degrees maps to 1000 to 2000 microseconds
            For other servos, adjust min_angle, max_angle, min_pwn, max_pwn accordingly
        """
        degree_value = np.rad2deg(radian_value)

        # Normalize degree value to [0, 1]
        alpha_norm = (degree_value - min_angle) / (max_angle - min_angle)

        # Map alpha_norm to PWM range
        pwm_value = int(np.clip(min_pwm + alpha_norm * (max_pwm - min_pwm), min_pwm, max_pwm))
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