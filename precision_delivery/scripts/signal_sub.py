#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandLong
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
        
        # Service client for MAVLink servo commands
        self.servo_client = self.create_client(CommandLong, 'mavros/cmd/command')
        
        # Wait for service to be available (but don't block startup)
        if not self.servo_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('MAVROS cmd/command service not available - servo commands will be queued')
        else:
            self.get_logger().info('MAVROS cmd/command service available')
        
        self.get_logger().info("Servo signal subscriber initialized.")
        self.get_logger().info("Listening to servo_n topic")
        self.get_logger().info("Using mavros/cmd/command service for servo control")
        
        # Add message counter for debugging
        self.message_count = 0

    def listener_callback(self, msg: Float64MultiArray):
        # Increment message counter
        self.message_count += 1
        
        # convert incoming array to numpy array for easier handling
        values: np.ndarray = np.array(msg.data, dtype=np.float64)
        
        # Log message reception
        self.get_logger().info(f"Received message #{self.message_count} with {len(values)} servo values")
        
        # Send individual servo commands for each channel
        for i, radian_value in enumerate(values):
            pwm_value = self.radian_to_pwm(radian_value)
            
            # Create MAVLink servo command request
            request = CommandLong.Request()
            request.command = 183  # MAV_CMD_DO_SET_SERVO
            request.param1 = float(i + 1)  # Servo number (1-8 for PixRacer)
            request.param2 = float(pwm_value)  # PWM value in microseconds
            request.param3 = 0.0  # Not used
            request.param4 = 0.0  # Not used
            request.param5 = 0.0  # Not used
            request.param6 = 0.0  # Not used
            request.param7 = 0.0  # Not used
            request.broadcast = False  # Send to specific vehicle
            
            # Call service asynchronously (non-blocking)
            future = self.servo_client.call_async(request)
            
            # Log the command (less frequently to avoid spam)
            if self.message_count % 10 == 0:  # Log every 10th message
                self.get_logger().info(f"Servo {i+1}: {radian_value:.4f} rad -> {pwm_value} PWM")

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