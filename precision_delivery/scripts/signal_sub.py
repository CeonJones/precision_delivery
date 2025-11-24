#!/usr/bin/env python3
import time
import math
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node

from sid_interface.msg import SIDCmd   # same message as your multisine publisher
from pymavlink import mavutil


class SignalSubscriber(Node):
    def __init__(self):
        super().__init__('signal_subscriber_pymavlink')

        # --- Parameters ---
        self.declare_parameter('baudrate', 921600)        
        self.declare_parameter('mav_connection_string', 'udp:127.0.0.1:14552')
        self.mav_connection_string = self.get_parameter('mav_connection_string')\
            .get_parameter_value().string_value
        
        self.declare_parameter('min_angle_deg', -90.0)
        self.declare_parameter('max_angle_deg',  90.0)
        self.declare_parameter('min_pwm', 1000)
        self.declare_parameter('max_pwm', 2200)
        self.declare_parameter('servo_start_index', 1)  # map channel 0 -> SERVO1 by default
        self.declare_parameter('cmd_rate_hz', 20.0)     # rate-limit commands

        self.min_angle: float = self.get_parameter('min_angle_deg').value
        self.max_angle: float = self.get_parameter('max_angle_deg').value
        self.min_pwm: int = self.get_parameter('min_pwm').value
        self.max_pwm: int = self.get_parameter('max_pwm').value
        self.servo_base: int = self.get_parameter('servo_start_index').value
        self.period: float = 1.0 / float(self.get_parameter('cmd_rate_hz').value)
        self.baudrate: float = self.get_parameter('baudrate').value
        self._next_send_ts = 0.0
        self.msg_count = 0

        # --- MAVLink connection ---
        self.get_logger().info(f'Connecting to MAVLink on {self.mav_connection_string} @ {self.baudrate}')
        if self.mav_connection_string.startswith('udp:') or self.mav_connection_string.startswith('tcp:'):
            self.master = mavutil.mavlink_connection(self.mav_connection_string)
        else:
            self.master = mavutil.mavlink_connection(self.mav_connection_string, baud=self.baudrate)
        
        self.get_logger().info('Waiting for MAVLink heartbeat...')
        self.master.wait_heartbeat()
        self.get_logger().info(
            f'Heartbeat received from system {self.master.target_system}, '
            f'component {self.master.target_component}'
        )

        # --- Subscriber (same topic as before) ---
        self.subscription = self.create_subscription(
            SIDCmd,
            'servo_n',
            self.listener_callback,
            10
        )

        self.get_logger().info("Signal subscriber (pymavlink) initialized.")
        self.get_logger().info("Listening to 'servo_n' topic.")
        self.get_logger().info("Sending MAV_CMD_DO_SET_SERVO via pymavlink.")

    def listener_callback(self, msg: SIDCmd):
        now = time.monotonic()
        if now < self._next_send_ts:
            return  # rate limit
        self._next_send_ts = now + self.period

        self.msg_count += 1
        values = np.asarray(msg.data, dtype=np.float64)

        self.get_logger().info(
            f"Received message #{self.msg_count} with {len(values)} servo values"
        )

        for i, radian_value in enumerate(values):
            pwm_value = self.radian_to_pwm(radian_value)

            servo_num = self.servo_base + i  # SERVO1, SERVO2, ...
            if servo_num < 1 or servo_num > 8:
                # Pixracer typically supports SERVO1–8
                continue

            try:
                # MAV_CMD_DO_SET_SERVO = 183
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0,                  # confirmation
                    float(servo_num),   # param1: servo number (1-8)
                    float(pwm_value),   # param2: PWM in microseconds
                    0, 0, 0, 0, 0       # param3-7: unused
                )
            except Exception as e:
                self.get_logger().error(f"Failed to send servo command: {e}")

        # Log every 10th message like before
        if self.msg_count % 10 == 0 and len(values) > 0:
            self.get_logger().info(
                f"Sent {len(values)} cmd(s). Example ch{self.servo_base}: {pwm_value}us"
            )

    # --- Helper: rad → PWM ---
    def radian_to_pwm(self, radian_value: float) -> int:
        """
        Convert angle in radians to PWM microseconds.
        Default: -90 to +90 deg -> min_pwm to max_pwm.
        """
        degree_value = np.rad2deg(radian_value)

        # Normalize degree value to [0,1] over [min_angle, max_angle]
        alpha_norm = (degree_value - self.min_angle) / (self.max_angle - self.min_angle)
        pwm_value = self.min_pwm + alpha_norm * (self.max_pwm - self.min_pwm)

        # Clip to safe bounds
        pwm_value = int(np.clip(pwm_value, self.min_pwm, self.max_pwm))
        return pwm_value

    def destroy_node(self):
        try:
            if self.master:
                self.master.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SignalSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
