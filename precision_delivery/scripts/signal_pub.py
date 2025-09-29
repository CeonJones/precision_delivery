#!/usr/bin/env python3
from fileinput import filename
from re import S
import rclpy
import math
import numpy as np

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.publisher import Publisher
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64, Float64MultiArray
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os, threading, time
from typing import Dict

from ros2_sid.inputdesign import multi_sine, _save_input_signal

timestr = time.strftime("%Y%m%d-%H%M%S")

class MultisinePublisher(Node):
    def __init__(self, ns=''):
        super().__init__('multisine_publisher')
        
        # runtime tunable parameters
        self.declare_parameters(
            namespace=ns,
            
            # static verison of multisine parameters
            parameters=[
                ('servo_num', 1),
                ('amp_deg', 5.0),
                ('min_freq_hz', 0.1),
                ('max_freq_hz', 1.5),
                ('time_step', 0.02),
                ('total_time', 15.0),
                ('save_csv', True),
                ('csv_path', ''),
                ('csv_filename', f'input_signal({timestr}).csv')
            ]
        )

        # caching most frequently used parameter (amount of servos)
        self.servo_num: int = (self.get_parameter('servo_num').value)

        # building maneuver
        self.maneuver = self.build_maneuver()

        # publishing servo_n topic for subscriber 
        self.array_pub: Publisher = self.create_publisher(Float64MultiArray, 'servo_n', 10)

        # internal counter for maneuver progress
        self.k: int = 0
        
        self.timer = self.create_timer(self.maneuver['time_step'], self.timer_callback)
    
    def timer_callback(self):
        if self.k < len(self.maneuver['time']):

            row = self.maneuver['signal'][self.k, :]
            msg = Float64MultiArray()
            msg.data = row.tolist()
            self.array_pub.publish(msg)
            self.k += 1
        else:
            self.get_logger().info('Maneuver complete.')
            self.timer.cancel()

    def build_maneuver(self):
        """
        build an N-channel multisine manuever and optionally save it to CSV for rerun

        return a dictionary with:
        - time: (N_samples,) float64
        - signal: (N_samples, servo_num) float64
        - time_step: float (time_step)
        """    
        
        amp_deg: float = self.get_parameter('amp_deg').value
        min_freq_hz: float = self.get_parameter('min_freq_hz').value
        max_freq_hz: float = self.get_parameter('max_freq_hz').value
        time_step: float = self.get_parameter('time_step').value
        total_time: float = self.get_parameter('total_time').value
        save_csv: bool = self.get_parameter('save_csv').value
        csv_path: str = self.get_parameter('csv_path').value
        csv_filename: str = self.get_parameter('csv_filename').value

        # error handling for multisine metrics
        if time_step <= 0.0:
            raise ValueError("time step must > 0")
        if total_time <= 0.0:
            raise ValueError("total time must > 0")
        if max_freq_hz <= min_freq_hz:
            raise ValueError("max frequency must > min frequency")
        if self.servo_num <= 0:
            raise ValueError("servo number must >= 1")

        amp_rad  = np.deg2rad(amp_deg)
        
        # Print signal generation parameters
        print(f"\n=== Multisine Generation ===")
        print(f"Amplitude: {amp_deg}° ({amp_rad:.4f} rad)")
        print(f"Frequency Range: {min_freq_hz:.2f} - {max_freq_hz:.2f} Hz")
        print(f"Time Step: {time_step:.4f}s ({1/time_step:.1f} Hz sample rate)")
        print(f"Total Time: {total_time:.1f}s")
        print(f"Servo Channels: {self.servo_num}")
        
        time, signal, *_ = multi_sine(
            amp_rad, 
            min_freq_hz, 
            max_freq_hz, 
            time_step, 
            total_time, 
            num_channels=self.servo_num)
        
        # Print signal characteristics
        print(f"\n=== Generated Signal ===")
        print(f"Signal Shape: {signal.shape} (samples x channels)")
        print(f"Time Vector Length: {len(time)} samples")
        print(f"Signal Range: [{signal.min():.4f}, {signal.max():.4f}] rad")
        print(f"Signal Range: [{np.rad2deg(signal.min()):.2f}, {np.rad2deg(signal.max()):.2f}]°")
        print(f"First 5 signal values (channel 0): {signal[:5, 0]}")
        print(f"RMS value per channel: {np.sqrt(np.mean(signal**2, axis=0))}")
        
        if save_csv:
            mat = np.column_stack((time, signal))
            # Save to precision_delivery package directory (one level up from scripts/)
            scripts_dir = os.path.dirname(os.path.abspath(__file__))
            package_dir = os.path.dirname(scripts_dir)  # Go up one level from scripts/
            filepath = os.path.join(package_dir, csv_filename)
            _save_input_signal(mat, filename=filepath)
            print(f"Signal saved to: {filepath}")

        return {'time': time, 'signal': signal, 'time_step': time_step, 'total_time': total_time}

def main(args=None):
    rclpy.init(args=args)
    signal_pub = MultisinePublisher()
    while rclpy.ok():
        try:
            rclpy.spin(signal_pub)
        
        except KeyboardInterrupt:
            break
    
    signal_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()