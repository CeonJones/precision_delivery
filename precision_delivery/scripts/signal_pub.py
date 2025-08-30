#!/usr/bin/env python3
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
                ('servo_num', 1)
                ('amp_deg', 5.0)
                ('min_freq_hz', 0.1)
                ('max_freq_hz', 1.5)
                ('time_step', 0.02)
                ('total_time', 15.0)
                ('save_csv', False)
                ('csv_path', '')
                ('csv_filename', 'input_signal({timestr}).csv')
            ]
        )

        # caching most frequently used parameter (amount of servos)
        self.servo_num: int = (self.get_parameter('servo_num').value)

        # building maneuver and saving csv
        self.maneuver = self.build_maneuver(save_csv=bool(self.get_parameter('save_csv').value),
                                            csv_path=self.get_parameter('csv_path').value,
                                            csv_filename=self.get_parameter('csv_filename').value)

        # publishing servo_n topic for subscriber 
        self.array_pub: Publisher = self.create_publisher(Float64MultiArray, 'servo_n')
        
        self.timer = self.create_timer(self.maneuver['dt'], self.timer_callback)

    def build_maneuver(self, 
                       save_csv: bool = False,
                       csv_path: str = '',
                       csv_filename: str = 'input_signal({timestr}).csv') -> Dict[str, np.ndarray]:
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
        time, signal, = multi_sine(
            amp_rad, 
            min_freq_hz, 
            max_freq_hz, 
            time_step, 
            total_time, 
            num_channels=self.servo_num)

        return {'time': time, 'signal': signal, 'time_step': time_step, 'total_time': total_time}

def main():
    rclpy.init()
    node = MultisinePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

