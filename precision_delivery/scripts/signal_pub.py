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

from ros2_sid.inputdesign import multi_sine

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
                ('total_time', 5.0),
                ('save_csv', True),
                ('csv_path', ''),
                ('csv_filename', f'input_signals{timestr}.csv'),
                ('use_csv', True),
                ('loop', False)
            ]
        )
        
        # fetching parameters
        self.use_csv: bool = (self.get_parameter('use_csv').value)
        self.loop: bool = (self.get_parameter('loop').value)

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
            if self.loop:
                self.k = 0
                self.get_logger().info('Restarting maneuver loop.')
            else:
                self.get_logger().info('Maneuver complete.')
                self.timer.cancel()

    def build_maneuver(self) -> Dict[str, np.ndarray]:
        """
        build an N-channel multisine manuever and optionally save it to CSV for rerun

        return a dictionary with:
        - time: (N_samples,) float64
        - signal: (N_samples, servo_num) float64
        - time_step: float (time_step)
        """
        time_step: float = self.get_parameter('time_step').value
        total_time: float = self.get_parameter('total_time').value
        csv_path: str = self.get_parameter('csv_path').value
        save_csv: bool = self.get_parameter('save_csv').value
        csv_filename: str = self.get_parameter('csv_filename').value

        filepath = os.path.join(csv_path, csv_filename)

        # Optionally load from CSV
        if self.use_csv:
            # Search in the precision_delivery signals directory
            package_share_dir = get_package_share_directory('precision_delivery')
            signals_dir = os.path.join(package_share_dir, 'data', 'signals')
            search_dir = signals_dir
            
            if os.path.exists(search_dir):
                csv_files = [f for f in os.listdir(search_dir) if f.startswith("input_signals") and f.endswith(".csv")]
                if csv_files:
                    latest_file = max(csv_files, key=lambda f: os.path.getmtime(os.path.join(search_dir, f)))
                    filepath = os.path.join(search_dir, latest_file)
                    self.get_logger().info(f"Loading most recent input signal from: {filepath}")
                    data = np.loadtxt(filepath, delimiter=',', skiprows=1)  # Skip header
                    time = data[:, 0]
                    signal_data = data[:, 1:]  # All columns after time are signal channels
                    
                    # Determine number of channels from CSV data
                    num_channels = signal_data.shape[1]
                    self.get_logger().info(f"CSV contains {len(time)} samples with {num_channels} channels")
                    
                    # Check if CSV channels match parameter
                    if num_channels != self.servo_num:
                        self.get_logger().warn(f"CSV has {num_channels} channels but servo_num parameter is {self.servo_num}. Using CSV channel count.")
                        self.servo_num = num_channels
                    
                    return {'time': time, 'signal': signal_data, 'time_step': time_step, 'total_time': total_time}
            else:
                self.get_logger().warn(f"Signals directory not found: {search_dir}")
            
            self.get_logger().warn("No CSV files found — generating new multisine signal.")
        
        # Otherwise, generate new multisine signal
        amp_deg: float = self.get_parameter('amp_deg').value
        min_freq_hz: float = self.get_parameter('min_freq_hz').value
        max_freq_hz: float = self.get_parameter('max_freq_hz').value
        amp_rad  = np.deg2rad(amp_deg)



        # error handling for multisine metrics
        if time_step <= 0.0:
            raise ValueError("time step must > 0")
        if total_time <= 0.0:
            raise ValueError("total time must > 0")
        if max_freq_hz <= min_freq_hz:
            raise ValueError("max frequency must > min frequency")
        if self.servo_num <= 0:
            raise ValueError("servo number must >= 1")
        
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
            # Save to precision_delivery signals directory
            package_share_dir = get_package_share_directory('precision_delivery')
            signals_dir = os.path.join(package_share_dir, 'data', 'signals')
            os.makedirs(signals_dir, exist_ok=True)
            filepath = os.path.join(signals_dir, csv_filename)
            
            # Create header and save CSV
            num_channels = mat.shape[1] - 1
            header = ['time'] + [f'channel_{i+1}' for i in range(num_channels)]
            
            with open(filepath, mode='w', newline='') as file:
                import csv
                writer = csv.writer(file)
                writer.writerow(header)
                writer.writerows(mat)
            
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