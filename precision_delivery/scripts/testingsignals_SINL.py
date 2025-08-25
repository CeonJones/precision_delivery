#!/usr/bin/env python3
from re import S
import rclpy
import math
import os
import numpy as np

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.publisher import Publisher
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from drone_interfaces.msg import Telem, CtlTraj
import threading

from ros2_sid.inputdesign import frequency_sweep, multi_step, multi_sine


class PubInputSignals(Node):
    def __init__(self, ns=''):
        super().__init__('excitation_node')
        self.run_switch: int = 0
        self.maneuver_mode: int = 1
        self.maneuvers()
        self.initial_counter: int = 0
        self.initial_timer_period: float = 0.02

        self.counter: int = self.initial_counter
        self.input_signal: Publisher = self.create_publisher(
            CtlTraj, 'trajectory', 10)
        self.current_timer_period: float = self.initial_timer_period
        self.timer = self.create_timer(
            self.current_timer_period, self.logic_loop)
        
        self.userthread = threading.Thread(target=self.user_input_loop, daemon=True)
        self.userthread.start()

    def maneuvers(self) -> None:
        # Maneuvers can either be imported from a pre-saved CSV file (e.g., 'input_signal.csv') or generated live when the node starts.
        # Using a CSV allows for reproducibility and avoids regenerating signals on each run.
        # To create and save a maneuver, use the 'save_input_signal()' function in 'inputdesign.py'.
        # All maneuver arrays must have shape (N, 4), where the columns are:
        # [time, roll signal, pitch signal, yaw signal], and the first time value must be zero.

        file_path = "/develop_ws/src/ros2_sid/ros2_sid/ros2_sid/input_signal.csv"
        data = np.loadtxt(file_path, delimiter=',', skiprows=1)
        time = data[:, 0]
        empty = np.zeros_like(time)
        self.allsines = np.array([time, data[:, 1], data[:, 2], data[:, 3]]).T

        self.rolsines = np.array([time, data[:, 1], empty, empty]).T

        self.pitsines = np.array([time, empty, data[:, 2], empty]).T

        self.yawsines = np.array([time, empty, empty, data[:, 3]]).T
        

        amplitude: float = np.deg2rad(5)  
        natural_frequency: float = 1.0
        pulses: list = [1, 1]
        time_delay: float = 1.0
        time_step: float = 0.02
        final_time: float = 15.
        time, doublet = multi_step(amplitude, natural_frequency, pulses, time_delay, time_step, final_time)
        empty = np.zeros_like(time)
        self.roldoublet = np.array([time, doublet, empty, empty]).T
        
        amplitude: float = np.deg2rad(5)
        natural_frequency: float = 1.0
        pulses: list = [1, 1]
        time_delay: float = 1.0
        time_step: float = 0.02
        final_time: float = 15.
        time, doublet = multi_step(amplitude, natural_frequency, pulses, time_delay, time_step, final_time)
        empty = np.zeros_like(time)
        self.pitdoublet = np.array([time, empty, doublet, empty]).T
        
        amplitude: float = np.deg2rad(10)
        natural_frequency: float = 1.0
        pulses: list = [1, 1]
        time_delay: float = 1.0
        time_step: float = 0.02
        final_time: float = 15.
        time, doublet = multi_step(amplitude, natural_frequency, pulses, time_delay, time_step, final_time)
        empty = np.zeros_like(time)
        self.yawdoublet = np.array([time, empty, empty, doublet]).T


        amplitude: float = np.deg2rad(5) 
        minimum_frequency: float = 0.1
        maximum_frequency: float = 1.5
        time_step: float = 0.02
        final_time: float = 15.
        time, sweep = frequency_sweep(amplitude, minimum_frequency, maximum_frequency, time_step, final_time)
        empty = np.zeros_like(time)
        self.rolsweep = np.array([time, sweep, empty, empty]).T

        amplitude: float = np.deg2rad(5)
        minimum_frequency: float = 0.1
        maximum_frequency: float = 1.5
        time_step: float = 0.02
        final_time: float = 15.
        time, sweep = frequency_sweep(amplitude, minimum_frequency, maximum_frequency, time_step, final_time)
        empty = np.zeros_like(time)
        self.pitsweep = np.array([time, empty, sweep, empty]).T

        amplitude: float = np.deg2rad(10)
        minimum_frequency: float = 0.1
        maximum_frequency: float = 1.5
        time_step: float = 0.02
        final_time: float = 15.
        time, sweep = frequency_sweep(amplitude, minimum_frequency, maximum_frequency, time_step, final_time)
        empty = np.zeros_like(time)
        self.yawsweep = np.array([time, empty, empty, sweep]).T
        
    def user_input_loop(self) -> None:
        while rclpy.ok():
            try:
                userswitch = int(input("\nTesting Switch (0-1):\n"))
                if userswitch not in [0, 1]:
                    print("Invalid switch. Enter 0 or 1.")
                    continue
            except ValueError:
                print("Invalid input. Please enter an integer (0 or 1).")
                continue
            if userswitch != self.run_switch:
                self.run_switch = userswitch

            # TODO: Make this code match the number and type of maneuvers.
            if self.run_switch == 0:
                print("\nManeuvers")
                print("=========")
                print("0: All Multisines")
                print("1: Multisine - Roll")
                print("2: Multisine - Pitch")
                print("3: Multisine - Yaw")
                print("4: Doublet - Roll")
                print("5: Doublet - Pitch")
                print("6: Doublet - Yaw")
                print("7: Sweep - Roll")
                print("8: Sweep - Pitch")
                print("9: Sweep - Yaw")
                while True:
                    try:
                        maneuver_input = int(input("\nEnter a Maneuver (0-9):\n"))
                        if maneuver_input not in range(0, 10):
                            print("Invalid maneuver. Enter a number between 0 and 9.")
                            continue
                        self.maneuver_mode = maneuver_input
                        break
                    except ValueError:
                        print("Invalid input. Please enter an integer between 0 and 9.")

    def logic_loop(self) -> None:
        if (self.run_switch == 1):
            if (self.counter == 0):
                # TODO: Make this code match the number and type of maneuvers.
                if (self.maneuver_mode == 0):
                    self.current_maneuver = self.allsines
                elif (self.maneuver_mode == 1):
                    self.current_maneuver = self.rolsines
                elif (self.maneuver_mode == 2):
                    self.current_maneuver = self.pitsines
                elif (self.maneuver_mode == 3):
                    self.current_maneuver = self.yawsines
                elif (self.maneuver_mode == 4):
                    self.current_maneuver = self.roldoublet
                elif (self.maneuver_mode == 5):
                    self.current_maneuver = self.pitdoublet
                elif (self.maneuver_mode == 6):
                    self.current_maneuver = self.yawdoublet
                elif (self.maneuver_mode == 7):
                    self.current_maneuver = self.rolsweep
                elif (self.maneuver_mode == 8):
                    self.current_maneuver = self.pitsweep
                elif (self.maneuver_mode == 9):
                    self.current_maneuver = self.yawsweep
                else:
                    self.current_maneuver = None
                
                if (self.current_maneuver is not None):
                    maneuver_timer_period: float = self.current_maneuver[1, 0]
                else:
                    maneuver_timer_period: float = self.initial_timer_period
                
                if (maneuver_timer_period != self.current_timer_period):
                    self.update_timer_period(maneuver_timer_period)

            if (self.current_maneuver is not None):
                if (self.counter < len(self.current_maneuver)):
                    self.publish_trajectory()
                    self.counter += 1
                else:
                    self.run_switch = 0
                    print("MANEUVER COMPLETE")

            else:
                self.run_switch = 0
                print("NO CURRENT MANEUVER")

        elif (self.run_switch == 0):
            self.counter = self.initial_counter

    def update_timer_period(self, new_timer_period) -> None:
        if (self.timer is not None):
            self.timer.cancel()

        self.current_timer_period = new_timer_period
        self.timer = self.create_timer(
            self.current_timer_period, self.logic_loop)
        
    def publish_trajectory(self) -> None:
        if (self.current_maneuver is not None):
            trajectory: CtlTraj = CtlTraj()
            # trajectory.header.stamp = self.get_clock().now().to_msg()
            trajectory.roll  = [self.current_maneuver[self.counter, 1], self.current_maneuver[self.counter, 1]]
            trajectory.pitch = [self.current_maneuver[self.counter, 2], self.current_maneuver[self.counter, 2]]
            trajectory.yaw   = [self.current_maneuver[self.counter, 3], self.current_maneuver[self.counter, 3]]
            trajectory.thrust = [0.5, 0.5]
            trajectory.idx = 0
            self.input_signal.publish(trajectory)
            # print(f"Publishing trajectory: {trajectory.roll}, {trajectory.pitch}, {trajectory.yaw}")


def main(args=None):
    rclpy.init(args=args)
    pub_signals = PubInputSignals()

    while rclpy.ok():
        try:
            rclpy.spin_once(pub_signals, timeout_sec=0.1)

        except KeyboardInterrupt:
            break

    pub_signals.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
