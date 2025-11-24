#!/usr/bin/env python3
"""
Payload Info Module

This module defines the PayloadInfo class, which is responsible for configuring
the drone to send telemetry data at a desired frequency, retrieving that data
via MAVLink, and publishing it to a ROS2 topic using a provided publisher.
"""

from pymavlink import mavutil
from sid_interface.msg import Telem
from rclpy.node import Node
from std_msgs.msg import Header
import math


class PayloadInfo:
    """
    Class for handling drone telemetry information via MAVLink.
    """

    def __init__(self, master, telem_publisher, payload_info_frequency,
                 node: Node) -> None:
        self.master = master
        self.telem_publisher = telem_publisher
        self.payload_info_frequency = payload_info_frequency
        self.node = node

        self.__startListening()

    def __startListening(self) -> None:
        """
        Configure the drone to send telemetry messages.
        """
        freq = self.payload_info_frequency

        self.__requestMessageInterval(
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            freq
        )
        self.__requestMessageInterval(
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            freq
        )
        self.__requestMessageInterval(
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            freq
        )
        self.__requestMessageInterval(
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
            freq
        )
        # Prefer SCALED_IMU; fall back to RAW_IMU if needed
        self.__requestMessageInterval(
            mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU,
            freq
        )
        self.__requestMessageInterval(
            mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,
            freq
        )

    def __getData(self) -> Telem:
        """
        Retrieve the latest telemetry data from the drone.
        """
        output = Telem()
        output.header = Header()
        output.header.stamp = self.node.get_clock().now().to_msg()

        # Wait for at least one of these to update master.messages
        msg = self.master.recv_match(
            type=[
                'LOCAL_POSITION_NED',
                'ATTITUDE',
                'ATTITUDE_QUATERNION',
                'GLOBAL_POSITION_INT',
                'SCALED_IMU',
                'RAW_IMU',
            ],
            blocking=True
        )

        try:
            # ---------------- GPS / GLOBAL POSITION ----------------
            gp = self.master.messages['GLOBAL_POSITION_INT']
            # Convert from MAVLink units:
            # lat/lon: 1e7 deg, alt: mm, hdg: cdeg
            output.lat = gp.lat / 1e7
            output.lon = gp.lon / 1e7
            output.alt = gp.alt / 1000.0
            output.heading = gp.hdg / 100.0  # degrees

            # ---------------- ATTITUDE QUATERNION ----------------
            aq = self.master.messages['ATTITUDE_QUATERNION']
            output.qx = aq.q1
            output.qy = aq.q2
            output.qz = aq.q3
            output.qw = aq.q4

            # ---------------- ATTITUDE (Euler + rates) ----------------
            att = self.master.messages['ATTITUDE']
            output.roll = att.roll       # rad
            output.pitch = att.pitch     # rad
            output.yaw = att.yaw         # rad

            output.roll_rate = att.rollspeed   # rad/s
            output.pitch_rate = att.pitchspeed # rad/s
            output.yaw_rate = att.yawspeed     # rad/s

            # ---------------- LOCAL POSITION (NED) ----------------
            lp = self.master.messages['LOCAL_POSITION_NED']
            output.x = lp.x
            output.y = lp.y
            output.z = lp.z
            output.vx = lp.vx
            output.vy = lp.vy
            output.vz = lp.vz

            # ---------------- IMU DATA ----------------
            # Prefer SCALED_IMU if available (nice physical units)
            if 'SCALED_IMU' in self.master.messages:
                imu = self.master.messages['SCALED_IMU']
                # ArduPilot: accel in milli-g (mG), gyro in deg/s, mag in milliGauss or raw
                mG_to_mps2 = 9.80665 / 1000.0
                deg_to_rad = math.pi / 180.0

                output.ax = imu.xacc * mG_to_mps2
                output.ay = imu.yacc * mG_to_mps2
                output.az = imu.zacc * mG_to_mps2

                output.gx = imu.xgyro * deg_to_rad
                output.gy = imu.ygyro * deg_to_rad
                output.gz = imu.zgyro * deg_to_rad

                output.mx = float(imu.xmag)
                output.my = float(imu.ymag)
                output.mz = float(imu.zmag)

            elif 'RAW_IMU' in self.master.messages:
                # Fallback: RAW_IMU (raw sensor units; not scaled)
                imu = self.master.messages['RAW_IMU']
                self.node.get_logger().warn('Using RAW_IMU (unscaled) instead of SCALED_IMU')

                output.ax = float(imu.xacc)
                output.ay = float(imu.yacc)
                output.az = float(imu.zacc)

                output.gx = float(imu.xgyro)
                output.gy = float(imu.ygyro)
                output.gz = float(imu.zgyro)

                output.mx = float(imu.xmag)
                output.my = float(imu.ymag)
                output.mz = float(imu.zmag)

            # Debug print once in a while
            # self.node.get_logger().info(
            #     f"IMU ax={output.ax:.3f}, ay={output.ay:.3f}, az={output.az:.3f}, "
            #     f"gx={output.gx:.3f}, gy={output.gy:.3f}, gz={output.gz:.3f}"
            # )

        except KeyError:
            # If some data is missing, return what we have so far
            return output

        return output

    def __requestMessageInterval(self, message_id: int, frequency_hz: int) -> None:
        """
        Request a specific MAVLink message at a desired frequency.
        """
        if frequency_hz <= 0:
            self.node.get_logger().warn(
                f"Frequency for msg ID {message_id} is <= 0, not requesting stream."
            )
            return

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,                      # confirmation
            message_id,             # message ID
            1e6 / frequency_hz,     # interval in microseconds
            0, 0, 0, 0, 0
        )

    def publishTelemInfo(self) -> Telem:
        """
        Retrieve and publish the latest telemetry information.
        """
        self.__startListening()
        output = self.__getData()
        self.telem_publisher.publish(output)
        return output
