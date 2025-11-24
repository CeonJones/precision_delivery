#!/usr/bin/env python3
"""
Payload Telemetry Node

- Connects to Pixracer via MAVLink (through mavlink-router UDP endpoint).
- Requests attitude, position, and IMU streams.
- Publishes them as a sid_interface/Telem message at a fixed rate.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from pymavlink import mavutil

from sid_interface.msg import Telem


class PayloadTelemNode(Node):
    """
    ROS2 node that:
      - connects to MAVLink
      - configures message intervals
      - reads MAVLink messages
      - publishes Telem messages
    """

    def __init__(self):
        super().__init__('payload_telem_node')
        self.get_logger().info('Payload telemetry node started')

        # Parameters
        self.declare_parameter('payload_frequency', 30)                  # Hz
        self.declare_parameter('mav_connection_string', 'udp:127.0.0.1:14551')

        self.payload_frequency = self.get_parameter(
            'payload_frequency').get_parameter_value().integer_value
        self.mav_connection_string = self.get_parameter(
            'mav_connection_string').get_parameter_value().string_value

        # Init MAVLink connection
        self._init_master_connection()

        # Publisher
        self.telem_pub = self.create_publisher(Telem, 'telem', 10)

        # Configure telemetry streams once
        self._start_listening()

        # Timer to periodically poll MAVLink and publish Telem
        self.timer = self.create_timer(
            1.0 / float(self.payload_frequency),
            self.timer_callback
        )

    # --------------------------------------------------------------------- #
    # MAVLink connection + stream configuration
    # --------------------------------------------------------------------- #

    def _init_master_connection(self) -> None:
        """
        Initialize the MAVLink master connection.
        """
        self.get_logger().info(f'mav_connection_string: {self.mav_connection_string}')

        # Connect via mavlink-router UDP endpoint
        self.master: mavutil.mavlink_connection = mavutil.mavlink_connection(
            self.mav_connection_string
        )
        self.master.wait_heartbeat()
        self.get_logger().info(
            f"Connected to sysid={self.master.target_system}, "
            f"compid={self.master.target_component}"
        )

        # Send one heartbeat so mavlink-router latches this endpoint
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )

    def _start_listening(self) -> None:
        """
        Configure the drone to send telemetry messages.
        """
        freq = self.payload_frequency

        self._request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            freq
        )
        self._request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            freq
        )
        self._request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            freq
        )
        self._request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
            freq
        )
        # IMU-ish messages: prefer SCALED_IMU; RAW_IMU as fallback
        self._request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU,
            freq
        )
        self._request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,
            freq
        )

    def _request_message_interval(self, msg_id: int, frequency_hz: int) -> None:
        """
        Request a specific MAVLink message at a desired frequency.
        """
        if frequency_hz <= 0:
            self.get_logger().warn(
                f"Frequency for msg ID {msg_id} is <= 0, not requesting stream."
            )
            return

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,                      # confirmation
            msg_id,                 # message ID
            1e6 / frequency_hz,     # interval in microseconds
            0, 0, 0, 0, 0
        )

    # --------------------------------------------------------------------- #
    # Data path: MAVLink -> Telem
    # --------------------------------------------------------------------- #

    def _get_telem(self) -> Telem:
        """
        Retrieve the latest telemetry data from the drone and pack into Telem.
        """
        telem = Telem()
        telem.header = Header()
        telem.header.stamp = self.get_clock().now().to_msg()

        # Wait for at least one of these to update master.messages
        _ = self.master.recv_match(
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
            # lat / lon: 1e7 deg, alt: mm, hdg: cdeg
            telem.lat = gp.lat / 1e7
            telem.lon = gp.lon / 1e7
            telem.alt = gp.alt / 1000.0
            telem.heading = gp.hdg / 100.0  # degrees

            # ---------------- ATTITUDE QUATERNION ----------------
            aq = self.master.messages['ATTITUDE_QUATERNION']
            telem.qx = aq.q1
            telem.qy = aq.q2
            telem.qz = aq.q3
            telem.qw = aq.q4

            # ---------------- ATTITUDE (Euler + rates) ----------------
            att = self.master.messages['ATTITUDE']
            telem.roll = att.roll       # rad
            telem.pitch = att.pitch     # rad
            telem.yaw = att.yaw         # rad

            telem.roll_rate = att.rollspeed     # rad/s
            telem.pitch_rate = att.pitchspeed   # rad/s
            telem.yaw_rate = att.yawspeed       # rad/s

            # ---------------- LOCAL POSITION (NED) ----------------
            lp = self.master.messages['LOCAL_POSITION_NED']
            telem.x = lp.x
            telem.y = lp.y
            telem.z = lp.z
            telem.vx = lp.vx
            telem.vy = lp.vy
            telem.vz = lp.vz

            # ---------------- IMU DATA ----------------
            # Prefer SCALED_IMU if available (nice physical units)
            if 'SCALED_IMU' in self.master.messages:
                imu = self.master.messages['SCALED_IMU']
                # ArduPilot: accel in milli-g (mG), gyro in deg/s, mag in some scaled units
                mG_to_mps2 = 9.80665 / 1000.0
                deg_to_rad = math.pi / 180.0

                telem.ax = imu.xacc * mG_to_mps2
                telem.ay = imu.yacc * mG_to_mps2
                telem.az = imu.zacc * mG_to_mps2

                telem.gx = imu.xgyro * deg_to_rad
                telem.gy = imu.ygyro * deg_to_rad
                telem.gz = imu.zgyro * deg_to_rad

                telem.mx = float(imu.xmag)
                telem.my = float(imu.ymag)
                telem.mz = float(imu.zmag)

            elif 'RAW_IMU' in self.master.messages:
                # Fallback: RAW_IMU (raw sensor units; not scaled)
                imu = self.master.messages['RAW_IMU']
                self.get_logger().warn('Using RAW_IMU (unscaled) instead of SCALED_IMU')

                telem.ax = float(imu.xacc)
                telem.ay = float(imu.yacc)
                telem.az = float(imu.zacc)

                telem.gx = float(imu.xgyro)
                telem.gy = float(imu.ygyro)
                telem.gz = float(imu.zgyro)

                telem.mx = float(imu.xmag)
                telem.my = float(imu.ymag)
                telem.mz = float(imu.zmag)
            else:
                self.get_logger().warn('No IMU data (SCALED_IMU or RAW_IMU) available!')

        except KeyError:
            # If some data is missing, just return what we have (zeros for missing)
            return telem

        return telem

    # --------------------------------------------------------------------- #
    # Timer callback
    # --------------------------------------------------------------------- #

    def timer_callback(self):
        telem = self._get_telem()
        self.telem_pub.publish(telem)
        # Optional debug:
        # self.get_logger().info(
        #     f"IMU ax={telem.ax:.3f}, ay={telem.ay:.3f}, az={telem.az:.3f}"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = PayloadTelemNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
