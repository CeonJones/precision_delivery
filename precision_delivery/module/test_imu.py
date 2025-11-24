from pymavlink import mavutil

# Connect to mavlink-router UDP endpoint, NOT /dev/ttyACM0
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
master.wait_heartbeat()
print("Connected:", master.target_system, master.target_component)

# Send at least one packet out so mavlink-router "latches" this endpoint
master.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_GCS,
    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
    0, 0, 0
)

# Ask for all IMU-ish streams at 50 Hz
for msg_id in (
    mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,
    mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU,
    mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2,
    mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU3,
    mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,
):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        1e6 / 50.0,  # 50 Hz
        0, 0, 0, 0, 0
    )

while True:
    msg = master.recv_match(
        type=['RAW_IMU', 'SCALED_IMU', 'SCALED_IMU2', 'SCALED_IMU3', 'HIGHRES_IMU'],
        blocking=True
    )
    print(msg)
    print('---')