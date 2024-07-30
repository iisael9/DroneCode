import time
from pymavlink import mavutil

# Connection parameters (replace with your actual values)
master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)


def get_gps_coordinates():
    """
    Fetches the drone's current GPS coordinates.

    Returns:
        (float, float, float): The latitude, longitude, and altitude in meters.
    """
    while True:
        master.mav.message_factory.global_position_int_send(
            master.target_system, 0, int(time.time() * 1e6), 0, 0, 0, 0)
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg is not None:
            latitude = msg.lat / 10**7
            longitude = msg.lon / 10**7
            altitude = msg.alt / 1000
            return latitude, longitude, altitude


def arm_and_takeoff(target_altitude):
    """
    Arms the drone and takes it off to the specified target altitude.

    Args:
        target_altitude (float): The desired altitude in meters.
    """
    master.wait_heartbeat()
    print("Heartbeat!")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 1, 0, 0, 0, 0, 0)
    master.motors_armed_wait()
    print('Armed!')

    while master.mav.altitude < target_altitude:
        time.sleep(0.5)
        master.mav.set_position_target_local_ned_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0, 0, 0, 0, 0, 0, target_altitude, 0, 0, 0)


def hover_and_return(current_altitude, hover_duration):
    """
    Holds the drone at its current altitude for a specified duration and then returns to launch.

    Args:
        current_altitude (float): The drone's current altitude in meters.
        hover_duration (float): The duration to hover in seconds.
    """
    hover_altitude = current_altitude + 1.5  # Set hover altitude 5ft above current

    start_time = time.time()
    while time.time() - start_time < hover_duration:
        master.mav.set_position_target_local_ned_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0, 0, 0, 0, 0, 0, hover_altitude, 0, 0, 0)

    # Initiate RTL
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0,
        0, 0, 0, 0, 0, 0, 0)


if __name__ == "__main__":
    # Get current GPS coordinates
    current_lat, current_lon, current_alt = get_gps_coordinates()

    # Arm and take off to a safe altitude (adjust as needed)
    arm_and_takeoff(5)

    # Hover at 5ft above current altitude for 15 seconds
    hover_and_return(current_alt, 15)
