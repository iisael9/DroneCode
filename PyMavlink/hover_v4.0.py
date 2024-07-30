import time
from pymavlink import mavutil

# Connection parameters (replace with your actual values)
master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)

def check_battery():
    """
    Checks the battery voltage and warns if it is below a threshold.
    """
    msg = master.recv_match(type='SYS_STATUS', blocking=True)
    if msg is not None:
        battery_voltage = msg.voltage_battery / 1000  # Convert from millivolts to volts
        print(f"Battery Voltage: {battery_voltage}V")
        if battery_voltage < 11.0:  # Example threshold
            print("Warning: Battery voltage is low!")
            return False
    return True

def check_gps():
    """
    Checks the GPS signal quality.
    """
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
    if msg is not None:
        gps_fix = msg.fix_type
        if gps_fix < 3:  # GPS quality: 0=No GPS, 1=No fix, 2=2D fix, 3=3D fix
            print(f"Warning: GPS signal quality is low ({gps_fix})!")
            return False
    return True

def check_system_status():
    """
    Checks if the system is in a healthy state.
    """
    msg = master.recv_match(type='SYS_STATUS', blocking=True)
    if msg is not None:
        print(f"System Status - Load: {msg.load}%, Battery: {msg.voltage_battery/1000}V")
        if msg.load > 80:  # Example threshold
            print("Warning: System load is high!")
            return False
    return True

def check_flight_mode(expected_mode):
    """
    Checks if the drone is in the expected flight mode.
    """
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if msg is not None:
        mode = mavutil.mode_string_v10(msg)
        print(f"Current Flight Mode: {mode}")
        if mode != expected_mode:
            print(f"Warning: Expected flight mode is {expected_mode}.")
            return False
    return True

def get_gps_coordinates():
    """
    Fetches the drone's current GPS coordinates.

    Returns:
        (float, float, float): The latitude, longitude, and altitude in meters.
    """
    if not check_gps():
        print("Cannot fetch GPS coordinates due to poor GPS signal.")
        return None, None, None

    while True:
        # Request GPS messages
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            1,  # 1 Hz stream rate
            1   # Start streaming
        )

        # Wait for a message with GPS data
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg is not None:
            latitude = msg.lat / 10**7  # Convert from degrees*10^7 to degrees
            longitude = msg.lon / 10**7  # Convert from degrees*10^7 to degrees
            altitude = msg.alt / 1000  # Convert from millimeters to meters
            return latitude, longitude, altitude

def arm_and_takeoff(target_altitude):
    """
    Arms the drone and takes it off to the specified target altitude using default climb rate.

    Args:
        target_altitude (float): The desired altitude in meters.
    """
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print("Heartbeat received!")

    # Pre-flight checks
    if not check_battery():
        return
    if not check_gps():
        return
    if not check_system_status():
        return
    if not check_flight_mode("GUIDED"):  # Assuming "GUIDED" mode is desired for takeoff
        return

    # Arm the drone
    print("Arming...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 1, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("Armed!")

    # Takeoff to target altitude
    print(f"Taking off to {target_altitude} meters...")
    while True:
        # Send takeoff command without specifying climb rate
        master.mav.set_position_target_local_ned_send(
            0,  # Time boot ms (usually set to 0)
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Frame of reference
            0b111111000000,  # Type mask (only positions are set, others are ignored)
            0, 0, 0,  # Position coordinates (x, y, z local coordinates)
            0, 0, 0,  # Velocity (vx, vy, vz) (not used)
            0, 0, target_altitude,  # Acceleration (ax, ay, az) (set altitude change if needed)
            0,  # Yaw (no change in heading)
            0  # Yaw rate (no change in yaw rate)
        )
        # Check current altitude
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg is not None:
            current_altitude = msg.alt / 1000  # Convert from millimeters to meters
            if current_altitude >= target_altitude:
                break
        time.sleep(0.5)  # Wait before checking again
    print("Reached target altitude!")

def hover_and_return(current_altitude, hover_duration):
    """
    Holds the drone at its current altitude for a specified duration and then returns to launch.

    Args:
        current_altitude (float): The drone's current altitude in meters.
        hover_duration (float): The duration to hover in seconds.
    """
    hover_altitude = current_altitude + 1.5  # Set hover altitude 1.5 meters above current

    # Hover for the specified duration
    print(f"Hovering at {hover_altitude} meters for {hover_duration} seconds...")
    start_time = time.time()
    while time.time() - start_time < hover_duration:
        # Send hovering command (adjust based on your autopilot)
        master.mav.set_position_target_local_ned_send(
            0,  # Time boot ms (usually set to 0)
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Frame of reference
            0b111111000000,  # Type mask (only positions are set, others are ignored)
            0, 0, 0,  # Position coordinates (x, y, z local coordinates)
            0, 0, 0,  # Velocity (vx, vy, vz)
            0, 0, hover_altitude,  # Acceleration (ax, ay, az)
            0,  # Yaw (no change in heading)
            0  # Yaw rate (no change in yaw rate)
        )
        time.sleep(1)  # Adjust the sleep time as needed to prevent flooding the channel

    # Return to launch (RTL) command
    print("Returning to launch...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

if __name__ == "__main__":
    target_altitude = 15  # Example target altitude in meters
    hover_duration = 15  # Example hover duration in seconds

    # Example usage of the functions
    if not check_gps():
        print("Unable to proceed with the flight due to GPS issues.")
    else:
        arm_and_takeoff(target_altitude)
        current_latitude, current_longitude, current_altitude = get_gps_coordinates()
        if current_latitude is not None:
            hover_and_return(current_altitude, hover_duration)
