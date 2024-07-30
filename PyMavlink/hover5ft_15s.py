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
        # Request GPS messages
        master.mav.message_factory.global_position_int_send(
            master.target_system, 0,  # Time since system boot (ignored)
            int(time.time() * 1e6),  # Time in microseconds
            0, 0, 0,  # Ignore: latitude, longitude, altitude relative to ground
            0  # Ignore: relative altitude from terrain
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
    Arms the drone and takes it off to the specified target altitude.

    Args:
        target_altitude (float): The desired altitude in meters.
    """
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print("Heartbeat received!")

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
        # Send takeoff command (adjust based on your autopilot)
        master.mav.set_position_target_local_ned_send(
            0,  # Time boot ms (usually set to 0)
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Frame of reference (adjust if needed)
            0b111111000000,  # Type mask (only positions are set, others are ignored)
            0, 0, 0,  # Position coordinates (x, y, z local coordinates)
            0, 0, 0,  # Velocity (vx, vy, vz)
            0, 0, target_altitude,  # Acceleration (ax, ay, az)
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
    hover_altitude = current_altitude + 1.5  # Set hover altitude 5ft above current

    # Hover for the specified duration
    print(f"Hovering at {hover_altitude} meters for {hover_duration} seconds...")
    start_time = time.time()
    while time.time() - start_time < hover_duration:
        # Send hovering command (adjust based on your autopilot)
        master.mav.set_position_target_local_ned_send(
            0,  # Time boot ms (usually set to 0)
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Frame of reference (adjust if needed)
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
    arm_and_takeoff(target_altitude)
    current_latitude, current_longitude, current_altitude = get_gps_coordinates()
    hover_and_return(current_altitude, hover_duration)
