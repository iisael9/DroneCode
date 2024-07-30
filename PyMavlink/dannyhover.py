import time
from pymavlink import mavutil

# Connect to the drone
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)

def wait_for_heartbeat():
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print("Heartbeat received!")

def get_current_gps():
    """
    Fetches the drone's current GPS coordinates.
    """
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt = msg.relative_alt / 1000.0  # Convert from mm to meters
    return lat, lon, alt

def print_status(action):
    lat, lon, alt = get_current_gps()
    print(f"{action} | Current GPS coordinates: lat={lat}, lon={lon}, alt={alt} meters")

def set_flight_mode(mode):
    """
    Sets the flight mode of the drone.
    """
    mode_map = {
        'STABILIZE': 0,
        'ACRO': 1,
        'ALT_HOLD': 2,
        'GUIDED': 4,
        'LOITER': 5,
        'RTL': 6
    }
    mode_id = mode_map.get(mode, None)
    if mode_id is not None:
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print(f"Setting flight mode to {mode}...")
    else:
        print(f"Unknown flight mode: {mode}")

def arm():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    master.motors_armed_wait()
    print_status("Armed")

def disarm():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0)
    master.motors_disarmed_wait()
    print_status("Disarmed")

def takeoff(target_altitude):
    """
    Initiates takeoff to the specified altitude.
    """
    print(f"Taking off to {target_altitude} meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_altitude)
    print("Takeoff command sent")

def land():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0)
    print_status("Landing")

def go_to_waypoint(lat, lon, alt):
    master.mav.set_position_target_global_int_send(
        0,  # time_boot_ms (not used)
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
        0b0000111111111000,  # type_mask (only positions enabled)
        int(lat * 1e7),  # lat_int - X Position in WGS84 frame in 1e7 * meters
        int(lon * 1e7),  # lon_int - Y Position in WGS84 frame in 1e7 * meters
        alt,  # alt - Altitude in meters
        0, 0, 0,  # X, Y, Z velocity in m/s (not used)
        0, 0, 0,  # X, Y, Z acceleration (not used)
        0, 0)  # Yaw, yaw rate (not used)
    print(f"Going to waypoint: lat={lat}, lon={lon}, alt={alt}")

def hover_and_return(current_lat, current_lon, current_alt, hover_duration):
    """
    Holds the drone at its current altitude for a specified duration and then returns to launch.
    """
    hover_altitude = current_alt + 1.5  # Set hover altitude 1.5 meters above current

    # Hover for the specified duration
    print(f"Hovering at {current_lat}, {current_lon}, {hover_altitude} meters for {hover_duration} seconds...")
    start_time = time.time()
    while time.time() - start_time < hover_duration:
        master.mav.set_position_target_global_int_send(
            0,  # time_boot_ms (not used)
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            0b0000111111111000,  # type_mask (only positions enabled)
            int(current_lat * 1e7),  # lat_int - X Position in WGS84 frame in 1e7 * meters
            int(current_lon * 1e7),  # lon_int - Y Position in WGS84 frame in 1e7 * meters
            hover_altitude,  # alt - Altitude in meters
            0, 0, 0,  # X, Y, Z velocity in m/s (not used)
            0, 0, 0,  # X, Y, Z acceleration (not used)
            0, 0)  # Yaw, yaw rate (not used)
        time.sleep(1)  # Adjust the sleep time as needed to prevent flooding the channel

    # Return to launch (RTL) command
    print("Returning to launch...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0,
        0, 0, 0, 0, 0, 0, 0)

if __name__ == "__main__":
    wait_for_heartbeat()

    # Print current GPS coordinates
    print("Fetching current GPS coordinates...")
    current_lat, current_lon, current_alt = get_current_gps()
    print(f"Current GPS coordinates: lat={current_lat}, lon={current_lon}, alt={current_alt} meters")

    # Get desired GPS coordinates from the user
    target_lat_input = input("Enter desired latitude (or 'same' to keep current): ")
    target_lon_input = input("Enter desired longitude (or 'same' to keep current): ")
    target_alt_input = input("Enter desired altitude (meters) (or 'same' to keep current): ")
    hover_duration = float(input("Enter hover duration (seconds): "))

    # Handle 'same' input
    target_lat = current_lat if target_lat_input.lower() == 'same' else float(target_lat_input)
    target_lon = current_lon if target_lon_input.lower() == 'same' else float(target_lon_input)
    target_alt = current_alt if target_alt_input.lower() == 'same' else float(target_alt_input)

    # Set flight mode to GUIDED
    set_flight_mode("GUIDED")

    # Arm the drone
    print("Arming drone...")
    arm()

    # Takeoff to specified altitude
    print("Taking off...")
    takeoff(target_alt)

    # Wait for takeoff and hover
    time.sleep(15)
    print_status("Hovering")

    # Move to desired GPS coordinates and hover
    go_to_waypoint(target_lat, target_lon, target_alt)
    time.sleep(10)  # Adjust this based on how long it takes to reach the waypoint
    hover_and_return(target_lat, target_lon, target_alt, hover_duration)

    # Wait for drone to return to launch
    time.sleep(30)  # Adjust based on the time required to return to launch

    # Disarm the drone
    print("Disarming drone...")
    disarm()
