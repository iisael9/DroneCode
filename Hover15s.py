from pymavlink import mavutil
import time

# Connect to the drone
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)

# Wait for the heartbeat from the drone
master.wait_heartbeat()

# Function to arm the drone
def arm():
    master.arducopter_arm()
    master.motors_armed_wait()
    print_status("Armed")

# Function to disarm the drone
def disarm():
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print_status("Disarmed")

# Function to takeoff
def takeoff(target_altitude):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_altitude)
    print_status(f"Taking off to {target_altitude} meters")

# Function to land
def land():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0)
    print_status("Landing")

# Function to send the drone to a specific GPS coordinate
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
    print_status(f"Going to waypoint: lat={lat}, lon={lon}, alt={alt}")

# Function to get current GPS coordinates
def get_current_gps():
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt = msg.relative_alt / 1000.0  # Convert from mm to meters
    return lat, lon, alt

# Function to print the current status and GPS coordinates
def print_status(action):
    lat, lon, alt = get_current_gps()
    print(f"{action} | Current GPS coordinates: lat={lat}, lon={lon}, alt={alt} meters")

# Arm the drone
print("Arming drone...")
arm()

# Wait a bit for the drone to be armed
time.sleep(5)

# Get current GPS coordinates
print("Getting current GPS coordinates...")
current_lat, current_lon, current_alt = get_current_gps()
print(f"Current coordinates: lat={current_lat}, lon={current_lon}, alt={current_alt} meters")

# Takeoff to 2 meters
print("Taking off...")
takeoff(2)

# Wait a bit for the drone to take off and hover
time.sleep(15)
print_status("Hovering")

# Land the drone
print("Landing...")
land()

# Wait a bit for the drone to land
time.sleep(10)
print_status("Landed")

# Disarm the drone
print("Disarming drone...")
disarm()
