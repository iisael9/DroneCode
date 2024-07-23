from pymavlink import mavutil
import time

 # Random change


# Connect to the drone
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)

# Wait for the heartbeat from the drone
master.wait_heartbeat()

# Function to arm the drone
def arm():
    master.arducopter_arm()
    master.motors_armed_wait()

# Function to disarm the drone
def disarm():
    master.arducopter_disarm()
    master.motors_disarmed_wait()

# Function to takeoff
def takeoff(target_altitude):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_altitude)

# Function to land
def land():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0)

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

# Function to get current GPS coordinates
def get_current_gps():
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    return lat, lon

# Arm the drone
print("Arming drone...")
arm()

# Wait a bit for the drone to be armed
time.sleep(5)

# Get current GPS coordinates
print("Getting current GPS coordinates...")
current_lat, current_lon = get_current_gps()
print(f"Current coordinates: {current_lat}, {current_lon}")

# Takeoff to 15 meters
print("Taking off...")
takeoff(15)

# Wait a bit for the drone to take off
time.sleep(10)

# Go to first waypoint
print("Going to first waypoint...")
go_to_waypoint(34.188691248645355, -117.31825188557909, 15)
time.sleep(20)  # Adjust sleep time as needed

# Go to second waypoint
print("Going to second waypoint...")
go_to_waypoint(34.18901271761967, -117.31872161827444, 15)
time.sleep(20)  # Adjust sleep time as needed

# Go to third waypoint
print("Going to third waypoint...")
go_to_waypoint(34.1886185685335, -117.31915755725076, 15)
time.sleep(20)  # Adjust sleep time as needed

# Return to home (initial takeoff point)
print("Returning to home...")
go_to_waypoint(current_lat, current_lon, 15)
time.sleep(20)  # Adjust sleep time as needed

# Land the drone
print("Landing...")
land()

# Wait a bit for the drone to land
time.sleep(10)

# Disarm the drone
print("Disarming drone...")
disarm()
 
 # Random change