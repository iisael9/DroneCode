from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
import time

# Connect to the drone
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)

# Wait for the heartbeat from the drone
master.wait_heartbeat()

# Function to create a mission item
def create_mission_item(seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z):
    return mavlink2.MAVLink_mission_item_message(
        master.target_system, master.target_component,
        seq, frame, command, current, autocontinue,
        param1, param2, param3, param4, x, y, z)

# Function to upload the mission
def upload_mission(mission_items):
    mission_count = len(mission_items)
    master.mav.mission_count_send(master.target_system, master.target_component, mission_count)
    
    for i, item in enumerate(mission_items):
        master.mav.send(item)
        master.recv_match(type=['MISSION_REQUEST'], blocking=True)
    
    print("Mission uploaded successfully")

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

# Define the mission items
takeoff_altitude = 2
hover_duration = 15

# Arm the drone
print("Arming drone...")
arm()

# Wait a bit for the drone to be armed
time.sleep(5)

# Get current GPS coordinates
print("Getting current GPS coordinates...")
current_lat, current_lon, current_alt = get_current_gps()
print(f"Current coordinates: lat={current_lat}, lon={current_lon}, alt={current_alt} meters")

# Define the mission
mission_items = [
    # Takeoff to 2 meters
    create_mission_item(0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 1, 1, 0, 0, 0, 0, current_lat, current_lon, takeoff_altitude),
    
    # Hover (using a delay command as a placeholder for hover)
    create_mission_item(1, mavutil.mavlink.MAV_FRAME_MISSION, mavutil.mavlink.MAV_CMD_NAV_DELAY, 0, 1, hover_duration, 0, 0, 0, 0, 0, 0),
    
    # Land
    create_mission_item(2, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 1, 0, 0, 0, 0, current_lat, current_lon, 0)
]

# Upload the mission
print("Uploading mission...")
upload_mission(mission_items)

# Start the mission
print("Starting mission...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_MISSION_START,
    0, 0, 0, 0, 0, 0, 0, 0)

# Monitor the mission
while True:
    msg = master.recv_match(type=['MISSION_ITEM_REACHED', 'MISSION_CURRENT', 'STATUSTEXT'], blocking=True)
    if msg.get_type() == 'MISSION_ITEM_REACHED':
        print(f"Reached mission item {msg.seq}")
    elif msg.get_type() == 'MISSION_CURRENT':
        print(f"Current mission item {msg.seq}")
    elif msg.get_type() == 'STATUSTEXT':
        print(f"Status: {msg.text}")
    if msg.get_type() == 'MISSION_CURRENT' and msg.seq == len(mission_items):
        print("Mission completed")
        break

# Disarm the drone
print("Disarming drone...")
disarm()
