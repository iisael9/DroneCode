from pymavlink import mavutil
import time

# Establish a connection to the drone
master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)
print("Connection")

# Wait for the heartbeat message to find the system ID
master.wait_heartbeat()
print("Heartbeat")

# Get the current location from the GPS
msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
current_lat = msg.lat / 1e7
current_lon = msg.lon / 1e7
print(f"Current Location: Latitude {current_lat}, Longitude {current_lon}")

# Define the takeoff command (hover at 50 feet)
takeoff = mavutil.mavlink.MAVLink_mission_item_int_message(
    master.target_system,
    master.target_component,
    0,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 1, 0, 0, 0, 0, int(current_lat * 1e7), int(current_lon * 1e7), 50 * 0.3048
)

# Define the hover command (hover for 10 seconds)
hover_here = mavutil.mavlink.MAVLink_mission_item_int_message(
    master.target_system,
    master.target_component,
    1,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
    0, 1, 10, 0, 0, 0, int(34.18819833931459 * 1e7), int(-117.31836431530598 * 1e7), 50 * 0.3048
)

# Define the first waypoint command
waypoint_1 = mavutil.mavlink.MAVLink_mission_item_int_message(
    master.target_system,
    master.target_component,
    2,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    0, 1, 0, 0, 0, 0, int(34.18869126166431 * 1e7), int(-117.31838442523032 * 1e7), 50 * 0.3048
)

# Define the second waypoint command
waypoint_2 = mavutil.mavlink.MAVLink_mission_item_int_message(
    master.target_system,
    master.target_component,
    3,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    0, 1, 0, 0, 0, 0, int(34.188657875210346 * 1e7), int(-117.31881739091219 * 1e7), 50 * 0.3048
)

# Define the third waypoint command
waypoint_3 = mavutil.mavlink.MAVLink_mission_item_int_message(
    master.target_system,
    master.target_component,
    4,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    0, 1, 0, 0, 0, 0, int(34.18818439316763 * 1e7), int(-117.31865961528234 * 1e7), 50 * 0.3048
)

# Define a return-to-launch command
rtl = mavutil.mavlink.MAVLink_mission_item_int_message(
    master.target_system,
    master.target_component,
    5,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
    0, 1, 0, 0, 0, 0, 0, 0, 0
)

# Create the mission list
mission_list = [takeoff, hover_here, waypoint_1, waypoint_2, waypoint_3, rtl]

# Clear existing mission
master.mav.mission_clear_all_send(master.target_system, master.target_component)

# Send the mission items to the drone
for i, mission_item in enumerate(mission_list):
    master.mav.send(mission_item)
    # Wait for the mission item to be requested
    ack = master.recv_match(type='MISSION_REQUEST', blocking=True)
    print(f"Mission item requested: {ack.seq}")

# Send the mission count
master.mav.mission_count_send(master.target_system, master.target_component, len(mission_list))

# Arm the drone
master.arducopter_arm()

# Set mode to auto
master.set_mode_auto()

print("Mission uploaded and started")

# Monitor mission progress (optional)
while True:
    msg = master.recv_match(type=['MISSION_CURRENT', 'STATUSTEXT'], blocking=True)
    print(msg)
    if msg.get_type() == 'STATUSTEXT':
        print(f"Drone message: {msg.text}")

    time.sleep(1)  # Delay to reduce console spam
