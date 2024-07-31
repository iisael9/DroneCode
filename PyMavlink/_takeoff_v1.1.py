import pymavlink
from pymavlink import mavutil
import time


def send_command_with_ack(master, command, param1, param2, param3, param4, param5, param6, param7):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        0,  # confirmation
        param1, param2, param3, param4, param5, param6, param7)
    print("Command sent")

    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(f"Command acknowledged: {msg}")

    # Process the acknowledgment message (e.g., check for errors)
    if msg.result == mavutil.mavlink.MAV_RESULT.MAV_RESULT_ACCEPTED:
        print("Command accepted")
    else:
        print(f"Command rejected: {msg.result}")


# Create the connection
master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)
master.wait_heartbeat()
print("Heartbeat!")

# Safety checks


def is_armed():
    return master.motors_armed()


def get_altitude():
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    return msg.relative_alt / 1000.0


def check_battery(threshold=20):
    # Replace with actual battery level check
    battery_level = 50  # Example value
    return battery_level > threshold


# Arm the drone
if not is_armed():
    send_command_with_ack(
        master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0)

# Takeoff to 2 meters
if is_armed() and check_battery():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # confirmation
        0, 0, 0, 0, 2  # Minimum pitch, yaw, speed, altitude
    )
    print("Takeoff command sent")

    # Altitude monitoring
    target_altitude = 2.0  # Target altitude in meters
    altitude_tolerance = 0.2  # Allowable deviation from target altitude

    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_altitude = msg.relative_alt / 1000.0  # Convert to meters

        print(f"Current altitude: {current_altitude:.2f} meters")

        if current_altitude >= target_altitude - altitude_tolerance:
            print("Target altitude reached")
            break
