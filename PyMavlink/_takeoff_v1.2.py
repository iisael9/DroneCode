import time
import math
from pymavlink import mavutil

def takeoff_and_hover(master, target_altitude, hover_time):
    # Arm the drone
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,  # Arm
        0,
        0,
        0,
        0,
        0,
        0
    )

    # Wait for arming
    while not master.motors_armed:
        time.sleep(1)

    # Takeoff
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        target_altitude
    )

    # Wait for takeoff
    while master.location.alt < target_altitude * 0.95:
        time.sleep(1)

    # Hover for specified time
    time.sleep(hover_time)

    # Land
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
    )

    # Wait for landing
    while master.system_status.state != mavutil.mavlink.MAV_STATE_LANDED:
        time.sleep(1)

if __name__ == "__main__":
    master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)

    # Get current altitude
    current_altitude = master.location.alt

    # Set target altitude and hover time
    target_altitude = current_altitude + 2
    hover_time = 15

    takeoff_and_hover(master, target_altitude, hover_time)

    master.close()
