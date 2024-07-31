import pymavlink
from pymavlink import mavutil

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

# Arm the drone
send_command_with_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0)

# Takeoff to 2 meters
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,  # confirmation
    0, 0, 0, 0, 2  # Minimum pitch, yaw, speed, altitude
)
print("Takeoff command sent")

# Wait for takeoff to complete (you might want to implement a more robust check)
# ...
