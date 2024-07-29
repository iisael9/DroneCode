from pymavlink import mavutil

# Initialize connection to the drone
def connect_drone():
    master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)
    master.wait_heartbeat()
    print("Heartbeat received!")
    return master

# Arm the drone
def arm_drone(master):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print('Drone armed!')

# Disarm the drone
def disarm_drone(master):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    master.motors_disarmed_wait()
    print('Drone disarmed!')

# Command the drone to adjust yaw (horizontal rotation)
def set_yaw(master, yaw_angle):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        yaw_angle, 0, 1, 0, 0, 0, 0
    )
    print(f'Yaw set to {yaw_angle} degrees
