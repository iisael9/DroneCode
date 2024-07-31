from pymavlink import mavutil
import time

# Connect to the drone
master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)

# Wait for a heartbeat from the drone
master.wait_heartbeat()

# Define takeoff altitude and hold time
target_altitude = 2  # meters above current position
hold_time = 15  # seconds


def arm_and_takeoff(target_alt):
    # Ensure the vehicle is armable
    while not master.motors_armed():
        print("Arming motors")
        master.arducopter_arm()
        master.motors_armed_wait()
        print("Motors armed")

    print("Taking off")

    # Command the drone to take off to the target altitude
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, target_alt
    )

    # Wait until the drone reaches the target altitude
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_altitude = msg.relative_alt / 1000.0  # in meters
        if current_altitude >= target_alt * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def hold_position(hold_time):
    print(f"Holding position for {hold_time} seconds")
    time.sleep(hold_time)


def land():
    print("Landing")

    # Command the drone to land
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

    # Wait until the drone lands
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_altitude = msg.relative_alt / 1000.0  # in meters
        if current_altitude <= 0.1:
            print("Landed")
            break
        time.sleep(1)


if __name__ == "__main__":
    # Start the sequence
    arm_and_takeoff(target_altitude)
    hold_position(hold_time)
    land()
