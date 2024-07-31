from pymavlink import mavutil
import time

# Connect to the drone
master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)

# Wait for a heartbeat from the drone
master.wait_heartbeat()

# Define takeoff altitude and hold time
target_altitude = 2  # meters above current position
hold_time = 15  # seconds
battery_threshold = 50  # percentage


def set_mode(mode):
    if mode not in master.mode_mapping():
        print("Unknown mode: {}".format(mode))
        print("Available modes:", list(master.mode_mapping().keys()))
        return

    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    while True:
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()
        if ack_msg['command'] == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            print(mavutil.mavlink.enums['MAV_RESULT']
                  [ack_msg['result']].description)
            break


def check_battery():
    msg = master.recv_match(type='BATTERY_STATUS', blocking=True)
    if msg is not None:
        battery_remaining = msg.battery_remaining  # percentage
        return battery_remaining
    return 100  # assume full battery if no data is received


def return_to_launch():
    print("Returning to launch")
    set_mode('RTL')


def arm_and_takeoff(target_alt):
    set_mode('GUIDED')
    while not master.motors_armed():
        print("Arming motors")
        master.arducopter_arm()
        master.motors_armed_wait()
        print("Motors armed")

    print("Taking off")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, target_alt
    )

    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_altitude = msg.relative_alt / 1000.0
        if current_altitude >= target_alt * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def hold_position(hold_time):
    print(f"Holding position for {hold_time} seconds")
    start_time = time.time()
    while time.time() - start_time < hold_time:
        battery_level = check_battery()
        if battery_level < battery_threshold:
            return_to_launch()
            return
        time.sleep(1)


def land():
    print("Landing")
    set_mode('LAND')
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_altitude = msg.relative_alt / 1000.0
        if current_altitude <= 0.1:
            print("Landed")
            break
        time.sleep(1)


if __name__ == "__main__":
    try:
        arm_and_takeoff(target_altitude)
        hold_position(hold_time)
        land()
    except KeyboardInterrupt:
        return_to_launch()
