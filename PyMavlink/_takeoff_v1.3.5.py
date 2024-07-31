from pymavlink import mavutil
import time
from geopy.distance import geodesic

# Connect to the drone
master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)

# Wait for a heartbeat from the drone
master.wait_heartbeat()

# Define takeoff altitude, hold time, and other parameters
target_altitude = 2  # meters above current position
hold_time = 15  # seconds
battery_threshold = 50  # percentage
low_battery_warning_threshold = 60  # percentage
max_altitude = 10  # maximum safe altitude in meters
deviation_threshold = 0.2  # maximum allowed deviation in meters


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
    print("Warning: Deviation or low battery detected. Returning to launch.")
    set_mode('RTL')


def check_gps_lock():
    while True:
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg.fix_type >= 2:  # 2D fix or better
            print("GPS lock acquired")
            break
        print("Waiting for GPS lock...")
        time.sleep(1)


def pre_flight_checks():
    check_gps_lock()

    if master.motors_armed():
        print("Motors already armed, disarming")
        master.arducopter_disarm()
        master.motors_disarmed_wait()

    print("Pre-flight checks passed")


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


def get_current_position():
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        return (lat, lon)
    return None


def hold_position(hold_time):
    print(f"Holding position for {hold_time} seconds")
    start_time = time.time()
    initial_position = get_current_position()

    while time.time() - start_time < hold_time:
        battery_level = check_battery()
        current_position = get_current_position()

        if battery_level < battery_threshold:
            return_to_launch()
            return
        elif battery_level < low_battery_warning_threshold:
            print(f"Warning: Low battery ({battery_level}%)")

        if current_position and initial_position:
            deviation = geodesic(initial_position, current_position).meters
            if deviation > deviation_threshold:
                print(f"Warning: Deviation detected ({deviation} meters)")
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
        pre_flight_checks()
        arm_and_takeoff(target_altitude)
        hold_position(hold_time)
        land()
    except KeyboardInterrupt:
        return_to_launch()
