import time
from pymavlink import mavutil

# Connection parameters (replace with your actual values)
# Adjust these based on your autopilot and communication method (e.g., serial, UDP)
master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)


def get_gps_coordinates():
  """
  Fetches the drone's current GPS coordinates.

  Returns:
      (float, float, float): The latitude, longitude, and altitude in meters.
  """
  while True:
    # Request GPS messages
    master.mav.message_factory.global_position_int_send(
        master.target_system, 0,  # Time since system boot (ignored)
        int(time.time() * 1e6),  # Time in microseconds
        0, 0, 0,  # Ignore: latitude, longitude, altitude relative to ground
        0  # Ignore: relative altitude from terrain
    )

    # Wait for a message with GPS data
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg is not None:
      latitude = msg.lat / 10**7  # Convert from degrees*10^7 to degrees
      longitude = msg.lon / 10**7  # Convert from degrees*10^7 to degrees
      altitude = msg.alt / 1000  # Convert from millimeters to meters
      return latitude, longitude, altitude


def arm_and_takeoff(target_altitude):
  """
  Arms the drone and takes it off to the specified target altitude.

  Args:
      target_altitude (float): The desired altitude in meters.
  """
  print("Waiting for heartbeat...")
  master.wait_heartbeat()
  print("Heartbeat received!")

  # Arm the drone
  print("Arming...")
  master.mav.command_long_send(
      master.target_system,
      master.target_component,
      mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
      0,
      1, 1, 0, 0, 0, 0, 0)
  master.motors_armed_wait()
  print("Armed!")

  # Takeoff to target altitude
  print(f"Taking off to {target_altitude} meters...")
  while master.mav.altitude < target_altitude:
    time.sleep(0.5)
    # Send takeoff command (adjust based on your autopilot)
    master.mav.set_position_target_local_ned_send(
        master.target_system,
        master.target_component,
        MAV_FRAME_LOCAL_NED,  # Frame of reference (adjust if needed)
        0, 0, 0,  # Ignore: x, y, z local coordinates
        0, 0, 0,  # Ignore: vx, vy, vz velocity in m/s
        target_altitude,  # Target altitude
        0, 0, 0  # Ignore: x, y, z local position offsets in m
    )


def hover_and_return(current_altitude, hover_duration):
  """
  Holds the drone at its current altitude for a specified duration and then returns to launch.

  Args:
      current_altitude (float): The drone's current altitude in meters.
      hover_duration (float): The duration to hover in seconds.
  """
  hover_altitude = current_altitude + 1.5  # Set hover altitude 5ft above current

  # Hover for the specified duration
  print(f"Hovering at {hover_altitude} meters for {hover_duration} seconds...")
  start_time = time.time()
  while time.time() - start_time < hover_duration:
    # Send hovering command (adjust based on your autopilot)
    master.mav.set_position_target_local_ned_send(
        master.target_system,
        master.target_component,
        MAV_FRAME_LOCAL_NED,  # Frame of reference (adjust if needed)
        0, 0, 0,  # Ignore: x, y, z local coordinates
        0, 0, 0,  # Ignore: vx, vy, vz velocity in m/s
        hover_altitude,  # Target altitude
        0,
