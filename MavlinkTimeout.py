from pymavlink import mavutil
import time

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

timeout_sec = 30  # Timeout duration in seconds
start_time = time.time()

while True:
    # Attempt to receive an 'ATTITUDE' message with a timeout
    msg = the_connection.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
    
    if msg is not None:
        print("Received ATTITUDE message:")
        print(msg)
    else:
        print("Timeout reached waiting for ATTITUDE message.")

    # Check if timeout duration has elapsed
    if time.time() - start_time > timeout_sec:
        print(f"Overall timeout of {timeout_sec} seconds reached. Exiting.")
        break

    # Wait for a short interval before attempting to receive again
    time.sleep(1.0)  # Adjust this delay as needed
