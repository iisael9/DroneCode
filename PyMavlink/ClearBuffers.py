"""
Example of how to Clear Buffers for an Autopilot with pymavlink
"""
# Import mavutil
from pymavlink import mavutil

def clear_buffers(connection):
    """Clear all messages in the buffer."""
    while connection.recv_match(timeout=0.5) is not None:
        pass

# Create the connection
master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)

# Clear buffers initially
clear_buffers(master)

print("Buffers cleared!")
