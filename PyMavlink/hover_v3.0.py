import time
from pymavlink import mavutil


class DroneController:
    def __init__(self, connection_string, baudrate):
        self.master = mavutil.mavlink_connection(connection_string, baudrate)

    def arm_and_takeoff(self, target_altitude):
        # ... (similar to previous implementation)
        
    def hover_and_return(self, hover_altitude, hover_duration):
        # ... (similar to previous implementation)
        
    def get_gps_coordinates(self):
        # ... (similar to previous implementation)

    def control_loop(self):
        # ... (state machine logic, if desired)

        # Example usage:
drone = DroneController("/dev/ttyAMA0", 57600)
drone.arm_and_takeoff(5)
current_lat, current_lon, current_alt = drone.get_gps_coordinates()
drone.hover_and_return(current_alt + 1.5, 15)
