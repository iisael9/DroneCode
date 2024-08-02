#!/usr/bin/env python3

from pymavlink import mavutil

def arm_drone(master):
    # Create the command to arm the drone
    master.arducopter_arm()
    
    # Wait for the drone to arm
    master.motors_armed_wait()
    print("Drone is armed")

def disarm_drone(master):
    # Create the command to disarm the drone
    master.arducopter_disarm()
    
    # Wait for the drone to disarm
    master.motors_disarmed_wait()
    print("Drone is disarmed")

def main():
    # Connect to the drone
    master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)
    master.wait_heartbeat()
    print("Connected to drone")

    # Arm the drone
    arm_drone(master)
    
    # Wait for user input to disarm
    input("Press Enter to disarm the drone...")

    # Disarm the drone
    disarm_drone(master)

if __name__ == "__main__":
    main()
