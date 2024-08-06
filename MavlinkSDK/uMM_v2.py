#!/usr/bin/env python3

import math
from pymavlink import mavutil

# Class for formatting the mission item 
class MissionItem:
    def __init__(self, i, current, x, y, z):
        self.seq = i
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT  # use Global Lat & Lon for mission pos data
        self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT       # move to waypoint
        self.current = current
        self.auto = 1
        self.param1 = 0.0
        self.param2 = 2.0
        self.param3 = 20.0
        self.param4 = math.nan
        self.param5 = x
        self.param6 = y
        self.param7 = z
        self.mission_type = mavutil.mavlink.MAV_MISSION_TYPE_MISSION

# Arm the drone
def arm(the_connection):
    print("Arming.....")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)
    ack(the_connection, "COMMAND_ACK")

# Takeoff the drone
def takeoff(the_connection):
    print("Takeoff Initiated...")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, math.nan, 0, 0, 10)
    ack(the_connection, "COMMAND_ACK")

# Upload the mission items
def upload_mission(the_connection, mission_items):
    n = len(mission_items)
    print("-- Sending message out")

    the_connection.mav.mission_count_send(the_connection.target_system, the_connection.target_component, n)
    ack(the_connection, "MISSION_REQUEST")

    for waypoint in mission_items:
        print("-- Creating a waypoint")
        the_connection.mav.mission_item_send(
            the_connection.target_system,
            the_connection.target_component,
            waypoint.seq,
            waypoint.frame,
            waypoint.command,
            waypoint.current,
            waypoint.auto,
            waypoint.param1,
            waypoint.param2,
            waypoint.param3,
            waypoint.param4,
            waypoint.param5,
            waypoint.param6,
            waypoint.param7,
            waypoint.mission_type)
        ack(the_connection, "MISSION_REQUEST")

    ack(the_connection, "MISSION_ACK")

# Tell drone to return to launch point
def set_return(the_connection):
    print("-- Set Return To Launch")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0,
        0, 0, 0, 0, 0, 0, 0)
    ack(the_connection, "COMMAND_ACK")

# Start Mission
def start_mission(the_connection):
    print("-- Mission start")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0,
        0, 0, 0, 0, 0, 0, 0)
    ack(the_connection, "COMMAND_ACK")

# Acknowledgement from the drone
def ack(the_connection, keyword):
    print("-- Waiting for ACK: " + keyword)
    msg = the_connection.recv_match(type=keyword, blocking=True)
    print("-- Received ACK: " + str(msg))

# Main Function
if __name__ == "__main__":
    print("-- Program Started")
    
    the_connection = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)

    print("-- checking Heartbeat...")
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

    mission_waypoints = [
        MissionItem(0, 0, 34.18383737493528, -117.32407480304997, 10),
        MissionItem(1, 0, 34.18383737493528, -117.32407480304997, 10),
        MissionItem(2, 0, 34.18383737493528, -117.32407480304997, 15)
    ]

    upload_mission(the_connection, mission_waypoints)
    arm(the_connection)
    takeoff(the_connection)
    start_mission(the_connection)

    for mission_item in mission_waypoints:
        print("-- Waiting for mission item reached: " + str(mission_item.seq))
        msg = the_connection.recv_match(type="MISSION_ITEM_REACHED", blocking=True)
        print("-- Mission item reached: " + str(msg))

    set_return(the_connection)
