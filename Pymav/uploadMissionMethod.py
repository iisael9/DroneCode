
import math 
from pymavlink import mavutil

#class for formating the mission item 
class mission_item:
    def __init__(self, i, current, x,y,z):
        self.seq = i
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBALRELATIVE_ALT  # use Global Lat & Lon for mission pos data
        self.command = mavutil.mav_type.MAV_CMD_NAV_WAYPOINT       #move to waypoint
        self.current = current
        self.auto = 1
        self.param1 = 0.0
        self.param2 = 2.0 
        self.param3 = 20.0
        self.param4 = math.nan
        self.param5 = x
        self.param6 = y
        self.param7 = z
        self.mission_type

# Arm the drone
def arm (the_connection):
    print("Arming.....")

    the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

    ack(the_connection, "COMMAND_ACK")

#Takeoff the drone
def takeoff(the_connection):
    print("Takeoff Initiated...")

    the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_TAKEOFF,
    0,
    0, 0, 0, math.nan, 0, 0, 10)

    ack(the_connection, "COMMAND_ACK")


# Upload the mission item 
def upload_mission(the_connection, mission_items):
    n = len(mission_items)
    print("-- Sending message out")

    the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component,
    n, 0)

    ack(the_connection, "COMMAND_REQEST")

    for waypoint in mission_items:
        print("-- Creating a waypoint")

        the_connection.mav.command_long_send(
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
        
    if waypoint != mission_items[n-1]:
        ack(the_connection, "MISSION_REQUEST")

    ack(the_connection, "COMMAND_ACK")
    
        

# Tell drone to return to launch point
def set_return(the_connection):
    print("-- Set Return To Launch")

    the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component, 
    mavutil.mavlink.MAV_CMD_RETURN_TO_LAUNCH, 
    0,
    0, 0, 0, 0, 0, 0, 0)
    
    ack(the_connection, "COMMAND_ACK")


#Start Mission
def start_mission(the_connection):
    print("-- Mission start")

    the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component, 
    mavutil.mavlink.MAV_CMD_MISSION_START, 
    0,
    0, 0, 0, 0, 0, 0, 0)
    
    ack(the_connection, "COMMAND_ACK")


#Acknowledgement from the drone
def ack(the_connection, keyword):
    print("-- Message Read " + str(the_connection.recv_match(type=keyword, blocking=True)))



#Main Function
if __name__ == "__main__":
    print("-- Program Started")
    
    the_connection = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)

    while(the_connection.target_system == 0):
        print("-- checking Heartbeat...")
        the_connection.heartbeat()
        print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

    mission_waypoints = []

    mission_waypoints.append(mission_item(0, 0, 34.18383737493528, -117.32407480304997, 10))
    mission_waypoints.append(mission_item(1, 0, 34.18383737493528, -117.32407480304997, 10))
    mission_waypoints.append(mission_item(2, 0, 34.18383737493528, -117.32407480304997, 10))

    upload_mission(the_connection, mission_waypoints)

    arm(the_connection)

    takeoff(the_connection)

    start_mission(the_connection)

    for mission_item in mission_waypoints:
        print("-- Message Read " + str(the_connection.recv_match(type="MISSION_ITEM_REACHED" , condition= "MISSION_ITEM_REACHED.seq == {0}.format")))

    set_return(the_connection)
















