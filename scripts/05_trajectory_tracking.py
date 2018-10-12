"""
Circular Trajectory Tracking

The system waits for a valid mission to be uploaded. It needs only one (the first) waypoint.

The drone takes off and then performs a circular trajectory around the way point. It
keeps constant forward velocity, while the lateral velocity is calculated in order to
keep the drone at the same distance from the waypoint. The heading is calculated for
maintaining a 90 degrees angle respect to the bearing

This is a simple script, but it shows how to setup a simple trajectory tracking algorithm
that might be used for later applications

"""


import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default = '')
args = parser.parse_args()


connection_string = args.connect

#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------
#-- Define arm and takeoff
def arm_and_takeoff(altitude):

   while not vehicle.is_armable:
      print("waiting to be armable")
      time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      time.sleep(1)

 #-- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    
    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, 
    bit 4: vx, bit 5: vy, bit 6: vz, 
    bit 7: ax, bit 8: ay, bit 9:
    
    
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
    

def clear_mission(vehicle):
    """
    Clear the current mission.
    """
    cmds = vehicle.commands
    vehicle.commands.clear()
    vehicle.flush()

    # After clearing the mission you MUST re-download the mission from the vehicle
    # before vehicle.commands can be used again
    # (see https://github.com/dronekit/dronekit-python/issues/230)
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

def download_mission(vehicle):
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.
    

def get_current_mission(vehicle):
    """
    Downloads the mission and returns the wp list and number of WP 
    
    Input: 
        vehicle
        
    Return:
        n_wp, wpList
    """

    print "Downloading mission"
    download_mission(vehicle)
    missionList = []
    n_WP        = 0
    for wp in vehicle.commands:
        missionList.append(wp)
        n_WP += 1 
        
    return n_WP, missionList
    

def ChangeMode(vehicle, mode):
    while vehicle.mode != VehicleMode(mode):
            vehicle.mode = VehicleMode(mode)
            time.sleep(0.5)
    return True


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint(vehicle):
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

def bearing_to_current_waypoint(vehicle):
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    bearing = get_bearing(vehicle.location.global_relative_frame, targetWaypointLocation)
    return bearing

def get_bearing(my_location, tgt_location):
    """
    Aproximation of the bearing for medium latitudes and sort distances
    """
    dlat = tgt_location.lat - my_location.lat
    dlong = tgt_location.lon - my_location.lon
    
    return math.atan2(dlong,dlat)


def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,       # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0,          #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def saturate(value, minimum, maximum):
    if value > maximum: value = maximum
    if value < minimum: value = minimum
    return value

def add_angles(ang1, ang2):
    ang = ang1 + ang2
    if ang > 2.0*math.pi:
        ang -= 2.0*math.pi
    
    elif ang < -0.0:
        ang += 2.0*math.pi
    return ang

#--------------------------------------------------
#-------------- INITIALIZE  
#--------------------------------------------------      
#-- Setup the commanded flying speed
gnd_speed = 8 # [m/s]
radius    = 80
max_lat_speed = 4
k_err_vel   = 0.2
n_turns     = 3
direction   = 1 # 1 for cw, -1 ccw

mode      = 'GROUND'

#--------------------------------------------------
#-------------- CONNECTION  
#--------------------------------------------------    
#-- Connect to the vehicle
print('Connecting...')
vehicle = connect(connection_string)
#vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)


#--------------------------------------------------
#-------------- MAIN FUNCTION  
#--------------------------------------------------    
while True:
    
    if mode == 'GROUND':
        #--- Wait until a valid mission has been uploaded
        n_WP, missionList = get_current_mission(vehicle)
        time.sleep(2)
        if n_WP > 0:
            print ("A valid mission has been uploaded: takeoff!")
            mode = 'TAKEOFF'
            
    elif mode == 'TAKEOFF':
        time.sleep(1)
        #-- Takeoff
        arm_and_takeoff(5)
        
      
        #-- Change mode, set the ground speed
        vehicle.groundspeed = gnd_speed
        mode = 'MISSION'
        vehicle.commands.next = 1
        
        vehicle.flush()

        #-- Calculate the time for n_turns
        time_flight = 2.0*math.pi*radius/gnd_speed*n_turns
        time0 = time.time()

        print ("Swiitch mode to MISSION")
        
    elif mode == 'MISSION':
        #-- We command the velocity in order to maintain the vehicle on track
        #- vx = constant
        #- vy = proportional to off track error
        #- heading = along the path tangent
        
        my_location = vehicle.location.global_relative_frame
        bearing     = bearing_to_current_waypoint(vehicle)
        dist_2_wp   = distance_to_current_waypoint(vehicle)
        
        try:
            print "bearing  %.0f  dist = %.0f"%(bearing*180.0/3.14, dist_2_wp)
            heading = add_angles(bearing,-direction*0.5*math.pi)
            #print heading*180.0/3.14
            condition_yaw(heading*180/3.14)
            
            v_x     = gnd_speed
            v_y     = -direction*k_err_vel*(radius - dist_2_wp)
            v_y     = saturate(v_y, -max_lat_speed, max_lat_speed)
            print "v_x = %.1f  v_y = %.1f"%(v_x, v_y)
            set_velocity_body(vehicle, v_x, v_y, 0.0)
            
        except Exception as e:
            print e


        if time.time() > time0 + time_flight: 
            ChangeMode(vehicle, 'RTL')    
            clear_mission(vehicle)        
            mode = 'BACK'
            print (">> time to head Home: switch to BACK")
            
    elif mode == "BACK":
        if vehicle.location.global_relative_frame.alt < 1:
            print (">> Switch to GROUND mode, waiting for new missions")
            mode = 'GROUND'
    
    
    
    
    time.sleep(0.5)


    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
      
