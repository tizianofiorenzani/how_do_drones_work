from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__)))

import time
import math
import argparse

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

#--- Import our Aruco Library
from opencv.lib_aruco_pose import *

#--- Parse the input arguments
parser  = argparse.ArgumentParser()
parser.add_argument('--connect')
args    = parser.parse_args()


#------------------ FUNCTIONS
def get_location_m(original_location, d_north_m, d_east_m):
    #--- Get the new position (lat, lon) given the current and the offset in [m]
    earth_radius    = 6378137 #- [m]
    d_lat           = d_north_m/earth_radius
    d_lon           = d_east_m/(earth_radius*math.cos(math.radians(original_location.lat)))
    
    new_lat         = original_location.lat + math.degrees(d_lat)
    new_lon         = original_location.lon + math.degrees(d_lon)
    
    return(new_lat, new_lon)
    
#-- Camera to UAV frame: camera has X right and Y down. UAV has X forward, Y right
def camera_2_uav(x_cam, y_cam):
    x_uav =-y_cam
    y_uav = x_cam
    
#-- UAV to NE reference frame
def uav_2_ne(x_uav, y_uav, yaw_rad):
    #-- define cos and sin
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    
    north = x_uav*c - y_uav*s
    east  = x_uav*s + y_uav*c 
    
    return(north, east)
    
#-- Estimate the LOS angles to the marker
def marker_position_to_angle(x, y, z):
    angle_x = math.atan2(x, z)
    angle_y = math.atan2(y, z)
    
    return(angle_x, angle_y)
    
    
#-------------------------------------------------
#------------  MAIN LOOP
#-------------------------------------------------

#--- Connection:
print('Connecting...')
vehicle = connect(args.connect)

#-- parameters
rad_2_deg   = 180.0/math.pi
deg_2_rad   = 1.0/rad_2_deg

#-- tag
id_to_find  = 72
marker_size = 10 #- [cm]
freq_send   = 1  #- [Hz] Frequency to send commands

land_alt_cm         = 25.0          #- [cm] Land when below
angle_descend       = 20*deg_2_rad  #- [rad] angle within which we descend
descend_speed_cms   = 30.0          #- [cm/s] Descending speed

#--- CAMERA AND ARUCO TAG
cwd             = path.dirname(path.abspath(__file__))
calib_path      = cwd + "/../opencv/"   #- assuming this will run from the scripts folder
camera_matrix   = np.loadtxt(calib_path + 'cameraMatrix_raspi.txt', delimiter=',')
camera_distort  = np.loadtxt(calib_path + 'cameraDistortion_raspi.txt', delimiter=',')

#-- Create the object
aruco_tracker   = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size,
                        camera_matrix=camera_matrix, camera_distortion=camera_distort)
                        
#-- save the time
time_0  = time.time()

while True:

    #--- Find the marker
    marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)
    
    if marker_found:
    
        #--- convert to UAV frame
        x_cm, y_cm      = camera_2_uav(x_cm, y_cm)
        
        #--- save UAV location in a temporary object
        uav_location    = vehicle.location.global_relative_frame
        
        #--- Use barometric altitude if higher than 2 meters
        if uav_location.alt >= 5.0:
            z_cm = uav_location.alt*100.0
            
        #--- Convert the marker location to angles
        angle_x, angle_y    = marker_position_to_angle(x_cm, y_cm, z_cm)
        
        #--- Send the message
        if time.time() >= time_0 + 1.0/freq_send:
            time_0 = time.time()
            
            print ("Marker Found x = %5.0f  y = %5.0f  y = %5.0f")
            
            #-- Convert to North East
            north, east     = uav_2_ne(x_cm, y_cm, vehicle.attitude.yaw)
            
            #-- Get the marker location (pasing values in meters)
            marker_lat, marker_lon  = get_location_m(uav_location, north*0.01, east*0.01)
            
            #-- If the angle is small, descend, otherwise, maintain the current altitude
            if (angle_x <= angle_descend and angle_y <= angle_descend):
                print ("Low error: Descending")
                #-- subtract a step given by the descending speed times the sample time
                target_altitude = uav_location.alt - (descend_speed_cms*0.01/freq_send)
            else:
                target_altitude = uav_location.alt
                
            #-- Get the target 
            target_location = LocationGlobalRelative(marker_lat, marker_lon, target_altitude)
            
            #-- Send the commands
            vehicle.simple_goto(target_location)
            
        #--- If too low, command to land (Only if in GUIDED mode)
        if z_cm <= land_alt_cm and vehicle.mode == "GUIDED"
            print (" --> Commanding to LAND")
            vehicle.mode = "LAND"
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
        
        
        
        
        
        
        
        
        
        
        
        
        
        

























































    














    
    
    
    
    
    