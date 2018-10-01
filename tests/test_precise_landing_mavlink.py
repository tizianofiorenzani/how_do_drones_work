"""

NOTE: Call it with Mavlink 2.0 as follows:
MAVLINK20=1 python test_.....py

NOTE: be sure to be using the latest dronekit. 
sudo pip uninstall dronekit
sudo pip uninstall pymavlink

cd dronekit-python
git pull

sudo python setup.py build
sudo python setup.py install

"""
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import time
import math
import argparse


from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
from opencv.lib_aruco_pose import *

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default = '')
args = parser.parse_args()
    
#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------     
# Define function to send landing_target mavlink message for mavlink based precision landing
# http://mavlink.org/messages/common#LANDING_TARGET
def send_land_message_v2(x_rad=0, y_rad=0, dist_m=0, x_m=0,y_m=0,z_m=0, time_usec=0, target_num=0):
    msg = vehicle.message_factory.landing_target_encode(
        time_usec,          # time target data was processed, as close to sensor capture as possible
        target_num,          # target num, not used
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame, not used
        x_rad,          # X-axis angular offset, in radians
        y_rad,          # Y-axis angular offset, in radians
        dist_m,          # distance, in meters
        0,          # Target x-axis size, in radians
        0,          # Target y-axis size, in radians
        x_m,          # x	float	X Position of the landing target on MAV_FRAME
        y_m,          # y	float	Y Position of the landing target on MAV_FRAME
        z_m,          # z	float	Z Position of the landing target on MAV_FRAME
        (1,0,0,0),  # q	float[4]	Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        2,          # type of landing target: 2 = Fiducial marker
        1,          # position_valid boolean
    )
    print msg
    vehicle.send_mavlink(msg)

    
def send_land_message_v1(x_rad=0, y_rad=0, dist_m=0, time_usec=0, target_num=0):
    msg = vehicle.message_factory.landing_target_encode(
        time_usec,          # time target data was processed, as close to sensor capture as possible
        target_num,          # target num, not used
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame, not used
        x_rad,          # X-axis angular offset, in radians
        y_rad,          # Y-axis angular offset, in radians
        dist_m,          # distance, in meters
        0,          # Target x-axis size, in radians
        0,          # Target y-axis size, in radians
    )
    print msg
    vehicle.send_mavlink(msg)
  
        
# Define function to send distance_message mavlink message for mavlink based rangefinder, must be >10hz
# http://mavlink.org/messages/common#DISTANCE_SENSOR
def send_distance_message( dist):
    msg = vehicle.message_factory.distance_sensor_encode(
        0,          # time since system boot, not used
        1,          # min distance cm
        10000,      # max distance cm
        dist,       # current distance, must be int
        0,          # type = laser?
        0,          # onboard id, not used
        mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # must be set to MAV_SENSOR_ROTATION_PITCH_270 for mavlink rangefinder, represents downward facing
        0           # covariance, not used
    )
    vehicle.send_mavlink(msg)     

def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)
    
def camera_to_uav(x_cam, y_cam):
    x_uav = x_cam
    y_uav = y_cam
    return(x_uav, y_uav)
        
#--------------------------------------------------
#-------------- CONNECTION  
#--------------------------------------------------    
#-- Connect to the vehicle
print('Connecting...')
vehicle = connect(args.connect)  

#--------------------------------------------------
#-------------- PARAMETERS  
#-------------------------------------------------- 
vehicle.parameters['PLND_ENABLED']       = 1
vehicle.parameters['PLND_TYPE']          = 1 # Mavlink landing backend

vehicle.parameters['LAND_REPOSITION']   = 0 # !!!!!! ONLY FOR SITL IF NO RC IS CONNECTED


# vehicle.parameters['RNGFND_TYPE']       = 10
# vehicle.parameters['RNGFND_MIN_CM']     = 1
# vehicle.parameters['RNGFND_MAX_CM']     = 10000
# vehicle.parameters['RNGFND_GNDCLEAR']   = 5     

#--------------------------------------------------
#-------------- LANDING MARKER  
#--------------------------------------------------    
#--- Define Tag
id_to_find      = 72
marker_size     = 4 #- [cm]
freq_send       = 15 #- Hz


#--- Get the camera calibration path
# Find full directory path of this script, used for loading config and other files
cwd                 = path.dirname(path.abspath(__file__))
calib_path          = cwd+"/../opencv/"
camera_matrix       = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')                                      
aruco_tracker       = ArucoSingleTracker(id_to_find=72, marker_size=4, show_video=False, 
                camera_matrix=camera_matrix, camera_distortion=camera_distortion)
                
                
time_0 = time.time()

#--- Check mavlink standard
mavlink20 = 'MAVLINK20' in os.environ:

while True:                

    marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)
    if True:#marker_found:
        x_cm, y_cm          = camera_to_uav(x_cm, y_cm)
        z_cm                = vehicle.location.global_relative_frame.alt*100.0
        angle_x, angle_y    = marker_position_to_angle(x_cm, y_cm, z_cm)
        
        if time.time() >= time_0 + 1.0/freq_send:
            time_0 = time.time()
            print "Marker found x = %5.0f cm  y = %5.0f cm -> angle_x = %5f  angle_y = %5f"%(x_cm, y_cm, angle_x, angle_y)
            # send_land_message(x_m=x_cm*0.01, y_m=y_cm*0.01, z_m=z_cm*0.01)
            if mavlink20:
                send_land_message_v2(x_rad=angle_x, y_rad=angle_y, dist_m=z_cm*0.01, time_usec=time.time()*1e6)
            else:
                send_land_message_v1(x_rad=angle_x, y_rad=angle_y, dist_m=z_cm*0.01, time_usec=time.time()*1e6)