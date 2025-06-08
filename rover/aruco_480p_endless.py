##########DEPENDENCIES#############
import cv2
import cv2.aruco as aruco # type: ignore
import numpy as np
from imutils.video import WebcamVideoStream
import imutils

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
import time
import socket
import math
import argparse
from pymavlink import mavutil

###################################

width=640
height=480
cap = WebcamVideoStream(src=0, height=height, width=width).start()

############ARUCO/CV2############
id_to_find=72
marker_size=19 #cm

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

#Rpi
calib_path="/home/bob/xlakbay-dronekit/rover/camera/calibrationFiles/"
#Windows
#calib_path="C:\\Users\\Bob\\Desktop\\xlakbay-dronekit\\rover\\camera\\calibrationFiles"
cameraMatrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
cameraDistortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')


#########FUNCTIONS#################

def connectMyCopter():

	parser = argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect')
	args = parser.parse_args()

	connection_string = args.connect
	vehicle = connect(connection_string,baud=57600,wait_ready=True)

	return vehicle

def arm():
	while vehicle.is_armable!=True:
		print("Waiting for vehicle to become armable.")
		time.sleep(1)
	print("Vehicle is now armable")

	vehicle.mode = VehicleMode("GUIDED")

	while vehicle.mode!='GUIDED':
		print("Waiting for drone to enter GUIDED flight mode")
		time.sleep(1)
	print("Vehicle now in GUIDED MODE. Have fun!!")

	vehicle.armed = True
	while vehicle.armed==False:
		print("Waiting for vehicle to become armed.")
		time.sleep(1)
	print("Vehicle is now armed.")

	return None

def get_distance_meters(targetLocation,currentLocation):
	dLat=targetLocation.lat - currentLocation.lat
	dLon=targetLocation.lon - currentLocation.lon
	
	return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5

def goto(targetLocation):
	distanceToTargetLocation = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)

	vehicle.simple_goto(targetLocation)

	while vehicle.mode.name=="GUIDED":
		currentDistance = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
		if currentDistance<distanceToTargetLocation*.05:
			print("Reached target waypoint.")
			time.sleep(2)
			break
		time.sleep(1)
	return None

##Send a velocity command with +x being the heading of the drone.
def send_local_ned_velocity(vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0, 0,
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		0b0000111111000111,
		0, 0, 0,
		vx, vy, vz,
		0, 0, 0,
		0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()

def reverse(direction):
	# create the CONDITION_YAW command using command_long_encode()
	msg = vehicle.message_factory.command_long_encode(
		0, 0, # target system, target component
		mavutil.mavlink.MAV_CMD_DO_SET_REVERSE, #command
		0, #confirmation
		direction, #Param 1, 0 for forward 1 for backward.
		0,  #Param 2, yaw speed deg/s
		0, #Param 3, Direction -1 ccw, 1 cw
		0, # Param 4, relative offset 1, absolute angle 0
		0,0, 0) # Param 5-7 not used
	vehicle.send_mavlink(msg)
	vehicle.flush()

##Send a velocity command with +x being the heading of the drone.
def send_global_ned_velocity(vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0, # time_boot_ms (not used)
		0, 0, # target system, target component
		mavutil.mavlink.MAV_FRAME_LOCAL_NED, #frame
		0b0000111111000111, #type_mask (only speeds enabled)
		0, 0, 0, # x, y, z positions (not used)
		vx, vy, vz, # x, y, z velocity in m/s
		0, 0, 0, #x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0) #yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
	vehicle.send_mavlink(msg)
	vehicle.flush()

def get_distance_meters(targetLocation,currentLocation):
	dLat=targetLocation.lat - currentLocation.lat
	dLon=targetLocation.lon - currentLocation.lon
	
	return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5

def goto(targetLocation):
	distanceToTargetLocation = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
	print("HIIII")
	vehicle.simple_goto(targetLocation)

	while vehicle.mode.name=="GUIDED":
		currentDistance = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
		if currentDistance<distanceToTargetLocation*.01:
			print("Reached target waypoint.")
			time.sleep(2)
			break
		time.sleep(1)
	return None


def get_aruco_coordinates():
        frame = cap.read() #for Threaded webcam

        #    frame = cv2.resize(frame,(width,height))

        frame_np = np.array(frame)
        gray_img = cv2.cvtColor(frame_np,cv2.COLOR_BGR2GRAY)
        
        ids=''
        corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)
        if ids is not None:
                print("Found these IDs in the frame:")
                print(ids)
        if ids is not None and ids[0] == id_to_find:
                ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=cameraMatrix,distCoeffs=cameraDistortion)
                rvec,tvec = ret[0][0,0,:], ret[1][0,0,:]
                x="{:.2f}".format(tvec[0])
                y="{:.2f}".format(tvec[1])
                z="{:.2f}".format(tvec[2])
                marker_position="MARKER POSITION: x="+x+" y="+y+" z="+z
                print(marker_position)
                print("")
                return x,y,z
        else:
                x=0
                y=0
                z=0
                return x,y,z

def park_at_aruco():
        turn_angle_max=2 ##-2 to 2
        speed_max=.5 #m/s
        
        speed=0
        turn_angle=0
        turn_angle_sign=1 #either 1 or -1

        P_TURN_GAIN=0

        x,y,z=get_aruco_coordinates()
        x=float(x)
        y=float(y)
        z=float(z)
        if x == 0 and y == 0 and z == 0:
                print("Marker not found. Exiting this iteration")
                return None

        xm=x/100 #change default cm into m
        zm=z/100

        if x > 0: 
                turn_angle_sign=1
            	#if the marker is to the right of the rover
				#positive sign would make the rover turn right as well
        else:
                turn_angle_sign=-1
                #negative sign would make the rover turn left
        if z > 500:
                P_TURN_GAIN=.061
                #if far away we want the rover to be less sensitive in its turning
        if z < 500:
                P_TURN_GAIN=.125
                #if near we want the rover to be more sensitive in its turning

        turn_angle = abs(xm)*P_TURN_GAIN
        if turn_angle > turn_angle_max:
                turn_angle = turn_angle_max

        turn_angle = turn_angle*turn_angle_sign
        
        if z > 200:
                speed=speed_max
                #farther than 2m, set speed to 0.5m/s
        elif z>100 and z < 200:
                speed=.3 
                #between 1-2m, set speed to 0.3m/s
        elif z<100:
                speed=0
                print("COULD BE HOME")
                send_local_ned_velocity(0,0,0)
                #less than 1m, stop the rover
                return 0 ##THIS INDICATES THE ROVER MADE IT
       			
        velocity_string="VELOCITY= "+str(speed)
        print(velocity_string)
        send_local_ned_velocity(speed,turn_angle,0)
        return None
##########MAIN EXECUTABLE###########

vehicle = connectMyCopter()

arm()

success_counter = 0
success_break_point = 5

try:
    while True:
        ret = park_at_aruco()
        if ret == 0:
            success_counter += 1
            print(f"Success counter: {success_counter}")
            
            if success_counter == success_break_point:
                print("Vehicle successfully parked. Disarming...")
                send_local_ned_velocity(0, 0, 0)  # Stop the vehicle
                vehicle.armed = False  # Disarm for safety
                success_counter = 0  # Reset the counter
                print("Waiting for marker detection >100cm away to rearm...")
                
                #Until marker is detected again at >100cm, then rearm
                while True:
                    _, _, z = get_aruco_coordinates()
                    if float(z) > 100:  # Marker detected again
                        print("Marker detected. Rearming...")
                        arm()  # Rearm the vehicle
                        break #Break goes back to first while loop
                    
        time.sleep(0.05)
        
except KeyboardInterrupt:
    print("Program interrupted. Stopping the vehicle.")
    send_local_ned_velocity(0, 0, 0)  # Ensure the vehicle stops
    vehicle.armed = False
