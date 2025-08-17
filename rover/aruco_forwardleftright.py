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
#ID Numbers
fwd = 272
left = 3
right = 896

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

#Rpi
calib_path="/home/bob/xlakbay-dronekit/rover/camera/calibrationFiles/"
#Windows
#calib_path="C:\\Users\\Bob\\Desktop\\xlakbay-dronekit\\rover\\camera\\calibrationFiles\\"
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

##########MAIN EXECUTABLE###########

vehicle = connectMyCopter()
arm()

seconds = 99999999  # Run indefinitely until manually stopped

start_time = time.time()
last_print_time = time.time()  # Track the last time text was printed

try:
    while time.time() - start_time < seconds:
        frame = cap.read()
        frame_np = np.array(frame)
        gray_img = cv2.cvtColor(frame_np, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)

        detected_ids = []
        if ids is not None:
            detected_ids = [int(i[0]) for i in ids]

        # Print detected IDs every 1 second
        if time.time() - last_print_time >= 2.5:
            if detected_ids:
                print("Detected marker IDs:", detected_ids)
                if fwd in detected_ids:
                    print("Moving forward")
                    send_local_ned_velocity(0.25, 0, 0)
                elif right in detected_ids:
                    print("Turning right")
                    send_local_ned_velocity(0, 0.25, 0)
                elif left in detected_ids:
                    print("Turning left")
                    send_local_ned_velocity(0, -0.25, 0)
                else:
                    print("No movement command detected, stopping.")
                    #send_local_ned_velocity(0, 0, 0)
            else:
                print("No markers detected.")
                #send_local_ned_velocity(0, 0, 0)
            print("")
            last_print_time = time.time()

except KeyboardInterrupt:
    print("Program interrupted. Stopping the vehicle.")
    #send_local_ned_velocity(0, 0, 0)  # Ensure the vehicle stops
    vehicle.armed = False

