import cv2
import cv2.aruco as aruco
import numpy as np
from imutils.video import WebcamVideoStream
import imutils
import time

#############################

width = 640
height = 480
cap = WebcamVideoStream(src=0, height=height, width=width).start()

############ARUCO/CV2############
id_to_find = 72
marker_size = 19  # cm

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

calib_path = "C:/Users/Bob/Desktop/CameraCalibration/calibrationFiles/"
cameraMatrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
cameraDistortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')
#############################

seconds = 1000000  # Run indefinitely until manually stopped
counter = 0

start_time = time.time()
while time.time() - start_time < seconds:
    frame = cap.read()  # For Threaded webcam
    frame_np = np.array(frame)
    gray_img = cv2.cvtColor(frame_np, cv2.COLOR_BGR2GRAY)
    ids = ''
    corners, ids, rejected = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)
    if ids is not None:
        print("Found these IDs in the frame:")
        print(ids)
    if ids is not None and ids[0] == id_to_find:
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix=cameraMatrix, distCoeffs=cameraDistortion)
        rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]
        x = "{:.2f}".format(tvec[0])
        y = "{:.2f}".format(tvec[1])
        z = "{:.2f}".format(tvec[2])
        marker_position = "MARKER POSITION: x=" + x + " y=" + y + " z=" + z
        print(marker_position)
        print("")
        aruco.drawDetectedMarkers(frame_np, corners)
        aruco.drawAxis(frame_np, cameraMatrix, cameraDistortion, rvec, tvec, 10)
        #this one breaks the program for some reason. drawAxis not found
    else:
        print("ARUCO " + str(id_to_find) + " NOT FOUND IN FRAME.")
        print("")
    cv2.imshow('frame', frame_np)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
        break
    counter += 1

cv2.destroyAllWindows()