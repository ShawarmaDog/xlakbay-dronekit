import cv2
import cv2.aruco as aruco # type: ignore
import numpy as np
from imutils.video import WebcamVideoStream
import imutils
import time

#############################

width = 640
height = 480
cap = WebcamVideoStream(src=0, height=height, width=width).start()

############ARUCO/CV2############
fwd = 272
left = 3
right = 896

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

calib_path = "C:/Users/Bob/Desktop/xlakbay-dronekit/rover/camera/calibrationFiles/"
cameraMatrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
cameraDistortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')
#############################

seconds = 9999999  # Run indefinitely until manually stopped
counter = 0

start_time = time.time()
last_print_time = time.time()  # Track the last time text was printed

while time.time() - start_time < seconds:
    frame = cap.read()  # For Threaded webcam
    frame_np = np.array(frame)
    gray_img = cv2.cvtColor(frame_np, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)

    detected_ids = []
    if ids is not None:
        detected_ids = [int(i[0]) for i in ids]
        aruco.drawDetectedMarkers(frame_np, corners)

    # Print detected IDs every 1 second
    if time.time() - last_print_time >= 1:
        if detected_ids:
            print("Detected marker IDs:", detected_ids)
            if fwd in detected_ids:
                print("forward")
            if right in detected_ids:
                print("right")
            if left in detected_ids:
                print("left")
        else:
            print("No markers detected.")
        print("")
        last_print_time = time.time()

    cv2.imshow('frame', frame_np)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
        break
    counter += 1

cv2.destroyAllWindows()
