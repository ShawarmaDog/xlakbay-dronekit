seconds = 9999999  # Run indefinitely until manually stopped
counter = 0

start_time = time.time()
last_print_time = time.time()  # Track the last time text was printed

while time.time() - start_time < seconds:
    frame = cap.read()  # For Threaded webcam
    frame_np = np.array(frame)
    gray_img = cv2.cvtColor(frame_np, cv2.COLOR_BGR2GRAY)
    ids = ''
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