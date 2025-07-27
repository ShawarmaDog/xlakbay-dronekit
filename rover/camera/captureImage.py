import cv2
import os

# Define the path to save images
images_folder = r"C:\Users\Bob\Desktop\xlakbay-dronekit\rover\camera\images"

# Open the webcam
cap = cv2.VideoCapture(0)

# Set the resolution to 640x480
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("Press 'f' to capture an image. Press 'q' to quit.")

image_count = 0

while True:
    # Read a frame from the webcam
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame. Exiting...")
        break

    # Display the frame
    cv2.imshow("Webcam", frame)

    # Wait for a key press
    key = cv2.waitKey(1) & 0xFF

    if key == ord('f'):
        # Save the image
        image_path = os.path.join(images_folder, f"image_{image_count}.png")
        cv2.imwrite(image_path, frame)
        print(f"Image saved: {image_path}")
        image_count += 1
    elif key == ord('q'):
        # Quit the program
        print("Exiting...")
        break

# Release the webcam and close the window
cap.release()
cv2.destroyAllWindows()