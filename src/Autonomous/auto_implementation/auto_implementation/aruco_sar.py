import cv2
import cv2.aruco as aruco

# Initialize the camera
cap = cv2.VideoCapture(0)

# Check if the camera is opened correctly
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Create an ArUco dictionary and parameters for detection
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image.")
        break

    # Convert the image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers in the image
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # If markers are detected, draw them
    if len(corners) > 0:
        frame = aruco.drawDetectedMarkers(frame, corners, ids)

    # Display the resulting frame
    cv2.imshow('ArUco Tag Detection', frame)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()
