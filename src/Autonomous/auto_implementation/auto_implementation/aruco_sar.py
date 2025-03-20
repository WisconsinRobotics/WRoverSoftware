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

# Create a named window
cv2.namedWindow('ArUco Tag Detection', cv2.WINDOW_NORMAL)

# Set the window size (e.g., 1280x720 or any size you prefer)
cv2.resizeWindow('ArUco Tag Detection', 1280, 720)

# Define an additional padding factor to add more padding if needed
padding_factor = 2  # This factor controls how much padding to add

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

    # Resize the frame to a square aspect ratio
    height, width = frame.shape[:2]
    size = min(height, width)

    # Apply padding factor (but ensure it doesn't exceed the image size)
    size = int(min(height, width) * padding_factor)
    size = min(size, height, width)  # Ensure size doesn't exceed original image dimensions

    # Calculate padding for left and right to center the square
    top_padding = (height - size) // 2
    bottom_padding = height - size - top_padding
    left_padding = (width - size) // 2
    right_padding = width - size - left_padding

    # Add padding to make it square
    square_frame = cv2.copyMakeBorder(frame, top_padding, bottom_padding, left_padding, right_padding, cv2.BORDER_CONSTANT)

    # Resize the square image to fit in a smaller window (e.g., 720x720)
    resized_frame = cv2.resize(square_frame, (720, 720))

    # Display the resulting frame
    cv2.imshow('ArUco Tag Detection', resized_frame)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()
