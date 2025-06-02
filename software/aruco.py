import cv2
import cv2.aruco as aruco

# Choose dictionary (DICT_4X4_50 is common and good for demos)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# Open webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Could not open webcam.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to grayscale (ArUco needs grayscale)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print(corners)
    # Draw detected markers
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

    # Display the result
    cv2.imshow('ArUco Marker Detection', frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
