import cv2
import cv2.aruco as aruco
import numpy as np
import random
import csv
import time
TRIAL_NO = 4
# Choose dictionary (DICT_4X4_50 is common and good for demos)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# Open webcam
cap = cv2.VideoCapture(1)
# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'MJPG') #(*'MP42')
out = cv2.VideoWriter(f'software/output/output_{TRIAL_NO}.avi', fourcc, 20.0, (640, 480))

limits = [(-70,-110),(-20,20)]
counter = 1

if not cap.isOpened():
    print("Could not open webcam.")
    exit()

def generate_point(corners):
    x=random.randrange(corners[0][1],corners[0][0])/ratio
    y=random.randrange(corners[1][0], corners[1][1])/ratio
    return np.array([int(x),int(y)])

def generate_square_points(counter):
    points = [(-70,-20),(-70,20),(-90,20),(-90,-20)]
    return (np.array(points[counter])/ratio).astype(int)

def calibrate(target_id,size):
    ret, frame = cap.read()
    if not ret:
        print("No value from camera")
        return    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    ids = ids[0]
    if target_id in ids:
        base_index = np.where(ids==target_id)[0][0]
    else:
        print("Target ID not found")
        return
    base_corners = corners[base_index][0]
    top = base_corners[0] - base_corners[1]
    origin = (base_corners[0]-(top/2)).astype(int)
    top = np.linalg.norm(top)
    bottom = np.linalg.norm(base_corners[3] - base_corners[2])
    ratio = (2*size)/(top+bottom)
    return origin, ratio

def calculate_distance(target_id,corners,ids,origin):
    if target_id in ids:
        target_index = np.where(ids==target_id)[0][0]
    else:
        print("Target ID not found")
        return
    target_corners = corners[target_index][0]
    diag = target_corners[0] - target_corners[2]
    center = (target_corners[0] - (diag/2)).astype(int)
    distance = (center-origin)
    return center,distance

start = time.time() 
origin,ratio = calibrate(0,30)
rand_coord = origin+[int(-80/ratio),0]
with open(f'software/output/data_{TRIAL_NO}.csv', 'w',newline='') as csvfile:
    spamwriter = csv.writer(csvfile, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
    spamwriter.writerow(['Timestamp','Block_x','Block_y','Block_orientation','Target_x','Target_y','Target_orientation'])
    while True:
        
        ret, frame = cap.read()
        if not ret:
            break

        # Convert to grayscale (ArUco needs grayscale)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        # Draw detected markers
        # if ids is not None:
        #     aruco.drawDetectedMarkers(frame, corners, ids)
        
        cv2.drawMarker(frame,origin,color=[0,0,255],thickness=2,markerType=cv2.MARKER_TILTED_CROSS,markerSize=7)
        if ids is not None and 1 in ids:
            center,distance = calculate_distance(1,corners,ids,origin)
            cv2.drawMarker(frame,center,color=[0,0,255],thickness=2,markerType=cv2.MARKER_DIAMOND,markerSize=3)
            #cv2.line(frame,origin,center,(0,255,0),2)
            #print(distance*ratio)
        


        if cv2.waitKey(1) & 0xFF == ord('a'):
            #rand_coord = generate_point(limits) + origin
            counter = counter%4
            rand_coord = generate_square_points(counter) + origin
            counter = counter +1
            print(rand_coord)
            
        cv2.drawMarker(frame,rand_coord,color=[0,255,0],thickness=2,markerType=cv2.MARKER_DIAMOND,markerSize=3)
        out.write(frame)        
        cv2.imshow('ArUco Marker Detection', frame)
        # Exit on pressing 'q'
        scaled_center = (center-origin)*ratio
        scaled_target = (rand_coord-origin)*ratio
        spamwriter.writerow([time.time() - start,scaled_center[0],scaled_center[1],0.0,scaled_target[0],scaled_target[1],0.0])
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Cleanup
cap.release()
out.release()
cv2.destroyAllWindows( )
