#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Pose2D, Pose
from cv_bridge import CvBridge

# Load camera calibration parameters (replace with your calibration data)
camera_matrix = np.array([[514.52065011, 0, 360.97046643], 
                          [0, 514.59986572, 248.94976737], 
                          [0, 0, 1]], dtype=float)
dist_coeffs = np.array([0.14421438, -0.36465162, 0.00264097, -0.00279912, 0.06839954], dtype=float)

# Define ArUco dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

# Start video capture (replace with your IP camera URL if needed)
IP_Cam = "http://192.168.200.200:8080/video"
cap = cv2.VideoCapture(IP_Cam)

# Initialize ROS node
rospy.init_node('aruco_detector', anonymous=True)
goal_pub = rospy.Publisher('/mytb3/slam/goal', Pose, queue_size=10)
obstacle_pub = rospy.Publisher('/mytb3/slam/obstacle', Pose, queue_size=10)

# Create a CvBridge to convert OpenCV images to ROS messages
bridge = CvBridge()

def draw_dashed_line(img, pt1, pt2, color, thickness=1, dash_length=10):
    """Draw a dashed line from pt1 to pt2."""
    x1, y1 = pt1
    x2, y2 = pt2
    dist = int(np.sqrt((x2 - x1)**2 + (y2 - y1)**2))
    if dist == 0:
        return
    for i in range(0, dist, 2 * dash_length):
        start_point = (int(x1 + (x2 - x1) * i / dist), int(y1 + (y2 - y1) * i / dist))
        end_point = (int(x1 + (x2 - x1) * (i + dash_length) / dist), int(y1 + (y2 - y1) * (i + dash_length) / dist))
        cv2.line(img, start_point, end_point, color, thickness)

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        continue

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is not None:
        for corner, marker_id in zip(corners, ids):
            # Convert marker ID from numpy array to integer
            marker_id = marker_id[0]

            # Check if the marker is the goal or an obstacle
            if marker_id == 1:
                label = "Goal"
                color = (0, 255, 0)  # Green for goal
                publisher = goal_pub  # Publisher for goals
            else:
                label = "Obstacle"
                color = (0, 0, 255)  # Red for obstacle
                publisher = obstacle_pub  # Publisher for obstacles

            # Estimate pose of each marker
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, 0.05, camera_matrix, dist_coeffs)

            # Draw marker and axis
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

            # Calculate distance and angle
            distance = np.linalg.norm(tvec[0][0])
            angle = np.degrees(np.arctan2(-tvec[0][0][0], tvec[0][0][2]))  # Angle in degrees

            # Convert to Pose2D and publish
            pose = Pose()
            pose.position.x = tvec[0][0][2]  # Forward distance
            pose.position.y = -tvec[0][0][0]  # Lateral distance
            pose.position.z = distance

            if (-10 < angle) and (angle < 10):
                pose.orientation.z = 0
            else:
                pose.orientation.z = np.radians(angle)  # Orientation in radians

            publisher.publish(pose)

            # Draw dashed line from the center of the camera to the marker center
            marker_center = tuple(np.mean(corner[0], axis=0).astype(int))
            frame_center = (frame.shape[1] // 2, frame.shape[0])
            draw_dashed_line(frame, frame_center, marker_center, color, thickness=2)

            # Display distance, angle, and label on the frame
            cv2.putText(frame, f"ID: {marker_id} ({label}) Dist: {distance:.2f}m Angle: {angle:.2f} deg",
                        (marker_center[0] + 10, marker_center[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # Display the frame
    cv2.imshow("ArUco Detection", frame)

    # Break the loop with 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
