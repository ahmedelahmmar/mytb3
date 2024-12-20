#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from cv_bridge import CvBridge
import math
import tf

# Load camera calibration parameters (replace with your calibration data)
camera_matrix = np.array([[514.52065011, 0, 360.97046643],
                          [0, 514.59986572, 248.94976737],
                          [0, 0, 1]], dtype=float)
dist_coeffs = np.array([0.14421438, -0.36465162, 0.00264097, -0.00279912, 0.06839954], dtype=float)

# Define ArUco dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

# Start video capture (replace with your IP camera URL if needed)
IP_Cam = "http://172.20.10.3:8080/video"
cap = cv2.VideoCapture(IP_Cam)

# Initialize ROS node
rospy.init_node('aruco_detector', anonymous=True)
goal_pub = rospy.Publisher('/mytb3/slam/goal', Pose, queue_size=10)
obstacle_pub = rospy.Publisher('/mytb3/slam/obstacle', Pose, queue_size=10)
cmd_vel_pub = rospy.Publisher('/mytb3/cmd_vel', Twist, queue_size=10)

# Robot's current position
current_position = None

# Create a CvBridge to convert OpenCV images to ROS messages
bridge = CvBridge()

# Ask for an initial goal coordinate
initial_goal_x = float(input("Enter initial goal x-coordinate: "))
initial_goal_y = float(input("Enter initial goal y-coordinate: "))

# Configurable IDs for goal markers
GOAL_IDS = [25]  # Modify this as needed

# Number of consecutive frames without ArUco markers before fallback
FRAMES_WITHOUT_MARKERS_THRESHOLD = 10  # Modify this as needed
frames_without_markers = 0  # Counter for frames without ArUco markers


def is_goal(marker_id):
    """Determine if the given marker ID is a goal."""
    if GOAL_IDS is None:
        return False  # Any marker can be a goal
    elif isinstance(GOAL_IDS, int):
        return marker_id == GOAL_IDS  # Check for a single specific ID
    elif isinstance(GOAL_IDS, list):
        return marker_id in GOAL_IDS  # Check if the ID is in the list
    return False


def robot_pose_callback(msg):
    """Callback function to update the robot's current position."""
    global current_position
    current_position = msg.pose.pose


def calculate_distance_and_angle_to_goal():
    """Calculate the distance and angular difference to the goal."""
    if current_position is None:
        rospy.loginfo("Waiting for current robot position...")
        return None, None

    dx = initial_goal_x - current_position.position.x
    dy = initial_goal_y - current_position.position.y

    # Distance to the goal
    distance = np.sqrt(dx**2 + dy**2)

    # Angle to the goal in the global frame
    goal_angle = math.atan2(dy, dx)

    # Robot's current orientation (yaw)
    orientation_q = current_position.orientation
    _, _, current_yaw = tf.transformations.euler_from_quaternion([
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w
    ])

    # Angular difference to the goal
    angular_difference = goal_angle - current_yaw
    # angular_difference = math.atan2(math.sin(angular_difference), math.cos(angular_difference))  # Normalize

    return distance, angular_difference


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

# Subscriber for robot's position
rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, robot_pose_callback)

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        continue

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is not None:
        # Reset the counter for frames without ArUco markers
        frames_without_markers = 0

        closest_marker = None
        min_distance = float('inf')

        # Find the closest marker
        for corner, marker_id in zip(corners, ids):
            marker_id = marker_id[0]
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, 0.05, camera_matrix, dist_coeffs)
            distance = np.linalg.norm(tvec[0][0])  # Distance to marker

            if distance < min_distance:
                min_distance = distance
                closest_marker = (marker_id, tvec[0][0], corner)

        # Process the closest marker
        if closest_marker:
            marker_id, tvec, corner = closest_marker

            # Determine if the marker is a goal or obstacle
            if is_goal(marker_id):
                label = "Goal"
                color = (0, 255, 0)  # Green for goal
                publisher = goal_pub
            else:
                label = "Obstacle"
                color = (0, 0, 255)  # Red for obstacle
                publisher = obstacle_pub

            # Draw marker and axis
            cv2.aruco.drawDetectedMarkers(frame, [corner])
            rvec, _, _ = cv2.aruco.estimatePoseSingleMarkers([corner], 0.05, camera_matrix, dist_coeffs)
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

            angle = np.degrees(np.arctan2(-tvec[0], tvec[2]))  # Angle in degrees

            # Create and publish the Pose message
            pose = Pose()

            if ((-20 < angle) and (angle < 20)):
                pose.orientation.z = 0
            else:
                pose.orientation.z = np.radians(angle)
            
            pose.position.x = tvec[2]
            pose.position.y = -tvec[0]
            pose.position.z = min_distance

            publisher.publish(pose)

                        # Draw dashed line from the center of the camera to the marker center
            marker_center = tuple(np.mean(corner[0], axis=0).astype(int))
            frame_center = (frame.shape[1] // 2, frame.shape[0])
            draw_dashed_line(frame, frame_center, marker_center, color, thickness=2)

            # Display distance, angle, and label on the frame
            cv2.putText(frame, f"ID: {marker_id} ({label}) Dist: {min_distance:.2f}m Angle: {angle:.2f} degrees",
                        (marker_center[0] + 10, marker_center[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    else:
        # Increment the counter for frames without markers
        frames_without_markers += 1

        # Only execute fallback logic after the threshold
        if frames_without_markers >= FRAMES_WITHOUT_MARKERS_THRESHOLD:
            distance_to_goal, angular_difference = calculate_distance_and_angle_to_goal()
            if distance_to_goal is not None:
                rospy.loginfo(f"No markers detected. Distance to initial goal: {distance_to_goal:.2f}m. "
                              f"Angular difference: {math.degrees(angular_difference):.2f}°")
                
            initial_goal_pose = Pose()
            initial_goal_pose.position.x = initial_goal_x
            initial_goal_pose.position.y = initial_goal_y
            initial_goal_pose.position.z = distance_to_goal if distance_to_goal else 0
            initial_goal_pose.orientation.z = angular_difference if angular_difference else 0

            if initial_goal_pose.position.z > 0.1:
                goal_pub.publish(initial_goal_pose)
            else:
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0
                cmd_vel_msg.angular.z = 0
                cmd_vel_pub.publish(cmd_vel_msg)

    # Display the frame
    cv2.imshow("ArUco Detection", frame)

    # Break the loop with 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
