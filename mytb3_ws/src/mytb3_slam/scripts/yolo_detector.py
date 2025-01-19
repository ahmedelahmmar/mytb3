#!/usr/bin/env python3
import cv2
import numpy as np
from ultralytics import YOLO
import threading
import math
import time
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
import rospy
import tf

WATER_BOTTLE_GOAL = 39
SMALL_CUP_OBSTACLE = 41
CAR_OBSTACLE = 2

# Load YOLO model (nano version for speed)
model = YOLO('yolo11n.pt', verbose=False)

# Camera matrix (from calibration)
camera_matrix = np.array([[514.52065011, 0, 360.97046643],
                          [0, 514.59986572, 248.94976737],
                          [0, 0, 1]], dtype=float)
focal_length = camera_matrix[0, 0]  # Focal length in pixels



# IP Camera URL
ip_camera_url = "http://192.168.1.3:8080/video"
cap = cv2.VideoCapture(ip_camera_url)
if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()


# Frame queue for multithreading
frame_queue = []
frame_lock = threading.Lock()


# ROS publishers
rospy.init_node('yolo_detector', anonymous=True)
goal_pub = rospy.Publisher('/mytb3/slam/goal', Pose, queue_size=10)
obstacle_pub = rospy.Publisher('/mytb3/slam/obstacle', Pose, queue_size=10)


# ROS subscriber for robot's current position
current_position = None
def robot_pose_callback(data):
    global current_position
    current_position = data.pose.pose

rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, robot_pose_callback)


initial_goal_x = float(input("Enter initial goal x-coordinate: "))
initial_goal_y = float(input("Enter initial goal y-coordinate: "))

# Function to draw a dashed line
def draw_dashed_line(img, start_point, end_point, color, thickness=1, dash_length=10):
    x1, y1 = start_point
    x2, y2 = end_point
    distance = int(math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))
    for i in range(0, distance, 2 * dash_length):
        start_x = int(x1 + (x2 - x1) * i / distance)
        start_y = int(y1 + (y2 - y1) * i / distance)
        end_x = int(x1 + (x2 - x1) * (i + dash_length) / distance)
        end_y = int(y1 + (y2 - y1) * (i + dash_length) / distance)
        cv2.line(img, (start_x, start_y), (end_x, end_y), color, thickness)


# Function to capture frames in a separate thread
def capture_frames(cap, frame_queue):
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        with frame_lock:
            if len(frame_queue) < 2:  # Limit queue size for minimal latency
                frame_queue.append(frame)

capture_thread = threading.Thread(target=capture_frames, args=(cap, frame_queue))
capture_thread.daemon = True
capture_thread.start()


# Function to calculate distance and angle to the goal
def calculate_distance_and_angle_to_goal():
    global relative_goal_y, relative_goal_y
    if current_position is None:
        rospy.loginfo("[yolo_detector] Waiting for current robot position...")
        return None, None

    dx = initial_goal_x - current_position.position.x
    dy = initial_goal_y - current_position.position.y

    # Distance to the goal
    distance = np.sqrt(dx**2 + dy**2)

    # Angle to the goal in the global frame
    goal_angle = math.atan2(dy, dx)

    # Robot's current orientation (yaw)
    orientation_q = current_position.orientation
    _, _, current_yaw = tf.transformations.euler_from_quaternion([orientation_q.x,
                                                                   orientation_q.y,
                                                                   orientation_q.z,
                                                                   orientation_q.w])

    # Angular difference to the goal
    angular_difference = goal_angle - current_yaw
    return distance, angular_difference



# Variable for frames without detection
frames_without_detections = 0
FRAMES_WITHOUT_DETECTIONS_THRESHOLD = 5


# YOLO inference loop
while not rospy.is_shutdown():
    start_time = time.time()

    # Get a frame from the queue
    with frame_lock:
        if frame_queue:
            frame = frame_queue.pop(0)
        else:
            continue

    # Downscale for faster processing
    frame_resized = cv2.resize(frame, (640, 480))

    # YOLO inference
    results = model(frame_resized, classes=[SMALL_CUP_OBSTACLE, WATER_BOTTLE_GOAL, CAR_OBSTACLE], verbose=False)

    closest_object = None
    closest_distance = float('inf')

    # Iterate over all results and find the closest object
    for result in results:
        for box in result.boxes:
            bbox = box.xyxy[0].cpu().numpy()  # Bounding box [x1, y1, x2, y2]
            cls = int(box.cls.cpu().numpy())  # Class ID
            confidence = float(box.conf.cpu().numpy())  # Confidence score

            # Confidence threshold for valid detection
            if confidence < 0.5:
                continue

            # Determine if the detected object is a goal or an obstacle
            if cls == WATER_BOTTLE_GOAL:  # Class 41: Goal
                label = "Goal"
                color = (0, 255, 0)  # Green for goal
                publisher = goal_pub
                real_object_height = 20  # Goal height in cm

            elif cls == SMALL_CUP_OBSTACLE:  # Class 39: Small Cup (Obstacle)
                label = "Obstacle"
                color = (0, 0, 255)  # Red for obstacle
                publisher = obstacle_pub
                real_object_height = 8.5  # Height for small cup in cm

            elif cls == CAR_OBSTACLE:  # Class 74: Standing Clock (Obstacle)
                label = "Obstacle"
                color = (0, 0, 255)  # Red for obstacle
                publisher = obstacle_pub
                real_object_height = 9  # Height for standing clock in cm
            else:
                continue  # Skip other classes

            # Calculate center of the bounding box
            x_center = int((bbox[0] + bbox[2]) / 2)
            y_center = int((bbox[1] + bbox[3]) / 2)
            object_center = (x_center, y_center)

            # Try using both width and height of bounding box for distance calculation
            object_pixel_height = abs(bbox[3] - bbox[1])
            object_pixel_width = abs(bbox[2] - bbox[0])

            if object_pixel_height > 0:  # Avoid division by zero
                object_distance = (real_object_height * focal_length) / object_pixel_height
            elif object_pixel_width > 0:
                object_distance = (real_object_height * focal_length) / object_pixel_width
            else:
                object_distance = 0

            if object_pixel_width > 0:
                height_to_width_ratio = object_pixel_height / object_pixel_width
            else:
                height_to_width_ratio = 1  # Default to 1 if no width

            # Adjust the distance scaling logic for each type of object
            if cls == WATER_BOTTLE_GOAL:  # Water bottle detected
                if height_to_width_ratio > 1:
                    adjusted_distance = (object_distance * height_to_width_ratio)
                    adjusted_distance = (adjusted_distance / 225) * 80  # Scale to map 225 to 80
                else:
                    adjusted_distance = (object_distance / (height_to_width_ratio + 1e-6))
                    adjusted_distance = (adjusted_distance / 80) * 30  # Scale to map 80 to 30
            else:
                adjusted_distance = object_distance  # Default, no scaling

            # Select the closest object
            if adjusted_distance < closest_distance:
                closest_distance = adjusted_distance
                closest_object = (object_center, label, color, adjusted_distance, bbox)

    # Handling no detection
    if closest_object is None:
        frames_without_detections += 1

        if frames_without_detections >= FRAMES_WITHOUT_DETECTIONS_THRESHOLD:
            frames_without_detections = 0

            goal_distance, goal_angle = calculate_distance_and_angle_to_goal()

            if goal_distance is not None:
                initial_goal_pose               = Pose()
                initial_goal_pose.position.x    = initial_goal_x
                initial_goal_pose.position.y    = initial_goal_y
                initial_goal_pose.position.z    = goal_distance
                initial_goal_pose.orientation.z = goal_angle

                goal_pub.publish(initial_goal_pose)

    else:
        frames_without_detections = 0

        object_center, label, color, object_distance, bbox = closest_object
            
        # Draw bounding box
        cv2.rectangle(frame_resized, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), color, 2)

        # Draw a dashed line from camera center to object center
        camera_center = (frame_resized.shape[1] // 2, frame_resized.shape[0])
        draw_dashed_line(frame_resized, camera_center, object_center, color, thickness=2)

        # Calculate angle to object
        dx = object_center[0] - camera_center[0]
        dy = object_center[1] - camera_center[1]
        angle = (math.pi / 2) - abs(math.atan2(dy, -dx))

        if (angle < (-np.pi / 2)) or ((np.pi / 2) < angle):
            pass

        # Update goal position if detecting goal
        if label == "Goal" and current_position is not None:
            # Get relative coordinates and distance
            rel_x = dx / 100  # Convert to meters
            rel_y = dy / 100
            rel_distance = object_distance / 100
            
            # Get robot's current position and orientation
            robot_x = current_position.position.x
            robot_y = current_position.position.y
            
            # Get robot's orientation (yaw)
            orientation_q = current_position.orientation
            _, _, robot_yaw = tf.transformations.euler_from_quaternion([
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            
            # Calculate goal's global angle
            goal_global_angle = robot_yaw + angle
            
            # Update global goal coordinates
            initial_goal_x = robot_x + rel_distance * np.cos(goal_global_angle)
            initial_goal_y = robot_y + rel_distance * np.sin(goal_global_angle)
            
            rospy.loginfo(f"[yolo_detector] Updated global goal coordinates: ({initial_goal_x:.2f}, {initial_goal_y:.2f})")

        # Display metrics for the object
        label_text = f"{label}: Dist {object_distance / 100:.1f}m, Angle {np.degrees(angle):.1f}°"
        cv2.putText(frame_resized, label_text, (object_center[0], object_center[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Create and publish the Pose message for goal or obstacle
        pose = Pose()
        pose.position.x = dx / 100                 # X (m)
        pose.position.y = dy / 100                 # Y (m)
        pose.position.z = (object_distance) / 100  # Distance (m) (adjust for real height)
        pose.orientation.z = angle                  # Angle (rad)
        publisher.publish(pose)

    # Show the video feed
    cv2.imshow("YOLO Detection", frame_resized)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    rospy.sleep(0.1)

cap.release()
cv2.destroyAllWindows()
