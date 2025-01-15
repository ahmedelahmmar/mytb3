#!/usr/bin/env python3
import cv2
import numpy as np
from ultralytics import YOLO
import threading
import math
import time

# Load YOLO model (nano version for speed)
model = YOLO('yolo11n.pt')

# Camera matrix (from calibration)
camera_matrix = np.array([[514.52065011, 0, 360.97046643],
                          [0, 514.59986572, 248.94976737],
                          [0, 0, 1]], dtype=float)
scaling_factor = 0.026  # Example scaling factor

# IP Camera URL
ip_camera_url = "http://172.20.10.2:8080/video"
cap = cv2.VideoCapture(ip_camera_url)
if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

# Frame queue for multithreading
frame_queue = []
frame_lock = threading.Lock()
processed_frames = []

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

# YOLO inference loop
while True:
    start_time = time.time()

    # Get a frame from the queue
    with frame_lock:
        if frame_queue:
            frame = frame_queue.pop(0)
        else:
            continue

    # Downscale for faster processing
    frame_resized = cv2.resize(frame, (640, 480))

    # YOLO inference (class filter for specific detection, e.g., bottles)
    results = model(frame_resized, classes=[39])  # Focus on class 39

    # Draw detections and compute metrics
    for result in results:
        for box in result.boxes:
            bbox = box.xyxy[0].cpu().numpy()  # Bounding box [x1, y1, x2, y2]
            cls = int(box.cls.cpu().numpy())  # Class ID

            # Calculate center of the bounding box
            x_center = int((bbox[0] + bbox[2]) / 2)
            y_center = int((bbox[1] + bbox[3]) / 2)
            object_center = (x_center, y_center)

            # Draw a dashed line from camera center to object center
            camera_center = (frame_resized.shape[1] // 2, frame_resized.shape[0])
            draw_dashed_line(frame_resized, camera_center, object_center, color=(0, 255, 255), thickness=2)

            # Calculate distance and angle
            dx = x_center - camera_center[0]
            dy = y_center - camera_center[1]
            distance = math.sqrt(dx ** 2 + dy ** 2) * scaling_factor
            angle = 90 - abs(math.degrees(math.atan2(dy, dx)))

            # Display metrics
            label = f"Obj: {model.names[cls]}, Dist: {distance:.2f}cm, Angle: {angle:.2f}°"
            cv2.putText(frame_resized, label, (x_center, y_center - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # Display FPS (optional)
    fps = 1 / (time.time() - start_time)
   # cv2.putText(frame_resized, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    # Show the video feed
    cv2.imshow("YOLO Detection", frame_resized)

    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()

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

SMALL_CUP = 41
WATER_BOTTLE = 39

# Load YOLO model (nano version for speed)
model = YOLO('yolo11n.pt')

# Camera matrix (from calibration)
camera_matrix = np.array([[514.52065011, 0, 360.97046643],
                          [0, 514.59986572, 248.94976737],
                          [0, 0, 1]], dtype=float)
focal_length = camera_matrix[0, 0]  # Focal length in pixels


# IP Camera URL
ip_camera_url = "http://172.20.10.2:8080/video"
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


# Ask for an initial goal coordinate
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

# Function to calculate distance using the bounding box size
def calculate_object_distance(bbox, real_object_height, camera_matrix):
    object_pixel_height = abs(bbox[3] - bbox[1])
    if object_pixel_height > 0:  # Avoid division by zero
        return (real_object_height * camera_matrix[0, 0]) / object_pixel_height
    else:
        return None


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
    results = model(frame_resized)  # Detect all objects

    closest_object = None
    closest_distance = float('inf')

    # Iterate over all results and find the closest object
    for result in results:
        for box in result.boxes:
            bbox = box.xyxy[0].cpu().numpy()  # Bounding box [x1, y1, x2, y2]
            cls = int(box.cls.cpu().numpy())  # Class ID

            # Determine if the detected object is a goal or an obstacle
            if cls == SMALL_CUP:  # Class 41: Goal
                label = "Obstacle"
                color = (0, 0, 255)  
                publisher = obstacle_pub
                real_object_height = 8.5  # Goal height in cm
            elif cls == WATER_BOTTLE:  # Class 39: Obstacle
                label = "Goal"
                color = (0, 255, 0)  # Green for goal
                publisher = goal_pub
                real_object_height = 20  # Obstacle height in cm
            else:
                continue  # Skip other classes

            # Calculate the center of the bounding box
            x_center = int((bbox[0] + bbox[2]) / 2)
            y_center = int((bbox[1] + bbox[3]) / 2)
            object_center = (x_center, y_center)

            # Calculate object distance using the bounding box size
            object_distance = calculate_object_distance(bbox, real_object_height, camera_matrix)
            if object_distance is None:
                continue

            # Select the closest object
            if object_distance < closest_distance:
                closest_distance = object_distance
                closest_object = (object_center, label, color, object_distance)

    # Handling no detection: update the number of frames without detection
    if closest_object is None:
        frames_without_detections += 1

        if frames_without_detections >= FRAMES_WITHOUT_DETECTIONS_THRESHOLD:
            # Calculate distance and angle to the goal and publish the goal pose
            goal_distance, goal_angle = calculate_distance_and_angle_to_goal()
            if goal_distance is not None:
                initial_goal_pose = Pose()
                initial_goal_pose.position.x = initial_goal_x
                initial_goal_pose.position.y = initial_goal_y
                initial_goal_pose.position.z = goal_distance if goal_distance else 0
                initial_goal_pose.orientation.z = goal_angle if goal_angle else 0

                if goal_distance > 0.2:
                    goal_pub.publish(initial_goal_pose)

    else:
        frames_without_detections = 0

        # If a closest object was found, draw and publish it
        if closest_object:
            object_center, label, color, object_distance = closest_object

            # Draw a dashed line from camera center to object center
            camera_center = (frame_resized.shape[1] // 2, frame_resized.shape[0])
            draw_dashed_line(frame_resized, camera_center, object_center, color, thickness=2)

            # Calculate angle to object
            dx = object_center[0] - camera_center[0]
            dy = object_center[1] - camera_center[1]
            angle = (math.pi / 2) - abs(math.atan2(dy, -dx))

            # Display metrics for the object
            label_text = f"{label}: Dist {object_distance:.2f}cm, Angle {angle:.2f}°"
            cv2.putText(frame_resized, label_text, (object_center[0], object_center[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Create and publish the Pose message for goal or obstacle
            pose = Pose()
            pose.position.x = dx / 100                 # X (m)
            pose.position.y = dy / 100                 # Y (m)
            pose.position.z = (object_distance)/ 100       # Distance (m)
            pose.orientation.z = angle                  # Angle (rad)
            publisher.publish(pose)

    # Show the video feed
    cv2.imshow("YOLO Detection", frame_resized)

    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()