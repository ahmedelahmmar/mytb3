#!/usr/bin/env python3

import joblib
import rospy
import numpy as np

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist

# Load the trained GradientBoostingRegressor model and scalers
model = joblib.load('/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/gb_model.pkl')  # Load the sklearn model
scaler_X = joblib.load('/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/gb_scaler_X.pkl')  # Load the feature scaler
scaler_y = joblib.load('/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/gb_scaler_y.pkl')  # Load the target scaler

# Initialize global variables to store pose and obstacle data
robot_pose = None
goal_pose = None
obstacle_pose = None

# Callback for robot pose (odom_combined)
def odom_callback(msg):
    global robot_pose
    robot_pose = msg.pose.pose  # msg is of type PoseWithCovarianceStamped

# Callback for goal pose
def goal_callback(msg):
    global goal_pose
    goal_pose = msg  # msg is of type Pose

# Callback for obstacle pose
def obstacle_callback(msg):
    global obstacle_pose
    obstacle_pose = msg  # msg is of type Pose

# Function to calculate the error and make predictions
def calculate_error_and_predict():
    if robot_pose is None or goal_pose is None or obstacle_pose is None:
        # Log a warning if the data is missing
        rospy.logwarn("Missing data: robot_pose, goal_pose, or obstacle_pose are None. Publishing a default safe velocity.")

        # Fallback logic: return a safe default linear velocity and neutral angular velocity
        return 0.2, 0  # Safe default velocity (e.g., linear velocity of 0.2 m/s, angular velocity of 0)

    # Calculate the error between robot and goal position
    goal_error_x = goal_pose.position.x - robot_pose.position.x
    goal_error_y = goal_pose.position.y - robot_pose.position.y

    # Extract obstacle information
    relative_obstacle_distance = obstacle_pose.position.z
    relative_obstacle_orientation = obstacle_pose.orientation.z

    # Prepare the feature vector for the model
    features = np.array([[relative_obstacle_distance, relative_obstacle_orientation, goal_error_x, goal_error_y]])

    # Scale the features using the same scaler used during training
    features_scaled = scaler_X.transform(features)

    # Make the prediction using the trained model (sklearn)
    output = model.predict(features_scaled)

    # Extract linear and angular velocities from the output
    linear_velocity = output[0][0]
    angular_velocity = output[0][1]

    return linear_velocity, angular_velocity

def main():
    # Initialize ROS node
    rospy.init_node('ai_nav_node')

    # Initialize subscribers
    rospy.Subscriber('/mytb3/slam/goal', Pose, goal_callback, queue_size=10)
    rospy.Subscriber('/mytb3/slam/obstacle', Pose, obstacle_callback, queue_size=10)
    rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, odom_callback, queue_size=10)

    cmd_vel_pub = rospy.Publisher('/mytb3/cmd_vel', Twist, queue_size=10)
    cmd_vel = Twist()

    # Set the loop rate
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        cmd_vel.linear.x, cmd_vel.angular.z = calculate_error_and_predict()
        cmd_vel_pub.publish(cmd_vel)

        rate.sleep()

if __name__ == '__main__':
    main()
