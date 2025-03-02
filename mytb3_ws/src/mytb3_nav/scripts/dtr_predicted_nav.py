#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
import joblib

# Load the trained Decision Tree model and scalers
model = joblib.load('/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/decision_tree_model_yolo.pkl')
scaler_X = joblib.load('/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/scaler_X_dtr_yolo.pkl')
scaler_y = joblib.load('/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/scaler_y_dtr_yolo.pkl')

# Initialize global variables
robot_pose = None
goal_pose = None
obstacle_pose = None
last_obstacle_time = None
OBSTACLE_TIMEOUT = rospy.Duration(1.0)

def odom_callback(msg):
    global robot_pose
    robot_pose = msg.pose.pose

def goal_callback(msg):
    global goal_pose
    goal_pose = msg

def obstacle_callback(msg):
    global obstacle_pose, last_obstacle_time
    obstacle_pose = msg
    last_obstacle_time = rospy.Time.now()

def is_obstacle_valid():
    if last_obstacle_time is None:
        return False
    return (rospy.Time.now() - last_obstacle_time) < OBSTACLE_TIMEOUT

def get_default_obstacle_values():
    return 10.0, 0.0

def calculate_error_and_predict():
    try:
        if robot_pose is None or goal_pose is None:
            rospy.logwarn_throttle(1, "Missing robot_pose or goal_pose data")
            return 0.0, 0.0

        # Calculate goal errors
        goal_error_x = goal_pose.position.x
        goal_error_y = goal_pose.position.y

        # Get obstacle information
        if is_obstacle_valid() and obstacle_pose is not None:
            relative_obstacle_distance = obstacle_pose.position.z
            relative_obstacle_orientation = obstacle_pose.orientation.z
        else:
            relative_obstacle_distance, relative_obstacle_orientation = get_default_obstacle_values()

        # Create feature array
        features = np.array([[
            relative_obstacle_distance,
            relative_obstacle_orientation,
            goal_error_x,
            goal_error_y
        ]])
        
        # Check for NaN or infinite values
        if np.any(np.isnan(features)) or np.any(np.isinf(features)):
            rospy.logwarn_throttle(1, "Invalid feature values detected")
            return 0.0, 0.0

        # Scale features
        features_scaled = scaler_X.transform(features)
        
        # Make prediction (will be in scaled form)
        predictions_scaled = model.predict(features_scaled)
        
        # Inverse transform the predictions to get actual velocities
        predictions = scaler_y.inverse_transform(predictions_scaled.reshape(1, -1))
        
        # Extract and limit velocities
        linear_velocity = np.clip(predictions[0, 0], 0.0, 0.5)
        angular_velocity = np.clip(predictions[0, 1], -5.9, 5.9)

        return linear_velocity, angular_velocity

    except Exception as e:
        rospy.logerr(f"Error in prediction: {str(e)}")
        return 0.0, 0.0

def main():
    rospy.init_node('ai_nav_node')

    # Publishers and Subscribers
    cmd_vel_pub = rospy.Publisher('/mytb3/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/mytb3/slam/goal', Pose, goal_callback, queue_size=10)
    rospy.Subscriber('/mytb3/slam/obstacle', Pose, obstacle_callback, queue_size=10)
    rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, odom_callback, queue_size=10)

    rate = rospy.Rate(10)
    cmd_vel = Twist()

    while not rospy.is_shutdown():
        try:
            # Get predicted velocities
            linear_vel, angular_vel = calculate_error_and_predict()
            
            # Update command velocities
            cmd_vel.linear.x = linear_vel
            cmd_vel.angular.z = angular_vel
            
            # Publish velocities
            cmd_vel_pub.publish(cmd_vel)
            
            # Debug information
            if rospy.get_param('~debug', False):
                rospy.loginfo(f"Linear: {linear_vel:.2f}, Angular: {angular_vel:.2f}")
                
        except Exception as e:
            rospy.logerr(f"Error in main loop: {str(e)}")
            # Stop the robot in case of error
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            cmd_vel_pub.publish(cmd_vel)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass