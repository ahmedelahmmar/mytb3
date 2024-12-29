#!/usr/bin/env python3

import rospy
import torch
import torch.nn as nn
import numpy as np
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from tf.transformations import euler_from_quaternion
import joblib



# Define the PyTorch MLP model as used during training
class MLPModel(nn.Module):
    def __init__(self, input_size, output_size):
        super(MLPModel, self).__init__()
        self.hidden1 = nn.Linear(input_size, 100)
        self.hidden2 = nn.Linear(100, 50)
        self.output = nn.Linear(50, output_size)

    def forward(self, x):
        x = torch.relu(self.hidden1(x))
        x = torch.relu(self.hidden2(x))
        x = self.output(x)
        return x

# Load the trained PyTorch model and scaler
model = MLPModel(input_size=4, output_size=2)
model.load_state_dict(torch.load('/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/mlp_model.pth', map_location=torch.device('cpu'), weights_only=True))
model.eval()  # Set the model to evaluation mode

scaler = joblib.load('/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/scaler.pkl')

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
        return 0.2, 0# Ensure that all required data is available

    # Calculate the error between robot and goal position
    goal_error_x = goal_pose.position.x - robot_pose.position.x
    goal_error_y = goal_pose.position.y - robot_pose.position.y

    # Extract obstacle information
    relative_obstacle_distance = obstacle_pose.position.z
       
    relative_obstacle_orientation = obstacle_pose.orientation.z

    # Prepare the feature vector for the model
    features = np.array([[relative_obstacle_distance, relative_obstacle_orientation, goal_error_x, goal_error_y]])
    
    # Make the prediction using the trained model (PyTorch)
    with torch.no_grad():
        features_tensor = torch.tensor(features, dtype=torch.float32)
        output = model(features_tensor).numpy()
    
    # Extract linear and angular velocities from model output
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
    rate = rospy.Rate(1)  # 10 Hz
    while not rospy.is_shutdown():

        cmd_vel.linear.x, cmd_vel.angular.z = calculate_error_and_predict()
        cmd_vel_pub.publish(cmd_vel)

        rate.sleep()


if __name__ == '__main__':
    main()
