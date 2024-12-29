#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Twist
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import tf

# Initialize the plot
fig, ax = plt.subplots(figsize=(10, 10))
ax.set_xlabel("X position (m)")
ax.set_ylabel("Y position (m)")
ax.set_title("Differential Drive Robot Trajectory")
ax.axis('equal')
ax.grid()

# Data lists for robot position, obstacles, goal, and planned path
x_data = []
y_data = []
theta_data = []  # Orientation data (theta)
obstacles = []   # List of absolute obstacle coordinates
goal = None      # Absolute goal coordinate
planned_path = []  # List of future predicted positions

# Robot's current velocity
current_linear_velocity = 0.0
current_angular_velocity = 0.0

# Prediction parameters
PREDICTION_TIME = 2.0  # Predict 2 seconds into the future
TIME_STEP = 0.1  # Time step for prediction (s)

# Function to handle incoming odometry data
def odom_callback(msg):
    global x_data, y_data, theta_data, current_robot_x, current_robot_y, current_robot_theta

    current_robot_x = msg.pose.pose.position.x
    current_robot_y = msg.pose.pose.position.y

    # Extract x, y position from the PoseWithCovarianceStamped message
    x_data.append(current_robot_x)
    y_data.append(current_robot_y)
    
    # Convert quaternion to Euler angles (yaw)
    orientation_q = msg.pose.pose.orientation
    euler = tf.transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    current_robot_theta = euler[2]
    theta_data.append(current_robot_theta)  # Get yaw (theta) from the Euler angles

# Function to handle obstacle data
def obstacle_callback(msg):
    global obstacles, current_robot_x, current_robot_y, current_robot_theta
    # Transform obstacle position from relative to absolute
    rel_x, rel_y = msg.position.x, msg.position.y
    abs_x = current_robot_x + rel_x * np.cos(current_robot_theta) - rel_y * np.sin(current_robot_theta)
    abs_y = current_robot_y + rel_x * np.sin(current_robot_theta) + rel_y * np.cos(current_robot_theta)
    # Add the obstacle to the list
    obstacles.append((abs_x, abs_y))

# Function to handle goal data
def goal_callback(msg):
    global goal, current_robot_x, current_robot_y, current_robot_theta
    # Transform goal position from relative to absolute
    rel_x, rel_y = msg.position.x, msg.position.y
    abs_x = current_robot_x + rel_x * np.cos(current_robot_theta) - rel_y * np.sin(current_robot_theta)
    abs_y = current_robot_y + rel_x * np.sin(current_robot_theta) + rel_y * np.cos(current_robot_theta)
    goal = (abs_x, abs_y)

# Function to handle velocity commands and predict the planned path
def cmd_vel_callback(msg):
    global current_linear_velocity, current_angular_velocity, planned_path, current_robot_x, current_robot_y, current_robot_theta

    # Update current velocities
    current_linear_velocity = msg.linear.x
    current_angular_velocity = msg.angular.z

    # Predict future positions based on current velocities
    planned_path = []
    x, y, theta = current_robot_x, current_robot_y, current_robot_theta
    for t in np.arange(0, PREDICTION_TIME, TIME_STEP):
        # Update position and orientation using kinematics
        x += current_linear_velocity * np.cos(theta) * TIME_STEP
        y += current_linear_velocity * np.sin(theta) * TIME_STEP
        theta += current_angular_velocity * TIME_STEP
        planned_path.append((x, y))

# ROS Node and subscribers
def main():
    # Initialize ROS node
    rospy.init_node('robot_trajectory_plotter', anonymous=True)

    # Subscribe to the /robot_pose_ekf/odom_combined topic
    rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, odom_callback)

    # Subscribe to the /mytb3/slam/obstacle topic
    rospy.Subscriber('/mytb3/slam/obstacle', Pose, obstacle_callback)

    # Subscribe to the /mytb3/slam/goal topic
    rospy.Subscriber('/mytb3/slam/goal', Pose, goal_callback)

    # Subscribe to the /mytb3/cmd_vel topic
    rospy.Subscriber('/mytb3/cmd_vel', Twist, cmd_vel_callback)

    # Animation update function
    def update_plot(frame):
        ax.clear()  # Clear the plot for dynamic updating
        ax.set_xlabel("X position (m)")
        ax.set_ylabel("Y position (m)")
        ax.set_title("Differential Drive Robot Trajectory")
        ax.axis('equal')
        ax.grid()

        # Plot robot trajectory
        if len(x_data) > 0:
            ax.plot(x_data, y_data, 'b-', label="Trajectory")  # Trajectory line
            ax.plot(x_data[-1], y_data[-1], 'ro', label="Current Position")  # Current position marker

            # Add orientation arrow
            arrow_length = 0.2  # Length of the orientation arrow
            ax.quiver(
                x_data[-1], y_data[-1], 
                np.cos(theta_data[-1]), np.sin(theta_data[-1]), 
                color='r', scale=1/arrow_length, scale_units='xy', width=0.005,
                label="Orientation"
            )

        # Plot obstacles as red points
        if obstacles:
            obstacle_x, obstacle_y = zip(*obstacles)
            ax.scatter(obstacle_x, obstacle_y, color='r', label="Obstacles", s=50)

        # Plot goal as a green point
        if goal:
            goal_x, goal_y = goal
            ax.scatter(goal_x, goal_y, color='g', label="Goal", s=50)

        # Plot planned path as a dashed line
        if planned_path:
            planned_x, planned_y = zip(*planned_path)
            ax.plot(planned_x, planned_y, 'y--', label="Planned Path")

        ax.legend()

    # ROS spin and plot animation
    ani = FuncAnimation(fig, update_plot, interval=100)
    plt.show()
    rospy.spin()  # Keep the node alive

if __name__ == '__main__':
    main()