#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
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

# Data lists for robot position
x_data = []
y_data = []
theta_data = []  # Orientation data (theta)

# Function to handle incoming odometry data
def odom_callback(msg):
    global x_data, y_data, theta_data
    # Extract x, y position from the PoseWithCovarianceStamped message
    x_data.append(msg.pose.pose.position.x)
    y_data.append(msg.pose.pose.position.y)
    
    # Convert quaternion to Euler angles (yaw)
    orientation_q = msg.pose.pose.orientation
    euler = tf.transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    theta_data.append(euler[2])  # Get yaw (theta) from the Euler angles

# ROS Node and subscriber
def main():
    # Initialize ROS node
    rospy.init_node('robot_trajectory_plotter', anonymous=True)

    # Subscribe to the /robot_pose_ekf/odom_combined topic, which is of type PoseWithCovarianceStamped
    rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, odom_callback)

    # Animation update function
    def update_plot(frame):
        ax.clear()  # Clear the plot for dynamic updating
        ax.set_xlabel("X position (m)")
        ax.set_ylabel("Y position (m)")
        ax.set_title("Differential Drive Robot Trajectory")
        ax.axis('equal')
        ax.grid()

        if len(x_data) > 0:
            # Plot the trajectory as a line
            ax.plot(x_data, y_data, 'b-', label="Trajectory")
            # Highlight the current position
            ax.plot(x_data[-1], y_data[-1], 'ro', label="Current Position")
            # Add an arrow to indicate the orientation
            arrow_length = 0.2  # Length of the orientation arrow
            ax.quiver(
                x_data[-1], y_data[-1], 
                np.cos(theta_data[-1]), np.sin(theta_data[-1]), 
                color='r', scale=1/arrow_length, scale_units='xy', width=0.005,
                label="Orientation"
            )
            ax.legend()

    # ROS spin and plot animation
    ani = FuncAnimation(fig, update_plot, interval=100)
    plt.show()
    rospy.spin()  # Keep the node alive

if __name__ == '__main__':
    main()
