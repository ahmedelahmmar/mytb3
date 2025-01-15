#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Twist
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import tf

class TrajectoryPlotter:
    def __init__(self):
        rospy.init_node('robot_trajectory_plotter', anonymous=True)
        
        # Initialize the plot
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.setup_plot()
        
        # Robot state
        self.current_robot_x = 0
        self.current_robot_y = 0
        self.current_robot_theta = 0
        
        # Trajectory data
        self.x_data = []
        self.y_data = []
        self.theta_data = []
        
        # Cost map data
        self.obstacle_dict = {}  # Dictionary to store obstacles
        self.goal_position = None
        self.goal_distance = None
        self.goal_angle = None
        
        # Motion parameters
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.planned_path = []
        
        # Constants
        self.PREDICTION_TIME = 2.0
        self.TIME_STEP = 0.1
        self.plot_range = 5.0  # meters
        
        # Initialize subscribers
        self.setup_subscribers()

    def setup_plot(self):
        """Initialize plot settings"""
        self.ax.set_xlabel("X position (m)")
        self.ax.set_ylabel("Y position (m)")
        self.ax.set_title("Robot Navigation Cost Map")
        self.ax.axis('equal')
        self.ax.grid(True)

    def setup_subscribers(self):
        """Initialize ROS subscribers"""
        rospy.Subscriber('/robot_pose_ekf/odom_combined', 
                        PoseWithCovarianceStamped, self.odom_callback)
        rospy.Subscriber('/mytb3/slam/obstacle', 
                        Pose, self.obstacle_callback)
        rospy.Subscriber('/mytb3/slam/goal', 
                        Pose, self.goal_callback)
        rospy.Subscriber('/mytb3/cmd_vel', 
                        Twist, self.cmd_vel_callback)

    def transform_to_global(self, rel_msg):
        """
        Transform relative coordinates to global coordinates
        rel_msg: Pose message containing relative coordinates and yaw in radians
        """
        # Extract relative coordinates and yaw (already in radians)
        rel_x = rel_msg.position.x
        rel_y = rel_msg.position.y
        rel_distance = rel_msg.position.z
        rel_yaw = rel_msg.orientation.z  # Angle in radians relative to robot's heading

        # Calculate absolute angle in global frame (in radians)
        global_angle = self.current_robot_theta + rel_yaw

        # Calculate global coordinates using relative distance and angle
        abs_x = self.current_robot_x + rel_distance * np.cos(global_angle)
        abs_y = self.current_robot_y + rel_distance * np.sin(global_angle)

        return abs_x, abs_y

    def odom_callback(self, msg):
        """Handle odometry updates"""
        self.current_robot_x = msg.pose.pose.position.x
        self.current_robot_y = msg.pose.pose.position.y
        
        self.x_data.append(self.current_robot_x)
        self.y_data.append(self.current_robot_y)
        
        # Get orientation
        orientation_q = msg.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_robot_theta = euler[2]  # yaw in radians
        self.theta_data.append(self.current_robot_theta)

    def obstacle_callback(self, msg):
        """Handle obstacle updates"""
        # Transform relative coordinates to global
        abs_x, abs_y = self.transform_to_global(msg)
        
        # Add obstacle
        obstacle_id = f"{abs_x:.2f}_{abs_y:.2f}"
        self.obstacle_dict[obstacle_id] = {
            'position': (abs_x, abs_y),
            'distance': msg.position.z,  # Store relative distance
            'angle': msg.orientation.z   # Store relative angle in radians
        }

    def goal_callback(self, msg):
        """Handle goal updates"""
        # Transform relative coordinates to global
        abs_x, abs_y = self.transform_to_global(msg)
        
        self.goal_position = (abs_x, abs_y)
        self.goal_distance = msg.position.z
        self.goal_angle = msg.orientation.z  # in radians

    def cmd_vel_callback(self, msg):
        """Handle velocity commands and update planned path"""
        self.current_linear_velocity = msg.linear.x
        self.current_angular_velocity = msg.angular.z
        
        # Update planned path
        self.planned_path = []
        x, y, theta = self.current_robot_x, self.current_robot_y, self.current_robot_theta
        
        for t in np.arange(0, self.PREDICTION_TIME, self.TIME_STEP):
            x += self.current_linear_velocity * np.cos(theta) * self.TIME_STEP
            y += self.current_linear_velocity * np.sin(theta) * self.TIME_STEP
            theta += self.current_angular_velocity * self.TIME_STEP
            self.planned_path.append((x, y))

    def update_plot(self, frame):
        """Update plot with current data"""
        self.ax.clear()
        self.setup_plot()
        
        # Set plot limits around robot's current position
        self.ax.set_xlim(self.current_robot_x - self.plot_range, 
                        self.current_robot_x + self.plot_range)
        self.ax.set_ylim(self.current_robot_y - self.plot_range, 
                        self.current_robot_y + self.plot_range)
        
        # Plot trajectory
        if self.x_data:
            self.ax.plot(self.x_data, self.y_data, 'b-', label="Trajectory")
            self.ax.plot(self.x_data[-1], self.y_data[-1], 'ro', 
                        label="Robot")
            
            # Plot robot orientation
            arrow_length = 0.2
            self.ax.quiver(
                self.current_robot_x, self.current_robot_y,
                np.cos(self.current_robot_theta), 
                np.sin(self.current_robot_theta),
                color='r', scale=1/arrow_length, scale_units='xy',
                width=0.005, label="Robot Heading"
            )
        
        # Plot obstacles with relative information
        if self.obstacle_dict:
            for obs_data in self.obstacle_dict.values():
                x, y = obs_data['position']
                self.ax.scatter(x, y, color='r', s=50)
                
                # Add text showing relative distance and angle (in radians)
                self.ax.annotate(
                    f'd={obs_data["distance"]:.2f}m\nθ={obs_data["angle"]:.2f}rad',
                    (x, y), xytext=(5, 5), textcoords='offset points'
                )
        
        # Plot goal with relative information
        if self.goal_position:
            x, y = self.goal_position
            self.ax.scatter(x, y, color='g', s=100, marker='*', label="Goal")
            
            # Add text showing relative distance and angle (in radians)
            self.ax.annotate(
                f'd={self.goal_distance:.2f}m\nθ={self.goal_angle:.2f}rad',
                (x, y), xytext=(5, 5), textcoords='offset points'
            )
        
        # Add robot-centric information
        info_text = f"Robot Position: ({self.current_robot_x:.2f}, {self.current_robot_y:.2f})\n" \
                   f"Robot Heading: {self.current_robot_theta:.2f}rad"
        self.ax.text(0.02, 0.98, info_text, transform=self.ax.transAxes, 
                    verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        
        self.ax.legend()
        self.ax.grid(True)

    def run(self):
        """Main run loop"""
        ani = FuncAnimation(self.fig, self.update_plot, interval=100)
        plt.show()
        rospy.spin()

if __name__ == '__main__':
    try:
        plotter = TrajectoryPlotter()
        plotter.run()
    except rospy.ROSInterruptException:
        pass