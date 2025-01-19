#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf

class TrajectoryHeatmap:
    def __init__(self):
        rospy.init_node('trajectory_heatmap', anonymous=True)
        
        # Initialize the plot
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        
        # Grid parameters
        self.x_min, self.x_max = 0, 4
        self.y_min, self.y_max = -4, 4
        self.resolution = 0.1  # 10cm resolution
        
        # Create grid
        self.x_bins = int((self.x_max - self.x_min) / self.resolution)
        self.y_bins = int((self.y_max - self.y_min) / self.resolution)
        self.heatmap = np.zeros((self.y_bins, self.x_bins))
        
        # Trajectory data
        self.x_data = []
        self.y_data = []
        
        # Robot state
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0
        
        # Setup subscriber
        rospy.Subscriber('/robot_pose_ekf/odom_combined', 
                        PoseWithCovarianceStamped, 
                        self.odom_callback)
        
        self.setup_plot()

    def setup_plot(self):
        """Initialize plot settings"""
        self.ax.set_xlabel("X position (m)")
        self.ax.set_ylabel("Y position (m)")
        self.ax.set_title("Robot Trajectory Heatmap")
        
    def update_heatmap(self, x, y):
        """Update heatmap with new position"""
        # Convert position to grid indices
        x_idx = int((x - self.x_min) / self.resolution)
        y_idx = int((y - self.y_min) / self.resolution)
        
        # Ensure indices are within bounds
        if (0 <= x_idx < self.x_bins and 0 <= y_idx < self.y_bins):
            # Add Gaussian around current position
            sigma = 3  # Standard deviation in grid cells
            x_grid = np.arange(max(0, x_idx-3*sigma), min(self.x_bins, x_idx+3*sigma))
            y_grid = np.arange(max(0, y_idx-3*sigma), min(self.y_bins, y_idx+3*sigma))
            
            for i in x_grid:
                for j in y_grid:
                    self.heatmap[j, i] += np.exp(-((i-x_idx)**2 + (j-y_idx)**2)/(2*sigma**2))

    def odom_callback(self, msg):
        """Handle odometry updates"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Store trajectory
        self.x_data.append(self.current_x)
        self.y_data.append(self.current_y)
        
        # Update heatmap
        self.update_heatmap(self.current_x, self.current_y)
        
        # Get orientation
        orientation_q = msg.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_theta = euler[2]

    def update_plot(self, frame):
        """Update plot with current data"""
        self.ax.clear()
        self.setup_plot()
        
        # Plot heatmap
        im = self.ax.imshow(self.heatmap, 
                           extent=[self.x_min, self.x_max, self.y_min, self.y_max],
                           origin='lower',
                           cmap='YlOrRd',
                           aspect='equal')
        
        # Plot trajectory
        if self.x_data:
            self.ax.plot(self.x_data, self.y_data, 'b-', label="Trajectory", alpha=0.7)
            
            # Plot current position
            self.ax.plot(self.x_data[-1], self.y_data[-1], 'ro', label="Robot")
            
            # Plot robot orientation
            arrow_length = 0.2
            self.ax.quiver(
                self.current_x, self.current_y,
                np.cos(self.current_theta), np.sin(self.current_theta),
                color='r', scale=1/arrow_length, scale_units='xy',
                width=0.005, label="Heading"
            )
        
        # Add colorbar
        plt.colorbar(im, ax=self.ax, label='Visit Frequency')
        
        # Set fixed limits
        self.ax.set_xlim(self.x_min, self.x_max)
        self.ax.set_ylim(self.y_min, self.y_max)
        
        # Add grid
        self.ax.grid(True, alpha=0.3)
        
        # Add legend
        self.ax.legend()
        
        # Add current position text
        info_text = f"Robot Position: ({self.current_x:.2f}, {self.current_y:.2f})\n" \
                   f"Robot Heading: {self.current_theta:.2f}rad"
        self.ax.text(0.02, 0.98, info_text, 
                    transform=self.ax.transAxes,
                    verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    def run(self):
        """Main run loop"""
        ani = FuncAnimation(self.fig, self.update_plot, interval=100)
        plt.show()
        rospy.spin()

if __name__ == '__main__':
    try:
        heatmap = TrajectoryHeatmap()
        heatmap.run()
    except rospy.ROSInterruptException:
        pass