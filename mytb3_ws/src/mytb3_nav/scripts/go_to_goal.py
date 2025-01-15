#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped
import math
import tf.transformations

class GoalNavigator:
    def __init__(self):
        rospy.init_node('go_to_goal', anonymous=True)
        
        # Control parameters
        self.LINEAR_MAX_SPEED = 0.5
        self.ANGULAR_MAX_SPEED = 5.9
        self.LINEAR_MIN_SPEED = 0.15
        self.linear_tolerance = 0.1
        
        # PID parameters
        self.kp_linear = 1.5
        self.kp_angular = 2.5
        
        # State variables
        self.current_pose = None
        self.current_goal = None
        self.is_moving = False
        
        # Publishers and Subscribers
        self.velocity_publisher = rospy.Publisher('/mytb3/cmd_vel', Twist, queue_size=10)
        # rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.update_pose)
        rospy.Subscriber('/mytb3/slam/goal', Pose, self.goal_callback)

    # def update_pose(self, msg):
    #     """Update robot's current pose"""
    #     self.current_pose = msg.pose.pose

    def goal_callback(self, goal_msg):
        """Handle new goal messages"""
        self.current_goal = goal_msg
        self.navigate()

    def calculate_velocities(self, distance, angle):
        """Calculate appropriate linear and angular velocities"""
        # First handle the angular velocity to align with the goal
        if abs(angle) > math.radians(5):  # If angle error is greater than 5 degrees
            linear_vel = 0  # Stop and turn
            angular_vel = self.kp_angular * angle
        else:
            # Once aligned, calculate velocities
            linear_vel = self.kp_linear * distance
            angular_vel = self.kp_angular * angle * 0.5  # Reduced angular correction while moving

            # Apply minimum linear speed if moving and well-aligned
            if distance > self.linear_tolerance:
                linear_vel = max(linear_vel, self.LINEAR_MIN_SPEED)

        # Cap velocities
        linear_vel = max(-self.LINEAR_MAX_SPEED, min(linear_vel, self.LINEAR_MAX_SPEED))
        angular_vel = max(-self.ANGULAR_MAX_SPEED, min(angular_vel, self.ANGULAR_MAX_SPEED))

        return linear_vel, angular_vel

    def navigate(self):
        """Main navigation logic"""
        if not self.current_goal:
            return

        # Get current goal parameters
        distance = self.current_goal.position.z
        angle = self.current_goal.orientation.z

        # Check if goal reached
        if distance <= self.linear_tolerance:
            self.stop_robot()
            rospy.loginfo("Goal reached!")
            return

        # Calculate velocities
        linear_vel, angular_vel = self.calculate_velocities(distance, angle)

        # Create and publish velocity command
        vel_msg = Twist()
        
        # Prioritize turning when angle is large
        if abs(angle) > math.radians(30):
            vel_msg.linear.x = linear_vel * 0.5  # Reduce linear velocity while turning
            vel_msg.angular.z = angular_vel
        else:
            vel_msg.linear.x = linear_vel
            vel_msg.angular.z = angular_vel

        self.velocity_publisher.publish(vel_msg)
        
        # Log navigation info
        rospy.logdebug(f"Distance: {distance:.2f}m, Angle: {math.degrees(angle):.2f}°")
        rospy.logdebug(f"Velocities - Linear: {linear_vel:.2f} m/s, Angular: {angular_vel:.2f} rad/s")

    def stop_robot(self):
        """Stop the robot smoothly"""
        vel_msg = Twist()
        self.velocity_publisher.publish(vel_msg)
        self.is_moving = False

    def run(self):
        """Main run loop"""
        rate = rospy.Rate(10)  # 10 Hz
        rospy.loginfo("Goal navigator is ready...")
        
        while not rospy.is_shutdown():
            if self.current_goal and self.current_pose:
                self.navigate()
            rate.sleep()

if __name__ == '__main__':
    try:
        navigator = GoalNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass