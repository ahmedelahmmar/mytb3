#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose, Pose2D, PoseWithCovarianceStamped
import math
import tf.transformations

# Global variables
current_pose = PoseWithCovarianceStamped()  # Current pose of the robot
goal = Pose()                               # Current goal position

LINEAR_TOLERANCE = 0.25      # Tolerance for reaching the goal (meters)
kp_linear = 1               # Proportional gain for linear velocity
kp_angular = 2.5            # Proportional gain for angular velocity
velocity_publisher = None   # Publisher for robot velocity commands


def update_pose(msg):
    """
    Callback function to update the robot's current position and orientation.
    """
    global current_pose
    current_pose.pose.pose = msg.pose.pose


def move_to_goal(goal_msg):
    """
    Callback function to move the robot toward the received goal.
    Triggered upon receiving a new goal message.
    """
    global current_pose
    vel_msg = Twist()

    # Calculate the distance to the goal
    distance_to_goal = goal_msg.position.z

    rospy.loginfo(f"[go_to_goal] Recieved Goal Data! Dist: {goal_msg.position.z:.2f}m, theta: {math.degrees(goal_msg.orientation.z):.2f}°")
    
    # If the robot is close enough to the goal, stop
    if distance_to_goal <= LINEAR_TOLERANCE:
        rospy.loginfo("[go_to_goal] Goal reached!")
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        return


    angle_diff = goal_msg.orientation.z

    # Calculate linear and angular velocities
    linear_velocity = kp_linear * distance_to_goal
    angular_velocity = kp_angular * angle_diff

    # if abs(angle_diff) > math.radians(15):
    #     linear_velocity = 0.15

    # Cap velocities
    linear_velocity = max(-0.5, min(linear_velocity, 0.5))  # Cap linear velocity
    angular_velocity = max(-2.5, min(angular_velocity, 2.5))  # Cap angular velocity

    # Publish the velocity commands
    vel_msg = Twist()

    vel_msg.linear.x = linear_velocity
    vel_msg.angular.z = angular_velocity

    velocity_publisher.publish(vel_msg)



def main():
    """
    Main function to initialize the node, publishers, and subscribers.
    """
    global velocity_publisher

    # Initialize the ROS node
    rospy.init_node('nav_to_goal', anonymous=True)

    # Publisher for robot velocity commands
    velocity_publisher = rospy.Publisher('/mytb3/cmd_vel', Twist, queue_size=10)

    # Subscribers
    rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, update_pose)
    rospy.Subscriber('/mytb3/slam/goal', Pose, move_to_goal)
    rospy.spin()  # Keep the node running


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass