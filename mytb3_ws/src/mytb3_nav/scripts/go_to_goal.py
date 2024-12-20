#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose, Pose2D, PoseWithCovarianceStamped
import math
import tf.transformations

# Global variables
current_pose = PoseWithCovarianceStamped()  # Current pose of the robot
goal = Pose()  # Current goal position
linear_tolerance = 0.2  # Tolerance for reaching the goal (meters)
kp_linear = 1.5 # Proportional gain for linear velocity
kp_angular = 2.5 # Proportional gain for angular velocity
velocity_publisher = None  # Publisher for robot velocity commands


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

    # rospy.loginfo(f"New goal received: x={goal_msg.x}, y={goal_msg.y}, theta={goal_msg.theta}")
    
    # If the robot is close enough to the goal, stop
    if distance_to_goal <= linear_tolerance:
        rospy.loginfo("Goal reached!")
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        return


    angle_diff = goal_msg.orientation.z

    # Calculate linear and angular velocities
    linear_velocity = kp_linear * distance_to_goal
    angular_velocity = kp_angular * angle_diff

    # Cap velocities
    linear_velocity = max(-0.25, min(linear_velocity, 0.25))  # Cap linear velocity
    angular_velocity = max(-3, min(angular_velocity, 3))  # Cap angular velocity

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
    rospy.init_node('go_to_goal', anonymous=True)

    # Publisher for robot velocity commands
    velocity_publisher = rospy.Publisher('/mytb3/cmd_vel', Twist, queue_size=10)

    # Subscribers
    rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, update_pose)
    rospy.Subscriber('/mytb3/slam/goal', Pose, move_to_goal)

    rospy.loginfo("Move-to-goal node is ready and waiting for goals...")
    rospy.spin()  # Keep the node running


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass