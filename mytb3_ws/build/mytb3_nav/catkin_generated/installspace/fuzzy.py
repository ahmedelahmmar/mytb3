#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# ROS initialization
rospy.init_node('obstacle_avoidance_publisher', anonymous=True)
velocity_publisher = rospy.Publisher('/mytb3/cmd_vel', Twist, queue_size=10)

# Define fuzzy input variables
distance = ctrl.Antecedent(np.arange(0, 201, 1), 'distance')  # Distance in cm
angle = ctrl.Antecedent(np.arange(np.radians(-45), np.radians(46), np.radians(1)), 'angle')       # Angle in radians

# Define fuzzy output variables
linear_velocity = ctrl.Consequent(np.arange(0.13, 0.26, 0.01), 'linear_velocity')  # Linear velocity (m/s)
angular_velocity = ctrl.Consequent(np.arange(-5.5, 5.6, 0.1), 'angular_velocity')  # Angular velocity (rad/s)

# Adjusted membership functions for angle
angle['right']   = fuzz.trimf(angle.universe, [np.radians(-45), np.radians(-25), np.radians(0)])  # Wider "left"
angle['center'] = fuzz.trimf(angle.universe, [np.radians(-20), np.radians(0), np.radians(20)])   # Narrower "center"
angle['left']  = fuzz.trimf(angle.universe, [np.radians(0), np.radians(25), np.radians(45)])   # Wider "right"

# Adjusted membership functions for linear velocity
linear_velocity['slow'] = fuzz.trimf(linear_velocity.universe, [0.13, 0.14, 0.15])
linear_velocity['medium'] = fuzz.trimf(linear_velocity.universe, [0.14, 0.175, 0.2])
linear_velocity['fast'] = fuzz.trimf(linear_velocity.universe, [0.175, 0.225, 0.25])

# Adjusted membership functions for angular velocity
angular_velocity['right'] = fuzz.trimf(angular_velocity.universe, [-1.5, -1, 0])
angular_velocity['straight'] = fuzz.trimf(angular_velocity.universe, [-0.5, 0, 0.5])
angular_velocity['left'] = fuzz.trimf(angular_velocity.universe, [0, 1, 1.5])

# angular_velocity['left'] = fuzz.trimf(angular_velocity.universe, [-1, -0.7, 0])
# angular_velocity['straight'] = fuzz.trimf(angular_velocity.universe, [-0.1, 0, 0.1])
# angular_velocity['right'] = fuzz.trimf(angular_velocity.universe, [0, 0.7, 1])

# Membership functions for distance remain the same
distance['close'] = fuzz.trimf(distance.universe, [0, 30, 75])
distance['medium'] = fuzz.trimf(distance.universe, [30, 50, 100])
distance['far'] = fuzz.trimf(distance.universe, [75, 175, 200])

# Define fuzzy rules (unchanged)
rules = [
    ctrl.Rule(distance['close'] & angle['center'], 
              (linear_velocity['slow'], angular_velocity['right'])),
    ctrl.Rule(distance['close'] & angle['left'], 
              (linear_velocity['slow'], angular_velocity['right'])),
    ctrl.Rule(distance['close'] & angle['right'], 
              (linear_velocity['slow'], angular_velocity['left'])),
    ctrl.Rule(distance['medium'] & angle['center'], 
              (linear_velocity['medium'], angular_velocity['right'])),
    ctrl.Rule(distance['medium'] & angle['left'], 
              (linear_velocity['medium'], angular_velocity['right'])),
    ctrl.Rule(distance['medium'] & angle['right'], 
              (linear_velocity['medium'], angular_velocity['left'])),
    ctrl.Rule(distance['far'] & angle['center'], 
              (linear_velocity['fast'], angular_velocity['straight'])),
    ctrl.Rule(distance['far'] & angle['left'], 
              (linear_velocity['fast'], angular_velocity['right'])),
    ctrl.Rule(distance['far'] & angle['right'], 
              (linear_velocity['fast'], angular_velocity['left'])),
]

# Create the control system and simulation
control_system = ctrl.ControlSystem(rules)
sim = ctrl.ControlSystemSimulation(control_system)

# Function to process obstacle information
def obstacle_avoidance_logic(obstacle_distance, obstacle_angle):
    """
    Given the distance and angle to an obstacle, compute linear and angular velocities using fuzzy logic.
    """
    # if obstacle_distance <= 0 or obstacle_distance > 70:
    #     rospy.loginfo("No obstacle detected or out of range. Stopping the robot.")
    #     vel_msg = Twist()
    #     vel_msg.linear.x = 0.0
    #     vel_msg.angular.z = 0.0
    #     velocity_publisher.publish(vel_msg)
    #     return

    # Provide inputs to the fuzzy logic system
    sim.input['distance'] = obstacle_distance
    sim.input['angle'] = obstacle_angle

    # Compute the outputs
    sim.compute()

    # Get the velocities
    linear_speed = sim.output['linear_velocity']
    angular_speed = sim.output['angular_velocity']

    # # Determine turning direction
    # if angular_speed > 0.1:  # Turning right
    #     turn_direction = "right"
    # elif angular_speed < -0.1:  # Turning left
    #     turn_direction = "left"
    # else:  # Going straight
    #     turn_direction = "straight"

    # rospy.loginfo(f"Distance: {obstacle_distance:.2f} cm, Angle: {obstacle_angle:.5f} rad")
    # rospy.loginfo(f"Linear Velocity: {linear_speed:.2f} m/s, Angular Velocity: {angular_speed:.2f} rad/s")
    # rospy.loginfo(f"Robot is turning {turn_direction}.")

    vel_msg = Twist()
    vel_msg.linear.x = linear_speed
    vel_msg.angular.z = angular_speed
    velocity_publisher.publish(vel_msg)

# Callback function to update obstacle pose
def update_obstacle_pose(msg):
    obstacle_distance = msg.position.z * 100  # Convert to cm
    obstacle_angle = msg.orientation.z       # Angle from pose

    rospy.loginfo(f"Received Distance: {obstacle_distance:.2f} cm, Angle: {obstacle_angle:.2f}°")
    obstacle_avoidance_logic(obstacle_distance, obstacle_angle)

def main():
    rospy.Subscriber('/mytb3/slam/obstacle', Pose, update_obstacle_pose)
    rospy.loginfo("Obstacle avoidance node with fuzzy logic is running.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass