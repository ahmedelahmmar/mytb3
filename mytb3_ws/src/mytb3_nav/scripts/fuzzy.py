#!/usr/bin/env python3
from geometry_msgs.msg import Twist, Pose
from skfuzzy import control as ctrl
import skfuzzy as fuzz
import numpy as np
import rospy

# ROS initialization
rospy.init_node('fuzzy_obstacle_avoidance_node', anonymous=False)
pub_vel = rospy.Publisher('/mytb3/cmd_vel', Twist, queue_size=10)


# Define fuzzy input variables
distance = ctrl.Antecedent(np.arange(0, 251, 1), 'distance')  # Distance in cm
angle    = ctrl.Antecedent(np.arange(np.radians(-70), np.radians(71), np.radians(1)), 'angle')       # Angle in radians


# Define fuzzy output variables
linear_velocity     = ctrl.Consequent(np.arange(0.15, 0.5, 0.05), 'linear_velocity')  # Linear velocity (m/s)
angular_velocity    = ctrl.Consequent(np.arange(-5.9, 6.0, 0.1), 'angular_velocity')  # Angular velocity (rad/s)


# Adjusted membership functions for distance
distance['close']   = fuzz.trapmf(distance.universe, [0, 0, 25, 50])
distance['medium']  = fuzz.trapmf(distance.universe, [25, 50, 75, 100])
distance['far']     = fuzz.trapmf(distance.universe, [100, 125, 140, 140])


# Adjusted membership functions for angle with expanded range and wider straight zone
angle['far_right']    = fuzz.trapmf(angle.universe, [np.radians(-70), np.radians(-70), np.radians(-60), np.radians(-50)])
angle['right']        = fuzz.trapmf(angle.universe, [np.radians(-50), np.radians(-40), np.radians(-25), np.radians(-15)])
angle['slight_right'] = fuzz.trapmf(angle.universe, [np.radians(-20), np.radians(-15), np.radians(-10), np.radians(-5)])
angle['straight']     = fuzz.trapmf(angle.universe, [np.radians(-10), np.radians(-5), np.radians(5), np.radians(10)])
angle['slight_left']  = fuzz.trapmf(angle.universe, [np.radians(5), np.radians(10), np.radians(15), np.radians(20)])
angle['left']         = fuzz.trapmf(angle.universe, [np.radians(15), np.radians(25), np.radians(40), np.radians(50)])
angle['far_left']     = fuzz.trapmf(angle.universe, [np.radians(50), np.radians(60), np.radians(70), np.radians(70)])


# Adjusted membership functions for linear velocity
linear_velocity['slow']   = fuzz.trapmf(linear_velocity.universe, [0.15, 0.15, 0.175, 0.2])
linear_velocity['medium'] = fuzz.trapmf(linear_velocity.universe, [0.175, 0.225, 0.275, 0.3])
linear_velocity['fast']   = fuzz.trapmf(linear_velocity.universe, [0.275, 0.3, 0.4, 0.50])


# Adjusted membership functions for angular velocity
angular_velocity['sharp_right']  = fuzz.trapmf(angular_velocity.universe, [-5.9, -5.9, -4.0, -3.0])
angular_velocity['right']        = fuzz.trapmf(angular_velocity.universe, [-4.0, -3.0, -2.0, -1.0])
angular_velocity['slight_right'] = fuzz.trapmf(angular_velocity.universe, [-1.5, -1.0, -0.5, -0.2])
angular_velocity['straight']     = fuzz.trapmf(angular_velocity.universe, [-0.2, -0.1, 0.1, 0.2])
angular_velocity['slight_left']  = fuzz.trapmf(angular_velocity.universe, [0.2, 0.5, 1.0, 1.5])
angular_velocity['left']         = fuzz.trapmf(angular_velocity.universe, [1.0, 2.0, 3.0, 4.0])
angular_velocity['sharp_left']   = fuzz.trapmf(angular_velocity.universe, [3.0, 4.0, 5.9, 5.9])


# Define fuzzy rules with left turn preference
rules = [
    # CLOSE distance rules - Emergency avoidance
    ctrl.Rule(distance['close'] & angle['far_right'], 
              (linear_velocity['slow'], angular_velocity['slight_left'])),
              
    ctrl.Rule(distance['close'] & angle['right'], 
              (linear_velocity['slow'], angular_velocity['sharp_left'])),

    ctrl.Rule(distance['close'] & angle['slight_right'], 
              (linear_velocity['slow'], angular_velocity['left'])),

    ctrl.Rule(distance['close'] & angle['straight'], 
              (linear_velocity['slow'], angular_velocity['sharp_left'])),

    ctrl.Rule(distance['close'] & angle['slight_left'], 
              (linear_velocity['slow'], angular_velocity['right'])),

    ctrl.Rule(distance['close'] & angle['left'], 
              (linear_velocity['slow'], angular_velocity['sharp_right'])),

    ctrl.Rule(distance['close'] & angle['far_left'], 
              (linear_velocity['slow'], angular_velocity['slight_right'])),

    # MEDIUM distance rules - Preventive turning
    ctrl.Rule(distance['medium'] & angle['far_right'], 
              (linear_velocity['medium'], angular_velocity['straight'])),

    ctrl.Rule(distance['medium'] & angle['right'], 
              (linear_velocity['medium'], angular_velocity['slight_left'])),

    ctrl.Rule(distance['medium'] & angle['slight_right'], 
              (linear_velocity['medium'], angular_velocity['slight_left'])),

    ctrl.Rule(distance['medium'] & angle['straight'], 
              (linear_velocity['medium'], angular_velocity['slight_left'])),

    ctrl.Rule(distance['medium'] & angle['slight_left'], 
              (linear_velocity['medium'], angular_velocity['slight_right'])),

    ctrl.Rule(distance['medium'] & angle['left'], 
              (linear_velocity['medium'], angular_velocity['slight_right'])),

    ctrl.Rule(distance['medium'] & angle['far_left'], 
              (linear_velocity['medium'], angular_velocity['straight'])),

    # FAR distance rules - Gentle adjustment
    ctrl.Rule(distance['far'] & angle['far_right'], 
              (linear_velocity['fast'], angular_velocity['straight'])),

    ctrl.Rule(distance['far'] & angle['right'], 
              (linear_velocity['fast'], angular_velocity['straight'])),

    ctrl.Rule(distance['far'] & angle['slight_right'], 
              (linear_velocity['fast'], angular_velocity['straight'])),

    ctrl.Rule(distance['far'] & angle['straight'], 
              (linear_velocity['fast'], angular_velocity['straight'])),

    ctrl.Rule(distance['far'] & angle['slight_left'], 
              (linear_velocity['fast'], angular_velocity['straight'])),

    ctrl.Rule(distance['far'] & angle['left'], 
              (linear_velocity['fast'], angular_velocity['straight'])),

    ctrl.Rule(distance['far'] & angle['far_left'], 
              (linear_velocity['fast'], angular_velocity['straight'])),
]

# Create the control system and simulation
control_system = ctrl.ControlSystem(rules)
sim = ctrl.ControlSystemSimulation(control_system)

# Function to process obstacle information
def obstacle_avoidance_logic(obstacle_distance, obstacle_angle):
    """
    Given the distance and angle to an obstacle, compute linear and angular velocities using fuzzy logic.
    """
    # Provide inputs to the fuzzy logic system
    sim.input['distance'] = obstacle_distance
    sim.input['angle'] = obstacle_angle

    # Compute the outputs
    sim.compute()

    # Get the velocities
    linear_speed = sim.output['linear_velocity']
    angular_speed = sim.output['angular_velocity']

    vel_msg = Twist()
    vel_msg.linear.x = linear_speed
    vel_msg.angular.z = angular_speed
    pub_vel.publish(vel_msg)

# Callback function to update obstacle pose
def update_obstacle_pose(msg):
    obstacle_distance = msg.position.z * 100  # Convert to cm
    obstacle_angle = msg.orientation.z       # Angle from pose

    rospy.loginfo(f"[fuzzy] Obstacle Detected! Dist: {obstacle_distance / 100:.2f}m, Theta: {np.degrees(obstacle_angle):.2f}°")
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

