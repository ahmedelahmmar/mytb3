#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

# Increment values for linear and angular velocities
LINEAR_INCREMENT = 0.1  # Increment for linear velocity (m/s)
ANGULAR_INCREMENT = 0.6  # Increment for angular velocity (rad/s)

# Velocity limits
LINEAR_MAX = 0.5  # Maximum linear velocity (m/s)
LINEAR_MIN = -LINEAR_MAX # Minimum linear velocity (m/s)
ANGULAR_MAX = 5.9  # Maximum angular velocity (rad/s)
ANGULAR_MIN = -ANGULAR_MAX  # Minimum angular velocity (rad/s)

# Define the key mappings
key_mappings = {
    'w': 'linear_increase',   # Increment linear velocity
    's': 'linear_decrease',   # Decrement linear velocity
    'a': 'angular_increase',  # Increment angular velocity
    'd': 'angular_decrease',  # Decrement angular velocity
    'x': 'stop'               # Stop the robot
}



def get_key():
    """
    Get a single keypress from the terminal.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key



def clamp(value, min_value, max_value):
    """
    Clamp a value between a minimum and maximum.
    """
    return max(min(value, max_value), min_value)



def main():
    # Initialize the ROS node
    rospy.init_node('mytb3_teleop', anonymous=True)

    pub = rospy.Publisher('/mytb3/cmd_vel', Twist, queue_size=10)

    # Twist message to publish
    twist = Twist()
    linear_velocity = 0.0
    angular_velocity = 0.0

    print("Use the following keys to control the robot:")
    print("  W: Increment linear velocity")
    print("  S: Decrement linear velocity")
    print("  A: Increment angular velocity")
    print("  D: Decrement angular velocity")
    print("  X: Stop")
    print("Press CTRL+C to exit.")

    try:
        while not rospy.is_shutdown():
            key = get_key()

            if key in key_mappings:
                command = key_mappings[key]
                if command == 'linear_increase':
                    linear_velocity += LINEAR_INCREMENT
                    linear_velocity = clamp(linear_velocity, LINEAR_MIN, LINEAR_MAX)
                elif command == 'linear_decrease':
                    linear_velocity -= LINEAR_INCREMENT
                    linear_velocity = clamp(linear_velocity, LINEAR_MIN, LINEAR_MAX)
                elif command == 'angular_increase':
                    angular_velocity += ANGULAR_INCREMENT
                    angular_velocity = clamp(angular_velocity, ANGULAR_MIN, ANGULAR_MAX)
                elif command == 'angular_decrease':
                    angular_velocity -= ANGULAR_INCREMENT
                    angular_velocity = clamp(angular_velocity, ANGULAR_MIN, ANGULAR_MAX)
                elif command == 'stop':
                    linear_velocity = 0.0
                    angular_velocity = 0.0

                # Update Twist message
                twist.linear.x = linear_velocity
                twist.angular.z = angular_velocity

                # Log the current velocities
                print(f"Linear Velocity: {twist.linear.x:.2f}, Angular Velocity: {twist.angular.z:.2f}")

            elif key == '\x03':  # CTRL+C to quit
                break
            else:
                print("Unknown key! Use W/S/A/D/X.")

            # Publish the Twist message
            pub.publish(twist)
    except rospy.ROSInterruptException:
        pass
    finally:
        # Stop the robot on exit
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

if __name__ == '__main__':
    main()


#!/usr/bin/env python3

# import rospy
# from geometry_msgs.msg import Twist
# import sys
# import termios
# import tty

# # Define velocities for each direction
# FORWARD_VELOCITY = 1.0   # Linear velocity for forward movement (m/s)
# BACKWARD_VELOCITY = -1.0 # Linear velocity for backward movement (m/s)
# TURN_VELOCITY = 1.0      # Angular velocity for turning (rad/s)

# # Define the key mappings for directions
# key_mappings = {
#     'w': 'forward',    # Move forward
#     's': 'backward',   # Move backward
#     'a': 'left',       # Turn left
#     'd': 'right',      # Turn right
#     'x': 'stop'        # Stop the robot
# }

# def get_key():
#     """
#     Get a single keypress from the terminal.
#     """
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     try:
#         tty.setraw(fd)
#         key = sys.stdin.read(1)
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#     return key

# def main():
#     # Initialize the ROS node
#     rospy.init_node('teleop_direction', anonymous=True)

#     # Publisher for the /pc_twist_to_tb3 topic
#     pub = rospy.Publisher('/mytb3/cmd_vel', Twist, queue_size=10)

#     # Twist message to publish
#     twist = Twist()

#     print("Use the following keys to control the robot:")
#     print("  W: Move forward")
#     print("  S: Move backward")
#     print("  A: Turn left")
#     print("  D: Turn right")
#     print("  X: Stop")
#     print("Press CTRL+C to exit.")

#     try:
#         while not rospy.is_shutdown():
#             key = get_key()

#             if key in key_mappings:
#                 command = key_mappings[key]

#                 # Update Twist message based on the command
#                 if command == 'forward':
#                     twist.linear.x = FORWARD_VELOCITY
#                     twist.angular.z = 0.0
#                 elif command == 'backward':
#                     twist.linear.x = BACKWARD_VELOCITY
#                     twist.angular.z = 0.0
#                 elif command == 'left':
#                     twist.linear.x = 0.0
#                     twist.angular.z = TURN_VELOCITY
#                 elif command == 'right':
#                     twist.linear.x = 0.0
#                     twist.angular.z = -TURN_VELOCITY
#                 elif command == 'stop':
#                     twist.linear.x = 0.0
#                     twist.angular.z = 0.0

#                 # Log the command
#                 print(f"Command: {command} | Linear: {twist.linear.x:.2f}, Angular: {twist.angular.z:.2f}")

#                 # Publish the Twist message
#                 pub.publish(twist)

#             elif key == '\x03':  # CTRL+C to quit
#                 break
#             else:
#                 print("Unknown key! Use W/S/A/D/X.")

#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         # Stop the robot on exit
#         twist.linear.x = 0.0
#         twist.angular.z = 0.0
#         pub.publish(twist)

# if __name__ == '__main__':
#     main()
