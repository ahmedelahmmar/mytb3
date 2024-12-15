#!/usr/bin/env python3

import rospy, geometry_msgs.msg
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
import tf2_ros


# Function to broadcast the transform from imu_link to base_footprint
def broadcast_transform():
    # Define the transform broadcaster
    br = tf2_ros.TransformBroadcaster()

    # Define the static transform
    transform = geometry_msgs.msg.TransformStamped()
    transform.header.frame_id = "base_footprint"  # Parent frame
    transform.child_frame_id = "imu_link"         # Child frame

    # Translation: IMU's position relative to base_footprint
    transform.transform.translation.x = 0.0  
    transform.transform.translation.y = 0.0  
    transform.transform.translation.z = 0.15 

    # Rotation: No rotation relative to base_footprint
    quat = quaternion_from_euler(0, 0, 0)  # Roll, pitch, yaw = 0
    transform.transform.rotation.x = quat[0]
    transform.transform.rotation.y = quat[1]
    transform.transform.rotation.z = quat[2]
    transform.transform.rotation.w = quat[3]

    # Publish the transform at 10 Hz
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        transform.header.stamp = rospy.Time.now()
        br.sendTransform(transform)
        rate.sleep()


# Callback function for /mytb3/raw_odom_data
def odom_data_callback(msg):
    # Publishers
    global pub_odom, pub_imu_data

    # Prepare Odometry message
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"

    # Set position and orientation (theta converted to quaternion)
    odom_msg.pose.pose.position.x = msg.position.x
    odom_msg.pose.pose.position.y = msg.position.y
    odom_msg.pose.pose.position.z = 0.0

    quat = quaternion_from_euler(0, 0, msg.position.z)  # Convert theta to quaternion
    odom_msg.pose.pose.orientation.x = quat[0]
    odom_msg.pose.pose.orientation.y = quat[1]
    odom_msg.pose.pose.orientation.z = quat[2]
    odom_msg.pose.pose.orientation.w = quat[3]

    # Pose covariance
    odom_msg.pose.covariance = [
        0.25, 0,   0,   0,  0,  0,
        0,   0.25, 0,   0,  0,  0,
        0,   0,   -1, 0,  0,  0,
        0,   0,   0,  -1,  0,  0,
        0,   0,   0,   0, -1,  0,
        0,   0,   0,   0,  0,  0.75
    ]

    # Twist (linear and angular velocities set to 0)
    odom_msg.twist.twist.linear.x = 0.0
    odom_msg.twist.twist.linear.y = 0.0
    odom_msg.twist.twist.angular.z = 0.0
    odom_msg.twist.covariance = [
        -1,  0,  0,  0,  0,  0,
         0, -1,  0,  0,  0,  0,
         0,  0, -1,  0,  0,  0,
         0,  0,  0, -1,  0,  0,
         0,  0,  0,  0, -1,  0,
         0,  0,  0,  0,  0, -1
    ]

    # Publish Odometry message
    pub_odom.publish(odom_msg)

    # Prepare IMU message
    imu_msg = Imu()
    imu_msg.header.stamp = odom_msg.header.stamp
    imu_msg.header.frame_id = "imu_link"

    # Set linear acceleration
    imu_msg.linear_acceleration.x = msg.orientation.x
    imu_msg.linear_acceleration.y = msg.orientation.y
    imu_msg.linear_acceleration.z = 0.0
    imu_msg.linear_acceleration_covariance = [
        0.25, 0,   0,
        0,   0.25, 0,
        0,   0,  -1
    ]

    # Set angular velocity
    imu_msg.angular_velocity.x = 0.0
    imu_msg.angular_velocity.y = 0.0
    imu_msg.angular_velocity.z = msg.orientation.w
    imu_msg.angular_velocity_covariance = [
        -1,  0, 0,
         0, -1, 0,
         0,  0, 0.25
    ]

    # Set orientation
    quat = quaternion_from_euler(0, 0, msg.orientation.z)  # Convert theta to quaternion
    imu_msg.orientation.x = quat[0]
    imu_msg.orientation.y = quat[1]
    imu_msg.orientation.z = quat[2]
    imu_msg.orientation.w = quat[3]
    imu_msg.orientation_covariance = [
        -1,  0, 0,
         0, -1, 0,
         0,  0, 0.01
    ]

    # Publish IMU message
    pub_imu_data.publish(imu_msg)


# Main function
def main():
    global pub_odom, pub_imu_data

    rospy.init_node("odom_bridge", anonymous=True)

    # Subscriber to /mytb3/raw_odom_data
    rospy.Subscriber("/mytb3/raw_odom_data", Pose, odom_data_callback)

    # Publishers for /odom and /imu_data
    pub_odom = rospy.Publisher("/odom", Odometry, queue_size=10)
    pub_imu_data = rospy.Publisher("/imu_data", Imu, queue_size=10)

    # Start the transform broadcaster in a separate thread
    rospy.Timer(rospy.Duration(0.1), lambda event: broadcast_transform())

    rospy.spin()


if __name__ == "__main__":
    main()
