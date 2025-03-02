#!/usr/bin/env python3

import joblib
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist

class RFNavigator:
    def __init__(self):
        """Initialize the Random Forest Navigator"""
        # Load models and scalers
        self.load_models()
        
        # Initialize state variables
        self.robot_pose = None
        self.goal_pose = None
        self.obstacle_pose = None
        self.last_obstacle_time = None
        
        # Constants
        self.OBSTACLE_TIMEOUT = rospy.Duration(1.0)
        self.DEFAULT_LINEAR_VEL = 0.2
        self.DEFAULT_ANGULAR_VEL = 0.0
        self.MAX_LINEAR_VEL = 0.5
        self.MAX_ANGULAR_VEL = 5.9
        self.UPDATE_RATE = 10  # Hz
        
        # Initialize ROS node and publishers/subscribers
        self.setup_ros()

    def load_models(self):
        """Load the trained model and scaler"""
        try:
            self.model = joblib.load('/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/rf_model_yolo.pkl')
            self.scaler = joblib.load('/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_nav/models/rf_scaler_yolo.pkl')
            rospy.loginfo("Models loaded successfully")
        except Exception as e:
            rospy.logerr(f"Error loading models: {str(e)}")
            raise

    def setup_ros(self):
        """Initialize ROS node and publishers/subscribers"""
        rospy.init_node('ai_nav_node')
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/mytb3/cmd_vel', Twist, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/mytb3/slam/goal', Pose, self.goal_callback, queue_size=10)
        rospy.Subscriber('/mytb3/slam/obstacle', Pose, self.obstacle_callback, queue_size=10)
        rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, 
                        self.odom_callback, queue_size=10)
        
        rospy.loginfo("ROS node initialized")

    def odom_callback(self, msg):
        """Handle odometry updates"""
        self.robot_pose = msg.pose.pose

    def goal_callback(self, msg):
        """Handle goal updates"""
        self.goal_pose = msg

    def obstacle_callback(self, msg):
        """Handle obstacle updates"""
        self.obstacle_pose = msg
        self.last_obstacle_time = rospy.Time.now()

    def is_obstacle_valid(self):
        """Check if the obstacle detection is recent"""
        if self.last_obstacle_time is None:
            return False
        return (rospy.Time.now() - self.last_obstacle_time) < self.OBSTACLE_TIMEOUT

    def get_default_obstacle_values(self):
        """Return safe default values when no obstacle is detected"""
        return 10.0, 0.0  # Large distance and neutral orientation

    def prepare_features(self):
        """Prepare feature vector for prediction"""
        try:
            if None in (self.robot_pose, self.goal_pose):
                return None

            # Calculate goal errors
            goal_error_x = self.goal_pose.position.x
            goal_error_y = self.goal_pose.position.y

            # Get obstacle information
            if self.is_obstacle_valid() and self.obstacle_pose is not None:
                relative_obstacle_distance = self.obstacle_pose.position.z
                relative_obstacle_orientation = self.obstacle_pose.orientation.z
            else:
                relative_obstacle_distance, relative_obstacle_orientation = self.get_default_obstacle_values()

            features = np.array([[
                relative_obstacle_distance,
                relative_obstacle_orientation,
                goal_error_x,
                goal_error_y
            ]])

            return features

        except Exception as e:
            rospy.logerr(f"Error preparing features: {str(e)}")
            return None

    def predict_velocities(self, features):
        """Make velocity predictions using the model"""
        try:
            # Scale features
            features_scaled = self.scaler.transform(features)
            
            # Make prediction
            predictions = self.model.predict(features_scaled)
            
            # Extract and limit velocities
            linear_vel = np.clip(predictions[0][0], 0.0, self.MAX_LINEAR_VEL)
            angular_vel = np.clip(predictions[0][1], -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL)
            
            return linear_vel, angular_vel
            
        except Exception as e:
            rospy.logerr(f"Error in prediction: {str(e)}")
            return self.DEFAULT_LINEAR_VEL, self.DEFAULT_ANGULAR_VEL

    def calculate_velocities(self):
        """Calculate velocities based on current state"""
        features = self.prepare_features()
        
        if features is None:
            rospy.logwarn_throttle(1, "Missing data for prediction. Using default velocities.")
            return self.DEFAULT_LINEAR_VEL, self.DEFAULT_ANGULAR_VEL
            
        return self.predict_velocities(features)

    def run(self):
        """Main run loop"""
        rate = rospy.Rate(self.UPDATE_RATE)
        cmd_vel = Twist()

        while not rospy.is_shutdown():
            try:
                # Calculate velocities
                linear_vel, angular_vel = self.calculate_velocities()
                
                # Update command velocities
                cmd_vel.linear.x = linear_vel
                cmd_vel.angular.z = angular_vel
                
                # Publish velocities
                self.cmd_vel_pub.publish(cmd_vel)
                
                # Debug information
                if rospy.get_param('~debug', False):
                    rospy.loginfo(f"Velocities - Linear: {linear_vel:.2f}, Angular: {angular_vel:.2f}")
                    
            except Exception as e:
                rospy.logerr(f"Error in main loop: {str(e)}")
                # Stop the robot
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel)

            rate.sleep()

if __name__ == '__main__':
    try:
        navigator = RFNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass