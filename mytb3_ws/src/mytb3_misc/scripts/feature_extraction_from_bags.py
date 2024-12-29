import rosbag
import os
import csv
import numpy as np
import tf

# Function to read data from a single ROS bag
def read_rosbag(bag_path):
    bag = rosbag.Bag(bag_path)

    combined_data = []
    robot_position = None
    robot_orientation = None

    for topic, msg, t in bag.read_messages(topics=["/robot_pose_ekf/odom_combined",
                                                   "/mytb3/slam/obstacle",
                                                   "/mytb3/slam/goal",
                                                   "/mytb3/cmd_vel"]):

        if topic == "/robot_pose_ekf/odom_combined":
            # Extract robot position and orientation
            robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            orientation_q = msg.pose.pose.orientation
            _, _, robot_orientation = tf.transformations.euler_from_quaternion(
                [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            )

        if topic == "/mytb3/slam/obstacle" and robot_position is not None:
            # Calculate relative distance and orientation to the obstacle
            rel_distance = msg.position.z
            rel_orientation = msg.orientation.z

        if topic == "/mytb3/slam/goal" and robot_position is not None:
            # Calculate goal error in x and y
            rel_x, rel_y = msg.position.x, msg.position.y
            error_x = rel_x - robot_position[0]
            error_y = rel_y - robot_position[1]

        if topic == "/mytb3/cmd_vel":
            # Extract commanded velocities
            linear_velocity = msg.linear.x
            angular_velocity = msg.angular.z

        # Append all features into a single row
        combined_data.append([
            rel_distance if 'rel_distance' in locals() else None,
            rel_orientation if 'rel_orientation' in locals() else None,
            error_x if 'error_x' in locals() else None,
            error_y if 'error_y' in locals() else None,
            linear_velocity if 'linear_velocity' in locals() else None,
            angular_velocity if 'angular_velocity' in locals() else None
        ])

    bag.close()
    return combined_data

# Save combined data to a single CSV file
def save_to_csv(file_path, data, headers):
    with open(file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(headers)
        writer.writerows(data)

# Main function
def main():
    bag_dir = "/home/ahmar/docs/prj/mytb3/mytb3_ws/src/bags"
    output_csv = "/home/ahmar/docs/prj/mytb3/mytb3_ws/src/mytb3_misc/features.csv"

    combined_data = []

    for case_num in range(1, 11):
        bag_path = f"{bag_dir}/case_{case_num}.bag"
        if not os.path.exists(bag_path):
            print(f"Bag file {bag_path} does not exist. Skipping...")
            continue

        print(f"Processing {bag_path}...")
        case_data = read_rosbag(bag_path)
        combined_data.extend(case_data)

    # Define headers for the CSV file
    headers = [
        "Relative Obstacle Distance",
        "Relative Obstacle Orientation",
        "Goal Error X",
        "Goal Error Y",
        "Linear Velocity",
        "Angular Velocity"
    ]

    # Save the combined data to a CSV file
    save_to_csv(output_csv, combined_data, headers)
    print(f"Data saved to {output_csv}")

if __name__ == "__main__":
    main()
