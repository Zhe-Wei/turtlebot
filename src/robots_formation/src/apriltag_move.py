#!/usr/bin/env python3
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
import sys
import threading
import math
from geometry_msgs.msg import Twist
import tf.transformations as tf

# Lists to store the relative positions
relative_positions_01 = None
relative_positions_02 = None

class RobotController:
    def __init__(self, record_duration=5.0):
        # Initialize ROS node
        rospy.init_node('robot_controller')

        # Subscriber and Publisher
        self.subscriber = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_callback)
        self.vel_pub_01 = rospy.Publisher('/slave_1/cmd_vel', Twist, queue_size=10)
        self.vel_pub_02 = rospy.Publisher('/slave_2/cmd_vel', Twist, queue_size=10)

        # Tracking parameters
        self.k_v = 1 # Tunable parameter
        self.k_w = 0.3 # Tunable parameter
        self.k_yaw = 0.5 # Tunable parameter

        # 初始化錄製標記
        self.initial_positions_recorded = False

        # Variables for position tracking
        self.saved_relative_position = False
        self.init_relative_position_01 = np.array([0, 0, 0])
        self.init_relative_position_02 = np.array([0, 0, 0])

        # Record initial positions for a duration
        self.record_duration = record_duration
        self.recording_thread = threading.Thread(target=self.record_initial_positions)
        self.recording_thread.start()

        # Maximum velocity limit
        self.max_vel_x = 1.0
        self.min_vel_x = 0.05
        self.max_vel_theta = 1.0
        self.min_vel_theta = 0.05

        # Register a ROS shutdown hook
        rospy.on_shutdown(self.stop_robot)

    def record_initial_positions(self):
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < self.record_duration:
            if not self.initial_positions_recorded:
                # 這裡可以添加一些條件，以確保已經收到了有效的AprilTag數據
                if relative_positions_01 is not None and relative_positions_02 is not None:
                    self.init_relative_position_01 = relative_positions_01.copy()
                    self.init_relative_position_02 = relative_positions_02.copy()
                    self.initial_positions_recorded = True
            rospy.sleep(0.1)
    
    def quaternion_to_euler(self, quaternion):
        """
        Convert a quaternion to Euler angles (roll, pitch, yaw)
        """
        euler = tf.euler_from_quaternion([
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w
        ])
        return euler  # Returns a tuple (roll, pitch, yaw)
    
    def tag_callback(self, data):
        # Process tag detections
        global relative_positions_01, relative_positions_02
        poses = {detection.id[0]: detection.pose.pose.pose for detection in data.detections if detection.id[0] in [0, 1, 2]}
        
        if 0 in poses and 1 in poses and 2 in poses:
            # Calculate relative positions in x and y
            relative_x_01 = poses[1].position.y - poses[0].position.y
            relative_y_01 = poses[1].position.x - poses[0].position.x
            relative_x_02 = poses[2].position.y - poses[0].position.y
            relative_y_02 = poses[2].position.x - poses[0].position.x

            # Calculate relative yaw (orientation difference)
            euler_0 = self.quaternion_to_euler(poses[0].orientation)
            euler_1 = self.quaternion_to_euler(poses[1].orientation)
            euler_2 = self.quaternion_to_euler(poses[2].orientation)

            yaw_diff_01 = euler_1[2] - euler_0[2]
            yaw_diff_02 = euler_2[2] - euler_0[2]

            # Combine position and orientation differences
            relative_positions_01 = np.array([relative_x_01, relative_y_01, yaw_diff_01])
            relative_positions_02 = np.array([relative_x_02, relative_y_02, yaw_diff_02])

            # print('Relative position and orientation of tag_1 w.r.t tag_0: {}'.format(relative_positions_01))
            # print('Relative position and orientation of tag_2 w.r.t tag_0: {}'.format(relative_positions_02))


    def wait_for_user_input(self):
        input("Press any key to save the initial relative position...")
        # Save the initial relative position
        global relative_positions_01, relative_positions_02
        self.init_relative_position_01 = relative_positions_01
        self.init_relative_position_02 = relative_positions_02
        self.saved_relative_position = True

    def calculate_velocity(self):
        vel_msg_01 = Twist()
        vel_msg_02 = Twist()

        # Calculate errors for tag 1 and tag 2
        error_01 = relative_positions_01 - self.init_relative_position_01
        error_02 = relative_positions_02 - self.init_relative_position_02

        # Check if position error is within 5 cm for tag 1
        if np.linalg.norm(error_01[:2]) <= 0.05:
            vel_msg_01.linear.x = 0
            # Use only yaw error to adjust angular.z
            vel_msg_01.angular.z = min(max(self.k_yaw * error_01[2], -self.max_vel_theta), self.max_vel_theta)
        else:
            vel_msg_01.linear.x = min(max(self.k_v * error_01[0], -self.max_vel_x), self.max_vel_x)
            # Use error_y to adjust angular.z
            vel_msg_01.angular.z = min(max(self.k_w * error_01[1], -self.max_vel_theta), self.max_vel_theta)

        # Check if position error is within 5 cm for tag 2
        if np.linalg.norm(error_02[:2]) <= 0.05:
            vel_msg_02.linear.x = 0
            # Use only yaw error to adjust angular.z
            vel_msg_02.angular.z = min(max(self.k_yaw * error_02[2], -self.max_vel_theta), self.max_vel_theta)
        else:
            vel_msg_02.linear.x = min(max(self.k_v * error_02[0], -self.max_vel_x), self.max_vel_x)
            # Use error_y to adjust angular.z
            vel_msg_02.angular.z = min(max(self.k_w * error_02[1], -self.max_vel_theta), self.max_vel_theta)

        # Print errors and velocities
        print(f'error_01: {np.round(error_01, 2)} vel x: {np.round(vel_msg_01.linear.x, 2)} vel z: {np.round(vel_msg_01.angular.z, 2)}')
        print(f'error_02: {np.round(error_02, 2)} vel x: {np.round(vel_msg_02.linear.x, 2)} vel z: {np.round(vel_msg_02.angular.z, 2)}')

        return vel_msg_01, vel_msg_02
    
    def run(self):
        # Wait for the initial recording to complete
        self.recording_thread.join()

        # Wait for user input to save the relative position
        self.wait_for_user_input()

        # Start tracking
        while not rospy.is_shutdown():
            if self.saved_relative_position:
                velocity_command_01, velocity_command_02 = self.calculate_velocity()
                # Publish velocities for each tag
                self.vel_pub_01.publish(velocity_command_01)
                self.vel_pub_02.publish(velocity_command_02)

            rospy.sleep(0.1)

    def stop_robot(self):
        # Create a zero velocity message
        stop_msg = Twist()
        stop_msg.linear.x = 0
        stop_msg.angular.z = 0

        # Publish the stop message
        self.vel_pub_01.publish(stop_msg)
        self.vel_pub_02.publish(stop_msg)

        rospy.loginfo("Robot has been stopped due to shutdown.")

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
