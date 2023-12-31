#!/usr/bin/env python3

import rospy
import tf
from tf import transformations
from geometry_msgs.msg import Twist

import math
import numpy as np

class RobotController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('robot_controller')

        # Subscriber and Publisher
        self.vel_pub_01 = rospy.Publisher('/slave_1/cmd_vel', Twist, queue_size=10)
        self.vel_pub_02 = rospy.Publisher('/slave_2/cmd_vel', Twist, queue_size=10)

        # TF listener
        self.tf_listener = tf.TransformListener()

        # Tracking parameters
        self.k_v = 1 # Tunable parameter
        self.k_w = 0.5 # Tunable parameter
        self.k_yaw = 0.4 # Tunable parameter

        # Relative error storage
        self.trans_error_01 = None
        self.trans_error_02 = None

        # Relative rotation storage
        self.rot_error_01 = None
        self.rot_error_02 = None

        # Speed limits
        self.max_vel_x = 0.20 # 最大線速度，可根據需要調整
        self.max_vel_theta = 0.4 # 最大角速度，可根據需要調整
        self.min_vel_theta = 0.01

        rospy.on_shutdown(self.stop_robots)

    def stop_robots(self):
        rospy.loginfo("Shutting down: Stopping all follower robots")
        stop_msg = Twist()  # 創建一個空的速度消息
        self.vel_pub_01.publish(stop_msg)  # 將速度設為 0
        self.vel_pub_02.publish(stop_msg)  # 將速度設為 0

        ## 警告
        rospy.logwarn("All follower robots stopped")
        rospy.sleep(0.5)


    def compute_relative_error(self):
        try:
            # Wait for the transforms to become available
            # self.tf_listener.waitForTransform('/tag_0', '/tag_1', rospy.Time(0), rospy.Duration(4.0))
            # self.tf_listener.waitForTransform('/tag_0', '/tag_2', rospy.Time(0), rospy.Duration(4.0))

            # Get the transforms
            (trans_01, rot_01) = self.tf_listener.lookupTransform('/tag_1', '/tag_0/tag_1', rospy.Time(0))
            (trans_02, rot_02) = self.tf_listener.lookupTransform('/tag_2', '/tag_0/tag_2', rospy.Time(0))
            
            #quaternion to euler
            euler_01 = transformations.euler_from_quaternion(rot_01)
            euler_02 = transformations.euler_from_quaternion(rot_02)

            # Compute relative errors (you can modify the error computation as needed)
            self.trans_error_01 = trans_01
            self.trans_error_02 = trans_02
            self.rot_error_01 = euler_01
            self.rot_error_02 = euler_02

            # Print or log the errors, decimal 2 places
            rospy.loginfo(f'Relative Err01: {np.round(self.trans_error_01, 3), np.round(self.rot_error_01, 3)}') 
            rospy.loginfo(f'Relative Err02: {np.round(self.trans_error_02, 3), np.round(self.rot_error_02, 3)}') 

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f'Transform error: {e}')
    

    def apply_speed_limits(self, vel_msg):
        # 限制線速度
        if vel_msg.linear.x > self.max_vel_x:
            vel_msg.linear.x = self.max_vel_x
        elif vel_msg.linear.x < -self.max_vel_x:
            vel_msg.linear.x = -self.max_vel_x
        if vel_msg.linear.x < 0:
            vel_msg.linear.x = 0

        # 限制角速度
        if vel_msg.angular.z > self.max_vel_theta:
            vel_msg.angular.z = self.max_vel_theta
        elif vel_msg.angular.z < -self.max_vel_theta:
            vel_msg.angular.z = -self.max_vel_theta

        if abs(vel_msg.angular.z) < self.min_vel_theta:  # Check minimum angular speed
            vel_msg.angular.z = 0


    def calculate_velocity_command(self, trans_error, rot_error):
        vel_msg = Twist()
        if trans_error:

            error_distance = math.sqrt(trans_error[0]**2 + trans_error[1]**2)
            if error_distance < 0.05:
                vel_msg.linear.x = 0
                vel_msg.angular.z = 1.5 * math.sin(rot_error[2])
                return vel_msg
            else:
                k_y = 0.5

                angle_to_target = math.atan2(trans_error[1], trans_error[0])
                
                # vel_msg.angular.z = 0.9 * trans_error[1] + 0.4 * math.sin(rot_error[2]) ## good
                vel_msg.angular.z = 1 * trans_error[1] + 0.5 * math.sin(rot_error[2])

                # if  trans_error[1] < 0.5 or trans_error[1] < 0.8 :
                #     vel_msg.angular.z = 0
                
                # vel_msg.angular.z = 2* math.sin(rot_error[2])
                # vel_msg.angular.z = 1 * trans_error[1]
                # + k_y * error_distance_y
                # vel_msg.angular.z = self.k_w 
                vel_msg.linear.x = self.k_v * trans_error[0]

        # 應用速度限制
        self.apply_speed_limits(vel_msg)
        return vel_msg

    def compute_velocity_commands(self):
        vel_msg_01 = self.calculate_velocity_command(self.trans_error_01, self.rot_error_01)
        vel_msg_02 = self.calculate_velocity_command(self.trans_error_02, self.rot_error_02)

        vel_msg_02.angular.z = vel_msg_02.angular.z + 0.01  # fine tune
        vel_msg_02.angular.z = vel_msg_02.angular.z - 0.01  # fine tune

        # Print or log the linear, angular velocities, with 2 decimal places
        rospy.loginfo(f'01: Linerar Velocity : {np.round(vel_msg_01.linear.x, 2)}, Angular Velocity: {np.round(vel_msg_01.angular.z, 2)}')
        rospy.loginfo(f'02: Linerar Velocity : {np.round(vel_msg_02.linear.x, 2)}, Angular Velocity: {np.round(vel_msg_02.angular.z, 2)}')

        return vel_msg_01, vel_msg_02


    def send_velocity_commands(self):
        vel_msg_01, vel_msg_02 = self.compute_velocity_commands()

        # 發送命令
        self.vel_pub_01.publish(vel_msg_01)
        self.vel_pub_02.publish(vel_msg_02)


if __name__ == '__main__':
    controller = RobotController()
    rate = rospy.Rate(10) # 10 Hz

    vel_msg_01 = Twist()
    vel_msg_02 = Twist()

    while not rospy.is_shutdown():
        controller.compute_relative_error()
        controller.send_velocity_commands() 
        rate.sleep()
