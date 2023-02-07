#!/usr/bin/python3

import rospy
import random
import numpy as np

from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
import message_filters
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Point, Quaternion, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, translation_from_matrix, quaternion_from_matrix, compose_matrix, quaternion_matrix, rotation_matrix

class Visualiser:
    def __init__(self):
        self.fig = plt.figure(figsize=(6,6))
        self.line2, = plt.plot([], [], 'bo', markersize=1)
        self.line1, = plt.plot([], [], 'rx', markersize=2)
        self.x_data1, self.y_data1 = [] , []
        self.x_data2, self.y_data2 = [] , []
        self.est_x, self.est_y = 0,0
        self.tmp_x, self.tmp_y = 0,0
        cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        
    def onclick(self, event):
        # print('button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
        #   (event.button, event.x, event.y, event.xdata, event.ydata))
        print("clear data")
        self.x_data1, self.y_data1 = [] , []
        self.x_data2, self.y_data2 = [] , []
    def plot_init(self):
        # plt.xlim(-200, 30)
        # plt.ylim(-200, 30)
        plt.xlim(-200, 50)
        plt.ylim(-200, 50)
        return self.line2

    # def getYaw(self, pose):
    #     quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
    #             pose.orientation.w)
    #     euler = tf.transformations.euler_from_quaternion(quaternion)
    #     yaw = euler[2] 
    #     return yaw

    def odom_callback(self, robot_data):
       
        # print(robot_data.pose.pose.position.x*100, robot_data.pose.pose.position.y*100)
        if self.est_x==0 and self.est_y==0:
            self.est_x = robot_data.pose.pose.position.x*100
            self.est_y = robot_data.pose.pose.position.y*100
            self.tmp_x = self.est_x
            self.tmp_y = self.est_y
        else:
            if (self.tmp_x == robot_data.pose.pose.position.x) and (self.tmp_y == robot_data.pose.pose.position.y):
                return

            if abs(self.est_x - robot_data.pose.pose.position.x*100) > 25 or abs(self.est_y - robot_data.pose.pose.position.y*100) > 25 :
                factor = 0.01
            else:
                factor = 0.15
                self.est_x = factor * (robot_data.pose.pose.position.x*100) + (1-factor) * self.est_x
                self.est_y = factor * (robot_data.pose.pose.position.y*100) + (1-factor) * self.est_y

            self.tmp_x = robot_data.pose.pose.position.x
            self.tmp_y = robot_data.pose.pose.position.y

            self.x_data1.append(self.est_x)
            self.y_data1.append(self.est_y)
            



        # print(desktop_data.detections[0].pose.pose.pose.position.x*100, desktop_data.detections[0].pose.pose.pose.position.y*100)

        # self.x_data2.append(desktop_data.detections[0].pose.pose.pose.position.x*100)
        # self.y_data2.append(desktop_data.detections[0].pose.pose.pose.position.y*100)

        # self.x_data2.append(robot_data.pose.pose.position.x*100)
        # self.y_data2.append(robot_data.pose.pose.position.y*100)

    def update_plot(self, frame):
        self.line2.set_data(self.x_data2, self.y_data2)
        self.line1.set_data(self.x_data1, self.y_data1)
        return self.line2

if __name__ == '__main__':
    rospy.init_node('draw_xy')
    rate = rospy.Rate(10) # 10hz or 100 ms
    vis = Visualiser()
    # while not rospy.is_shutdown():

    # subsribe turlbot odometry
    rospy.Subscriber('turtlebot02/map2odom', PoseWithCovarianceStamped, vis.odom_callback)

    # robot = message_filters.Subscriber("turtlebot02/map2odom", PoseWithCovarianceStamped)
    # desktop_camera = message_filters.Subscriber("/desktop_detection", AprilTagDetectionArray)
    # time_sync = message_filters.ApproximateTimeSynchronizer([robot, desktop_camera], 10, 0.1, allow_headerless=True)
    # time_sync.registerCallback(vis.odom_callback)


    ani =animation.FuncAnimation(fig=vis.fig, func=vis.update_plot, init_func=vis.plot_init)
    plt.show()
        # rate.sleep()
