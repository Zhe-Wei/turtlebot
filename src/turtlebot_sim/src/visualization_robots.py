#!/usr/bin/python3

import rospy
import numpy as np
import math
import matplotlib.pyplot as plt

import tf
import re
# import message_filters
# import matplotlib.animation as animation
# from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Point, Quaternion, PoseWithCovarianceStamped
# from tf.transformations import quaternion_from_euler, euler_from_quaternion, translation_from_matrix, quaternion_from_matrix, compose_matrix, quaternion_matrix, rotation_matrix

tf_listeners = []
current_time = 0

class TFListenerNode(object):

    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.listener = tf.TransformListener()
        self.x_list = []
        self.y_list = []
        self.z_list = []
        self.orientation_list = []
        

    def update_coordinate_plot(self, ax1):
        try:
            (trans, rot) = self.listener.lookupTransform('map', self.robot_name+'/base_footprint', rospy.Time(0))
            self.x_list.append(trans[0])
            self.y_list.append(trans[1])
            self.z_list.append(trans[2])
            self.orientation_list.append(tf.transformations.euler_from_quaternion(rot)[2])
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        ax1.scatter(self.x_list, self.y_list, label=self.robot_name)
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_title('Robot Coordinates')
        ax1.legend()

        # Dynamically adjust the x and y axis ranges based on the data
        ax1.set_xlim([min(self.x_list)-10, max(self.x_list)+10])
        ax1.set_ylim([min(self.y_list)-10, max(self.y_list)+10])
        # ax1.relim()
        # ax1.autoscale_view()

        # Draw arrows to represent robot orientations
        x = self.x_list[-1]
        y = self.y_list[-1]
        orientation = self.orientation_list[-1]
        arrow_length = 0.5
        ax1.arrow(x, y, arrow_length*np.cos(orientation), arrow_length*np.sin(orientation), head_width=0.2, head_length=0.1, fc='k', ec='k')

    # def get_translation(self):
    #     try:
    #         (trans, rot) = self.listener.lookupTransform('world', self.robot_name+'/base_footprint', rospy.Time(0))
    #         return trans
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         return None

class Show_Master2Slave_Error(object):
    y_min = -0.5
    y_max = 0.5

    def __init__(self, tf_1, tf_2, slave_robot_name):
        self.tf_listener_1 = tf_1
        self.tf_listener_2 = tf_2
        self.error_distance_list = []
        self.time_list = []
        self.slave_robot_name = slave_robot_name
        self.listener = tf.TransformListener()
        
        
    def updateErrorPlot(self, ax3, master_robot_name, slave_robot_name):
        # try:

        # Get the current time
        self.time_list.append(rospy.Time.now().to_sec() - start_time)

        # ax3.cla()
        ax3.set_xlabel('Time')
        ax3.set_ylabel('Distance')
        ax3.set_title('Error Distance')

        ax3.legend()

        # Calculate the error distance
        # try:
        frame_1 = master_robot_name + '/' + slave_robot_name
        frame_2 = slave_robot_name + '/base_footprint'
        (trans, rot) = self.listener.lookupTransform(frame_1, frame_2, rospy.Time(0))
        # print(trans)
        error_dis = math.fabs(np.sqrt((trans[0])**2 + (trans[1])**2))
        # error_dis = math.fabs(np.sqrt((self.tf_listener_1.x_list[-1]-self.tf_listener_2.x_list[-1])**2 + (self.tf_listener_1.y_list[-1]-self.tf_listener_2.y_list[-1])**2))
        self.error_distance_list.append(error_dis)

        # Dynamically adjust the x and y axis ranges based on the data
        self.__class__.y_min = min(min(self.error_distance_list), self.__class__.y_min)
        self.__class__.y_max = max(max(self.error_distance_list), self.__class__.y_max)
        
        ax3.set_ylim(self.__class__.y_min-0.5, self.__class__.y_max+0.5)
        # print(error_dis)
        ax3.plot(self.time_list, self.error_distance_list, label=slave_robot_name, markersize=1, linewidth=0.5, linestyle='solid')
        # except:
        #     print("updateErrorPlot -> No data")
        #     return
        

def main():
    rospy.init_node('tf_listener')

    # Get the robot name from the launch file
    master_robot_name = rospy.get_param('~master_robot_name', 'robot_0')
    slave_robot_name_1 = rospy.get_param('~slave_robot_name', 'robot_1')
    slave_robot_name_2 = rospy.get_param('~slave_robot_name', 'robot_2')

    fig = plt.figure(figsize=(12, 5))
    ax1 = fig.add_subplot(1, 3, 1)
    ax2 = fig.add_subplot(1, 3, 2)
    ax3 = fig.add_subplot(1, 3, 3)

    # Create a TFListenerNode for each robot
    tf_listener_1 = TFListenerNode(master_robot_name)
    tf_listener_2 = TFListenerNode(slave_robot_name_1)
    tf_listener_3 = TFListenerNode(slave_robot_name_2)

    # Create a Show_Master2Slave_Error for each robot
    Show_Master2Slave_Error.y_max = 0.5
    Show_Master2Slave_Error.y_min = -0.5
    distance_error_1 = Show_Master2Slave_Error(tf_listener_1, tf_listener_2, slave_robot_name_1)
    distance_error_2 = Show_Master2Slave_Error(tf_listener_1, tf_listener_3, slave_robot_name_2)

    distance_list = []
    time_list = []

    global start_time
    start_time = rospy.Time.now().to_sec()
       

    def update_plots():
        # Clear the plots
        ax1.clear()
        ax2.clear()
        ax3.clear()

        # Update the coordinate plot (i.e. ax1)
        tf_listener_1.update_coordinate_plot(ax1)
        tf_listener_2.update_coordinate_plot(ax1)
        tf_listener_3.update_coordinate_plot(ax1)
        ax1.legend()

    
        # Update the distance plot (i.e. ax2)
        # get_distance(current_time)
        # show_master2slave(current_time)

        # Display the error distance with master robot (i.e. ax3)
        distance_error_1.updateErrorPlot(ax3, master_robot_name, slave_robot_name_1)
        distance_error_2.updateErrorPlot(ax3, master_robot_name, slave_robot_name_2)
        ax3.legend()

        # Update the plots
        plt.draw()
        plt.pause(0.001)

    def clear_plots(event):

        # Clear ax1 data
        ax1.clear()
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_title('Robot Coordinates')
        tf_listener_1.x_list.clear()
        tf_listener_1.y_list.clear()
        tf_listener_1.z_list.clear()
        tf_listener_2.x_list.clear()
        tf_listener_2.y_list.clear()
        tf_listener_2.z_list.clear()
        tf_listener_3.x_list.clear()
        tf_listener_3.y_list.clear()
        tf_listener_3.z_list.clear()

        # Clear ax2 data
        ax2.clear()
        ax2.set_xlabel('Time')
        ax2.set_ylabel('Distance')
        ax2.set_title('Distance between robots')
        distance_list.clear()
        time_list.clear()

        # Clear ax3 data
        ax3.clear()
        ax3.set_xlabel('Time')
        ax3.set_ylabel('Distance')
        ax3.set_title('Error Distance')
        distance_error_1.error_distance_list.clear()
        distance_error_1.time_list.clear()
        distance_error_2.error_distance_list.clear()
        distance_error_2.time_list.clear()

        plt.draw()
        plt.pause(0.001)

    
    def get_distance(current_time):
        # Compute the distance between the two robots (igonore the z axis)
        try:
            distance = np.sqrt((tf_listener_1.x_list[-1]-tf_listener_2.x_list[-1])**2 + (tf_listener_1.y_list[-1]-tf_listener_2.y_list[-1])**2)
            distance_list.append(distance)
            time_list.append(rospy.Time.now().to_sec()-start_time)

            ax2.cla()
            ax2.set_xlabel('Time')
            ax2.set_ylabel('Distance')
            ax2.set_title('Distance between robots')
            ax2.plot(time_list, distance_list, 'bo', markersize=1, linewidth=0.5, linestyle='solid')
            # print('Distance: ', distance, 'Time',rospy.Time.now().to_sec()-start_time)
        except:
            print('No TF data')
            return

    clear_button = plt.axes([0.95, 0.05, 0.03, 0.055])
    clear_ax = plt.Button(clear_button, 'CLR')
    clear_ax.on_clicked(clear_plots)

    timer = fig.canvas.new_timer(interval=1)
    timer.add_callback(update_plots)
    timer.start()

    rospy.sleep(1)
    while not rospy.is_shutdown():
        update_plots()
        
    rospy.signal_shutdown('ROS shutdown')
    plt.show()

if __name__ == '__main__':
    main()
