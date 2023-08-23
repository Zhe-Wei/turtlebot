#!/usr/bin/python3

import rospy
import tf
import csv
import os
from datetime import datetime
from openpyxl import Workbook

class RobotDataSaver:
    def __init__(self, robot_names):
        rospy.init_node('Save_robot_raw_data')

        # pid controller or fuzzy controller
        controller_type = rospy.get_param('~PID', 'none')
        print('controller_type: ', controller_type)
        if controller_type == True or controller_type == 'true':
            controller_type = 'pid'
        elif controller_type == False or controller_type == 'false':
            controller_type = 'fuzzy'
        
        self.listener = tf.TransformListener()
        self.relative_error_listener = tf.TransformListener()
        self.robot_names = robot_names
        self.csv_file_path = os.path.expanduser(f'/home/ubuntu/catkin_ws/src/turtlebot_sim/raw_data/{controller_type}')
        self.create_directory(self.csv_file_path)

        file_name = datetime.now().strftime("%Y%m%d%H%M%S")
        self.csv_file = open(os.path.join(self.csv_file_path, f'{file_name}.csv'), 'w')
        self.csv_writer = csv.writer(self.csv_file)

        # header example [timestamp robot_0_x robot_0_y robot_0_theta robot_1_x robot_1_y robot_1_theta robot_2_x robot_2_y robot_2_theta]
        header = ['timestamp'] + [f'{robot}_x {robot}_y {robot}_theta' for robot in robot_names] + [f'relative_err_{robot}_x relative_err_{robot}_y relative_err_{robot}_theta' for robot in slave_robot_names]

        # flat array
        header = [item for sublist in header for item in sublist.split()]

        # write header to csv file
        self.csv_writer.writerow(header)

    def create_directory(self, path):
        if not os.path.exists(path):
            os.makedirs(path)

    def save_data_to_csv(self, data_dict):

        # create row
        timestamp = rospy.Time.now().to_sec()
        row = [timestamp]

        # add robot data to row
        for robot_name in self.robot_names:
            row.extend([round(data_dict[robot_name][0][0], 3), # robot_i_x
                        round(data_dict[robot_name][0][1], 3), # robot_i_y
                        round(data_dict[robot_name][1][2], 3)])

        # add relative error data to row
        for robot_name in slave_robot_names:
            row.extend([round(data_dict['relative_err_'+robot_name][0][0], 3), # relative_err_robot_i_x
                        round(data_dict['relative_err_'+robot_name][0][1], 3), # relative_err_robot_i_y
                        round(data_dict['relative_err_'+robot_name][1][2], 3)])

        # write row to csv file
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def run(self):
        rate = rospy.Rate(10)  # Update rate in Hz
        while not rospy.is_shutdown():
            try:
                # self.listener.waitForTransform('map', f'{robot_name}/base_footprint', rospy.Time(0), rospy.Duration(1.0))
                data_dict = {}
                # print(self.robot_names)

                # Save each robot's pose
                for robot_name in self.robot_names:
                    self.listener.waitForTransform('map', f'{robot_name}/base_footprint', rospy.Time(0), rospy.Duration(1.0))
                    trans, rot = self.listener.lookupTransform('map', f'{robot_name}/base_footprint', rospy.Time(0))

                    # quaternion to euler
                    rot = tf.transformations.euler_from_quaternion(rot)

                    data_dict[robot_name] = (trans, rot)

                # Save relative error between leader robot
                for robot_name in self.robot_names:
                    # skip master robot
                    if robot_name == master_robot_name:
                        continue
                    
                    frame_1 = '' + master_robot_name + '/' + robot_name # desire tracking point frame
                    frame_2 = robot_name + '/base_footprint' # slave robot frame

                    self.relative_error_listener.waitForTransform(frame_1, frame_2, rospy.Time(0), rospy.Duration(1.0))
                    trans, rot = self.relative_error_listener.lookupTransform(frame_1, frame_2, rospy.Time(0))

                    # quaternion to euler
                    rot = tf.transformations.euler_from_quaternion(rot)

                    data_dict['relative_err_'+robot_name] = (trans, rot)

                self.save_data_to_csv(data_dict)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print(f'map -> {robot_name}/base_footprint tf not available')
                rospy.sleep(0.1)  # Wait 0.1 second if TF listener is not available
                
            rate.sleep()

if __name__ == '__main__':
    print('Start saving robot raw data')
    master_robot_name = rospy.get_param('~master_robot_name', 'robot_0')
    slave_robot_names = [rospy.get_param('~slave_robot_name_1', 'robot_1'), rospy.get_param('~slave_robot_name_2', 'robot_2')]
    robot_names = [master_robot_name] + slave_robot_names

    data_saver = RobotDataSaver(robot_names)
    data_saver.run()
