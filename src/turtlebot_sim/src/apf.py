#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Float32
from laser_geometry import LaserProjection
from geometry_msgs.msg import Twist
import tf
from tf import transformations
from tf import broadcaster
from std_msgs.msg import Int32, Bool

import math
import numpy as np
from matplotlib import pyplot as plt


# Defining constants
PI = 3.14
USER_FORCE = 1 # Change this to prioritize goal
ROBOT_RADIUS = 0.2 # Change this to robot's dimensions.
DEBUG = False

# Declaring global variables
user_input = 180
max_distance = 1.2
factor_angular = 400


fig, axs = None, None
theta = 0

cmd_vel_msg = Twist()
cmd_vel_data = Twist()
vel_pub = None
leader_robot_name = None
follower_robot_name = None

switching = False

def userCallback(user_data):
    global user_input
    user_input = user_data.data


def callback(scan_in):
    global vel_pub, cmd_vel_msg, cmd_vel_data, user_input 
    global switching

    # Copy lidar data
    scan_filtered = scan_in

    # Change lidar data type from tuple to list
    scan_filtered.ranges = list(scan_in.ranges)
    total_points = int((scan_filtered.angle_max - scan_filtered.angle_min) / scan_filtered.angle_increment)
    force = 0

    # if switching is true, ouput original velocity
    if switching:
        vel_pub.publish(cmd_vel_data)
        return

    # Preprocess LIDAR data to remove inf values and remove 0 values.
    for i in range(total_points+1):
        # Remove inf values from LIDAR data
        if scan_filtered.ranges[i] > max_distance:
            scan_filtered.ranges[i] = max_distance

        # If there are 0 readings, make them the LIDAR sensor's maximum range.
        if scan_filtered.ranges[i] == 0:
            scan_filtered.ranges[i] = max_distance

        # Make the robot a point by subtracting the robot radius from the reading.
        # if scan_filtered.ranges[i] > ROBOT_RADIUS:
        #     scan_filtered.ranges[i] = scan_filtered.ranges[i] - ROBOT_RADIUS

    # Remove last data 
    scan_filtered.ranges.pop()
    # print("scan_filtered.ranges: %s", scan_filtered.ranges)
    
    # Variables for potential field are initialized.
    force_x = 0
    force_y = 0
    _force_x = 0
    _force_y = 0

    user_force = USER_FORCE
    # The goal attraction is obtained from the user input subscriber
    user_x = user_force * math.cos(user_input * PI / 180)
    user_y = user_force * math.sin(user_input * PI / 180)
    non_zero_points = 1
    # Forces are computed for the LIDAR data.
    if DEBUG:
        print("Total points: ", total_points)

    # Forces are computed for the LIDAR data.
    x_datas = []
    y_datas = []

    for i in range(total_points):
        if i > 90 and i < 270:
            x_datas.append(0)
            y_datas.append(0)
            continue
        if scan_filtered.ranges[i] != max_distance:
            x = scan_filtered.ranges[i] * math.cos((i + 0) * PI / 180)
            y = scan_filtered.ranges[i] * math.sin((i + 0) * PI / 180)
            # print(f"x = {np.round(x, 2)}, y = {np.round(y, 2)}")

            if scan_filtered.ranges[i] > 0:
                # # Transforming to robot frame
                # x = x_lidar * math.cos(-90 * PI / 180) - y_lidar * math.sin(-90 * PI / 180)
                # y = x_lidar * math.sin(-90 * PI / 180) + y_lidar * math.cos(-90 * PI / 180)
                # The scale is inverted to make nearer obstacles have higher force and the scale is reduced to 1.
                if abs(x) > 0:
                    x = -1 / math.pow(x, 2) * math.copysign(1, x)
                if abs(y) > 0:
                    y = -1 / math.pow(y, 2) * math.copysign(1, y)

                x_datas.append(x/1)
                y_datas.append(y/1)

                force_x += x
                force_y += y
                _force_x += x
                _force_y += y

                non_zero_points += 1
        else:
            x_datas.append(0)
            y_datas.append(0)
            non_zero_points += 1
    if DEBUG:
        print(f"force_x = {np.round(force_x, 2)}, force_y = {np.round(force_y, 2)}")
        print(f"_force_x = {np.round(_force_x, 2)}, _force_y = {np.round(_force_y, 2)}")


    # The resultant force is calculated
    norm_factor = math.sqrt(pow(_force_x, 2) + pow(_force_y, 2))

    if DEBUG:
        print(f"norm_factor = {np.round(norm_factor, 2)}")

    # theta = math.atan2((_force_y / total_points) - user_y, (_force_x / total_points) - user_x) * 180 / PI
    # print(f"theta = {np.round(theta, 2)}")
    total_points = non_zero_points
    
    # The resultant direction is calculated
    resultant_x = 0
    resultant_y = 0

    # Normalizing the resultant force
    if norm_factor != 0:
        resultant_x = (_force_x / norm_factor)
        resultant_y = (_force_y / norm_factor)
    if DEBUG:
        print(f'resultant_x = {np.round(resultant_x, 2)}, resultant_y = {np.round(resultant_y, 2)}')

    # Unit: degree
    theta = math.atan2(resultant_y, resultant_x) * 180 / PI

    if theta < 0:
        theta = 360 + theta
    if DEBUG:
        print(f"theta = {np.round(theta, 2)}")

    # calculate angular velocity theta/(2*100)
    w = 0
    if theta > 180: # 180 ~ 360 右轉
        w = (theta - 360) / factor_angular
    else: # 0 ~ 180 左轉
        w = theta / factor_angular 
    if DEBUG:
        print(f"w = {np.round(w, 2)}")

    # user input added to the resultant force
    user_x = 1
    user_y = 0
    # The resultant force is calculated
    agg_x = resultant_x
    agg_y = resultant_y
    if DEBUG:
        print(f"agg_x = {np.round(agg_x, 2)}, agg_y = {np.round(agg_y, 2)}")

    # The resultant direction is calculated
    agg_theta = math.atan2(agg_y, agg_x) * 180 / PI
    if DEBUG:
        print(f"agg_theta = {np.round(agg_theta, 2)}")

    # publish the velocity
    vel_msg = Twist()
    vel_msg.linear.x = cmd_vel_data.linear.x

    # 沒有障礙物
    if force_x == 0 and force_y == 0:
        vel_msg.angular.z = cmd_vel_data.angular.z
    else:
        # 障礙物在前方
        if theta < 30 or theta > 330:
            vel_msg.angular.z = 2 * w + 0.8 * cmd_vel_data.angular.z   
        # 障礙物在其他地方
        else:
            vel_msg.angular.z = 2 * w + 0.8 * cmd_vel_data.angular.z

    # 障礙物在正面
    if resultant_y <= -0.95:
        vel_msg.linear.x = -0.15
        vel_msg.angular.z = 0.1

    # 障礙物在後方
    if resultant_y >= 0.9:
        vel_msg.linear.x = 0.4
        vel_msg.angular.z = 0
    # 障礙物在正左右方
    if resultant_x <= -0.8 or resultant_x >= 0.8:
        vel_msg.linear.x = 0.4
        vel_msg.angular.z = 2 * w + 0.5 * cmd_vel_data.angular.z

    vel_pub.publish(vel_msg)
    
    if False:
        # Plot the LIDAR data
        plt.cla()

        # theta and agg_theta are degrees(0-360), draw arrow in the plot 2d 

        # Define the starting point of the arrow
        x0, y0 = 0, 0

        # Define the direction and length of the arrow for theta
        theta_dir = theta * np.pi / 180  # Convert theta to radians
        theta_len = 0.5  # Define the length of the arrow
        theta_dx, theta_dy = theta_len * np.cos(theta_dir), theta_len * np.sin(theta_dir)

        # Define the direction and length of the arrow for agg_theta
        agg_theta_dir = agg_theta * np.pi / 180  # Convert agg_theta to radians
        agg_theta_len = 0.5  # Define the length of the arrow
        agg_theta_dx, agg_theta_dy = agg_theta_len * np.cos(agg_theta_dir), agg_theta_len * np.sin(agg_theta_dir)

        # Plot the arrows
        plt.arrow(x0, y0, 1, 0, head_width=0.05, head_length=0.1, fc='k', ec='k', label="attraction")
        plt.arrow(x0, y0, theta_dx, theta_dy, head_width=0.05, head_length=0.1, fc='r', ec='r', label="theta")
        plt.arrow(x0, y0, agg_theta_dx, agg_theta_dy, head_width=0.05, head_length=0.1, fc='b', ec='b', label="agg_theta")
        plt.xlim(-1, 1)
        plt.ylim(-1, 1)
        plt.grid(True)

        # Show the plot
        # plt.show()
        plt.pause(0.001)



    # global theta
    if False:
        plt.clf()
        plt.title(f"Robot name = {follower_robot_name}\n\
                  force_x = {np.round(force_x, 2)}, force_y = {np.round(force_y, 2)}\n\
                  resultant_x = {np.round(resultant_x, 2)}, resultant_y = {np.round(resultant_y, 2)}\n\
                  theta = {np.round(theta, 2)}, w = {np.round(w, 2)}, vel_msg.linear.x = {np.round(vel_msg.linear.x, 2)}\n\
                  vel_msg.linear.x = {np.round(vel_msg.linear.x, 2)}, vel_msg.angular.z = {np.round(vel_msg.angular.z, 2)}\n"
                  )
        plt.plot(scan_filtered.ranges)
        plt.plot(np.array(x_datas)/ norm_factor)
        plt.plot(np.array(y_datas)/ norm_factor)

        plt.legend(['Lidar datas', 'x_datas', 'y_datas'])
        plt.grid(True)
        plt.draw()
        plt.pause(0.001)

    
    # plot theta(range 0-360 degree) in 2D
    # center in (0,0) and radius = 1 give me a arrow represent theta
    # rontate the polar plot 90 degree to make it same as the robot frame
    if False:
        plt.clf()
        plt.title(f"theta = {np.round(theta, 2)}")
        # plt.polar(theta, 1, 'ro')
        plt.polar(theta * PI / 180, 1, 'ro')
        plt.grid(True)
        plt.draw()
        plt.pause(0.001)


def cmd_vel_ori_Callback(msg):
    global cmd_vel_msg, cmd_vel_data

    cmd_vel_msg.linear.x = msg.linear.x
    cmd_vel_msg.angular.z = msg.angular.z

    cmd_vel_data.linear.x = msg.linear.x
    cmd_vel_data.angular.z = msg.angular.z

    # print("cmd_vel_ori_Callback")

def switchingCallback(msg):
    global switching

    switching = msg.data
    # print("switchingCallback")


def main():
    # Initialize the node
    rospy.init_node('potential_field')
    print("potential_field node started\n")

    global max_distance, cmd_vel_msg
    # Get the parameters
    leader_robot_name = rospy.get_param('~leader_robot_name', "robot_0")
    follower_robot_name = rospy.get_param('~follower_robot_name', "robot_1")
    # max_vel_x = rospy.get_param('~max_vel_x', 1.5)
    # min_vel_x = rospy.get_param('~min_vel_x', 0.05)
    # max_vel_theta = rospy.get_param('~max_vel_theta', 1.5)
    # min_vel_theta = rospy.get_param('~min_vel_theta', 0.05)
    max_distance = rospy.get_param('~max_distance', 0.8)
    
    # Subscribing to user input which is the goal direction
    # rospy.Subscriber('/tunnel/user_input', Float32, userCallback)
    rospy.Subscriber(f'/{follower_robot_name}/scan', LaserScan, callback)

    # Subscribing to the velocity of the slave robot
    vel_sub = rospy.Subscriber(f'/{follower_robot_name}/cmd_vel_ori', Twist, cmd_vel_ori_Callback)

    # Subscribing switching topic from leader robot
    rospy.Subscriber(f'/{leader_robot_name}/switching', Bool, switchingCallback)

    # Publish the velocity of the slave robot
    global vel_pub
    vel_pub = rospy.Publisher(f'/{follower_robot_name}/cmd_vel', Twist, queue_size=1)

    # Create a transform listener
    listener = tf.TransformListener()

    # global theta_pub, f_pub
    # Rate is set to 10 Hz. 
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # Get the transform from the base frame to the slave frame
        try:
            # 追隨者座標與期望座標的相對位置
            ## Ex: (追隨者座標) /follower/base_footprint -> (期望座標) /leader/follower
            frame_1 = f"{follower_robot_name}/base_footprint"
            frame_2 = f"{leader_robot_name}/{follower_robot_name}"

            (trans, rot) = listener.lookupTransform(frame_1, frame_2, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Waiting for TF data or no TF data available [apf.py]")
            rospy.sleep(1)
            continue
        if DEBUG:
            print(f'-------------{follower_robot_name}---------------')
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
