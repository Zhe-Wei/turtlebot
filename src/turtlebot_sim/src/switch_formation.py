#!/usr/bin/env python3

import rospy
import tf
from tf import transformations
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

slave1_x, slave1_y, slave2_x, slave2_y = 0, 0, 0, 0
tmp_slave1_x, tmp_slave1_y, tmp_slave2_x, tmp_slave2_y = 0, 0, 0, 0
switching = False
trans_1, rot_1, trans_2, rot_2 = None, None, None, None
# leader_w = 0

def lidar_callback(scan_data):

    # get the width of the path
    def getPathWidth(scan_data):
        deg = setDegree
        a1 = scan_data.ranges[deg]
        b1 = scan_data.ranges[360-deg]
        a2 = scan_data.ranges[deg+10]
        b2 = scan_data.ranges[360-deg-10]
        a3 = scan_data.ranges[deg+20]
        b3 = scan_data.ranges[360-deg-20]
        a4 = scan_data.ranges[deg+30]
        b4 = scan_data.ranges[360-deg-30]
        a5 = scan_data.ranges[deg+40]
        b5 = scan_data.ranges[360-deg-40]
        a6 = scan_data.ranges[deg+45]
        b6 = scan_data.ranges[360-deg-45]
        a7 = scan_data.ranges[deg+50]
        b7 = scan_data.ranges[360-deg-50]
        a8 = scan_data.ranges[deg+60]
        b8 = scan_data.ranges[360-deg-60]


        # cossine law to calculate the width of the path  c1=30(deg), c2=40(deg), c3=50(deg)
        c1 = math.pow(((math.pow(a1,2)+math.pow(b1,2)) - 2*a1*b1*math.cos(math.radians(2*deg))), 0.5)
        c2 = math.pow(((math.pow(a2,2)+math.pow(b2,2)) - 2*a2*b2*math.cos(math.radians(2*deg+20))), 0.5)
        c3 = math.pow(((math.pow(a3,2)+math.pow(b3,2)) - 2*a3*b3*math.cos(math.radians(2*deg+40))), 0.5)
        c4 = math.pow(((math.pow(a4,2)+math.pow(b4,2)) - 2*a4*b4*math.cos(math.radians(2*deg+60))), 0.5)
        c5 = math.pow(((math.pow(a5,2)+math.pow(b5,2)) - 2*a5*b5*math.cos(math.radians(2*deg+80))), 0.5)
        c6 = math.pow(((math.pow(a6,2)+math.pow(b6,2)) - 2*a6*b6*math.cos(math.radians(2*deg+90))), 0.5)
        c7 = math.pow(((math.pow(a7,2)+math.pow(b7,2)) - 2*a7*b7*math.cos(math.radians(2*deg+100))), 0.5)
        c8 = math.pow(((math.pow(a8,2)+math.pow(b8,2)) - 2*a8*b8*math.cos(math.radians(2*deg+120))), 0.5)

        # if c1 ~ c4 is nan value, set it to 5m
        if math.isnan(c1) or c1 == "inf": c1 = 99
        if math.isnan(c2) or c2 == "inf": c2 = 99
        if math.isnan(c3) or c3 == "inf": c3 = 99
        if math.isnan(c4) or c4 == "inf": c4 = 99
        if math.isnan(c5) or c5 == "inf": c5 = 99
        if math.isnan(c6) or c6 == "inf": c6 = 99
        if math.isnan(c7) or c7 == "inf": c7 = 99
        if math.isnan(c8) or c8 == "inf": c8 = 99

        print(f"c1: {c1:.2f}, c2: {c2:.2f}, c3: {c3:.2f}, c4: {c4:.2f}")
        nonlocal d_ij
        d_ij = min(c1, c2, c3, c4, c5, c6, c7, c8)
        print(f"d_ij: {d_ij:.2f}")

        return d_ij


    def getPriotity():
        # calculate distance between leader
        dis_1 = math.sqrt(trans_1[0] ** 2 + trans_1[1] ** 2)
        dis_2 = math.sqrt(trans_2[0] ** 2 + trans_2[1] ** 2)

        # calculate priority
        if dis_1 < dis_2:
            priority = 1
        elif dis_1 > dis_2:
            priority = 2
        else:
            priority = 1

        return priority
    

    def switch_formation(pri):
        nonlocal d_ij
        global tmp_slave1_x, tmp_slave1_y, tmp_slave2_x, tmp_slave2_y
        global slave1_x, slave1_y, slave2_x, slave2_y
        print("--------------------")
        print(f"pri: {pri}")
        # r_p = robot protection distance, turtlebot size = 281mm x 306mm x 141mm => set 0.4m as the safe distance
        r_p =  0.4
        d_m = 0.5 * d_ij
        d_iw = d_m - r_p
        if d_iw < 0:
            raise Exception("d_iw < 0")

        # l_i 維持與leader的距離
        if pri == 2:
            l_i = math.sqrt(math.pow(slave2_x, 2) + math.pow(slave2_y, 2))
        else:
            l_i = math.sqrt(math.pow(slave1_x, 2) + math.pow(slave1_y, 2))
        

        print(f"d_ij: {d_ij:.2f}, d_m: {d_m:.2f}, d_iw: {d_iw:.2f}, l_i: {l_i:.2f}")
        
        if pri == 2:
            tmp_slave2_x = -math.sqrt(math.pow(l_i, 2) - math.pow(d_iw, 2)) # 負號代表在leader後方
            tmp_slave2_y = -d_iw
        else:
            tmp_slave1_x = -math.sqrt(math.pow(l_i, 2) - math.pow(d_iw, 2))
            tmp_slave1_y = d_iw

        d_jw = d_iw

        if pri == 2:
            try:
                # tmp = math.sqrt(math.pow((2 * r_p), 2) - math.pow((d_iw + d_jw), 2))
                tmp = 2 * r_p + math.fabs(tmp_slave2_x)
            except:
                tmp = 2 * r_p + math.fabs(tmp_slave2_x)
            print(f"d_jw: {d_jw:.2f} tmp: {tmp:.2f}")
            # l_j = math.sqrt(math.pow(tmp, 2) + math.pow(d_jw, 2))
            tmp_slave1_x = -tmp
            tmp_slave1_y = d_jw
        else:
            try:
                # tmp = math.sqrt(math.pow((2 * r_p), 2) - math.pow((d_iw + d_jw), 2))
                tmp = 2 * r_p + math.fabs(tmp_slave1_x)
            except:
                tmp = 2 * r_p + math.fabs(tmp_slave1_x)
            print(f"d_jw: {d_jw:.2f} tmp: {tmp:.2f}")
            # l_j = math.sqrt(math.pow(tmp, 2) + math.pow(d_jw, 2))
            tmp_slave2_x = -tmp
            tmp_slave2_y = -d_jw
        print(f"slave1 ({tmp_slave1_x:.2f}, {tmp_slave1_y:.2f}), slave2 ({tmp_slave2_x:.2f}, {tmp_slave2_y:.2f})")


    # isNarrowPath(scan_data)
    global tmp_slave1_x, tmp_slave1_y, tmp_slave2_x, tmp_slave2_y
    global slave1_x, slave1_y, slave2_x, slave2_y
    global switching
    d_ij = 0
    d_m = 0
    d_ij = getPathWidth(scan_data)
    
    # leader robot 沿著中心線行進
    # center_leader_robot(scan_data)

    # 陣行水平寬度半徑設定 d_f=(|y1|+|y2|+variable)/2, variable=保留給機器人本身
    df = (math.fabs(slave1_y) + math.fabs(slave2_y) + 0.4) / 2
    print(f"s1_y: {slave1_y:.2f}, s2_y: {slave2_y:.2f}, df: {df:.2f}")

    # 陣形半徑設定 d_formation_r，此參數給leader通過障礙物時使用，判斷何時恢復陣行
    d_formation_r = math.sqrt(math.pow(slave1_x, 2) + math.pow(slave1_y, 2)) + 1.2
    print(f"d_formation_r: {d_formation_r:.2f}")

    if (0.5 * d_ij) < df: # 陣形過窄，切換陣形
        switching = 1
        print("Narrow path")
        pri = getPriotity()
        switch_formation(pri)
    elif switching == 1 and scan_data.ranges[135] > d_formation_r and scan_data.ranges[360-135] > d_formation_r: 
        # 偵測左後方與右後方是否有足夠空間->恢復陣形
        switching = 0
        tmp_slave1_x = slave1_x
        tmp_slave1_y = slave1_y
        tmp_slave2_x = slave2_x
        tmp_slave2_y = slave2_y
        print("Wide path")
    print()

if __name__ == '__main__':

    # Initialize the node
    rospy.init_node('publish_slave_link')

    # Get the parameters
    slave1_x = rospy.get_param("~slave1_x", -1.2)
    slave1_y = rospy.get_param("~slave1_y", 1.2)
    slave2_x = rospy.get_param("~slave2_x", -1.2)
    slave2_y = rospy.get_param("~slave2_y", -1.2)

    leader_robot_name = rospy.get_param('~leader_robot_name', "robot_0")
    slave1_robot_name = rospy.get_param('~slave_robot_1', "robot_1")
    slave2_robot_name = rospy.get_param('~slave_robot_2', "robot_2")

    setDegree = rospy.get_param('~setDegree', 30)
    # dangerDistance = rospy.get_param('~dangerDistance', 0.8)
    # safeDistance = rospy.get_param('~safeDistance', 1)


    # publish /robot0/cmd_vel
    leader_pub = rospy.Publisher(f'/{leader_robot_name}/cmd_vel', Twist, queue_size=10)
    
    # publish switching topic
    switching_pub = rospy.Publisher(f'/{leader_robot_name}/switching', Bool, queue_size=10)

    # subscribe leader robot's /scan
    rospy.Subscriber(f'/{leader_robot_name}/scan', LaserScan, lidar_callback)

    # Create a transform broadcaster
    rate = rospy.Rate(10.0)

    # Create a transform broadcaster
    br = tf.TransformBroadcaster()

    # create transform listener
    listener1 = tf.TransformListener()
    listener2 = tf.TransformListener()

    # Initialize slave1 and slave2 position
    tmp_slave1_x = slave1_x
    tmp_slave1_y = slave1_y
    tmp_slave2_x = slave2_x
    tmp_slave2_y = slave2_y

    while not rospy.is_shutdown():
        try:
            # global trans_1, rot_1, trans_2, rot_2
            (trans_1, rot_1) = listener1.lookupTransform(slave1_robot_name+'/base_link', leader_robot_name+'/base_link', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo(f"Wait for robot_1 tf. [switch_formation.py]")
            continue

        try:
            (trans_2, rot_2) = listener2.lookupTransform(slave2_robot_name+'/base_link', leader_robot_name+'/base_link', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo(f"Wait for robot_2 tf. [switch_formation.py]")
            continue
        
        # Publish the transform over tf
        br.sendTransform((tmp_slave1_x, tmp_slave1_y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         leader_robot_name + '/' + slave1_robot_name,
                         leader_robot_name + '/' + "base_footprint")
        br.sendTransform((tmp_slave2_x, tmp_slave2_y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         leader_robot_name + '/' + slave2_robot_name,
                         leader_robot_name + '/' + "base_footprint")
        
        # Publish switching topic to slave robots
        # inform them to switch formation and follower close apf algorithm
        switching_pub.publish(switching)

        rate.sleep()