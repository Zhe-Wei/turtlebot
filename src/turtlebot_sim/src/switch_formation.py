#!/usr/bin/env python3

import rospy
import tf
from tf import transformations
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

slave1_x, slave1_y, slave2_x, slave2_y = 0, 0, 0, 0
tmp_slave1_x, tmp_slave1_y, tmp_slave2_x, tmp_slave2_y = 0, 0, 0, 0
switching = False


def switch_formation():
    global tmp_slave1_x, tmp_slave1_y, tmp_slave2_x, tmp_slave2_y
    
    tmp_slave1_x = -0.8
    tmp_slave1_y = 0
    rospy.sleep(4)
    tmp_slave2_x = -1.6
    tmp_slave2_y = 0


def callback_function(event):
    # Perform necessary actions here
    print("Switch_formation")
    switch_formation()


def start_timer():
    # Create a timer that expires after 10 seconds
    rospy.Timer(rospy.Duration(20), callback_function)


def lidar_callback(scan_data):

    def isNarrowPath(scan_data):# check if there have two obstacles in front of the leader robot
        global tmp_slave1_x, tmp_slave1_y, tmp_slave2_x, tmp_slave2_y
        global switching
        MAX_DISTANCE = 5

        d1 = scan_data.ranges[setDegree]
        d2 = scan_data.ranges[360-setDegree]

        if scan_data.ranges[setDegree] > MAX_DISTANCE:
            d1 = MAX_DISTANCE
        if scan_data.ranges[360-setDegree] > MAX_DISTANCE:
            d2 = MAX_DISTANCE
        
        print(f"L: {scan_data.ranges[setDegree]:.2f}, R: {scan_data.ranges[360-setDegree]:.2f}")

        try:
            d = math.pow(((math.pow(d1,2)+math.pow(d2,2)) - 2*d1*d2*math.cos(math.radians(2*setDegree))), 0.5)
            #dislplay the distance float number .2f
            print(f"d: {d:.2f}")
        except:
            # Have inf value in d1 or d2
            print(f"{d1:.2f}, {d2:.2f}")

        if switching == True:
            pass

        if d < 0.7 * (math.fabs(slave1_y) + math.fabs(slave2_y)):
            def change_slave_2(event):
                global tmp_slave1_x, tmp_slave1_y, tmp_slave2_x, tmp_slave2_y
                if d < 0.7 * (math.fabs(slave1_y) + math.fabs(slave2_y)):
                    tmp_slave2_x = -1.6
                    tmp_slave2_y = 0
            # tmp_slave1_y = (d / 2)
            # tmp_slave2_y = -(d / 2)
            print("Dangerous")
            if switching == False and d < 0.7 * (math.fabs(slave1_y) + math.fabs(slave2_y)):
                switching = True

                tmp_slave1_x = -0.8
                tmp_slave1_y = 0
                # rospy.sleep(1)
                rospy.Timer(rospy.Duration(3), change_slave_2, oneshot=True)
                tmp_slave2_x = -1.6
                tmp_slave2_y = 0

                switching = False
            else:
                return
            # rospy.sleep(.1)
            print("Switch_formation")
        elif d < 1.2 * (math.fabs(slave1_y) + math.fabs(slave2_y)):
            tmp_slave1_x = slave1_x
            tmp_slave2_x = slave2_x
            tmp_slave1_y = (d / 2)
            tmp_slave2_y = -(d / 2)

            if math.fabs(tmp_slave1_y) + math.fabs(tmp_slave2_y) > math.fabs(slave1_y) + math.fabs(slave2_y):
                tmp_slave1_x = slave1_x
                tmp_slave2_x = slave2_x
                tmp_slave1_y = slave1_y    
                tmp_slave2_y = slave2_y

            print("Not safe")
        else:
            tmp_slave1_x = slave1_x
            tmp_slave2_x = slave2_x
            tmp_slave1_y = slave1_y
            tmp_slave2_y = slave2_y
            print("Safe")

        print('----------------------\n')
        # if d < dangerDistance:
        #     print("Dangerous")
        # elif d < safeDistance:
        #     print("Not safe")
        # else:
        #     print("Safe")
    
    

    isNarrowPath(scan_data)
        

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
    dangerDistance = rospy.get_param('~dangerDistance', 0.8)
    safeDistance = rospy.get_param('~safeDistance', 1)


    # publish /robot0/cmd_vel
    pub = rospy.Publisher(f'/{leader_robot_name}/cmd_vel', Twist, queue_size=10)

    # subscribe leader robot's /scan
    rospy.Subscriber(f'/{leader_robot_name}/scan', LaserScan, lidar_callback)

    # Create a transform broadcaster
    rate = rospy.Rate(10.0)

    # Create a transform broadcaster
    br = tf.TransformBroadcaster()

    # Start the timer
    # start_timer()

    # Initialize slave1 and slave2 position
    tmp_slave1_x = slave1_x
    tmp_slave1_y = slave1_y
    tmp_slave2_x = slave2_x
    tmp_slave2_y = slave2_y

    while not rospy.is_shutdown():
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
    
        rate.sleep()

    