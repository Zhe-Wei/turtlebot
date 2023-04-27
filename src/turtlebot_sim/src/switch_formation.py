#!/usr/bin/env python3

import rospy
import tf
from tf import transformations
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist

slave1_x, slave1_y, slave2_x, slave2_y = 0, 0, 0, 0
switch = True

# def clicked_point_cb(msg):
#     global slave1_x, slave1_y, slave2_x, slave2_y
#     global switch
#     if switch:
#         # switch formation shape to triangle
#         slave1_x = -1.2
#         slave1_y = 1.2
#         rospy.sleep(3)
#         slave2_x = -1.2
#         slave2_y = -1.2
#     else:
#         # switch formation shape to line
#         slave1_x = -0.8
#         slave1_y = 0
#         rospy.sleep(5)
#         slave2_x = -1.6
#         slave2_y = 0
#     switch = not switch
#     print(msg.point.x, msg.point.y)

def switch_formation():
    global slave1_x, slave1_y, slave2_x, slave2_y
    global switch
    if switch:
        # switch formation shape to triangle
        slave1_x = -1.2
        slave1_y = 1.2
        rospy.sleep(3)
        slave2_x = -1.2
        slave2_y = -1.2
    else:
        # switch formation shape to line
        slave1_x = -0.8
        slave1_y = 0
        rospy.sleep(4)
        slave2_x = -1.6
        slave2_y = 0
    switch = not switch

def callback_function(event):
    # Perform necessary actions here
    print("Switch_formation")
    switch_formation()

def start_timer():
    # Create a timer that expires after 10 seconds
    rospy.Timer(rospy.Duration(20), callback_function)

if __name__ == '__main__':
    # Get the parameters
    slave1_x = rospy.get_param("~slave1_x", -1.2)
    slave1_y = rospy.get_param("~slave1_y", 1.2)
    slave2_x = rospy.get_param("~slave2_x", -1.2)
    slave2_y = rospy.get_param("~slave2_y", -1.2)

    leader_robot_name = rospy.get_param('~leader_robot_name', "robot_0")
    slave1_robot_name = rospy.get_param('~slave_robot_1', "robot_1")
    slave2_robot_name = rospy.get_param('~slave_robot_2', "robot_2")


    # Subscribe topic 
    # rospy.Subscriber('/clicked_point', PointStamped, clicked_point_cb)

    # publish /robot0/cmd_vel
    pub = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)

    # Initialize the node
    rospy.init_node('publish_slave_link')

    # Create a transform broadcaster
    rate = rospy.Rate(10.0)

    # Create a transform broadcaster
    br = tf.TransformBroadcaster()

    # Start the timer
    start_timer()

    while not rospy.is_shutdown():
        # Publish the transform over tf
        print("test")
        br.sendTransform((slave1_x, slave1_y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         leader_robot_name + '/' + slave1_robot_name,
                         leader_robot_name + '/' + "base_footprint")
        br.sendTransform((slave2_x, slave2_y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         leader_robot_name + '/' + slave2_robot_name,
                         leader_robot_name + '/' + "base_footprint")
    
        rate.sleep()

    