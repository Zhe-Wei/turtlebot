#!/usr/bin/env python3

import rospy
import tf
from tf import transformations
from tf import broadcaster
import geometry_msgs.msg

import math
import numpy as np
from simple_pid import PID

from dynamic_reconfigure.server import Server # http://wiki.ros.org/dynamic_reconfigure/Tutorials/HowToWriteYourFirstCfgFile
from turtlebot_sim.cfg import tf_pidConfig  

pid_linear = PID(2, 0.0, 0.0)
pid_linear.output_limits = (-0.5, 0.5)
pid_angular = PID(5, 0.0, 0.0)
pid_angular.output_limits = (-1.0, 1.0)

# dynamic reconfigure callback
def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {linear_kp}, {linear_ki}, {linear_kd}, {angular_kp}, {angular_ki}, {angular_kd}""".format(**config))
    # update pid parameters
    pid_linear.tunings = [float(config.get(key)) for key in ['linear_kp', 'linear_ki', 'linear_kd']]
    pid_angular.tunings = [float(config.get(key)) for key in ['angular_kp','angular_ki','angular_kd']]
    return config

if __name__ == '__main__':
    rospy.init_node('leader_follower')

    # get parameters
    target_frame = rospy.get_param('~target_frame')
    follower_robot_name = rospy.get_param('~follower_robot_name')
    set_distance = rospy.get_param('~set_distance')

    # create transform listener
    listener = tf.TransformListener()

    # create publisher
    follower_vel = rospy.Publisher(follower_robot_name + '/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)

    # create dynamic reconfigure server
    srv = Server(tf_pidConfig, callback)


    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(follower_robot_name+'/base_link', target_frame, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # calculate linear and angular velocity
        dis = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        # math.atan2(y, x) returns the angle between the positive x-axis and the ray from (0,0) to (x,y)
        angular = pid_angular(-math.atan2(trans[1],trans[0]))
        linear =  pid_linear(set_distance - dis)
        print("dis: ", round(dis, 2), "angular: ", round(angular, 2), "deg: ", round(-math.atan2(trans[1],trans[0])/math.pi*180, 2))
        
        # if the distance is too small, stop the robot
        if abs(linear) < 0.02:
            linear = 0

        # publish velocity
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        follower_vel.publish(msg)

        rate.sleep()