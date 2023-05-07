#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf 
import os


rpy = [0, 0, 0]
leader_linear_x = 0
leader_angular_z = 0

class RobotController():
    def __init__(self):
        self.velocity_publisher = rospy.Publisher(leader_robot_name + '/cmd_vel', Twist, queue_size=10)
        self.current_time = rospy.Time.now().to_sec()
        self.curremt_yaw = 0
        self.rate = rospy.Rate(10) # 10hz
        self.mode = None
        self.circle_complete = False
    
    def set_mode(self, mode):
        self.mode = mode
        self.circle_complete = False

    def move_robot(self, linear_speed, angular_speed):
        velocity_msg = Twist()
        velocity_msg.linear.x = linear_speed
        velocity_msg.angular.z = angular_speed
        self.velocity_publisher.publish(velocity_msg)
    
    def readCurrentRaw(self):
        self.curremt_yaw = rpy[2]
    def run(self):
        
        # Register a shutdown callback
        def shutdown_cb():
            global leader_linear_x, leader_angular_z
            if leader_linear_x < 0.1 and leader_angular_z < 0.1:
                self.move_robot(0.0, 0.0)
            else:
                while leader_linear_x > 0.1 or leader_angular_z > 0.1:
                    if leader_linear_x > 0.1:
                        leader_linear_x = leader_linear_x - 0.05
                    if leader_angular_z > 0.1:
                        leader_angular_z = leader_angular_z - 0.2
                    rospy.sleep(0.05)
                    self.move_robot(leader_linear_x, leader_angular_z)
                self.move_robot(0.0, 0.0)
        rospy.on_shutdown(shutdown_cb)


        # Run the robot
        while not rospy.is_shutdown():
            if self.mode == 'straight':
                self.move_robot(straight_linear_speed, 0) # move straight at xx m/s
            elif self.mode == 'circle':
                self.move_robot(circle_linear_speed, circle_angular_speed)
            else:
                rospy.logwarn('Invalid mode selected')
            
            elapsed_time = rospy.Time.now().to_sec() - self.current_time
            if elapsed_time > 10.0:
                if self.mode == 'circle' and not self.circle_complete:
                    if rpy[2] < 0.1 and rpy[2] > -0.05:
                        self.circle_complete = True
                        # self.move_robot(0.0, 0.0)
                        rospy.loginfo('Circle complete')
                        rospy.signal_shutdown('Circle complete')
            self.rate.sleep()

def odom_cb(msg):
    # print(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
    global rpy
    rpy = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

def cmd_vel_cb(msg):
    global leader_linear_x, leader_angular_z
    leader_linear_x = msg.linear.x
    leader_angular_z = msg.angular.z

if __name__ == '__main__':

    # Initialize the node
    rospy.init_node('move_leader_robot')
    
    # Get the parameters
    leader_robot_name = rospy.get_param('~leader_robot_name', "robot_0")
    mode = rospy.get_param('~mode', "circle") # 'circle' or 'straight'
    straight_linear_speed = rospy.get_param('~straight_linear_speed', 0.5)
    circle_linear_speed = rospy.get_param('~circle_linear_speed', 0.5)
    circle_angular_speed = rospy.get_param('~circle_angular_speed', 0.1)

    # Subscribe to the odom topic
    leader_odom = rospy.Subscriber(leader_robot_name + '/odom', Odometry, odom_cb)

    # Subscribe to the cmd_vel topic
    leader_vel = rospy.Subscriber(leader_robot_name + '/cmd_vel', Twist, cmd_vel_cb)
    
    # Wait for the time to be published
    rospy.sleep(2)

    # Create the controller
    controller = RobotController()
    controller.readCurrentRaw()

    # Set the mode (straight or circle)
    controller.set_mode('straight') # set the initial mode to 'straight'

    # Wait for the time to be published
    rospy.sleep(4)

    # Run the leader robot
    controller.run()
    