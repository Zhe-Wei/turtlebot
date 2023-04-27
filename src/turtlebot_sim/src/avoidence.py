#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from wheeltec_multi.msg import avoid
import math
from turtlebot_sim.msg import position as PositionMsg

cmd_vel_msg = Twist()    # 速度控制信息数据
cmd_vel_avoid = Twist()    # 速度控制信息数据
cmd_vel_data = Twist()    # 速度控制信息数据

distance1 = 100    # 障碍物距离
dis_angleX = 0 #障碍物方向,前面为0度角，右边为正，左边为负	
safe_distance = 0
danger_distance = 0
danger_angular = 0
avoidance_kv = 0
avoidance_kw = 0
max_vel_x = 0
min_vel_x = 0
max_vel_theta = 0
min_vel_theta = 0

def current_position_Callback(msg):
    global distance1, dis_angleX
    distance1 = msg.distance
    dis_angleX = msg.angleX
    if dis_angleX > 0:
        dis_angleX = 3.1415 - dis_angleX
    else:
        dis_angleX = -(dis_angleX + 3.1415)
    # print(dis_angleX)
def cmd_vel_ori_Callback(msg):
    global cmd_vel_msg, cmd_vel_data
    cmd_vel_msg.linear.x = msg.linear.x
    cmd_vel_msg.angular.z = msg.angular.z

    cmd_vel_data.linear.x = msg.linear.x
    cmd_vel_data.angular.z = msg.angular.z

def main():
    temp_count = 0  # 计数变量
    rospy.init_node('avoidance')  # 初始化 ROS 节点
    global distance1, dis_angleX, safe_distance, danger_distance, danger_angular, avoidance_kv, avoidance_kw, max_vel_x, min_vel_x, max_vel_theta, min_vel_theta
    safe_distance = rospy.get_param('~safe_distance', 0.5)  # 安全距离，默认 0.5 m
    danger_distance = rospy.get_param('~danger_distance', 0.2)  # 危险距离，默认 0.2 m
    danger_angular = rospy.get_param('~danger_angular', 0.785)  # 危险航向角，默认 0.785 rad
    avoidance_kv = rospy.get_param('~avoidance_kv', 0.2)  # 速度控制系数，默认 0.2
    avoidance_kw = rospy.get_param('~avoidance_kw', 0.5)  # 航向角控制系数，默认 0.3
    max_vel_x = rospy.get_param('~max_vel_x', 1.5)  # 最大线速度，默认 1.5 m/s
    min_vel_x = rospy.get_param('~min_vel_x', 0.05)  # 最小线速度，默认 0.05 m/s
    max_vel_theta = rospy.get_param('~max_vel_theta', 1.5)  # 最大角速度，默认 1.5 rad/s
    min_vel_theta = rospy.get_param('~min_vel_theta', 0.05)  # 最小角速度，默认 0.05 rad/s
    # rospy.loginfo("avoidance_kv = %f", avoidance_kv)
    # rospy.loginfo("avoidance_kw = %f", avoidance_kw)

    cmd_vel_msg = Twist()  # 初始化速度控制消息
    cmd_vel_Pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)  # 创建速度控制话题发布者
    vel_sub = rospy.Subscriber('cmd_vel_ori', Twist, cmd_vel_ori_Callback)  # 创建底盘运动话题订阅者
    current_position_sub = rospy.Subscriber('object_tracker/current_position', PositionMsg, current_position_Callback)  # 创建障碍物方位话题订阅者
    
    rate = 10  # 频率 10 Hz
    loopRate = rospy.Rate(rate)
    
    while not rospy.is_shutdown():
        avoidance_kw = abs(avoidance_kw)
        # rospy.spinOnce()
        if(distance1 < safe_distance and distance1 > danger_distance):  # 障碍物在安全距离和危险距离之间，调整速度角度避让障碍物
            # rospy.logwarn("distance1 less than 0.6")
            # rospy.logwarn("distance1 = %f", distance1)
            rospy.logwarn("Bef:cmd_vel_data.linear.x = %f", cmd_vel_data.linear.x)
            rospy.logwarn("avoidance_kv = %f", avoidance_kv)
            rospy.logwarn("distance1 = %f", distance1)
            cmd_vel_msg.linear.x = cmd_vel_data.linear.x + avoidance_kv*math.cos(dis_angleX)/distance1  # 原始速度，减去一个后退的速度
            rospy.logwarn("Aft:cmd_vel_data.linear.x = %f", cmd_vel_data.linear.x)
            if(dis_angleX < 0):
                avoidance_kw = avoidance_kw  # 车左右边的障碍物避障，车头调转方向不一致
            cmd_vel_msg.angular.z = cmd_vel_data.angular.z - avoidance_kw*math.cos(dis_angleX)/distance1
        elif(distance1 < danger_distance):  # 障碍物在危险距离之内时，以远离障碍物为主
            # rospy.logwarn("distance1 less than 0.3")
            # rospy.logwarn("distance1 = %f", distance1)
            cmd_vel_msg.linear.x = avoidance_kv*math.cos(dis_angleX)/distance1
            if(dis_angleX < 0):
                avoidance_kw = -avoidance_kw
            cmd_vel_msg.angular.z = avoidance_kw*math.cos(dis_angleX)/distance1
        else:  # 其他情况直接输出原始速度
            cmd_vel_msg.linear.x = cmd_vel_data.linear.x
            cmd_vel_msg.angular.z = cmd_vel_data.angular.z
    
        # 速度限制
        if cmd_vel_msg.linear.x > max_vel_x:
            cmd_vel_msg.linear.x = max_vel_x
        elif cmd_vel_msg.linear.x < -max_vel_x:
            cmd_vel_msg.linear.x = -max_vel_x
        if abs(cmd_vel_msg.linear.x) < min_vel_x:
            cmd_vel_msg.linear.x = 0
        if cmd_vel_msg.angular.z > max_vel_theta:
            cmd_vel_msg.angular.z = max_vel_theta
        elif cmd_vel_msg.angular.z < -max_vel_theta:
            cmd_vel_msg.angular.z = -max_vel_theta
        if abs(cmd_vel_msg.angular.z) < min_vel_theta:
            cmd_vel_msg.angular.z = 0
    
        cmd_vel_Pub.publish(cmd_vel_msg)
        loopRate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass