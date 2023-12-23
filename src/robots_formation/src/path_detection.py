#!/usr/bin/env python3

import rospy
import math
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped

def deg2index(deg, scan_data):
    return math.floor(math.radians(deg) / scan_data.angle_increment)

def getPriotity(real_tag_1_pos, real_tag_2_pos):

    # calculate Euclidean distance (leader tag_0 -> follower tag_1) and (leader tag_0 -> follower tag_2)
    dis_1 = math.sqrt((real_tag_1_pos[0])**2 + (real_tag_1_pos[1])**2)
    dis_2 = math.sqrt((real_tag_2_pos[0])**2 + (real_tag_2_pos[1])**2)

    # calculate priority
    if dis_1 < dis_2:
        priority = 1
    elif dis_1 > dis_2:
        priority = 2
    else:
        priority = 1

    rospy.loginfo(f"priority: {priority}")

    return priority

def getPathWidth(scan_data):
    # 設定需要計算寬度的角度
    deg = 60

    # 取得左邊和右邊的 LiDAR 數據
    left_data = scan_data.ranges[deg2index(20, scan_data):deg2index(90, scan_data)]
    right_data = scan_data.ranges[deg2index(270, scan_data):deg2index(340, scan_data)]

    # 清理數據，移除無效值
    left_data = [10 if x == 0 else x for x in left_data]
    right_data = [10 if x == 0 else x for x in right_data]

    # 清理數據，將小於 0.1 的值設置為 10
    left_data = [10 if x < 0.14 else x for x in left_data]
    right_data = [10 if x < 0.14 else x for x in right_data]
    

    # 計算路徑寬度
    path_widths = []
    for i in range(len(left_data)):
        c = math.sqrt((left_data[i]**2 + right_data[len(left_data)-i-1]**2) - 2 * left_data[i] * right_data[len(left_data)-i-1] * math.cos(math.radians(2 * deg)))

        path_widths.append(c)

    # 返回最小的路徑寬度
    return min(path_widths)

def adjust_and_publish_tf(tf_listener, tf_broadcaster, original_frame, new_frame, adjusted_trans):
    try:
        (_, rot) = tf_listener.lookupTransform('/tag_0', original_frame, rospy.Time(0))

        tf_broadcaster.sendTransform(adjusted_trans, rot, rospy.Time.now(), new_frame, "/tag_0")
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

def publish_original_tf(tf_listener, tf_broadcaster, original_frame, new_frame):
    try:
        (trans, rot) = tf_listener.lookupTransform('/tag_0', original_frame, rospy.Time(0))
        tf_broadcaster.sendTransform(trans, rot, rospy.Time.now(), new_frame, "/tag_0")
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

def lidar_callback(scan_data):
    path_width = getPathWidth(scan_data)
    rospy.loginfo(f"Path width: {path_width} meters")

    try:
        # 獲取初始陣形TF的位置
        (tag1_pos, tag1_rot) = tf_listener.lookupTransform('/tag_0', '/ori_tag_0/tag_1', rospy.Time(0))
        (tag2_pos, tag2_rot) = tf_listener.lookupTransform('/tag_0', '/ori_tag_0/tag_2', rospy.Time(0))
        # rospy.loginfo(f"tag1_pos: {tag1_pos}")
        # rospy.loginfo(f"tag2_pos: {tag2_pos}")
        
        # 獲取實際陣行TF的位置
        (real_tag1_pos, real_tag1_rot) = tf_listener.lookupTransform('/tag_0', '/tag_1', rospy.Time(0))
        (real_tag2_pos, real_tag2_rot) = tf_listener.lookupTransform('/tag_0', '/tag_2', rospy.Time(0))


        # 計算y軸寬度總和
        y_width_sum = abs(tag1_pos[1]) + abs(tag2_pos[1]) + 0.15
        rospy.loginfo(f"y_width_sum: {y_width_sum} meters")
        if y_width_sum > path_width:

            # get priority
            priority = getPriotity(real_tag1_pos, real_tag2_pos)

            # tmp
            tmp1_trans = list(tag1_pos)
            tmp2_trans = list(tag2_pos)

            # 根據優先級調整TF位置
            rospy.loginfo("Adjusting TF positions inward")

            # 計算l_1, l_2
            l_1 = math.sqrt((tag1_pos[0])**2 + (tag1_pos[1])**2)
            l_2 = math.sqrt((tag2_pos[0])**2 + (tag2_pos[1])**2)


            d_i = path_width - r_p
            d_j = path_width - r_p

            if priority == 2:
                # 更新tag2的位置
                tmp2_trans[0] = tag2_pos[0]
                tmp2_trans[1] = tag2_pos[1] + d_i
                if (tmp2_trans[1] > 0):
                    tmp2_trans[1] = 0

                # 更新tag1的位置
                # tmp2_trans[0] = math.sqrt((2*r_p)**2 - (d_i+d_j)**2)
                tmp1_trans[0] = tag2_pos[0] - 1.5*r_p  
                tmp1_trans[1] = tag1_pos[1] - d_i
                if (tmp1_trans[1] < 0):
                    tmp1_trans[1] = 0
            elif priority == 1:
                tmp1_trans[0] = tag1_pos[0]
                tmp1_trans[1] = tag1_pos[1] - d_i
                if (tmp1_trans[1] < 0):
                    tmp1_trans[1] = 0

                tmp2_trans[0] = tag2_pos[0] - 1.5*r_p  
                tmp2_trans[1] = tag2_pos[1] + d_i

            # print
            print(tmp1_trans)
            print(tmp2_trans)

            adjust_and_publish_tf(tf_listener, tf_broadcaster, '/ori_tag_0/tag_1', '/tag_0/tag_1', tmp1_trans)
            adjust_and_publish_tf(tf_listener, tf_broadcaster, '/ori_tag_0/tag_2', '/tag_0/tag_2', tmp2_trans)
        else:
            rospy.loginfo("Using original TF positions")
            publish_original_tf(tf_listener, tf_broadcaster, '/ori_tag_0/tag_1', '/tag_0/tag_1')
            publish_original_tf(tf_listener, tf_broadcaster, '/ori_tag_0/tag_2', '/tag_0/tag_2')
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

def listener():
    # 初始化 ROS 節點
    rospy.init_node('lidar_listener', anonymous=True)
    global tf_listener
    tf_listener = tf.TransformListener()

    global tf_broadcaster
    tf_broadcaster = tf.TransformBroadcaster()

    # 訂閱 /scan topic
    rospy.Subscriber("/scan", LaserScan, lidar_callback)

    # 保持執行，直到節點被關閉
    rospy.spin()

if __name__ == '__main__':

    r_p = rospy.get_param('~r_p', 0.4)

    while not rospy.is_shutdown():
        try:
            listener()
        except rospy.ROSInterruptException:
            pass
   

