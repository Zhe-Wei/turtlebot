#!/usr/bin/python3
import rospy
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Point, Quaternion
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray

new_map2odom_pub = rospy.Publisher('turtlebot02/new_map2odom', PoseStamped, queue_size=10)
new_map2odom = PoseStamped()


def main():
    print(1231231)
    rospy.init_node('robot_coordinate', anonymous=True)
    r = rospy.Rate(10) # 10hz
    detection_sub = message_filters.Subscriber("turtlebot02/tag_detections", AprilTagDetectionArray)
    odom_sub = message_filters.Subscriber("turtlebot02/odom", Odometry)
    map2odom_sub = message_filters.Subscriber("turtlebot02/map2odom", PoseStamped)

    time_sync = message_filters.ApproximateTimeSynchronizer([detection_sub, odom_sub, map2odom_sub], 10, 0.1, allow_headerless=True) 
    time_sync.registerCallback(my_callback)
    rospy.spin()

def my_callback(detection, odom, map2odom):
    # print(detection)
    last_postion = [0]*3
    if detection.detections:
        last_postion[0] = odom.pose.pose.position.x
        last_postion[1] = odom.pose.pose.position.y
        last_postion[2] = odom.pose.pose.position.z
        
        new_map2odom.header.stamp = rospy.Time.now()
        new_map2odom.header.frame_id = "map"
        print(map2odom)
        new_map2odom.pose.position.x = map2odom.pose.position.x
        new_map2odom.pose.position.y = map2odom.pose.position.y
        new_map2odom.pose.position.z = map2odom.pose.position.z
        new_map2odom.pose.orientation.w = map2odom.pose.orientation.w
        new_map2odom.pose.orientation.x = map2odom.pose.orientation.x
        new_map2odom.pose.orientation.y = map2odom.pose.orientation.y
        new_map2odom.pose.orientation.z = map2odom.pose.orientation.z
        new_map2odom_pub.publish(new_map2odom)

    else:
        curr_postion = [0]*7
        curr_postion[0] = odom.pose.pose.position.x
        curr_postion[1] = odom.pose.pose.position.y
        curr_postion[2] = odom.pose.pose.position.z
        curr_postion[3] = odom.pose.pose.orientation.w
        curr_postion[4] = odom.pose.pose.orientation.x
        curr_postion[5] = odom.pose.pose.orientation.y
        curr_postion[6] = odom.pose.pose.orientation.z


        curr_map2odom = [0]*3
        curr_map2odom[0] = map2odom.pose.position.x
        curr_map2odom[1] = map2odom.pose.position.y
        curr_map2odom[2] = map2odom.pose.position.z

        
        new_map2odom.header.stamp = rospy.Time.now()
        new_map2odom.header.frame_id = "map"
        new_map2odom.pose.position.x = curr_map2odom[0] + (curr_postion[0] - last_postion[0])
        new_map2odom.pose.position.y = curr_map2odom[1] + (curr_postion[1] - last_postion[1])
        new_map2odom.pose.position.z = curr_map2odom[2] + (curr_postion[2] - last_postion[2])
        new_map2odom.pose.orientation.x = curr_postion[3]
        new_map2odom.pose.orientation.y = curr_postion[4]
        new_map2odom.pose.orientation.z = curr_postion[5]
        new_map2odom.pose.orientation.w = curr_postion[6]
        
        new_map2odom_pub.publish(new_map2odom)

        # print(curr_map2odom + (curr_postion - last_postion))
if __name__=='__main__':
    main()