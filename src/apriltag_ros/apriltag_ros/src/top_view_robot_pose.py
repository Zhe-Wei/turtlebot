#!/usr/bin/python3

import rospy
import numpy as np

from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Point, Quaternion, PoseWithCovarianceStamped
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion, translation_from_matrix, quaternion_from_matrix, compose_matrix, quaternion_matrix, rotation_matrix


## Global variables
nrTfRetrys = 1
retryTime = 0.05
rospy.init_node('top_view_robot_pose', log_level=rospy.INFO, anonymous=False)

# Initializes a tf2 listener
tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)
pub = rospy.Publisher('/desktop_detection', AprilTagDetectionArray, queue_size=10)

r = rospy.Rate(10) # 10hz

def strip_forward_slash(frame_id):
    '''
    Removes forward slash for tf2 to work
    '''
    if frame_id[0] == "/":
        new_frame_id = frame_id[1:]
    else:
        new_frame_id = frame_id
    return new_frame_id

def main():
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, apriltag_callback, queue_size = 10)

    while not rospy.is_shutdown():
        r.sleep()

def apriltag_callback(data):
    if data.detections:
        top_view_map2apriltag_array = AprilTagDetectionArray()
        top_view_map2apriltag_array.header.stamp = rospy.Time.now()
        for detection in data.detections:
            tag = AprilTagDetection()
            bundle_id = "bundle_" + str(int(detection.id[0]/5))
            _pose = PoseStamped()
            transform = tf_buffer.lookup_transform("map", bundle_id, rospy.Time(0), rospy.Duration(1.0))
            # print(transform)
            pose_transformed = tf2_geometry_msgs.do_transform_pose(_pose, transform)
            # print(pose_transformed)
            # return
            tag.id = detection.id
            tag.pose.header.stamp = rospy.Time.now()
            tag.pose.header.frame_id = "desktop_camera"

            tag.pose.pose.pose.orientation.w = pose_transformed.pose.orientation.w
            tag.pose.pose.pose.orientation.x = pose_transformed.pose.orientation.x
            tag.pose.pose.pose.orientation.y = pose_transformed.pose.orientation.y
            tag.pose.pose.pose.orientation.z = pose_transformed.pose.orientation.z
            
            # e = euler_from_quaternion([pose_transformed.pose.orientation.w,
            #                             pose_transformed.pose.orientation.x,
            #                             pose_transformed.pose.orientation.y,
            #                             pose_transformed.pose.orientation.z]
            #                             )
            # M_PI = 3.1415926       
            # print(e[0]* 180.0 / M_PI, e[1]* 180.0 / M_PI, e[2]* 180.0 / M_PI)
            # print(pose_transformed.pose.position.x, pose_transformed.pose.position.y)

            tag.pose.pose.pose.position.x = pose_transformed.pose.position.x
            tag.pose.pose.pose.position.y = pose_transformed.pose.position.y
            tag.pose.pose.pose.position.z = 0
            top_view_map2apriltag_array.detections.append(tag)
        # print("pub")
        pub.publish(top_view_map2apriltag_array)
            
if __name__=='__main__':
    main()