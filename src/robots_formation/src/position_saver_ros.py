#!/usr/bin/env python3
import rospy
import tf
import time

def wait_and_get_transforms(listener, frame1, frame2):
    try:
        listener.waitForTransform(frame1, frame2, rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform(frame1, frame2, rospy.Time(0))
        rospy.loginfo(f"Transform obtained between {frame1} and {frame2}")
        return trans, rot
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Error getting transform")
        return None, None

def main():
    rospy.init_node('tag_transforms')
    rospy.loginfo("Initialized 'tag_transforms' node")

    # 創建tf listener 和 broadcaster
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()

    # 等待3秒以確保tf資料是最新的
    rospy.sleep(3)
    rospy.loginfo("Waited 3 seconds for tf data to update")

    # 獲取tag_0到tag_1 和 tag_0到tag_2的轉換
    trans_01, rot_01 = wait_and_get_transforms(listener, "tag_0", "tag_1")
    trans_02, rot_02 = wait_and_get_transforms(listener, "tag_0", "tag_2")

    if trans_01 and trans_02:
        rospy.loginfo("Transforms for tag_0 to tag_1 and tag_0 to tag_2 obtained")
        # 儲存轉換並發布
        while not rospy.is_shutdown():
            # 發布 tag_0 到 tag_1 的轉換
            broadcaster.sendTransform(trans_01, rot_01, rospy.Time.now(), "ori_tag_0/tag_1", "tag_0")
            # 發布 tag_0 到 tag_2 的轉換
            broadcaster.sendTransform(trans_02, rot_02, rospy.Time.now(), "ori_tag_0/tag_2", "tag_0")
            rospy.sleep(0.1)
    else:
        rospy.logerr("Failed to obtain necessary transforms")

if __name__ == '__main__':
    main()
