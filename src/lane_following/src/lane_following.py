#!/usr/bin/env python3
import os
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

import tf
# import turtlesim.msg
cmd_vel_pub = rospy.Publisher("/turtlebot02/cmd_vel", Twist, queue_size=10)

# os.environ['DISPLAY'] = ':0'
HOSTNAME = os.getenv('HOSTNAME')
print(HOSTNAME)
class lanefollowing:
	def __init__(self):
		self.bridge_ = CvBridge()
		self.twist = Twist()

	def callback(self, data):
		# br = tf.TransformBroadcaster()
		# br.sendTransform((msg.x, msg.y, 0),
		# 				tf.transformations.quaternion_from_euler(0, 0, msg.theta),
		# 				rospy.Time.now(),
		# 				turtlename,
		# 				"world")
		# print("revived")
		self.latest_image_ = data
		try:
			cv_image = self.bridge_.compressed_imgmsg_to_cv2(self.latest_image_)
			original = cv_image.copy()
			image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
			
			# red mask
			lower_red = np.array([0,50,50])
			upper_red = np.array([10,255,255])
			mask0 = cv2.inRange(image, lower_red, upper_red)
			lower_red = np.array([170,50,50])
			upper_red = np.array([180,255,255])
			mask1 = cv2.inRange(image, lower_red, upper_red)
			mask = mask0+mask1

			# yellow mask
			# lower = np.array([22, 93, 0], dtype="uint8")
			# upper = np.array([45, 255, 255], dtype="uint8")
			# mask = cv2.inRange(image, lower, upper)

			h, w, d= cv_image.shape
			search_top = int(4*h/5)
			search_bot = int(4*h/5) + 20
			mask[0:search_top, 0:w] = 0
			mask[search_bot:h, 0:w] = 0

			M = cv2.moments(mask)
			# print(M)
			if M['m00'] > 0:
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				cv2.circle(original, (cx, cy), 20, (0,0,255), -1)

				err = cx - w/2
				# print(err)
				self.twist.linear.x = 0.1
				self.twist.angular.z = -float(err) / 600
				cmd_vel_pub.publish(self.twist)

			# self.result_pub_.publish(self.bridge_.cv2_to_compressed_imgmsg(original))
		except CvBridgeError as e:
			print(e)
if __name__ == '__main__':
	rospy.init_node('lane_following')
	rate = rospy.Rate(1)

	lf = lanefollowing()

	sub = rospy.Subscriber("/turtlebot02/usb_cam/image_raw/compressed", CompressedImage, lf.callback)
	# result_pub_ = rospy.Publisher("~image/compressed", CompressedImage, queue_size=1000000)
	

	rospy.spin()
