#!/usr/bin/env python2

"""
ROS Node for interfacing airsim with the VT&R
"""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

# Import class that computes the desired positions
from airsim_ros_pkgs.srv import *
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import time
import copy

class StereoCalib(object):
	def __init__(self):

		self.cam_fps_pub = rospy.Publisher('/airsim_interface/cam_fps_period', Twist, queue_size=32)

		
		self.left_img_sub = rospy.Subscriber('/airsim_node/drone/front_left_custom/Scene',Image, self.left_img_cb)
		self.right_img_sub = rospy.Subscriber('/airsim_node/drone/front_right_custom/Scene',Image, self.right_img_cb)

		self.left_info_sub = rospy.Subscriber('/airsim_node/drone/front_left_custom/Scene/camera_info',CameraInfo, self.left_info_cb)
		self.right_info_sub = rospy.Subscriber('/airsim_node/drone/front_right_custom/Scene/camera_info',CameraInfo, self.right_info_cb)

		self.left_img_pub = rospy.Publisher('/airsim_interface/left_camera_img', Image, queue_size=32)
		self.right_img_pub = rospy.Publisher('/airsim_interface/right_camera_img', Image, queue_size=32)

		self.left_info_pub = rospy.Publisher('/airsim_interface/left_camera_info', CameraInfo, queue_size=32)
		self.right_info_pub = rospy.Publisher('/airsim_interface/right_camera_info', CameraInfo, queue_size=32)

		self.left_info_msg = CameraInfo()
		self.right_info_msg = CameraInfo()
		self.left_img = Image()

		self.prev_time = 0.0
		self.new_time = 0.0

		self.idx = 1
		


	def left_info_cb(self, msg):
		self.left_info_msg = msg

	def right_info_cb(self, msg):
		self.right_info_msg = msg

	def left_img_cb(self, msg):
		self.left_img = msg
		# self.left_img_pub.publish(msg)
		# print(msg.header.seq)
		# self.left_info_msg.D = [0.0, 0.0, 0.0,0.0,0.0]
		# self.left_info_msg.R = [1.0, 0.0, 0.0,0.0,1.0, 0.0, 0.0, 0.0, 1.0]
		# self.left_info_pub.publish(self.left_info_msg)

	def right_img_cb(self, msg):
		if self.prev_time == 0 and self.new_time ==0:
			self.prev_time = rospy.Time.now().to_sec()
		else:
			self.new_time = rospy.Time.now().to_sec()

			dt = (self.new_time - self.prev_time)
			twist_msg = Twist()
			twist_msg.linear.x = float(dt)
			self.cam_fps_pub.publish(twist_msg)

			self.prev_time = self.new_time

		msg.header.seq = self.idx
		msg.header.stamp = rospy.Time.now()
		self.right_img_pub.publish(msg)
		self.left_img.header.seq = self.idx
		self.left_img.header.stamp = msg.header.stamp
		self.left_img_pub.publish(self.left_img)
		self.idx += 1

		self.right_info_msg.header.stamp = msg.header.stamp
		self.right_info_msg.D = [0.0, 0.0, 0.0,0.0,0.0]
		self.right_info_msg.R = [1.0, 0.0, 0.0,0.0,1.0, 0.0, 0.0, 0.0, 1.0]
		self.right_info_msg.P = [336.0, 0.0, 336.0, -336.0*0.12, 0.0, 336.0, 188.0, 0.0, 0.0, 0.0, 1.0, 0.0]
		self.right_info_pub.publish(self.right_info_msg)


		self.left_info_msg.header.stamp = msg.header.stamp
		self.left_info_msg.D = [0.0, 0.0, 0.0,0.0,0.0]
		self.left_info_msg.R = [1.0, 0.0, 0.0,0.0,1.0, 0.0, 0.0, 0.0, 1.0]
		self.left_info_pub.publish(self.left_info_msg)





	# def set_local_over_pos(self):
	# 	rate = rospy.Rate(1)
	# 	x_dist = 2*np.random.normal()
	# 	y_dist = 2*np.random.normal()
	# 	ang = 0.0
	# 	rad = 30
	# 	y=10
	# 	x=0
	# 	ang=0
	# 	while not rospy.is_shutdown():
	# 		# x = rad*np.cos(ang) - rad
	# 		# y = rad*np.sin(ang)
			

	# 		response = self.local_pos_over_srv(y,-x,-10,ang,'drone')
	# 		print('Set Pos response: ', response)
	# 		rate.sleep()

	# 		if y>0:
	# 			y -=1
	# 		else:
	# 			print("y=10")
	# 			break

	# 		# if y<10:
	# 		# 	y +=1
	# 		# else:
	# 		# 	print("y=10")
	# 		# 	break

	# 		#ang += 0.01


if __name__ == '__main__':
    rospy.init_node('StereoCalib')
    s =  StereoCalib()
    rospy.spin()
