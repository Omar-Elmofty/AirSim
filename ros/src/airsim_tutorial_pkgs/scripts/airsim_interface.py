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
from airsim_ros_pkgs.msg import *
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
import time
import copy
import airsim
from asrl__messages.msg import TrackingStatus
from dji_osdk_ros.srv import SDKControlAuthority, SDKControlAuthorityResponse
from sensor_msgs.msg import Joy
from asrl__messages.msg import TrackingStatus
from std_msgs.msg import UInt8


class AirsimInterface(object):
	def __init__(self):

		# services
		self.local_pos_over_srv = rospy.ServiceProxy('/airsim_node/drone/local_position_goal/override', SetLocalPosition)
		self.control_auth_srv = rospy.Service('dji_sdk/sdk_control_authority',SDKControlAuthority, self.set_control_authority)
		self.request_ctrl_srv = rospy.ServiceProxy('/request_control', 
                                             SDKControlAuthority)
		#Publishers
		self.gimbal_cmd_pub = rospy.Publisher('/airsim_interface/gimbal_angle_cmd', TwistStamped, queue_size = 30)
		self.airsim_vel_cmd = rospy.Publisher('/airsim_node/drone/vel_cmd_world_frame', VelCmd, queue_size = 30)
		self.display_mode_pub = rospy.Publisher('/dji_sdk/display_mode', UInt8, queue_size=30)


		#Subscribers
		#self.cmd_vel_sub = rospy.Subscriber('/cmd_vel',TwistStamped, self.cmd_vel_cb)
		self.sub_nav = rospy.Subscriber('/ShamNav/out/tracker_status',
                                    TrackingStatus,
                                    self.update_nav_status)
		self.pub_ctrl = rospy.Subscriber('/dji_sdk/flight_control_setpoint_generic', Joy, self.ctrl_cb)

		# connect to the AirSim simulator
		self.client = airsim.MultirotorClient()
		self.client.confirmConnection()
		self.client.enableApiControl(True, vehicle_name='drone')
		self.client.armDisarm(True,  vehicle_name='drone')

		#capture vt&R state
		self.state = ""

		#capture vt&r commands
		self.roll = 0
		self.pitch = 0
		self.yaw_rate = 0
		self.z_rate = 0

		#Authorize Control
		self.control_auth = 0

	def ctrl_cb(self, msg):
		self.roll = msg.axes[0]
		self.pitch = msg.axes[1]
		self.z_rate = msg.axes[2]
		self.yaw_rate = msg.axes[3]

	def set_control_authority(self,req):
		if req.control_enable:
			self.control_auth = 1
		else:
			self.control_auth = 0

		response = SDKControlAuthorityResponse()
		response.result = True
		return response

	def update_nav_status(self, msg):
		#callback for navigation status
		self.state = msg.state


	def cmd_vel_cb(self,msg):
		#callback for cmd_vel
		self.roll = msg.twist.linear.x
		self.pitch = msg.twist.linear.y
		self.yaw_rate = msg.twist.angular.z
		self.z_rate = msg.twist.linear.z


	def set_vel(self, max_angle = 0.5, speed = 1.0):
		rate = rospy.Rate(50)
		ang = 0.0
		rad = 30.

		while not rospy.is_shutdown():
			vx = speed*np.cos(ang)
			vy = speed*np.sin(ang)
			#self.pub_gimbal(ang)

			if ang < max_angle:
				ang += speed/rad/50.0*0.25 # divide by rate and multipy by simulation speed
			else:
				print('reached end')
				break

			msg = VelCmd()
			msg.twist.linear.x = vx
			msg.twist.linear.y = vy
			msg.twist.angular.z = speed/rad

			self.airsim_vel_cmd.publish(msg)

			rate.sleep()

	def set_local_over_pos(self, max_angle = 2.0, target_speed=0.5):
		rate = rospy.Rate(10)

		rad = 30 #radius of 30 m
		ang = 0.0 # angle
		speed = 0.0

		while not rospy.is_shutdown():
			x = rad*np.cos(ang) - rad
			y = rad*np.sin(ang)

			response = self.local_pos_over_srv(y,-x,-10,ang,'drone')
			print('Set Pos response: ', response)

			if speed < target_speed:
				speed += 0.1/10.0
			
			if ang < max_angle:
				ang += speed / rad /10.0
			else:
				self.client.hoverAsync(vehicle_name="drone")
				print('reached end')
				break

			rate.sleep()

	def follow_path(self):
		rate = rospy.Rate(50)
		

		while not rospy.is_shutdown():
			self.client.moveByRollPitchYawrateZrateAsync(float(self.roll), float(self.pitch), self.yaw_rate, float(-self.z_rate), float(1.0/50.0)).join()
			if self.roll !=0 and self.control_auth == 0:
				response = self.request_ctrl_srv(1)
				print(response.result)

			self.display_mode_pub.publish(17)
			print(self.control_auth)
			if self.state == "::Hover::MetricLocalize":
				#reached end of follow run
				print('reached goal')
				self.client.hoverAsync(vehicle_name="drone")
				break

			rate.sleep()

	def update_angle_gains(self):

		kp = 2.0
		kd = 0.1
		ki=10.0
		gains = airsim.AngleLevelControllerGains(roll_gains = airsim.PIDGains(kp, ki, kd),
                       pitch_gains = airsim.PIDGains(kp, ki, kd),
                       yaw_gains = airsim.PIDGains(kp, ki, kd))

		self.client.setAngleLevelControllerGains(gains,vehicle_name="drone")

	def update_anglerate_gains(self):
		kp = 1.0 #0.25
		kd = 0.0
		gains = airsim.AngleRateControllerGains(roll_gains = airsim.PIDGains(kp, 0, kd),
                       pitch_gains = airsim.PIDGains(kp, 0, kd),
                       yaw_gains = airsim.PIDGains(10.0, 0, 0.1))

		self.client.setAngleRateControllerGains(gains,vehicle_name="drone")

	def update_position_gains(self):
		kp = 0.1
		kd = 0.001
		gains = airsim.PositionControllerGains(x_gains = airsim.PIDGains(0.002, 0, kd),
                       y_gains = airsim.PIDGains(0.002, 0, kd),
                       z_gains = airsim.PIDGains(0.1, 0, 0.1))

		self.client.setPositionControllerGains(gains,vehicle_name="drone")

	def update_velocity_gains(self):
		kp = 0.2
		kd = 0.0
		ki = 0.2
		gains = airsim.VelocityControllerGains(x_gains = airsim.PIDGains(kp, ki, kd),
                       y_gains = airsim.PIDGains(kp, ki, kd),
                       z_gains = airsim.PIDGains(kp, 2.0, kd))

		self.client.setVelocityControllerGains(gains,vehicle_name="drone")

	def pub_gimbal(self, ang):
		msg = TwistStamped()

		msg.twist.angular.y = -0.32-ang/3.0
		msg.twist.angular.x = 0.0
		msg.twist.angular.z = -ang/2.0

		self.gimbal_cmd_pub.publish(msg)

	def takoff(self):
		#response = self.local_pos_over_srv(0,0,-10,0,'drone')
		self.client.moveToPositionAsync(0, 0,-10, 5.0, vehicle_name="drone")
		#wait till vehicle takes off
		rospy.sleep(15)




if __name__ == '__main__':
    # write code to create ROSControllerNode
    rospy.init_node('airsim_interface')
    inter = AirsimInterface()
    inter.update_angle_gains()
    inter.update_anglerate_gains()
    inter.update_position_gains()
    inter.update_velocity_gains()
    inter.takoff()
    inter.set_vel()
    #inter.set_local_over_pos()
    inter.follow_path()
    rospy.spin()
