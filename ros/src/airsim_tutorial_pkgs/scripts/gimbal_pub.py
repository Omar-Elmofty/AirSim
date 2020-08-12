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
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, TwistStamped
from dji_osdk_ros.srv import Activation, ActivationResponse
from dji_osdk_ros.msg import Gimbal
from sensor_msgs.msg import Image
import time
import copy
import tf
import airsim

class GimbalPub(object):
	def __init__(self):

		#Publishers
		self.gimbal_angle_pub = rospy.Publisher('/dji_sdk/gimbal_angle', Vector3Stamped, queue_size=32)
		self.vehicle_attitude_pub = rospy.Publisher('/dji_sdk/attitude', QuaternionStamped, queue_size=32)
		self.activation_srv = rospy.Service('dji_sdk/activation',Activation, self.activation_resp)

		self.gimbal_cmd_sub = rospy.Subscriber('/dji_sdk/gimbal_angle_cmd', Gimbal, self.gimbal_angle_cmd_cb)

		self.gimbal_angle_msg = Vector3Stamped()

		# connect to the AirSim simulator
		self.client = airsim.MultirotorClient()
		self.client.confirmConnection()
		self.client.enableApiControl(True, vehicle_name='drone')
		self.client.armDisarm(True,  vehicle_name='drone')

		self.pitch = 0.0
		self.roll = 0.0
		self.yaw = 0.0

		self.pitch_cmd = 0.52
		self.roll_cmd = 0.0
		self.yaw_cmd = 0.0


		#All fixed transforms
		self.T_left_sensorNED = np.identity(4)
		self.T_left_sensorNED[0:3,3] = np.array([0.0, -0.06, 0.0])

		self.T_right_sensorNED = np.identity(4)
		self.T_right_sensorNED[0:3,3] = np.array([0.0, 0.06, 0.0])

		self.T_sensor_pitch = np.identity(4)
		self.T_sensor_pitch[0:3,3] = np.array([0.05, -0.06, -0.02])

		self.T_link_control = np.identity(4)
		self.T_link_control[0:3,3] = np.array([0.0, 0.0, -0.04])

		self.T_control_NED = np.array([[1, 0, 0 ,0],
								  [0,-1, 0, 0],
								  [0, 0,-1, 0],
								  [0, 0, 0, 1]])

		self.T_sensorNED_sensor = np.array([[1, 0, 0 ,0],
								  [0,-1, 0, 0],
								  [0, 0,-1, 0],
								  [0, 0, 0, 1]])

	def activation_resp(self,msg):
		resp = ActivationResponse()
		resp.result = True
		return resp

	def gimbal_angle_cmd_cb(self, msg):
		self.pitch_cmd = -msg.pitch
		self.roll_cmd = msg.roll
		self.yaw_cmd = -msg.yaw

	def publish_vehicle_attitude(self):

		T_rot = np.array([[1, 0, 0 ,0],
								  [0,-1, 0, 0],
								  [0, 0,-1, 0],
								  [0, 0, 0, 1]])


		pose = self.client.simGetVehiclePose(vehicle_name='drone')

		w = pose.orientation.w_val
		x = pose.orientation.x_val
		y = pose.orientation.y_val
		z = pose.orientation.z_val

		R = tf.transformations.quaternion_matrix([x,y,z,w])

		attitude = T_rot.dot(R).dot(T_rot)

		q = tf.transformations.quaternion_from_matrix(attitude)

		msg = QuaternionStamped()

		msg.quaternion.x = q[0]
		msg.quaternion.y = q[1]
		msg.quaternion.z = q[2]
		msg.quaternion.w = q[3]

		self.vehicle_attitude_pub.publish(msg)


	def gimbal_control(self):

		rate = rospy.Rate(200)


		while not rospy.is_shutdown():
			if abs(self.pitch - self.pitch_cmd) > 0.02:
				self.pitch += 0.001* (self.pitch_cmd - self.pitch)/abs(self.pitch - self.pitch_cmd)

			if abs(self.roll - self.roll_cmd) > 0.02:
				self.roll += 0.001* (self.roll_cmd - self.roll)/abs(self.roll - self.roll_cmd)

			if abs(self.yaw - self.yaw_cmd) > 0.02:
				self.yaw += 0.001* (self.yaw_cmd - self.yaw)/abs(self.yaw - self.yaw_cmd)

			T_pitch_roll = np.array([[np.cos(self.pitch),0,np.sin(self.pitch),0.16],
									 [0,1,0,0.11],
									 [-np.sin(self.pitch),0, np.cos(self.pitch),0],
									 [0,0,0,1]])

			T_roll_yaw = np.array([[1,0,0,-0.087],
								   [0,np.cos(self.roll),-np.sin(self.roll),0],
								   [0,np.sin(self.roll),np.cos(self.roll),-0.2],
								   [0,0,0,1]])

			T_yaw_link = np.array([[np.cos(self.yaw), -np.sin(self.yaw), 0, 0],
								   [np.sin(self.yaw), np.cos(self.yaw), 0, 0],
								   [0,0,1,-0.021],
								   [0,0,0,1]])
			


			T_sensorNED_NED = self.T_control_NED.dot(self.T_link_control).dot(T_yaw_link).dot(T_roll_yaw).dot(T_pitch_roll).dot(self.T_sensor_pitch).dot(self.T_sensorNED_sensor)
			T_left_NED = T_sensorNED_NED.dot(self.T_left_sensorNED)
			T_right_NED = T_sensorNED_NED.dot(self.T_right_sensorNED)

			left_euler = tf.transformations.euler_from_matrix(T_left_NED,'ryxz')
			left_trans = T_left_NED[0:3,3]

			right_euler = tf.transformations.euler_from_matrix(T_right_NED,'ryxz')
			right_trans = T_right_NED[0:3,3]

			camera_pose = airsim.Pose(airsim.Vector3r(left_trans[0], left_trans[1], left_trans[2]), airsim.to_quaternion(left_euler[0],left_euler[1] , left_euler[2]))
			self.client.simSetCameraPose("front_left_custom", camera_pose)

			camera_pose = airsim.Pose(airsim.Vector3r(right_trans[0], right_trans[1], right_trans[2]), airsim.to_quaternion(right_euler[0],right_euler[1] , right_euler[2]))
			self.client.simSetCameraPose("front_right_custom", camera_pose)

			#Publish gimbal state
			self.gimbal_angle_msg.vector.z = self.yaw*180/np.pi
			self.gimbal_angle_msg.vector.y = self.pitch*180/np.pi
			self.gimbal_angle_msg.vector.x = self.roll*180/np.pi

			self.gimbal_angle_pub.publish(self.gimbal_angle_msg)

			#publish vehicle attitude
			self.publish_vehicle_attitude()

			rate.sleep()

if __name__ == '__main__':
    rospy.init_node('gimbal_pub')
    s =  GimbalPub()
    s.gimbal_control()
    rospy.spin()



