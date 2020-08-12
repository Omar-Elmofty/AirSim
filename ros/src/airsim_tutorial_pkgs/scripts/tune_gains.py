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
from geometry_msgs.msg import Twist, Vector3Stamped
import time
import copy
import airsim
from asrl__messages.msg import TrackingStatus
import tf
from asrl__pose_graph.data_bubble import ChunkStream
from babelfish.robochunk.geometry_msgs import TwistStamped, QuaternionStamped

class AirsimInterface(object):
	def __init__(self):

		#Publishers
		self.rpy_pub = rospy.Publisher('/airsim_interface/rpy', Twist, queue_size=32)
		self.cmd_vel_pub = rospy.Publisher('/airsim_interface/cmd_vel', Twist, queue_size=32)

		# connect to the AirSim simulator
		self.client = airsim.MultirotorClient()
		self.client.confirmConnection()
		self.client.enableApiControl(True, vehicle_name='drone')
		self.client.armDisarm(True,  vehicle_name='drone')

		#cmd_vel lists
		self.cmd_roll = np.empty([0, 0])
		self.cmd_pitch = np.empty([0, 0])
		self.cmd_yaw_rate = np.empty([0, 0])
		self.cmd_vel_stamps = np.empty([0, 0])
		self.cmd_z_rate = np.empty([0, 0])

	def update_angle_gains(self):

		kp = 2.0
		kd = 0.1
		ki=10.0
		gains = airsim.AngleLevelControllerGains(roll_gains = airsim.PIDGains(kp, ki, kd),
                       pitch_gains = airsim.PIDGains(kp, ki, kd),
                       yaw_gains = airsim.PIDGains(kp, ki, kd))

		self.client.setAngleLevelControllerGains(gains,vehicle_name="drone")

	def update_anglerate_gains(self):
		kp = 1.0#0.25
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
		ki = 2.0
		gains = airsim.VelocityControllerGains(x_gains = airsim.PIDGains(kp, ki, kd),
                       y_gains = airsim.PIDGains(kp, ki, kd),
                       z_gains = airsim.PIDGains(kp, ki, kd))

		self.client.setVelocityControllerGains(gains,vehicle_name="drone")

	def publish_vehicle_attitude(self):

		pose = self.client.simGetVehiclePose(vehicle_name='drone')

		w = pose.orientation.w_val
		x = pose.orientation.x_val
		y = pose.orientation.y_val
		z = pose.orientation.z_val

		rpy = tf.transformations.euler_from_quaternion([x,y,z,w])

		msg = Twist()

		msg.angular.x = rpy[0]
		msg.angular.y = -rpy[1]
		msg.angular.z = -rpy[2]

		self.rpy_pub .publish(msg)

	def excite_system(self):
		rate = rospy.Rate(50)

		start_time = rospy.Time.now().to_sec()

		roll = 0.0
		pitch = -0.1
		yaw_rate = 0
		z = 5

		while not rospy.is_shutdown():
			#self.client.moveToPositionAsync(z, 0,-15, 1.0, vehicle_name="drone")
			#self.client.moveByVelocityZAsync(0, 0,-10+z, 5.0, vehicle_name="drone").join()
			self.client.moveByRollPitchYawZAsync(float(roll), float(pitch), float(yaw_rate), float(-10.0), float(1.0/50.0)).join()
			self.publish_vehicle_attitude()
			cur_time = rospy.Time.now().to_sec()

			if (cur_time - start_time) > 10:
				start_time = rospy.Time.now().to_sec()
				roll = -roll
				pitch = -pitch
				yaw_rate = -yaw_rate
				z = -z
				#self.client.hoverAsync(vehicle_name="drone")
				#break
			rate.sleep()
	def move_to_origin(self):
		self.client.moveToPositionAsync(0, 0,-100, 5.0, vehicle_name="drone")
		#self.client.moveByVelocityZAsync(0, 0,-10, 5.0, vehicle_name="drone").join()

	def load_cmd_vel(self):

		mission_directory='/home/omar/backyard_190816_test2'

		cmd_vel_stream = ChunkStream(mission_directory +'/run_000000', 'M600/cmd_vel')
		cmd_vel_stream.open()

		# Now make an iterator with the message type
		iterator = cmd_vel_stream.iter(TwistStamped)

		# Iterate to extract the data, an exception is thrown when we get to the end
		try: 
		    while True:
		        # get the next message
		        msg = iterator.next()
		        
		        # Extract the timestamp
		        self.cmd_vel_stamps = np.append(self.cmd_vel_stamps,[msg.header.sensor_time_stamp.nanoseconds_since_epoch])
		        
		        # Extract the payload
		        msg.extract()
		        
		        # Put the extracted data into the numpy arrays
		        self.cmd_roll = np.append(self.cmd_roll,[msg.twist.linear.x])
		        self.cmd_pitch = np.append(self.cmd_pitch,[msg.twist.linear.y])
		        self.cmd_yaw_rate = np.append(self.cmd_yaw_rate,[msg.twist.angular.z])
		        self.cmd_z_rate = np.append(self.cmd_z_rate,[msg.twist.linear.z])
		        
		except StopIteration:
		    pass # We got to the end of the iterated data

	def load_m600_att(self):

		mission_directory='/home/omar/backyard_190816_test2'

		att_roll = np.empty([0, 0])
		att_pitch = np.empty([0, 0])
		att_yaw = np.empty([0, 0])
		att_stamps = np.empty([0, 0])

		att_stream = ChunkStream(mission_directory +'/run_000000', 'M600/attitude')
		att_stream.open()

		# Now make an iterator with the message type
		iterator = att_stream.iter(QuaternionStamped)

		# Iterate to extract the data, an exception is thrown when we get to the end
		try: 
		    while True:
		        # get the next message
		        msg = iterator.next()
		        
		        # Extract the timestamp
		        att_stamps = np.append(att_stamps,[msg.header.sensor_time_stamp.nanoseconds_since_epoch])
		        
		        # Extract the payload
		        msg.extract()
		        
		        q = [msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w]
		        
		        angles = tf.transformations.euler_from_quaternion(q,  axes='sxyz')
		        
		        
		        # Put the extracted data into the numpy arrays
		        att_roll = np.append(att_roll,[angles[0]])
		        att_pitch = np.append(att_pitch,[angles[1]])
		        att_yaw = np.append(att_yaw,[angles[2]])
		        
		except StopIteration:
		    pass # We got to the end of the iterated data

	def move_by_cmd_vel(self):

		#Truncate cmd_vel
		self.cmd_roll = self.cmd_roll[-2000:-1]
		self.cmd_pitch = self.cmd_pitch[-2000:-1]
		self.cmd_yaw_rate = self.cmd_yaw_rate[-2000:-1]
		self.cmd_vel_stamps = self.cmd_vel_stamps[-2000:-1]
		self.cmd_z_rate = self.cmd_z_rate[-2000:-1]

		#convert time stamps to seconds, and start at origin
		stamps_secs = (self.cmd_vel_stamps - self.cmd_vel_stamps[0])/10**9



		#Inital time and idx and rate
		rate = rospy.Rate(100)
		start_time = rospy.Time.now().to_sec()
		roll = self.cmd_roll[0]
		pitch = self.cmd_pitch[0]
		yaw_rate = self.cmd_yaw_rate[0]
		idx = 1

		while not rospy.is_shutdown():

			current_time = rospy.Time.now().to_sec()
			dt = (current_time - start_time)/2.0 #divide by 2 since simulation is running at half speed

			if dt < stamps_secs[idx]:
				self.client.moveByRollPitchYawrateZAsync(roll, pitch, yaw_rate, -100.0, 1.0/100.0)
				rate.sleep()
				continue

			roll = self.cmd_roll[idx]
			pitch = self.cmd_pitch[idx]
			yaw_rate = self.cmd_yaw_rate[idx]

			msg = Twist()
			msg.angular.x = roll
			msg.angular.y = pitch

			self.cmd_vel_pub.publish(msg)
			self.publish_vehicle_attitude()

			self.client.moveByRollPitchYawrateZAsync(roll, pitch, 0.0, -100.0, 1.0/100.0)
			rate.sleep()

			idx +=1

			if idx == 1999:
				print('reached goal')
				self.client.hoverAsync(vehicle_name="drone")
				break




if __name__ == '__main__':
    # write code to create ROSControllerNode
    rospy.init_node('airsim_interface')
    inter = AirsimInterface()
    #inter.client.landAsync(vehicle_name='drone')
    # inter.client.takeoffAsync(vehicle_name='drone')
    inter.update_angle_gains()
    inter.update_anglerate_gains()
    inter.update_position_gains()
    inter.update_velocity_gains()
    inter.load_cmd_vel()
    #inter.move_to_origin()
    inter.move_by_cmd_vel()
    #inter.excite_system()
    rospy.spin()
