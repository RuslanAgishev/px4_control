#!/usr/bin/env python

from __future__ import division
import rospy
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

from math import *
import math
import time
from time import sleep
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
import message_filters
import sys
import numpy as np
from tf import TransformListener
from crazyflie_driver.msg import FullState
from crazyflie_driver.msg import Position
import os
np.set_printoptions(formatter={'float': '{: 0.2f}'.format})



class Mocap_object: # superclass
	def __init__(self, name):
		self.name = name
		self.tf = '/vicon/'+name+'/'+name
		self.tl = TransformListener()
		self.pose = np.array([0.,0.,0.])
		self.orient = np.array([0,0,0]) # Euler angles
		self.path = Path()
		# for velocity:
		sub = message_filters.Subscriber(self.tf, TransformStamped)
		self.cache = message_filters.Cache(sub, 100)
		self.vel = np.array([0,0,0])
	def position(self):
		self.tl.waitForTransform("/world", self.tf, rospy.Time(0), rospy.Duration(1))
		position, quaternion = self.tl.lookupTransform("/world", self.tf, rospy.Time(0))
		self.pose = np.array(position)
		return np.array(position)
	def orientation(self):
		self.tl.waitForTransform("/world", self.tf, rospy.Time(0), rospy.Duration(1))
		position, quaternion = self.tl.lookupTransform("/world", self.tf, rospy.Time(0))
		self.orient = get_angles(np.array(quaternion))
		return get_angles(np.array(quaternion))
	def publish_position(self):
		publish_pose(self.pose, self.orient, self.name+"_pose")
	def publish_path(self, limit=1000):
		publish_path(self.path, self.pose, self.orient, self.name+"_path", limit)
	def velocity(self):
		aver_interval = 0.1 # sec
		msg_past = self.cache.getElemAfterTime(self.cache.getLatestTime() - rospy.rostime.Duration(aver_interval))
		msg_now = self.cache.getElemAfterTime(self.cache.getLatestTime())
		if (msg_past is not None) and  (msg_now is not None) and (msg_now.header.stamp != msg_past.header.stamp):
			vel = vel_estimation_TransformStamped(msg_past, msg_now)
			self.vel = vel


class Drone(Mocap_object): # TODO: use superclass mocap_object
	def __init__(self, name, leader = False):
		Mocap_object.__init__(self, name)
		self.sp = np.array([0.,0.,0.])
		sub_sp = message_filters.Subscriber(self.name+"_sp", PoseStamped)
		self.cache_sp = message_filters.Cache(sub_sp, 100)
		self.vel_sp = np.array([0,0,0])

	def publish_sp(self):
		publish_pose(self.sp, np.array([0,0,0]), self.name+"_sp")
	def publish_path_sp(self, limit=1000):
		publish_path(self.path, self.sp, self.orient, self.name+"_path_sp", limit)
	def fly(self, yaw=0):
		publish_goal_pos(self.sp, yaw, "/"+self.name)
	def apply_limits(self, uper_limits, lower_limits):
		np.putmask(self.sp, self.sp >= uper_limits, uper_limits)
		np.putmask(self.sp, self.sp <= lower_limits, lower_limits)
	def update_radius_imp(self, delta):
		if self.rad_imp.inside:
			radius_obstacle_impedance(self)
			self.sp += self.rad_imp.pose
	def velocity_sp(self):
		aver_interval = 0.2 # sec
		if self.cache_sp.getLatestTime() is not None:
			msg_past = self.cache_sp.getElemAfterTime(self.cache_sp.getLatestTime() - rospy.rostime.Duration(aver_interval))
			msg_now = self.cache_sp.getElemAfterTime(self.cache_sp.getLatestTime())
			if (msg_past is not None) and (msg_now is not None) and (msg_now.header.stamp != msg_past.header.stamp):
				vel_sp = vel_estimation_PoseStamped(msg_past, msg_now)
				self.vel_sp = vel_sp



# Service functions ###############################################################
def publish_goal_pos(cf_goal_pos, cf_goal_yaw, cf_name):
	name = cf_name + "/cmd_position"
	msg = msg_def_crazyflie(cf_goal_pos, cf_goal_yaw)
	pub = rospy.Publisher(name, Position, queue_size=1)
	pub.publish(msg)
def publish_pose(pose, orient, topic_name):
	msg = msg_def_PoseStamped(pose, orient)
	pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
	pub.publish(msg)
def publish_cylinder(pose, orient, R, topic_name):
	shape = Marker.CYLINDER
	msg = msg_def_Cylinder(pose, orient, shape, R=R)
	pub = rospy.Publisher(topic_name, Marker, queue_size=1)
	pub.publish(msg)
def publish_path(path, pose, orient, topic_name, limit=1000):
	msg = msg_def_PoseStamped(pose, orient)
	path.header = msg.header
	path.poses.append(msg)
	if limit>0:
		path.poses = path.poses[-limit:]
	pub = rospy.Publisher(topic_name, Path, queue_size=1)
	pub.publish(path)
def publish_vel(vel, topic_name):
	msg = Twist()
	msg.linear.x = vel[0]
	msg.linear.y = vel[1]
	msg.linear.z = vel[2]
	pub = rospy.Publisher(topic_name, Twist, queue_size=1)
	pub.publish(msg)
def get_angles(message):
	quat = ( message[0], message[1], message[2], message[3] )
	euler = tf.transformations.euler_from_quaternion(quat)
	return euler
def msg_def_crazyflie(pose, yaw):
	worldFrame = rospy.get_param("~worldFrame", "/world")
	msg = Position()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = worldFrame
	msg.x = pose[0]
	msg.y = pose[1]
	msg.z = pose[2]
	msg.yaw = yaw
	now = rospy.get_time()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	return msg
def msg_def_PoseStamped(pose, orient):
	worldFrame = "world"
	msg = PoseStamped()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = worldFrame
	msg.pose.position.x = pose[0]
	msg.pose.position.y = pose[1]
	msg.pose.position.z = pose[2]
	quaternion = tf.transformations.quaternion_from_euler(orient[0], orient[1], orient[2]) #1.57
	msg.pose.orientation.x = quaternion[0]
	msg.pose.orientation.y = quaternion[1]
	msg.pose.orientation.z = quaternion[2]
	msg.pose.orientation.w = quaternion[3]
	msg.header.seq += 1
	return msg
def msg_def_Cylinder(pose, orient, shape, R):
	worldFrame = "world"
	msg = Marker()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = worldFrame
	msg.type = shape
	msg.pose.position.x = pose[0]
	msg.pose.position.y = pose[1]
	msg.pose.position.z = pose[2] * 0.5
	# quaternion = tf.transformations.quaternion_from_euler(orient[0], orient[1], orient[2])
	quaternion = tf.transformations.quaternion_from_euler(0,0,0)
	msg.pose.orientation.x = quaternion[0]
	msg.pose.orientation.y = quaternion[1]
	msg.pose.orientation.z = quaternion[2]
	msg.pose.orientation.w = quaternion[3]
	msg.scale.x = R
	msg.scale.y = R
	msg.scale.z = 2.0
	msg.color.r = 0.0
	msg.color.g = 1.0
	msg.color.b = 0.0
	msg.color.a = 1.0
	msg.header.seq += 1
	msg.header.stamp = rospy.Time.now()
	return msg
def vel_estimation_TransformStamped(msg_past, msg_now): # from two TransformStamped messages
	x_now = msg_now.transform.translation.x
	x_past = msg_past.transform.translation.x
	y_now = msg_now.transform.translation.y
	y_past = msg_past.transform.translation.y
	z_now = msg_now.transform.translation.z
	z_past = msg_past.transform.translation.z
	time_now = msg_now.header.stamp.to_sec()
	time_past = msg_past.header.stamp.to_sec()
	vel_x = (x_now-x_past)/(time_now-time_past)
	vel_y = (y_now-y_past)/(time_now-time_past)
	vel_z = (z_now-z_past)/(time_now-time_past)
	vel = np.array([vel_x, vel_y, vel_z])
	return vel
def vel_estimation_PoseStamped(msg_past, msg_now): # from two TransformStamped messages
	x_now = msg_now.pose.position.x
	x_past = msg_past.pose.position.x
	y_now = msg_now.pose.position.y
	y_past = msg_past.pose.position.y
	z_now = msg_now.pose.position.z
	z_past = msg_past.pose.position.z
	time_now = msg_now.header.stamp.to_sec()
	time_past = msg_past.header.stamp.to_sec()
	vel_x = (x_now-x_past)/(time_now-time_past)
	vel_y = (y_now-y_past)/(time_now-time_past)
	vel_z = (z_now-z_past)/(time_now-time_past)
	vel = np.array([vel_x, vel_y, vel_z])
	return vel

