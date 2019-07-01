#!/usr/bin/env python

import rospy
import tf
from tf import TransformListener
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
import message_filters


class MocapObject:
	def __init__(self, name):
		self.name = name
		self.tf = '/vicon/'+name+'/'+name
		self.tl = TransformListener()
		self.pose = np.array([0.,0.,0.])
		self.orient = np.array([0,0,0]) # Euler angles
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
