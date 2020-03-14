#!/usr/bin/env python

import triad_openvr
import numpy as np

class VRController:
	def __init__(self, name="controller_1"):
		self.name = name
		self.pose = None # [x,y,z](m)
		self.orient = None # Euler angles
		self.vr = triad_openvr.triad_openvr()

	def position(self):
		"""
		return: [x,y,z]
		"""
		pose = self.vr.devices["controller_1"].get_pose_euler()
		try:
			self.pose = np.array(pose[:3])
		except TypeError:
			pass

		return self.pose


	def orientation(self):
		pose = self.vr.devices[self.name].get_pose_euler()
		self.orient = np.array(pose[3:])
		return self.orient
