#!/usr/bin/env python

import triad_openvr
import numpy as np

class VRController:
	def __init__(self):
		self.pose = None # [x,y,z](m)
		self.orient = None # Euler angles
		self.vr = triad_openvr.triad_openvr()


	def position(self):
		pose = self.vr.devices["controller_1"].get_pose_euler()
		return np.array(pose[:3])
	def orientation(self):
		pose = self.vr.devices["controller_1"].get_pose_euler()
		return np.array(pose[3:])
