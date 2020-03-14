#!/usr/bin/env python

import rospy
from drone import Drone
from vr_object import VRController
import numpy as np
import time


initialized = False
pos_coef = 2.0

if __name__ == '__main__':
	rospy.init_node('drone_control', anonymous=True)
	drone = Drone()
	human = VRController()

	drone.arm()
	drone.takeoff(1.0)
	drone.hover(1.0)

	print('Following human movements...')
	while not rospy.is_shutdown():
		if not initialized:
			print('Initial position is defined')
			human_pose_init = human.position()
			drone_pose_init = drone.pose
			initialized = True
		dx, dy, dz = human.position() - human_pose_init

		# tmp = dx; dx = -dy; dy = tmp # switch coordinate axes for correct mapping
		
		# switch coordinate axes for correct mapping
		tmp_y = dy.copy()
		dy = -dz
		dz = tmp_y 

		drone.sp = np.array([drone_pose_init[0] + pos_coef*dx,
							 drone_pose_init[1] + pos_coef*dy,
							 drone_pose_init[2] + pos_coef*dz])

		# print("dx", dx)
		# print("dy", dy)
		# print("dz", dz)

		if dz < -0.5:
			break

		# Update timestamp and publish sp 
		drone.publish_setpoint(drone.sp)
		drone.rate.sleep()

	drone.land()