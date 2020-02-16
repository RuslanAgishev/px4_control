#!/usr/bin/env python

import rospy
from drone import Drone
from mocap_object import MocapObject
import numpy as np


initialized = False
pos_coef = 1

if __name__ == '__main__':
	rospy.init_node('drone_control', anonymous=True)
	drone = Drone()
	palm = MocapObject('karton_big')

	drone.arm()
	drone.takeoff(1.0)
	drone.hover(1.0)

	print('Following human movements...')
	while not rospy.is_shutdown():
		if not initialized:
			print('Initial position is defined')
			palm_pose_init = palm.position()
			drone_pose_init = drone.pose
			initialized = True
		dx, dy, dz = palm.position() - palm_pose_init
		tmp = dx; dx = -dy; dy = tmp
		drone.sp = np.array([drone_pose_init[0] + pos_coef*dx,
							   	 drone_pose_init[1] + pos_coef*dy,
							   	 drone_pose_init[2] + pos_coef*dz])

		if palm.position()[2] < 0.5:
			break

		# Update timestamp and publish sp 
		drone.publish_setpoint(drone.sp)
		drone.rate.sleep()

	drone.land()