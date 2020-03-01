#!/usr/bin/env python

import rospy
from drone import Drone
from vr_object import VRController
import numpy as np


initialized_drone_pose = False
vel_coef = 2.0
yaw_coef = 2.5
takeoff_height = 1.0

# only movements of controller in this limits are taken into account
pitch_thresh = [0.05, 0.30]
roll_thresh = [0.05, 0.30]
yaw_thresh = [0.05, 0.50]

if __name__ == '__main__':
	rospy.init_node('drone_control', anonymous=True)
	drone = Drone()
	controller = VRController()

	# controller mean angle estimation
    print('Controller mean orientation estimation...\n') 
    time_to_eval = 0.8 # sec
    angles = controller.orientation()
    for i in range(int(time_to_eval*100)):
        angles = np.vstack([angles, controller.orientation()])
        time.sleep(0.01)
    mean_angles = np.array([np.mean(angles[:,0]), np.mean(angles[:,1]), np.mean(angles[:,2])])

    # arm the drone and takeoff
	drone.arm()
	drone.takeoff(takeoff_height)
	drone.hover(1.0)

	print('Following controller movements...')
	while not rospy.is_shutdown():
		# get resent controller pose
		controller.orientation()
		controller.position()

		if not initialized_drone_pose:
		    drone.sp = np.array( [drone.pose[0], drone.pose[1], takeoff_height] )
		    time_prev = time.time()
		    initialized_drone_pose = True

		# estimate controller movements relative to initial orientation
		roll = controller.orient[0] - mean_angles[0]
		pitch = controller.orient[1] - mean_angles[1]
		yaw = controller.orient[2] - mean_angles[2]

		# put limits on pitch movements
		if abs(pitch)<pitch_thresh[0]: x_input = 0
		elif abs(pitch)>pitch_thresh[1]: x_input = 0 #- np.sign(pitch) * pitch_thresh[1]
		else: x_input = - pitch
		# put limits on roll movements
		if abs(roll)<roll_thresh[0]: y_input = 0
		elif abs(roll)>roll_thresh[1]: y_input = 0 #np.sign(roll) * roll_thresh[1]
		else: y_input = roll
		# put limits on yaw movements
		if abs(yaw)<yaw_thresh[0]: yaw_input = 0
		elif abs(yaw)>roll_thresh[1]: yaw_input = np.sign(yaw) * yaw_thresh[1]
		else: yaw_input = yaw

		# constructing drone's commands
		cmd_vel = vel_coef * np.array([x_input, y_input, z_input])
		yaw_input = yaw_coef * yaw_input
		# print('cmd_vel', cmd_vel)
		# np.putmask(cmd_vel, abs(cmd_vel) <= (vel_koef*0.035), 0)
		# np.putmask(cmd_vel, abs(cmd_vel) <= (0.20), 0)
		time_now = time.time()
		drone.sp += cmd_vel*(time.time()-time_prev)
		time_prev = time_now

		# landing condition
		if controller.position()[2] < 0.7:
			break

		# Update timestamp and publish sp 
		drone.publish_setpoint(drone.sp)
		drone.rate.sleep()

	drone.land()