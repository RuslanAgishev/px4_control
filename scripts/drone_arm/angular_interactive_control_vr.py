#!/usr/bin/env python

import rospy
from drone import Drone
from vr_object import VRController
import numpy as np
import time


initialized_drone_pose = False
vel_coef = 0.1
yaw_coef = 1
takeoff_height = 1

# only movements of controller in this limits are taken into account
pitch_limits = [6.0, 35.0] #[-0.15, 0.15]
roll_limits = [3.5, 10.0] #[-0.15, 0.15]
yaw_limits = [3.5, 10.0] #[-0.15, 0.15]



def limit_angle(angle, angle_limits):

	# put limits on pitch movements
	if abs(angle)<angle_limits[0]: vel = 0
	elif abs(angle)>angle_limits[1]: vel = 0 #- np.sign(pitch) * pitch_thresh[1]
	else: vel = angle

	return vel




# def limit_angle(angle, angle_limits):
# 	if angle < angle_limits[0]:
# 		angle = angle_limits[0]
# 	elif angle > angle_limits[1]:
# 		angle = angle_limits[1]

# 	return angle



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

	print mean_angles

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
		roll = controller.orient[2] - mean_angles[2]
		pitch = controller.orient[0] - mean_angles[0]
		yaw = controller.orient[0] - mean_angles[0]

		# print "yaw_z", yaw
		# print "pitch_x", pitch
		print "roll_y", roll





		v_x = 0#limit_angle(pitch, pitch_limits)
		v_y = limit_angle(roll, roll_limits)
		v_z = 0
		v_yaw = limit_angle(yaw, yaw_limits)

		# constructing drone's commands
		v_yaw = yaw_coef * v_yaw
		cmd_vel = vel_coef * np.array([ -v_x, v_y, v_z]) #, yaw_input])
		# print('cmd_vel', cmd_vel)

		time_now = time.time()
		drone.sp = drone.pose
		# drone.sp += cmd_vel*(time.time()-time_prev)
		time_prev = time_now

		# landing condition
		# print "yaw_input", yaw_input
		# if abs(v_yaw) > 0.7:
		# 	break

		# Update timestamp and publish sp 
		drone.publish_setpoint(drone.sp)
		drone.rate.sleep()

	drone.land()