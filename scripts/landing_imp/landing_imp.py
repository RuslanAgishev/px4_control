#!/usr/bin/env python

import rospy
import tf
from tf import TransformListener
from geometry_msgs.msg import PoseStamped, TransformStamped
from math import *
import time
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
from message_filters import TimeSynchronizer, Subscriber
import numpy as np
import geometry_msgs
import tf_conversions
import crazyflie
import swarmlib
from impedance_modeles import velocity_impedance
import matplotlib.pyplot as plt

# PARAMETERs #############
toFly            = 0
put_limits       = 0
TakeoffHeight    = 1.5 # meters
land_imp         = 1
TakeoffTime      = 5     # seconds
limits           = np.array([ 1.7, 1.7, 2.5 ]) # limits desining safety flight area in the room
limits_negative  = np.array([ -1.7, -1.5, -0.1 ])

cf_name          = 'cf2'

# Variables #########################
imp_pose_prev = np.array( [0,0,0] )
imp_vel_prev = np.array( [0,0,0] )
imp_time_prev = time.time()

if __name__ == '__main__':
	rospy.init_node('impedance_landing', anonymous=True)

	if toFly:
		print "takeoff"
		cf = crazyflie.Crazyflie(cf_name, '/vicon/'+cf_name+'/'+cf_name)
		cf.setParam("commander/enHighLevel", 1)
		cf.setParam("stabilizer/estimator", 2) # Use EKF
		cf.setParam("stabilizer/controller", 2) # Use mellinger controller
		cf.takeoff(targetHeight=TakeoffHeight, duration=TakeoffTime)
		time.sleep(TakeoffTime)

	# Objects init
	drone1 = swarmlib.Drone(cf_name)

	start_landing_point = drone1.position()[:2].tolist() + [TakeoffHeight]
	drone1.sp = start_landing_point if toFly else np.array([0,0,TakeoffHeight])
	rate = rospy.Rate(60)
	print "Landing..."
	z_array = []; vz_array = []
	while not rospy.is_shutdown():
		drone1.sp[2] -= 0.01
		
		""" impedance model """
		####################################################
		drone_vel = swarmlib.velocity(drone1.sp)
		if land_imp:
			""" ipedance terms calculation """
			imp_pose, imp_vel, imp_time_prev = velocity_impedance(drone_vel, imp_pose_prev, imp_vel_prev, imp_time_prev)
			imp_pose_prev = imp_pose
			imp_vel_prev = imp_vel
			""" correct pose with impedance model """
			drone1.sp[2] += 0.04 * imp_pose[2]
			# print drone_vel[2]
		z_array.append(drone1.sp[2]); vz_array.append(drone_vel[2])
		#####################################################
		if toFly: drone1.fly()
		if drone1.sp[2]<-0.5:
			if toFly:
				time.sleep(0.5)
				for t in range(3): cf.stop()
			print 'reached the floor, shutdown'
			rospy.signal_shutdown('landed')
		rate.sleep()


		if put_limits:
			np.putmask(drone1.sp, drone1.sp >= limits, limits)
			np.putmask(drone1.sp, drone1.sp <= limits_negative, limits_negative)


		# TO FLY
		if toFly:
			drone1.fly()

		# TO VISUALIZE
		drone1.publish_sp()
		path_limit = 1000 # [frames]
		drone1.publish_path_sp(limit=path_limit)	#set -1 for unlimited path


		rate.sleep()

plt.figure()
plt.plot(z_array)
plt.title('Z')
plt.grid()
plt.xlabel('time')
plt.ylabel('z [m]')


plt.figure()
plt.plot(vz_array)
plt.title('VZ')
plt.grid()
plt.xlabel('time')
plt.ylabel('v_z [m/s]')


plt.show()