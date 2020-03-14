#!/usr/bin/env python

import numpy as np
import time
import rospy
import math

from drone import Drone
from vr_object import VRController

import crazyflie
from crazyflie_driver.msg import FullState
from crazyflie_driver.msg import Position


import os
from multiprocessing import Process

def publish_goal_pos(cf_goal_pos, cf_goal_yaw, cf_name):
    name = cf_name + "/cmd_position"
    msg = msg_def_crazyflie(cf_goal_pos, cf_goal_yaw)
    pub = rospy.Publisher(name, Position, queue_size=1)
    pub.publish(msg)

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

def hover(t=5):
    print "Hovering...\n"
    while not rospy.is_shutdown():
        for i in range(int(t*100)):
            drone.fly()
            drone.publish_sp()
            time.sleep(0.01)
        break

def landing():
    print 'Landing!!!'
    drone.sp = drone.position()
    while(1):
        drone.sp[2] = drone.sp[2]-0.05
        drone.fly()
        time.sleep(0.1)

        if drone.sp[2] < -0.5:
            print 'reached the floor'
            time.sleep(0.1)
            for _ in range(3):
                cf.stop()
            break

def takeoff(cf):
    cf.setParam("commander/enHighLevel", 1)
    cf.setParam("stabilizer/estimator",  2) # Use EKF
    cf.setParam("stabilizer/controller", 2) # Use mellinger controller
    for _ in range(3):
        print "takeoff.. ", cf.prefix
        cf.takeoff(targetHeight = TAKEOFFHEIGHT, duration = TAKEOFFTIME)
    time.sleep(TAKEOFFTIME+2.0)

rospy.init_node('CrazyflieAPI', anonymous=False)

toFly          = 1
TAKEOFFHEIGHT  = 0.8
TAKEOFFTIME    = 3.0
LANDTIME       = 2.0
CONTROLLER_HEIGHT = 0.7

initialized    = False
pos_coef       = 0.5
yaw_coef       = 3.5
put_limits     = 1
limits_up      = np.array([ 1.7, 1.7, 2.5 ]) # limits desining safety flight area in the room
limits_down    = np.array([-1.7, -1.7, -0.1 ])
rate = rospy.Rate(100)
cf_name = 'cf4'

pitch_thresh = [0.05, 0.30]
roll_thresh = [0.05, 0.30]
yaw_thresh = [0.05, 0.50]
dz_thresh = [0.04, 0.15]


if __name__ == "__main__":
    
    controller = VRController("controller_1")
    drone = Drone(cf_name)
    cf = crazyflie.Crazyflie(cf_name, '/vicon/'+cf_name+'/'+cf_name)

    # Crazyflie takeoff
    takeoff(cf)
        
    print('[INFO] Starting interaction \n')

    while not rospy.is_shutdown():
        # get resent controller's pose
        controller.position()

        # get resent drone's pose
        drone.pose = drone.position()

        if not initialized:
            print('Initial position is defined')
            controller_pose_init = controller.position()
            drone_pose_init = drone.pose
            initialized = True
        try:
            dx, dy, dz = controller.position() - controller_pose_init
        except TypeError:
            dx, dy, dz = 0, 0, 0

        # switch coordinate axes for correct mapping
        tmp_y = dy.copy()
        dy = -dx
        dx = -dz
        dz = tmp_y

        drone.sp = np.array([drone_pose_init[0] + pos_coef*dx,
                             drone_pose_init[1] + pos_coef*dy,
                             drone_pose_init[2] + pos_coef*dz])


        # define flight area to operate inside of
        if put_limits:
            np.putmask(drone.sp, drone.sp >= limits_up, limits_up)
            np.putmask(drone.sp, drone.sp <= limits_down, limits_down)

        # send pose flight commands to the Crazyflie drone
        drone.fly()  # yaw=yaw_input

        # visualization: RVIZ
        drone.publish_sp()
        drone.publish_path_sp()

        # define landing condition
        # if controller.position()[2] < 0.5:
        #     landing()
        #     break

        if dz < -0.5:
            landing()
            break

        rate.sleep()
