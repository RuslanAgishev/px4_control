#!/usr/bin/env python

import numpy as np
import time
from drone import Drone
import rospy
import math

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

def takeoff(cf_name):
    cf = crazyflie.Crazyflie(cf_name, '/vicon/'+cf_name+'/'+cf_name)
    cf.setParam("commander/enHighLevel", 1)
    cf.setParam("stabilizer/estimator",  2) # Use EKF
    cf.setParam("stabilizer/controller", 2) # Use mellinger controller
    for _ in range(3):
        print "takeoff.. ", cf.prefix
        cf.takeoff(targetHeight = TAKEOFFHEIGHT, duration = TAKEOFFTIME)
    time.sleep(TAKEOFFTIME+2.0)


toFly          = 1
TAKEOFFHEIGHT  = 0.8
TAKEOFFTIME    = 3.0
LANDTIME       = 2.0
CONTROLLER_HEIGHT = 0.7

initialized    = False
vel_koef       = 3.0
yaw_koef       = 3.5
put_limits     = 1
limits_up      = np.array([ 1.7, 1.7, 2.5 ]) # limits desining safety flight area in the room
limits_down    = np.array([-1.7, -1.7, -0.1 ])
rate = rospy.Rate(100)
cf_name = 'cf1'

pitch_thresh = [0.05, 0.30]
roll_thresh = [0.05, 0.30]
yaw_thresh = [0.05, 0.50]
dz_thresh = [0.04, 0.15]


if __name__ == "__main__":
    rospy.init_node('CrazyflieAPI', anonymous=False)

    controller = VRController("controller_1")
    drone = Drone(cf_name)

    # Crazyflie takeoff
    takeoff(cf_name)
        
    # controller mean angle estimation
    print('[INFO] Controller mean orientation estimation...\n') 
    time_to_eval = 0.8 # sec
    angles = controller.orientation()
    for i in range(int(time_to_eval*100)):
        angles = np.vstack([angles, controller.orientation()])
        time.sleep(0.01)
    mean_angles = np.array([np.mean(angles[:,0]), np.mean(angles[:,1]), np.mean(angles[:,2])])

    print('[INFO] Starting interaction \n')

    while not rospy.is_shutdown():
        # get resent controller's pose
        controller.orientation()
        controller.position()

        # get resent drone's pose
        drone.pose = drone.position()

        # estimate controller movements relative to initial orientation
        roll = controller.orient[0] - mean_angles[0]
        pitch = controller.orient[1] - mean_angles[1]
        yaw = controller.orient[2] - mean_angles[2]
        dz = controller.pose[2] - CONTROLLER_HEIGHT


        if not initialized:
            drone.sp = np.array( [drone.pose[0], drone.pose[1], TAKEOFFHEIGHT] )
            time_prev = time.time()
            initialized = True

        if abs(pitch)<pitch_thresh[0]: x_input = 0
        elif abs(pitch)>pitch_thresh[1]: x_input = 0#- np.sign(pitch) * pitch_thresh[1]
        else: x_input = - pitch

        if abs(roll)<roll_thresh[0]: y_input = 0
        elif abs(roll)>roll_thresh[1]: y_input = 0#np.sign(roll) * roll_thresh[1]
        else: y_input = roll

        if abs(yaw)<yaw_thresh[0]: yaw_input = 0
        elif abs(yaw)>roll_thresh[1]: yaw_input = np.sign(yaw) * yaw_thresh[1]
        else: yaw_input = yaw

        if abs(dz)<dz_thresh[0]: z_input = 0
        elif abs(dz)>dz_thresh[1]: z_input = np.sign(dz) * dz_thresh[1]
        else: z_input = dz

        cmd_vel = vel_koef*(np.array([x_input, y_input, z_input]))
        yaw_input = yaw_koef * yaw_input
        # print 'cmd_vel', cmd_vel
        # np.putmask(cmd_vel, abs(cmd_vel) <= (vel_koef*0.035), 0)
        # np.putmask(cmd_vel, abs(cmd_vel) <= (0.20), 0)
        time_now = time.time()
        drone.sp += cmd_vel*(time.time()-time_prev)
        time_prev = time_now

        # define flight area to operate inside of
        if put_limits:
            np.putmask(drone.sp, drone.sp >= limits_up, limits_up)
            np.putmask(drone.sp, drone.sp <= limits_down, limits_down)

        # send pose flight commands to the Crazyflie drone
        drone.fly(yaw=yaw_input)

        # visualization: RVIZ
        drone.publish_sp(orient=np.array([0,0,yaw_input]))
        drone.publish_path_sp()

        # define landing condition
        if controller.position()[2] < 0.5:
            landing()
            break

        rate.sleep()
