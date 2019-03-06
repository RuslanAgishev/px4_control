#!/usr/bin/env python

import rospy
import mavros
from geometry_msgs.msg import PoseStamped, TransformStamped
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import *
from math import *
import numpy as np
import argparse
import sys

current_state = State()
offb_set_mode = SetMode()
global sp
sp = PoseStamped()

# callback method for state sub
def state_cb(state):
    global current_state
    current_state = state

# callback method for local pos sub
def locpos_cb(locpos):
    global lp
    lp = locpos

# callback method for marker local position
ump = TransformStamped()
def ump_cb(data):
    global ump
    ump = data

local_pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, locpos_cb)
local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
state_sub = rospy.Subscriber('/mavros/state', State, state_cb)
arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode) 
ump_pos_sub = rospy.Subscriber('/vicon/ump/ump', TransformStamped, ump_cb)


def trajgen(mode='setpoint_relative',number_of_points=200, x_des=0,y_des=0,z_des=0):
    if mode=='line':
        # line trajectory
        x = np.hstack( [np.linspace(0, -0.5, number_of_points), np.linspace(-0.5, 0, number_of_points)] )
        y = np.zeros(2*number_of_points)
        z = np.ones(2*number_of_points)
    elif mode=='setpoint_global':
    	x0 = lp.pose.position.x
    	y0 = lp.pose.position.y
    	z0 = lp.pose.position.z
    	x = np.linspace(x0,x_des,number_of_points)
    	y = np.linspace(y0,y_des,number_of_points)
    	z = np.linspace(z0,z_des,number_of_points)
    elif mode=='setpoint_relative':
    	x0 = lp.pose.position.x
    	y0 = lp.pose.position.y
    	z0 = lp.pose.position.z
    	x = np.linspace(x0,x0+x_des,number_of_points)
    	y = np.linspace(y0,y0+y_des,number_of_points)
    	z = np.linspace(z0,z0+z_des,number_of_points)
    elif mode=='circle':
        R = 0.5
        x = R*np.cos( np.linspace(0,radians(360),number_of_points) )
        y = R*np.sin( np.linspace(0,radians(360),number_of_points) )
        z = np.ones(number_of_points)
    return x,y,z


def takeoff(height):
    prev_state = current_state
    global rate
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    # send a few setpoints before starting
    sp.pose.position.x = lp.pose.position.x
    sp.pose.position.y = lp.pose.position.y
    sp.pose.position.z = -1
    q = quaternion_from_euler(0,0,radians(90))
    sp.pose.orientation = lp.pose.orientation
    for i in range(100):
        local_pos_pub.publish(sp)
        rate.sleep()
    
    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()

    last_request = rospy.get_time()
    while not rospy.is_shutdown():
        now = rospy.get_time()
        if current_state.mode != "OFFBOARD" and (now - last_request > 2.):
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            last_request = now 
        else:
            if not current_state.armed and (now - last_request > 2.):
               arming_client(True)
               last_request = now 

        # older versions of PX4 always return success==True, so better to check Status instead
        if prev_state.armed != current_state.armed:
            rospy.loginfo("Vehicle armed: %r" % current_state.armed)

        if prev_state.mode != current_state.mode: 
            rospy.loginfo("Current mode: %s" % current_state.mode)
        prev_state = current_state

        # Update timestamp and publish sp 
        sp.header.stamp = rospy.Time.now()
        local_pos_pub.publish(sp)

        if current_state.armed:
        	break
        rate.sleep()

    rospy.loginfo('Takeoff')
    sp.pose.position.z = 0
    while sp.pose.position.z < height:
        sp.header.stamp = rospy.Time.now()
        sp.pose.position.z += 0.01
        local_pos_pub.publish(sp)
        rate.sleep()

def landing(x_land, y_land):
    while sp.pose.position.z > -0.5:
        sp.header.stamp = rospy.Time.now()
        sp.pose.position.x = x_land
        sp.pose.position.y = y_land
        sp.pose.position.z -= 0.01
        local_pos_pub.publish(sp)
        rate.sleep()

def holding(x,y,z,holdtime):
    sp.pose.position.x = x
    sp.pose.position.y = y
    sp.pose.position.z = z
    last_request = rospy.get_time()
    now = -1
    while rospy.Duration( now - last_request ) < rospy.Duration(holdtime) :
    	now = rospy.get_time()
    	local_pos_pub.publish(sp)
    	rate.sleep()


def position_control():
    x_home = lp.pose.position.x; y_home = lp.pose.position.y
    takeoff(height=1)
    # hold position before trajectory
    holding(x_home, y_home, z=1, holdtime=5.)
    
    x_ump = ump.transform.translation.x; y_ump = ump.transform.translation.y; z_ump = ump.transform.translation.z
    x,y,z = trajgen(mode='setpoint_global',x_des=x_ump,y_des=y_ump,z_des=z_ump+1)
    rospy.loginfo('Moving to the UMP position: ('+str(round(x_ump,2))+','+str(round(y_ump,2))+','+str(round(z_ump+1,2))+')')
    for i in range(len(x)):
    	sp.pose.position.x = x[i]
    	sp.pose.position.y = y[i]
    	sp.pose.position.z = z[i]
    	local_pos_pub.publish(sp)
    	rate.sleep()

    rospy.loginfo('UMP following...')
    while not rospy.is_shutdown():
        x_ump = ump.transform.translation.x; y_ump = ump.transform.translation.y; z_ump = ump.transform.translation.z
        sp.pose.position.x = x_ump
        sp.pose.position.y = y_ump
        sp.pose.position.z = z_ump+1
        sp.pose.orientation.x = ump.transform.rotation.x
        sp.pose.orientation.y = ump.transform.rotation.y
        sp.pose.orientation.z = ump.transform.rotation.z
        sp.pose.orientation.w = ump.transform.rotation.w
        local_pos_pub.publish(sp)
        rate.sleep()

    rospy.loginfo('Landing at current position...')
    landing(lp.pose.position.x, lp.pose.position.y)


if __name__ == '__main__':
    try:
        # main function
        rospy.init_node('des_location', anonymous=True)
        position_control()
    except:
        pass

        