#!/usr/bin/env python

import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import *
from math import *
import numpy as np


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

def disarming(state):
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode) 
    last_request = rospy.get_time()
    while state.armed or state.mode == "OFFBOARD":
        now = rospy.get_time()
        if current_state.armed and (now - last_request > 2.):
            arming_client(False)
        if current_state.mode == "OFFBOARD" and (now - last_request > 2.):
            set_mode_client(base_mode=0, custom_mode="MANUAL")
            last_request = now
        rate.sleep()


        