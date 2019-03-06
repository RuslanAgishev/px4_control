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
import argparse
import sys
#import RPi.GPIO as GPIO


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
ardata = AlvarMarkers()
def marker_cb(data):
    global ardata
    ardata = data

local_pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, locpos_cb)
local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
state_sub = rospy.Subscriber('/mavros/state', State, state_cb)
arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode) 

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

def marker_search():
	# searching landing place location
    '''
    rospy.loginfo('Searching for the marker...')
    sp.pose.position = lp.pose.position
    while not ardata.markers:
        # Update timestamp and publish sp 
        sp.header.stamp = rospy.Time.now()
        local_pos_pub.publish(sp)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, marker_cb)
        rate.sleep()
    rospy.loginfo('Marker is found')

    # quadrotor aligning with the marker (yaw)
    rospy.loginfo('Orienting quadrotor before landing...')
    qm = np.zeros(4)
    qm[0] = ardata.markers[0].pose.pose.orientation.x
    qm[1] = ardata.markers[0].pose.pose.orientation.y
    qm[2] = ardata.markers[0].pose.pose.orientation.z
    qm[3] = ardata.markers[0].pose.pose.orientation.w
    _,_,yaw_marker =  euler_from_quaternion(qm)
    q_des = quaternion_from_euler(0,0,yaw_marker)
    sp.header = ardata.markers[0].header
    sp.pose.orientation.x = q_des[0]
    sp.pose.orientation.y = q_des[1]
    sp.pose.orientation.z = q_des[2]
    sp.pose.orientation.w = q_des[3]
    '''

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


def landing(x_land,y_land):
    while sp.pose.position.z > -0.5:
        sp.header.stamp = rospy.Time.now()
        sp.pose.position.x = x_land
        sp.pose.position.y = y_land
        sp.pose.position.z -= 0.01
        local_pos_pub.publish(sp)
        rate.sleep()

def holding(x,y,z,holdtime,switch=''):
    '''
    # switching IR-marker
    PIN=17
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN, GPIO.OUT)
    sp.pose.position.x = x
    sp.pose.position.y = y
    sp.pose.position.z = z
    if switch=='small':
        GPIO.output(PIN, 0)
    elif switch=='big':
        GPIO.output(PIN, 1)
    else:
        pass
    # end switching
    '''
    last_request = rospy.get_time()
    now = -1

    while rospy.Duration( now - last_request ) < rospy.Duration(holdtime) :
    	now = rospy.get_time()
    	local_pos_pub.publish(sp)
    	rate.sleep()


def position_control(setpoints):
    x_home = lp.pose.position.x; y_home = lp.pose.position.y
    takeoff(height=1)
    # hold position before trajectory
    holding(x_home, y_home, z=1, holdtime=5.)
    
    # trajectory generation
    number_of_setpoints = setpoints.shape[0]
    for i in range(number_of_setpoints):
	    x_des = setpoints[i][0]; y_des = setpoints[i][1]; z_des = setpoints[i][2]
	    x,y,z = trajgen(mode='setpoint_relative',x_des=x_des,y_des=y_des,z_des=z_des)
	    rospy.loginfo('Moving to the setpoint: ('+str(x_des)+','+str(y_des)+','+str(z_des)+')')
	    for i in range(len(x)):
	    	sp.pose.position.x = x[i]
	    	sp.pose.position.y = y[i]
	    	sp.pose.position.z = z[i]
	    	local_pos_pub.publish(sp)
	    	rate.sleep()
	    holding(x[-1],y[-1],z[-1],holdtime=5.)

    rospy.loginfo('Landing...')
    landing(lp.pose.position.x, lp.pose.position.y)

    # disarming
    last_request = rospy.get_time()
    while current_state.armed or current_state.mode == "OFFBOARD":
        now = rospy.get_time()
        if current_state.armed and (now - last_request > 2.):
            arming_client(False)
        if current_state.mode == "OFFBOARD" and (now - last_request > 2.):
            set_mode_client(base_mode=0, custom_mode="MANUAL")
            last_request = now
        rate.sleep()


if __name__ == '__main__':
    try:
    	# command-line arguments for desired location
    	parser = argparse.ArgumentParser(description="Command line tool for setting desired altitude and time of holding it.")
        parser.add_argument('-x', type=float, nargs=1, help="Setpoint x-position")
        parser.add_argument('-y', type=float, nargs=1, help="Setpoint y-position")
        parser.add_argument('-z', type=float, nargs=1, help="Setpoint z-position")
        args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
        # main function
        rospy.init_node('des_location', anonymous=True)
        if args.x and args.y and args.z is not None:
        	position_control( np.array([ [args.x[0], args.y[0], args.z[0]] ]) )
        else:
        	#position_control( np.array([ [1,0,0], [0,1,0], [1,1,0] ]) )
        	position_control( np.array([ [0,0,0.5], [0,1,0], [0,0.7,-1.5] ]) )
    except rospy.ROSInterruptException:
        pass

        