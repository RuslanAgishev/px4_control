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

# callback method for state sub
current_state = State() 
offb_set_mode = SetMode
global sp

sp = PoseStamped()

def state_cb(state):
    global current_state
    current_state = state

# callback method for local pos sub
def locpos_cb(locpos):
    global lp
    lp = locpos

def marker_cb(data):
    global ardata
    ardata = data

local_pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, locpos_cb)
local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
marker_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, marker_cb)
state_sub = rospy.Subscriber('/mavros/state', State, state_cb)
arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

def takeoff(height):
    rospy.loginfo("Takeoff")
    sp.pose.position.z = 0
    while lp.pose.position.z < height:
        sp.header.stamp = rospy.Time.now()
        sp.pose.position.z += 0.01
        local_pos_pub.publish(sp)
        rate.sleep()

def landing():
    rospy.loginfo("Landing...")
    sp.pose.position.z = lp.pose.position.z
    while sp.pose.position.z > -0.5:
        sp.header.stamp = rospy.Time.now()
        sp.pose.position.z -= 0.01
        local_pos_pub.publish(sp)
        rate.sleep()


def position_control(h_des):
    rospy.init_node('des_location', anonymous=True)
    prev_state = current_state
    global rate
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    # send a few setpoints before starting
    sp.pose.position.x = lp.pose.position.x
    sp.pose.position.y = lp.pose.position.y
    sp.pose.position.z = -1

    q = quaternion_from_euler(0,0,radians(90))
    sp.pose.orientation.x = q[0]
    sp.pose.orientation.y = q[1]
    sp.pose.orientation.z = q[2]
    sp.pose.orientation.w = q[3]
    
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

        if current_state.armed:
            break
        # Update timestamp and publish sp 
        sp.header.stamp = rospy.Time.now()
        local_pos_pub.publish(sp)
        rate.sleep()

    takeoff(height=h_des)
    
    # mission
    rospy.loginfo('Searching the marker...')
    sp.pose.position = lp.pose.position
    while not ardata.markers:
        # Update timestamp and publish sp 
        sp.header.stamp = rospy.Time.now()
        local_pos_pub.publish(sp)
        rate.sleep()
    
    # quadrotor aligning with the marker (x,y,yaw)
    rospy.loginfo('Orienting quadrotor before landing')
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
    hover_end = 0
    hover_start = rospy.get_time()
    while hover_end - hover_start < 5.:
        local_pos_pub.publish(sp)
        hover_end = rospy.get_time()

    landing()

    # disarming
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
        parser = argparse.ArgumentParser(description="Command line tool for setting desired altitude and time of holding it.")
        parser.add_argument('-alt', '--altitude', type=float, nargs=1, help="Desired takeoff altitude")
        args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
        h_des = args.altitude[0]
        position_control(h_des)
    except rospy.ROSInterruptException:
        pass