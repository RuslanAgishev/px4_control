#!/usr/bin/env python

import rospy
import mavros
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import *
from math import *
import numpy as np

# callback method for state sub
current_state = State() 
offb_set_mode = SetMode
global sp, marker
marker = None

sp = PoseStamped()

def state_cb(state):
    global current_state
    current_state = state

# callback method for local pos sub
def locpos_cb(locpos):
    global lp
    lp = locpos

def ar_cb(data):
    if not data.markers:
        #rospy.loginfo("No markers")
        marker = None
    else:
        marker = data.markers[0].pose.pose
        rospy.loginfo("Marker detected")

#ar_marker_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, ar_cb)
local_pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, locpos_cb)
local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
state_sub = rospy.Subscriber('mavros/state', State, state_cb)
arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)


def trajectory(number_of_frames=500):
    # line trajectory
    x = np.linspace(0, number_of_frames/100, number_of_frames)
    y = np.linspace(0, number_of_frames/100, number_of_frames)
    z = 3 * np.ones(number_of_frames)
    # send trajectory setpoints
    for i in range(len(x)):
        sp.pose.position.x = x[i]
        sp.pose.position.y = y[i]
        sp.pose.position.z = z[i]
        local_pos_pub.publish(sp)
        rate.sleep()

def takeoff(height):
    sp.pose.position.z = 0.
    while lp.pose.position.z <  height:
        sp.header.stamp = rospy.Time.now()
        sp.pose.position.z += 0.01
        local_pos_pub.publish(sp)
        rate.sleep()

def landing():
    while not rospy.is_shutdown():
        sp.header.stamp = rospy.Time.now()
        sp.pose.position.z -= 0.01
        local_pos_pub.publish(sp)
        rate.sleep()

def marker_following():
    rospy.loginfo("Marker detected")
    sp.pose.position.x += (marker.position.x - sp.pose.position.x) / 2.0
    sp.pose.position.y += (marker.position.y - sp.pose.position.y) / 2.0
    sp.pose.position.z += (marker.position.z - sp.pose.position.z) / 2.0
    local_pos_pub.publish(sp)
    rate.sleep()

def position_control():
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

    last_request = rospy.get_rostime()
    landflag = 0
    rospy.loginfo('Mission start')
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(2.)):
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            last_request = now 
        else:
            if not current_state.armed and (now - last_request > rospy.Duration(2.)):
               arming_client(True)
               last_request = now 

        # older versions of PX4 always return success==True, so better to check Status instead
        if prev_state.armed != current_state.armed:
            rospy.loginfo("Vehicle armed: %r" % current_state.armed)

        if prev_state.mode != current_state.mode: 
            rospy.loginfo("Current mode: %s" % current_state.mode)
        prev_state = current_state

        # TAKEOFF
        if current_state.armed:
            takeoff(height=1)

        # Update timestamp and publish sp 
        sp.header.stamp = rospy.Time.now()
        local_pos_pub.publish(sp)
        rate.sleep()

        # FLIGHT
        now = rospy.get_rostime()
        if not landflag and marker is not None and (now - last_request > rospy.Duration(5.)):
            last_request = now
            marker_following()
            #trajectory()
            landflag = 1

        if landflag:
            break    
        

    # descending
    rospy.loginfo("Landing...")
    landing()

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass