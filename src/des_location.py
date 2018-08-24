#!/usr/bin/python

import rospy
import mavros
#from mavros_msgs.msg import State
from mavros import setpoint as SP
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped 
from mavros_msgs.srv import SetMode
import time
from tf.transformations import *
import numpy as np
from math import *


#Publisher to  mavros/setpoint_position/local
def setpoint():
    pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rospy.init_node('des_location', anonymous=True)
    
    msg = SP.PoseStamped(
        header=SP.Header(
            frame_id="base_footprint",  # no matter, plugin don't use TF
            stamp=rospy.Time.now()),    # stamp should update
    )

    while not rospy.is_shutdown():
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.3

        quaternion = quaternion_from_euler(0, 0, 0)
        msg.pose.orientation = SP.Quaternion(*quaternion)

        pub.publish(msg)
        print msg

if __name__ == '__main__':
    try:
        setpoint()
    except rospy.ROSInterruptException:
        pass


