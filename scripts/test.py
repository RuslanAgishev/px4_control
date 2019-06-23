import rospy
from drone import Drone


rospy.init_node('test', anonymous=True)
drone = Drone()

drone.arm()
drone.takeoff(1.0)
drone.hover(1.0)
drone.goTo([1,0,0], 'relative')
drone.hover(1.0)
drone.land()