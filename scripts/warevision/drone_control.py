#! /usr/bin/env python
import rospy
import numpy as np
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import tf.transformations


class DroneControlNode(object):
    def __init__(self):
        self.current_drone_state = None
        self.drone_pose = None
        self.prev_request = None
        self.prev_state = None
        self.state = None
        self.cmd_id = None

        self.z = None
        self.target_x = None
        self.target_y = None
        self.target_z = None
        self.target_yaw = None

        self.cmd_type = None
        self.cmd_id = None

        self.timer = None
        self.rate = 10

        self.setpoint_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.drone_pose_callback)
        rospy.Subscriber("cmd_drone", String, self.cmd_drone_callback)

    def state_callback(self, state):
        self.current_drone_state = state

    def drone_pose_callback(self, pose):
        self.drone_pose = pose

    def cmd_drone_callback(self, data):
        # self.mutex.acquire()
        rospy.loginfo("==========================================")

        rospy.loginfo("NEW CMD:\t" + str(data.data))
        data_splitted = data.data.split()
        cmd_id = data_splitted[0]
        cmd_type = data_splitted[1]
        args = data_splitted[2:]

        if cmd_type == "takeoff":
            self.target_z = float(args[0])
        elif cmd_type == "holding":
            self.target_x = float(args[0])
            self.target_y = float(args[1])
            self.target_z = float(args[2])
            self.target_yaw = float(args[3])
        elif cmd_type == "holding_z":
            self.target_x = 0
            self.target_y = 0
            self.target_z = float(args[0])
            self.target_yaw = np.pi / 2
        self.start_cmd(cmd_id, cmd_type)

    def start_cmd(self, cmd_id, cmd_type):
        if self.state != "finish":
            self.terminate_cmd("terminated")
        self.cmd_id = cmd_id
        self.cmd_type = cmd_type
        self.state = "start"
        self.timer = rospy.Timer(rospy.Duration(1. / self.rate), self.timer_callback)

    def terminate_cmd(self, status):
        rospy.loginfo("Cmd finished with status" + status)
        self.state = "finish"

    def timer_callback(self, event):
        rospy.loginfo("State: " + self.state)
        if self.cmd_type == "takeoff":
            if self.state == "start":
                self.state = "wait"
                self.wait()
            elif self.state == "wait":
                self.wait()
            elif self.state == "ready":
                self.state = "takeoff"
                self.z = 0
                self.takeoff()
            elif self.state == "takeoff":
                self.takeoff()
            elif self.state == "finish":
                self.terminate_cmd("finish")
        elif self.cmd_type == "holding":
            if self.state == "start":
                self.state = "holding"
                self.holding()
            elif self.state == "holding":
                self.holding()
            elif self.state == "finish":
                self.terminate_cmd("finish")
        elif self.cmd_type == "landing":
            if self.state == "start":
                self.state = "landing"
                self.z = self.drone_pose.position.z
                self.landing()
            elif self.state == "landing":
                self.landing()
            elif self.state == "finish":
                self.terminate_cmd("finish")

    def holding(self):
        self.publish_setpoint(self.get_setpoint(self.target_x, self.target_y, self.target_z, self.target_yaw))
        self.state = "finish"

    def takeoff(self):
        self.z += 0.1
        self.publish_setpoint(self.get_setpoint(0, 0, self.z, np.pi / 2))
        if self.z > self.target_z:
            self.state = "finish"

    def landing(self):
        self.z -= 0.1
        if self.z < -0.5:
            self.state = "finish"

    def wait(self):
        if self.current_drone_state is None or self.drone_pose is None:
            return
        now = rospy.get_time()
        if self.current_drone_state.mode != "OFFBOARD" and (now - self.prev_request > 2.):
            self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            self.prev_request = now
        elif not self.current_drone_state.armed and (now - self.prev_request > 2.):
            self.arming_client(True)
            self.prev_request = now

        # older versions of PX4 always return success==True, so better to check Status instead
        if self.prev_state.armed != self.current_drone_state.armed:
            rospy.loginfo("Vehicle armed: %r" % self.current_drone_state.armed)

        if self.prev_state.mode != self.current_drone_state.mode:
            rospy.loginfo("Current mode: %s" % self.current_drone_state.mode)

        self.prev_state = self.current_drone_state
        # Update timestamp and publish sp
        self.publish_setpoint(self.get_setpoint(0, 0, -1, np.pi / 2))

        if self.current_drone_state.armed:
            self.state = "ready"

    @staticmethod
    def get_setpoint(x, y, z, angle):
        set_pose = PoseStamped()
        set_pose.pose.position.x = x
        set_pose.pose.position.y = y
        set_pose.pose.position.z = z
        q = tf.transformations.quaternion_from_euler(0, 0, angle)
        set_pose.pose.orientation.x = q[0]
        set_pose.pose.orientation.y = q[1]
        set_pose.pose.orientation.z = q[2]
        set_pose.pose.orientation.w = q[3]
        return set_pose

    def publish_setpoint(self, setpoint):
        setpoint.header.stamp = rospy.Time.now()
        self.setpoint_publisher.publish(setpoint)


if __name__ == "__main__":
    node = DroneControlNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
