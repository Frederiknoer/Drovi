#!/usr/bin/env python
import numpy as np
import rospy
import cv2
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu, FluidPressure
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Int8
from mav_msgs.msg import RollPitchYawrateThrust
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

def get_rpy_orientation (orientation_q):
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return [roll, pitch, yaw]

class QuadStateNode:
    def __init__(self):
        rospy.init_node('drovi_state_m')
        self.current_alt = 0.0
        self.current_yaw = 0.0

        self.heading = 99.9
        self.tracker_status = 0

        self.pose = Pose()
        self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w = 0,0,0,0
        self.pose.position.x, self.pose.position.y, self.pose.position.z = 0,0,0

        self.setpoint_pub = rospy.Publisher("/uav_setpoint", Pose, queue_size=1)

        self.odometry_pose = rospy.Subscriber("/hummingbird/odometry_sensor1/pose", Pose, self.set_odom_pose)
        self.altitude = rospy.Subscriber("/pid_controllers/altitude/state", Float64, self.set_current_alt)
        self.marker_pose = rospy.Subscriber("/nFold_markerPose", Float64, self.set_heading)
        self.tracker_status = rospy.Subscriber("/marker_status", Int8, self.set_tracker_status)

        self.state = 1
        rospy.Timer(rospy.Duration(1./100.), self.state_machine)
        rospy.spin()


    def set_current_alt(self, msg):
        self.current_alt = msg.data

    def set_odom_pose(self, msg):
        _, _, self.current_yaw = get_rpy_orientation(msg.orientation)

    def set_heading(self, msg):
        self.heading = msg.data

    def set_tracker_status(self, msg):
        self.tracker_status = msg.data

    def state_machine(self, state):
        print(self.state)
        #Search Mode
        if self.state == 1:
            self.pose.position.z = 18
            self.setpoint_pub.publish(self.pose)

            print("alt: ", self.current_alt)
            if (self.current_alt > 17.5 and self.current_alt < 18.5) or (self.tracker_status == 1):
                self.state = 2

        #Marker Tracking
        elif self.state == 2:
            self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w = quaternion_from_euler(0,0,self.heading)
            self.setpoint_pub.publish(self.pose)

            print("heading: ", self.heading)
            print("Yaw: ", self.current_yaw)
            #if self.heading < 0.1 and self.heading > -0.1:
                #self.state = 3
        #Align
        elif self.state == 3:
            if self.tracker_status != 2 and self.tracker_status != 3:
                self.pose.orientation = quaternion_from_euler(0,0.1,self.heading)
                self.setpoint_pub.publish(self.pose)
            elif self.tracker_status == 2:
                self.pose.orientation = quaternion_from_euler(0,0,self.heading)
                self.setpoint_pub.publish(self.pose)
            elif self.tracker_status == 3:
                self.pose.orientation = quaternion_from_euler(0,0,self.heading)
                self.state = 4

        elif self.state == 4:
            self.pose.position.z = 3
            self.setpoint_pub(self.pose)
            if tracker_status == 10:
                self.state = 5

        elif self.state == 5:
            pass

if __name__ == "__main__":
    node = QuadStateNode()
