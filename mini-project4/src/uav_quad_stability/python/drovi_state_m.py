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
from uav_quad_stability.msg import markerpose
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
        self.current_x = 0.0
        self.current_y = 0.0
        self.marker_x = 0.0
        self.marker_y = 0.0
        self.current_dist = 0

        self.aruco_x = 0.0
        self.aruco_y = 0.0

        self.heading_adjust = 0.0

        self.heading = 99.9
        self.tracker_status = 0

        self.pose = Pose()
        self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w = 0,0,0,0
        self.pose.position.x, self.pose.position.y, self.pose.position.z = 0,0,0

        self.setpoint_pub = rospy.Publisher("/uav_setpoint", Pose, queue_size=1)

        self.odometry_pose = rospy.Subscriber("/hummingbird/odometry_sensor1/pose", Pose, self.set_odom_pose)
        self.altitude = rospy.Subscriber("/pid_controllers/altitude/state", Float64, self.set_current_alt)

        self.marker_pose = rospy.Subscriber("/nFold_markerPose", markerpose, self.set_heading)
        self.aruco_pose = rospy.Subscriber("/aruco_markerPose", markerpose, self.set_aruco)

        self.tracker_status = rospy.Subscriber("/marker_status", Int8, self.set_tracker_status)

        self.heading_adjust_sub = rospy.Subscriber("/heading_adjust", Float64, self.set_heading_adjust)

        self.state = 1
        rospy.Timer(rospy.Duration(1./100.), self.state_machine)
        rospy.spin()

    def set_heading_adjust(self, msg):
        self.heading_adjust = msg.data

    def set_current_alt(self, msg):
        self.current_alt = msg.data

    def set_odom_pose(self, msg):
        _, _, self.current_yaw = get_rpy_orientation(msg.orientation)
        self.current_x, self.current_y = msg.position.x, msg.position.y

    def set_heading(self, msg):
        self.marker_x = msg.x * (self.current_alt -1)
        self.marker_y = msg.y * (self.current_alt -1)

    def set_aruco(self, msg):
        self.aruco_x = msg.x * (self.current_alt -1)
        self.aruco_y = msg.y * (self.current_alt -1)

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
            self.heading = (math.atan2((self.marker_y+self.current_y),(self.marker_x+self.current_x)))
            #self.heading = (math.atan2((self.current_y - self.marker_y),(self.current_x - self.current_x)))
            #self.heading += math.atan2((self.marker_y),(self.current_x))
            self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w = quaternion_from_euler(0,0,self.heading + self.heading_adjust)
            self.setpoint_pub.publish(self.pose)

            print("heading: ", self.heading + self.heading_adjust)
            print("Yaw: ", self.current_yaw)
            if self.heading - 0.01 < self.current_yaw and self.current_yaw < self.heading + 0.01:
                self.state = 3
        #Align
        elif self.state == 3:
            self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w = quaternion_from_euler(0,0.02,self.heading)
            self.setpoint_pub.publish(self.pose)

            self.current_dist = math.sqrt(self.marker_x**2 + self.marker_y**2)
            print("current_dist: ", self.current_dist)
            if self.current_dist < 3:
                self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w = quaternion_from_euler(0,0,self.heading)
                self.pose.position.z = 5
                self.setpoint_pub.publish(self.pose)
                self.state = 4

        elif self.state == 4:
            self.heading = (math.atan2((self.current_y-self.aruco_y ),(self.current_x-self.aruco_x)))
            self.heading += math.atan2((self.current_y),(self.current_x))
            #self.heading = (math.atan2((self.aruco_y+self.current_y),(self.aruco_x+self.current_x)))
            self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w = quaternion_from_euler(0,0,self.heading + self.heading_adjust)
            self.setpoint_pub.publish(self.pose)
            print ("aruco position", self.aruco_x + self.current_x, self.aruco_y + self.current_y)
            print("heading: ", self.heading + self.heading_adjust)
            print("Yaw: ", self.current_yaw)
            if self.heading - 0.01 < self.current_yaw and self.current_yaw < self.heading + 0.01:
                self.state = 5

        elif self.state == 5:
            if abs(self.heading) > math.pi/2:
                pitch = 0.01
            else:
                pitch = 0.01

            self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w = quaternion_from_euler(0,pitch,self.heading)
            self.setpoint_pub.publish(self.pose)

            self.current_dist = math.sqrt(self.aruco_x**2 + self.aruco_y**2)
            print("current_dist: ", self.current_dist)
            if self.current_dist < 1.2:
                self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w = quaternion_from_euler(0,0,self.heading)
                self.setpoint_pub.publish(self.pose)
                self.pose.position.z = 2
                self.setpoint_pub.publish(self.pose)
                self.state = 6

        elif self.state == 6:

            self.pose.position.z = 1
            self.setpoint_pub.publish(self.pose)
            pass


if __name__ == "__main__":
    node = QuadStateNode()
