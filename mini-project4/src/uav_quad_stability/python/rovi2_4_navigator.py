#!/usr/bin/env python
import numpy as np
import rospy
import cv2
from markerlocator.msg import markerpose
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu


class UAVNavigator:
    def __init__(self):
        rospy.init_node('uav_navigator')
        # Initialize variables
        self.current_markerpose = markerpose()
        self.yaw_stepsize = 0.1
        self.yaw_last = 0
        self.uav_yaw = 0

        # Initialize subscribers
        self.markerpose_sub = rospy.Subscriber("/markerlocator/markerpose", markerpose, self.set_markerpose)
        self.uavpose_sub = rospy.Subscriber("/hummingbird/imu", Imu, self.set_uav_pose)

        # Initialize publishers
        self.setpoint_pub = rospy.Publisher("/uav_setpoint", Pose, queue_size=1)

        # Setup the callback timer, aka. 'main' function
        rospy.Timer(rospy.Duration(1./100.), self.timer_callback)
        rospy.spin()


    def set_markerpose(self, msg):
        self.current_markerpose = msg

    def set_uav_pose(self, msg):
        self.uav_yaw = msg.orientation.z


    def navigate(self, mp):
        pout = Pose()

        
        if (mp.x > 0):
            pout.orientation.x = -0.01
        elif (mp.x < 0):
            pout.orientation.x = 0.01

        if (mp.y > 0):
            pout.orientation.y = -0.01
        elif (mp.y < 0):
            pout.orientation.y = 0.01
        
        '''
        # The yaw cannot move from 0.99 to -0.99, meaning that it will try to spin a whole rotation, which is not optimal.

        # if (mp.theta < 0 and mp.theta < self.yaw_last):
        if (mp.theta < -0.1):
            self.uav_yaw += -self.yaw_stepsize
        # elif (mp.theta > 0 and mp.theta > self.yaw_last):
        elif (mp.theta > 0.1):
            self.uav_yaw += self.yaw_stepsize

        if (self.uav_yaw < -1):
            self.uav_yaw = 2 + self.uav_yaw
        elif(self.uav_yaw > 1):
            self.uav_yaw = -2 + self.uav_yaw

        self.yaw_last = mp.theta
        pout.orientation.z = self.uav_yaw
        '''

        return pout



    def timer_callback(self, event):
        position_out = self.navigate(self.current_markerpose)
        position_out.position.z = 15
        print(self.current_markerpose.x, self.current_markerpose.y)
        print(position_out.orientation.x, position_out.orientation.y, position_out.orientation.z, position_out.position.z)
        self.setpoint_pub.publish(position_out)



if __name__ == "__main__":
    node = UAVNavigator()
