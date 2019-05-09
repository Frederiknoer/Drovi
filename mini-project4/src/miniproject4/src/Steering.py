#!/usr/bin/python

import rospy
import numpy as np
import math

class Stabilize:
    def __init__(self):
        self.gyro_x = 0
        self.gyro_y = 0
        self.gyro_z = 0

        self.acc_x = 0
        self.acc_y = 0
        self.acc_z = 0

        self.set_roll = 0
        self.set_pitch = 0
        self.set_yaw = 0
        self.set_alt = 0

    def update_imu(self, data):
        pass

    def update_set_point(self, r, p, y, alt):
        self.set_roll = r
        self.set_pitch = p
        self.set_yaw = y
        self.set_alt = alt

    def stabilize(self):
        motor_speed = [None]*4

        return motor_speed
        pass



class Steer:
    def __init__(self):
        pass

    def steer_setpoint(self, x, y, z):
        pass



class Ros_steer_stabilize(self):
    def __init__(self):
        rospy.init_node('Steer_stabilize', anonymous=True)
        self.stab = Stabilize()
        self.stab.stabilize()
        self.steer = Steer()



    def callback(self, data):
        pass


if __name__ == '__main__':
    pass
