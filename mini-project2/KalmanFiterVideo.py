#!/usr/bin/python
import rospy
import cv2
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import math
import random

class KalmanFilterVideo:
    def __init__(self, ID, x_pos, y_pos):
        self.ID = ID
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.speed = 0
        self.dt = 1.0

        self.KF = KalmanFilter(dim_x=4, dim_z=2)
        self.KF.x = np.array([0.,0.,0.,0.])
        self.KF.P = np.array([  [1000., 0., 0., 0.],
                                [ 0., 1000., 0., 0.],
                                [0., 0., 1000., 0.],
                                [0., 0., 0., 1000.]])

        self.KF.Q = np.eye(4) * .01
        self.KF.R = np.array([  [15., 0.],
                                [0., 15.]])

        self.KF.H = np.array([  [1.,0.,0.,0.,],
                                [0.,0.,1.,0.,]])

        self.KF.F = np.array(([ [1., self.dt, 0., 0.],
                                [0., 1., 0., 0.],
                                [0., 0., 1., self.dt],
                                [0., 0., 0., 1.]]))

    def KalmanPrediction(self):
        self.KF.predict()

    def updateValues(self, new_x_pos, new_y_pos):
        self.KF.update([[new_x_pos], [new_y_pos]])
        self.speed = math.sqrt(math.pow(float(self.KF.x[1]),2) + math.pow(float(self.KF.x[3]),2))

class KfArray:
    def __init__(self):
        self.arr = []

    def CheckID(self, ID, x_pos, y_pos):
        itemExists = 0
        for elem in arr:
            if elem.ID == ID:
                itemExists += 1
                elem.updateValues(x_pos, y_pos)
        if itemExists == 0:
            KF = KalmanFilterVideo(ID=ID, x_pos=x_pos, y_pos=y_pos)
            self.arr.append(KF)

    def arrayPredict(self):
        for elem in arr:
            elem.KalmanPrediction()

def drawSquares(img, x, y):
    cv2.rectangle(img, ((x-3),(y-8)),((x+3),(y+8)))

class KFros:
    def __init__(self):
        rospy.init_node('KF', anonymous=True)
        self.image_sub = rospy.Subscriber("Image_BG_filtered", data, self.callback)
        #self.image_pub = rospy.Publisher("KF_position_estimate", self.KF_array, queue_size=10)
        self.KF_array = KfArray()

    def callback(self, data):
        self.KF_array.arrayPredict()
        if len(data) > 1:
            self.KF_array.CheckID(data[0], data[1], data[2])



if __name__ == '__main__':
        print("Launching Kalman filter")
        kf = KFros()
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()






    #end
