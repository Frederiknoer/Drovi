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

        self.f_x = KalmanFilter(dim_x=2, dim_z=1)
        self.f_y = KalmanFilter(dim_x=2, dim_z=1)

        self.f_x.x = np.array([[2.], [0.]])  # position, velocity
        self.f_x.F = np.array([[1., 1.], [0., 1.]])
        self.f_x.H = np.array([[1., 0.]])
        self.f_x.P = np.array([[1000.,    0.],
                        [   0., 1000.] ])
        self.f_x.R = np.array([[5.]])
        self.f_x.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.13)

        self.f_y.x = np.array([[2.], [0.]])  # position, velocity
        self.f_y.F = np.array([[1., 1.], [0., 1.]])
        self.f_y.H = np.array([[1., 0.]])
        self.f_y.P = np.array([[1000.,    0.],
                        [   0., 1000.] ])
        self.f_y.R = np.array([[5.]])
        self.f_y.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.13)

    def KalmanPrediction(self):
        self.f_x.predict()
        self.f_y.predict()

    def updateValues(self, new_x_pos, new_y_pos):
        self.f_x.update(new_x_pos)
        self.f_y.update(new_y_pos)


    def calcSpeed(self):
        self.speed = math.sqrt(math.pow(float(self.f_x.x[1]),2) + math.pow(float(self.f_y.x[1]),2))
        return self.speed

if __name__ == '__main__':

    vis = np.zeros((800, 800,1), np.uint8)
    kalmanImg = np.zeros((800, 800,1), np.uint8)

    cv2.namedWindow("Kalman")
    cv2.namedWindow("Video Feed")

    KF = KalmanFilterVideo(ID=1, x_pos=0, y_pos=0)

    for i in range(720):
        x = i + random.randint(-5,20)
        y = i + random.randint(-25,15)
        #vis[:][:] = 0
        vis[i+y][i+x] = 255

        KF.KalmanPrediction()
        KF.updateValues(new_x_pos=i+x, new_y_pos=i+y)
        print(KF.calcSpeed())

        #print(int(f_x.x[0]))
        kalmanImg[int(KF.f_x.x[0]), int(KF.f_y.x[0])] = 255
        cv2.imshow("Video Feed", vis)
        cv2.imshow("Kalman", kalmanImg)
        cv2.waitKey(20)
        #vis[i][i] = 255
