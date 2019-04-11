#!/usr/bin/python
import rospy
import cv2
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import math
import random
from miniproject2.msg import Car, Cars

class KalmanFilterVideo:
    def __init__(self, ID, x_pos, y_pos):
        self.ID = ID
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.speed = 0.0
        self.dt = 1.0
        self.update_counter = 0

        self.KF = KalmanFilter(dim_x=4, dim_z=2)
        self.KF.x = np.array([0.,0.,0.,0.])
        self.KF.P = np.array([  [1000., 0., 0., 0.],
                                [ 0., 1000., 0., 0.],
                                [0., 0., 1000., 0.],
                                [0., 0., 0., 1000.]])

        self.KF.Q = np.eye(4) * .001
        self.KF.R = np.array([  [5., 0.],
                                [0., 5.]])

        self.KF.H = np.array([  [1.,0.,0.,0.,],
                                [0.,0.,1.,0.,]])

        self.KF.F = np.array(([ [1., self.dt, 0., 0.],
                                [0., 1., 0., 0.],
                                [0., 0., 1., self.dt],
                                [0., 0., 0., 1.]]))

    def KalmanPrediction(self):
        self.KF.predict()
        self.x_pos = self.KF.x[0]
        self.y_pos = self.KF.x[2]

    def updateValues(self, new_x_pos, new_y_pos):
        self.KF.update([[new_x_pos], [new_y_pos]])
        self.speed = math.sqrt(math.pow(float(self.KF.x[1]),2) + math.pow(float(self.KF.x[3]),2))

class KfArray:
    def __init__(self):
        self.arr = []

    def CheckID(self, ID, x_pos, y_pos):
        itemExists = 0
        for elem in self.arr:
            if elem.ID == ID:
                itemExists += 1
                elem.updateValues(x_pos, y_pos)
                elem.update_counter = 0
        if itemExists == 0:
            KF = KalmanFilterVideo(ID=ID, x_pos=x_pos, y_pos=y_pos)
            self.arr.append(KF)

    def arrayPredict(self):
        for elem in self.arr:
            elem.KalmanPrediction()
            elem.update_counter += 1
            if elem.update_counter > 15:
                self.arr.remove(elem)
            elif elem.y_pos < 1 or elem.y_pos > 999:
                self.arr.remove(elem)
            elif elem.x_pos < 1 or elem.x_pos > 600:
                self.arr.remove(elem)

class KFros:
    def __init__(self):
        rospy.init_node('KF_proj', anonymous=True)
        self.car_sub = rospy.Subscriber("Cars_list_proj", Cars, self.callback)
        self.KF_pub = rospy.Publisher("KF_list_proj", Cars, queue_size=10)
        self.KF_array = KfArray()
        self.car_array_pub = []

    def callback(self, data):
        header = data.header
        if len(self.KF_array.arr) > 0:
            self.KF_array.arrayPredict()
        carlist = data.listOfCars
        self.car_array_pub = []

        if len(carlist) > 0:
            for car in carlist:
                self.KF_array.CheckID(car.id, car.x, car.y)

        for car in self.KF_array.arr:
            msg = Car()
            msg.id = car.ID
            msg.x = int(car.x_pos)
            msg.y = int(car.y_pos)
            msg.vel = car.speed * 0.504
            self.car_array_pub.append(msg)

        self.KF_pub.publish(header=header,listOfCars=self.car_array_pub)

if __name__ == '__main__':
        print("Launching projected Kalman filter")
        kf = KFros()
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()






    #end
