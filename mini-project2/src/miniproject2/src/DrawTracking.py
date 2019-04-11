#!/usr/bin/python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge
from miniproject2.msg import Car, Cars
import random
import message_filters
import math

class DrawTracking:
    def __init__(self):
        rospy.init_node('Draw_Tracking', anonymous=True)

        self.image_pub = rospy.Publisher("Image_draw_tracking", Image, queue_size=5)

        #self.image_sub = rospy.Subscriber("analyzed_image", Image, self.callback)
        #self.tracking_sub_kf = rospy.Subscriber("KF_list", Cars, self.callback_pos)
        #self.tracking_sub_kf_proj = rospy.Subscriber("KF_list_proj", Cars, self.callback_vel)


        self.image_sub = message_filters.Subscriber("Image_original", Image)
        self.tracking_sub_kf = message_filters.Subscriber("KF_list", Cars)
        self.tracking_sub_kf_proj = message_filters.Subscriber("KF_list_proj", Cars)
        self.synchronizer = message_filters.TimeSynchronizer([self.image_sub,self.tracking_sub_kf,self.tracking_sub_kf_proj],10)
        self.synchronizer.registerCallback(self.callback)
        #self.synchronizer.registerCallback(self.callback)

        self.bridge = CvBridge()
        self.car_list = []
        self.car_list_vel = []
        self.sec_dt = 0.0
        self.sec_prev_t = 0.0
        self.ns_dt = 0.0
        self.ns_prev_t = 0.0

        self.dt = 0.0
        #self.frame = np.zeros((1080,1920,3), np.uint8)

    def callback_pos(self, data):
        self.car_list = []
        self.car_list = data.listOfCars

    def callback_vel(self, data):
        self.car_list_vel = []
        self.car_list_vel = data.listOfCars

    def callback(self, img,data,data_proj):
        print img.header.stamp, data.header.stamp, data_proj.header.stamp
        frame = CvBridge().imgmsg_to_cv2(img,'bgr8')
        self.car_list = data.listOfCars
        self.car_list_vel = data_proj.listOfCars

        #print("Data income")
        font = cv2.FONT_HERSHEY_PLAIN
        even_x = 0
        odd_x = 1600
        colour = np.array([255,255,255])
        txt_scale = 2
        radius = 12
        thickness = 2

        seconds = str((img.header.stamp.secs % 60))
        m_seconds = ":" + str(int(img.header.stamp.nsecs/math.pow(10,6)))
        minutes = str( int(img.header.stamp.secs/60) ) + ":"

        self.sec_dt = (img.header.stamp.secs - self.sec_prev_t)
        self.sec_prev_t = img.header.stamp.secs
        self.ns_dt = (img.header.stamp.nsecs - self.ns_prev_t)
        self.ns_prev_t = img.header.stamp.nsecs

        self.dt = (self.sec_dt) + (self.ns_dt/(math.pow(10,9)))


        time = (minutes + seconds + m_seconds)
        cv2.putText(frame, time,(75, 75), font, fontScale=5, color=np.array([0,0,255]), thickness=5)

        for car in self.car_list:
            cv2.circle(frame, (car.x, car.y), radius, colour, thickness)

            imgtext1 = "ID: " + str(car.id)
            imgtext2 = "No Vel"
            #print(len(self.car_list_vel))
            for vel_car in self.car_list_vel:
                if vel_car.id == car.id:
                    imgtext2 = "Vel: " + str((vel_car.vel/self.dt)*3.6)[0:4] + " Km/h"
                    break

            if car.id % 2 == 0: #even
                pt1 = (car.x - (radius/2), car.y)

                cv2.line(frame, pt1,(even_x, car.y), colour, thickness)
                cv2.putText(frame, imgtext1,(even_x, car.y), font, txt_scale, colour, thickness)
                cv2.putText(frame, imgtext2,(even_x, car.y+25), font, txt_scale, colour, thickness)

            elif car.id % 2 == 1: #odd
                pt1 = (car.x + (radius/2), car.y)

                cv2.line(frame, pt1,(odd_x, car.y), colour, thickness)
                cv2.putText(frame, imgtext1,(odd_x, car.y), font, txt_scale, colour, thickness)
                cv2.putText(frame, imgtext2,(odd_x, car.y+25), font, txt_scale, colour, thickness)

        Tracker = CvBridge().cv2_to_imgmsg(frame,'bgr8')
        self.image_pub.publish(Tracker)


if __name__ == '__main__':
    print("Launching Drawing function")
    DT = DrawTracking()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
