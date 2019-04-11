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

class DrawTracking:
    def __init__(self):
        rospy.init_node('Draw_Tracking', anonymous=True)

        self.image_pub = rospy.Publisher("Image_draw_tracking", Image, queue_size=1)

        self.image_sub = rospy.Subscriber("analyzed_image", Image, self.callback)
        self.tracking_sub_kf = rospy.Subscriber("KF_list", Cars, self.callback_pos)
        self.tracking_sub_kf_proj = rospy.Subscriber("KF_list_proj", Cars, self.callback_vel)
        #self.synchronizer = message_filters.TimeSynchronizer([self.image_pub,self.tracking_sub_kf,self.tracking_sub_kf_proj],1)

        self.bridge = CvBridge()
        self.car_list = []
        self.car_list_vel = []
        #self.frame = np.zeros((1080,1920,3), np.uint8)

    def callback_pos(self, data):
        self.car_list = []
        self.car_list = data.listOfCars

    def callback_vel(self, data):
        self.car_list_vel = []
        self.car_list_vel = data.listOfCars

    def callback(self, img):
        frame = CvBridge().imgmsg_to_cv2(img)
        frame_clone = frame.copy()

        #print("Data income")
        font = cv2.FONT_HERSHEY_PLAIN
        even_x = 0
        odd_x = 1800
        colour = np.array([255,255,255])
        txt_scale = 2
        radius = 12
        thickness = 2


        for car in self.car_list:
            cv2.circle(frame_clone, (car.x, car.y), radius, colour, thickness)

            imgtext1 = "ID: " + str(car.id)
            imgtext2 = "No Vel"
            #print(len(self.car_list_vel))
            for vel_car in self.car_list_vel:
                if vel_car.id == car.id:
                    imgtext2 = "Vel: " + str(vel_car.vel)
                    break

            if car.id % 2 == 0: #even
                pt1 = (car.x - (radius/2), car.y)

                cv2.line(frame_clone, pt1,(even_x, car.y), colour, thickness)
                cv2.putText(frame_clone, imgtext1,(even_x, car.y), font, txt_scale, colour, thickness)
                cv2.putText(frame_clone, imgtext2,(even_x, car.y+25), font, txt_scale, colour, thickness)

            elif car.id % 2 == 1: #odd
                pt1 = (car.x + (radius/2), car.y)

                cv2.line(frame_clone, pt1,(odd_x, car.y), colour, thickness)
                cv2.putText(frame_clone, imgtext1,(odd_x, car.y), font, txt_scale, colour, thickness)
                cv2.putText(frame_clone, imgtext2,(odd_x, car.y+25), font, txt_scale, colour, thickness)

        Tracker = CvBridge().cv2_to_imgmsg(frame_clone,'bgr8')
        self.image_pub.publish(Tracker)


if __name__ == '__main__':
    print("Launching Drawing function")
    DT = DrawTracking()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
