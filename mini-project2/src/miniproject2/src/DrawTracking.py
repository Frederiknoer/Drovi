#!/usr/bin/python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge
from miniproject2.msg import Car, Cars
import random

class DrawTracking:
    def __init__(self):
        rospy.init_node('Draw_Tracking', anonymous=True)

        self.image_pub = rospy.Publisher("Image_draw_tracking", Image, queue_size=10)

        self.image_sub = rospy.Subscriber("analyzed_image", Image, self.callback_img)
        self.tracking_sub_kf = rospy.Subscriber("KF_list", Cars, self.callback)
        self.tracking_sub_kf_proj = rospy.Subscriber("KF_list_proj", Cars, self.callback_vel)

        self.bridge = CvBridge()
        self.car_list = []
        self.car_list_vel = []
        self.frame = np.zeros((1920,1080,3), np.uint8)

    def callback_img(self, img):
        self.frame = np.zeros((1920,1080,3), np.uint8)
        self.frame = CvBridge().imgmsg_to_cv2(img)
        #print("img income")

    def callback_vel(self, data):
        self.car_list_vel = data.listOfCars

    def callback(self, data):
        #print("Data income")
        font = cv2.FONT_HERSHEY_PLAIN
        even_x = 1800
        odd_x = 0
        colour = (255,255,255)
        txt_scale = 2
        radius = 10

        self.car_list = data.listOfCars

        for car in self.car_list:
            #colour = (random.randint(0,255), random.randint(0,255), random.randint(0,255))
            cv2.circle(self.frame, (car.x, car.y), radius, colour)

            imgtext1 = "ID: " + str(car.id)
            imgtext2 = ""
            for vel_car in self.car_list_vel:
                if vel_car.id == car.id:
                    imgtext2 = "Vel: " + str(vel_car.vel)
                    break

            if car.id % 2 == 0: #even
                pt1 = (car.x + (radius/2), car.y)

                cv2.line(self.frame, pt1,(even_x, car.y), colour)
                cv2.putText(self.frame, imgtext1,(even_x, car.y), font, txt_scale, colour)
                cv2.putText(self.frame, imgtext2,(even_x, car.y-40), font, txt_scale, colour)

            elif car.id % 2 == 1: #odd
                pt1 = (car.x - (radius/2), car.y)

                cv2.line(self.frame, pt1,(odd_x, car.y), colour)
                cv2.putText(self.frame, imgtext1,(odd_x, car.y), font, txt_scale, colour)
                cv2.putText(self.frame, imgtext2,(odd_x, car.y-40), font, txt_scale, colour)


        Tracker = CvBridge().cv2_to_imgmsg(self.frame,'bgr8')
        self.image_pub.publish(Tracker)


if __name__ == '__main__':
    print("Launching Drawing function")
    DT = DrawTracking()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
