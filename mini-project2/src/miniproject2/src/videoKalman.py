#!/usr/bin/python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

fgbg = cv2.createBackgroundSubtractorMOG2(detectShadows=False)

kernel = np.ones((5,5),np.uint8)

#def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def kalman(data):

    #vid = cv2.VideoCapture("../2017_06_23_dyrskuepladsen.mp4")
    frame = CvBridge().imgmsg_to_cv2(data,'bgr8')

    #print np.shape(frame)


    fgmask = fgbg.apply(frame)
    fgmask_morph = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)

    cv2.imshow('frame',fgmask_morph)
    k = cv2.waitKey(30) & 0xff


def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("analyzed_image", Image, kalman)
    rospy.spin()

if __name__ == '__main__':
    listener()






