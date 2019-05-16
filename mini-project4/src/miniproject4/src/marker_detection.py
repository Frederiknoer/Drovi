#!/usr/bin/python
import rospy
import cv2

import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge
import random
import message_filters
import math
import sys

from miniproject4.msg import markerpose

sys.path.insert(0, './fiducial/nfoldedge/')
from MarkerLocator import MarkerPose, MarkerTracker
#from miniproject4.MarkerLocator import MarkerPose, MarkerTracker


print sys.path

class markerDetection:
    def __init__(self):
        rospy.init_node("marker_detection",anonymous=True)

        self.tracker = MarkerTracker.MarkerTracker(
            order=4,
            kernel_size=20,
            scale_factor=1)
        # Tell the tracker that we are looking for markers without
        # an unique orientation
        self.tracker.track_marker_with_missing_black_leg = True

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

        self.parameters = cv2.aruco.DetectorParameters_create()


        self.image_sub = rospy.Subscriber("hummingbird/camera_/image_raw", Image, self.callback)
        self.markerpose_pub = rospy.Publisher("nFold_markerpose", markerpose,queue_size=10)
        self.image_pub = rospy.Publisher("nFold_markerimage", Image, queue_size=10)





    def callback(self,data):

        frame = CvBridge().imgmsg_to_cv2(data, 'bgr8')

        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        pose = self.tracker.locate_marker(gray)
        #print(pose.quality)

        markerpose_msg = markerpose()
        markerpose_msg.header = data.header
        markerpose_msg.order = pose.order
        markerpose_msg.x = pose.x
        markerpose_msg.y = pose.y
        markerpose_msg.theta = pose.theta
        markerpose_msg.quality = pose.quality

        if markerpose_msg.quality > 0.7:
            cv2.circle(frame, (pose.x, pose.y), 20, (0, 0, 255), 2)
        self.markerpose_pub.publish(markerpose_msg)

        #self.image_pub.publish(CvBridge().cv2_to_imgmsg(frame,'bgr8'))

        ##custom_dictionary = cv2.aruco.custom_dictionary(6, 4)
        #cv2.aruco.detectMarkers(frame)
        #markers = cv2.aruco.detectMarkers(gray, cv2.getPrede,parameters=parameters)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        self.determineHeight(corners[0]);
        frame_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)

        self.image_pub.publish(CvBridge().cv2_to_imgmsg(frame_markers, 'bgr8'))

    def determineHeight(self, corners):
        #diff = corners[0][0] - corners[0][3]
        crosssection = math.sqrt((corners[0][0][0] - corners[0][3][0])**2 + (corners[0][0][1] - corners[0][3][1])**2)
        print crosssection

        #position:
        #x: 2.7
        #y: 7.7
        #z: 1.6


if __name__ == '__main__':
    print "marker detection node started"
    mark = markerDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
