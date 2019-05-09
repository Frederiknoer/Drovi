#!/usr/bin/python
import rospy
import cv2
from fiducial.nFoldEdges.MarkerLocator import markerTracker, MarkerPose
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge
import random
import message_filters
import math



class markerDetection:
    def __init__(self):
        print "hej"



if __name__ == '__main__':
    print "marker detection node started"
    mark = markerDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
