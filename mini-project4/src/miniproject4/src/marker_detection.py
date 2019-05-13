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

sys.path.insert(0, 'fiducial/nfoldedge/MarkerLocator/')

from MarkerPose import MarkerPose
from MarkerTracker import MarkerTracker


class markerDetection:
    def __init__(self):
        rospy.init_node("marker_detection",anonymous=True)

        tracker = MarkerTracker.MarkerTracker(
            order=4,
            kernel_size=20,
            scale_factor=1)
        # Tell the tracker that we are looking for markers without
        # an unique orientation
        tracker.track_marker_with_missing_black_leg = True

        self.image_sub = rospy.Subscriber("hummingbird/camera_/image_raw", Image, self.callback)



    def callback(self,data):

        frame = CvBridge().imgmsg_to_cv2(data, 'bgr8')







if __name__ == '__main__':
    print "marker detection node started"
    mark = markerDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
