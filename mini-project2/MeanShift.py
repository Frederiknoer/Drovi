import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class MeanShift:
    def __init__(self):
        self.term_crit = ( cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 1 )
        r,h,c,w = 250,90,400,125  # simply hardcoded the values
        self.track_window = (c,r,w,h)

    def update():
        ret, self.track_window = cv.meanShift(dst, self.track_window, self.term_crit)


class MSros:
    def __init__(self):
        rospy.init_node('MS', anonymous=True)
        self.image_sub = rospy.Subscriber("Image_BG_filtered", img, self.callback)

        self.MsArray = []
        self.ROItop = cv2.selectROI


    def callback(self, img):
        roisum = 0
        thrs = 10

        for x in range(width):
            for y in range(height):
                roisum += img[x,y]

        if roisum > thrs:
            ms = MeanShift()
            self.MsArray.append(ms)

        for obj in self.MsArray:
            obj.update()

if __name__ == '__main__':
        print("Launching Mean shift algorithm")
        msr = MSros()
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()
