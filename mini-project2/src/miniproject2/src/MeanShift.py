import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class MeanShift:
    def __init__(self, roi):
        self.term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
        self.track_window = roi #c,r,w,h

    def update(img):
        ret, self.track_window = cv2.meanShift(img, self.track_window, self.term_crit)


class MSros:
    def __init__(self):
        rospy.init_node('MS', anonymous=True)
        self.image_sub = rospy.Subscriber("Image_BG_filtered", img, self.callback)

        self.MsArray = []


    def callback(self, img):
        x, y, w, h = 644, 278, 48, 9
        ROItop = img[y:y+h, x:x+w]
        x, y, w, h = 990, 1034, 74, 23
        ROIbot = img[y:y+h, x:x+w]

        thrs = 25

        #check if a car is entering at the top
        roisum = 0
        for x in range(w):
            for y in range(h):
                roisum += ROItop[x,y]
        if roisum > thrs:
            ms = MeanShift(ROItop)
            self.MsArray.append(ms)

        #Check if a car is entering at the bottom
        roisum = 0
        for x in range(w):
            for y in range(h):
                roisum += ROIbot[x,y]
        if roisum > thrs:
            ms = MeanShift(ROIbot)
            self.MsArray.append(ms)

        #update all the cars in the mean shift array
        for obj in self.MsArray:
            obj.update(img)

if __name__ == '__main__':
        print("Launching Mean shift algorithm")
        msr = MSros()
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()
