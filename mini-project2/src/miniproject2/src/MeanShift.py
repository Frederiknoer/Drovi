import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge

class MeanShift:
    def __init__(self, roi):
        self.term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
        self.track_window = (roi[0],roi[1],roi[2],roi[3]) #c,r,w,h
        #print roi
        self.pos = roi[0],roi[1]


    def update(self, img):
        ret, self.track_window = cv2.meanShift(img,self.track_window, self.term_crit)
        if ret == True:
            self.pos = self.track_window[0],self.track_window[1]


class MSros:
    def __init__(self):
        rospy.init_node('MS', anonymous=True)
        self.image_sub = rospy.Subscriber("Image_BG_filtered", Image, self.callback)
        self.arr_pub = rospy.Publisher("MsArray",Int16MultiArray,queue_size=10)
        self.MsArray = []


    def callback(self, data):
        img = CvBridge().imgmsg_to_cv2(data)

        #print cv2.findNonZero(img)
        xt, yt, wt, ht = 644, 278, 48, 9
        ROItop = img[yt:yt+ht, xt:xt+wt]
        xb, yb, wb, hb = 990, 1034, 74, 23
        ROIbot = img[yb:yb+hb, xb:xb+wb]

        thrs = 100

        #check if a car is entering at the top
        roisum = 0
        roisum = cv2.sumElems(ROItop)
#        for x_top in range(0,wt):
#            for y_top in range(0,ht):
#                roisum += ROItop[y_top,x_top]
                #print ROItop[x_top,y_top]
        if roisum > thrs:
            ms = MeanShift([xt,yt,wt,ht])
            self.MsArray.append(ms)

        #Check if a car is entering at the bottom
        roisum = 0
        roisum = cv2.sumElems(ROItop)
#        for x_bot in range(0,wb):
#            for y_bot in range(0,hb):
#                roisum += ROIbot[y_bot,x_bot]
        if roisum > thrs:
            ms = MeanShift([xb,yb,wb,hb])
            self.MsArray.append(ms)

        #update all the cars in the mean shift array
        for obj in self.MsArray:
            obj.update(img)

        retArray = []
        for obj in self.MsArray:
            retArray.append(obj.pos[0])
            retArray.append(obj.pos[1])
        #print retArray

        self.arr_pub.publish(data=retArray)

if __name__ == '__main__':
        print("Launching Mean shift algorithm")
        msr = MSros()
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()
