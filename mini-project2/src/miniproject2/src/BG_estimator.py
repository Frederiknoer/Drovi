#!/usr/bin/python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Car:
    def __init__(self,x,y,frame):
        self.pos = np.array('I',[x,y])
        self.timestamp = rospy.rostime.get_rostime()
        # avg color ?
        self.term_crit = (cv2.TERM_CRITERIA_COUNT, 10)
        #self.roi = np.array("I",[x-20,y-20,x+20,y+20])

        self.roi = frame[y-20:y+20, x-20:x + 20]
        hsv_roi = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_roi, np.array((0., 60., 32.)), np.array((180., 255., 255.)))
        roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0, 180])


    def update_pos(self,frame):
        cv2.calcBackProject(frame,[0],roi_hist,[0,180],1)





class BackGroundFilter:
    def __init__(self):
        rospy.init_node('BG_estimator', anonymous=True)

        self.image_pub = rospy.Publisher("Image_BG_filtered", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("analyzed_image", Image, self.callback)
        self.bridge = CvBridge()
        self.roi = np.array([1920/2-400,100,1920/2+100,1000])
        self.upperleft = np.array([47,187])
        self.buttomright = np.array([210,867])
        # calculate H
        pIn = np.array([[543, 559], [1173, 127], [1735, 271], [478, 900]], np.float32)  # Pic coordinates
        pOut = np.array([[75, 703], [410, 113], [579, 479], [79, 872]], np.float32)  # meters real world
        self.H = cv2.getPerspectiveTransform(pIn, pOut)
        self.image_pubPers = rospy.Publisher("Image_Perspect_filtered", Image, queue_size=10)


        self.fgbg = cv2.createBackgroundSubtractorMOG2(detectShadows=False)
        self.kernel = np.ones((3,3),np.uint8)
        self.kernel2 = np.ones((5,5), np.uint8)
        self.lowerdetector = np.array([800,1920/2-400,1920/2+100])

    def changeKernel(self,data):
        self.kernel = np.ones((data,data),np.uint8)

    def callback(self, data):
        frame = CvBridge().imgmsg_to_cv2(data,'bgr8')
        #frame1 = warp = cv2.warpPerspective(frame, self.H, (579, 872))[self.upperleft[1]:self.buttomright[1],self.upperleft[0]:self.buttomright[0]]
        #self.image_pubPers.publish(self.bridge.cv2_to_imgmsg(frame1, "bgr8"))

        #frm = frame[self.upperleft[1]:self.buttomright[1],self.upperleft[0]:self.buttomright[0]]
        #frm = frame[:, self.roi[0]:self.roi[2]]
#        frm = frame[self.roi[1]:self.roi[3], self.roi[0]:self.roi[2]]


        fgmask = self.fgbg.apply(frame)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, self.kernel,iterations=2)
        fgmask = cv2.morphologyEx(fgmask,cv2.MORPH_DILATE,self.kernel,iterations=2)

        cc, labels, stat, cent = cv2.connectedComponentsWithStats(fgmask, 4, cv2.CV_32S)
        #print cc

        for ind, i in enumerate(cent):
            #print stat[ind][4]
            if stat[ind][4] < 2000 and stat[ind][4]>5:
                cv2.circle(fgmask,(int(i[0]),int(i[1])),20,np.array([200,10,200]),2)

        for i in cent:
            for j in cent:
                if i[:].any() != j[:].any():
                    break

        #print np.shape(fgmask)
        BG_filtered = CvBridge().cv2_to_imgmsg(fgmask)

        self.image_pub.publish(BG_filtered)
#        cv2.imshow('frame',fgmask_morph)
#        k = cv2.waitKey(30) & 0xff
        ld = frame[self.lowerdetector[0],self.lowerdetector[1]:self.lowerdetector[2], :]  # self.lowerdetector[1]:self.lowerdetector[2]]
        self.image_pubPers.publish(self.bridge.cv2_to_imgmsg(ld))


if __name__ == '__main__':
    print("Launching Background removal filter")
    BG = BackGroundFilter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()






