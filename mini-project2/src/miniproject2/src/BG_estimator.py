#!/usr/bin/python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
from miniproject2.msg import Car, Cars


class Tracker:
    def __init__(self,frame,x,y,id):
        self.car = Car()
        self.car.id = id
        self.car.x = x
        self.car.y = y

        self.pos = np.array([x,y])
        self.timestamp = rospy.rostime.get_rostime()
        # avg color ?
        self.term_crit = (cv2.TERM_CRITERIA_COUNT, 8, 1)
        #self.roi = np.array("I",[x-20,y-20,x+20,y+20])

        #self.roi = frame[y-20:y+20 , x-20:x+20]
        self.track_window = (int(x-10),int(y-30),int(20),int(60))

        #hsv_roi = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #mask = cv2.inRange(hsv_roi, np.array((0., 60., 32.)), np.array((180., 255., 255.)))
        #roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0, 180])


    def update_pos(self,frame):
#        cv2.calcBackProject(frame,[0],roi_hist,[0,180],1)
        if self.isEmpty(frame):
            return False
        else:
            ret, self.track_window = cv2.meanShift(frame, self.track_window, self.term_crit)
            self.car.x = self.track_window[0] + 10
            self.car.y = self.track_window[1] + 30
            self.car.roi = self.track_window
        return True

    def isEmpty(self, img):
        roi = img[int(self.track_window[1]):int(self.track_window[1]) + int(self.track_window[3]), int(self.track_window[0]):int(self.track_window[0]) + int(self.track_window[2])]
        #print (int(self.track_window[1]),int(self.track_window[1]) + int(self.track_window[3]), int(self.track_window[0]),int(self.track_window[0]) + int(self.track_window[2]))
        return cv2.sumElems(roi)[0] < 1000.0

    def isUnique(self,arr):
        count = 0
        for obj in arr:
            if(count > 0):
                return False
            if self == obj and self.car.id > obj.car.id:
                #print self.track_window, "   :   ", obj.track_window
                count += 1
        return True

    def __eq__(self, other):
        #print np.array(self.track_window), "   :    ", np.array(other.track_window[:])
        return self.track_window[0] == other.track_window[0] and self.track_window[1] == other.track_window[1]


class BackGroundFilter:
    def __init__(self):
        rospy.init_node('BG_estimator', anonymous=True)


        self.track_list = []

        self.startTime = rospy.get_rostime()

        self.TOPid = 0
        self.BOTid = 1

        self.bridge = CvBridge()
        self.roi = np.array([1920/2-400,100,1920/2+100,1000])
        self.upperleft = np.array([47,187])
        self.buttomright = np.array([210,867])
        # calculate H
        pIn = np.array([[543, 559], [1173, 127], [1735, 271], [478, 900]], np.float32)  # Pic coordinates
        pOut = np.array([[75, 703], [410, 113], [579, 479], [79, 872]], np.float32)  # meters real world
        self.H = cv2.getPerspectiveTransform(pIn, pOut)

        self.image_pubPers = rospy.Publisher("Image_original", Image, queue_size=10)


        self.fgbg = cv2.createBackgroundSubtractorMOG2(detectShadows=False)
        self.kernel = np.ones((3,3),np.uint8)
        self.kernel2 = np.ones((5,5), np.uint8)
        self.lowerdetector = np.array([800,1920/2-400,1920/2+100])

        self.image_pub = rospy.Publisher("Image_BG_filtered", Image, queue_size=10)
        self.car_pub = rospy.Publisher("Cars_list",Cars,queue_size=10)
        self.image_sub = rospy.Subscriber("analyzed_image", Image, self.callback)

    def changeKernel(self,data):
        self.kernel = np.ones((data,data),np.uint8)

    def callback(self, data):
        header = Header()
        header.stamp = (rospy.get_rostime() - self.startTime)

        data.header = header
        frame = CvBridge().imgmsg_to_cv2(data,'bgr8')
        self.image_pubPers.publish(data)
        #frame1 = warp = cv2.warpPerspective(frame, self.H, (579, 1000))#[self.upperleft[1]:self.buttomright[1],self.upperleft[0]:self.buttomright[0]]
        #self.image_pubPers.publish(self.bridge.cv2_to_imgmsg(frame1, "bgr8"))
        #print np.shape(warp),"\n", warp

        #frm = frame[self.upperleft[1]:self.buttomright[1],self.upperleft[0]:self.buttomright[0]]
        #frm = frame[:, self.roi[0]:self.roi[2]]
#        frm = frame[self.roi[1]:self.roi[3], self.roi[0]:self.roi[2]]

        listOfCars = []

        fgmask = self.fgbg.apply(frame)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, self.kernel,iterations=1)
        fgmask = cv2.morphologyEx(fgmask,cv2.MORPH_DILATE,self.kernel,iterations=2)

        cc, labels, stat, cent = cv2.connectedComponentsWithStats(fgmask, 4, cv2.CV_32S)
        #print cc


        for i in cent:
            if (i[1] > 1000 and i[1] < 1010 and i[0] > 970 and i[0] < 1040):
                self.track_list.append(Tracker(fgmask,i[0],i[1],self.BOTid))
                self.BOTid += 2
            elif (i[1] > 245 and i[1] < 255 and i[0] > 650 and i[0] < 700):
                self.track_list.append(Tracker(fgmask, i[0], i[1], self.TOPid))
                self.TOPid += 2

        # for ind, i in enumerate(cent):
        #     #print stat[ind][4]
        #     if stat[ind][4] < 2000 and stat[ind][4]>5:
        #         cv2.circle(fgmask,(int(i[0]),int(i[1])),20,np.array([200,10,200]),2)

        #for obj in self.track_list:
            #cv2.rectangle(fgmask,(obj.car.roi[0],obj.car.roi[1]),(obj.car.roi[0]+obj.car.roi[2],obj.car.roi[1]+obj.car.roi[3]),127,thickness=4)

        BG_filtered = CvBridge().cv2_to_imgmsg(fgmask)



        for i, obj in  enumerate(self.track_list):
            if not obj.isUnique(self.track_list):
                #print obj.track_window
                del self.track_list[i]
            elif not obj.update_pos(fgmask):
                self.track_list.remove(obj)
            else:
                listOfCars.append((obj.car))


        self.car_pub.publish(header=header,listOfCars=listOfCars)
        self.image_pub.publish(BG_filtered)
#        cv2.imshow('frame',fgmask_morph)
#        k = cv2.waitKey(30) & 0xff
#         ld = frame[self.lowerdetector[0],self.lowerdetector[1]:self.lowerdetector[2], :]  # self.lowerdetector[1]:self.lowerdetector[2]]
#         self.image_pubPers.publish(self.bridge.cv2_to_imgmsg(ld))


if __name__ == '__main__':
    print("Launching Background removal filter")
    BG = BackGroundFilter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
