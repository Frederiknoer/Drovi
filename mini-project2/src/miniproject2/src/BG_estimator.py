#!/usr/bin/python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


#def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


class BackGroundFilter:
    def __init__(self):
        rospy.init_node('BG_estimator', anonymous=True)

        self.image_pub = rospy.Publisher("Image_BG_filtered", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("analyzed_image", Image, self.callback)
        self.bridge = CvBridge()
        rospy.get_param("kernelSize",self.changeKernel)


        self.fgbg = cv2.createBackgroundSubtractorMOG2(history=3,detectShadows=False)
        self.kernel = np.ones((5,5),np.uint8)

    def changeKernel(self,data):
        self.kernel = np.ones((data,data),np.uint8)

    def callback(self, data):
        frame = CvBridge().imgmsg_to_cv2(data,'bgr8')

        fgmask = self.fgbg.apply(frame)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, self.kernel,iterations=1)

        cc, labels, stat, cent = cv2.connectedComponentsWithStats(fgmask, 4, cv2.CV_32S)
        print cc

        for ind, i in enumerate(cent):
            print stat[ind][4]
            if stat[ind][4] < 200:
                cv2.circle(fgmask,(int(i[0]),int(i[1])),20,np.array([200,10,200]),2)


        BG_filtered = CvBridge().cv2_to_imgmsg(fgmask)

        self.image_pub.publish(BG_filtered)
#        cv2.imshow('frame',fgmask_morph)
#        k = cv2.waitKey(30) & 0xff


if __name__ == '__main__':
    print("Launching Background removal filter")
    BG = BackGroundFilter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()






