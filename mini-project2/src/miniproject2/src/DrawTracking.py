#!/usr/bin/python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge

class DrawTracking:
    def __init__(self):
        rospy.init_node('Draw_Tracking', anonymous=True)

        self.image_pub = rospy.Publisher("Image_draw_tracking", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("analyzed_image", Image, self.callback)
        self.tracking_sub = rospy.Subscriber("MsArray", Int16MultiArray, self.callback_track)
        self.bridge = CvBridge()
        self.MsArray = []

    def callback_track(self,data):
        self.MsArray = data.data


    def callback(self, data):
        frame = CvBridge().imgmsg_to_cv2(data)

        #for i, k in zip(data[0::2], data[1::2]):
            #print str(i), '+', str(k), '=', str(i + k)
        clone = cv2.copyMakeBorder(frame,0,0,0,0,cv2.BORDER_REFLECT)
        #print self.MsArray

        for i in range(0,len(self.MsArray[:]),2):
            #print i
            cv2.circle(frame,(self.MsArray[i],self.MsArray[i+1]),20,np.array([100,4,100]),5)

        Tracker = CvBridge().cv2_to_imgmsg(frame,'bgr8')
        #cv2.imshow("test",clone)

        #cv2.waitKey(0)
        #print np.shape(clone), "   -   ", np.size(clone)
        #print np.shape(Tracker), "   -   ", np.size(Tracker)
        self.image_pub.publish(Tracker)


if __name__ == '__main__':
    print("Launching Background removal filter")
    DT = DrawTracking()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()






