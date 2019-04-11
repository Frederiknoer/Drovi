#!/usr/bin/python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge
from miniproject2.msg import Car, Cars

class DrawTracking:
    def __init__(self):
        rospy.init_node('Draw_Tracking', anonymous=True)

        self.image_pub = rospy.Publisher("Image_draw_tracking", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("analyzed_image", Image, self.callback)
        #self.tracking_sub = rospy.Subscriber("MsArray", Int16MultiArray, self.callback_track)
        self.tracking_sub = rospy.Subscriber("Cars_list", Cars, self.callback_track)
        self.bridge = CvBridge()
        self.car_list = []

    def callback_track(self, data):
        self.car_list = data.listOfCars


    def callback(self, data):
        frame = CvBridge().imgmsg_to_cv2(data)

        #for i, k in zip(data[0::2], data[1::2]):
            #print str(i), '+', str(k), '=', str(i + k)

        #print self.car_list

        # for i in range(0,len(self.car_list[:]),4):
        #     x = i
        #     y =i+1
        #     w = i+2
        #     h=i+3
        #     cv2.rectangle(frame, (self.car_list[x], self.car_list[y]), (self.car_list[x] + self.car_list[w], self.car_list[y] + self.car_list[h]), np.array([0,0,255]), 2)
        #     #cv2.circle(frame,(self.MsArray[i],self.MsArray[i+1]),20,np.array([100,4,100]),5)
        for car in self.car_list:
            #cv2.rectangle(frame, (car.roi[0], car.roi[1] ,car.roi[0] +car.roi[2],car.roi[1]+car.roi[3]), np.array([0, 0, 255]), 2)
            #print car.id, "   ",  car.roi
            cv2.rectangle(frame, car.roi,np.array([0, 0, 255]), 2)

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






