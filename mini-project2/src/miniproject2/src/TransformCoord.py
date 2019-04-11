#!/usr/bin/python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from miniproject2.msg import Car, Cars



class Projection:
    def __init__(self):
        rospy.init_node('TransformationCoord', anonymous=True)

        pIn = np.array([[543, 559], [1173, 127], [1735, 271], [478, 900]], np.float32)  # Pic coordinates
        pOut = np.array([[75, 703], [410, 113], [579, 479], [79, 872]], np.float32)  # meters real world
        self.H = cv2.getPerspectiveTransform(pIn, pOut)

        self.car_pub = rospy.Publisher("Cars_list_proj",Cars,queue_size=10)
        self.car_sub = rospy.Subscriber("Cars_list",Cars,self.callback)
        #self.image_sub = rospy.Subscriber("Image_Perspect_filtered",Image,self.callback_image)
        #self.image_pub = rospy.Publisher("Image_perspect_test",Image,queue_size=10)
        self.frame = np.array([])

    def callback_image(self,data):
        self.frame = CvBridge().imgmsg_to_cv2(data, 'bgr8')


    def callback(self,data):
        header = data.header
        for car in data.listOfCars:
            #print self.H, "\n"
            pos = np.array([[car.x],[car.y],[1.0]])

            #pos = np.array([car.x, car.y, 1.0])
            #proj = np.multiply(pos,self.H)
            #print pos
            #proj = self.H.dot(pos)
            w = (self.H[2][0]*pos[0] + self.H[2][1]*pos[1] + self.H[2][2])
            proj_x = (self.H[0][0]*pos[0] + self.H[0][1]*pos[1] + self.H[0][2])/w
            proj_y = (self.H[1][0] * pos[0] + self.H[1][1] * pos[1] + self.H[1][2]) / w
            #print proj, "\n"
            car.x = int(proj_x)
            car.y = int(proj_y)
            cv2.circle(self.frame,(car.x,car.y),20,np.array([200,10,200]),2)
        #perspect_image = CvBridge().cv2_to_imgmsg(self.frame,'bgr8')
        #self.image_pub.publish(perspect_image)
        self.car_pub.publish(data)#header=header,listOfCars=data.listOfCars)


if __name__ == '__main__':
    print("Launching transform coordinates")
    PT = Projection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


