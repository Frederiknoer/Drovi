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

    def callback(self,data):

        for car in data.listOfCars:
            #pos = np.array([[car.x],[car.y],[1.0]])
            pos = np.array([car.x, car.y, 1.0])
            #proj = np.multiply(pos,self.H)
            proj = self.H.dot(pos)
            #print proj
            car.x = int(proj[0])
            car.y = int(proj[1])

        self.car_pub.publish(data.listOfCars)

if __name__ == '__main__':
    print("Launching transform coordinates")
    PT = Projection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


