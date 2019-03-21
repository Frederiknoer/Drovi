#!/usr/bin/python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
import sys
import string
from cv_bridge import CvBridge, CvBridgeError


class image_converter:
  def __init__(self):
    rospy.init_node('image_converter', anonymous=True)
    self.image_pub = rospy.Publisher("analyzed_image", Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_raw", Image, self.callback)

    self.stabilizer = VideoStabilizer([[409, 250], [300, 762], [1123, 848],
        [1600, 238]])


  def callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv_image = self.analyze_image(cv_image)
    # self.showImage(cv_image)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)


  def analyze_image(self, image):
    image = self.stabilizer.stabilize_frame(image)
    return image


  def showImage(self, image):
    cv2.imshow("Image window", image)
    cv2.waitKey(3)


def stream(pathtovid):
    print pathtovid
    pub = rospy.Publisher("vid",Image,queue_size=10)
    rospy.init_node('talker')
    rate = rospy.Rate(10)
    vid = cv2.VideoCapture(pathtovid[1])
    #msg = Image()
    while not rospy.is_shutdown():
        ret, frame = vid.read()
        #print np.array([frame[:,0,0]]) , "  ", ret
        #if ret > 0:
        #msg.data = np.array([frame[:,0,0]])

        #msg.data = np.array([frame]).reshape(1920*1080*3)
        #msg.height = len(frame)
        #msg.width = len(frame[1])
        #msg.step = 1920*3
        #print msg.data


        pub.publish(msg)

if __name__ == '__main__':
    stream(sys.argv)
