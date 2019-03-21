import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image

def stream():
    pub = rospy.Publisher("vid",Image,queue_size=10)
    rospy.init_node('talker')
    rate = rospy.Rate(10)
    vid = cv2.VideoCapture("../2017_06_23_dyrskuepladsen.mp4")
    msg = Image()
    while not rospy.is_shutdown():
        ret, frame = vid.read()
        #print np.array([frame[:,0,0]]) , "  ", ret
        #if ret > 0:
        #msg.data = np.array([frame[:,0,0]])
        msg.data = np.array([frame]).reshape(1920*1080*3)
        msg.height = len(frame)
        msg.width = len(frame[1])
        msg.step = 1920*3
        print msg.data



        pub.publish(msg)

if __name__ == '__main__':
    stream()