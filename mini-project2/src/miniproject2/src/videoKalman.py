import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image

vid = cv2.VideoCapture("../2017_06_23_dyrskuepladsen.mp4")
fgbg = cv2.createBackgroundSubtractorMOG2()


while(True):
    ret, frame = vid.read()
    fgmask = fgbg.apply(frame)
    cv2.imshow('frame',fgmask)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break




cap.release()
cv2.destroyAllWindows()
