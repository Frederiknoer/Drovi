#!/usr/bin/python
import rospy
import cv2

import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float64, Int8
from cv_bridge import CvBridge
import random
import message_filters
import math
import sys
import MarkerPose
import MarkerTracker

from miniproject4.msg import markerpose

#sys.path.insert(0, './fiducial/nfoldedge/')
#from MarkerLocator import MarkerPose, MarkerTracker
#from miniproject4.MarkerLocator import MarkerPose, MarkerTracker

def get_yaw_matrix(yaw_angle):
    return np.array([[np.cos(yaw_angle), np.sin(yaw_angle), 0],
                     [-np.sin(yaw_angle), np.cos(yaw_angle), 0],
                     [0, 0, 1]])


def get_pitch_matrix(pitch_angle):
    return np.array([[1, 0, 0],
                     [0, np.cos(pitch_angle), -np.sin(pitch_angle)],
                     [0, np.sin(pitch_angle), np.cos(pitch_angle)]])


def get_roll_matrix(roll_angle):
    return np.array([[np.cos(roll_angle), 0, np.sin(roll_angle)],
                     [0, 1, 0],
                     [-np.sin(roll_angle), 0, np.cos(roll_angle)]])

class markerDetection:
    def __init__(self):
        rospy.init_node("marker_detection",anonymous=True)

        self.k = np.array([[429.37045941975094, 0.0, 400.5], [0.0, 429.37045941975094, 300.5], [0.0, 0.0, 1.0]])
        armlength = 0.17
        self.camToDroneMat = np.array([[0.0, 0.0, -1, 0.0], [0.0, 1, 0.0, 0.0], [1, 0.0, 0.0, armlength/3],[0.0, 0.0, 0.0, 1]]) #homogeneous transformation
        self.k_inv = np.linalg.inv(self.k)
        #print self.k_inv

        #self.R_ose = np.matmul(get_yaw_matrix(0.0), np.matmul(get_pitch_matrix(-math.pi/2), get_roll_matrix(0.0)))    #not right
        #print self.R_ose
        self.rotationFromDrone = np.array([[0.0, 0.0, 1], [0.0, 1,  0.0], [-1, 0.0, 0.0]]) # rotation between camera and drone

        self.R_pose = self.rotationFromDrone.transpose()
        #print self.R_pose

        self.cam = np.array([[0.0, -1.0, 0.0], [-1.0,0.0,0.0],[0.0,0.0,1.0]])
        #self.R_pose = np.matmul(self.cam,self.R_pose)
        #print self.R_pose

        self.image_width = 800
        self.image_height = 600

        self.tracker = MarkerTracker.MarkerTracker(
            order=4,
            kernel_size=20,
            scale_factor=1)
        # Tell the tracker that we are looking for markers without
        # an unique orientation
        self.tracker.track_marker_with_missing_black_leg = True

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

        self.parameters = cv2.aruco.DetectorParameters_create()


        self.image_sub = rospy.Subscriber("hummingbird/camera_/image_raw", Image, self.callback)
        self.markerpose_pub = rospy.Publisher("nFold_markerPose", markerpose ,queue_size=10)
        self.image_pub = rospy.Publisher("nFold_markerimage", Image, queue_size=10)
        self.markerstatus_pub = rospy.Publisher("marker_status",Int8,queue_size=10)
        self.arucopose_pub = rospy.Publisher("aruco_markerPose", markerpose, queue_size=10)

    def camToDrone(self, X, Y, Z):
        arr = np.array([[X],[Y],[Z],[1]])
        return np.dot(self.camToDroneMat,arr)





    def callback(self,data):
        retstat = 0
        pos = 99

        frame = CvBridge().imgmsg_to_cv2(data, 'bgr8')

        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        pose = self.tracker.locate_marker(gray)


        #print(pose.quality)

        markerpose_msg = markerpose()
        markerpose_msg.header = data.header
        markerpose_msg.order = pose.order
        markerpose_msg.x = pose.x
        markerpose_msg.y = pose.y
        markerpose_msg.theta = pose.theta
        markerpose_msg.quality = pose.quality

        pos = self.determinePos(pose.x,pose.y)
        markerpose_msg.x = pos[0]
        markerpose_msg.y = pos[1]
        if markerpose_msg.quality > 0.5:
            cv2.circle(frame, (pose.x, pose.y), 20, (0, 0, 255), 2)
            orientation = min(abs(math.atan2(-(pose.x -self.image_width/2),pose.y-self.image_height/2)),  abs((math.pi - math.atan2(-(pose.x -self.image_width/2),pose.y-self.image_height/2))))
            #print math.degrees(orientation)
            print orientation
            self.markerpose_pub.publish(markerpose_msg)
            #self.markerstatus_pub.publish(1)
            if (orientation > 0.1):
                retstat = 1
            else:
                retstat = 2



        #elif markerpose_msg.quality > 0.2:
           # self.markerstatus_pub.publish()
        else:
            retstat = 0
            #self.markerstatus_pub.publish(0)

            #print "pose x and y", pose.x, pose.y+


        #self.markerpose_pub.publish(markerpose_msg)

        self.image_pub.publish(CvBridge().cv2_to_imgmsg(frame,'bgr8'))

        ##custom_dictionary = cv2.aruco.custom_dictionary(6, 4)
        #cv2.aruco.detectMarkers(frame)
        #markers = cv2.aruco.detectMarkers(gray, cv2.getPrede,parameters=parameters)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        #print corners, ids
        if any(x != None for x in ids):
            aruco = markerpose()
            aruco.header = data.header
            aruco.order = 0
            aruco.theta = 0
            aruco.quality = 1

            x = (corners[0][0][0] + corners[0][1][0]+corners[0][2][0] + corners[0][3][0])/4
            y = (corners[0][0][1] + corners[0][1][1] + corners[0][2][1] + corners[0][3][1]) / 4
            arucopos = self.determinePos(x, y)
            aruco.x = arucopos[0]
            aruco.y = arucopos[1]
            self.arucopose_pub.publish(aruco)



        #self.determineHeight(corners[0]);
        frame_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)

        #self.image_pub.publish(CvBridge().cv2_to_imgmsg(frame_markers, 'bgr8'))
        self.markerstatus_pub.publish(retstat)

    def determinePos(self, x, y):
        self.retArray = np.array([[x], [y], [1]])
        #print "retArray", self.retArray.transpose()
        xyz = np.matmul(self.k_inv , self.retArray)
        #print "XYZ", xyz.transpose()
        directions = np.matmul(self.R_pose, xyz)
        #print "directions", directions.transpose()
        self.relativeCoord = np.matmul(self.cam,xyz)
        #print 11 * directions
        #print self.relativeCoord
        #return math.sqrt(self.relativeCoord[0]**2 + self.relativeCoord[1]**2)
        return self.relativeCoord

        #diff = corners[0][0] - corners[0][3]
        #crosssection = math.sqrt((corners[0][0][0] - corners[0][3][0])**2 + (corners[0][0][1] - corners[0][3][1])**2)
        #print crosssection

        #position:
        #x: 2.7
        #y: 7.7
        #z: 1.6

#D: [0.0, 0.0, 0.0, 0.0, 0.0]
#K: [429.37045941975094, 0.0, 400.5, 0.0, 429.37045941975094, 300.5, 0.0, 0.0, 1.0]
#R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
#P: [429.37045941975094, 0.0, 400.5, -0.0, 0.0, 429.37045941975094, 300.5, 0.0, 0.0, 0.0, 1.0, 0.0]


if __name__ == '__main__':
    print "marker detection node started"
    mark = markerDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
