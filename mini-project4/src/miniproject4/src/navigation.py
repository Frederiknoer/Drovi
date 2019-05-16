#!/usr/bin/python
import rospy
import cv2
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
import sys


class navigation:
    def __init__(self):
        rospy.init_node("Navigation",anonymous=True)

        self.search_mode = 0
        self.marker_tracking_mode = 1
        self.landing_mode = 2
        self.mode = self.search_mode

        self.current_position = Pose()
        self.marker_position = Pose()

        self.pose = rospy.Subscriber("/hummingbird/ground_truth/pose", Pose, self.steer_towards)
        self.steer = rospy.Publisher("/uav_setpoint", Pose, queue_size=1)

    def steer_towards(self, data):
        if self.mode == self.search_mode:
            starting_position = Pose()
            starting_position.position.z = 30
            print(starting_position)
            self.steer.publish(starting_position)
            print("Sent")
            self.mode = self.marker_tracking_mode
        elif self.mode == self.marker_tracking_mode:
            pass
        else:
            pass



if __name__ == '__main__':
    print("Navigation node started")
    mark = navigation()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
