#!/usr/bin/env python
import numpy as np
import rospy
import cv2
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from mav_msgs.msg import Actuators
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_rpy_orientation (q_orientation_q):
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return [roll, pitch, yaw]

class QuadStabilityNode:
    def __init__(self):
        rospy.init_node('uav_quad_stability')
        self.current_uav_setpoint = Pose()
        self.current_uav_setpoint.position.z = 2.0 # starting setpoint
        self.current_uav_pose = Pose()
        self.alt_effort = 0.0

        self.setpoint_sub = rospy.Subscriber("/uav_setpoint", Pose, self.set_uav_setpoint)
        self.uavpose_sub = rospy.Subscriber("/hummingbird/odometry_sensor1/pose", Pose, self.set_uav_pose)
        self.alt_effort_sub = rospy.Subscriber("/pid_controllers/altitude/control_effort", Float64, self.set_alt_effort)

        self.altitude_setpoint_pub = rospy.Publisher("/pid_controllers/altitude/setpoint", Float64, queue_size=1)
        self.altitude_state_pub = rospy.Publisher("/pid_controllers/altitude/state", Float64, queue_size=1)
        self.motor_speed_pub = rospy.Publisher("/hummingbird/command/motor_speed", Actuators, queue_size=1)


        rospy.Timer(rospy.Duration(1./100.), self.timer_callback)
        rospy.spin()


    def set_uav_setpoint(self, msg):
        self.current_uav_setpoint = msg

    def set_uav_pose(self, msg):
        self.current_uav_pose = msg


    def set_alt_effort(self, msg):
        self.alt_effort = msg.data

    def timer_callback(self, event):
        self.altitude_setpoint_pub.publish(self.current_uav_setpoint.position.z)
        self.altitude_state_pub.publish(self.current_uav_pose.position.z)

        ac = Actuators()
        alt_ref = 490
        output = alt_ref #+self.alt_effort
        ac.angular_velocities = [output,output,output,output]
        self.motor_speed_pub.publish(ac)

if __name__ == "__main__":
    node = QuadStabilityNode()
