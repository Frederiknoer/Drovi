#!/usr/bin/env python
import numpy as np
import rospy
import cv2
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu, FluidPressure
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from mav_msgs.msg import RollPitchYawrateThrust
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def get_rpy_orientation (q_orientation_q):
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return [roll, pitch, yaw]


class QuadStabilityNode:
    def __init__(self):
        rospy.init_node('uav_quad_stability')
        self.current_uav_setpoint = Pose()
        self.current_uav_setpoint.position.z = 0.0 # starting setpoint for height
        self.current_uav_setpoint.orientation.y = 0.0
        self.current_uav_setpoint.orientation.x = 0.0
        self.current_uav_height = 0.0
        self.fluid_pressure_a = -0.0876
        self.fluid_pressure_b = 8362.3
        self.yaw_effort = 0.0
        self.alt_effort = 0.0
        self.alt_rate_effort = 0.0
        self.alt_est_speed = 0.0
        self.alt_height = 0.0
        self.yaw = 0.0

        self.sample_time = rospy.get_rostime().to_sec()

        # Define subscriptions
        self.setpoint_sub = rospy.Subscriber("/uav_setpoint", Pose, self.set_uav_setpoint)
        self.uavpose_sub = rospy.Subscriber("/hummingbird/imu", Imu, self.set_uav_pose)
        self.uavpres_sub = rospy.Subscriber("/hummingbird/air_pressure", FluidPressure, self.set_uav_pres)
        self.yaw_effort_sub = rospy.Subscriber("/pid_controllers/yaw/control_effort", Float64, self.set_yaw_effort)
        self.alt_effort_sub = rospy.Subscriber("/pid_controllers/altitude/control_effort", Float64, self.set_alt_effort)
        self.alt_rate_effort_sub = rospy.Subscriber("/pid_controllers/altitude_rate/control_effort", Float64, self.set_alt_rate_effort)

        # Define publishers
        self.yaw_state_pub = rospy.Publisher("/pid_controllers/yaw/state", Float64, queue_size=1)
        self.yaw_setpoint_pub = rospy.Publisher("/pid_controllers/yaw/setpoint", Float64, queue_size=1)
        self.altitude_state_pub = rospy.Publisher("/pid_controllers/altitude/state", Float64, queue_size=1)
        self.altitude_setpoint_pub = rospy.Publisher("/pid_controllers/altitude/setpoint", Float64, queue_size=1)
        self.altitude_rate_state_pub = rospy.Publisher("/pid_controllers/altitude_rate/state", Float64, queue_size=1)
        self.altitude_rate_setpoint_pub = rospy.Publisher("/pid_controllers/altitude_rate/setpoint", Float64, queue_size=1)
        self.rpyt_pub = rospy.Publisher("/hummingbird/command/roll_pitch_yawrate_thrust", RollPitchYawrateThrust, queue_size=1)

        # Setup the callback timer, aka. 'main' function
        rospy.Timer(rospy.Duration(1./100.), self.timer_callback)
        rospy.spin()



    def set_uav_setpoint(self, msg):
        '''
        Writes the recieved input to a variable.
        '''
        print(msg)
        self.current_uav_setpoint = msg


    def set_uav_pose(self, msg):
        '''
        Writes the recieved height to a variable, and calculates and saves the speed to another variable.
        '''
        self.yaw = msg.orientation.z


    def set_uav_pres(self, msg):
        '''
        Writes the recieved height to a variable, and calculates and saves the speed to another variable.
        '''
        now = rospy.get_rostime().to_sec()
        if((now-self.sample_time)>0.05):
            T = now-self.sample_time
            self.alt_est_speed = ((self.fluid_pressure_a * msg.fluid_pressure + self.fluid_pressure_b) - self.current_uav_height) / T
            self.sample_time = now
            self.current_uav_height = self.fluid_pressure_a * msg.fluid_pressure + self.fluid_pressure_b
        self.alt_height = self.fluid_pressure_a * msg.fluid_pressure + self.fluid_pressure_b



    def set_yaw_effort(self, msg):
        '''
        Writes the recieved output of the yaw PID to a variable.
        '''
        self.yaw_effort = msg.data


    def set_alt_effort(self, msg):
        '''
        Writes the recieved output of the altitude PID to a variable.
        '''
        self.alt_effort = msg.data


    def set_alt_rate_effort(self, msg):
        '''
        Writes the recieved output of the altitude-rate PID to a variable.
        '''
        self.alt_rate_effort = msg.data


    def timer_callback(self, event):
        '''
        Pass the variables to the relevant publishers.self

        yaw PID:
            input.orientation.z -> setpoint
            measured_yaw -> state

        Altitude PID:
            input.position.z (how high we want to go) -> setpoint
            measured_height -> state

        Altitude-rate PID:
            altitude_PID_output -> setpoint
            estimated_speed -> state

        RollPitchYawrateThrust (input to UAV):
            input.pitch -> pitch
            input.roll -> roll
            yaw_PID_output -> yaw-rate
            altitude_rate_PID_output -> thrust
        '''

        print(self.alt_height)
        # Pass wished yaw position (must be between -1 and 1) and measured yaw
        self.yaw_setpoint_pub.publish(self.current_uav_setpoint.orientation.z)
        self.yaw_state_pub.publish(self.yaw)

        # Pass wished altitude and measured altitude
        self.altitude_setpoint_pub.publish(self.current_uav_setpoint.position.z)
        self.altitude_state_pub.publish(self.alt_height)

        # Pass wished alt-rate and estimated speed
        self.altitude_rate_setpoint_pub.publish(self.alt_effort)
        #self.altitude_rate_setpoint_pub.publish(self.current_uav_setpoint.position.z)
        self.altitude_rate_state_pub.publish(self.alt_est_speed)

        rpyt = RollPitchYawrateThrust()
        # Define standard thrust, this should be roughly when the drone is standing still in the air.
        thrust_ref = 7.125
        output = thrust_ref+self.alt_rate_effort
        # Pass thrust to uav
        rpyt.thrust.z = thrust_ref+self.alt_rate_effort
        # Pass pitch to uav
        rpyt.pitch = self.current_uav_setpoint.orientation.y
        # Pass roll to uav
        rpyt.roll = self.current_uav_setpoint.orientation.x
        # Pass yaw-rate to uav
        rpyt.yaw_rate = self.yaw_effort
        self.rpyt_pub.publish(rpyt)


if __name__ == "__main__":
    node = QuadStabilityNode()
