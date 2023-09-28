#!/usr/bin/python2.7
import math
from math import sin, cos, pi

import rospy
import tf2_ros
import tf_conversions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped

class VelocityUtils(object):
    def __init__(self):
        self.rospy = rospy
        self.init_params()
        if self.publish_vel_raw: self.init_publishers()

    def init_params(self):
        self.skidsteer = self.rospy.get_param("traction_general/skidsteer", False)
        self.publish_vel_raw = self.rospy.get_param("traction_general/pub_vel_raw", False)
        self.current_vel_topic = self.rospy.get_param("traction_general/vel_topic", "/vel_raw")
        self.wheels_separation = self.rospy.get_param("traction_general/wheels_separation", 0.4) # TODO: Check!
        self.wheels_separation_param = self.wheels_separation / 2
        # We're not assuming both wheels are equal, though they should be (and likelly are)
        self.left_wheel_radius = self.rospy.get_param("traction_general/left_wheel_radius", 0.15/2) # TODO: Check! # In meters
        self.right_wheel_radius = self.rospy.get_param("traction_general/right_wheel_radius", 0.15/2) # TODO: Check! # In meters
        self.rpm_to_ms_const  = self.left_wheel_radius * 2 * 3.1415 / 60 
        # self.coef_alfa = 0.96851
        # self.coef_Ycir = 0.34242
        self.coef_alfa = 0.85
        self.coef_Ycir = 0.279

        return
    # In case we want to publish the raw velocity
    def init_publishers(self):
        self.solver_vel = self.rospy.Publisher(self.current_vel_topic, Twist, queue_size = 100)    
        self.solver_vel_msg = Twist()
        return
    
    def rpm_to_meterspersecond_left(self,vel_in_rpm):
        vel_in_ms = self.rpm_to_ms_const * vel_in_rpm
        return vel_in_ms
    
    def rpm_to_meterspersecond_right(self,vel_in_rpm):
        vel_in_ms = self.rpm_to_ms_const * vel_in_rpm
        return vel_in_ms

        
    def meterspersecond_to_rpm(self,vel_in_ms):
        vel_in_rpm = vel_in_ms/self.rpm_to_ms_const 
        return vel_in_rpm


    def wheels_to_solver(self,left_wheel_vel,right_wheel_vel):
        # From left and right wheels velocities (m/s)
        # To solver's linear (m/s) and angular (rad/s) velocities
        if (self.skidsteer):
            # print("estou utilizando o modelo skidsteer")        
            solver_linear_vel = ((left_wheel_vel + right_wheel_vel)) / 2
            solver_angular_vel = (right_wheel_vel - left_wheel_vel)*self.coef_alfa / (2*self.coef_Ycir)
        else:
            solver_linear_vel = (left_wheel_vel + right_wheel_vel) / 2
            solver_angular_vel = (right_wheel_vel - left_wheel_vel) / self.wheels_separation 
        return solver_linear_vel, solver_angular_vel

    # def wheels_to_solver_skidsteer(self,left_wheel_vel,right_wheel_vel): # criado por Miguel Saldanha em 07/07/2021
    #     # From left and right wheels velocities (m/s)
    #     # To solver's linear (m/s) and angular (rad/s) velocities
    #     # modelo cinematico para robos do tipo skidsteer
    #     solver_linear_vel = (self.coef_alfa*left_wheel_vel + self.coef_alfa*right_wheel_vel) / 2
    #     solver_angular_vel = (right_wheel_vel - left_wheel_vel)*self.coef_alfa / (2*self.coef_Ycir)
    #     return solver_linear_vel, solver_angular_vel

    def solver_to_wheels(self,lin,ang):
        # From solver's linear (m/s) and angular (rad/s) velocities
        # To left and right wheels velocities (m/s)
        left_wheel_vel = lin - (ang*self.wheels_separation/2) # we're dividing by two on the init as a minor optimization
        right_wheel_vel = lin + (ang * self.wheels_separation/2) # we're dividing by two on the init as a minor optimization
        return left_wheel_vel, right_wheel_vel

    # def get_solver_vel_skidsteer(self,left_wheel_vel_sub,right_wheel_vel_sub):
    #     # We have the velocity of each wheel in rpm
    #     # Lets put those in m/s
    #     left_wheel_vel_in_ms = self.rpm_to_meterspersecond_left(left_wheel_vel_sub.data)
    #     right_wheel_vel_in_ms = self.rpm_to_meterspersecond_right(right_wheel_vel_sub.data)

    #     # Get the velocities of the solver frame
    #     if self.skidsteer: # usar modelo cinematico de robo movel do tipo skidsteer
    #         solver_linear_vel, solver_angular_vel = self.wheels_to_solver_skidsteer(left_wheel_vel_in_ms,right_wheel_vel_in_ms)
    #         print("estou utilizando o modelo skidsteer")

    #     else: # usar modelo cinematico de robo movel com tracao diferencial
    #         solver_linear_vel, solver_angular_vel = self.wheels_to_solver(left_wheel_vel_in_ms,right_wheel_vel_in_ms)
    #     return solver_linear_vel, solver_angular_vel

    def get_solver_vel(self,left_wheel_vel,right_wheel_vel):
        # We have the velocity of each wheel in rpm
        # Lets put those in m/s
        left_wheel_vel_in_ms = self.rpm_to_meterspersecond_left(left_wheel_vel)
        right_wheel_vel_in_ms = self.rpm_to_meterspersecond_right(right_wheel_vel)
        # Get the velocities of the solver frame
        solver_linear_vel, solver_angular_vel = self.wheels_to_solver(left_wheel_vel_in_ms,right_wheel_vel_in_ms)
        return solver_linear_vel, solver_angular_vel
    
    # In case we want to publish the raw velocity
    def publish_velocity(self,solver_linear_vel,solver_angular_vel):
        self.solver_vel_msg.linear.x = solver_linear_vel
        self.solver_vel_msg.angular.z = solver_angular_vel
        self.solver_vel.publish(self.solver_vel_msg)
