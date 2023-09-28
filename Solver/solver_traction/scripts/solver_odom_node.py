#!/usr/bin/python3
import rospy
from solver_traction.odometry import SolverOdometry
from solver_traction.velocity_util import VelocityUtils
from md49_controller.msg import MD49_encoders
from std_msgs.msg import Float64

'''
This node compiles the velocity readings from each wheel (in rpm) into 
velocities of the walker frame (or base frame) to publish the odometry
'''
class SolverrOdomNode(object):
    def __init__(self):
        # ROS initialization
        self.rospy = rospy
        self.rospy.init_node("Solverr_odom_node", anonymous = True)
        self.rospy.loginfo("Starting Solver Odom Node")

        # Initialize odometry
        self.odom = SolverOdometry()
        # Initialize vel util
        self.vel_util = VelocityUtils()

        # Parameter initialization
        self.initParameters()
        self.initSubscribers()

        # Main loop
        self.main_loop()

    def initParameters(self):
        self.encoders_topic = "/md49_encoders"
    def initSubscribers(self):
        self.sub_encoders=self.rospy.Subscriber(self.encoders_topic,MD49_encoders,self.encoders_vel_callback)
        return 
    # We're using message_filters to sync messages from different topics
    # The callback process pairs of messages that arrived at approximately the same time     
    def encoders_vel_callback(self,msg):
        # Messages are sync'ed, OK
        # We want the robot velocity, not each wheels' separately
        left_wheel_vel=msg.raw_speed_l
        right_wheel_vel=msg.raw_speed_r
        solver_linear_vel,solver_angular_vel = self.vel_util.get_solver_vel(left_wheel_vel,right_wheel_vel)
        
        #############
        #############
        # Given the current velocities, update and publish odometry
        self.odom.update_velocity(solver_linear_vel,solver_angular_vel)
        self.odom.update_odometry()
        self.odom.publish_odometry()

    # Main loop
    def main_loop(self):
        while not self.rospy.is_shutdown():
            # rate.sleep()
            rospy.spin()


if __name__ == '__main__':
    try:
        wv = SolverrOdomNode()
    except rospy.ROSInterruptException:
        pass
