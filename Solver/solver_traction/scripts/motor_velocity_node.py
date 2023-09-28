#!/usr/bin/python3
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from md49_messages.msg import md49_encoders
import time
import sys
import os
"""Global variables"""
joints = ["left_wheel","right_wheel"]
class MotorVel(object):
    def __init__(self,joint_name):
        self.joint_name = joint_name
        self.enc_status = 0
        self.cpr =  235.0#588.0
        self.ppr = self.cpr*4                             # Encoder resolution. 1 CPR = 4 PPR
        self.ratio = 1.0                             # Motor transmission
        self.window_size = 5.0                         # Running mean filter - window size
        self.vel_array = [0] * int(self.window_size)   # Running mean filter - past velocities array (sizeof(self.window_size))
        self.meas_time = time.time()                   # Current measurement time
        """ROS initialization"""
        self.init_pubs_()
        self.init_subs_()
    
    def init_pubs_(self):
        if (self.joint_name=="left_wheel"):
            self.vel_pub = rospy.Publisher('left_wheel_vel', Float64, queue_size=100)
        else:
            self.vel_pub = rospy.Publisher('right_wheel_vel', Float64, queue_size=100)

    
    def init_subs_(self):
        rospy.Subscriber("/md49_encoders", md49_encoders, self.encoders_callback)

    def encoders_callback(self,msg):
        if(self.joint_name=="left_wheel"):
            current_vel = self.enc2vel(msg.encoder_l)
            filtered_vel = self.running_mean(current_vel)
        else:
            current_vel = self.enc2vel(msg.encoder_r)
            filtered_vel = self.running_mean(current_vel)
        '''Publish motor velocity after converting encoder reading'''
        self.vel_pub.publish(filtered_vel)

    """ Converts from encoder reading to motor velocity.
        The characteristic funtion depends on the encoder resolution and the motor transmission."""
    def enc2vel(self, enc_count):
        delta_time = time.time() - self.meas_time
        delta_count = enc_count - self.enc_status
        wheel_vel = ((delta_count / self.ppr) * 1.0/self.ratio) / (delta_time / 60.0)
        # print(" {} [rpm]: {}  delta_time: {} delta_count: {}".format(self.joint_name,wheel_vel,delta_time,delta_count))
        self.enc_status = enc_count
        self.meas_time = time.time()
        return wheel_vel

    ''' Running mean filter to smooth velocity readings '''
    def running_mean(self,current_vel):
        self.vel_array.pop()
        self.vel_array.insert(0,current_vel)
        return sum(self.vel_array) / self.window_size

def main():
    """Module initialization"""
    rospy.init_node('wheels_vel_node', anonymous=True)       
    for joint_name in joints:
        print(joint_name)
        exec("{} = MotorVel('{}')".format(joint_name, joint_name))

    rospy.logwarn("Processing sensor data...")
    while not rospy.is_shutdown():
        rospy.spin()



if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt as e:
        print("Program finished\n")
        sys.stdout.close()
        os.system('clear')
        raise
