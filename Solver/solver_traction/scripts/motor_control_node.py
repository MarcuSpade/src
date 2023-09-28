#!/usr/bin/python3
import rospy
from simple_pid import PID
from solver_traction.velocity_util import VelocityUtils

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import ChannelFloat32


class TractionMotorControl(object):

    def __init__(self, control_rate):    
        self.control_rate = control_rate
        self.rospy = rospy
        # Start the i2c bus

        # Parameter initialization
        self.initParameters()

        self.initSubscribers()
        self.initPublishers()
        
        # Controllers initialization
        if(self.controller_flag):
            self.initPIDs()
        self.vel_utils  = VelocityUtils()

    def __del__(self):
        self.stop_motors()
        # if self.debug == True:
        #     self.left_control_signal.publish(Float64(data=self.left_wheel_signal_volts))
        #     self.right_control_signal.publish(Float64(data=self.right_wheel_signal_volts))

    def initParameters(self):
        self.debug = self.rospy.get_param("traction_control/debug", True)

        self.control_rate = self.rospy.get_param("traction_control/control_rate", 100)
        self.controller_flag = self.rospy.get_param("traction_control/is_active", False)
        self.rate = self.rospy.Rate(self.control_rate)
        self.voltageCommand = ChannelFloat32()
        
        '''
        I'm following the given convention for ROS topic names:
        cmd_vel: the actual velocity command (geometry_msgs.Twist)
        aux_cmd_vel: an aux velocity command that may not be suitable for the actual output (geometry_msgs.Twist)
        odom: actual odometry readings which - ideally - consider multiple sensors (encoders and IMU, at least) (nav_msgs.Odometry)
        odom_raw: odometry estimated from encoders' readings (nav_msgs.Odometry)
        vel_raw: robot velocity as given by the encoders (ideally, should never be used; use odom instead) (geometry_msgs.Twist) 
        '''
        self.cmd_vel_topic = self.rospy.get_param("traction_general/cmd_vel_topic", "/cmd_vel")
        self.odom_topic = self.rospy.get_param("traction_general/odom_topic", "/odom")
        # PID parameters for each wheel
        time_sample = 1.0/self.control_rate        
        self.controller_params_left = {
            "kp": self.rospy.get_param("traction_control/pid_params_left/P",10),
            "ki": self.rospy.get_param("traction_control/pid_params_left/I",0),
            "kd": self.rospy.get_param("traction_control/pid_params_left/D",0),
            "Ts": self.rospy.get_param("traction_control/pid_params_left/Ts",time_sample)}
        self.controller_params_right = {
            "kp": self.rospy.get_param("traction_control/pid_params_right/P",10),
            "ki": self.rospy.get_param("traction_control/pid_params_right/I",0),
            "kd": self.rospy.get_param("traction_control/pid_params_right/D",0),
            "Ts": self.rospy.get_param("traction_control/pid_params_right/Ts",time_sample)}
        
        self.reference_linear_vel = 0.0
        self.reference_angular_vel = 0.0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        return

    def initSubscribers(self):
        # Subscribe to a current vel topic and a comand vel topic
        if(self.controller_flag== True):
            self.sub_odom = self.rospy.Subscriber(self.odom_topic,Odometry,self.callback_odom)    
        self.sub_cmd_vel = self.rospy.Subscriber(self.cmd_vel_topic,Twist,self.callback_high_level_control)
        return

    def initPublishers(self):
        # self.left_control_signal = self.rospy.Publisher("left_control_signal", Float64, queue_size = 10)  
        # self.right_control_signal = self.rospy.Publisher("right_control_signal", Float64, queue_size = 10)
        # self.ref_left_wheel_ms = self.rospy.Publisher("ref_left_wheel_ms", Float64, queue_size = 10)  
        # self.ref_right_wheel_ms = self.rospy.Publisher("ref_right_wheel_ms", Float64, queue_size = 10)
        self.md25_command_publisher = self.rospy.Publisher("/md49_driverCommand",ChannelFloat32,queue_size=10)
        return

    def initPIDs(self):
        # PID initiallization
        # Sample time equal to the control loop period
        # Output limits to keep our control signal away from the driver's +-24V boundaries (keeping it safe)
        self.pid_left  = PID(self.controller_params_left["kp"], self.controller_params_left["ki"], self.controller_params_left["kd"], setpoint=0,sample_time=self.controller_params_left["Ts"],output_limits=(-24.0, 24.0))
        self.pid_right = PID(self.controller_params_right["kp"], self.controller_params_right["ki"], self.controller_params_right["kd"], setpoint=0,sample_time=self.controller_params_right["Ts"],output_limits=(-24.0, 24.0))
        return

    def callback_high_level_control(self,msg):
        # From high level controller
        # Extract walker's reference linear and angular velocities
        self.reference_linear_vel = msg.linear.x
        self.reference_angular_vel = msg.angular.z
        return

    def callback_odom(self,msg):
        # From odometry
        # Extract walker's current linear and angular velocities
        self.current_linear_vel = msg.twist.twist.linear.x
        self.current_angular_vel = msg.twist.twist.angular.z
        return

    def pid_controller_left(self, reference_vel_wheel, current_vel_wheel):
        self.pid_left.setpoint = reference_vel_wheel
        # print("current vel: {}".format(current_vel_wheel))
        control_signal = self.pid_left(current_vel_wheel)
        return control_signal

    def pid_controller_right(self, reference_vel_wheel, current_vel_wheel):
        self.pid_right.setpoint = reference_vel_wheel
        control_signal = self.pid_right(current_vel_wheel)
        return control_signal

    # Main loop
    def update_controller(self):
        # Get velocities for each wheels
        reference_left_wheel_vel, reference_right_wheel_vel = self.vel_utils.solver_to_wheels(self.reference_linear_vel, self.reference_angular_vel)
        if (self.controller_flag):
            current_left_wheel_vel, current_right_wheel_vel = self.vel_utils.solver_to_wheels(self.current_linear_vel, self.current_angular_vel)
        # Call PID (returned control signal in volts)
            self.left_wheel_signal_volts = self.pid_controller_left(reference_left_wheel_vel, current_left_wheel_vel)
            self.right_wheel_signal_volts = self.pid_controller_right(reference_right_wheel_vel, current_right_wheel_vel)
        else:
            self.left_wheel_rpm = self.vel_utils.meterspersecond_to_rpm(reference_left_wheel_vel)
            self.right_wheel_rpm = self.vel_utils.meterspersecond_to_rpm(reference_right_wheel_vel)

            self.left_wheel_signal_volts = self.rpm_to_voltage(self.left_wheel_rpm)
            self.right_wheel_signal_volts = self.rpm_to_voltage(self.right_wheel_rpm)
        # print("left signal: {} right signal: {}".format(self.left_wheel_signal_volts, self.right_wheel_signal_volts))

    def rpm_to_voltage(self,rpm_vel):
        return (rpm_vel *0.19672131147540983)

    def send_command(self):
        self.md25_command_publisher.publish(values=[self.left_wheel_signal_volts,self.right_wheel_signal_volts])
        return True
    def stop_motors(self):
        self.md25_command_publisher.publish(values=[0.0,0.0])

def main():
    rospy.init_node('motor_control_node', anonymous=True)
    rospy.loginfo("Starting Motor Control Node")

    rate_param = rospy.get_param("traction_control/control_rate", 100)
    rate = rospy.Rate(rate_param)
    
    traction_control = TractionMotorControl(rate_param)
    if (traction_control.controller_flag):
        rospy.loginfo("[Motor Control] Entering control loop...")
    else: 
        rospy.loginfo("[Motor Control] No Control loop...")
    while not rospy.is_shutdown():
        traction_control.update_controller()
        traction_control.send_command()
        rate.sleep()

    traction_control.stop_motors()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
