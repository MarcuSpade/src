#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import ChannelFloat32
from std_msgs.msg import Float32
import math


class laser_control():
    def __init__(self,name):
        self.name=name
        self.rospy=rospy
        self.rospy.init_node(self.name,anonymous=True)
        self.rospy.loginfo("[%s] Starting Node ",self.name)
        self.initParams()
        self.initVariables()
        self.initSubscribers()
        self.initPublishers()
        self.main()
    
    def initParams(self):
        self.subscribe_topic1 = self.rospy.get_param("control/subscribe_topic1","/laser_front")
        self.subscribe_topic2 = self.rospy.get_param("control/subscribe_topic2","/laser_back")
        self.subscribe_topic3 = self.rospy.get_param("control/subscribe_topic3","/solver_traction/cmd_vel")   
        self.publish_topic = self.rospy.get_param("control/publish_topic","/cmd_vel")
        self.publish_led = self.rospy.get_param("control/publish_led","/led_frequency")
        self.acquisition_rate = self.rospy.get_param("laser/acquisition_rate",10)
        return

    def initVariables(self):
        self.rate = self.rospy.Rate(self.acquisition_rate)
        self.laserMessage1 = ChannelFloat32()
        self.laserMessage2 = ChannelFloat32()
        self.speedIn = Twist()
        self.speedOut = Twist()
        self.speedAux = Twist()
        self.speedAux.linear.x = 0
        self.speedAux.angular.z = 0
        self.led_frequency = 0.0
        self.change_front=False
        self.change_back=False
        self.turn_back = False
        self.vel_flag = False
        self.stop_frontal = 90
        # self.linear_value=[0.0,0.0]
        return

    def initSubscribers(self):
        self.sub_laserFront =  self.rospy.Subscriber(self.subscribe_topic1,ChannelFloat32, self.front_callback) 
        self.sub_laserBack =  self.rospy.Subscriber(self.subscribe_topic2,ChannelFloat32, self.back_callback)    
        self.sub_vel =  self.rospy.Subscriber(self.subscribe_topic3,Twist, self.vel_callback)   
        return

    def initPublishers(self):
        self.pub_vel = self.rospy.Publisher(self.publish_topic,Twist,queue_size=10)
        self.pub_led = self.rospy.Publisher(self.publish_led,Float32,queue_size=10)

        return

    def front_callback(self,data1):
        self.laserMessage1 = data1
        self.change_front=True
        return

    def back_callback(self,data2):
        self.laserMessage2 = data2
        self.change_back=True
        return

    def vel_callback(self,data):
        self.speedIn = data
        return

    def linearControl(self,linear_in):
        self.linear_value = linear_in
        frontais = self.laserMessage1.values
        traseiros = self.laserMessage2.values

        if (math.fabs(self.linear_value) > 0.55):
            self.stop_frontal = 70
        elif (math.fabs(self.linear_value) > 0.35):
            self.stop_frontal = 60
        else:
            self.stop_frontal = 30

        # Condicao para que o robo pare caso identifique algum obstaculo
        if (self.linear_value > 0.0 and (frontais[0] < self.stop_frontal)):
            linear = 0.0
        elif (self.linear_value < 0.0 and (traseiros[0] < self.stop_frontal)):  
            linear = 0.0
        else:
            linear = self.linear_value

        # Caso seja identicado um vao a frente, ele inverte o sentido de giro e altera a FLAG turn_back
        if (frontais[1] > 20.0 or traseiros[1] > 20.0):
            linear = -self.linear_value
            self.turn_back = True

        # Ao identificar que saiu do buraco, ele compara com a FLAG para ver se esta girando no sentido contrario para que o robo pare.
        if (frontais[1] < 20.0 and traseiros[1] < 20.0 and self.turn_back == True):
            self.linear_value = 0
            linear = self.linear_value
            self.turn_back = False
            self.speedIn.linear.x = 0

        # A freqûencia de acionamento dos lasers e alterada conforme identifica obstaculos
        if (linear != 0 and frontais[0] > 60.0 and traseiros[0] > 60): # Obstaculo acima de 60 cm
            self.led_frequency = 1.0
        elif (linear != 0 and ( frontais[0] > 30.0 or traseiros[0] > 30 )): # Obstaculo acima de 30 cm
            self.led_frequency = 0.5
        elif (linear != self.linear_value): # Obstaculo abaixo de 30 cm
            self.led_frequency = 0.25
        elif (self.speedIn.linear.x == 0 and self.speedIn.angular.z == 0): # Robô parado
            self.led_frequency = 0.0
        return linear

    def angularControl(self,angular_in):
        self.angular_value = angular_in
        frontais1 = self.laserMessage1.values
        traseiros1 = self.laserMessage2.values
        if (self.angular_value > 0.0 and (frontais1[0] <30.0  or  traseiros1[0] <30.0  )):
            angular = 0.0
        elif (self.angular_value < 0.0 and (traseiros1[0] <30.0   or frontais1[0] <30.0  )):  
            angular = 0.0
        else:
            angular = self.angular_value
        if (angular != 0):
            self.led_frequency = 1.0
        return angular

    def main(self):
        while not self.rospy.is_shutdown():
            if(self.change_back and self.change_front):
                speed_angular_z = self.angularControl(self.speedIn.angular.z)                
                speed_linear_x = self.linearControl(self.speedIn.linear.x)
                self.speedOut.linear.x = speed_linear_x
                self.speedOut.angular.z = speed_angular_z
                if((self.speedOut.linear.x != self.speedAux.linear.x)or(self.speedOut.angular.z != self.speedAux.angular.z)):
                    self.pub_vel.publish(self.speedOut)
                    self.speedAux.linear.x = self.speedOut.linear.x
                    self.speedAux.angular.z = self.speedOut.angular.z                   
                self.pub_led.publish(self.led_frequency)
            self.rate.sleep()

if __name__ == '__main__':
	try:
		laserControl = laser_control('LASER_CONTROL_DATA_NODE')
	except rospy.ROSInterruptException:
		pass
