#!/usr/bin/env python3

import rospy
from adc_raw.msg import Mcp3208_data
from sensor_msgs.msg import ChannelFloat32

class laser_raw():
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
        self.subscribe_topic = self.rospy.get_param("laser/subscribe_topic","/adc_data")        
        self.publish_topic1 = self.rospy.get_param("laser/publish_topic1","/laser_front")
        self.publish_topic2 = self.rospy.get_param("laser/publish_topic2","/laser_back")
        self.acquisition_rate = self.rospy.get_param("laser/acquisition_rate",10)
        return

    def initVariables(self):
        self.rate = self.rospy.Rate(self.acquisition_rate)
        self.adcMessage = Mcp3208_data()
        self.laserMessage1 = ChannelFloat32()
        self.laserMessage2 = ChannelFloat32()
        self.laserMessage1.name = "Laser Frontal"
        self.laserMessage2.name = "Laser Traseiro"
        self.flag = False
        return

    def initSubscribers(self):
        self.sub_adcRaw =  self.rospy.Subscriber(self.subscribe_topic,Mcp3208_data, self.adc_callback)      
        return

    def initPublishers(self):
        self.pub_laserRaw1 = self.rospy.Publisher(self.publish_topic1,ChannelFloat32,queue_size=10)
        self.pub_laserRaw2 = self.rospy.Publisher(self.publish_topic2,ChannelFloat32,queue_size=10)
        return

    def adcConverter(self,adc_value):
        self.adc_value = adc_value

        if(self.adc_value > 3150):
            distanciaConv = self.adc_value/(-233.33) + 22.5 +2
        
        elif(self.adc_value > 2020):
            distanciaConv = self.adc_value/(-183.33) + 22.725 +2
        
        elif(self.adc_value > 1340):
            distanciaConv = self.adc_value/(-68) + 44.706 + 2
        
        elif(self.adc_value > 910):
            distanciaConv = self.adc_value/(-28.66) + 71.74
        
        elif(self.adc_value > 640):
            distanciaConv = self.adc_value/(-13.5) + 107.407
        else:
            distanciaConv = self.adc_value/(-8.75) + 133.14
        return distanciaConv

    def adc_callback(self,data):
        self.adcMessage = data
        distancia0 = self.adcConverter(self.adcMessage.channel_0)
        distancia1 = self.adcConverter(self.adcMessage.channel_1)
        distancia2 = self.adcConverter(self.adcMessage.channel_2)
        distancia3 = self.adcConverter(self.adcMessage.channel_3)
        self.laserMessage1.values = [distancia0,distancia1]
        self.laserMessage2.values = [distancia2,distancia3] 
        self.flag = True
        return

    def main(self):
        while not self.rospy.is_shutdown():
            if(self.flag):
                frontais = self.laserMessage1.values
                traseiros = self.laserMessage2.values
                dist1 = round(frontais[0])
                dist2 = round(frontais[1])
                dist3 = round(traseiros[0])
                dist4 = round(traseiros[1])
                self.laserMessage1.values = [dist1,dist2]
                self.laserMessage2.values = [dist3,dist4] 
                self.pub_laserRaw1.publish(self.laserMessage1)
                self.pub_laserRaw2.publish(self.laserMessage2)
            self.rate.sleep()

if __name__ == '__main__':
	try:
		laserRaw = laser_raw('LASER_RAW_DATA_NODE')
	except rospy.ROSInterruptException:
		pass
