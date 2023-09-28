#!/usr/bin/env python3

import rospy
from mcp3208 import MCP3208 # spi bus 0, device 1
from adc_raw.msg import Mcp3208_data
#if you want to use spi_cs1, just change the value in the python library

class adc_raw():
    def __init__(self,name):
        self.name=name
        self.rospy=rospy
        self.rospy.init_node(self.name,anonymous=True)
        self.rospy.loginfo("[%s] Starting Node ",self.name)
        self.initParams()
        self.initVariables()
        self.initPublishers()
        self.main()
    
    def initParams(self):
        self.publish_topic = self.rospy.get_param("adc/publish_topic","/adc_data")
        self.acquisition_rate = self.rospy.get_param("adc/acquisition_rate",10)
        return

    def initVariables(self):
        self.rate = self.rospy.Rate(self.acquisition_rate)
        self.sensor = MCP3208()
        self.adcMessage = Mcp3208_data()
        return
    
    def initPublishers(self):
        self.pub_adcRaw = self.rospy.Publisher(self.publish_topic,Mcp3208_data,queue_size=10)
        return
    
    def make_header(self):
        self.adcMessage.header.frame_id = "adc_rawdata"
        self.adcMessage.header.stamp = self.rospy.Time.now()
        return

    def get_rawdata(self):
        self.adcMessage.channel_0 = self.sensor.read(0)
        self.adcMessage.channel_1 = self.sensor.read(1)
        self.adcMessage.channel_2 = self.sensor.read(2)
        self.adcMessage.channel_3 = self.sensor.read(3)
        self.adcMessage.channel_4 = self.sensor.read(4)
        self.adcMessage.channel_5 = self.sensor.read(5)
        self.adcMessage.channel_6 = self.sensor.read(6)
        self.adcMessage.channel_7 = self.sensor.read(7)
        return
    
    def main(self):
        while not self.rospy.is_shutdown():
            self.make_header()
            self.get_rawdata()
            self.pub_adcRaw.publish(self.adcMessage)
            self.rate.sleep()

if __name__ == '__main__':
	try:
		adcRaw = adc_raw('ADC_RAW_DATA_NODE')
	except rospy.ROSInterruptException:
		pass