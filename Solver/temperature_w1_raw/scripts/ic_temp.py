#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Temperature
#if you want to use spi_cs1, just change the value in the python library
import os
import glob
import time

# These tow lines mount the device:
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')
class temperature_raw():
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
        self.publish_topic = self.rospy.get_param("w1_temperature_sensor/publish_topic","/ic/temperature")
        self.acquisition_rate = self.rospy.get_param("w1_temperature_sensor/acquisition_rate",0.1)
        self.base_dir= self.rospy.get_param("w1_temperature_sensor/base_dir",'/sys/bus/w1/devices/')
        
        return


    def initVariables(self):
        self.rate = self.rospy.Rate(self.acquisition_rate)
        self.device_folder= glob.glob(self.base_dir+'28*')[0]
        self.device_file = self.device_folder+ '/w1_slave'
        self.tempMessage = Temperature()
        return
    
    def initPublishers(self):
        self.pub_temp = self.rospy.Publisher(self.publish_topic,Temperature,queue_size=10)
        return
    
    def make_header(self):
        self.tempMessage.header.frame_id = "ic_temperature"
        self.tempMessage.header.stamp = self.rospy.Time.now()
        return

    def read_temp_raw(self):
        f=open(self.device_file,'r')
        lines = f.readlines()
        f.close()
        return lines

    def read_temp(self):
        lines = self.read_temp_raw()
         # Analyze if the last 3 characters are 'YES'.
        while (lines[0].strip()[-3:] != 'YES'):
            time.sleep(0.01)
            lines = self.read_temp_raw()
        # Find the index of 't=' in a string.
        equals_pos = lines[1].find('t=')
        if equals_pos != -1:
        # Read the temperature .
            temp_string = lines[1][equals_pos+2:]
            temp_c = float(temp_string) / 1000.0
            #temp_f = temp_c * 9.0 / 5.0 + 32.0
            self.tempMessage.temperature=temp_c
        return
    
    def main(self):
        while not self.rospy.is_shutdown():
            self.make_header()
            self.read_temp()
            self.pub_temp.publish(self.tempMessage)
            self.rate.sleep()

if __name__ == '__main__':
	try:
		adcRaw = temperature_raw('IC_TEMPERATURE_NODE')
	except rospy.ROSInterruptException:
		pass




