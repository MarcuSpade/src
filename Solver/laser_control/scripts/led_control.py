#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

ledPin = 5 # BOARD pin 15 GPIO 5

# Pin Setup:
GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
GPIO.setup(ledPin, GPIO.OUT) # LED pin set as output

# Initial state for LEDs:
GPIO.output(ledPin, GPIO.LOW)

class led_control():
    def __init__(self,name):
        self.name=name
        self.rospy=rospy
        self.rospy.init_node(self.name,anonymous=True)
        self.rospy.loginfo("[%s] Starting Node ",self.name)
        self.initParams()
        self.initVariables()
        self.initSubscribers()
        self.main()

    def initParams(self):
        self.subscribe_led = self.rospy.get_param("control/subscribe_led","/led_frequency")
        self.acquisition_rate = self.rospy.get_param("laser/acquisition_rate",10)
        return        

    def initVariables(self):
        self.rate = self.rospy.Rate(self.acquisition_rate)
        self.ledsFrequency = Float32()
        self.ledsFrequency=0.0
        # self.change = False

    def initSubscribers(self):
        self.sub_frequency =  self.rospy.Subscriber(self.subscribe_led,Float32, self.leds_callback) 
        return        
    
    def leds_callback(self,msg):
        self.ledsFrequency = msg.data
        # self.change=True
        return

    def blink(self,data):
        if (data == 0.0):
            GPIO.output(ledPin, GPIO.HIGH)
            time.sleep(0.1)
        else:
            GPIO.output(ledPin, GPIO.HIGH)
            time.sleep(data)
            GPIO.output(ledPin, GPIO.LOW)
            time.sleep(data)

    def main(self):
        while not self.rospy.is_shutdown():
            
            self.blink(self.ledsFrequency)
            
            self.rate.sleep()

if __name__ == '__main__':
	try:
		ledControl = led_control('LED_CONTROL_DATA_NODE')
	except rospy.ROSInterruptException:
		pass
