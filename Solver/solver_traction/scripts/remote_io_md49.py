#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
import time
import sys
import os
"""Global variables"""
gpio_pin=17
GPIO.setmode(GPIO.BCM)
GPIO.setup(gpio_pin,GPIO.OUT)

GPIO.setwarnings(False)
class Remote_io():
    def __init__(self,name):
        self.name=name
        self.rospy=rospy
        self.rospy.init_node(self.name,anonymous=True)
        self.rospy.loginfo("[%s] Starting Node ",self.name)

        self.initVariables()
        self.initPublishers()
        self.initSubscribers()
        self.main()

    def initVariables(self):
        self.rate = self.rospy.Rate(10)
        self.remote_io_mode = True
    
        return

    def initPublishers(self):
        self.remote_io_mode_publish = rospy.Publisher('solver/remote_MD49', Bool, queue_size=100)
        return
    
    def initSubscribers(self):
        rospy.Subscriber("/remote_MD49", Bool, self.remote_io_mode_callback)
        return

    def remote_io_mode_callback(self,msg):
        self.remote_io_mode=msg.data
        return

    def main(self):
        while not self.rospy.is_shutdown():
            if(self.remote_io_mode):
                GPIO.output(gpio_pin,GPIO.HIGH)
            else:
                GPIO.output(gpio_pin,GPIO.LOW)
            self.remote_io_mode_publish.publish(self.remote_io_mode)
            self.rate.sleep()



if __name__ == "__main__":
    try:
       remote_io = Remote_io('Solver_Remote_io_Mode')
    except KeyboardInterrupt as e:
        print("Program finished\n")
        sys.stdout.close()
        os.system('clear')
        raise
    finally:
        GPIO.cleanup()

