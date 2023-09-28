#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
import time
import sys
import os
"""Global variables"""
ledPin=6
GPIO.setmode(GPIO.BCM)
GPIO.setup(ledPin,GPIO.OUT)

GPIO.setwarnings(False)
class Night_Mode():
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
        self.night_mode = False
    
        return

    def initPublishers(self):
        self.night_mode_publish = rospy.Publisher('solver/night_mode', Bool, queue_size=100)
        return
    
    def initSubscribers(self):
        rospy.Subscriber("/night_mode", Bool, self.night_mode_callback)
        return

    def night_mode_callback(self,msg):
        self.night_mode=msg.data
        return

    def main(self):
        while not self.rospy.is_shutdown():
            if(self.night_mode):
                GPIO.output(ledPin,GPIO.HIGH)
            else:
                GPIO.output(ledPin,GPIO.LOW)
            self.night_mode_publish.publish(self.night_mode)
            self.rate.sleep()



if __name__ == "__main__":
    try:
       nightmode = Night_Mode('Solver_Night_Mode')
    except KeyboardInterrupt as e:
        print("Program finished\n")
        sys.stdout.close()
        os.system('clear')
        raise
    finally:
        GPIO.cleanup()

