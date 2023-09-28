#!/usr/bin/env python3
from smbus2 import SMBus,i2c_msg
import rospy
from std_msgs.msg import Int16
import time
import sys
import os

class Charger_attiny():
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
        self.rate = self.rospy.Rate(0.2)
        self.bus=SMBus(0)
        time.sleep(1)
        self.address=0x19
        self.ic_battery= Int16()
        self.driver_battery=Int16()
        return

    def initPublishers(self):
        self.ic_battery_publish = rospy.Publisher('ic_battery', Int16,queue_size=100)
        self.driver_battery_publish = rospy.Publisher('driver_battery', Int16,queue_size=100)
        return

    def initSubscribers(self):
        rospy.Subscriber("/calibrate_attiny",Int16,self.calibration_callback)
        return
    
    def calibration_callback(self,msg):
        command = int(msg.data)
        if(msg.data ==4):
            self.bus.write_byte(self.address,command)
        return
    
    def main(self):
        while not self.rospy.is_shutdown():
            self.bus.write_byte(self.address,int(1))
            time.sleep(0.5)
            self.ic_battery=(self.bus.read_byte(self.address)<<8) | (self.bus.read_byte(self.address) & 0xFF)
            time.sleep(0.5)
            self.bus.write_byte(self.address,int(2))
            time.sleep(0.5)
            self.driver_battery=(self.bus.read_byte(self.address)<<8) | (self.bus.read_byte(self.address) & 0xFF)
            
            self.ic_battery_publish.publish(self.ic_battery)
            self.driver_battery_publish.publish(self.driver_battery)
            self.rate.sleep()
    
if __name__ == "__main__":
    try:
        
        charger = Charger_attiny('Solver_charger')
    except KeyboardInterrupt as e:
        print("Program finished\n")
        sys.stdout.close()
        os.system('clear')
        raise
        
            
