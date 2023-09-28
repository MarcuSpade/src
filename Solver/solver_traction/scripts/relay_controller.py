#!/usr/bin/env python3
import rospy
import sys
import os
from std_msgs.msg import Bool
from smbus2 import SMBus

class Relay_Controller():
    def __init__(self, name):
        self.name = name
        self.rospy = rospy
        self.rospy.init_node(self.name, anonymous=True)
        self.rospy.loginfo("[%s] Starting Node ", self.name)

        self.initVariables()
        self.initPublishers()
        self.initSubscribers()
        #self.main()

    def initVariables(self):
        self.rate = self.rospy.Rate(1)
        self.remote_io_mode = True
        self.night_mode = True
        self.byte = 0x00
        self.relay_notused_0 = 2
        self.relay_notused_1 = 3
        self.relay_headlight = 0
        self.relay_drive = 1
        self.bus = SMBus(1)
        self.address = 0x27
        return

    def initPublishers(self):
        self.remote_io_mode_publish = rospy.Publisher('solver/remote_MD49', Bool, queue_size=100)
        self.night_mode_publish = rospy.Publisher('solver/night_mode', Bool, queue_size=100)
        return

    def initSubscribers(self):
        rospy.Subscriber("/remote_MD49", Bool, self.remote_io_mode_callback)
        rospy.Subscriber("/night_mode", Bool, self.night_mode_callback)
        return

    def remote_io_mode_callback(self, msg):
        self.remote_io_mode = msg.data
        return

    def night_mode_callback(self, msg):
        self.night_mode = msg.data
        return

    def main(self):
        sent = False
        while not sent:
            try:
                self.bus.write_byte(self.address, 0x00)
                sent = True
            except IOError:
                self.rate.sleep()
        while not self.rospy.is_shutdown():
            try:
                self.byte = self.byte|(0x01 << self.relay_headlight) if self.night_mode else self.byte&~(0x01 << self.relay_headlight)
                self.byte = self.byte|(0x01 << self.relay_drive) if self.remote_io_mode else self.byte&~(0x01 << self.relay_drive)
                self.bus.write_byte(self.address, self.byte)
            except IOError:
                self.rate.sleep()
            self.remote_io_mode_publish.publish(self.remote_io_mode)
            self.night_mode_publish.publish(self.night_mode)
            self.rate.sleep()
        if self.night_mode:
            self.byte = self.byte&~(0x01 << self.relay_headlight)
        if self.remote_io_mode:
            self.byte = self.byte&~(0x01 << self.relay_drive)
        sent = False
        while not sent:
            try:
                self.bus.write_byte(self.address, self.byte)
                sent = True
            except IOError:
                sent = False

if __name__ == "__main__":
    try:
        relay_controller = Relay_Controller('Solver_Relay_Controller_Node')
        relay_controller.main()
    except KeyboardInterrupt as e:
        print("Program finished\n")
        sys.stdout.close()
        os.system('clear')
        raise
    rospy.loginfo("Exiting")
