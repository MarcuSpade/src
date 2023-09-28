#!/usr/bin/python3

"""
Código para converter as mensagens publicadas por TOF 10120 em mensagens de distância em metros.
"""

import rospy
from std_msgs.msg import Float32
from rangefinders_i2c.ReadSensor import ReadSensor
#import tf

class Tof10120Node():
    def __init__(self):
        # ROS initialization
        self.rospy = rospy
        self.rospy.init_node("tof10120_node")
        self.rospy.loginfo("Starting TOF10120 Rangefinder Node")
        self.node_name = rospy.get_name()

        # Parameter initialization
        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.initVariables()
        self.main()

    def initParameters(self):
        self.tof_topic = self.rospy.get_param(self.node_name+"/tof_topic","/tof_distance")
        self.acquisition_rate = self.rospy.get_param("//tof/acquisition_rate", 100)
        self.i2c_address = self.rospy.get_param(self.node_name+"/i2c_address", 0x40)
        self.average_qnt = self.rospy.get_param("/tof/average_filter_size", 3)

    def initVariables(self):
        self.rate = self.rospy.Rate(self.acquisition_rate)
        try:
            self.sensor = ReadSensor(1)
        except TypeError:
            raise ValueError(" --- I2C bus not found !!!")
        self.distance = 2000
        self.last_distance = 2000
        self.average_measures = []
        #self.br = tf.TransformBroadcaster()
        return


    def initSubscribers(self):
        return

    def initPublishers(self):
        self.pub_tof_topic = self.rospy.Publisher(self.tof_topic, Float32, queue_size=10)
        return

    # Main loop
    def main(self):
        while not self.rospy.is_shutdown():
            try:
#                self.rospy.loginfo(self.node_name+"i2c_address")
#                self.rospy.loginfo(self.i2c_address)
                self.distance = float(self.sensor.GetDistance(self.i2c_address))
                if self.distance > 25 and self.distance <= 2100:
                    if len(self.average_measures) < self.average_qnt:
                        self.average_measures.append(self.distance)
                    else:
                        self.average_measures.pop(0)
                        self.distance = (sum(self.average_measures)+self.distance)/self.average_qnt
                        self.average_measures.append(self.distance)
                        self.last_distance = self.distance
                        self.pub_tof_topic.publish(self.distance)
                else:
                    self.distance = self.last_distance
                    self.pub_tof_topic.publish(self.distance)
                self.rate.sleep()
            except TypeError:
                self.rate.sleep()


if __name__ == '__main__':
    try:
        tof10120_rangefinder = Tof10120Node()
    except rospy.ROSInterruptException:
        pass
