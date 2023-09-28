#!/usr/bin/python3

"""
Código para converter as mensagens publicadas por gy88_interface_node em mensagens padrão para os filtros de Imu_tools.
"""

import rospy
from adc_raw.msg import Mcp3208_data
from battery_charge.msg import Battery_state

class Battery_State():
    def __init__(self):
        # ROS initialization
        self.rospy = rospy
        self.rospy.init_node("battery_charge_node")
        self.rospy.loginfo("Starting Battery Charge Node")
        self.node_name = rospy.get_name()

        # Parameter initialization
        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.initVariables()
        self.main()

    def initParameters(self):
        self.battery_readings_topic = self.rospy.get_param("battery/readings_topic","/adc_data")
        self.battery_state_topic = self.rospy.get_param("battery/data_topic","/battery_state")
        self.acquisition_rate = self.rospy.get_param("battery/acquisition_rate", 1)
        self.HIGH_READING = self.rospy.get_param("battery/HIGH_READING", 4095) # 12 bit
        self.LOW_READING = self.rospy.get_param("battery/LOW_READING", 0)
        self.READING_RATIO_1 = self.rospy.get_param("battery/READING_RATIO_1", 0.10) # 3K9/36K9
        self.READING_RATIO_2 = self.rospy.get_param("battery/READING_RATIO_2", 0.10) # 3K9/36K9
        self.VREF = self.rospy.get_param("battery/VREF", 3.0)

    def initVariables(self):
        self.rate = self.rospy.Rate(self.acquisition_rate)
        self.battery_reading_msg = Mcp3208_data()
        self.battery_state_msg = Battery_state()
        return


    def initSubscribers(self):
        self.sub_battery_readings = self.rospy.Subscriber(self.battery_readings_topic, Mcp3208_data, self.battery_reader)
        return

    def initPublishers(self):
        self.pub_battery_state = self.rospy.Publisher(self.battery_state_topic, Battery_state, queue_size=10)
        return


    def battery_reader(self, data):
        
        self.battery_reading_msg = data

    def battery_publisher(self):

        self.battery_state_msg.charging = self.battery_reading_msg.channel_1 > self.HIGH_READING/2

        self.battery_state_msg.battery1_voltage = self.battery_reading_msg.channel_0*(self.VREF/self.HIGH_READING)/self.READING_RATIO_1

        self.battery_state_msg.battery2_voltage = self.battery_reading_msg.channel_2*(self.VREF/self.HIGH_READING)/self.READING_RATIO_2
        
        self.pub_battery_state.publish(self.battery_state_msg)

    # Main loop
    def main(self):
        while not self.rospy.is_shutdown():
            self.battery_publisher()
            self.rate.sleep()
            #self.rospy.spin()


if __name__ == '__main__':
    try:
        battery_state_node = Battery_State()
    except rospy.ROSInterruptException:
        pass
