#include "ros/ros.h"
#include "temperature_w1_raw/temperature_class.h"
#include <memory>

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "temperature_w1_raw");
  std::shared_ptr <TemperatureRaw> myTemperatureRaw = std::make_shared<TemperatureRaw>();
  ros::Rate loop_rate(myTemperatureRaw->acquisition_rate);
  ROS_INFO("temperature_w1_raw: temperature_w1_raw running...");
  while (myTemperatureRaw->n.ok())
  {
    myTemperatureRaw->readTemperature();
    myTemperatureRaw->publish_temp();
    ros::spinOnce();  
    loop_rate.sleep();    
  }
  return 0;
}
