#include "ros/ros.h"
#include "adc_raw/MCP_3208.h"   
#include "adc_raw/adc_class.h" 
#include <memory>

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "adc_raw_node");
  std::shared_ptr<AdcRaw> myAdcRaw = std::make_shared<AdcRaw>();
  myAdcRaw->setupSPI(0,0);
  char mode = {1};
  myAdcRaw->setMode(mode);
  ros::Rate loop_rate(myAdcRaw->acquisition_rate);
  ROS_INFO("adc_raw: adc_raw_node running...");
  while (myAdcRaw->n.ok())
  {
    myAdcRaw->get_rawdata();
    myAdcRaw->publish_adc();
    ros::spinOnce();  
    loop_rate.sleep();    
  }
  
  return 0;
}
