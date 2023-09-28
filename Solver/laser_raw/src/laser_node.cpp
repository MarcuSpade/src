#include "ros/ros.h"
#include "laser_raw/laser_class.h"                                        
#include <memory>

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "laser_raw_node");
  std::shared_ptr <LaserRaw> myLaserRaw = std::make_shared<LaserRaw>();
  ros::Rate loop_rate(myLaserRaw->acquisition_rate);
  ROS_INFO("laser_raw: laser_raw_node running...");
  while (myLaserRaw->n.ok())
  {
    myLaserRaw->publish_front();
    myLaserRaw->publish_back();
    //myLaserRaw->clearMessages();
    ros::spinOnce();  
    loop_rate.sleep();    
  }
  return 0;
}
