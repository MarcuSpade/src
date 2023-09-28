#include "ros/ros.h"
#include "laser_control/laser_control_class.h"
#include <memory>

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "laser_control_node");
  std::shared_ptr <LaserController> myLaserController = std::make_shared<LaserController>();
  ros::Rate loop_rate(myLaserController -> acquisition_rate);
  ROS_INFO("laser_controller: laser_control_node running...");

  while (myLaserController->n.ok())
  {
    // if(myLaserController->get_change_back() && myLaserController -> get_change_front())
    // {
    myLaserController->setAngular();
    myLaserController->setLinear();
    if((myLaserController->getOutLinear() != myLaserController->getAuxLinear())||(myLaserController->getOutAngular() != myLaserController->getAuxAngular()))
    {
      myLaserController->publish_speed();
      myLaserController->setAux();
    }
    myLaserController->publish_frequency();
    // }
    ros::spinOnce();  
    loop_rate.sleep();
  }
  return 0;
}
