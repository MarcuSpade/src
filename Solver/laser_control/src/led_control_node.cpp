#include "ros/ros.h"
#include "laser_control/led_control_class.h"
#include <memory>

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "led_control_node");
  std::shared_ptr <LedController> myLedController = std::make_shared<LedController>();
  ros::Rate loop_rate(10);
  ROS_INFO("laser_controller: led_control_node running...");
  myLedController->setupPin();

  while (myLedController->n.ok())
  {
    myLedController->blink();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
