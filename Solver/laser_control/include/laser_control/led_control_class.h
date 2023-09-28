#ifndef LED_CONTROLLER_CLASS_H_
#define LED_CONTROLLER_CLASS_H_

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <wiringPi.h>


class LedController
{
  public:
  ros::NodeHandle n;

  LedController();
  ~LedController();
   
  void ledsCallback(const std_msgs::Float32& msg);
  void blink();
  void setupPin();

  private:
  float ledsFrequency {0.0};
  int ledPin {5};
  ros::Subscriber sub_led_frequency;
};

#endif
