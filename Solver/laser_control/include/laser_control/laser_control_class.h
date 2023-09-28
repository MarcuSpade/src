#ifndef LASER_CONTROLLER_CLASS_H_
#define LASER_CONTROLLER_CLASS_H_

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/ChannelFloat32.h"
#include <stdlib.h>
#include <vector>
#include <math.h>  
#include <string>

class LaserController
{
  public:
  
  ros::NodeHandle n;
  int acquisition_rate {100};
  
  LaserController();
  ~LaserController();
  
  void front_callback(const sensor_msgs::ChannelFloat32& msg1);
  void fl_callback(const std_msgs::Float32 & msg1);
  void fm_callback(const std_msgs::Float32 & msg1);
  void fr_callback(const std_msgs::Float32 & msg1);
  void bl_callback(const std_msgs::Float32 & msg1);
  void bm_callback(const std_msgs::Float32 & msg1);
  void br_callback(const std_msgs::Float32 & msg1);
  void back_callback(const sensor_msgs::ChannelFloat32& msg2);
  void vel_callback(const geometry_msgs::Twist& data);
  bool get_change_front();
  bool get_change_back();
  void setAngular();
  void setLinear();
  float getOutLinear();
  float getOutAngular();
  float getAuxLinear();
  float getAuxAngular();
  void publish_speed();
  void publish_frequency();
  void setAux();
  void setStopDist();

  private:
  float ledsFrequency;
  std_msgs::Float32 ledsFrequency32;
  sensor_msgs::ChannelFloat32 laserMessage1;
  sensor_msgs::ChannelFloat32 laserMessage2;
  std_msgs::Float32 fl_msg;
  std_msgs::Float32 fm_msg;
  std_msgs::Float32 fr_msg;
  std_msgs::Float32 bl_msg;
  std_msgs::Float32 bm_msg;
  std_msgs::Float32 br_msg;  
  geometry_msgs::Twist speedIn;
  geometry_msgs::Twist speedOut;
  geometry_msgs::Twist speedAux;
  float speed_angular_z {0};
  float speed_linear_x {0};
  // bool change_front = false;
  // bool change_back = false;
  bool turn_back = false;
  bool reduction = false;
  int reduction_count = 0;
  float reduction_rate = 0.05;
  float stop_frontal {200.0};
  float stop_frontal_fast {750.0};
  ros::Subscriber sub_sensor_fl;
  ros::Subscriber sub_sensor_fm;
  ros::Subscriber sub_sensor_fr;
  ros::Subscriber sub_sensor_bl;
  ros::Subscriber sub_sensor_bm;
  ros::Subscriber sub_sensor_br;
  ros::Subscriber sub_vel;
  ros::Publisher pub_vel;
  ros::Publisher pub_led;
  std::vector<float> frontais {100.0 , 10.0, 100};
  std::vector<float> traseiros {100.0 , 10.0, 100};
  int frontHorizontal1 = 0;
  int frontVertical = 1;
  int frontHorizontal2 = 2;
  int backHorizontal1 = 0;
  int backVertical = 1;
  int backHorizontal2 = 2;
  float vertical_degree {100.0};
  std::vector<float> lastRead {0.0 , 0.0, 0.0};
  std::vector<float> lastValues1 {0.0 , 0.0, 0.0};
  std::vector<float> lastValues2 {0.0 , 0.0, 0.0};
  std::vector<float> lastValues3 {0.0 , 0.0, 0.0};
  std::vector<float> lastValues4 {0.0 , 0.0, 0.0};
  std::string receive_vel_topic;
  std::string publish_vel_topic;
  std::string tof_sensor_fl_topic;
  std::string tof_sensor_fm_topic;
  std::string tof_sensor_fr_topic;
  std::string tof_sensor_bl_topic;
  std::string tof_sensor_bm_topic;
  std::string tof_sensor_br_topic;  
};
#endif
