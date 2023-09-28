#ifndef LASER_RAW_CLASS_H_
#define LASER_RAW_CLASS_H_

#include "sensor_msgs/ChannelFloat32.h"
#include <vector>
#include <math.h>
#include <adc_raw/Mcp3208_data.h> 
#include "ros/ros.h"
#include <math.h>
#include <wiringPi.h>

class LaserRaw
{
    public:
    ros::NodeHandle n;
    int acquisition_rate {100};

    LaserRaw();
    ~LaserRaw();
    
    float adcConverter(int adc_value);
    void adc_callback(const adc_raw::Mcp3208_data& msg);
    void removeDecimals(float &x);
    void publish_front();
    void publish_back();
    void clearMessages();
    void calcMedia();
    void convertToDistance();
    void setupMessage();
    void removeALLDecimals();
    float calcVar(float media, float value1,float value2,float value3,float value4);
    void enableLasers();
    void disableLasers();

    private:
    float distancia0 {100};
    float distancia1 {10};
    float distancia2 {100};
    float distancia3 {100};
    float distancia4 {10};
    float distancia5 {100};
    adc_raw::Mcp3208_data adcMessage;
    sensor_msgs::ChannelFloat32 laserMessage1;
    sensor_msgs::ChannelFloat32 laserMessage2;
    ros::Subscriber sub_adcRaw;
    ros::Publisher pub_laserRaw1;
    ros::Publisher pub_laserRaw2;
    bool flag;
    std::vector<float> lastRead {0.0 , 0.0, 0.0, 0.0 , 0.0, 0.0};
    std::vector<float> lastValues1 {0.0 , 0.0, 0.0, 0.0 , 0.0, 0.0};
    std::vector<float> lastValues2 {0.0 , 0.0, 0.0, 0.0 , 0.0, 0.0};
    std::vector<float> lastValues3 {0.0 , 0.0, 0.0, 0.0 , 0.0, 0.0};
    std::vector<float> lastValues4 {0.0 , 0.0, 0.0, 0.0 , 0.0, 0.0};
    float variancia {100};
    int enablePin {27};
};

#endif
