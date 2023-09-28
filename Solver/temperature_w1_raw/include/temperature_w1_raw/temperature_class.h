#ifndef TEMPERATURE_CLASS_H_
#define TEMPERATURE_CLASS_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <string.h>
#include <time.h>
#include "sensor_msgs/Temperature.h"
#include "ros/ros.h"

class TemperatureRaw
{
    public:
    ros::NodeHandle n;
    int acquisition_rate {100};

    TemperatureRaw();
    ~TemperatureRaw();

    void mountDevice(); // Mount the device:
    void readDirectories();
    void appendPath();
    void checkErros();
    void readTemperature();
    void publish_temp();
    
    private:
    char path[50] = "/sys/bus/w1/devices/";
    char rom[20];
    char buf[100];
    DIR *dirp;
    struct dirent *direntp;
    int fd =-1;
    char *temp;
    float value {31.0};
    sensor_msgs::Temperature tempMessage;
    ros::Publisher pub_temperatureRaw;
    bool checkErro {false};

};

#endif
