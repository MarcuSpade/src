#include "temperature_w1_raw/temperature_class.h"

TemperatureRaw::TemperatureRaw()
{
    // Topics to publish
    pub_temperatureRaw = n.advertise<sensor_msgs::Temperature>("/ic/temperature",acquisition_rate);
    // Topic to subscribe
    // Without subscibed topic

    // Get publish rate from file

    n.param("temperature_w1_raw/acquisition_rate", acquisition_rate, 100);
    mountDevice();
    readDirectories();
    appendPath();
}

TemperatureRaw::~TemperatureRaw(){}

void TemperatureRaw::mountDevice() // Mount the device:
{
    system("sudo modprobe w1-gpio");
    system("sudo modprobe w1-therm");
    // Check if /sys/bus/w1/devices/ exists.
    if((dirp = opendir(path)) == NULL)
    {
        ROS_WARN("Opendir error!");
        checkErro = true;
    }
}

void TemperatureRaw::readDirectories()
{
    while((direntp = readdir(dirp)) != NULL)
    {
        // If 28-00000 is the substring of d_name,
        // then copy d_name to rom and print rom.  
        if(strstr(direntp->d_name,"28-00000"))
        {
            strcpy(rom,direntp->d_name);
        }
    }
    closedir(dirp);
}

void TemperatureRaw::appendPath()
{
    strcat(path,rom);
    strcat(path,"/w1_slave");
}

void TemperatureRaw::checkErros()
{
    // Open the file in the path.
    if((fd = open(path,O_RDONLY)) < 0)
    {
        ROS_WARN("Open error!");
        checkErro = true;
    }
    // Read the file
    if(read(fd,buf,sizeof(buf)) < 0)
    {
        ROS_WARN("Read error!");
        checkErro = true;
    }
}
void TemperatureRaw::readTemperature()
{   

    checkErros();

    if (checkErro == true)
    {
        ROS_WARN("Repeating the last reading!");
        checkErro = false;
    }
    else
    {  
        temp = strchr(buf,'t');
        // Read the string following "t=".
        sscanf(temp,"t=%s",temp);
        // atof: changes string to float.
        value = atof(temp)/1000;
    }
    tempMessage.temperature = value;
}

void TemperatureRaw::publish_temp()
{
    pub_temperatureRaw.publish(tempMessage);
}