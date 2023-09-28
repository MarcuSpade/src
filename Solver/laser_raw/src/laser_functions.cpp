#include "laser_raw/laser_class.h"      

LaserRaw::LaserRaw()
{
    n.param("laser_raw/acquisition_rate", acquisition_rate, 100);

    // Topics to publish
    pub_laserRaw1 = n.advertise<sensor_msgs::ChannelFloat32>("/laser_front",acquisition_rate);
    pub_laserRaw2 = n.advertise<sensor_msgs::ChannelFloat32>("/laser_back",acquisition_rate);

    // Topic to subscribe
    sub_adcRaw = n.subscribe("/adc_data", acquisition_rate, &LaserRaw::adc_callback, this);

    laserMessage1.name = "Laser Frontal";
    laserMessage2.name = "Laser Traseiro";
    
    flag = false;
    enableLasers();
}

LaserRaw::~LaserRaw()
{
    disableLasers();
}

float LaserRaw::adcConverter(int adc_value)
{   
    float distanciaConv;
    distanciaConv = 40*pow(static_cast<float>(adc_value)*5/4096,-2) + 5;
    // if(adc_value > 3150)
    // { distanciaConv = static_cast<float>(adc_value)/(-233.33) + 23.5; }
    
    // else if(adc_value > 2020)
    // { distanciaConv = static_cast<float>(adc_value)/(-183.33) + 26.725; }
    
    // else if(adc_value > 1340)
    // { distanciaConv = static_cast<float>(adc_value)/(-68) + 45.706; }
    
    // else if(adc_value > 910)
    // { distanciaConv = static_cast<float>(adc_value)/(-28.66) + 73.74; }
    
    // else if(adc_value > 640)
    // { distanciaConv = static_cast<float>(adc_value)/(-13.5) + 109.407; }
    
    // else
    // { distanciaConv = static_cast<float>(adc_value)/(-8.75) + 135.14; }
    
    return distanciaConv;
}

void LaserRaw::adc_callback(const adc_raw::Mcp3208_data& msg)
{
    adcMessage = msg;
    convertToDistance();
    calcMedia();
    removeALLDecimals();
    setupMessage();
}

void LaserRaw::removeALLDecimals()
{
    removeDecimals(distancia0);
    removeDecimals(distancia1);
    removeDecimals(distancia2);
    removeDecimals(distancia3);
    removeDecimals(distancia4);
    removeDecimals(distancia5);
}

float LaserRaw::calcVar(float media, float value1,float value2,float value3,float value4)
{
    float var{0};
    var = (pow(media-value1,2) + pow(media-value2,2) + pow(media-value3,2) + pow(media-value4,2))/4;
    ROS_INFO("%f",var);
    return var;
}

void LaserRaw::removeDecimals(float &x)
{
    x *= 100;
    int temp;
    temp = static_cast<int>(x);
    x = temp/100;
}

void LaserRaw::publish_front()
{
    pub_laserRaw1.publish(laserMessage1);
}

void LaserRaw::publish_back()
{   
    pub_laserRaw2.publish(laserMessage2);
}

void LaserRaw::clearMessages()
{
    laserMessage1.values.clear();
    laserMessage2.values.clear();
}

void LaserRaw::calcMedia()
{
    lastRead.clear();
    lastRead.push_back(distancia0);
    lastRead.push_back(distancia1);
    lastRead.push_back(distancia2);
    lastRead.push_back(distancia3);
    lastRead.push_back(distancia4);
    lastRead.push_back(distancia5);
    
    //variancia = calcVar(distancia3,lastValues1[3],lastValues2[3],lastValues3[3],lastValues4[3]);
    distancia0 = (distancia0 + lastValues1[0] + lastValues2[0] + lastValues3[0] + lastValues4[0])/5;
    distancia1 = (distancia1 + lastValues1[1] + lastValues2[1] + lastValues3[1] + lastValues4[1])/5;
    distancia2 = (distancia2 + lastValues1[2] + lastValues2[2] + lastValues3[2] + lastValues4[2])/5;
    distancia3 = (distancia3 + lastValues1[3] + lastValues2[3] + lastValues3[3] + lastValues4[3])/5;
    distancia4 = (distancia4 + lastValues1[4] + lastValues2[4] + lastValues3[4] + lastValues4[4])/5;
    distancia5 = (distancia5 + lastValues1[5] + lastValues2[5] + lastValues3[5] + lastValues4[5])/5;
    
    lastValues4 = lastValues3;
    lastValues3 = lastValues2;
    lastValues2 = lastValues1;
    lastValues1 = lastRead;
}

void LaserRaw::convertToDistance()
{
    distancia0 = adcConverter(adcMessage.channel_0);
    distancia1 = adcConverter(adcMessage.channel_1);
    distancia2 = adcConverter(adcMessage.channel_2);
    distancia3 = adcConverter(adcMessage.channel_3);
    distancia4 = adcConverter(adcMessage.channel_6);
    distancia5 = adcConverter(adcMessage.channel_5);
}

void LaserRaw::setupMessage()
{
    clearMessages();
    laserMessage1.values.push_back(distancia0);
    laserMessage1.values.push_back(distancia1);
    laserMessage1.values.push_back(distancia2);
    laserMessage2.values.push_back(distancia3);
    laserMessage2.values.push_back(distancia4);
    laserMessage2.values.push_back(distancia5);
    flag = true;
}

void LaserRaw::enableLasers()
{
    wiringPiSetup();			// Setup the library
    pinMode(enablePin, OUTPUT);		// Configure GPIO as an output
    digitalWrite (enablePin, HIGH);
}

void LaserRaw::disableLasers()
{
    digitalWrite (enablePin, LOW);
}
