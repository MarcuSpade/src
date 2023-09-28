#ifndef LASER_RAW_CLASS_H_
#define LASER_RAW_CLASS_H_

#include "ros/ros.h"
#include <adc_raw/Mcp3208_data.h>
#include "adc_raw/MCP_3208.h"  
#include <memory>                          

#define Qtd_Amostras 20
                               
class AdcRaw
{
    public:
    ros::NodeHandle n;
    int acquisition_rate {100};

    AdcRaw();
    ~AdcRaw();
    
    void get_rawdata();
    void setMode(int setedMode);
    void setupSPI(int bus,int device);
    void publish_adc();

    private:
    char channel_0 = {0};
    char channel_1 = {1};
    char channel_2 = {2};
    char channel_3 = {3};
    char channel_4 = {4};
    char channel_5 = {5};
    char channel_6 = {6};
    char channel_7 = {7};

    int Posicao = 0;
    __uint16_t Leituras_anteriores[8][Qtd_Amostras] = {0};
    __uint16_t Nova_Leitura = 0;
    __uint16_t Media = 0;
    long Soma[8] = {0};
    

    adc_raw::Mcp3208_data adcMessage;
    std::shared_ptr<MCP_3208> sensor = std::make_shared<MCP_3208>();
    ros::Publisher pub_adcRaw;
};
#endif
