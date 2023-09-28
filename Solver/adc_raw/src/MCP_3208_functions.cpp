#include "adc_raw/MCP_3208.h"   

MCP_3208::MCP_3208(){}
MCP_3208::~MCP_3208(){}

void MCP_3208::generateMsg()
{   char mode1 = mode;
    char channel1 =channel;
    char msgPart1 {0x00};                // Channel will receive -> 0 0 0 0 0 a b c <- binary containing channel to be read
    char msgPart2 {0x00};                // Channel will be divided in 2 msgs to make two chars to be sent in SPI
    msg1 = {0x04};                       // 0 0 0 0 0 1 0 0  
    mode1 = mode << 1;                   // 0 0 0 0 0 0 x 0
    msg1 = msg1 | mode1;                 // 0 0 0 0 0 1 x 0
    msgPart1 = channel1 >> 2;            // 0 0 0 0 0 0 0 a
    msg1 = msg1 | msgPart1;              // 0 0 0 0 0 1 x a
    msg2 = channel1 << 6;                // b c 0 0 0 0 0 0
    data[0] = msg1;
    data[1] = msg2;
    data[2] = {0x00};
}

void MCP_3208::defineSPI(int busSPI,int device,int clock,int mode)
{
    bus=busSPI;
    wiringPiSPISetupMode(device, clock,mode);
}

__uint16_t MCP_3208::readChannel(char input)
{   
    channel = input;
    generateMsg();
    wiringPiSPIDataRW(bus,data,3);
    data[1]=data[1]<<4;                   //b11 b10 b9 b8 0 0 0 0
    __uint16_t result = 0x0000 | data[1]; //0000 0000 b11 b10 b9 b8 0000
    result = result<<4;                   //0000 b11 b10 b9 b8 0000 0000
    result = result | data[2];            // 0 0 0 0 b11 b10 b9 b8 b7 b6 b5 b4 b3 b2 b1 b0 
    return result;
}

void MCP_3208::setMode(char input)
{
    mode = input;
}
