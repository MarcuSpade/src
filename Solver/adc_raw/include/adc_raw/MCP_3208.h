#ifndef MCP_3208_CLASS_H_
#define MCP_3208_CLASS_H_

#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>  

class MCP_3208
{   
    public:
    
    MCP_3208();
    ~MCP_3208();
    
    void generateMsg();
    void defineSPI(int busSpi,int device,int clock = 500000,int mode = 0);
    __uint16_t readChannel(char input);
    void setMode(char input);

    private:
    int bus {0};
    char mode {1};
    char channel {0};
    char msg1 {0x06};
    char msg2 {0x00};
    unsigned char data[3] = {0x00,0x00,0x00};
};
#endif