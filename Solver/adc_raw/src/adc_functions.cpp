#include <adc_raw/adc_class.h>    

AdcRaw::AdcRaw()
{
    // Get publish rate from file
    n.param("adc_raw/acquisition_rate", acquisition_rate, 100);

    // Topics to publish
    pub_adcRaw = n.advertise<adc_raw::Mcp3208_data>("/adc_data",acquisition_rate);
    // Topic to subscribe
    // Without subscibed topic
}

AdcRaw::~AdcRaw(){}

void AdcRaw::get_rawdata()
{ 


    Nova_Leitura = sensor->readChannel(channel_0);
    Soma[0] += Nova_Leitura - Leituras_anteriores[0][Posicao];
    Leituras_anteriores[0][Posicao] = Nova_Leitura;
    Media =(__uint16_t) (Soma[0]/Qtd_Amostras);
    adcMessage.channel_0 = Media;

    Nova_Leitura = sensor->readChannel(channel_1);
    Soma[1] += Nova_Leitura - Leituras_anteriores[1][Posicao];
    Leituras_anteriores[1][Posicao] = Nova_Leitura;
    Media =(__uint16_t) (Soma[1]/Qtd_Amostras);
    adcMessage.channel_1 = Media;

    Nova_Leitura = sensor->readChannel(channel_2);
    Soma[2] += Nova_Leitura - Leituras_anteriores[2][Posicao];
    Leituras_anteriores[2][Posicao] = Nova_Leitura;
    Media =(__uint16_t) (Soma[2]/Qtd_Amostras);
    adcMessage.channel_2 = Media;

    Nova_Leitura = sensor->readChannel(channel_3);
    Soma[3] += Nova_Leitura - Leituras_anteriores[3][Posicao];
    Leituras_anteriores[3][Posicao] = Nova_Leitura;
    Media =(__uint16_t) (Soma[3]/Qtd_Amostras);
    adcMessage.channel_3 = Media;

    Nova_Leitura = sensor->readChannel(channel_4);
    Soma[4] += Nova_Leitura - Leituras_anteriores[4][Posicao];
    Leituras_anteriores[4][Posicao] = Nova_Leitura;
    Media =(__uint16_t) (Soma[4]/Qtd_Amostras);
    adcMessage.channel_4 = Media;

    Nova_Leitura = sensor->readChannel(channel_5);
    Soma[5] += Nova_Leitura - Leituras_anteriores[5][Posicao];
    Leituras_anteriores[5][Posicao] = Nova_Leitura;
    Media =(__uint16_t) (Soma[5]/Qtd_Amostras);
    adcMessage.channel_5 = Media;

    Nova_Leitura = sensor->readChannel(channel_6);
    Soma[6] += Nova_Leitura - Leituras_anteriores[6][Posicao];
    Leituras_anteriores[6][Posicao] = Nova_Leitura;
    Media =(__uint16_t) (Soma[6]/Qtd_Amostras);
    adcMessage.channel_6 = Media;

    Nova_Leitura = sensor->readChannel(channel_7);
    Soma[7] += Nova_Leitura - Leituras_anteriores[7][Posicao];
    Leituras_anteriores[7][Posicao] = Nova_Leitura;
    Media =(__uint16_t) (Soma[7]/Qtd_Amostras);
    adcMessage.channel_7 = Media;

    Posicao = (Posicao+1)%Qtd_Amostras;
}

void AdcRaw::setMode(int setedMode) 
{
    sensor->setMode(setedMode);
}

void AdcRaw::setupSPI(int bus,int device)
{
    sensor->defineSPI(bus,device);
}

void AdcRaw::publish_adc()
{
    pub_adcRaw.publish(adcMessage);
}
