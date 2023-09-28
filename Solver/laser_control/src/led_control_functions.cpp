#include "laser_control/led_control_class.h"

LedController::LedController()
{
    sub_led_frequency = n.subscribe("/led_frequency", 10, &LedController::ledsCallback, this);
    // Read initial parameters from parameter service
    float x {0.0};
    n.param("led_control/initial_frequency", ledsFrequency, x);
    n.param("led_control/led_pin", ledPin, 5);
}

LedController::~LedController(){}

void LedController::ledsCallback(const std_msgs::Float32& msg)
{
  ledsFrequency= msg.data;
}

void LedController::blink()
{   
    if (ledsFrequency == 0.0)
    {
        digitalWrite (ledPin, HIGH) ;
        delay (1);
    }
    else
    {
        digitalWrite (ledPin, HIGH);
        delay (ledsFrequency);
        digitalWrite (ledPin, LOW);
        delay (ledsFrequency); 
    }
}

void LedController::setupPin()
{
    wiringPiSetup();			// Setup the library
    pinMode(ledPin, OUTPUT);		// Configure GPIO0 as an output
    digitalWrite (ledPin, HIGH) ;
}