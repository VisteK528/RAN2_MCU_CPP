#ifndef ROBOTARMNUMBER2CPP_SENSORS_HPP
#define ROBOTARMNUMBER2CPP_SENSORS_HPP

#include "../../Inc/gpio.h"
#include <iostream>

typedef enum{UP, DOWN} ENDSTOP_TYPE;

class Sensor{
public:
    virtual ~Sensor() = default;
    virtual bool checkSensor()=0;
};

class Endstop: public Sensor{
public:
    Endstop(uint8_t signal_pin, ENDSTOP_TYPE type);
    Endstop() = default;
    ~Endstop() = default;
    bool checkSensor() override;
private:
    uint8_t signal_pin;
    ENDSTOP_TYPE type;

};
#endif //ROBOTARMNUMBER2CPP_SENSORS_HPP
