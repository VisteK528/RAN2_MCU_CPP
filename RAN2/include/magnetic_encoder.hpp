#ifndef RAN2_MCU_CPP_MAGNETIC_ENCODER_HPP
#define RAN2_MCU_CPP_MAGNETIC_ENCODER_HPP

#include "as5600.hpp"

class MagneticEncoder{
public:
    MagneticEncoder(uint8_t encoderAddress, uint8_t channelNumber, I2C_HandleTypeDef* i2c, float homingPosition, float degPerRotation);
    bool homeEncoder();
    bool isHomed();
    void updatePosition();

    float getPosition();
    float getTotalAngle();
    float getVelocity();
    float getAcceleration();

    float getHomingPosition();
    void setHomingPosition(float homingPosition);

    float getDegPerRotation();


private:
    void checkQuadrant();

    float totalAngle;
    float rawAngle;
    float currentPosition;
    float oldPosition;
    float degPerRotation;

    int turns;
    float offset;
    bool homed;

    unsigned char quadrant;
    unsigned char previousQuadrant;

    float velocity;
    float acceleration;

    float homingPosition;

    uint8_t channelNumber;
    uint8_t encoderAddress;
    I2C_HandleTypeDef* i2c;
};

#endif //RAN2_MCU_CPP_MAGNETIC_ENCODER_HPP
