#include <cmath>
#include "../include/magnetic_encoder.hpp"

static void selectI2CChannels(uint8_t i) {
    if (i > 7) return;
    unsigned char temp[1];
    temp[0] = 1 << i;

    //0x70 is address
    HAL_I2C_Master_Transmit(&hi2c1, 0x70<<1, temp, 1, 100);
}

MagneticEncoder::MagneticEncoder(uint8_t encoderAddress, uint8_t channelNumber, I2C_HandleTypeDef *i2c,
                                 float homing_position, float degPerRotation) {
    this->encoderAddress = encoderAddress;
    this->channelNumber = channelNumber;
    this->i2c = i2c;
    this->homingPosition = homing_position;
    this->degPerRotation = degPerRotation;
}

void MagneticEncoder::checkQuadrant() {
    if(rawAngle >= 0 && rawAngle < 90){
        quadrant = 1;
    }
    else if(rawAngle >= 90 && rawAngle < 180){
        quadrant = 2;
    }
    else if(rawAngle >= 180 && rawAngle < 270){
        quadrant = 3;
    }
    else if(rawAngle >= 270 && rawAngle < 360){
        quadrant = 4;
    }

    if(quadrant != previousQuadrant){
        if(quadrant == 1 && previousQuadrant == 4){
            turns++;
        }
        else if(quadrant == 4 && previousQuadrant == 1){
            turns--;
        }
        previousQuadrant = quadrant;
    }
    oldTotalAngle = totalAngle;
    totalAngle = (float)turns*360.f + rawAngle;
}

bool MagneticEncoder::homeEncoder() {
    selectI2CChannels(channelNumber);

    rawAngle = as5600_read_raw_angle(i2c, encoderAddress);

    if(rawAngle > homingPosition + 0.5 || rawAngle < homingPosition - 0.5){
        homed = false;
        return false;
    }
    currentPosition = 0;
    offset = rawAngle;
    rawAngle = 0;
    homed = true;
    return true;
}

bool MagneticEncoder::isHomed() {
    return this->homed;
}

void MagneticEncoder::updatePosition() {
   if(homed){
       selectI2CChannels(channelNumber);
       rawAngle = as5600_read_raw_angle(i2c, encoderAddress);

       oldPosition = currentPosition;
       currentPosition = rawAngle - offset;

       if(currentPosition < 0){
           currentPosition += 360;
       }
       checkQuadrant();
   }
}

float MagneticEncoder::getPosition() {
    return this->currentPosition;
}

float MagneticEncoder::getTotalAngle() {
    return this->totalAngle;
}

float MagneticEncoder::getVelocity() {
    return this->velocity;
}

float MagneticEncoder::getAcceleration() {
    return this->acceleration;
}

float MagneticEncoder::getHomingPosition() {
    return this->homingPosition;
}

void MagneticEncoder::setHomingPosition(float position) {
    this->homingPosition = position;
    homed = false;
}

float MagneticEncoder::getDegPerRotation() {
    return this->degPerRotation;
}

void MagneticEncoder::updateParameters() {
    float delta_angle;
    float elapsedTime = 50.f/1000;           //Measurement approximately every 50ms
    updatePosition();

    if(totalAngle > 0 && oldTotalAngle > 0){
        delta_angle = totalAngle - oldTotalAngle;
    }
    else if(totalAngle < 0 && oldTotalAngle < 0){
        delta_angle = -1*oldTotalAngle - (-1)*totalAngle;
    }
    else{
        delta_angle = totalAngle + oldTotalAngle;
    }


    oldVelocity = velocity;
    this->velocity = delta_angle/elapsedTime;
    this->acceleration = (velocity - oldVelocity)/elapsedTime;
}
