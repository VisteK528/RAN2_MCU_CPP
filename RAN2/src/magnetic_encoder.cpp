#include <cmath>
#include "../include/magnetic_encoder.hpp"

static void selectI2CChannels(uint8_t i) {
    if (i > 7) return;
    unsigned char temp[1];
    temp[0] = 1 << i;

    //0x70 is address
    HAL_I2C_Master_Transmit(&hi2c1, 0x70<<1, temp, 1, 100);
}

static operation_status operation_status_init_encoder(uint8_t encoder_number, operation_result result, operation_result_code code){
    operation_module_code module;
    switch (encoder_number) {
        case 0:
            module = encoder1;
            break;
        case 1:
            module = encoder2;
            break;
        case 2:
            module = encoder3;
            break;
        case 3:
            module = encoder4;
            break;
        case 4:
            module = encoder5;
            break;
        case 5:
            module = encoder6;
            break;
        default:
            return operation_status_init(encoder1, failure, code);

    }
    return operation_status_init(module, result, code);
}

MagneticEncoder::MagneticEncoder(uint8_t encoderNumber, uint8_t encoderAddress, uint8_t channelNumber,
                                 I2C_HandleTypeDef *i2c, float homing_position, float degPerRotation) {
    this->encoderNumber = encoderNumber;
    this->encoderAddress = encoderAddress;
    this->channelNumber = channelNumber;
    this->i2c = i2c;
    this->homingPosition = homing_position;
    this->degPerRotation = degPerRotation;
}

operation_status MagneticEncoder::checkEncoder() {
    uint8_t status;
    status = as5600_init(i2c, encoderAddress);

    if(status == 1){
        return operation_status_init_encoder(encoderNumber, failure, 0x02);
    }
    else if(status == 2){
        return operation_status_init_encoder(encoderNumber, failure, 0x03);
    }
    else if(status == 3){
        return operation_status_init_encoder(encoderNumber, failure, 0x04);
    }
    return operation_status_init_encoder(encoderNumber, success, 0x00);
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

operation_status MagneticEncoder::updatePosition() {
    if(as5600_check_presence(i2c, encoderAddress) == 1){
        return operation_status_init_encoder(encoderNumber, failure, 0x02);
    }

   if(homed){
       selectI2CChannels(channelNumber);
       rawAngle = as5600_read_raw_angle(i2c, encoderAddress);

       oldPosition = currentPosition;
       currentPosition = rawAngle - offset;

       if(currentPosition < 0){
           currentPosition += 360;
       }
       checkQuadrant();
       return operation_status_init_encoder(encoderNumber, success, 0x00);
   }
    return operation_status_init_encoder(encoderNumber, failure, 0x05);
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

operation_status MagneticEncoder::updateParameters() {
    float delta_angle;
    float elapsedTime = 50.f/1000;           //Measurement approximately every 50ms
    operation_status status;
    status = updatePosition();

    if(status.result == failure){
        return status;
    }

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
    return operation_status_init_encoder(encoderNumber, success, 0x00);
}
