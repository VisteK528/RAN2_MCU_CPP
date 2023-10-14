#ifndef RAN2_MCU_CPP_MAGNETIC_ENCODER_HPP
#define RAN2_MCU_CPP_MAGNETIC_ENCODER_HPP

#include "as5600.hpp"
#include "errors.hpp"

/*  Operation_status information
 *
 *  Module codes used in this module:
 *  - Encoders (Numbers from 1 to 6)    0x0f - 0x14
 *
 *  Operation result codes:
 *  Result                                                  Code
 *  Operation ended successfully                            0x00
 *  Operation continue                                      0x01
 *
 *  Connection cannot be established                        0x02
 *  Magnet too weak                                         0x03
 *  Magnet too strong                                       0x04
 *  Encoder not homed, cannot update position               0x05
 *
 * */

typedef struct{
    float position;
    float rawPosition;
    float velocity;
    float acceleration;
} MagneticEncoderData;

typedef enum{EncoderCW, EncoderCCW} encoder_direction;

class MagneticEncoder{
public:
    MagneticEncoder(uint8_t encoderNumber, uint8_t encoderAddress, uint8_t channelNumber, I2C_HandleTypeDef* i2c, float homingPosition, float degPerRotation, encoder_direction direction);

    /** @brief Checks if the encoder is ready for the work.
     * @returns
     * One of 4 possible operation status codes. If everything is operational then method returns successful result
     * with operation code 0x00
     * */
    operation_status checkEncoder();

    /** @brief Checks if the raw position of the encoder is at homing position with given tolerance
     *  @attention Be mindful that the minimum tolerance (in degrees) must be at least equal to the value of one step
     *  eg. If the step value of the motor is 1.8 deg then the minimum tolerance is 1.8 deg.
     *  @returns Returns status of the homing procedure
     * */
    bool homeEncoder();

    /** @brief Returns true if the encoder is homed, otherwise returns false.
     * */
    bool isHomed() const;

    /** @brief Returns the direction in which encoder counts from 0 to 360 degrees
     * */
    encoder_direction getEncoderDirection() const;

    /** @brief Updates position values (current position, totalAngle) if sensor is homed
     *  @returns None
     * */
    operation_status updatePosition();

    /** @brief Updates raw position (raw angle) without offset or homing
     *  @returns None
     * */
    operation_status updateRawAngle();

    /** @brief Returns current position of the encoder(with offsets), always between 0 and 360 degrees.
     * */
    float getPosition() const;

    /** @brief Returns current raw position of the encoder(without offsets), always between 0 and 360 degrees.
     * */
    float getRawPosition() const;
    /** @brief Returns total angle of movement of the encoder, may be negative.
     * */
    float getTotalAngle() const;

    // TODO To be done along with the getAcceleration function
    /** @brief Returns current velocity measured by the encoder, units not selected yet.
     * */
    float getVelocity() const;
    /** @brief Returns current acceleration measured by the encoder, units not selected yet.
     * */
    float getAcceleration() const;

    /** @brief Returns current homing position angle related to the raw angle.
     * */
    float getHomingPosition() const;

    /** @brief Sets new homing position related to the raw angle.
     *  @param homingPosition New homing position related to the raw angle, given in degrees.
     *  @returns None
     * */
    void setHomingPosition(float homingPosition);

    /** @brief If the encoder is placed on the joint with gear ratio other than 1, then one rotation of the joint is not
     *  equal to one rotation measured by the encoder on the motor. Thus the degPerRotation variable was created,
     *  which means how many rotations measured by the encoder are equal to one rotation of the joint.
     *  This function returns this value.
     * */
    float getDegPerRotation() const;

    /** @brief Updates position and calculates velocity and acceleration based on previous velocity
     * and acceleration measurements. This method should be used in ISR and be called every 100ms in order to get
     * good measurements.
     *
     * @returns
     * Status of the operation
     * */
    operation_status updateParameters();


private:
    // Checks in which quadrant the encoder is currently. Then updates the total angle value.
    void checkQuadrant();

    volatile float totalAngle;
    volatile float oldTotalAngle;
    volatile float rawAngle;
    volatile float currentPosition;
    volatile float oldPosition;
    float degPerRotation;

    volatile int turns = 0;
    volatile float offset = 0;
    bool homed;

    volatile unsigned char quadrant;
    volatile unsigned char previousQuadrant;

    volatile float oldVelocity = 0;
    volatile float velocity = 0;
    volatile float acceleration = 0;

    float homingPosition = 0;
    encoder_direction direction;

    uint8_t encoderNumber;
    uint8_t channelNumber;
    uint8_t encoderAddress;
    I2C_HandleTypeDef* i2c;
};

#endif //RAN2_MCU_CPP_MAGNETIC_ENCODER_HPP
