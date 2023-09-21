#ifndef RAN2_MCU_CPP_JOINT_HPP
#define RAN2_MCU_CPP_JOINT_HPP

#include "driver.hpp"
#include "sensors.hpp"
#include <iostream>
#include <memory>
#include "gpio.h"
#include "magnetic_encoder.hpp"

/*  Operation_status information
 *
 *  Module codes used in this module:
 *  - Joints (Numbers from 1 to 6)    0x09 - 0x0e
 *
 *  Operation result codes:
 *  Result                                                                              Code
 *  Operation ended successfully                                                        0x00
 *  Operation continue                                                                  0x01
 *
 *  Encoder present, but degPerRotation is not equal to 1                               0x02
 *  Endstop and encoder not present, homing impossible                                  0x03
 *  Endstop not present, homing by endstop impossible                                   0x04
 *  Encoder not present                                                                 0x05
 *
 *  Movement not possible, joint is not homed                                           0x06
 *  Homing not possible, joint motor is disabled                                        0x07
 *  Setting smart encoder homing not possible, encoder not available                    0x08
 *  Disabling smart encoder homing not possible, smart encoder homing hasn't been set   0x09
 *  Movement not possible, goal position is not within movement range                   0x0a
 *  Safeguard stop triggered                                                            0x0b
 ** */

using namespace drivers;

typedef enum {
    endstop,
    encoder
} HOMING_TYPE;

class Joint{
public:
    Joint(uint8_t number, std::unique_ptr<Driver>& driver, uint16_t gear_teeth, DIRECTION homing_direction, std::shared_ptr<Endstop> sensor = nullptr, std::shared_ptr<MagneticEncoder> encoder = nullptr);

    // General methods

    /*!
     * Emergency function, instantaneously stops the joint and sets safeguard_stop variable to true.
     * \returns
     * Status of the operation.
     * */
    operation_status stopJoint();

    /*!
     * Sets the safeguard_stop variable to False, thus enabling possible movement of the joint in the future.
     * \returns
     * Status of the operation.
     * */
    operation_status startJoint();

    /*!
     * Performs quick check of the subsystems of the joint and their readiness. If every system works properly
     * method sets the joint status result to success. Otherwise the result is set to failure.
     * \returns
     * Joint status.
     * */
    operation_status updateJointStatus();

    /*!
     * Joint status getter
     * \returns
     * Joint status.
     * */
    operation_status getJointStatus();

    /*!
     * Checks if the joint is currently moving and then, if the condition is satisfied, returns true.
     * Otherwise returns false.
     * */
    bool isMoving();


    operation_status move2Pos(float position, bool blocking);
    operation_status homeJoint();
    operation_status getJointPosition(float* position);
    operation_status isMovementPossible(float position) const;

    // Setters
    void setHomingSteps(uint16_t homing_steps);
    void setHomingVelocity(float homing_velocity);
    void setHomingAcceleration(float homing_acceleration);
    void setMaxVelocity(float max_velocity);
    void setMaxAcceleration(float max_acceleration);
    void setMinPosition(float min_position);
    void setMaxPosition(float max_position);
    void setOffset(float offset);
    void setDirection(DIRECTION direction);

    // Movement methods
    float getMinDelay(float max_speed);
    float getStartDelay(float acceleration) const;
    float motorVel(float phase_time);
    float phaseTime(float joint_ang_vel);
    float jointVelFromMotorVel(float motor_ang_vel);
    float motorVelFromJointVel(float joint_ang_vel);
    float getGearSpeedRatio() const;

    // Driver / Motor methods
    operation_status disableMotor();
    operation_status enableMotor();
    bool isMotorEnabled();

    // Encoder methods
    bool encoderAvailable();
    operation_status setEncoderHoming();
    operation_status setSmartEncoderHoming();
    operation_status disableSmartEncoderHoming();
    operation_status updateEncoder();
    operation_status getEncoderData(MagneticEncoderData* data);

    // Endstop methods
    operation_status setEndstopHoming();

private:
    unsigned int degrees2Steps(float degrees);
    void accelerateJoint(DIRECTION direction, float velocity, float acceleration);
    void moveJoint(DIRECTION direction, float velocity);
    void moveJointBySteps(unsigned int steps, DIRECTION direction, float max_velocity, float max_acceleration, bool blocking);
    std::unique_ptr<Driver> driver;
    std::shared_ptr<Endstop> endstop;
    std::shared_ptr<MagneticEncoder> encoder;

    // Variables
    float min_pos = 0;
    float max_pos = 360;
    float joint_position = 0;
    float offset = 0;

    uint8_t joint_number;

    float motor_step;
    uint16_t driver_microstep;
    float one_pulse_step;

    uint16_t motor_shaft_gear_teeth;

    // Gear ratios
    float speed_gear_ratio;
    float torque_gear_ratio;

    // Homing Variables
    DIRECTION homing_direction;
    float homing_velocity = 0.25;
    float homing_acceleration = 0.5;
    unsigned int homing_steps = 200;

    // Movement variables
    float max_velocity = 0.7;
    float max_acceleration = 0.75;

    uint16_t gear_teeth;
    bool homed = false;
    HOMING_TYPE homing_type = HOMING_TYPE::endstop;
    bool smart_encoder_homing = false;
    bool safeguard_stop = false;

    operation_status joint_status;
};

#endif // RAN2_MCU_CPP_DRIVER_HPP