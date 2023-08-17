#ifndef RAN2_MCU_CPP_ERRORS_HPP
#define RAN2_MCU_CPP_ERRORS_HPP

#include <stdint.h>

/*  Modules codes
 *  Some modules codes may be not used at the moment, but will be used in the future
 *
 *  Robot                               0x00
 *  KinematicsAlgorithm                 0x01
 *  GCODE Reader                        0x02
 *  Drivers (From 1 to 6)               0x03 - 0x08
 *  Joints (From 1 to 6)                0x09 - 0x0e
 *  Encoders (From 1 to 6)              0x0f - 0x14
 *  End Effector                        0x15
 *
 * */

typedef enum{
    robot=0x00,
    kinematics_algorithm=0x01,
    gcode_reader=0x02,
    driver1=0x03, driver2=0x04, driver3=0x05, driver4=0x06, driver5=0x07, driver6=0x08,
    joint1=0x09, joint2=0x0a, joint3=0x0b, joint4=0x0c, joint5=0x0d, joint6=0x0e,
    encoder1=0x0f, encoder2=0x10, encoder3=0x11, encoder4=0x12, encoder5=0x13, encoder6=0x14,
    end_effector=0x15
} operation_module_code;

typedef enum{
    success,
    failure
} operation_result;

typedef uint16_t operation_result_code;

typedef struct{
    operation_module_code module;
    operation_result result;
    operation_result_code code;

} operation_status;

operation_status operation_status_init(operation_module_code module, operation_result result, operation_result_code code);

#endif //RAN2_MCU_CPP_ERRORS_HPP
