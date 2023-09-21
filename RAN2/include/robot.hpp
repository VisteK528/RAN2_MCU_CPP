#ifndef ROBOTARMNUMBER2CPP_ROBOT_HPP
#define ROBOTARMNUMBER2CPP_ROBOT_HPP

#include "joint.hpp"
#include <memory>
#include <unordered_map>
#include "gpio.h"
#include "algorithm6dof.hpp"
#include "read_gcode.h"
#include "utilities.hpp"
#include "errors.hpp"
#include "gripper.hpp"

/*  Operation_status information (Robot)
 *
 *  Module codes used in this module:
 *  - Robot                                                                             0x00
 *
 *  Operation result codes:
 *  Result                                                                              Code
 *  Operation ended successfully                                                        0x00
 *  Operation continue                                                                  0x01
 *
 *  Systems check not done                                                              0x02
 *  Safeguard button triggered or not connected                                         0x03
 *
 ** */

/*  Operation_status information (GCODE Reader)
 *
 *  Module codes used in this module:
 *  - GCODE Reader                                                                      0x02
 *
 *  Operation result codes:
 *  Result                                                                              Code
 *  Operation ended successfully                                                        0x00
 *  Operation continue                                                                  0x01
 *
 *  GCODE command cannot be decoded                                                     0x02
 ** */


typedef enum {
    millimeters,
    inches
} LENGTH_UNITS;

typedef enum{
    degrees,
    radians
} ANGLE_UNITS;

class Robot{
public:
    Robot(std::vector<std::unique_ptr<Joint>>& joints, GPIO_PIN safeguard_pin);
    Robot()=default;

    operation_status home();
    operation_status move2Default();
    operation_status moveJoint(uint8_t joint_number, float position, bool blocking=true);
    operation_status moveJoints(float* angles);
    operation_status move2Coordinates(float x, float y, float z, float yaw, float pitch, float roll);
    operation_status homeJoint(uint8_t joint_number);
    operation_status wait(uint32_t seconds);
    operation_status disableJoint(uint8_t joint_number);
    operation_status disableJoints();
    operation_status enableJoint(uint8_t joint_number);
    operation_status enableJoints();

    // Units set / get functions
    operation_status setLengthUnits(LENGTH_UNITS units);
    LENGTH_UNITS getLengthUnits();
    operation_status setAngleUnits(ANGLE_UNITS units);
    ANGLE_UNITS getAngleUnits();

    operation_status updateEncoders();
    operation_status getEncoderData(uint8_t joint_number, MagneticEncoderData* data);
    operation_status getJointPosition(uint8_t joint_number, float* position);
    void getJointAngles(float* angles);
    void getRobotArmCoordinates(coordinates* coordinates);

    operation_status systemsCheck(bool initial);
    operation_status getSystemsStatus();
    bool getMovement();

    // Gripper
    operation_status setGripperClosedPercentage(float percentage);
    operation_status gripperSmartClose();
    operation_status gripperOpen();
    float readGripperRawPosition();
    operation_status enableGripper();
    operation_status disableGripper();

    operation_status safeguardTrigger();
    operation_status safeguardReset();

private:
    // Kinematics algorithms related variables
    std::unique_ptr<Algorithm6Dof> k_algorithms;
    std::unique_ptr<effector::Gripper> gripper;

    // Link map - ultimately stored in EEPROM / FLASH memory on the robot board, for now in software
    LINK_MAP map =  {
            {BASE_HEIGHT, 3.f},
            {SHOULDER_HEIGHT, 9.f},
            {SHOULDER_LENGTH, 20.76355f},
            {ELBOW_LENGTH, 16.50985f},
            {EE_LENGTH, 8},
    };

    // Offsets - ultimately stored in EEPROM / FLASH memory on the robot board, for now in software
    //TODO Complete the offsets values
    float joint_offsets[6] = {0, 0, 0, 0, 0, 0};
    float joint_angles[6] = {0, 0, 0, 0, 0, 0};

    coordinates robot_arm_points[6] = {{0, 0, 0},
                                       {0, 0, 0},
                                       {0, 0, 0},
                                       {0, 0, 0},
                                       {0, 0, 0},
                                       {0, 0, 0}};

    LENGTH_UNITS lengthUnits = millimeters;
    ANGLE_UNITS angleUnits = degrees;

    // Movement & Joint variables
    std::unordered_map<int, std::unique_ptr<Joint>> joints;
    bool homed = false;

    // Safety and status variables
    operation_status systems_status;
    bool safeguard_stop = false;
    GPIO_PIN safeguard_pin;
};

operation_status executeGCODE(Robot& robot, const char* command_str);

#endif //ROBOTARMNUMBER2CPP_ROBOT_HPP
