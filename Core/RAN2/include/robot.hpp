#ifndef ROBOTARMNUMBER2CPP_ROBOT_HPP
#define ROBOTARMNUMBER2CPP_ROBOT_HPP

#include "joint.hpp"
#include <memory>
#include <unordered_map>
#include "../../Inc/gpio.h"
#include "algorithm6dof.hpp"
#include "read_gcode.h"
#include "utilities.hpp"

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
    Robot(std::vector<std::unique_ptr<Joint>>& joints);
    Robot()=default;

    void home();
    void move2Default();
    void moveJoint(uint8_t joint_number, float position, bool blocking=true);
    void moveJoints(float* angles);
    void move2Coordinates(float x, float y, float z, float yaw, float pitch, float roll);
    void homeJoint(uint8_t joint_number);
    void wait(uint32_t seconds);
    void disableJoint(uint8_t joint_number);
    void disableJoints();
    void enableJoint(uint8_t joint_number);
    void enableJoints();

    // Units set / get functions
    void setLengthUnits(LENGTH_UNITS units);
    LENGTH_UNITS getLengthUnits();
    void setAngleUnits(ANGLE_UNITS units);
    ANGLE_UNITS getAngleUnits();

private:
    // Kinematics algorithms related variables
    std::unique_ptr<Algorithm6Dof> k_algorithms;

    // Link map - ultimately stored in EEPROM / FLASH memory on the robot board, for now in software
    LINK_MAP map =  {
            {BASE_HEIGHT, 3.f},
            {SHOULDER_HEIGHT, 9.f},
            {SHOULDER_LENGTH, 20.76355f},
            {ELBOW_LENGTH, 16.50985f},
            {EE_LENGTH, 5},
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
};

Robot buildRobot();
void executeGCODE(Robot& robot, const char* command_str);

#endif //ROBOTARMNUMBER2CPP_ROBOT_HPP
