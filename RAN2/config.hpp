#ifndef RAN2_MCU_CPP_CONFIG_HPP
#define RAN2_MCU_CPP_CONFIG_HPP

#define CW  0
#define CCW 1

#define DOF     6
#define GRIPPER

#define J1
#define J2
#define J3
#define J4
#define J5
#define J6

#ifdef J1
// Main settings
#define J1_JOINT_NUMBER                 0
#define J1_MOTOR_SHAFT_TEETH            20
#define J1_MOTOR_RESOLUTION             0.9f
#define J1_DRIVER_MICROSTEP             8
#define J1_JOINT_TEETH                  125
#define J1_HOMING_DIRECTION             CCW

#define J1_MIN_POS                      0.f
#define J1_MAX_POS                      350.f
#define J1_OFFSET                       0.f

#define J1_HOMING_VELOCITY              0.16f
#define J1_HOMING_ACCELERATION          0.2f
#define J1_HOMING_STEPS                 200

#define J1_MAX_VELOCITY                 1.6f
#define J1_MAX_ACCELERATION             1.5f

// Additional settings
//#define J1_ENCODER

#ifdef J1_ENCODER
    #define J1_ENCODER_ADDRESS          0x36
    #define J1_ENCODER_DIRECTION        CW
    #define J1_ENCODER_CHANNEL_NUMBER   0
    #define J1_ENCODER_HOMING_POSITION  0.f
    #define J1_ENCODER_DEG_PER_ROTATION 1.f
    
    //#define J1_ENCODER_HOMING
    //#define J1_SMART_ENCODER_HOMING
#endif

#endif

#ifdef J2
// Main settings
#define J2_JOINT_NUMBER                 1
#define J2_MOTOR_SHAFT_TEETH            20
#define J2_MOTOR_RESOLUTION             1.8f
#define J2_DRIVER_MICROSTEP             8
#define J2_JOINT_TEETH                  149
#define J2_HOMING_DIRECTION             CW

#define J2_MIN_POS                      0.f
#define J2_MAX_POS                      171.f
#define J2_OFFSET                       10.f

#define J2_HOMING_VELOCITY              0.16f
#define J2_HOMING_ACCELERATION          0.5f
#define J2_HOMING_STEPS                 200

#define J2_MAX_VELOCITY                 0.32f
#define J2_MAX_ACCELERATION             0.2f

// Additional settings
//#define J2_ENCODER

#ifdef J2_ENCODER
    #define J2_ENCODER_ADDRESS          0x36
    #define J2_ENCODER_DIRECTION        CW
    #define J2_ENCODER_CHANNEL_NUMBER   0
    #define J2_ENCODER_HOMING_POSITION  0.f
    #define J2_ENCODER_DEG_PER_ROTATION 1.f

    //#define J2_ENCODER_HOMING
    //#define J2_SMART_ENCODER_HOMING
#endif

#endif

#ifdef J3
// Main settings
#define J3_JOINT_NUMBER                 2
#define J3_MOTOR_SHAFT_TEETH            20
#define J3_MOTOR_RESOLUTION             0.9f
#define J3_DRIVER_MICROSTEP             8
#define J3_JOINT_TEETH                  62
#define J3_HOMING_DIRECTION             CCW

#define J3_MIN_POS                      0.f
#define J3_MAX_POS                      70.f
#define J3_OFFSET                       0.f

#define J3_HOMING_VELOCITY              0.25f
#define J3_HOMING_ACCELERATION          0.5f
#define J3_HOMING_STEPS                 200

#define J3_MAX_VELOCITY                 1.6f
#define J3_MAX_ACCELERATION             3.f

// Additional settings
//#define J3_ENCODER

#ifdef J3_ENCODER
    #define J3_ENCODER_ADDRESS          0x36
    #define J3_ENCODER_DIRECTION        CW
    #define J3_ENCODER_CHANNEL_NUMBER   0
    #define J3_ENCODER_HOMING_POSITION  0.f
    #define J3_ENCODER_DEG_PER_ROTATION 1.f

    //#define J3_ENCODER_HOMING
    //#define J3_SMART_ENCODER_HOMING
#endif

#endif

#ifdef J4
// Main settings
#define J4_JOINT_NUMBER                 3
#define J4_MOTOR_SHAFT_TEETH            1
#define J4_MOTOR_RESOLUTION             1.8f
#define J4_DRIVER_MICROSTEP             8
#define J4_JOINT_TEETH                  1
#define J4_HOMING_DIRECTION             CCW

#define J4_MIN_POS                      (-180.f)
#define J4_MAX_POS                      180.f
#define J4_OFFSET                       0.f

#define J4_HOMING_VELOCITY              0.25f
#define J4_HOMING_ACCELERATION          0.5f
#define J4_HOMING_STEPS                 100

#define J4_MAX_VELOCITY                 1.5f
#define J4_MAX_ACCELERATION             2.f

// Additional settings
#define J4_ENCODER

#ifdef J4_ENCODER
    #define J4_ENCODER_ADDRESS          0x36
    #define J4_ENCODER_DIRECTION        CCW
    #define J4_ENCODER_CHANNEL_NUMBER   3
    #define J4_ENCODER_HOMING_POSITION  120.f
    #define J4_ENCODER_DEG_PER_ROTATION 1.f

    #define J4_ENCODER_HOMING
    #define J4_SMART_ENCODER_HOMING
#endif

#endif

#ifdef J5
// Main settings
#define J5_JOINT_NUMBER                 4
#define J5_MOTOR_SHAFT_TEETH            20
#define J5_MOTOR_RESOLUTION             1.8f
#define J5_DRIVER_MICROSTEP             8
#define J5_JOINT_TEETH                  40
#define J5_HOMING_DIRECTION             CCW

#define J5_MIN_POS                      0.f
#define J5_MAX_POS                      250.f
#define J5_OFFSET                       0.f

#define J5_HOMING_VELOCITY              0.25f
#define J5_HOMING_ACCELERATION          0.5f
#define J5_HOMING_STEPS                 100

#define J5_MAX_VELOCITY                 3.f
#define J5_MAX_ACCELERATION             4.f

// Additional settings
//#define J5_ENCODER

#ifdef J5_ENCODER
    #define J5_ENCODER_ADDRESS          0x36
    #define J5_ENCODER_DIRECTION        CW
    #define J5_ENCODER_CHANNEL_NUMBER   0
    #define J5_ENCODER_HOMING_POSITION  0.f
    #define J5_ENCODER_DEG_PER_ROTATION 1.f

    //#define J5_ENCODER_HOMING
    //#define J5_SMART_ENCODER_HOMING
#endif

#endif

#ifdef J6
// Main settings
#define J6_JOINT_NUMBER                 5
#define J6_MOTOR_SHAFT_TEETH            1
#define J6_MOTOR_RESOLUTION             1.8f
#define J6_DRIVER_MICROSTEP             8
#define J6_JOINT_TEETH                  1
#define J6_HOMING_DIRECTION             CCW

#define J6_MIN_POS                      (-180.f)
#define J6_MAX_POS                      180.f
#define J6_OFFSET                       0.f

#define J6_HOMING_VELOCITY              0.5f
#define J6_HOMING_ACCELERATION          1.f
#define J6_HOMING_STEPS                 200

#define J6_MAX_VELOCITY                 3.f
#define J6_MAX_ACCELERATION             4.f

// Additional settings
#define J6_ENCODER

#ifdef J6_ENCODER
    #define J6_ENCODER_ADDRESS          0x36
    #define J6_ENCODER_DIRECTION        CW
    #define J6_ENCODER_CHANNEL_NUMBER   2
    #define J6_ENCODER_HOMING_POSITION  28.f
    #define J6_ENCODER_DEG_PER_ROTATION 1.f

    #define J6_ENCODER_HOMING
    //#define J6_SMART_ENCODER_HOMING
#endif

#endif









#endif //RAN2_MCU_CPP_CONFIG_HPP
