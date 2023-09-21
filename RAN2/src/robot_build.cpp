#include "../include/robot_build.hpp"
#include "../config.hpp"

static GPIO_PIN initPin(GPIO_TypeDef* gpio_port, uint16_t gpio_pin){
    GPIO_PIN pin;
    pin.gpio_port = gpio_port;
    pin.gpio_pin = gpio_pin;
    return pin;
}

Robot buildRobot(){
    // Waist
    GPIO_PIN waist_step = initPin(J1_STEP_GPIO_Port, J1_STEP_Pin);
    GPIO_PIN waist_dir = initPin(J1_DIR_GPIO_Port, J1_DIR_Pin);
    GPIO_PIN waist_en = initPin(J1_EN_GPIO_Port, J1_EN_Pin);
    GPIO_PIN waist_endstop_pin = initPin(J1_ENDSTOP_GPIO_Port, J1_ENDSTOP_Pin);


    std::unique_ptr<Driver> waist_driver = std::make_unique<drivers::Driver>(J1_JOINT_NUMBER, waist_step, waist_dir,
                                                                             waist_en, J1_MOTOR_SHAFT_TEETH,
                                                                             J1_MOTOR_RESOLUTION, J1_DRIVER_MICROSTEP);
    std::shared_ptr<Endstop> waist_endstop = std::make_shared<Endstop>(waist_endstop_pin, ENDSTOP_TYPE::UP);

    std::unique_ptr<Joint> waist_joint = std::make_unique<Joint>(J1_JOINT_NUMBER, waist_driver, J1_JOINT_TEETH,
                                                                 drivers::DIRECTION::ANTICLOCKWISE, waist_endstop);
#if J1_HOMING_DIRECTION == CCW
    waist_joint->setDirection(drivers::DIRECTION::ANTICLOCKWISE);
#elif J1_HOMING_DIRECTION == CW
    waist_joint->setDirection(drivers::DIRECTION::CLOCKWISE);
#endif

    waist_joint->setMinPosition(J1_MIN_POS);
    waist_joint->setMaxPosition(J1_MAX_POS);
    waist_joint->setOffset(J1_OFFSET);

    waist_joint->setHomingVelocity(J1_HOMING_VELOCITY);
    waist_joint->setHomingAcceleration(J1_HOMING_ACCELERATION);
    waist_joint->setHomingSteps(J1_HOMING_STEPS);

    waist_joint->setMaxVelocity(J1_MAX_VELOCITY);
    waist_joint->setMaxAcceleration(J1_MAX_ACCELERATION);

#ifdef J1_ENCODER_HOMING
    waist_joint->setEncoderHoming();
#endif

#ifdef J1_SMART_ENCODER_HOMING
    waist_joint->setSmartEncoderHoming();
#endif

    // Shoulder
    GPIO_PIN shoulder_step = initPin(J2_STEP_GPIO_Port, J2_STEP_Pin);
    GPIO_PIN shoulder_dir = initPin(J2_DIR_GPIO_Port, J2_DIR_Pin);
    GPIO_PIN shoulder_en = initPin(J2_EN_GPIO_Port, J2_EN_Pin);
    GPIO_PIN shoulder_endstop_pin = initPin(J2_ENDSTOP_GPIO_Port, J2_ENDSTOP_Pin);

    std::unique_ptr<Driver> shoulder_driver = std::make_unique<Driver>(J2_JOINT_NUMBER, shoulder_step, shoulder_dir,
                                                                       shoulder_en, J2_MOTOR_SHAFT_TEETH,
                                                                       J2_MOTOR_RESOLUTION, J2_DRIVER_MICROSTEP);
    std::shared_ptr<Endstop> shoulder_endstop = std::make_shared<Endstop>(shoulder_endstop_pin, ENDSTOP_TYPE::UP);

    std::unique_ptr<Joint> shoulder_joint = std::make_unique<Joint>(J2_JOINT_NUMBER, shoulder_driver, J2_JOINT_TEETH,
                                                                    drivers::DIRECTION::CLOCKWISE, shoulder_endstop);
#if J2_HOMING_DIRECTION == CCW
    shoulder_joint->setDirection(drivers::DIRECTION::ANTICLOCKWISE);
#elif J2_HOMING_DIRECTION == CW
    shoulder_joint->setDirection(drivers::DIRECTION::CLOCKWISE);
#endif

    shoulder_joint->setMinPosition(J2_MIN_POS);
    shoulder_joint->setMaxPosition(J2_MAX_POS);
    shoulder_joint->setOffset(J2_OFFSET);

    shoulder_joint->setHomingVelocity(J2_HOMING_VELOCITY);
    shoulder_joint->setHomingAcceleration(J2_HOMING_ACCELERATION);
    shoulder_joint->setHomingSteps(J2_HOMING_STEPS);

    shoulder_joint->setMaxVelocity(J2_MAX_VELOCITY);
    shoulder_joint->setMaxAcceleration(J2_MAX_ACCELERATION);

#ifdef J2_ENCODER_HOMING
    shoulder_joint->setEncoderHoming();
#endif

#ifdef J2_SMART_ENCODER_HOMING
    shoulder_joint->setSmartEncoderHoming();
#endif

    // Elbow
    GPIO_PIN elbow_step = initPin(J3_STEP_GPIO_Port, J3_STEP_Pin);
    GPIO_PIN elbow_dir = initPin(J3_DIR_GPIO_Port, J3_DIR_Pin);
    GPIO_PIN elbow_en = initPin(J3_EN_GPIO_Port, J3_EN_Pin);
    GPIO_PIN elbow_endstop_pin = initPin(J3_ENDSTOP_GPIO_Port, J3_ENDSTOP_Pin);

    std::unique_ptr<Driver> elbow_driver = std::make_unique<drivers::Driver>(J3_JOINT_NUMBER, elbow_step, elbow_dir,
                                                                             elbow_en, J3_MOTOR_SHAFT_TEETH,
                                                                             J3_MOTOR_RESOLUTION, J3_DRIVER_MICROSTEP);
    std::shared_ptr<Endstop> elbow_endstop = std::make_shared<Endstop>(elbow_endstop_pin, ENDSTOP_TYPE::UP);

    std::unique_ptr<Joint> elbow_joint = std::make_unique<Joint>(J3_JOINT_NUMBER, elbow_driver, J3_JOINT_TEETH,
                                                                 drivers::DIRECTION::ANTICLOCKWISE, elbow_endstop);

#if J3_HOMING_DIRECTION == CCW
    elbow_joint->setDirection(drivers::DIRECTION::ANTICLOCKWISE);
#elif J3_HOMING_DIRECTION == CW
    elbow_joint->setDirection(drivers::DIRECTION::CLOCKWISE);
#endif

    elbow_joint->setMinPosition(J3_MIN_POS);
    elbow_joint->setMaxPosition(J3_MAX_POS);
    elbow_joint->setOffset(J3_OFFSET);

    elbow_joint->setHomingVelocity(J3_HOMING_VELOCITY);
    elbow_joint->setHomingAcceleration(J3_HOMING_ACCELERATION);
    elbow_joint->setHomingSteps(J3_HOMING_STEPS);

    elbow_joint->setMaxVelocity(J3_MAX_VELOCITY);
    elbow_joint->setMaxAcceleration(J3_MAX_ACCELERATION);

#ifdef J3_ENCODER_HOMING
    elbow_joint->setEncoderHoming();
#endif

#ifdef J3_SMART_ENCODER_HOMING
    elbow_joint->setSmartEncoderHoming();
#endif

    // Elbow roll
    GPIO_PIN elbow_roll_step = initPin(J4_STEP_GPIO_Port, J4_STEP_Pin);
    GPIO_PIN elbow_roll_dir = initPin(J4_DIR_GPIO_Port, J4_DIR_Pin);
    GPIO_PIN elbow_roll_en = initPin(J4_EN_GPIO_Port, J4_EN_Pin);
    GPIO_PIN elbow_roll_endstop_pin = initPin(J4_ENDSTOP_GPIO_Port, J4_ENDSTOP_Pin);

    std::shared_ptr<MagneticEncoder> elbow_roll_encoder = std::make_shared<MagneticEncoder>(J4_JOINT_NUMBER,
                                                                                            J4_ENCODER_ADDRESS,
                                                                                            J4_ENCODER_CHANNEL_NUMBER,
                                                                                            &hi2c1,
                                                                                            J4_ENCODER_HOMING_POSITION,
                                                                                            J4_ENCODER_DEG_PER_ROTATION);

    std::unique_ptr<Driver> elbow_roll_driver = std::make_unique<drivers::Driver>(J4_JOINT_NUMBER, elbow_roll_step,
                                                                                  elbow_roll_dir, elbow_roll_en,
                                                                                  J4_MOTOR_SHAFT_TEETH,
                                                                                  J4_MOTOR_RESOLUTION,
                                                                                  J4_DRIVER_MICROSTEP);
    std::shared_ptr<Endstop> elbow_roll_endstop = std::make_shared<Endstop>(elbow_roll_endstop_pin, ENDSTOP_TYPE::UP);


    std::unique_ptr<Joint> elbow_roll_joint = std::make_unique<Joint>(J4_JOINT_NUMBER, elbow_roll_driver,
                                                                      J4_JOINT_TEETH, drivers::DIRECTION::ANTICLOCKWISE,
                                                                      elbow_roll_endstop, elbow_roll_encoder);

#if J4_HOMING_DIRECTION == CCW
    elbow_roll_joint->setDirection(drivers::DIRECTION::ANTICLOCKWISE);
#elif J4_HOMING_DIRECTION == CW
    elbow_roll_joint->setDirection(drivers::DIRECTION::CLOCKWISE);
#endif

    elbow_roll_joint->setMinPosition(J4_MIN_POS);
    elbow_roll_joint->setMaxPosition(J4_MAX_POS);
    elbow_roll_joint->setOffset(J4_OFFSET);

    elbow_roll_joint->setHomingVelocity(J4_HOMING_VELOCITY);
    elbow_roll_joint->setHomingAcceleration(J4_HOMING_ACCELERATION);
    elbow_roll_joint->setHomingSteps(J4_HOMING_STEPS);

    elbow_roll_joint->setMaxVelocity(J4_MAX_VELOCITY);
    elbow_roll_joint->setMaxAcceleration(J4_MAX_ACCELERATION);

#ifdef J4_ENCODER_HOMING
    elbow_roll_joint->setEncoderHoming();
#endif

#ifdef J4_SMART_ENCODER_HOMING
    elbow_roll_joint->setSmartEncoderHoming();
#endif

    // Pitch
    GPIO_PIN pitch_step = initPin(J5_STEP_GPIO_Port, J5_STEP_Pin);
    GPIO_PIN pitch_dir = initPin(J5_DIR_GPIO_Port, J5_DIR_Pin);
    GPIO_PIN pitch_en = initPin(J5_EN_GPIO_Port, J5_EN_Pin);
    GPIO_PIN pitch_endstop_pin = initPin(J5_ENDSTOP_GPIO_Port, J5_ENDSTOP_Pin);

    std::unique_ptr<Driver> wrist_pitch_driver = std::make_unique<drivers::Driver>(J5_JOINT_NUMBER, pitch_step,
                                                                                   pitch_dir, pitch_en,
                                                                                   J5_MOTOR_SHAFT_TEETH,
                                                                                   J5_MOTOR_RESOLUTION,
                                                                                   J5_DRIVER_MICROSTEP);
    std::shared_ptr<Endstop> wrist_pitch_endstop = std::make_shared<Endstop>(pitch_endstop_pin, ENDSTOP_TYPE::UP);

    std::unique_ptr<Joint> wrist_pitch_joint = std::make_unique<Joint>(J5_JOINT_NUMBER, wrist_pitch_driver,
                                                                       J5_JOINT_TEETH,
                                                                       drivers::DIRECTION::ANTICLOCKWISE,
                                                                       wrist_pitch_endstop);

#if J5_HOMING_DIRECTION == CCW
    wrist_pitch_joint->setDirection(drivers::DIRECTION::ANTICLOCKWISE);
#elif J5_HOMING_DIRECTION == CW
    wrist_pitch_joint->setDirection(drivers::DIRECTION::CLOCKWISE);
#endif

    wrist_pitch_joint->setMinPosition(J5_MIN_POS);
    wrist_pitch_joint->setMaxPosition(J5_MAX_POS);
    wrist_pitch_joint->setOffset(J5_OFFSET);

    wrist_pitch_joint->setHomingVelocity(J5_HOMING_VELOCITY);
    wrist_pitch_joint->setHomingAcceleration(J5_HOMING_ACCELERATION);
    wrist_pitch_joint->setHomingSteps(J5_HOMING_STEPS);

    wrist_pitch_joint->setMaxVelocity(J5_MAX_VELOCITY);
    wrist_pitch_joint->setMaxAcceleration(J5_MAX_ACCELERATION);

#ifdef J5_ENCODER_HOMING
    wrist_pitch_joint->setEncoderHoming();
#endif

#ifdef J5_SMART_ENCODER_HOMING
    wrist_pitch_joint->setSmartEncoderHoming();
#endif

    // Wrist roll
    GPIO_PIN wrist_roll_step = initPin(J6_STEP_GPIO_Port, J6_STEP_Pin);
    GPIO_PIN wrist_roll_dir = initPin(J6_DIR_GPIO_Port, J6_DIR_Pin);
    GPIO_PIN wrist_roll_en = initPin(J6_EN_GPIO_Port, J6_EN_Pin);

    std::shared_ptr<MagneticEncoder> wrist_roll_encoder = std::make_shared<MagneticEncoder>(J6_JOINT_NUMBER,
                                                                                            J6_ENCODER_ADDRESS,
                                                                                            J6_ENCODER_CHANNEL_NUMBER,
                                                                                            &hi2c1,
                                                                                            J6_ENCODER_HOMING_POSITION,
                                                                                            J6_ENCODER_DEG_PER_ROTATION);

    std::unique_ptr<Driver> wrist_roll_driver = std::make_unique<drivers::Driver>(J6_JOINT_NUMBER, wrist_roll_step,
                                                                                  wrist_roll_dir, wrist_roll_en,
                                                                                  J6_MOTOR_SHAFT_TEETH,
                                                                                  J6_MOTOR_RESOLUTION,
                                                                                  J6_DRIVER_MICROSTEP);

    std::unique_ptr<Joint> wrist_roll_joint = std::make_unique<Joint>(J6_JOINT_NUMBER, wrist_roll_driver, J6_JOINT_TEETH,
                                                                      drivers::DIRECTION::ANTICLOCKWISE,
                                                                      nullptr, wrist_roll_encoder);

#if J6_HOMING_DIRECTION == CCW
    wrist_roll_joint->setDirection(drivers::DIRECTION::ANTICLOCKWISE);
#elif J6_HOMING_DIRECTION == CW
    wrist_roll_joint->setDirection(drivers::DIRECTION::CLOCKWISE);
#endif

    wrist_roll_joint->setMinPosition(J6_MIN_POS);
    wrist_roll_joint->setMaxPosition(J6_MAX_POS);
    wrist_roll_joint->setOffset(J6_OFFSET);

    wrist_roll_joint->setHomingVelocity(J6_HOMING_VELOCITY);
    wrist_roll_joint->setHomingAcceleration(J6_HOMING_ACCELERATION);
    wrist_roll_joint->setHomingSteps(J6_HOMING_STEPS);

    wrist_roll_joint->setMaxVelocity(J6_MAX_VELOCITY);
    wrist_roll_joint->setMaxAcceleration(J6_MAX_ACCELERATION);

#ifdef J6_ENCODER_HOMING
    wrist_roll_joint->setEncoderHoming();
#endif

#ifdef J6_SMART_ENCODER_HOMING
    wrist_roll_joint->setSmartEncoderHoming();
#endif


    std::vector<std::unique_ptr<Joint>> joints;
    joints.push_back(std::move(waist_joint));
    joints.push_back(std::move(shoulder_joint));
    joints.push_back(std::move(elbow_joint));
    joints.push_back(std::move(elbow_roll_joint));
    joints.push_back(std::move(wrist_pitch_joint));
    joints.push_back(std::move(wrist_roll_joint));

    GPIO_PIN safeguard_pin = initPin(SAFEGUARD_PIN_GPIO_Port, SAFEGUARD_PIN_Pin);

    Robot robot(joints, safeguard_pin);
    return robot;
}
