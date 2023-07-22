#include "../include/robot.hpp"


Robot::Robot(std::vector<std::unique_ptr<Joint>>& joints) {

    this->joints.reserve(joints.size());

    for(int i = 0; i < joints.size(); i++){
        this->joints[i] = std::move(joints[i]);
    }
}

void Robot::home() {

    for(const auto & joint : joints){
        printf("Homing joint...\n");
        joint.second->homeJoint();
    }
    homed = true;
}

void Robot::homeJoint(int joint_number) {
    joints[joint_number]->homeJoint();
}

void Robot::wait(uint32_t ms) {
    HAL_Delay(ms);
}

Robot buildRobot(){
    // Waist
    GPIO_PIN waist_step, waist_dir, waist_en, waist_endstop_pin;
    waist_step.gpio_port = J1_STEP_GPIO_Port;
    waist_step.gpio_pin = J1_STEP_Pin;
    waist_dir.gpio_port = J1_DIR_GPIO_Port;
    waist_dir.gpio_pin = J1_DIR_Pin;
    waist_en.gpio_port = J1_EN_GPIO_Port;
    waist_en.gpio_pin = J1_EN_Pin;

    waist_endstop_pin.gpio_port = J1_ENDSTOP_GPIO_Port;
    waist_endstop_pin.gpio_pin = J1_ENDSTOP_Pin;

    std::unique_ptr<Driver> waist_driver = std::make_unique<drivers::TMC2209>(0, waist_step, waist_dir, waist_en, 20, 0.9f, 8);
    std::shared_ptr<Endstop> waist_endstop = std::make_shared<Endstop>(waist_endstop_pin, ENDSTOP_TYPE::UP);

    std::unique_ptr<Joint> waist_joint = std::make_unique<Joint>(0, waist_driver, waist_endstop, 125,
                                                                 drivers::DIRECTION::ANTICLOCKWISE);
    waist_joint->setMinPosition(-1);
    waist_joint->setMaxPosition(358);

    waist_joint->setMaxVelocity(0.2);
    waist_joint->setMaxAcceleration(0.5);

    // Shoulder
    GPIO_PIN shoulder_step, shoulder_dir, shoulder_en, shoulder_endstop_pin;
    shoulder_step.gpio_port = J2_STEP_GPIO_Port;
    shoulder_step.gpio_pin = J2_STEP_Pin;
    shoulder_dir.gpio_port = J2_DIR_GPIO_Port;
    shoulder_dir.gpio_pin = J2_DIR_Pin;
    shoulder_en.gpio_port = J2_EN_GPIO_Port;
    shoulder_en.gpio_pin = J2_EN_Pin;

    shoulder_endstop_pin.gpio_port = J2_ENDSTOP_GPIO_Port;
    shoulder_endstop_pin.gpio_pin = J2_ENDSTOP_Pin;

    std::unique_ptr<Driver> shoulder_driver = std::make_unique<DM556>(1, shoulder_step, shoulder_dir, shoulder_en, 8, 1.8f, 20);
    std::shared_ptr<Endstop> shoulder_endstop = std::make_shared<Endstop>(shoulder_endstop_pin, ENDSTOP_TYPE::UP);

    std::unique_ptr<Joint> shoulder_joint = std::make_unique<Joint>(1, shoulder_driver, shoulder_endstop, 149,
                                                                    drivers::DIRECTION::CLOCKWISE);
    shoulder_joint->setHomingVelocity(0.12);
    shoulder_joint->setMaxAcceleration(0.05);
    shoulder_joint->setMaxPosition(171);
    shoulder_joint->setOffset(15);

    // Elbow
    GPIO_PIN elbow_step, elbow_dir, elbow_en, elbow_endstop_pin;
    elbow_step.gpio_port = J3_STEP_GPIO_Port;
    elbow_step.gpio_pin = J3_STEP_Pin;
    elbow_dir.gpio_port = J3_DIR_GPIO_Port;
    elbow_dir.gpio_pin = J3_DIR_Pin;
    elbow_en.gpio_port = J3_EN_GPIO_Port;
    elbow_en.gpio_pin = J3_EN_Pin;

    elbow_endstop_pin.gpio_port = J3_ENDSTOP_GPIO_Port;
    elbow_endstop_pin.gpio_pin = J3_ENDSTOP_Pin;
    
    std::unique_ptr<Driver> elbow_driver = std::make_unique<drivers::TMC2209>(2, elbow_step, elbow_dir, elbow_en, 20, 0.9, 8);
    std::shared_ptr<Endstop> elbow_endstop = std::make_shared<Endstop>(elbow_endstop_pin, ENDSTOP_TYPE::UP);

    std::unique_ptr<Joint> elbow_joint = std::make_unique<Joint>(2, elbow_driver, elbow_endstop, 62,
                                                                 drivers::DIRECTION::ANTICLOCKWISE);

    elbow_joint->setMaxPosition(70);
    elbow_joint->setBaseAngle(50.3);

    // Roll
    GPIO_PIN roll_step, roll_dir, roll_en, roll_endstop_pin;
    roll_step.gpio_port = J4_STEP_GPIO_Port;
    roll_step.gpio_pin = J4_STEP_Pin;
    roll_dir.gpio_port = J4_DIR_GPIO_Port;
    roll_dir.gpio_pin = J4_DIR_Pin;
    roll_en.gpio_port = J4_EN_GPIO_Port;
    roll_en.gpio_pin = J4_EN_Pin;

    roll_endstop_pin.gpio_port = J4_ENDSTOP_GPIO_Port;
    roll_endstop_pin.gpio_pin = J4_ENDSTOP_Pin;
    
    std::unique_ptr<Driver> wrist_roll_driver = std::make_unique<drivers::TMC2209>(3, roll_step, roll_dir, roll_en, 1, 1.8f, 8);
    std::shared_ptr<Endstop> wrist_roll_endstop = std::make_shared<Endstop>(roll_endstop_pin, ENDSTOP_TYPE::UP);

    std::unique_ptr<Joint> wrist_roll_joint = std::make_unique<Joint>(3, wrist_roll_driver, wrist_roll_endstop,
                                                                      1, drivers::DIRECTION::ANTICLOCKWISE);

    wrist_roll_joint->setHomingAcceleration(0.25);
    wrist_roll_joint->setHomingVelocity(0.5);
    wrist_roll_joint->setHomingSteps(100);

    wrist_roll_joint->setMaxPosition(270);
    wrist_roll_joint->setOffset(-22);

    // Pitch
    GPIO_PIN pitch_step, pitch_dir, pitch_en, pitch_endstop_pin;
    pitch_step.gpio_port = J5_STEP_GPIO_Port;
    pitch_step.gpio_pin = J5_STEP_Pin;
    pitch_dir.gpio_port = J5_DIR_GPIO_Port;
    pitch_dir.gpio_pin = J5_DIR_Pin;
    pitch_en.gpio_port = J5_EN_GPIO_Port;
    pitch_en.gpio_pin = J5_EN_Pin;

    pitch_endstop_pin.gpio_port = J5_ENDSTOP_GPIO_Port;
    pitch_endstop_pin.gpio_pin = J5_ENDSTOP_Pin;
    
    std::unique_ptr<Driver> wrist_pitch_driver = std::make_unique<drivers::TMC2209>(4, roll_step, roll_dir, roll_en, 20, 1.8f, 8);
    std::shared_ptr<Endstop> wrist_pitch_endstop = std::make_shared<Endstop>(roll_endstop_pin, ENDSTOP_TYPE::UP);

    std::unique_ptr<Joint> wrist_pitch_joint = std::make_unique<Joint>(4, wrist_pitch_driver,
                                                                       wrist_pitch_endstop, 40,
                                                                       drivers::DIRECTION::ANTICLOCKWISE);
    wrist_pitch_joint->setHomingAcceleration(0.2);
    wrist_pitch_joint->setHomingVelocity(0.4);
    wrist_pitch_joint->setHomingSteps(100);

    wrist_pitch_joint->setMaxAcceleration(0.8);
    wrist_pitch_joint->setMaxVelocity(1.2);
    wrist_pitch_joint->setMaxPosition(250);

    std::vector<std::unique_ptr<Joint>> joints;
    joints.push_back(std::move(waist_joint));
    joints.push_back(std::move(shoulder_joint));
    joints.push_back(std::move(elbow_joint));
    joints.push_back(std::move(wrist_roll_joint));
    joints.push_back(std::move(wrist_pitch_joint));

    Robot robot(joints);
    return robot;
}
