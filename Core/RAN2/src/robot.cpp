#include "../include/robot.hpp"

static void convertAllToDeg(float* angles_rad, uint8_t number){
    for(int i = 0; i < number; i++){
        angles_rad[i] = rad2Deg(angles_rad[i]);
    }
}

Robot::Robot(std::vector<std::unique_ptr<Joint>>& joints) {

    // Initialize joints
    this->joints.reserve(joints.size());
    for (int i = 0; i < joints.size(); i++) {
        this->joints[i] = std::move(joints[i]);
    }

    // Initialize Kinematics Algorithm
    this->k_algorithms = std::make_unique<Algorithm6Dof>(map, joint_offsets);
}

void Robot::home(){

    for(int i = 0; i < joints.size(); i++){
        printf("Homing joint: %d\n", i);
        homeJoint(i);
    }
    homed = true;
}

void Robot::homeJoint(uint8_t joint_number) {
    if(joints[joint_number]->isEnabled()){
        joints[joint_number]->homeJoint();
    }
}

void Robot::wait(uint32_t ms) {
    HAL_Delay(ms);
}

void Robot::moveJoint(uint8_t joint_number, float position, bool blocking) {
    if(angleUnits == radians){
        position = rad2Deg(position);
    }

    if(homed){
        joints[joint_number]->move2Pos(position, blocking);
    }
}

void Robot::moveJoints(float* angles) {
    for(int i = 0; i < 6; i++){
        moveJoint(i, angles[i], false);
    }
}

void Robot::move2Coordinates(float x, float y, float z, float yaw, float pitch, float roll) {
    if(lengthUnits == inches){
        x = inches2Millimeters(x);
        y = inches2Millimeters(x);
        z = inches2Millimeters(z);
    }

    if(angleUnits == degrees){
        yaw = deg2Rad(yaw);
        pitch = deg2Rad(pitch);
        roll = deg2Rad(roll);
    }

    float rot_mat_d[9] = {};

    matrix_f32 rotation_matrix;
    matrix_init_f32(&rotation_matrix, 3, 3, rot_mat_d);

    k_algorithms->createRotationMatrix(yaw, pitch, roll, &rotation_matrix);
    for(int i = 0; i < 3; i ++){
        printf("%f %f %f\n", rot_mat_d[3*i], rot_mat_d[3*i+1], rot_mat_d[3*i+2]);
    }

    k_algorithms->inverseKinematics(x, y, z, &rotation_matrix, joint_angles);

    convertAllToDeg(joint_angles, 6);

    //TODO Add offsets handling before passing angles to the execution
    // Temporarily
    if(joint_angles[0] < 0){
        joint_angles[0] += 360;
    }

    joint_angles[1] = 180.f - joint_angles[1];
    joint_angles[2] = joint_angles[2] - 50.3f;
    joint_angles[3] = -joint_angles[3];

    for(int i = 0; i < 6; i++) {
        printf("Theta%d: %f\n", i, joint_angles[i]);
    }

    moveJoints(joint_angles);
}

void Robot::move2Default() {
    if(homed){
        moveJoint(0, 270, false);
        moveJoint(1, 60, false);
        moveJoint(2, 35, false);
        moveJoint(3, 0.1, false);
        moveJoint(4, 112, false);
    }
}

void Robot::enableJoint(uint8_t joint_number) {
    joints[joint_number]->enableMotor();
}

void Robot::enableJoints() {
    for(int i = 0; i < joints.size(); i++){
        this->joints[i]->enableMotor();
    }
}

void Robot::disableJoint(uint8_t joint_number) {
    joints[joint_number]->disableMotor();
}

void Robot::disableJoints() {
    for(int i = 0; i < joints.size(); i++){
        this->joints[i]->disableMotor();
    }
}

void Robot::setLengthUnits(LENGTH_UNITS units) {
    this->lengthUnits = units;
}

LENGTH_UNITS Robot::getLengthUnits() {
    return lengthUnits;
}

void Robot::setAngleUnits(ANGLE_UNITS units) {
    this->angleUnits = units;
}

ANGLE_UNITS Robot::getAngleUnits() {
    return angleUnits;
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

    waist_joint->setMaxVelocity(1.6);
    waist_joint->setMaxAcceleration(3);

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
    shoulder_joint->setHomingVelocity(0.16);

    shoulder_joint->setMaxVelocity(0.32);
    shoulder_joint->setMaxAcceleration(0.2);

    shoulder_joint->setMaxPosition(171);
    shoulder_joint->setOffset(10);

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
    elbow_joint->setBaseAngle(48);
    elbow_joint->setMaxVelocity(1.6);
    elbow_joint->setMaxAcceleration(3);

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

    //wrist_roll_joint->setHomingAcceleration(0.25);
    //wrist_roll_joint->setHomingVelocity(0.5);
    wrist_roll_joint->setHomingSteps(100);

    wrist_roll_joint->setMaxPosition(350);
    wrist_roll_joint->setOffset(-20);

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
    
    std::unique_ptr<Driver> wrist_pitch_driver = std::make_unique<drivers::TMC2209>(4, pitch_step, pitch_dir, pitch_en, 20, 1.8f, 8);
    std::shared_ptr<Endstop> wrist_pitch_endstop = std::make_shared<Endstop>(pitch_endstop_pin, ENDSTOP_TYPE::UP);

    std::unique_ptr<Joint> wrist_pitch_joint = std::make_unique<Joint>(4, wrist_pitch_driver,
                                                                       wrist_pitch_endstop, 40,
                                                                       drivers::DIRECTION::ANTICLOCKWISE);
    wrist_pitch_joint->setHomingSteps(100);

    wrist_pitch_joint->setMaxAcceleration(4);
    wrist_pitch_joint->setMaxVelocity(3);
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

static void executeGGCODE(Robot& robot, uint16_t code, uint8_t parse_result, uint16_t counter, const char* command_str){
    float value;
    char letter;

    switch (code) {
        case 00:
        {
            float coordinates[3];
            bool checkTable[3] = {false, false, false};

            float yaw = 0;
            float pitch = 0;
            float roll = 0;

            while(parse_result == 1){
                parse_result = parseMessage(&letter, &value, command_str, &counter);

                switch (letter) {
                    case 'X':
                        coordinates[0] = value;
                        checkTable[0] = true;
                        break;
                    case 'Y':
                        coordinates[1] = value;
                        checkTable[1] = true;
                        break;
                    case 'Z':
                        coordinates[2] = value;
                        checkTable[2] = true;
                        break;
                    case 'W':
                        yaw = value;
                        break;
                    case 'P':
                        pitch = value;
                        break;
                    case 'R':
                        roll = value;
                        break;
                    default:
                        break;
                }
            }

            // Check if all coordinates has been assigned
            if(checkTable[0] && checkTable[1] && checkTable[2]){
                robot.move2Coordinates(coordinates[0], coordinates[1], coordinates[2], yaw, pitch, roll);
            }
            break;
        }
        case 4:
        {
            uint32_t time = 0;
            if(parse_result == 0){
                //TODO Zachowanie takie samo jak w przypadku M400, ma zaczekać z przetwarzaniem kodu tak długo jak
                // zakończone zostaną wszystkie ruchy
                break;
            }
            else{
                while(parse_result == 1){
                    parse_result = parseMessage(&letter, &value, command_str, &counter);

                    if(letter == 'S' && value >= 0){
                        time += (uint32_t)seconds2Milliseconds(value);
                    }
                    else if(letter == 'P' && value >= 0){
                        time += (uint32_t)value;
                    }
                }

                robot.wait(time);
            }

        }
        case 27:
        {
            robot.move2Default();
            break;
        }
        case 28:
        {
            if(parse_result == 0){
                robot.home();
            }
            else{
                while(parse_result == 1){
                    parse_result = parseMessage(&letter, &value, command_str, &counter);

                    if(letter == 'J' && value >= 1 && value < 7){
                        robot.homeJoint((int)value - 1);
                    }
                }
            }
            break;
        }
        case 20:
            robot.setLengthUnits(inches);
            break;
        case 21:
            robot.setLengthUnits(millimeters);
            break;
        case 29:
            robot.setAngleUnits(degrees);
            break;
        case 30:
            robot.setAngleUnits(radians);
            break;
        default:
            break;

    }
}

static void executeMGCODE(Robot& robot, uint16_t code, uint8_t parse_result, uint16_t counter, const char* command_str){
    float value;
    char letter;

    switch (code) {
        case 0:
            break;
        case 17:
        {
            if(parse_result == 1){
                while(parse_result == 1){
                    parse_result = parseMessage(&letter, &value, command_str, &counter);

                    if(letter == 'J' && value >= 1 && value < 7){
                        robot.enableJoint((int)value - 1);
                    }
                }
            }
            else{
                robot.enableJoints();
            }
            break;
        }
        case 18:
        {
            if(parse_result == 1){
                while(parse_result == 1){
                    parse_result = parseMessage(&letter, &value, command_str, &counter);

                    if(letter == 'J' && value >= 1 && value < 7){
                        robot.disableJoint((int)value - 1);
                    }
                }
            }
            else{
                robot.disableJoints();
            }
            break;
        }
        default:
            break;
    }
}


void executeGCODE(Robot& robot, const char* command_str) {
    uint8_t parse_result;
    uint16_t counter;
    char g_letter;
    float value;

    parse_result = parseMessage(&g_letter, &value, command_str, &counter);

    switch (g_letter) {
        case 'G':
            executeGGCODE(robot, (int)value, parse_result, counter, command_str);
            break;
        case 'M':
            executeMGCODE(robot, (int)value, parse_result, counter, command_str);
            break;
        case 'J':
        {
            auto joint_number = (uint8_t)value;
            parseMessage(&g_letter, &value, command_str, &counter);
            if(g_letter == 'P'){
                robot.moveJoint(joint_number-1, value);
            }
            break;
        }
        default:
            break;

    }

}
