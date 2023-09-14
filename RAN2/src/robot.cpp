#include "../include/robot.hpp"

static void convertAllToDeg(float* angles_rad, uint8_t number){
    for(int i = 0; i < number; i++){
        angles_rad[i] = rad2Deg(angles_rad[i]);
    }
}

static void convertAllToRad(float* angles_rad, uint8_t number){
    for(int i = 0; i < number; i++){
        angles_rad[i] = deg2Rad(angles_rad[i]);
    }
}

static operation_status operation_status_init_robot(operation_result result, operation_result_code code){
    return operation_status_init(robot, result, code);
}

static operation_status operation_status_init_gcode_reader(operation_result result, operation_result_code code){
    return operation_status_init(gcode_reader, result, code);
}

static void add_offsets_to_angles(float* joint_angles){
    // Theta1
    if(joint_angles[0] < 0){
        joint_angles[0] += 360;
    }

    // Theta2
    joint_angles[1] = 180.f - joint_angles[1];

    // Theta3
    joint_angles[2] = joint_angles[2] - 50.3f;

    // Theta4
    joint_angles[3] = round(joint_angles[3]*1000)/1000;
    if(joint_angles[3] > 180){
        joint_angles[3] = - 360 + joint_angles[3];
    }

    // Theta5
    joint_angles[4] += 112;

    // Theta6
    joint_angles[5] = 180 - joint_angles[5];
    if(joint_angles[5] > 180){
        joint_angles[5] -= 360;
    }
}

static void remove_offsets_from_angles(float* joint_angles){
    // Theta1
    if(joint_angles[0] > 180){
        joint_angles[0] -= 360;
    }

    // Theta2
    joint_angles[1] = 180.f - joint_angles[1];

    // Theta3
    joint_angles[2] = joint_angles[2] + 50.3f + 180.f;

    // Theta4
    if(joint_angles[3] < 0){
        joint_angles[3] += 360;
    }

    // Theta5
    joint_angles[4] -= 112;

    // Theta6
    joint_angles[5] = 180 - joint_angles[5];
    if(joint_angles[5] < 0){
        joint_angles[5] += 360;
    }
}


Robot::Robot(std::vector<std::unique_ptr<Joint>>& joints, GPIO_PIN safeguard_pin) {
    // Initialize joints
    this->joints.reserve(joints.size());
    for (int i = 0; i < joints.size(); i++) {
        this->joints[i] = std::move(joints[i]);
    }

    // Initialize Kinematics Algorithm
    this->k_algorithms = std::make_unique<Algorithm6Dof>(map, joint_offsets);

    GPIO_PIN gripper_pin, feedback_pin;
    gripper_pin.gpio_pin = GRIPPER_PWM_Pin;
    gripper_pin.gpio_port = GRIPPER_PWM_GPIO_Port;

    feedback_pin.gpio_pin = GRIPPER_FEEDBACK_Pin;
    feedback_pin.gpio_port = GRIPPER_FEEDBACK_GPIO_Port;
    this->gripper = std::make_unique<Gripper>(gripper_pin, feedback_pin, &hadc1, &htim2);
    this->gripper->initGripper();

    this->safeguard_pin = safeguard_pin;

    // Ensure that the systems check if done before any action
    systems_status = operation_status_init_robot(failure, 0x02);
}

operation_status Robot::home(){
    operation_status status;
    for(uint16_t i = 0; i < joints.size(); i++){
        status = homeJoint(i);
        if(status.result == failure){
            return status;
        }
    }
    homed = true;
    return operation_status_init_robot(success, 0x00);
}

operation_status Robot::homeJoint(uint8_t joint_number) {
    operation_status status;
    status = joints[joint_number]->homeJoint();

    if(status.result == failure){
        return status;
    }
    return operation_status_init_robot(success, 0x00);
}

operation_status Robot::wait(uint32_t ms) {
    HAL_Delay(ms);
    return operation_status_init_robot(success, 0x00);
}

operation_status Robot::moveJoint(uint8_t joint_number, float position, bool blocking) {
    operation_status status;

    if(angleUnits == radians){
        position = rad2Deg(position);
    }

    status = joints[joint_number]->move2Pos(position, blocking);

    if(status.result == failure){
        return status;
    }
    return operation_status_init_robot(success, 0x00);
}

operation_status Robot::moveJoints(float* angles) {
    float joint_angles_with_removed_offsets[6];
    for(int i = 0; i < 6; i++){
        joint_angles_with_removed_offsets[i] = angles[i];
        moveJoint(i, angles[i], false);
    }

    remove_offsets_from_angles(joint_angles_with_removed_offsets);
    convertAllToRad(joint_angles_with_removed_offsets, 6);
    this->k_algorithms->forwardKinematics(joint_angles_with_removed_offsets, robot_arm_points);

    while(getMovement());
    return operation_status_init_robot(success, 0x00);
}

operation_status Robot::move2Coordinates(float x, float y, float z, float yaw, float pitch, float roll) {
    if(lengthUnits == inches){
        x = inches2Millimeters(x);
        y = inches2Millimeters(y);
        z = inches2Millimeters(z);
    }

    // Convert input millimeters to cm
    x /= 10.f;
    y /= 10.f;
    z /= 10.f;

    if(angleUnits == degrees){
        yaw = deg2Rad(yaw);
        pitch = deg2Rad(pitch);
        roll = deg2Rad(roll);
    }

    float rot_mat_d[9] = {};

    matrix_f32 rotation_matrix;
    matrix_init_f32(&rotation_matrix, 3, 3, rot_mat_d);

    k_algorithms->createRotationMatrix(yaw, pitch, roll, &rotation_matrix);
    k_algorithms->inverseKinematics(x, y, z, &rotation_matrix, joint_angles);
    convertAllToDeg(joint_angles, 6);

    add_offsets_to_angles(joint_angles);

    // Check if the movement is possible for each of joints
    operation_status status;
    for(int i = 0; i < 6; i++) {
        status = this->joints[i]->isMovementPossible(joint_angles[i]);
        if(status.result == failure){
            return status;
        }
    }

    return moveJoints(joint_angles);
}

operation_status Robot::move2Default() {
    float angles[6] = {270.f, 60.f, 35.f, 0.0f, 112.f, 0.0f};
    return moveJoints(angles);
}

operation_status Robot::enableJoint(uint8_t joint_number) {
    return joints[joint_number]->enableMotor();
}

operation_status Robot::enableJoints() {
    operation_status status;
    for(int i = 0; i < joints.size(); i++){
        status = enableJoint(i);

        if(status.result == failure){
            return status;
        }
    }
    return operation_status_init_robot(success, 0x00);
}

operation_status Robot::disableJoint(uint8_t joint_number) {
    return joints[joint_number]->disableMotor();
}

operation_status Robot::disableJoints() {
    operation_status status;
    for(int i = 0; i < joints.size(); i++){
        status = disableJoint(i);

        if(status.result == failure){
            return status;
        }
    }
    return operation_status_init_robot(success, 0x00);
}

operation_status Robot::setLengthUnits(LENGTH_UNITS units) {
    this->lengthUnits = units;
    return operation_status_init_robot(success, 0x00);
}

LENGTH_UNITS Robot::getLengthUnits() {
    return lengthUnits;
}

operation_status Robot::setAngleUnits(ANGLE_UNITS units) {
    this->angleUnits = units;
    return operation_status_init_robot(success, 0x00);
}

ANGLE_UNITS Robot::getAngleUnits() {
    return angleUnits;
}

operation_status Robot::updateEncoders() {
    operation_status status;
    for(uint8_t i = 0; i < 6; i++){
        if(this->joints[i]->encoderAvailable()){
            status = this->joints[i]->updateEncoder();

            if(status.result == failure){
                return status;
            }
        }
    }
    return operation_status_init_robot(success, 0x00);
}

operation_status Robot::getEncoderData(uint8_t joint_number, MagneticEncoderData *data) {
    operation_status status;
    status = this->joints[joint_number-1]->getEncoderData(data);

    if(status.result == failure){
        return status;
    }
    return operation_status_init_robot(success, 0x00);
}

operation_status Robot::getJointPosition(uint8_t joint_number, float *position) {
    this->joints[joint_number-1]->getJointPosition(position);
    return operation_status_init_robot(success, 0x00);
}

void Robot::getJointAngles(float *angles) {
    for(uint8_t i = 0; i < 6; i++){
        angles[i] = this->joint_angles[i];
    }

    if(angleUnits == radians){
        convertAllToRad(angles, 6);
    }
}

void Robot::getRobotArmCoordinates(coordinates* coordinates){
    for(uint8_t i = 0; i < 6; i++){
        coordinates[i] = this->robot_arm_points[i];
        coordinates[i].x *= 10;
        coordinates[i].y *= 10;
        coordinates[i].z *= 10;

        if(lengthUnits == inches){
            millimeters2Inches(coordinates[i].x);
            millimeters2Inches(coordinates[i].y);
            millimeters2Inches(coordinates[i].z);
        }
    }
}

operation_status Robot::systemsCheck(bool initial) {

    if(initial){
        // Check safeguard button status
        if(HAL_GPIO_ReadPin(safeguard_pin.gpio_port, safeguard_pin.gpio_pin) == GPIO_PIN_RESET){
            systems_status = operation_status_init_robot(failure, 0x03);
            return systems_status;
        }
    }

    // Check status of each joint
    for(uint8_t i = 0; i < 6; i++){
        systems_status = this->joints[i]->getJointStatus();

        if(systems_status.result == failure){
            return systems_status;
        }
    }

    systems_status = operation_status_init_robot(success, 0x00);
    return systems_status;
}

operation_status Robot::safeguardTrigger() {
    safeguard_stop = true;
    for(uint8_t i = 0; i < 6; i++){
        this->joints[i]->stopJoint();
    }
    homed = false;
    return operation_status_init_robot(success, 0x00);
}

operation_status Robot::safeguardReset() {
    safeguard_stop = false;
    for(uint8_t i = 0; i < 6; i++){
        this->joints[i]->startJoint();
    }
    return operation_status_init_robot(success, 0x00);
}

bool Robot::getMovement() {
    for(uint8_t i = 0; i < 6; i++){
        if(this->joints[i]->isMoving()){
            return true;
        }
    }
    return false;
}

operation_status Robot::enableGripper() {
    this->gripper->enableGripper();
    return operation_status_init_robot(success, 0x00);
}

operation_status Robot::disableGripper() {
    this->gripper->disableGripper();
    return operation_status_init_robot(success, 0x00);
}

operation_status Robot::setGripperClosedPercentage(float percentage) {
    enableGripper();
    this->gripper->setPosition(percentage);
    return operation_status_init_robot(success, 0x00);
}

float Robot::readGripperRawPosition() {
    return this->gripper->readCurrentPosition();
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

    std::unique_ptr<Driver> waist_driver = std::make_unique<drivers::Driver>(0, waist_step, waist_dir, waist_en, 20, 0.9f, 8);
    std::shared_ptr<Endstop> waist_endstop = std::make_shared<Endstop>(waist_endstop_pin, ENDSTOP_TYPE::UP);

    std::unique_ptr<Joint> waist_joint = std::make_unique<Joint>(0, waist_driver, 125,
                                                                 drivers::DIRECTION::ANTICLOCKWISE, waist_endstop);
    waist_joint->setMinPosition(-1);
    waist_joint->setMaxPosition(350);

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

    std::unique_ptr<Driver> shoulder_driver = std::make_unique<Driver>(1, shoulder_step, shoulder_dir, shoulder_en, 20, 1.8f, 8);
    std::shared_ptr<Endstop> shoulder_endstop = std::make_shared<Endstop>(shoulder_endstop_pin, ENDSTOP_TYPE::UP);

    std::unique_ptr<Joint> shoulder_joint = std::make_unique<Joint>(1, shoulder_driver, 149,
                                                                    drivers::DIRECTION::CLOCKWISE, shoulder_endstop);
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
    
    std::unique_ptr<Driver> elbow_driver = std::make_unique<drivers::Driver>(2, elbow_step, elbow_dir, elbow_en, 20, 0.9, 8);
    std::shared_ptr<Endstop> elbow_endstop = std::make_shared<Endstop>(elbow_endstop_pin, ENDSTOP_TYPE::UP);

    std::unique_ptr<Joint> elbow_joint = std::make_unique<Joint>(2, elbow_driver, 62,
                                                                 drivers::DIRECTION::ANTICLOCKWISE, elbow_endstop);

    elbow_joint->setMaxPosition(70);
    elbow_joint->setMaxVelocity(1.6);
    elbow_joint->setMaxAcceleration(3);

    // Elbow roll
    GPIO_PIN elbow_roll_step, elbow_roll_dir, elbow_roll_en, elbow_roll_endstop_pin;
    elbow_roll_step.gpio_port = J4_STEP_GPIO_Port;
    elbow_roll_step.gpio_pin = J4_STEP_Pin;
    elbow_roll_dir.gpio_port = J4_DIR_GPIO_Port;
    elbow_roll_dir.gpio_pin = J4_DIR_Pin;
    elbow_roll_en.gpio_port = J4_EN_GPIO_Port;
    elbow_roll_en.gpio_pin = J4_EN_Pin;

    elbow_roll_endstop_pin.gpio_port = J4_ENDSTOP_GPIO_Port;
    elbow_roll_endstop_pin.gpio_pin = J4_ENDSTOP_Pin;

    std::shared_ptr<MagneticEncoder> elbow_roll_encoder = std::make_shared<MagneticEncoder>(3, 0x36, 3, &hi2c1, 120.f, 1.f);

    std::unique_ptr<Driver> elbow_roll_driver = std::make_unique<drivers::Driver>(3, elbow_roll_step, elbow_roll_dir, elbow_roll_en, 1, 1.8f, 8);
    std::shared_ptr<Endstop> elbow_roll_endstop = std::make_shared<Endstop>(elbow_roll_endstop_pin, ENDSTOP_TYPE::UP);


    std::unique_ptr<Joint> elbow_roll_joint = std::make_unique<Joint>(3, elbow_roll_driver,
                                                                      1, drivers::DIRECTION::ANTICLOCKWISE, elbow_roll_endstop, elbow_roll_encoder);

    //elbow_roll_joint->setHomingAcceleration(0.25);
    //elbow_roll_joint->setHomingVelocity(0.5);
    elbow_roll_joint->setEncoderHoming();
    elbow_roll_joint->setSmartEncoderHoming();
    elbow_roll_joint->setHomingSteps(100);

    elbow_roll_joint->setMinPosition(-180);
    elbow_roll_joint->setMaxPosition(180);
    elbow_roll_joint->setMaxAcceleration(2);
    elbow_roll_joint->setMaxVelocity(1.5);

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
    
    std::unique_ptr<Driver> wrist_pitch_driver = std::make_unique<drivers::Driver>(4, pitch_step, pitch_dir, pitch_en, 20, 1.8f, 8);
    std::shared_ptr<Endstop> wrist_pitch_endstop = std::make_shared<Endstop>(pitch_endstop_pin, ENDSTOP_TYPE::UP);

    std::unique_ptr<Joint> wrist_pitch_joint = std::make_unique<Joint>(4, wrist_pitch_driver, 40,
                                                                       drivers::DIRECTION::ANTICLOCKWISE, wrist_pitch_endstop);
    wrist_pitch_joint->setHomingSteps(100);

    wrist_pitch_joint->setMaxAcceleration(4);
    wrist_pitch_joint->setMaxVelocity(3);
    wrist_pitch_joint->setMaxPosition(250);

    // Wrist roll
    GPIO_PIN wrist_roll_step, wrist_roll_dir, wrist_roll_en;
    wrist_roll_step.gpio_port = J6_STEP_GPIO_Port;
    wrist_roll_step.gpio_pin = J6_STEP_Pin;
    wrist_roll_dir.gpio_port = J6_DIR_GPIO_Port;
    wrist_roll_dir.gpio_pin = J6_DIR_Pin;
    wrist_roll_en.gpio_port = J6_EN_GPIO_Port;
    wrist_roll_en.gpio_pin = J6_EN_Pin;

    std::shared_ptr<MagneticEncoder> wrist_roll_encoder = std::make_shared<MagneticEncoder>(5, 0x36, 2, &hi2c1, 28.f, 1.f);

    std::unique_ptr<Driver> wrist_roll_driver = std::make_unique<drivers::Driver>(5, wrist_roll_step, wrist_roll_dir, wrist_roll_en, 1, 1.8f, 8);


    std::unique_ptr<Joint> wrist_roll_joint = std::make_unique<Joint>(5, wrist_roll_driver, 1,
                                                                       drivers::DIRECTION::CLOCKWISE, nullptr, wrist_roll_encoder);

    wrist_roll_joint->setMaxPosition(180);
    wrist_roll_joint->setMinPosition(-180);
    wrist_roll_joint->setMaxAcceleration(4);
    wrist_roll_joint->setMaxVelocity(3);
    wrist_roll_joint->setHomingVelocity(0.5);
    wrist_roll_joint->setHomingAcceleration(1);
    
    

    std::vector<std::unique_ptr<Joint>> joints;
    joints.push_back(std::move(waist_joint));
    joints.push_back(std::move(shoulder_joint));
    joints.push_back(std::move(elbow_joint));
    joints.push_back(std::move(elbow_roll_joint));
    joints.push_back(std::move(wrist_pitch_joint));
    joints.push_back(std::move(wrist_roll_joint));

    GPIO_PIN safeguard_pin;
    safeguard_pin.gpio_port = SAFEGUARD_PIN_GPIO_Port;
    safeguard_pin.gpio_pin = SAFEGUARD_PIN_Pin;

    Robot robot(joints, safeguard_pin);
    return robot;
}

static operation_status executeGGCODE(Robot& robot, uint16_t code, uint8_t parse_result, uint16_t counter, const char* command_str){
    float value;
    char letter;
    operation_status status;

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
                return robot.move2Coordinates(coordinates[0], coordinates[1], coordinates[2], yaw, pitch, roll);
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

                return robot.wait(time);
            }

        }
        case 27:
        {
            return robot.move2Default();
        }
        case 28:
        {
            if(parse_result == 0){
                return robot.home();
            }
            else{
                while(parse_result == 1){
                    parse_result = parseMessage(&letter, &value, command_str, &counter);

                    if(letter == 'J' && value >= 1 && value < 7){
                        status = robot.homeJoint((int)value - 1);
                        if(status.result == failure){
                            return status;
                        }
                    }
                }
                return status;
            }
        }
        case 20:
            return robot.setLengthUnits(inches);
        case 21:
            return robot.setLengthUnits(millimeters);
        case 29:
            return robot.setAngleUnits(degrees);
        case 30:
            return robot.setAngleUnits(radians);
        default:
            break;

    }
    return operation_status_init_gcode_reader(failure, 0x02);
}

static operation_status executeMGCODE(Robot& robot, uint16_t code, uint8_t parse_result, uint16_t counter, const char* command_str){
    float value;
    char letter;
    operation_status status;

    switch (code) {
        case 0:
            break;
        case 17:
        {
            if(parse_result == 1){
                while(parse_result == 1){
                    parse_result = parseMessage(&letter, &value, command_str, &counter);

                    if(letter == 'J' && value >= 1 && value < 7){
                        status = robot.enableJoint((int)value - 1);
                        if(status.result == failure){
                            return status;
                        }
                    }
                }
                return status;
            }
            else{
                return robot.enableJoints();
            }
        }
        case 18:
        {
            if(parse_result == 1){
                while(parse_result == 1){
                    parse_result = parseMessage(&letter, &value, command_str, &counter);

                    if(letter == 'J' && value >= 1 && value < 7){
                        status = robot.disableJoint((int)value - 1);
                        if(status.result ==  failure){
                            return status;
                        }
                    }
                }
                return status;
            }
            else{
                return robot.disableJoints();
            }
        }
        case 114:
        {
            float position;
            MagneticEncoderData data;

            if(parse_result == 1) {
                while(parse_result == 1){
                    parse_result = parseMessage(&letter, &value, command_str, &counter);

                    if(letter == 'J' && value >= 1 && value < 7){
                        status = robot.getJointPosition((uint8_t)value, &position);
                        printf("Joint%d position (calculated): %f deg\n", (uint8_t)value, position);
                        if(status.result ==  failure){
                            return status;
                        }
                    }
                    else if(letter == 'E' && value >= 1 && value < 7){
                        status = robot.getEncoderData((uint8_t)value, &data);
                        printf("Joint%d position (measured): %f deg\n", (uint8_t)value, data.position);
                        if(status.result ==  failure){
                            return status;
                        }
                    }
                }
            }
            return operation_status_init_gcode_reader(success, 0x00);
        }
        case 280:
        {
            if(parse_result == 1) {
                parseMessage(&letter, &value, command_str, &counter);

                if (letter == 'P' && value >= 0 && value <= 100) {
                    status = robot.setGripperClosedPercentage(value);
                    return status;
                }
            }
            break;
        }
        case 282:
        {
            status = robot.disableGripper();
            return status;
        }
        case 410:
        {
            status = robot.safeguardTrigger();
            return status;
        }
        case 999:
        {
            status = robot.safeguardReset();
            return status;
        }
        default:
            break;
    }
    return operation_status_init_gcode_reader(failure, 0x02);
}


operation_status executeGCODE(Robot& robot, const char* command_str) {
    uint8_t parse_result = 0;
    uint16_t counter = 0;
    char g_letter;
    float value = 0;

    parse_result = parseMessage(&g_letter, &value, command_str, &counter);

    switch (g_letter) {
        case 'G':
            return executeGGCODE(robot, (int)value, parse_result, counter, command_str);
        case 'M':
            return executeMGCODE(robot, (int)value, parse_result, counter, command_str);
        case 'J':
        {
            auto joint_number = (uint8_t)value;
            parseMessage(&g_letter, &value, command_str, &counter);
            if(g_letter == 'P'){
                return robot.moveJoint(joint_number-1, value);
            }
            break;
        }
        default:
            break;

    }
    return operation_status_init_gcode_reader(failure, 0x02);
}
