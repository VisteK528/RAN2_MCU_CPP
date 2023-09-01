#include "../include/algorithm6dof.hpp"

Algorithm6Dof::Algorithm6Dof(LINK_MAP link_map, float *offsets) {
    this->linkMap = std::move(link_map);
    this->offsets = offsets;
}

void Algorithm6Dof::createRotationMatrix(float yaw, float pitch, float roll, matrix_f32* rot_mat) {
    float yaw_matrix_d[9] = {
             cos(yaw), sin(yaw), 0,
             -sin(yaw),  cos(yaw), 0,
                    0,         0, 1,
    };

    float pitch_matrix_d[9] = {
             cos(pitch), 0, sin(pitch),
                      0, 1,          0,
            -sin(pitch), 0, cos(pitch),
    };

    float roll_matrix_d[9] = {
            1,         0,          0,
            0, cos(roll), sin(roll),
            0, -sin(roll),  cos(roll),
    };

    float buffer_matrix_d[9] = {};

    matrix_f32 yaw_matrix, pitch_matrix, roll_matrix, buffer_matrix;

    matrix_init_f32(&yaw_matrix, 3, 3, yaw_matrix_d);
    matrix_init_f32(&pitch_matrix, 3, 3, pitch_matrix_d);
    matrix_init_f32(&roll_matrix, 3, 3, roll_matrix_d);

    matrix_init_f32(&buffer_matrix, 3, 3, buffer_matrix_d);

    matrix_mul_f32(&yaw_matrix, &pitch_matrix, &buffer_matrix);
    matrix_mul_f32(&buffer_matrix, &roll_matrix, rot_mat);
}


void Algorithm6Dof::inverseKinematics(float x, float y, float z, matrix_f32* rot_mat, float* angles) {

    // New version V3.0
    float theta[6];
    coordinates tcp, wcp, elbow;
    float hyp, long_hyp, short_hyp;
    float alfa, beta;

    // Set position of the TCP(Tool center point)
    arm_position_points[5].x = x;
    arm_position_points[5].y = y;
    arm_position_points[5].z = z;
    tcp = arm_position_points[5];
    tcp.y *= -1;

    // Calculate position of the WCP (Wrist center point) by transforming the TCP coordinates using Rot0_6 matrix
    wcp.x = tcp.x - linkMap[EE_LENGTH] * rot_mat->p_data[2];
    wcp.y = tcp.y - linkMap[EE_LENGTH] * rot_mat->p_data[5];
    wcp.z = tcp.z - linkMap[EE_LENGTH] * rot_mat->p_data[8];

    arm_position_points[4] = wcp;

    // Calculate the angle for J1 (Waist)
    theta[0] = atan2(-wcp.y, wcp.x);

    // Calculate the angle for J2 (Shoulder)
    // Long hypotenuse is the guiding radius on the XY Plane
    long_hyp = sqrt(pow(wcp.x, 2) + pow(wcp.y, 2));
    alfa = atan((wcp.z - linkMap[BASE_HEIGHT] - linkMap[SHOULDER_HEIGHT]) / long_hyp);

    // Hypotenuse of the shoulder - elbow triangle
    hyp = long_hyp / cos(alfa);

    beta = acos((pow(linkMap[ELBOW_LENGTH], 2) - pow(linkMap[SHOULDER_LENGTH], 2) - pow(hyp, 2)) / (
            -2 * linkMap[SHOULDER_LENGTH] * hyp));

    theta[1] = alfa + beta;

    // Calculate the angle for J3 (Elbow)
    theta[2] = acos((pow(hyp, 2) - pow(linkMap[SHOULDER_LENGTH], 2) - pow(linkMap[ELBOW_LENGTH], 2) )/ (
            -2 * linkMap[SHOULDER_LENGTH] * linkMap[ELBOW_LENGTH]));

    elbow.z = linkMap[BASE_HEIGHT]+ linkMap[SHOULDER_HEIGHT] + sin(theta[1]) * linkMap[SHOULDER_LENGTH];

    short_hyp = cos(theta[1]) * linkMap[SHOULDER_LENGTH];
    elbow.x = short_hyp * cos(theta[0]);
    elbow.y = short_hyp * sin(theta[0]);

    arm_position_points[3] = elbow;

    // Matrices
    float rot_0_1_d[9] = {
            cos(theta[0]), 0, -sin(theta[0]),
            -sin(theta[0]), 0, -cos(theta[0]),
            0, 1, 0
    };

    float rot_1_2_d[9] = {
            cos(theta[1]), -sin(theta[1]), 0,
            sin(theta[1]), cos(theta[1]), 0,
            0, 0, 1
    };

    float rot_2_3_d[9] = {
            sin((float)M_PI - theta[2]), 0, cos((float)M_PI - theta[2]),
            cos((float)M_PI - theta[2]), 0, -sin((float)M_PI - theta[2]),
            0, 1, 0
    };

    float rot_0_2_d[9];
    float rot_0_3_d[9];
    float rot_3_6_d[9];
    float rot_inv_0_3_d[9] = {
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
    };

    matrix_f32 rot_0_1, rot_1_2, rot_2_3, rot_0_3, rot_3_6, inv_rot_0_3, rot_0_2;

    matrix_init_f32(&rot_0_1, 3, 3, rot_0_1_d);
    matrix_init_f32(&rot_1_2, 3, 3, rot_1_2_d);
    matrix_init_f32(&rot_2_3, 3, 3, rot_2_3_d);
    matrix_init_f32(&rot_0_3, 3, 3, rot_0_3_d);
    matrix_init_f32(&rot_3_6, 3, 3, rot_3_6_d);
    matrix_init_f32(&inv_rot_0_3, 3, 3, rot_inv_0_3_d);
    matrix_init_f32(&rot_0_2, 3, 3, rot_0_2_d);

    matrix_mul_f32(&rot_0_1, &rot_1_2, &rot_0_2);
    matrix_mul_f32(&rot_0_2, &rot_2_3, &rot_0_3);


    inverse_matrix_f32(&rot_0_3, &inv_rot_0_3);


    matrix_mul_f32(&inv_rot_0_3, rot_mat, &rot_3_6);

    theta[4] = acos((rot_3_6.p_data)[8]);
    theta[3] = M_PI + atan2(rot_3_6.p_data[5], -rot_3_6.p_data[2]);

    theta[5] =  atan2(rot_3_6.p_data[7], rot_3_6.p_data[6]);

    for(int i = 0; i < 6; i++){
        angles[i] = theta[i];
    }
}

void Algorithm6Dof::forwardKinematics(float *angles, coordinates *position_points) {

    float short_hyp, hyp, long_hyp;
    float alfa, beta;

    // World origin
    this->arm_position_points[0].x = 0;
    this->arm_position_points[0].y = 0;
    this->arm_position_points[0].z = 0;

    // Base point (J1 revolute point)
    this->arm_position_points[1].x = 0;
    this->arm_position_points[1].y = 0;
    this->arm_position_points[1].z = BASE_HEIGHT;

    // Shoulder point (J2 revolute point)
    this->arm_position_points[2].x  = 0;
    this->arm_position_points[2].y  = 0;
    this->arm_position_points[2].z  = BASE_HEIGHT + SHOULDER_HEIGHT;

    // Elbow point (J3 revolute point)
    short_hyp = cos(angles[1]) * SHOULDER_LENGTH;
    this->arm_position_points[3].x  = short_hyp * cos(angles[0]);
    this->arm_position_points[3].y  = short_hyp * sin(angles[0]);
    this->arm_position_points[3].z  = BASE_HEIGHT + SHOULDER_HEIGHT + sin(angles[1]) * SHOULDER_LENGTH;

    // Wrist center point
    hyp = sqrt(pow(SHOULDER_LENGTH, 2.f) + pow(ELBOW_LENGTH, 2.f) - 2*SHOULDER_LENGTH*ELBOW_LENGTH*cos(angles[2]-(float)M_PI));

    alfa = acos((pow(ELBOW_LENGTH, 2.f) - pow(SHOULDER_LENGTH, 2.f) - pow(hyp, 2.f))/(-2*SHOULDER_LENGTH*hyp));
    beta = angles[1] - alfa;
    long_hyp = hyp*cos(beta);

    this->arm_position_points[4].x  = long_hyp*cos(angles[0]);
    this->arm_position_points[4].y  = long_hyp*sin(angles[0]);
    this->arm_position_points[4].z  = sin(beta)*hyp + BASE_HEIGHT + SHOULDER_HEIGHT;

    // TCP
    float rot_0_6_col[3] = {
            -cos(angles[4])*(cos(angles[0])*sin(angles[1])*sin(angles[2]-(float)M_PI) - cos(angles[0])*cos(angles[1])*cos(angles[2]-(float)M_PI)) - sin(angles[0])*sin(angles[3])*sin(angles[4]) - cos(angles[3])*sin(angles[4])*(cos(angles[0])*cos(angles[1])*sin(angles[2]-(float)M_PI) + cos(angles[0])*cos(angles[2]-(float)M_PI)*sin(angles[1])),
            cos(angles[4])*(sin(angles[0])*sin(angles[1])*sin(angles[2]-(float)M_PI) - cos(angles[1])*cos(angles[2]-(float)M_PI)*sin(angles[0])) - cos(angles[0])*sin(angles[3])*sin(angles[4]) + cos(angles[3])*sin(angles[4])*(cos(angles[1])*sin(angles[0])*sin(angles[2]-(float)M_PI) + cos(angles[2]-(float)M_PI)*sin(angles[0])*sin(angles[1])),
            -cos(angles[4])*(cos(angles[1])*sin(angles[2]-(float)M_PI) + cos(angles[2]-(float)M_PI)*sin(angles[1])) - cos(angles[3])*sin(angles[4])*(cos(angles[1])*cos(angles[2]-(float)M_PI) - sin(angles[1])*sin(angles[2]-(float)M_PI))
    };
    
    this->arm_position_points[5].x = arm_position_points[4].x - EE_LENGTH*rot_0_6_col[0];
    this->arm_position_points[5].y = arm_position_points[4].y + EE_LENGTH*rot_0_6_col[1];
    this->arm_position_points[5].z = arm_position_points[4].z - EE_LENGTH*rot_0_6_col[2];

    for(uint8_t i = 0; i < 6; i++){
        position_points[i] = this->arm_position_points[i];
    }
}