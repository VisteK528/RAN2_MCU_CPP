#include <utility>
#include "../include/algorithm6dof.hpp"

Algorithm6Dof::Algorithm6Dof(LINK_MAP link_map, float *offsets) {
    this->linkMap = std::move(link_map);
    this->offsets = offsets;
}

void Algorithm6Dof::createRotationMatrix(float yaw, float pitch, float roll, matrix_f32* rot_mat) {
    float yaw_matrix_d[9] = {
             cos(yaw), -sin(yaw), 0,
             sin(yaw),  cos(yaw), 0,
                    0,         0, 1,
    };

    float pitch_matrix_d[9] = {
             cos(pitch), 0, sin(pitch),
                      0, 1,          0,
            -sin(pitch), 0, cos(pitch),
    };

    float roll_matrix_d[9] = {
            1,         0,          0,
            0, cos(roll), -sin(roll),
            0, sin(roll),  cos(roll),
    };

    float buffer_matrix_d[9] = {};

    matrix_f32 yaw_matrix, pitch_matrix, roll_matrix, buffer_matrix;

    matrix_init_f32(&yaw_matrix, 3, 3, yaw_matrix_d);
    matrix_init_f32(&pitch_matrix, 3, 3, pitch_matrix_d);
    matrix_init_f32(&roll_matrix, 3, 3, roll_matrix_d);

    matrix_init_f32(&buffer_matrix, 3, 3, buffer_matrix_d);

    matrix_mult(&yaw_matrix, &pitch_matrix, &buffer_matrix);
    matrix_mult(&buffer_matrix, &roll_matrix, rot_mat);
}


void Algorithm6Dof::inverseKinematics(float x, float y, float z, matrix_f32* rot_mat,
                                      float* angles) {

    float theta[6];
    coordinates tcp, wcp, elbow;
    float hyp, long_hyp, short_hyp;
    float alfa, beta;

    // Set position of the TCP(Tool center point)
    arm_position_points[5].x = x;
    arm_position_points[5].y = y;
    arm_position_points[5].z = z;
    tcp = arm_position_points[5];

    // Calculate position of the WCP (Wrist center point) by transforming the TCP coordinates using Rot0_6 matrix
    wcp.x = tcp.x - linkMap[EE_LENGTH] * rot_mat->p_data[0];
    wcp.y = tcp.y - linkMap[EE_LENGTH] * rot_mat->p_data[3];
    wcp.z = tcp.z - linkMap[EE_LENGTH] * rot_mat->p_data[6];
    arm_position_points[4] = wcp;

    // Calculate the angle for J1 (Waist)
    theta[0] = atan2(wcp.y, wcp.x);

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
            cos(theta[0]), sin(theta[0]), 0,
            -sin(theta[0]), cos(theta[0]), 0,
            0, 0, 1
    };

    float rot_1_2_d[9] = {
            cos(theta[1]), 0, sin(theta[1]),
                       0, 1, 0,
            -sin(theta[1]), 0, cos(theta[1])
    };

    float rot_2_3_d[9] = {
            cos(theta[2]), 0, -sin(theta[2]),
            0, 1, 0,
            sin(theta[2]), 0, cos(theta[2])
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

    matrix_mult(&rot_0_1, &rot_1_2, &rot_0_2);
    matrix_mult(&rot_0_2, &rot_2_3, &rot_0_3);


    inverse_matrix(&rot_0_3, &inv_rot_0_3);


    matrix_mult(&inv_rot_0_3, rot_mat, &rot_3_6);

    theta[4] = acos((rot_3_6.p_data)[0]);
    theta[3] = asin(rot_3_6.p_data[3] / sin(theta[4]));
    theta[5] = asin(rot_3_6.p_data[1] / (sin(theta[4])));

    for(int i = 0; i < 6; i++){
        angles[i] = theta[i];
    }
}

void Algorithm6Dof::forwardKinematics(float *angles, coordinates *arm_position_points) {

    float short_hyp, hyp, long_hyp;
    float alfa, beta;

    // Origin
    arm_position_points[0].x = 0;
    arm_position_points[0].y = 0;
    arm_position_points[0].z = 0;

    // Base point (J1 revolute point)
    arm_position_points[1].x = 0;
    arm_position_points[1].y = 0;
    arm_position_points[1].z = linkMap[BASE_HEIGHT];

    // Shoulder point (J2 revolute point)
    arm_position_points[2].x = 0;
    arm_position_points[2].y = 0;
    arm_position_points[2].z = linkMap[BASE_HEIGHT] + linkMap[SHOULDER_HEIGHT];

    // Elbow point (J3 revolute point)
    short_hyp = cos(angles[1]) * linkMap[SHOULDER_LENGTH];

    arm_position_points[3].x = linkMap[BASE_HEIGHT]+ linkMap[SHOULDER_HEIGHT] + sin(angles[1]) * linkMap[SHOULDER_LENGTH];
    arm_position_points[3].y = short_hyp * cos(angles[0]);
    arm_position_points[3].z = short_hyp * sin(angles[0]);

    // Wrist center point
    hyp = sqrt(pow(linkMap[SHOULDER_LENGTH], 2) + pow(linkMap[ELBOW_LENGTH], 2) - 2*linkMap[SHOULDER_LENGTH]*linkMap[ELBOW_LENGTH]*cos(angles[2]));
    alfa = acos((pow(linkMap[ELBOW_LENGTH], 2) - pow(linkMap[SHOULDER_LENGTH], 2) - pow(hyp, 2))/(-2*linkMap[SHOULDER_LENGTH]*hyp));
    beta = angles[1] - alfa;
    long_hyp = hyp*cos(beta);

    arm_position_points[4].x = long_hyp*cos(angles[0]);
    arm_position_points[4].y = long_hyp*sin(angles[0]);
    arm_position_points[4].z = sin(beta)*hyp+linkMap[BASE_HEIGHT] + linkMap[SHOULDER_HEIGHT];

    float rot_0_6_d_col[3] = {
            sin(angles[4])*(sin(angles[0])*sin(angles[3]) - cos(angles[3])*(cos(angles[0])*cos(angles[1])*sin(angles[2]) + cos(angles[0])*cos(angles[2])*sin(angles[1]))) - cos(angles[4])*(cos(angles[0])*sin(angles[1])*sin(angles[2]) - cos(angles[0])*cos(angles[1])*cos(angles[2])),
            cos(angles[4])*(cos(angles[1])*sin(angles[2]) + cos(angles[2])*sin(angles[1])) + cos(angles[3])*sin(angles[4])*(cos(angles[1])*cos(angles[2]) - sin(angles[1])*sin(angles[2])),
            sin(angles[4])*(cos(angles[0])*sin(angles[3]) + cos(angles[3])*(cos(angles[1])*sin(angles[0])*sin(angles[2]) + cos(angles[2])*sin(angles[0])*sin(angles[1]))) + cos(angles[4])*(sin(angles[0])*sin(angles[1])*sin(angles[2]) - cos(angles[1])*cos(angles[2])*sin(angles[0]))
    };

    arm_position_points[5].x = arm_position_points[4].x + linkMap[EE_LENGTH]*rot_0_6_d_col[0];
    arm_position_points[5].y = arm_position_points[4].y + linkMap[EE_LENGTH]*rot_0_6_d_col[1];
    arm_position_points[5].z = arm_position_points[4].z + linkMap[EE_LENGTH]*rot_0_6_d_col[2];
}