#ifndef RAN2_MCU_CPP_ALGORITHM6DOF_HPP
#define RAN2_MCU_CPP_ALGORITHM6DOF_HPP

#include <vector>
#include <unordered_map>
#include <cmath>

extern "C" {
    #include "../../libraries/NumericalMath/include/matrix_math.h"
}

typedef enum {
    BASE_HEIGHT,
    SHOULDER_HEIGHT,
    SHOULDER_LENGTH,
    ELBOW_LENGTH,
    EE_LENGTH
} LINK;

typedef struct{
    float x;
    float y;
    float z;
} coordinates;

typedef std::unordered_map<LINK, float> LINK_MAP;

class Algorithm6Dof{
public:
    Algorithm6Dof(LINK_MAP link_map, float* offsets);

    void createRotationMatrix(float yaw, float pitch, float roll, matrix_f32* rot_mat);
    void forwardKinematics(float* angles, coordinates* position_points);
    void inverseKinematics(float x, float y, float z, matrix_f32* rot_mat, float* angles);
private:
    LINK_MAP linkMap;
    float* offsets;

    float joint_data;
    coordinates arm_position_points[6];
};

#endif //RAN2_MCU_CPP_ALGORITHM6DOF_HPP
