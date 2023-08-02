#ifndef RAN2_MCU_CPP_ALGORITHM6DOF_HPP
#define RAN2_MCU_CPP_ALGORITHM6DOF_HPP

#include <vector>
#include <unordered_map>
#include <cmath>

#include "../matrix_math/matrix_math.h"

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

typedef std::unordered_map<LINK, float32_t> LINK_MAP;

class Algorithm6Dof{
public:
    Algorithm6Dof(LINK_MAP link_map, float32_t* offsets);

    void forwardKinematics(float32_t* angles, coordinates* arm_position_points);
    void inverseKinematics(float32_t x, float32_t y, float32_t z, arm_matrix_instance_f32* rot_mat,
                           float32_t* angles);
private:
    LINK_MAP linkMap;
    float32_t* offsets;

    float32_t joint_data;
    coordinates arm_position_points[6];
};

#endif //RAN2_MCU_CPP_ALGORITHM6DOF_HPP
