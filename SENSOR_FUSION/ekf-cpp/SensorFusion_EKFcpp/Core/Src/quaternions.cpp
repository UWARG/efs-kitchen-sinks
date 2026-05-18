/*
 * quaternions.cpp
 *
 *  Created on: Oct 8, 2025
 *      Author: aahan
 */

#include "quaternions.hpp"

#include "arm_math.h"
#include "dsp/quaternion_math_functions.h"
#include "dsp/matrix_functions.h"

#include <cmath>

//namespace {
//
//constexpr float32_t kEpsilonNorm = 1.0e-9f;
//constexpr float32_t kEpsilonSmallAngle = 1.0e-12f;
//constexpr float32_t kPi = 3.14159265358979323846f;
//constexpr float32_t kRadToDeg = 180.0f / kPi;
//
//void SetIdentityQuaternion(float32_t* q_out) {
//    q_out[0] = 1.0f;
//    q_out[1] = 0.0f;
//    q_out[2] = 0.0f;
//    q_out[3] = 0.0f;
//}
//
//float32_t ClampFloat32(float32_t value, float32_t min_value, float32_t max_value) {
//    if (value < min_value) {
//        return min_value;
//    }
//
//    if (value > max_value) {
//        return max_value;
//    }
//
//    return value;
//}
//
//}  // namespace



void NormalizeQuaternion(const float32_t* q_in, float32_t* q_out) {
    float32_t norm_sq = 0.0f;
    arm_dot_prod_f32(q_in, q_in, 4, &norm_sq);

    if (norm_sq < kEpsilonNorm) {
        SetIdentityQuaternion(q_out);
        return;
    }

    arm_quaternion_normalize_f32(q_in, q_out, 1);
}



void BToIFrameRotMatrix(const float32_t* q_in, float32_t* C_out) {
    float32_t q[4];
    NormalizeQuaternion(q_in, q);

    const float32_t w = q[0];
    const float32_t x = q[1];
    const float32_t y = q[2];
    const float32_t z = q[3];

    /*
     * Rotation matrix from body frame to inertial frame.
     * C_out is row-major 3x3.
     */
    C_out[0] = 1.0f - 2.0f * y * y - 2.0f * z * z;
    C_out[1] = 2.0f * x * y - 2.0f * z * w;
    C_out[2] = 2.0f * x * z + 2.0f * y * w;

    C_out[3] = 2.0f * x * y + 2.0f * z * w;
    C_out[4] = 1.0f - 2.0f * x * x - 2.0f * z * z;
    C_out[5] = 2.0f * y * z - 2.0f * x * w;

    C_out[6] = 2.0f * x * z - 2.0f * y * w;
    C_out[7] = 2.0f * y * z + 2.0f * x * w;
    C_out[8] = 1.0f - 2.0f * x * x - 2.0f * y * y;
}


