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
#include "dsp/fast_math_functions.h"

#include <cmath>

namespace {

constexpr float32_t kEpsilonNorm = 1.0e-9f;
constexpr float32_t kEpsilonSmallAngle = 1.0e-12f;
constexpr float32_t kPi = 3.14159265358979323846f;
constexpr float32_t kRadToDeg = 180.0f / kPi;

void SetIdentityQuaternion(float32_t* q_out) {
    q_out[0] = 1.0f;
    q_out[1] = 0.0f;
    q_out[2] = 0.0f;
    q_out[3] = 0.0f;
}

float32_t AbsFloat32(float32_t value) {
    return value < 0.0f ? -value : value;
}

float32_t ClampFloat32(float32_t value, float32_t min_value, float32_t max_value) {
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

void CopyQuaternion(const float32_t* q_in, float32_t* q_out) {
    arm_copy_f32(q_in, q_out, 4);
}

}

void MultiplyQuaternions(const float32_t* q1, const float32_t* q2, float32_t* q_out) {
    arm_quaternion_product_single_f32(q1, q2, q_out);
}

void InverseQuaternion(const float32_t* q_in, float32_t* q_out) {
    float32_t norm_sq = 0.0f;
    arm_dot_prod_f32(q_in, q_in, 4, &norm_sq);

    if (norm_sq < kEpsilonNorm) {
        SetIdentityQuaternion(q_out);
        return;
    }

    arm_quaternion_inverse_f32(q_in, q_out, 1);
}

void NormalizeQuaternion(const float32_t* q_in, float32_t* q_out) {
    float32_t norm_sq = 0.0f;
    arm_dot_prod_f32(q_in, q_in, 4, &norm_sq);

    if (norm_sq < kEpsilonNorm) {
        SetIdentityQuaternion(q_out);
        return;
    }

    arm_quaternion_normalize_f32(q_in, q_out, 1);
}

void AverageQuaternions(const float32_t* q1, const float32_t* q2, float32_t* q_out) {
    float32_t q1_norm[4];
    float32_t q2_norm[4];

    NormalizeQuaternion(q1, q1_norm);
    NormalizeQuaternion(q2, q2_norm);

    /*
     * Relative rotation:
     * r = q1^-1 * q2
     */
    float32_t q1_inv[4];
    float32_t r[4];

    InverseQuaternion(q1_norm, q1_inv);
    MultiplyQuaternions(q1_inv, q2_norm, r);

    /*
     * Ensure shortest path.
     */
    if (r[0] < 0.0f) {
        arm_negate_f32(r, r, 4);
    }

    const float32_t r0 = ClampFloat32(r[0], -1.0f, 1.0f);

    const float32_t mu_norm = 2.0f * static_cast<float32_t>(std::acos(r0));

    if (mu_norm < kEpsilonSmallAngle) {
        CopyQuaternion(q1_norm, q_out);
        return;
    }

    const float32_t sin_half = arm_sin_f32(mu_norm * 0.5f);

    if (AbsFloat32(sin_half) < kEpsilonSmallAngle) {
        CopyQuaternion(q1_norm, q_out);
        return;
    }

    const float32_t mu_scale = mu_norm / sin_half;

    float32_t mu[3] = {
        r[1] * mu_scale,
        r[2] * mu_scale,
        r[3] * mu_scale
    };

    float32_t axis[3];
    arm_scale_f32(mu, 1.0f / mu_norm, axis, 3);

    const float32_t quarter_norm = mu_norm * 0.25f;

    float32_t r_n[4];
    r_n[0] = arm_cos_f32(quarter_norm);

    const float32_t sin_quarter = arm_sin_f32(quarter_norm);

    r_n[1] = axis[0] * sin_quarter;
    r_n[2] = axis[1] * sin_quarter;
    r_n[3] = axis[2] * sin_quarter;

    float32_t q_avg[4];
    MultiplyQuaternions(q1_norm, r_n, q_avg);

    NormalizeQuaternion(q_avg, q_out);
}

void BToIFrameRotMatrix(const float32_t* q_in, float32_t* C_out) {
    float32_t q[4];
    NormalizeQuaternion(q_in, q);

    const float32_t w = q[0];
    const float32_t x = q[1];
    const float32_t y = q[2];
    const float32_t z = q[3];

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

void IToBFrameRotMatrix(const float32_t* q_in, float32_t* C_out) {
    float32_t q_inv[4];
    InverseQuaternion(q_in, q_inv);

    BToIFrameRotMatrix(q_inv, C_out);
}

void QuaternionExponential(const float32_t* rotation_vector, float32_t* q_out) {
    float32_t theta_sq = 0.0f;
    arm_dot_prod_f32(rotation_vector, rotation_vector, 3, &theta_sq);

    float32_t theta = 0.0f;

    if (arm_sqrt_f32(theta_sq, &theta) != ARM_MATH_SUCCESS || theta < kEpsilonSmallAngle) {
        SetIdentityQuaternion(q_out);
        return;
    }

    const float32_t inv_theta = 1.0f / theta;

    float32_t unit_axis[3];
    arm_scale_f32(rotation_vector, inv_theta, unit_axis, 3);

    const float32_t half_theta = theta * 0.5f;

    const float32_t sin_half_theta = arm_sin_f32(half_theta);
    const float32_t cos_half_theta = arm_cos_f32(half_theta);

    q_out[0] = cos_half_theta;
    q_out[1] = unit_axis[0] * sin_half_theta;
    q_out[2] = unit_axis[1] * sin_half_theta;
    q_out[3] = unit_axis[2] * sin_half_theta;

    NormalizeQuaternion(q_out, q_out);
}

void RotateVector(const float32_t* v_in, const float32_t* q_in, float32_t* v_out) {
    float32_t R_data[9];
    BToIFrameRotMatrix(q_in, R_data);

    arm_matrix_instance_f32 R;
    arm_matrix_instance_f32 v;
    arm_matrix_instance_f32 result;

    arm_mat_init_f32(&R, 3, 3, R_data);
    arm_mat_init_f32(&v, 3, 1, const_cast<float32_t*>(v_in));
    arm_mat_init_f32(&result, 3, 1, v_out);

    arm_mat_mult_f32(&R, &v, &result);
}

float32_t AngularDistanceDegrees(const float32_t* q_true, const float32_t* q_est) {

    float32_t q_est_inv[4];
    float32_t q_err[4];

    InverseQuaternion(q_est, q_est_inv);
    MultiplyQuaternions(q_est_inv, q_true, q_err);
    NormalizeQuaternion(q_err, q_err);

    /*
     * Enforce shortest rotation.
     */
    if (q_err[0] < 0.0f) {
        arm_negate_f32(q_err, q_err, 4);
    }

    const float32_t w = ClampFloat32(q_err[0], -1.0f, 1.0f);

    const float32_t angle_rad = 2.0f * static_cast<float32_t>(std::acos(w));

    return angle_rad * kRadToDeg;
}

void QuatToEuler(const float32_t* q_in, float32_t* euler_out) {
    float32_t q[4];
    NormalizeQuaternion(q_in, q);

    const float32_t w = q[0];
    const float32_t x = q[1];
    const float32_t y = q[2];
    const float32_t z = q[3];

    /*
     * Roll: x-axis rotation.
     */
    const float32_t sinr_cosp = 2.0f * (w * x + y * z);
    const float32_t cosr_cosp = 1.0f - 2.0f * (x * x + y * y);

    const float32_t roll = static_cast<float32_t>(std::atan2(sinr_cosp, cosr_cosp));

    /*
     * Pitch: y-axis rotation.
     */
    const float32_t sinp = 2.0f * (w * y - z * x);

    float32_t pitch = 0.0f;

    if (AbsFloat32(sinp) >= 1.0f) {
        pitch = sinp >= 0.0f ? (kPi * 0.5f) : (-kPi * 0.5f);
    } else {
        pitch = static_cast<float32_t>(std::asin(sinp));
    }

    /*
     * Yaw: z-axis rotation.
     */
    const float32_t siny_cosp = 2.0f * (w * z + x * y);
    const float32_t cosy_cosp = 1.0f - 2.0f * (y * y + z * z);

    const float32_t yaw = static_cast<float32_t>(std::atan2(siny_cosp, cosy_cosp));

    euler_out[0] = roll;
    euler_out[1] = pitch;
    euler_out[2] = yaw;
}
