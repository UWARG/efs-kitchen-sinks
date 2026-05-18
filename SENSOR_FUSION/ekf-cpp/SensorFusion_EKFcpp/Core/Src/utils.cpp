/*
 * utils.cpp
 *
 *  Created on: Sep 30, 2025
 *      Author: aahan
 */

#include "utils.hpp"

#include "arm_math.h"
#include "dsp/matrix_functions.h"

const float32_t IDENTITY_QUATERNION[4] = {1.0f, 0.0f, 0.0f, 0.0f};
const float32_t GRAVITY_INERTIAL[3] = {0.0f, 0.0f, 9.81f};
const float32_t MAGNETOMETER_INERTIAL[3] = {1.0f, 0.0f, 0.0f};

bool NormalizeVector(const float32_t* v_in, float32_t* v_out, uint32_t length) {
    float32_t norm_sq = 0.0f;
    arm_dot_prod_f32(v_in, v_in, length, &norm_sq);

    if (norm_sq <= 0.0f) {
        for (uint32_t i = 0; i < length; ++i) {
            v_out[i] = 0.0f;
        }
        return false;
    }

    float32_t norm = 0.0f;
    if (arm_sqrt_f32(norm_sq, &norm) != ARM_MATH_SUCCESS || norm < 1.0e-12f) {
        for (uint32_t i = 0; i < length; ++i) {
            v_out[i] = 0.0f;
        }
        return false;
    }

    const float32_t inv_norm = 1.0f / norm;

    for (uint32_t i = 0; i < length; ++i) {
        v_out[i] = v_in[i] * inv_norm;
    }

    return true;
}

void SkewSymmetric(const float32_t* v_in, float32_t* S_out) {
    /*
     * S(v) = [  0, -vz,  vy
     *           vz,  0, -vx
     *          -vy, vx,   0 ]
     *
     * S_out is row-major 3x3.
     */
    S_out[0] = 0.0f;
    S_out[1] = -v_in[2];
    S_out[2] = v_in[1];

    S_out[3] = v_in[2];
    S_out[4] = 0.0f;
    S_out[5] = -v_in[0];

    S_out[6] = -v_in[1];
    S_out[7] = v_in[0];
    S_out[8] = 0.0f;
}

void EnsureSymmetricMatrix(const float32_t* A_in, float32_t* A_out, uint32_t rows, uint32_t cols) {
    /*
     * Equivalent to Python:
     * A_sym = 0.5 * (A + A.T)
     *
     * For the EKF covariance P, rows and cols should both be 9.
     */
    if (rows != cols) {
        return;
    }

    for (uint32_t r = 0; r < rows; ++r) {
        for (uint32_t c = 0; c < cols; ++c) {
            const uint32_t index_rc = r * cols + c;
            const uint32_t index_cr = c * cols + r;

            A_out[index_rc] = 0.5f * (A_in[index_rc] + A_in[index_cr]);
        }
    }
}

void CopyVector(const float32_t* v_in, float32_t* v_out, uint32_t length) {
    for (uint32_t i = 0; i < length; ++i) {
        v_out[i] = v_in[i];
    }
}

void BToIFrameRotMatrix(const float32_t* q_in, float32_t* C_out) {

    float32_t q[4];
    normalizeQuaternion(q_in, q);

    float32_t w = q[0];
    float32_t x = q[1];
    float32_t y = q[2];
    float32_t z = q[3];

    C_out[0] = 1 - 2*y*y - 2*z*z;
    C_out[1] = 2*x*y - 2*z*w;
    C_out[2] = 2*x*z + 2*y*w;
    C_out[3] = 2*x*y + 2*z*w;
    C_out[4] = 1 - 2*x*x - 2*z*z;
    C_out[5] = 2*y*z - 2*x*w;
    C_out[6] = 2*x*z - 2*y*w;
    C_out[7] = 2*y*z + 2*x*w;
    C_out[8] = 1 - 2*x*x - 2*y*y;
}



