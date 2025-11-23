/*
 * utils.cpp
 *
 *  Created on: Sep 30, 2025
 *      Author: aahan
 */

#include "arm_math.h"
#include "dsp/quaternion_math_functions.h"
#include "dsp/matrix_functions.h"
#include "dsp/matrix_functions_f16.h"
#include <cmath>
#include <stdexcept>

#include "utils.h"

const float32_t IDENTITY_QUATERNION[4] = {1.0,0.0,0.0,0.0};
const float32_t GRAVITY_INERTIAL[3] = {0.0,0.0,9.81};
const float32_t MAGNETOMETER_INERTIAL[3] = {1.0,0.0,0.0};

void NormalizeQuaternion(const float32_t* q_in, float32_t* q_out, uint32_t length){

    arm_quaternion_normalize_f32(q_in, q_out, length);

      // Manually check the norm of q_out to detect near-zero quaternions
//    float64_t norm_sq = q_out[0]*q_out[0] + q_out[1]*q_out[1] + q_out[2]*q_out[2] + q_out[3]*q_out[3];
//    float64_t norm;
//    arm_sqrt_f64(norm_sq, &norm);

//    if (norm < 1e-9) {
//        const float32_t identity_q[4] = {1.0, 0.0, 0.0, 0.0};
//        memcpy(q_out, identity_q, sizeof(identity_q));
//    }
}

void NormalizeVector(const float32_t* v_in, float32_t* v_out, uint32_t length){
    float32_t norm = 0;

    arm_dot_prod_f32(v_in, v_in, length, &norm);
    norm = sqrt(norm);

//    if (norm == 0) {
//        throw std::runtime_error("Cannot normalize zero vector");
//    }

    for(int i = 0; i < length; i++){
        v_out[i] = v_in[i]/norm;
    }
}

// Compute 3x3 skew-symmetric matrix from 3x1 vector v_in
// S(v) = [  0, -v_z,  v_y;
//          v_z,  0,  -v_x;
//         -v_y, v_x,   0 ]

void SkewSymmetric(const float32_t* v_in, float32_t* S_out) {
    // S_out is a 3x3 row-major matrix: 9 elements
    S_out[0] = 0.0;
    S_out[1] = -v_in[2];
    S_out[2] = v_in[1];

    S_out[3] = v_in[2];
    S_out[4] = 0.0;
    S_out[5] = -v_in[0];

    S_out[6] = -v_in[1];
    S_out[7] = v_in[0];
    S_out[8] = 0.0;
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



