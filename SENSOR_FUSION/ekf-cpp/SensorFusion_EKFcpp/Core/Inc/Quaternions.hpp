/*
 * quaternions.hpp
 *
 *  Created on: Oct 8, 2025
 *      Author: aahan
 */

#ifndef QUATERNIONS_HPP_
#define QUATERNIONS_HPP_

#include "arm_math.h"
#include <cstdint>

void MultiplyQuaternions(const float32_t* q1, const float32_t* q2, float32_t* q_out);

void InverseQuaternion(const float32_t* q_in, float32_t* q_out);

void NormalizeQuaternion(const float32_t* q_in, float32_t* q_out);

void AverageQuaternions(const float32_t* q1, const float32_t* q2, float32_t* q_out);

void BToIFrameRotMatrix(const float32_t* q_in, float32_t* C_out);

void IToBFrameRotMatrix(const float32_t* q_in, float32_t* C_out);

void QuaternionExponential(const float32_t* rotation_vector, float32_t* q_out);

void RotateVector(const float32_t* v_in, const float32_t* q_in, float32_t* v_out);

float32_t AngularDistanceDegrees(const float32_t* q_true, const float32_t* q_est);

void QuatToEuler(const float32_t* q_in, float32_t* euler_out);

#endif  // QUATERNIONS_HPP_
