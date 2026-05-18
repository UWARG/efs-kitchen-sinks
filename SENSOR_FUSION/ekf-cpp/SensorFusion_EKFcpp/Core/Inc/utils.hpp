/*
 * utils.hpp
 *
 *  Created on: Oct 8, 2025
 *      Author: aahan
 */

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include "arm_math.h"
#include <cstdint>

extern const float32_t IDENTITY_QUATERNION[4];
extern const float32_t GRAVITY_INERTIAL[3];
extern const float32_t MAGNETOMETER_INERTIAL[3];

bool NormalizeVector(const float32_t* v_in, float32_t* v_out, uint32_t length);

void SkewSymmetric(const float32_t* v_in, float32_t* S_out);

void EnsureSymmetricMatrix(const float32_t* A_in, float32_t* A_out, uint32_t rows, uint32_t cols);

void CopyVector(const float32_t* v_in, float32_t* v_out, uint32_t length);

#endif  // UTILS_HPP_
