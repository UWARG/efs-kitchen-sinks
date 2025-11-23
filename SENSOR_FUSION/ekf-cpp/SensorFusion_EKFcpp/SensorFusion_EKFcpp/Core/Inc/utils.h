/*
 * utils.h
 *
 *  Created on: Oct 8, 2025
 *      Author: aahan
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "arm_math.h"
#include <cstdint>

extern const float32_t IDENTITY_QUATERNION[4];
extern const float32_t GRAVITY_INERTIAL[3];
extern const float32_t MAGNETOMETER_INERTIAL[3];

void NormalizeQuaternion(const float32_t* q_in, float32_t* q_out, uint32_t length);
void NormalizeVector(const float32_t* v_in, float32_t* v_out);
void SkewSymmetric(const float32_t* v_in, float32_t* S_out);
void BToIFrameRotMatrix(const float32_t* q_in, float32_t* C_out);

#endif  // UTILS_H_

