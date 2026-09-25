/*
 * nominal_state.h
 *
 *  Created on: Oct 8, 2025
 *      Author: aahan
 */

#ifndef NOMINAL_STATE_H_
#define NOMINAL_STATE_H_

#include <utils.hpp>
#include "arm_math.h"

class NominalState {
public:
    NominalState();   // no parameters, everything initialized internally

    void Update();

private:
    // --- Data backing arrays ---
    float32_t displacementData[3];
    float32_t velocityData[3];
    float32_t quaternionData[4];
    float32_t gyroPrevData[3];
    float32_t accelPrevData[3];
    float32_t gravityData[3];

    // --- CMSIS matrix instances ---
    arm_matrix_instance_f32 prevDisplacement;
    arm_matrix_instance_f32 prevVelocity;
    arm_matrix_instance_f32 prevQuaternion;
    arm_matrix_instance_f32 prevGyroMeasurement;
    arm_matrix_instance_f32 prevAccelMeasurement;
    arm_matrix_instance_f32 gravityInertialMat;

    // --- Internal helper methods ---
    void UpdateQuaternion(const float32_t* gyroBar, float32_t dt, float32_t* quaternionOut);
    void ExpOmegaMatrix(const float32_t* gyroBar, float32_t dt, float32_t omegaMatrixOut[16]);
    void UpdateVelocity(const float32_t* quaternionNew, const float32_t* accelBodyNew, float32_t dt, float32_t* velocityOut);
    void UpdateDisplacement(const float32_t* velocityNew, float32_t dt, float32_t* displacementOut);
};

#endif  // NOMINAL_STATE_H_
