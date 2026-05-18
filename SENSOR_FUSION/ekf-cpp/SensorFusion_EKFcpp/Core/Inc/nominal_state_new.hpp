/*
 * nominal_state_new.hpp
 *
 *  Created on: May 18, 2026
 *      Author: aahan
 */

#ifndef NOMINAL_STATE_HPP_
#define NOMINAL_STATE_HPP_

#include "arm_math.h"
#include <cstdint>

class NominalState {
public:
    static constexpr uint32_t QUATERNION_SIZE = 4;
    static constexpr uint32_t VECTOR_SIZE = 3;

    NominalState();

    explicit NominalState(const float32_t* quaternion_initial);

    void StateExtrapolation(
        const float32_t* gyro_new,
        const float32_t* gyro_prev,
        float32_t dt
    );

    void CorrectState(const float32_t* small_angle_error);

    float32_t quaternion_prev[QUATERNION_SIZE];
    float32_t quaternion_new[QUATERNION_SIZE];

private:
    void ExtrapolateQuaternion(
        const float32_t* gyro_new,
        const float32_t* gyro_prev,
        float32_t dt,
        float32_t* quaternion_out
    );

    void ExpOmegaMatrix(
        const float32_t* gyro_bar,
        float32_t dt,
        float32_t* omega_matrix_out
    );

    void CopyQuaternion(const float32_t* q_in, float32_t* q_out);
    void SetIdentityQuaternion(float32_t* q_out);
};

#endif  // NOMINAL_STATE_HPP_
