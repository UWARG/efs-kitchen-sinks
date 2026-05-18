/*
 * ahrs_esmekf.hpp
 *
 *  Created on: May 18, 2026
 *      Author: aahan
 */

#ifndef AHRS_ESMEKF_HPP_
#define AHRS_ESMEKF_HPP_

#include "arm_math.h"
#include <cstdint>

#include "measurements.hpp"
#include "nominal_state_new.hpp"

class AHRS_ESMEKF {
public:
    static constexpr uint32_t ERROR_STATE_SIZE = 9;
    static constexpr uint32_t VECTOR_SIZE = 3;
    static constexpr uint32_t MEASUREMENT_SIZE = 3;

    AHRS_ESMEKF(
        const float32_t* gyro_initial = nullptr,
        const float32_t* accel_initial = nullptr,
        const float32_t* mag_initial = nullptr,
        const float32_t* quaternion_initial = nullptr,

        float32_t gyro_cov = 0.0f,
        float32_t accel_cov = 0.0f,
        float32_t magnetometer_cov = 0.0f,
        float32_t gyro_bias_cov = 0.0f,
        float32_t accel_bias_cov = 0.0f,

        float32_t accel_gate_threshold = 7.80f,
        float32_t magnetometer_gate_threshold = 16.3f,

        float32_t p_init_att = 0.1f,
        float32_t p_init_bias = 0.01f,

        const float32_t* gravity_inertial_in = nullptr,
        const float32_t* magnetometer_inertial_in = nullptr
    );

    void StateExtrapolation(const float32_t* gyro_new, float32_t dt);

    bool CorrectionAccelerometer(const float32_t* accelerometer_new);

    bool CorrectionMagnetometer(const float32_t* magnetometer_new);

    Measurements measurements;
    NominalState nominal_state;

    float32_t error_state[ERROR_STATE_SIZE];
    float32_t kalman_gain[ERROR_STATE_SIZE * MEASUREMENT_SIZE];

    float32_t P[ERROR_STATE_SIZE * ERROR_STATE_SIZE];

    float32_t gravity_inertial[VECTOR_SIZE];
    float32_t magnetometer_inertial[VECTOR_SIZE];

private:
    float32_t gyro_cov_mat[VECTOR_SIZE * VECTOR_SIZE];
    float32_t accel_cov_mat[VECTOR_SIZE * VECTOR_SIZE];
    float32_t magnetometer_cov_mat[VECTOR_SIZE * VECTOR_SIZE];
    float32_t gyro_bias_cov_mat[VECTOR_SIZE * VECTOR_SIZE];
    float32_t accel_bias_cov_mat[VECTOR_SIZE * VECTOR_SIZE];

    float32_t accel_gate_threshold_;
    float32_t magnetometer_gate_threshold_;

    void StateTransitionMatrix(float32_t dt, float32_t* Phi_out);

    void ErrorStateGradientMatrixF(float32_t* F_out);

    void ProcessNoiseCovMatrix(float32_t dt, float32_t* Q_out);

    bool ApplyUpdate(
        const float32_t* y,
        const float32_t* H,
        const float32_t* R,
        float32_t gate_threshold
    );

    void SetZero(float32_t* data, uint32_t length);

    void SetIdentity(float32_t* data, uint32_t size);

    void SetDiagonal3(float32_t* matrix_out, float32_t value);

    void CopyVector3(const float32_t* in, float32_t* out);

    void CopyMatrix(const float32_t* in, float32_t* out, uint32_t length);

    void SymmetrizeSquareMatrixInPlace(float32_t* matrix, uint32_t size);
};

#endif  // AHRS_ESMEKF_HPP_
