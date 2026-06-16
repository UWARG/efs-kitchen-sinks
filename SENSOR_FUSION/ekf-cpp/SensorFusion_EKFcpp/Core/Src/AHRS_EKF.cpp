/*
 * ahrs_esmekf.cpp
 *
 *  Created on: May 18, 2026
 *      Author: aahan
 */

#include "ahrs_ekf.hpp"

#include "utils.hpp"
#include "quaternions.hpp"

#include "arm_math.h"
#include "dsp/matrix_functions.h"

AHRS_ESMEKF::AHRS_ESMEKF(
    const float32_t* gyro_initial,
    const float32_t* accel_initial,
    const float32_t* mag_initial,
    const float32_t* quaternion_initial,

    float32_t gyro_cov,
    float32_t accel_cov,
    float32_t magnetometer_cov,
    float32_t gyro_bias_cov,
    float32_t accel_bias_cov,

    float32_t accel_gate_threshold,
    float32_t magnetometer_gate_threshold,

    float32_t p_init_att,
    float32_t p_init_bias,

    const float32_t* gravity_inertial_in,
    const float32_t* magnetometer_inertial_in
)
    : measurements(gyro_initial, accel_initial, mag_initial),
      nominal_state(quaternion_initial),
      accel_gate_threshold_(accel_gate_threshold),
      magnetometer_gate_threshold_(magnetometer_gate_threshold) {
		SetDiagonal3(gyro_cov_mat, gyro_cov);
	    SetDiagonal3(accel_cov_mat, accel_cov);
	    SetDiagonal3(magnetometer_cov_mat, magnetometer_cov);
	    SetDiagonal3(gyro_bias_cov_mat, gyro_bias_cov);
	    SetDiagonal3(accel_bias_cov_mat, accel_bias_cov);

	    SetZero(error_state, ERROR_STATE_SIZE);
	    SetZero(kalman_gain, ERROR_STATE_SIZE * MEASUREMENT_SIZE);

	    SetZero(P, ERROR_STATE_SIZE * ERROR_STATE_SIZE);

	    // P[0:3, 0:3] = p_init_att * I
	    // P[3:6, 3:6] = p_init_bias * I
	    // P[6:9, 6:9] = p_init_bias * I
	    for (uint32_t i = 0; i < VECTOR_SIZE; ++i) {
	        P[i * ERROR_STATE_SIZE + i] = p_init_att;
	        P[(i + 3) * ERROR_STATE_SIZE + (i + 3)] = p_init_bias;
	        P[(i + 6) * ERROR_STATE_SIZE + (i + 6)] = p_init_bias;
	    }

	    if (gravity_inertial_in != nullptr) {
	        CopyVector3(gravity_inertial_in, gravity_inertial);
	    } else {
	        CopyVector3(GRAVITY_INERTIAL, gravity_inertial);
	    }

	    if (magnetometer_inertial_in != nullptr) {
	        CopyVector3(magnetometer_inertial_in, magnetometer_inertial);
	    } else {
	        CopyVector3(MAGNETOMETER_INERTIAL, magnetometer_inertial);
	    }
}

void AHRS_ESMEKF::StateExtrapolation(const float32_t* gyro_new, float32_t dt) {
    // TODO: Update gyro measurements.
    // TODO: Extrapolate nominal quaternion state.
    // TODO: Propagate covariance P.
}

bool AHRS_ESMEKF::CorrectionAccelerometer(const float32_t* accelerometer_new) {
    // TODO: Update accelerometer measurement.
    // TODO: Compute predicted accelerometer measurement.
    // TODO: Compute innovation.
    // TODO: Build accelerometer H matrix.
    // TODO: Apply Kalman update.

    return false;
}

bool AHRS_ESMEKF::CorrectionMagnetometer(const float32_t* magnetometer_new) {
    // TODO: Normalize magnetometer measurement.
    // TODO: Update magnetometer measurement.
    // TODO: Compute predicted magnetometer measurement.
    // TODO: Compute innovation.
    // TODO: Build magnetometer H matrix.
    // TODO: Apply Kalman update.

    return false;
}

void AHRS_ESMEKF::StateTransitionMatrix(float32_t dt, float32_t* Phi_out) {
    // TODO: Compute Phi = I + dt * F + 0.5 * dt^2 * F^2.
}

void AHRS_ESMEKF::ErrorStateGradientMatrixF(float32_t* F_out) {
    // TODO: Build 9x9 error-state gradient matrix F.
}

void AHRS_ESMEKF::ProcessNoiseCovMatrix(float32_t dt, float32_t* Q_out) {
    // TODO: Build 9x9 process noise covariance matrix Q.
}

bool AHRS_ESMEKF::ApplyUpdate(
    const float32_t* y,
    const float32_t* H,
    const float32_t* R,
    float32_t gate_threshold
) {
    // TODO: Compute S = H * P * H^T + R.
    // TODO: Invert S.
    // TODO: Apply Mahalanobis gating.
    // TODO: Compute Kalman gain.
    // TODO: Compute error state.
    // TODO: Update covariance.
    // TODO: Correct nominal state.
    // TODO: Update accumulated biases.
    // TODO: Apply reset Jacobian.

    return false;
}

void AHRS_ESMEKF::SetZero(float32_t* data, uint32_t length) {
    arm_fill_f32(0.0f, data, length);
}

void AHRS_ESMEKF::SetIdentity(float32_t* data, uint32_t size) {
    SetZero(data, size * size);

    for (uint32_t i = 0; i < size; ++i) {
        data[i * size + i] = 1.0f;
    }
}

void AHRS_ESMEKF::SetDiagonal3(float32_t* matrix_out, float32_t value) {
    SetZero(matrix_out, VECTOR_SIZE * VECTOR_SIZE);

    matrix_out[0] = value;
    matrix_out[4] = value;
    matrix_out[8] = value;
}

void AHRS_ESMEKF::CopyVector3(const float32_t* in, float32_t* out) {
    arm_copy_f32(in, out, VECTOR_SIZE);
}

void AHRS_ESMEKF::CopyMatrix(const float32_t* in, float32_t* out, uint32_t length) {
    arm_copy_f32(in, out, length);
}

void AHRS_ESMEKF::SymmetrizeSquareMatrixInPlace(float32_t* matrix, uint32_t size) {
    for (uint32_t row = 0; row < size; ++row) {
        for (uint32_t col = row + 1; col < size; ++col) {
            const uint32_t rowColIndex = row * size + col;
            const uint32_t colRowIndex = col * size + row;

            const float32_t averageValue =
                0.5f * (matrix[rowColIndex] + matrix[colRowIndex]);

            matrix[rowColIndex] = averageValue;
            matrix[colRowIndex] = averageValue;
        }
    }
}




