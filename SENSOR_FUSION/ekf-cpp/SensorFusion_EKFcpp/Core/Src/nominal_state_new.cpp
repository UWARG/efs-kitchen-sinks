/*
 * nominal_state_new.cpp
 *
 *  Created on: May 18, 2026
 *      Author: aahan
 */


#include "nominal_state_new.hpp"

#include "quaternions.hpp"
#include "arm_math.h"
#include "dsp/matrix_functions.h"

#include <cmath>

namespace {
constexpr float32_t kSmallGyroNorm = 1.0e-9f;
}

NominalState::NominalState() {
    SetIdentityQuaternion(quaternion_prev);
    SetIdentityQuaternion(quaternion_new);
}

NominalState::NominalState(const float32_t* quaternion_initial) {
    if (quaternion_initial == nullptr) {
        SetIdentityQuaternion(quaternion_prev);
    } else {
        NormalizeQuaternion(quaternion_initial, quaternion_prev);
    }

    CopyQuaternion(quaternion_prev, quaternion_new);
}

void NominalState::StateExtrapolation(const float32_t* gyro_new, const float32_t* gyro_prev, float32_t dt) {

	CopyQuaternion(quaternion_prev, quaternion_new);

	ExtrapolateQuaternion(
			gyro_new,
			gyro_prev,
			dt,
			quaternion_new);

};

void NominalState::ExtrapolateQauternion(const float32_t* gyro_new,
		const float32_t* gyro_prev,
        float32_t dt,
        float32_t* quaternion_out) {

    float32_t gyro_bar[3];

    arm_add_f32(gyro_new, gyro_prev, gyro_bar, 3);
    arm_scale_f32(gyro_bar, 0.5f, gyro_bar, 3);

    float32_t omegaMatrixData [16];
    ExpOmegaMatrix(gyro_bar, dt, omegaMatrixData);

    //must instantiate to use arm dot pro

    arm_matrix_instance_f32 omegaMatrix;
    arm_matrix_instance_f32 qPrev;
    arm_matrix_instance_f32 qNew;

    arm_mat_init_f32(&omegaMatrix, 4, 4, omegaMatrixData);
    arm_mat_init_f32(&qPrev, 4, 1, quaternion_prev);
    arm_mat_init_f32(&qNew, 4, 1, quaternion_new);

    arm_mat_mult_f32(&omegaMatrix, &qPrev, &qNew);

    NormalizeQuaternion(quaternion_out, quaternion_out);
};

void NominalState::ExpOmegaMatrix(float32_t *gyroBar, float32_t dt, float32_t *omegaMatrixOut) {

	   float32_t normGyro;
	   normalizeVector(gyroBar, normGyro);
	   float32_t normSigma = 0.5 * dt * normGyro;

	   int gx = gyroBar[0];
	   int gy = gyroBar[1];
	   int gz = gyroBar[2];

	   float32_t gyroMultMatrix[16] = {
			0.0f, -gx, -gy, -gz,
			gx, 0.0f, gz, -gy,
			gy, -gz, 0.0f, gx,
			gz, gy, -gx, 0.0f
	   };

	   const float32_t iMatrix[16] = {
	       1,0,0,0,
	       0,1,0,0,
	       0,0,1,0,
	       0,0,0,1
	   };

	    // Compute cosine and sine terms
	    float32_t cosTerm = arm_cos_f32(sigma);
	    float32_t sinTerm = arm_sin_f32(sigma);

	    // Scale and combine
	    float32_t temp1[16];
	    float32_t temp2[16];

	    // temp1 = cos(σ)*I
	    arm_scale_f32(I4, cosTerm, temp1, 16);

	    // temp2 = (sin(σ)/|ω|)*Ω
	    arm_scale_f32(gyroMultMatrix, sinTerm / gyroNorm, temp2, 16);

	    // omegaMatrixOut = temp1 + temp2
	    arm_add_f32(temp1, temp2, omegaMatrixOut, 16);
	};

void NominalState::CorrectState(const float32_t* small_angle_error) {

	float32_t quaternion_error[4];

	quaternion_error[0] = 1.0f;
	quaternion_error[1] = 0.5f * small_angle_error[0];
	quaternion_error[2] = 0.5f * small_angle_error[1];
	quaternion_error[3] = 0.5f * small_angle_error[2];

	float32_t quaternion_corrected[4];

	MultiplyQuaternions(quaternion_new, quaternion_error, quaternion_corrected);
	NormalizeQuaternion(quaternion_new, quaternion_corrected);

	CopyQuaternion(quaternion_corrected, quaternion_new);
	CopyQuaternion(quaternion_corrected, quaternion_prev);
	};


void NominalState::CopyQuaternion(const float32_t *q_in, float32_t *q_out) {
	for (uint32_t i = 0; i < QUATERNION_SIZE; ++i) {
		q_out[i] = q_in[i];
	};

void NominalState::SetIdentityQuaternion(float32_t *q_out); {
	q_out[0] = 1.0f;
	q_out[1] = 0.0f;
	q_out[2] = 0.0f;
	q_out[3] = 0.0f;
	}

};


