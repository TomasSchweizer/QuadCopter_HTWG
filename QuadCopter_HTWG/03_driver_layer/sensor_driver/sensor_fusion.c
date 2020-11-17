/*
 * sensor_fusion.c
 *
 *  Created on: 02.09.2015
 *      Author: Daniel Eckstein
 */

/* Includes
=============================================================================*/
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "stdio.h"
#include <math.h>
#include "math_quaternion.h"
#include "sensor_fusion.h"
#include "basic_filters.h"
#include "qc_math.h"

/* Defines
=============================================================================*/
#define OMEGA_HP (0)
// if true, quaternions are used for fusion
#define QUATERNION_FUSION (1)

#define dt 0.002f

#define X_ACCEL		0
#define Y_ACCEL		1
#define Z_ACCEL		2
#define X_GYRO		3
#define Y_GYRO		4
#define Z_GYRO		5
#define X_MAGNET	6
#define Y_MAGNET	7
#define Z_MAGNET	8
#define AXIS_COUNT	9

extern math_pidController_s s_pidRoll;
extern math_pidController_s s_pidPitch;
extern math_pidController_s s_pidYaw;

struct quat fuseAccelAndGyro(float* axisValues){
	/*
	 * fuses the given sensor data to one rotation quaternion (unit quaternion)
	 */

	// init static quaternion with some start value
	static struct quat q_filtered = { 9.99999875E-01f, 0.0f, 0.0f, 4.99999979E-04f };
	static struct quat q_int = { 9.99999875E-01f, 0.0f, 0.0f, 4.99999979E-04f };

	// match acceleration to quadrocopter
	float accel_vect[] = {
		axisValues[Y_ACCEL],
		-axisValues[X_ACCEL],
		-axisValues[Z_ACCEL]
	};

	// match omega to quadrocopter
	float omega_vect[] = {
		-axisValues[Y_GYRO],
		axisValues[X_GYRO],
		axisValues[Z_GYRO]
	};


//	debug_float[1] = x_angle_g;
//	debug_float[2] = y_angle_g;

	/******* Acceleration *******/
	//normalize acceleration vector
	normVector(accel_vect);

	// constant vector
	float accel_vect_0[] = ACCEL_VECT_0;

	// resulting quaternion
	struct quat q_rot_accel;

	// get unit quaternion from g vector and rotated vector
	q_rot_accel = quatFromVectors(accel_vect, accel_vect_0);

#if (OMEGA_HP == 1)


	omeg_hp.x[0] = omega_vect[0]*2.0f*DEG_PER_SEC*M_PI/(RES_REG*180.0f);
	omeg_hp.x[1] = omega_vect[1]*2.0f*DEG_PER_SEC*M_PI/(RES_REG*180.0f);
	highPassArray(&omeg_hp);

	s_pidRoll.omega	= omeg_hp.y[0];
	s_pidPitch.omega	= omeg_hp.y[1];

	debug_float[1] = omega_vect[1]*2.0f*DEG_PER_SEC*M_PI/(RES_REG*180.0f);
	debug_float[2] = s_pidPitch.omega;
	debug_float[3] = omeg_hp.alpha;

	s_pidYaw.omega	= omega_vect[2]*2.0f*DEG_PER_SEC*M_PI/(RES_REG*180.0f);

#else
	/******* Gyro *******/

	// calulate omega in deg/s
	// times two because in 16 bit there is a range of 1000 deg per sec
	s_pidRoll.f_errorDt	= omega_vect[0]*2.0f*DEG_PER_SEC*M_PI/(RES_REG*180.0f);
	s_pidPitch.f_errorDt= omega_vect[1]*2.0f*DEG_PER_SEC*M_PI/(RES_REG*180.0f);
	s_pidYaw.f_errorDt	= omega_vect[2]*2.0f*DEG_PER_SEC*M_PI/(RES_REG*180.0f);
#endif

	// integrate gyro values via time step dt
	float del_phi_vect[] = {
		omega_vect[0]*2.0f*DEG_PER_SEC*M_PI/(RES_REG*180.0f)*dt/2.0f,
		omega_vect[1]*2.0f*DEG_PER_SEC*M_PI/(RES_REG*180.0f)*dt/2.0f,
		omega_vect[2]*2.0f*DEG_PER_SEC*M_PI/(RES_REG*180.0f)*dt/2.0f
	};


	// roation quaternion for this cycle
	struct quat q_del_rot = euler2Quaternion(del_phi_vect);

	// integrate quaternion via multiplication
	struct quat q_rot_gyro = mulQuaternions(&q_filtered, &q_del_rot);

	float gyro_angles[3];
	float accel_angles[3];



	q_int = mulQuaternions(&q_int, &q_del_rot);
	quaternion2Euler(&q_int, gyro_angles);
	quaternion2Euler(&q_rot_accel, accel_angles);


	/******* Filter *******/
//	int accel_total =  abs(sensorData->x_accel) + abs(sensorData->y_accel) + abs(sensorData->z_accel);
//	if((8192 < accel_total) && (accel_total < 24576)){
		q_filtered.w = K_FILT * q_rot_gyro.w + (1.0f - K_FILT) * q_rot_accel.w;
		q_filtered.x = K_FILT * q_rot_gyro.x + (1.0f - K_FILT) * q_rot_accel.x;
		q_filtered.y = K_FILT * q_rot_gyro.y + (1.0f - K_FILT) * q_rot_accel.y;
		q_filtered.z = K_FILT * q_rot_gyro.z + (1.0f - K_FILT) * q_rot_accel.z;
//	}else{
//		q_filtered = q_rot_gyro;
//	}



	return q_filtered;


}

#include "qc_setup.h"
#if(setup_SENSOR_INV_ROLL&setup_SENSOR)
	#define INV_ROLL(f_roll)		(-f_roll)
#else
	#define INV_ROLL(f_roll)		( f_roll)
#endif
#if(setup_SENSOR_INV_PITCH&setup_SENSOR)
	#define INV_PITCH(f_pitch)		(-f_pitch)
#else
	#define INV_PITCH(f_pitch)		( f_pitch)
#endif
#if(setup_SENSOR_INV_YAW&setup_SENSOR)
	#define INV_YAW(f_yaw)			(-f_yaw)
#else
	#define INV_YAW(f_yaw)			( f_yaw)
#endif




void sensorFusion(float* axisValues, float* fused_angles){

#if (QUATERNION_FUSION==1)

	// Get actual quaternion via sensor fusion
	struct quat q_rot;
	q_rot = fuseAccelAndGyro(axisValues);

	// convert quaternion to euler angles
	float fusedAngles[3];
	quaternion2Euler(&q_rot, fusedAngles);


	// relate axis
	#if(setup_SENSOR_SWOP_ROLL_PITCH&setup_SENSOR)
		fused_angles[ROLL]  = INV_ROLL(fusedAngles[PITCH]);
		fused_angles[PITCH] = INV_PITCH(fusedAngles[ROLL]);
	#else
		fused_angles[ROLL]  = INV_ROLL(fusedAngles[ROLL]);
		fused_angles[PITCH] = INV_PITCH(fusedAngles[PITCH]);
	#endif

	fused_angles[YAW] = INV_YAW(fusedAngles[YAW]);

#else

	struct tilt_angles fusedAngles;
	fusedAngles = fuseAccelAndGyroFast(axisValues);

	// relate axis
	#if(setup_SENSOR_SWOP_ROLL_PITCH&setup_SENSOR)
		fused_angles[ROLL]  = INV_PITCH(fusedAngles.y);
		fused_angles[PITCH] = INV_ROLL(fusedAngles.x);
	#else
		fused_angles[ROLL]  = INV_ROLL(fusedAngles.x);
		fused_angles[PITCH] = INV_PITCH(fusedAngles.y);
	#endif
		fused_angles[YAW] = INV_YAW(fusedAngles.z);
#endif

}


