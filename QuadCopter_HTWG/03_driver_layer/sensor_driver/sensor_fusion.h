/*
 * sensor_fusion.h
 *
 *  Created on: 02.09.2015
 *      Author: Ecki
 */

#ifndef MY_MPU9250_SIMPLE_COMP_FILTER2_SRC_SENSOR_FUSION_H_
#define MY_MPU9250_SIMPLE_COMP_FILTER2_SRC_SENSOR_FUSION_H_

/* Defines
=============================================================================*/
#define ACCEL_VECT_0 {0.0f, 0.0f, -1.0f}

//#define K_FILT 0.99f
#define K_FILT 0.98f
#define DEG_PER_SEC 500.0f // resolution gyro in degree per second
#define RES_REG 65536.0f // resolution of sensor register (2^16)

#ifndef ANGLES_RPY
#define ANGLES_RPY
#define ROLL		0
#define PITCH		1
#define YAW			2
#endif


/*
 * Structures
 */
struct tilt_angles {
	float x;
	float y;
	float z;
};


/*
 * Global variables
 */

/*
 * Function prototypes
 */
struct quat;
struct rawData;
extern struct quat fuseAccelAndGyro(float* axisValues);
extern void sensorFusion(float* axisValues, float* fused_angles);


#endif /* MY_MPU9250_SIMPLE_COMP_FILTER2_SRC_SENSOR_FUSION_H_ */
