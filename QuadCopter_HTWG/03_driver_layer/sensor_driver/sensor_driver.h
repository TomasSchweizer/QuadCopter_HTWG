/**
 * 		@file 	sensor_driver.h
 * 		@brief	functions to interact with the 9-Axis Gyro+Accel+Magn sensor
 *//*	@author Tobias Grimm
 * 		@date 	31.05.2016	(last modified)
 */

#ifndef __SENSOR_DRIVER_H__
#define	__SENSOR_DRIVER_H__

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <stdint.h>

/* ------------------------------------------------------------ */
/*				   Type Definitions			    				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*					Variable Declarations						*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*					Procedure Declarations						*/
/* ------------------------------------------------------------ */

extern void    Sensor_InitPeriph(void);
extern void    Sensor_InitSensor(void);
extern void    Sensor_ReadAndFusion(void);
extern void    Sensor_CalibrateRequire(void);
extern void    Sensor_Calibrate(int32_t elapseTimeMS);
extern void    Sensor_CalibrateStop(void);
extern uint8_t Sensor_IsCalibrateReady(void);
extern uint8_t Sensor_IsCalibrateRequired(void);
extern void    Sensor_DrawDisplay(void);
extern void    HIDE_Sensor_SendDataOverUSB (void);

/* ------------------------------------------------------------ */
/*					Global Variables							*/
/* ------------------------------------------------------------ */

extern volatile float gf_sensor_attitudeQuaternion[4];
extern float gf_sensor_fusedAngles[3];
extern float gf_sensor_pressure;
extern float gf_sensor_altitude;
/* ------------------------------------------------------------ */

#endif // __SENSOR_DRIVER_H__
