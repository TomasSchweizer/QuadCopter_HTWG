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

// TODO test and check
extern void    correctIMUOffset(float* sensor_data,uint8_t calibrate);
extern void    IMUAxis2QCAxis(float* sensor_data);
extern void    convertIMUData(float* sensor_data);

extern void    Sensor_CalibrateStop(void);
extern uint8_t Sensor_IsCalibrateReady(void);
extern uint8_t Sensor_IsCalibrateRequired(void);
extern void    Sensor_DrawDisplay(void);
extern void    HIDE_Sensor_SendDataOverUSB (void);

/* ------------------------------------------------------------ */
/*					Global Variables							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */

#endif // __SENSOR_DRIVER_H__
