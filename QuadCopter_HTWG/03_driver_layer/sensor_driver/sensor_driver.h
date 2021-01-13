//=====================================================================================================
// @file sensor_driver.h
//=====================================================================================================
//
// @brief API to interact with the sensors.
//
// Date                 Author                      Notes
// @date 31/05/2016     @author Tobias Grimm        Implementation
// @date 06/12/2020     @author Tomas Schweizer     Overall changes
//
// Source:
//
//
//=====================================================================================================

#ifndef __SENSOR_DRIVER_H__
#define	__SENSOR_DRIVER_H__

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

#include <stdint.h>

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/
//extern volatile EventGroupHandle_t gx_sensor_EventGroup;

extern volatile float gf_sensor_attitudeQuaternion[4];
extern volatile float gf_sensor_dotAttitudeQuaternion[4];
extern float gf_sensor_fusedAngles[3];
extern float gf_sensor_angularVelocity[3];
extern float gf_sensor_pressure;
extern float gf_sensor_altitude;
/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/

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


#endif // __SENSOR_DRIVER_H__

//=====================================================================================================
// End of file
//=====================================================================================================
