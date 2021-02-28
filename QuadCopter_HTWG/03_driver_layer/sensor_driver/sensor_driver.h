/*===================================================================================================*/
/*  sensor_driver.h                                                                                  */
/*===================================================================================================*/

/**
*   @file   sensor_driver.h
*
*   @brief  API to interact with the sensors.
*
*   @details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>31/05/2016      <td>Tobias Grimm        <td>Implementation & last modifications through MAs
*   <tr><td>29/12/2020      <td>Tomas Schweizer     <td>Completely overworked because of new sensors
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*/
/*====================================================================================================*/

#ifndef __SENSOR_DRIVER_H__
#define	__SENSOR_DRIVER_H__

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

// Standard libraries
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

extern volatile float gf_sensor_attitudeQuaternion[4];      // Global variable for attitude quaternion
extern volatile float gf_sensor_dotAttitudeQuaternion[4];   // Global variable for derivative of attitude quaternion
extern float gf_sensor_fusedAngles[3];                      // Global variable for fused Tait-Bryan angles [rad] (roll, pitch, yaw)
extern float gf_sensor_angularVelocity[3];                  // Global variable for angular velocity [rad/s] calculated from quaternions
extern float gf_sensor_gyroAngularVelocity[3];              // Global variable for angular velocity [rad/s] directly from gyroscope
extern float gf_sensor_pressure;                            // Global variable for pressure [mPa] of barometer
extern float gf_sensor_baroAltitude;                        // Global variable for altitude [cm] measured by barometer
extern float gf_sensor_lidarAltitude;                       // Global variable for altitude [cm] measured by lidar

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

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
