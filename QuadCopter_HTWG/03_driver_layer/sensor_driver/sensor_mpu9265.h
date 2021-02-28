/*===================================================================================================*/
/*  sensor_mpu9265.h                                                                                  */
/*===================================================================================================*/

/**
*   @file    sensor_mpu9265.h
*
*   @brief  API for the mpu9265
*
*   @details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>06/12/2021      <td>Tomas Schweizer     <td>Implementation
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*   TivaWare SensorLib
*/
/*====================================================================================================*/

#ifndef __SENSOR_MPU9265_H__
#define __SENSOR_MPU9265_H_

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/
#include <stdint.h>

#include "sensorlib/i2cm_drv.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/
/// Struct for MPU9265 and AK8963 raw data
typedef struct
{
    int16_t i16_accX;
    int16_t i16_accY;
    int16_t i16_accZ;
    int16_t i16_gyroX;
    int16_t i16_gyroY;
    int16_t i16_gyroZ;
    int16_t i16_magX;
    int16_t i16_magY;
    int16_t i16_magZ;

} MPU9265_AK8963_rawData_s;

/// Struct for Magnetometer AK8975_s integrated in MPU9250
typedef struct
{
    /// I2c master instance
    tI2CMInstance *ps_i2cMastInst;

    /// I2c device address
    uint8_t ui8_AK8693Address;

    /// State of the AK8963
    uint8_t ui8_AK8963State;

    /// Read Buffer
    uint8_t pui8_AK8963ReadBuffer[8];

    /// Write Buffer
    uint8_t pui8_AK8963WriteBuffer[8];

    /// Pointer to the function which is called when request is finished
    tSensorCallback *fp_AK8963Callback;

    /// Pointer to the callback data for the callbac function
    void *p_AK8963CallbackData;

} AK8963_s;


/// Struct for MPU9265
typedef struct
{
    /// I2c master instance
    tI2CMInstance *ps_i2cMastInst;

    /// Instannce of the Magnetometer
    AK8963_s s_AK8963Inst;

    /// I2c Address of MPU9250
    uint8_t ui8_MPU9265Address;

    /// State of the MPU9265
    uint8_t ui8_MPU9265State;

    /// Read Buffer
    uint8_t pui8_MPU9265ReadBuffer[24];

    /// Write Buffer
    uint8_t pui8_MPU9265WriteBuffer[8];

    /// Pointer to callback function
    tSensorCallback *fp_MPU9265Callback;

    /// Pointer to callback data of callback function
    void *p_MPU9265CallbackData;

} MPU9265_s;

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/
extern uint8_t MPU9265_Init(MPU9265_s *ps_inst, tI2CMInstance *ps_I2CMInst, uint8_t ui8_MPU9265Address,
                            tSensorCallback *fp_MPU9265Callback, void *p_MPU9265CallbackData);
extern uint8_t MPU9265_ReadData(MPU9265_s *ps_inst, tSensorCallback *fp_MPU9265Callback, void *p_MPU9265CallbackData);

extern void  MPU9265_AK8963_GetRawData(MPU9265_s *ps_inst, MPU9265_AK8963_rawData_s *s_rawData);

#endif /* _SENSOR_MPU925X_H_ */

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
