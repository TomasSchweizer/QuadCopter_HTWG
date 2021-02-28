/*===================================================================================================*/
/*  sensor_driver.h                                                                                  */
/*===================================================================================================*/

/**
*   @file   sensor_LidarLitev3HP.h
*
*   @brief  API to interact with the sensor LidarLitev3HP.
*
*   @details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>18/12/2021      <td>Tomas Schweizer     <td>Implementation
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*   - GARMIN github: https://github.com/garmin/LIDARLite_Arduino_Library
*/
/*====================================================================================================*/

#ifndef SENSOR_LIDARLITEV3HP_H_
#define SENSOR_LIDARLITEV3HP_H_

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

/// Struct for LidarLitev3HP instance
typedef struct
{
    /// I2c master instance
    tI2CMInstance *ps_i2cMastInst;

    /// I2c device address
    uint8_t ui8_LidarLitev3HPAddress;

    /// State of the lidar lite
    uint8_t ui8_LidarLitev3HPState;

    /// Read buffer
    uint8_t pui8_LidarLitev3HPReadBuffer[8];

    /// Write bufffer
    uint8_t pui8_LidarLitev3HPWriteBuffer[8];

    /// Pointer to the function which is called when request is finished
    tSensorCallback *fp_LidarLitev3HPCallback;

    /// Pointer to the callbackdata of the function
    void *p_LidarLitev3HPCallbackData;

} LidarLitev3HP_s;

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/
extern uint8_t LidarLitev3HP_Init(LidarLitev3HP_s *ps_li_inst, tI2CMInstance *ps_I2CMInst,
                                  uint8_t ui8_LidarLitev3HPAddress, tSensorCallback *fp_LidarLitev3HPCallback,
                                  void *p_LidarLitev3HPCallbackData);
extern uint8_t LidarLitev3HP_ReadData(LidarLitev3HP_s *ps_li_inst, tSensorCallback *fp_LidarLitev3HPCallback,
                               void *p_LidarLitev3HPCallbackData);
extern void LidarLitev3HP_GetRawData(LidarLitev3HP_s *ps_li_inst, uint16_t *ui16_distance, float *f_distance);

#endif /* SENSOR_LIDARLITEV3HP_H_ */

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
