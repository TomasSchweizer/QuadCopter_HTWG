//=====================================================================================================
// @file sensor_LidarLitev3HP.h
//=====================================================================================================
//
// @brief API to interact with the sensor LidarLitev3HP.
//
// Date                 Author                      Notes
// @date 18/12/2020     @author Tomas Schweizer     Implementation
//
// Source:
// GARMIN github: https://github.com/garmin/LIDARLite_Arduino_Library
//
//=====================================================================================================

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

// Struct for LidarLitev3HP instance
typedef struct
{
    // I2c master instance
    tI2CMInstance *ps_i2cMastInst;

    // I2c device address
    uint8_t ui8_LidarLitev3HPAddress;

    // State of the MS5611
    uint8_t ui8_LidarLitev3HPState;

    uint8_t pui8_LidarLitev3HPReadBuffer[8];

    uint8_t pui8_LidarLitev3HPWriteBuffer[8];

    // pointer to the function which is called when request is finished
    tSensorCallback *fp_LidarLitev3HPCallback;

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
