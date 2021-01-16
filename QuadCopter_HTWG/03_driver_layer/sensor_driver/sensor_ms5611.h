//=====================================================================================================
// @file sensor_ms5611.h
//=====================================================================================================
//
// @brief API to interact with the sensor ms5611.
//
// Date                 Author                      Notes
// @date 06/12/2020     @author Tomas Schweizer     Implementation
//
// Source:
// YMCA-Quadcopter: http://www.brokking.net/ymfc-32_auto_main.html
//
//=====================================================================================================

#ifndef SENSOR_MS5611_H_
#define SENSOR_MS5611_H_

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
// Struct for MS5611 raw data
typedef struct
{
    uint32_t ui32_baroT;
    uint32_t ui32_baroP;

} MS5611_rawData_s;

// Struct for MS5611 instance
typedef struct
{
    // I2c master instance
    tI2CMInstance *ps_i2cMastInst;

    // I2c device address
    uint8_t ui8_MS5611Address;

    // State of the MS5611
    uint8_t ui8_MS5611State;

    // PROM calibration values
    uint8_t pui8_MS5611PROMValues[12];

    uint8_t pui8_MS5611ReadBuffer[8];

    uint8_t pui8_MS5611WriteBuffer[8];

    uint8_t ui8_MS5611DataType;

    // pointer to the function which is called when request is finished
    tSensorCallback *fp_MS5611Callback;

    void *p_MS5611CallbackData;
} MS5611_s;

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/
extern uint8_t MS5611_Init(MS5611_s *ps_ms_inst, tI2CMInstance *ps_I2CMInst, uint8_t ui8_MS5611Address,
                           tSensorCallback *fp_MS5611Callback, void *p_MS5611CallbackData);
extern uint8_t MS5611_ReadData(MS5611_s *ps_ms_inst, tSensorCallback *fp_MS5611Callback,
                               void *p_MS5611CallbackData, uint8_t ui8_dataChooser);
extern void MS5611_GetCalibrationValues(MS5611_s *ps_ms_inst, uint16_t *ui16_baroCalValues);
extern uint8_t MS5611_GetRawData(MS5611_s *ps_ms_inst, MS5611_rawData_s *s_rawData);

#endif /* SENSOR_MS5611_H_ */

//=====================================================================================================
// End of file
//=====================================================================================================

