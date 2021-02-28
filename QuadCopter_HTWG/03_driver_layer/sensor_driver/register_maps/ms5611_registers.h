/*===================================================================================================*/
/*  ms5611_registers.h                                                                               */
/*===================================================================================================*/

/**
*   @file   ms5611_registers.h
*
*   @brief  List of used registers from Barometer
*
*   @details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>06/12/2020      <td>Tomas Schweizer     <td>Implementation
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*   - Sensor data sheet
*/
/*====================================================================================================*/

#ifndef MS5611_REGISTERS_H_
#define MS5611_REGISTERS_H_

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/
// i2c address of the MS5611
#define MS5611_ADDRESS                  0x77

// MS5611 commands and registers
#define MS5611_ADC_READ_COMMAND         0x00 ///< To read out conversion result
#define MS5611_RESET                    0x1E ///< Needs to be called at start-up
#define MS5611_PROM_READ                0xA2 ///< Needs to be read out to get 6 calibration values before start-up

/*
 * The calibration values lie at: (0xA0 is for the manufacturer only) 0xA2 to 0xAE
 * In this application we read them in 2 uint8 values at a time
 */

// options
#define MS5611_CONVER_PRESS_4096        0x48 ///< Conversion request for pressure with precision OSR=4096
#define MS5611_CONVER_TEMP_4096         0x58 ///< Conversion request for temperature with precision OSR=4096

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/

#endif /* MS5611_REGISTERS_H_ */

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
