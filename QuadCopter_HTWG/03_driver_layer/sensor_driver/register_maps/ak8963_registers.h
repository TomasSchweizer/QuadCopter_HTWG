/*===================================================================================================*/
/*  ak8963_registers.h                                                                               */
/*===================================================================================================*/

/**
*   @file   ak8963_registers.h
*
*   @brief  List of used registers from ak8963 magnetometer
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
*   - Sensor data sheet
*   - TivaWare sensorlib
*/
/*====================================================================================================*/

#ifndef AK8963_REGISTERS_H_
#define AK8963_REGISTERS_H_

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/

#define AK8963_ADDRESS              0x0C        ///< i2c address

// registers
#define AK8963_HXL                  0x03        ///< X-axis LSB output register
#define AK8963_HXH                  0x04        ///< X-axis MSB output register
#define AK8963_HYL                  0x05        ///< Y-axis LSB output register
#define AK8963_HYH                  0x06        ///< Y-axis MSB output register
#define AK8963_HZL                  0x07        ///< Z-axis LSB output register
#define AK8963_HZH                  0x08        ///< Z-axis MSB output register
#define AK8963_CNTL1                0x0A        ///< Control register
#define AK8963_CNTL2                0x0B        ///< Control 2 register
#define AK8963_ASTC                 0x0C        ///< Self-test register
#define AK8963_ASAX                 0x10        ///< X-axis sensitivity register
#define AK8963_ASAY                 0x11        ///< Y-axis sensitivity register
#define AK8963_ASAZ                 0x12        ///< Z-axis sensitivity register

// bits in registers
#define AK8963_CNTL2_SRST           0x01        ///< Register reset
#define AK8963_CNTL_MODE_CONT_2     0x06        ///< Continuous measurement mode 2 (100Hz)
#define AK8963_CNTL_MODE_SELF_TEST  0x08        ///< Self-test mode
#define AK8963_CNTL_BITM_16BIT      0x10        ///< 16-bit output
#define AK8963_CNTL_MODE_FUSE_ROM   0x0F        ///< Fuse ROM access mode
#define AK8963_ASTC_SELF            0x40        ///< Generate magnetic field for self-test

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/


#endif /* AK8963_REGISTERS_H_ */

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
