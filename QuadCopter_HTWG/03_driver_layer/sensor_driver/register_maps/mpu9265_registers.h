/*===================================================================================================*/
/*  mpu9265_registers.h                                                                              */
/*===================================================================================================*/

/**
*   @file   mpu9265_registers.h
*
*   @brief  List of used registers from mpu9265
*
*   @details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>09/03/2015      <td>Ecki                <td>Implementation
*   <tr><td>06/12/2020      <td>Tomas Schweizer     <td>Completely overworked
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*   - Sensor data sheet
*   - TivaWare SensorLib
*/
/*====================================================================================================*/

#ifndef MPU9265_REGISTERS_H_
#define MPU9265_REGISTERS_H_

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/

#define MPU9265_ADDRESS                     0x69    ///< i2c address, when AD0 Pin is connected to high (1)
#define MPU9265_WHO_AM_I                    0x75    ///< Who am i test register

// registers
#define MPU9265_SMPLRT_DIV                  0x19    ///< Sample rate divider register
#define MPU9265_GYRO_CONFIG                 0x1B    ///< Gyro configuration register
#define MPU9265_ACCEL_CONFIG                0x1C    ///< Accelerometer configuration register
#define MPU9265_INT_PIN_CFG                 0x37    ///< INT pin configuration register
#define MPU9265_ACCEL_XOUT_H                0x3B    ///< X-axis acceleration data MSB register
#define MPU9265_ACCEL_XOUT_L                0x3C    ///< X-axis acceleration data LSB register
#define MPU9265_ACCEL_YOUT_H                0x3D    ///< Y-axis acceleration data MSB register
#define MPU9265_ACCEL_YOUT_L                0x3E    ///< Y-axis acceleration data LSB register
#define MPU9265_ACCEL_ZOUT_H                0x3F    ///< Z-axis acceleration data MSB register
#define MPU9265_ACCEL_ZOUT_L                0x40    ///< Z-axis acceleration data LSB register
#define MPU9265_GYRO_XOUT_H                 0x43    ///< X-axis gyroscope data MSB register
#define MPU9265_GYRO_XOUT_L                 0x44    ///< X-axis gyroscope data LSB register
#define MPU9265_GYRO_YOUT_H                 0x45    ///< Y-axis gyroscope data MSB register
#define MPU9265_GYRO_YOUT_L                 0x46    ///< Y-axis gyroscope data LSB register
#define MPU9265_GYRO_ZOUT_H                 0x47    ///< Z-axis gyroscope data MSB register
#define MPU9265_GYRO_ZOUT_L                 0x48    ///< Z-axis gyroscope data LSB register
#define MPU9265_PWR_MGMT_1                  0x6B    ///< Power management 1 register

// bits in registers
#define MPU9265_PWR_MGMT_1_DEVICE_RESET     0x80    ///< Device reset
#define MPU9265_PWR_MGMT_1_INT_20MHZ_CLOCK  0x00    ///< Internal 20MHz clock
#define MPU9265_SMPLRT_DIV_S                0x00    ///< Sample rate divider = Internal sample rate / (1+SMPLRT_DIV)
#define MPU9265_ACCEL_CONFIG_AFS_SEL_2G     0x00    ///< Accelerometer full-scale range 2g
#define MPU9265_GYRO_CONFIG_FS_SEL_500      0x08    ///< Gyro full-scale range +/- 500 degrees/sec
#define MPU9265_INT_PIN_CFG_I2C_BYPASS_EN   0x02    ///< I2C bypass enable
#define MPU9265_PWR_MGMT_1_SLEEP            0x40    ///< Enter sleep mode status of register after reset

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/


#endif /* MPU9265_REGISTERS_H_ */

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
