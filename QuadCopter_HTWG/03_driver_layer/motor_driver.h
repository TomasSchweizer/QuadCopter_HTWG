/*===================================================================================================*/
/*  motor_driver.h                                                                                  */
/*===================================================================================================*/

/**
*   @file   motor_driver.h
*
*   @brief  API to interact with the motors.
*
*   @details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>31/05/2016      <td>Tobias Grimm        <td>Implementation & last modifications through MAs
*   <tr><td>24/12/2020      <td>Tomas Schweizer     <td>Completely overworked because of new sensors
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*/
/*====================================================================================================*/

#ifndef __MOTOR_DRIVER_H__
#define	__MOTOR_DRIVER_H__

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

#include <stdint.h>

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/

#define motor_COUNT		            ( 4 )       ///< Number of Motors

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/


/// Structure to store information for one motor.
typedef struct motor_Data_s
{
        uint16_t ui16_setPoint;
        float f_current;
        float f_temperature;
        float  f_rpm;
        float f_voltage;
        uint8_t  ui8_state;

} motor_Data_s;

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/
extern volatile motor_Data_s gs_motor[motor_COUNT];     // Global motor data struct array to store information of all motors


/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/

extern void Motor_InitPeriph(void);
extern void Motor_InitMotor(void);
extern void Motor_OutputAll(void);
extern void Motor_StopAll(void);
extern void Motor_DrawDisplay(void);



#endif // __MOTOR_DRIVER_H__

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
