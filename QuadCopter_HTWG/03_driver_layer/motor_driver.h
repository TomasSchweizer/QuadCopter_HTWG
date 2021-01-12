//=====================================================================================================
// @file motor_driver.h
//=====================================================================================================
//
// @brief API to interact with the motors.
//
// Date                 Author                      Notes
// @date 31/05/2016     @author Tobias Grimm        Implementation
// @date 06/12/2020     @author Tomas Schweizer     Overall changes
//
// Source:
//
//
//=====================================================================================================

#ifndef __MOTOR_DRIVER_H__
#define	__MOTOR_DRIVER_H__

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

#include <stdint.h>

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/
/** \brief	Number of Motors */
#define motor_COUNT		            ( 4 )

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/

// TODO changed to uint16 from uint8 fro all read data test if working
/** \brief	structure to store information for one motor. */
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
extern volatile motor_Data_s gs_motor[motor_COUNT];
extern volatile uint32_t     gui32_motor_fault;

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/
extern void Motor_InitPeriph(void);
extern void Motor_InitMotor(void);
extern void Motor_OutputAll(void);
extern void Motor_StopAll(void);
extern void Motor_DrawDisplay(void);
extern void HIDE_Motor_SendDataOverUSB(void);


#endif // __MOTOR_DRIVER_H__

//=====================================================================================================
// End of file
//=====================================================================================================
