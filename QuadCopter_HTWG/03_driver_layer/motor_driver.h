/**
 * 		@file 	motor_driver.h
 * 		@brief	Funktions to init, output and read the motor.
 *//*	@author Tobias Grimm
 * 		@date 	30.05.2016	(last modified)
 */

#ifndef __MOTOR_DRIVER_H__
#define	__MOTOR_DRIVER_H__

/* ------------------------------------------------------------ */
/*					Include File Definitions					*/
/* ------------------------------------------------------------ */

#include <stdint.h>

/* ------------------------------------------------------------ */
/*				   	Defines			    						*/
/* ------------------------------------------------------------ */

/** \brief	Number of Motors */
#define motor_COUNT		( 4 )

//  ### States of the motor
//#define motor_STATE_RUNNING     	  		  ( 255 )
//#define motor_STATE_NOT_RUNNING 	  		  ( 250 )						  /* ready for operation */
//#define motor_STATE_CURRENT_LIMIT(state)	  ( state > 50  && state < 247 )  /* 247==96,9% ... 50==19,6%   (255==100%) */
//#define motor_STATE_STARTING	      		  ( 40 )

/* ------------------------------------------------------------ */
/*				   	Type Definitions			    			*/
/* ------------------------------------------------------------ */

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

/* ------------------------------------------------------------ */
/*					API Procedure Declarations					*/
/* ------------------------------------------------------------ */

extern void Motor_InitPeriph(void);
extern void Motor_InitMotor(void);
extern void Motor_OutputAll(void);
extern void Motor_StopAll(void);
extern void Motor_Read(uint8_t ui8_motorNr);
extern void Motor_ReadAll(void);
extern void Motor_DrawDisplay(void);
void HIDE_Motor_SendDataOverUSB(void);


/* ------------------------------------------------------------ */
/*					Global Variables							*/
/* ------------------------------------------------------------ */

extern volatile motor_Data_s gs_motor[motor_COUNT];
extern volatile uint32_t 	 gui32_motor_fault;

/* ------------------------------------------------------------ */

#endif // __MOTOR_DRIVER_H__
