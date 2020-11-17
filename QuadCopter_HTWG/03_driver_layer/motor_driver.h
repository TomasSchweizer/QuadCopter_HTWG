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

/** \brief	structure to store information for one motor. */
typedef struct motor_Data_s
{
        uint16_t ui16_setPoint;              /**< [0 1] -> 0 motor off, 0xFFFF motor max speed */
        uint8_t  ui8_current;                /**< in 0.1 A steps, read back from BL */
        uint8_t  ui8_state;                  /**< read back from BL -> is less than 255 if BL is in current limit, not running (250) or starting (40)*/
        uint8_t  ui8_temperature;            /**< old BL-Ctrl will return a 255 here, the new version the temp. in Grad Celsius */
        uint8_t  ui8_rpm;                    /**< Raw value for RPM */
        uint8_t  ui8_voltage;                /**< in 0.1V (BL3 is limited to 255, BL2 is only low-byte) */
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

/* ------------------------------------------------------------ */
/*					Global Variables							*/
/* ------------------------------------------------------------ */

extern volatile motor_Data_s gs_motor[motor_COUNT];
extern volatile uint32_t 	 gui32_motor_fault;

/* ------------------------------------------------------------ */

#endif // __MOTOR_DRIVER_H__
