/**
 * 		@file 	prioritys.h
 * 		@brief	Set the prioritys for all Tasks and Interrupts,
 * 				every task gets an additional unique Number
 *//*	@author Tobias Grimm
 * 		@date 	01.06.2016	(last modified)
 */

#ifndef __PRIORITYS_H__
#define __PRIORITYS_H__

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <stdint.h>

// setup
#include "qc_setup.h"

/* ------------------------------------------------------------ */
/*				Task prioritys									*/
/* ------------------------------------------------------------ */

#define priority_COUNT					( 5 )				// priority 0 ... priority_COUNT-1

//  (A higher Number is a higher priotity)
#define priority_RTOS_DAEMON_TASK		(priority_COUNT - 1)
#define priority_FLIGHT_TASK			(priority_COUNT - 2)
#define priority_RECEIVER_TASK      	(priority_COUNT - 3)
#define priority_COMMAND_TASK			(priority_COUNT - 4)
#define priority_IDLE_TASK				( 0 )				// this macro is never used (but FreeRTOS gives the idle-Task priority 0)


/* ------------------------------------------------------------ */
/*				NVIC prioritys									*/
/* ------------------------------------------------------------ */

// (0=high prio, 7=low prio)
// only upper 3 bits are used in NVIC -> shift them up 5 times
// prio 0 can never call API from ISR!!! (see below)
#define priority_REMOTE_ISR				( 4 << 5 )
#define priority_DEBUGGER_ISR			( 6 << 5 )
#define priority_USB_ISR                ( 6 << 5 )
#define priority_SENSOR_ISR      		( 3 << 5 )
#define priority_SENSOR_ALT_ISR         ( 4 << 5 )
#define priority_MOTOR_ISR				( 5 << 5 )

// all ISRs with a priority number >= priority_MAX_API_CALL_INTERRUPT
// can call FreeRTOS API functions from ISR
#define priority_KERNEL         		( 7 << 5 )   // Priority 7, this should be the lowest priority.
#define priority_MAX_API_CALL_INTERRUPT ( 2 << 5 )	 // Priotity 0 is not allowed!

/* ------------------------------------------------------------ */
/*				Task Numbers									*/
/* ------------------------------------------------------------ */

// Every Task gets a unique Number (-> take care when two Tasks share the same priority!)
// starting from 0 to priority_NUM_COUNT-1
// when setup_DEV_WORKLOAD is used to display the workload, the highest Number will be at the top of the display
#define priority_NUM_COUNT          		 priority_COUNT

#define priority_NUM_RTOS_DAEMON_TASK        priority_RTOS_DAEMON_TASK
#define priority_NUM_IDLE_TASK               priority_IDLE_TASK
#define priotity_NUM_COMMAND_TASK            priority_COMMAND_TASK
#define priority_NUM_RECIVER_TASK            priority_RECEIVER_TASK
#define priority_NUM_FLIGHT_TASK             priority_FLIGHT_TASK

#endif // __PRIORITYS_H__
