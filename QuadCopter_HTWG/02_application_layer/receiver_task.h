/**
 * 		@file 	receiver_task.h
 * 		@brief	Receiver Task waits for asynchronous events
 *  			and handles/answers them
 *  			remote control, telemetrie, ...
 *//*	@author Tobias Grimm
 * 		@date 	31.05.2016	(last modified)
 */

#ifndef __RECEIVER_TASK_H__
#define	__RECEIVER_TASK_H__

/* ------------------------------------------------------------ */
/*					Include File Definitions					*/
/* ------------------------------------------------------------ */

#include <stdint.h>

//  FreeRTOS
#include "FreeRTOS.h"
#include "event_groups.h"

/* ------------------------------------------------------------ */
/*				   	Defines			    						*/
/* ------------------------------------------------------------ */

// Communication defines for Bits in gx_sync_EventBits (24 bits can be stored)
// Comm-Task waits for these bits and handles/answers them
// Bit order is irrelevant (but in range 0 ... fault_COUNT-1)
/** \brief	maximum Number of used receiver eventBits (max 24) */
#define receiver_COUNT						( 4 )
/** \brief	REMOTE_DATA receiver eventBit */
#define receiver_REMOTE_DATA				( 1 << 0 )
/** \brief	AUTOPILOT_DATA	 receiver eventBit */
#define receiver_AUTOPILOT_DATA				( 1 << 1 )
/** \brief	TELEMETRIE_DATA receiver eventBit */
#define receiver_TELEMETRIE_DATA			( 1 << 2 )
/** \brief	SENSOR_DATA receiver eventBit
 * 	\note	receiver_task doesn't wait for this bit (flight_task wait for it) */
#define receiver_SENSOR_DATA				( 1 << 3 )

/* ------------------------------------------------------------ */
/*				   	Type Definitions			    			*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*					API Procedure Declarations					*/
/* ------------------------------------------------------------ */

extern uint32_t ReceiverTask_Init(void);
extern void ReceiverTask_GetSetPoints(float f_setPoint[]);

#if	( setup_DEV_SUM_RECEIVS )
	extern void HIDE_Receive_SetEventName(uint32_t ui32_eventBit,const char* pc_name);
	extern void HIDE_Receive_DrawDisplay(void);
	extern void HIDE_Receive_Increment(uint32_t ui32_receiveEventBit);
#else
	#define HIDE_Receive_SetEventName(ui32_eventBit,pc_name)			// this define will be kicked off from the preprocessor
	#define HIDE_Receive_DrawDisplay							0		// 0 pointer
	#define HIDE_Receive_Increment(ui32_receiveEventBit)
#endif

/* ------------------------------------------------------------ */
/*					Global Variables							*/
/* ------------------------------------------------------------ */

extern volatile uint32_t 		   	gui32_receiver_flightStabInput;
extern float 						gf_receiver_setPoint[4];
extern volatile EventGroupHandle_t 	gx_receiver_eventGroup;

/* ------------------------------------------------------------ */

#endif // __RECEIVER_TASK_H__
