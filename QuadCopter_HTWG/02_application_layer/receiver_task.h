/*===================================================================================================*/
/*  receiver_task.h                                                                               */
/*===================================================================================================*/

/**
*   @file   receiver_task.h
*
*   @brief  API for the receiver task
*
*   @details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>31/05/2016      <td>Tobias Grimm        <td>Implementation & Last modification of MAs
*   <tr><td>20/01/2020      <td>Tomas Schweizer     <td>Changes to remote control start-up
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*   -
*/
/*====================================================================================================*/

#ifndef __RECEIVER_TASK_H__
#define	__RECEIVER_TASK_H__

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

// Standard libraries
#include <stdint.h>

//  FreeRTOS
#include "FreeRTOS.h"
#include "event_groups.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/

// Communication defines for Bits in gx_receiver_eventGroup (24 bits can be stored)
// Communication task waits for these bits and handles them.

/// Maximum number of used receiver eventBits (max 24)
#define receiver_COUNT						( 3 )
/// REMOTE_DATA receiver eventBit
#define receiver_REMOTE_DATA				( 1 << 0 )
///	AUTOPILOT_DATA	 receiver eventBit
#define receiver_AUTOPILOT_DATA				( 1 << 1 )
/// TELEMETRIE_DATA receiver eventBit
#define receiver_TELEMETRIE_DATA			( 1 << 2 )

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

extern volatile uint32_t            gui32_receiver_flightStabInput;     // Global variable for input method (receiver, telemetry, ...)
extern float                        gf_receiver_setPoint[4];            // Global variable for receiver setpoints
extern volatile EventGroupHandle_t  gx_receiver_eventGroup;             // Event group for receiver events

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/
extern uint32_t ReceiverTask_Init(void);
extern void ReceiverTask_GetSetPoints(float f_setPoint[]);

#if	( setup_DEV_SUM_RECEIVS ) || DOXYGEN
	extern void HIDE_Receive_SetEventName(uint32_t ui32_eventBit,const char* pc_name);
	extern void HIDE_Receive_DrawDisplay(void);
	extern void HIDE_Receive_Increment(uint32_t ui32_receiveEventBit);
#else
	#define HIDE_Receive_SetEventName(ui32_eventBit,pc_name)			// this define will be kicked off from the preprocessor
	#define HIDE_Receive_DrawDisplay							0		// 0 pointer
	#define HIDE_Receive_Increment(ui32_receiveEventBit)                // this define will be kicked off from the preprocessor
#endif


#endif // __RECEIVER_TASK_H__

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
