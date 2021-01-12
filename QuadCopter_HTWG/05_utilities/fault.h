/**
 * 		@file 	fault.h
 * 		@brief	central place for all fault Events.
 *
 * 				Fault Events can get Names and counted how often they happened.
 *//*	@author Tobias Grimm
 * 		@date 	17.04.2016	(last modified)
 */

#ifndef __FAULT_H__
#define	__FAULT_H__

/* ------------------------------------------------------------ */
/*					Include File Definitions					*/
/* ------------------------------------------------------------ */

#include <stdint.h>

// utils
#include "count_edges.h"

// freeRTOS
#include "FreeRTOS.h"
#include "event_groups.h"

// setup
#include "qc_setup.h"

/* ------------------------------------------------------------ */
/*				   	Defines			    						*/
/* ------------------------------------------------------------ */

// Fault defines for Bits in gx_fault_EventGroup (24 bits can be stored)
// FlyStab-Task polls over this bits and decide what to do
// Bit order is irrelevant (but in range 0 ... fault_COUNT-1)
/** \brief	maximum Number of used fault eventBits (max 24)
 * 	\note	all fault eventBits are set by default */
#define fault_COUNT						( 5 )						// 24 bits max
/** \brief	REMOTE fault eventBit */
#define fault_REMOTE_CONTROL			( 1 << 0 )
/** \brief	AUTOPILOT fault eventBit */
#define fault_AUTOPILOT					( 1 << 1 )
/** \brief	TELEMETRIE fault eventBit */
#define fault_TELEMETRIE				( 1 << 2 )
/** \brief	MOTOR fault eventBit */
#define fault_MOTOR						( 1 << 3 )
/** \brief	SENSOR fault eventBit */
#define fault_SENSOR					( 1 << 4 )



/* ------------------------------------------------------------ */
/*				   	Type Definitions			    			*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*					API Procedure Declarations					*/
/* ------------------------------------------------------------ */

extern uint32_t Fault_Init(void);
extern void HIDE_Fault_DrawDisplay(void);

#if	( setup_DEV_SUM_FAULTS ) || DOXYGEN
	extern void HIDE_Fault_Increment(uint32_t ui32_faultEventBit,uint32_t ui32_faultBitValue);
	extern void HIDE_Fault_SetEventName(uint32_t ui32_eventBit,const char* pc_name);
	extern void HIDE_Fault_DrawDisplay();
#else
	#define HIDE_Fault_Increment(faultEventBit,faultBitValue)			// this define will be kicked off from the preprocessor
	#define HIDE_Fault_SetEventName(ui32_eventBit,pc_name)				// this define will be kicked off from the preprocessor
	#define HIDE_Fault_DrawDisplay								0		// 0 pointer
#endif

/* ------------------------------------------------------------ */
/*					Global Variables							*/
/* ------------------------------------------------------------ */

extern volatile EventGroupHandle_t gx_fault_EventGroup;
extern volatile countEdges_handle_p gp_fault_coundEdges;

/* ------------------------------------------------------------ */

#endif // __FAULT_H__
