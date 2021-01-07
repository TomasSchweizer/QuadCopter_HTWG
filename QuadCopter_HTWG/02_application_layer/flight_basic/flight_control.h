/**
 * 		@file 	flight_control.h
 * 		@brief	flight control algorithm, and mixer functions
 *//*	@author Tobias Grimm
 * 		@date 	13.05.2016	(last modified)
 */

#ifndef __FLIGHT_CONTROL_H__
#define	__FLIGHT_CONTROL_H__

/* ------------------------------------------------------------ */
/*					Include File Definitions					*/
/* ------------------------------------------------------------ */

#include <stdint.h>

// setup
#include "qc_setup.h"

/* ------------------------------------------------------------ */
/*				   	Defines			    						*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				   	Type Definitions			    			*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*					API Procedure Declarations					*/
/* ------------------------------------------------------------ */

extern void Control_FlightStabilisation(void);
extern void Control_Reset(void);
extern void Control_Mixer(void);
extern void Control_MixerPassThrottle(void);
extern void Control_MotorSameSetPoint(uint16_t ui16_setPoint);


#if	( setup_DEV_PID_TUNE ) || DOXYGEN
	extern void HIDE_Control_DebugGetPid(void);
	extern void HIDE_Control_Debug_USB_GetPID(void);
    extern void HIDE_Control_PID_TUNE_DrawDisplay(void);
#else
	#define HIDE_Control_DebugGetPid()					// this define will be kicked off from the preprocessor
    #define HIDE_Control_Debug_USB_GetPID()
    #define HIDE_Control_PID_TUNE_DrawDisplay()
#endif

/* ------------------------------------------------------------ */
/*					Global Variables							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */

#endif // __FLIGHT_CONTROL_H__
