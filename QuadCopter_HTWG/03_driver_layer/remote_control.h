/**
 * 		@file 	remote_control.h
 * 		@brief	Driver to read values over a remote control
 *//*	@author Tobias Walter
 * 		@date 	12.03.2016	(last modified)
 */

#ifndef __REMOTE_CONTROL_H__
#define	__REMOTE_CONTROL_H__

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <stdint.h>

/* ------------------------------------------------------------ */
/*				   Type Definitions			    				*/
/* ------------------------------------------------------------ */

/** \brief	Number of channel ROLL in f_receiverSetPoint */
#define remote_ROLL			  		  ( 0 )		// do not change this
/** \brief	Number of channel PITCH in f_receiverSetPoint */
#define remote_PITCH			 	  ( 1 )		// do not change this
/** \brief	Number of channel YAW in f_receiverSetPoint */
#define remote_YAW			          ( 2 )		// do not change this
/** \brief	Number of channel THROTTLE in f_receiverSetPoint */
#define remote_THROTTLE		          ( 3 )	    // do not change this
/** \brief	Number of channel AUX1 in f_receiverSetPoint */
#define remote_AUX1			          ( 4 )
/** \brief	Number of channel RESERVE */
#define remote_RESERVE		          ( 5 )

/* ------------------------------------------------------------ */
/*					Variable Declarations						*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*					Procedure Declarations						*/
/* ------------------------------------------------------------ */

extern void 	RemoteControl_Init(void);
extern uint8_t  RemoteControl_GetData(volatile float f_receiverSetPoint[]);
extern void 	RemoteControl_Calibrate();

/* ------------------------------------------------------------ */

#endif // __REMOTE_CONTROL_H__
