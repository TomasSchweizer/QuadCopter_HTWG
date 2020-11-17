/**
 * 		@file 	flight_task.h
 * 		@brief	Task for flight stabilization
 *
 *  			sensorfusion, control algorithm, motor output,
 *  			state machine for critical events
 *//*	@author Tobias Grimm
 * 		@date 	01.06.2016	(last modified)
 */

#ifndef __FLIGHT_TASK_H__
#define	__FLIGHT_TASK_H__

/* ------------------------------------------------------------ */
/*					Include File Definitions					*/
/* ------------------------------------------------------------ */

#include <stdint.h>

/* ------------------------------------------------------------ */
/*				   	Defines			    						*/
/* ------------------------------------------------------------ */

/** \brief	define to access ROLL in gf_flight_setPoint */
#define flight_ROLL 		(0)		// do not change this
/** \brief	define to access PITCH in gf_flight_setPoint */
#define flight_PITCH 		(1)		// do not change this
/** \brief	define to access YAW in gf_flight_setPoint */
#define flight_YAW 			(2)		// do not change this
/** \brief	define to access THROTTLE in gf_flight_setPoint */
#define flight_THROTTLE		(3)		// do not change this

/* ------------------------------------------------------------ */
/*				   	Type Definitions			    			*/
/* ------------------------------------------------------------ */

/**
 * \brief	possible states of the QuadCopter
 */
enum flight_state_e
{
	FLYING,
	LANDING,
	RESTING
};

/* ------------------------------------------------------------ */
/*					API Procedure Declarations					*/
/* ------------------------------------------------------------ */

extern uint32_t FlightTask_Init(void);

/* ------------------------------------------------------------ */
/*					Global Variables							*/
/* ------------------------------------------------------------ */

extern float gf_flight_setPoint[4];
extern enum flight_state_e ge_flight_state;

/* ------------------------------------------------------------ */

#endif // __FLIGHT_TASK_H__
