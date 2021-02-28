/*===================================================================================================*/
/*  flight_task.h                                                                                    */
/*===================================================================================================*/

/**
*   @file flight_task.h
*
*   @brief API for flight task
*
*   @details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>01/06/2016      <td>Tobias Grimm        <td>Implementation & Last modification of MAs
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*   -
*/
/*====================================================================================================*/

#ifndef __FLIGHT_TASK_H__
#define	__FLIGHT_TASK_H__

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

// Standard libraries
#include <stdint.h>

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/

/// Define to access ROLL in gf_flight_setPoint
#define flight_ROLL 		(0)		// do not change this
/// Define to access PITCH in gf_flight_setPoint
#define flight_PITCH 		(1)		// do not change this
/// Define to access YAW in gf_flight_setPoint
#define flight_YAW 			(2)		// do not change this
/// Define to access THROTTLE in gf_flight_setPoint */
#define flight_THROTTLE		(3)		// do not change this

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/

/// Possible states of the state machine in the flight task
enum flight_state_e
{
	FLYING,
	LANDING,
	RESTING
};

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

extern float gf_flight_setPoint[4];
extern enum flight_state_e ge_flight_state;

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/

extern uint32_t FlightTask_Init(void);


#endif // __FLIGHT_TASK_H__

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
