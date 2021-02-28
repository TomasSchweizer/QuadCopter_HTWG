/*===================================================================================================*/
/*  remote_control.h                                                                                 */
/*===================================================================================================*/

/**
*   @file   remote_control.h
*
*   @brief  API to receive and interpret values from the remote control
*
*   @details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>12/03/2016      <td>Tobias Walter       <td>Implementation & last modifications through MAs
*   <tr><td>12/06/2021      <td>Tomas Schweizer     <td>Added features (QC axes, throttle rate slope)
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*/
/*====================================================================================================*/

#ifndef __REMOTE_CONTROL_H__
#define	__REMOTE_CONTROL_H__

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

// Standard library
#include <stdint.h>

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/

/** @brief	Number of channel ROLL in f_receiverSetPoint */
#define remote_ROLL			  		  ( 0 )		// do not change this
/** @brief	Number of channel PITCH in f_receiverSetPoint */
#define remote_PITCH			 	  ( 1 )		// do not change this
/** @brief	Number of channel YAW in f_receiverSetPoint */
#define remote_YAW			          ( 2 )		// do not change this
/** @brief	Number of channel THROTTLE in f_receiverSetPoint */
#define remote_THROTTLE		          ( 3 )	    // do not change this
/** @brief	Number of channel AUX1 in f_receiverSetPoint */
#define remote_AUX1			          ( 4 )
/** @brief	Number of channel RESERVE */
#define remote_RESERVE		          ( 5 )

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/
extern void 	RemoteControl_Init(void);
extern uint8_t  RemoteControl_GetData(volatile float f_receiverSetPoint[]);
extern void 	RemoteControl_Calibrate();

#endif // __REMOTE_CONTROL_H__

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
