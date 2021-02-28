/*===================================================================================================*/
/*  prioritys.h                                                                                      */
/*===================================================================================================*/

/**
*   @file   prioritys.h
*
*   @brief  Sets the priorities for all tasks and interrupts, every task gets an additional unique Number.
*
*   @details
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>01/06/2016      <td>Tobias Grimm        <td>Implementation & Last modification of MAs
*   <tr><td>14/12/2020      <td>Tomas Schweizer     <td>Added Watchdog and USB change of priorities (motor/receiver)
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*
*/
/*====================================================================================================*/

#ifndef __PRIORITYS_H__
#define __PRIORITYS_H__

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

// Standard libraries
#include <stdint.h>

// Setup
#include "qc_setup.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/


// Task priorities
#define priority_COUNT					( 5 )				        ///< priority 0 ... priority_COUNT-1

//  (A higher Number is a higher priority)
#define priority_RTOS_DAEMON_TASK		( priority_COUNT - 1 )        ///< RTOS Deamon task with priority 4
#define priority_FLIGHT_TASK			( priority_COUNT - 2 )        ///< Flight task with priority 3
#define priority_RECEIVER_TASK      	( priority_COUNT - 3 )        ///< Receiver task with priority 2
#define priority_COMMAND_TASK			( priority_COUNT - 4 )        ///< Command task with priority 1
#define priority_IDLE_TASK				( 0 )				        ///< FreeRTOS gives the idle task priority 0, This macro is not used just for a complete overview


// NVIC priorities (interrupts)

// (0=high prio, 7=low prio)
// Only upper 3 bits are used in NVIC -> shift them up 5 times
// Prio 0 can never call API from ISR!!! (see below)
#define priority_WATCHDOG_ISR           ( 1 << 5 )                  ///< Watchdog has the highest priority 1 interrupt even over FreeRTOS
#define priority_SENSOR_ISR             ( 3 << 5 )                  ///< Sensor MPU for attitude has the priority 3
#define priority_SENSOR_ALT_ISR         ( 3 << 5 )                  ///< Sensor lidar and barometer for altitude have also priority 3
#define priority_MOTOR_ISR              ( 4 << 5 )                  ///< Motor/ESCs have priority 4
#define priority_REMOTE_ISR				( 5 << 5 )                  ///< Remote Control priority 5
#define priority_DEBUGGER_ISR			( 6 << 5 )                  ///< Debugger interrupts have priority 6
#define priority_USB_ISR                ( 3 << 5 )                  ///< USB communication has also priority 6


// all ISRs with a priority number >= priority_MAX_API_CALL_INTERRUPT
// can call FreeRTOS API functions from ISR
#define priority_KERNEL         		( 7 << 5 )                  ///< The FreeRTOS Kernel has priority 7, this should be the lowest priority.
#define priority_MAX_API_CALL_INTERRUPT ( 2 << 5 )	                ///< All interrupts with priority higher then 2 can call the FreeRTOS API



// Task Numbers

// Every Task gets a unique Number (-> take care when two Tasks share the same priority!)
// starting from 0 to priority_NUM_COUNT-1
// when setup_DEV_WORKLOAD is used to display the workload, the highest Number will be at the top of the display
#define priority_NUM_COUNT          		 ( priority_COUNT )                 ///< Amount of different priorities

#define priority_NUM_RTOS_DAEMON_TASK        ( priority_RTOS_DAEMON_TASK )      ///< Copy of task priority for workload
#define priority_NUM_IDLE_TASK               ( priority_IDLE_TASK )             ///< Copy of task priority for workload
#define priotity_NUM_COMMAND_TASK            ( priority_COMMAND_TASK )          ///< Copy of task priority for workload
#define priority_NUM_RECIVER_TASK            ( priority_RECEIVER_TASK )         ///< Copy of task priority for workload
#define priority_NUM_FLIGHT_TASK             ( priority_FLIGHT_TASK )          ///< Copy of task priority for workload

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/


#endif // __PRIORITYS_H__

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
