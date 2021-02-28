/*===================================================================================================*/
/*  FreeRTOSConfig.h                                                                                 */
/*===================================================================================================*/


/**
*   @file  FreeRTOSConfig.h
*
*   @brief  Setup file for FreeRTOS port
*
    @details
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>21/03/2016      <td>Tobias Grimm        <td>Implementation & Last modification of MAs
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*   - FreeRTOS V9.0.0rc1 - Copyright (C) 2016 Real Time Engineers Ltd. - All rights reserved
*/
/*====================================================================================================*/


#ifndef __FREERTOS_CONFIG_H__
#define __FREERTOS_CONFIG_H__

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

// FreeRTOS port macros
#include "trace_hook_macros.h"

// Setup
#include "prioritys.h"
#include "qc_setup.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/

#define configUSE_PREEMPTION                		( 1 )               ///< 1 = preemptive || 0 = cooperative FreeRTOS Scheduler
#define configUSE_TICK_HOOK                 		( 0 )               ///< 1 = use idle hook || 0 = omit idle hook
#define configCPU_CLOCK_HZ                  		( 80000000 )        ///< Frequency of internal CPU clock
#define configTICK_RATE_HZ                  		( 1000 )            ///< Frequency of RTOS tick interrupt
#define configMINIMAL_STACK_SIZE            		( 64 )		        ///< Stack size for the idle task [words], !!!NOT THE MINIMAL_STACK_SIZE FOR OTHER TASKS!!!
#define configTOTAL_HEAP_SIZE               		( 5000 ) 		    ///< Heap size [bytes], freeRTOS builds his own heap from stack memory)
#define configMAX_TASK_NAME_LEN             		( 4	)		        ///< Includes NULL at end of string
#define configUSE_TRACE_FACILITY            		( 1 )               ///< Allows execution visualization and tracing
#define configUSE_16_BIT_TICKS              		( 0	)		        ///< 0 = uint32_t || 1 = uint16_t, for time measurement
#define configIDLE_SHOULD_YIELD             		( 0	)		        ///< Parameter controls behavior of tasks at idle priority.
#define configMAX_PRIORITIES						( priority_COUNT )  ///< Number of priorities available for co-routines
#define configSUPPORT_DYNAMIC_ALLOCATION            ( 1 )               ///< RTOS objects can be created using RAM which is automatically allocated from the FreeRTOS heap
#define configUSE_IDLE_HOOK             			( setup_DEV_WORKLOAD_STATISTIC || setup_DEV_WORKLOAD_LED )   ///< Function is called during each cycle of the idle task -> common to use the idle hook function to place the micro-controller CPU into a power saving mode.
#if   ( setup_DEV_STACK_OVERFLOW )
	#define configCHECK_FOR_STACK_OVERFLOW  		( 2 )			    ///< 2 = RTOS Kernel will color the Stack and look at every context switch if the "color has changed" -> vApplicationStackOverflowHook will be called  (!!!ONLY IN DEVELOPMENT!!!)
#else
	#define configCHECK_FOR_STACK_OVERFLOW  		( 0 )
#endif

// The RTOS Deamon Task (used for EventGroups & timers)
#define configUSE_TIMERS        					( 1 )                           ///< Include software timer functionality
#define configTIMER_TASK_PRIORITY           		( priority_RTOS_DAEMON_TASK )   ///< Set priority of deamon task
#define configTIMER_QUEUE_LENGTH            		( 5 )							///< For EventGroups from ISR commands lenght of the command queue
#define configTIMER_TASK_STACK_DEPTH        		( 100 )                         ///< Stack depth allocated for the deamon task
#define conficTIMER_TASK_NAME						( "Deamon" )                    ///< Set task name to Deamon

// Set the following definitions to 1 to include the API function, or zero to exclude the API function.
#define INCLUDE_vTaskDelayUntil             		( 1 )   ///< Includes the possibility to delay tasks for a certain time
#define INCLUDE_vTaskDelay                  		( 1 )   ///< Includes the possibility for delay tasks
#define INCLUDE_xEventGroupSetBitFromISR			( 1	)   ///< Used for eventBits
#define INCLUDE_xTimerPendFunctionCall 				( 1	)   ///< Used for eventBits
#define INCLUDE_uxTaskGetStackHighWaterMark 		( 0	)   ///< Can be used to estimate task stack usage

#if ( setup_DEV_WORKLOAD_CALC || setup_DEV_WORKLOAD_LED )
	#define INCLUDE_xTimerGetTimerDaemonTaskHandle	( 1 )   ///< Include functionality to get Deamon task handle
	#define INCLUDE_xTaskGetIdleTaskHandle      	( 1 )   ///< Include functionality to get Idle task handle
	#define INCLUDE_pcTaskGetTaskName				( 1 )   ///< Includes functionality to get task names
	#define INCLUDE_uxTaskPriorityGet 				( 1 )   ///< Includes functionality to get task priorities
#endif




// Be ENORMOUSLY careful if you want to modify these two values and make sure
// you read http://www.freertos.org/a00110.html#kernel_priority and
// http://www.freertos.org/RTOS-Cortex-M3-M4.html first!

#define configKERNEL_INTERRUPT_PRIORITY         	( priority_KERNEL )                     ///< Priority 7, This is the lowest priority.
#define configMAX_SYSCALL_INTERRUPT_PRIORITY   	 	( priority_MAX_API_CALL_INTERRUPT )     ///< Priority 2

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/

#endif // __FREERTOS_CONFIG_H__

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
