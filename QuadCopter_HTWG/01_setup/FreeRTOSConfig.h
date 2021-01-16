/*
    FreeRTOS V9.0.0rc1 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved
*/

#ifndef __FREERTOS_CONFIG_H__
#define __FREERTOS_CONFIG_H__

/* ------------------------------------------------------------ */
/*					Include File Definitions					*/
/* ------------------------------------------------------------ */

#include "trace_hook_macros.h"

// setup
#include "prioritys.h"
#include "qc_setup.h"

/* ------------------------------------------------------------ */
/*				   	Defines			    						*/
/* ------------------------------------------------------------ */

#define configUSE_PREEMPTION                		1			// 1 for preemptive RTOS scheduler, 0 for cooperative
#define configUSE_TICK_HOOK                 		0
#define configCPU_CLOCK_HZ                  		80000000
#define configTICK_RATE_HZ                  		1000
#define configMINIMAL_STACK_SIZE            		64 		// [words] Stack Size for the idle Task (this is NOT the MINIMAL_STACK_SIZE for other tasks!)
#define configTOTAL_HEAP_SIZE               		5000 		// [Bytes] (freeRTOS builds his own Heap from stack memory)
#define configMAX_TASK_NAME_LEN             		4			// including NULL at end of string
#define configUSE_TRACE_FACILITY            		1
#define configUSE_16_BIT_TICKS              		0			// 0 -> uint32_t, 1 -> uint16_t    for time measurement
#define configIDLE_SHOULD_YIELD             		0			// This parameter controls the behaviour of tasks at the idle priority.
#define configMAX_PRIORITIES						priority_COUNT
#define configSUPPORT_DYNAMIC_ALLOCATION            1
#define configUSE_IDLE_HOOK             			(setup_DEV_WORKLOAD_STATISTIC || setup_DEV_WORKLOAD_LED ) 			// function that is called during each cycle of the idle task -> common to use the idle hook function to place the micro-controller CPU into a power saving mode.
#if   ( setup_DEV_STACK_OVERFLOW )
	#define configCHECK_FOR_STACK_OVERFLOW  		2			// 2 -> RTOS Kernel will colour the Stack and look at every context switch if the "colour has changed" -> vApplicationStackOverflowHook will be called  (this should only be used for development)
#else
	#define configCHECK_FOR_STACK_OVERFLOW  		0
#endif

// The RTOS Deamon Task (used for EventGroups & timers)
#define configUSE_TIMERS        					1
#define configTIMER_TASK_PRIORITY           		priority_RTOS_DAEMON_TASK
#define configTIMER_QUEUE_LENGTH            		5							// for EventGroups fromISR commands
#define configTIMER_TASK_STACK_DEPTH        		100
#define conficTIMER_TASK_NAME						"Deamon"

// Set the following definitions to 1 to include the API function, or zero
// to exclude the API function.
#define INCLUDE_vTaskDelayUntil             		1
#define INCLUDE_vTaskDelay                  		1
#define INCLUDE_xEventGroupSetBitFromISR			1	// used for eventBits
#define INCLUDE_xTimerPendFunctionCall 				1	// used for eventBits
#define INCLUDE_uxTaskGetStackHighWaterMark 		0	// can be used to estimate Task Stack usage

#if ( setup_DEV_WORKLOAD_CALC || setup_DEV_WORKLOAD_LED )
	#define INCLUDE_xTimerGetTimerDaemonTaskHandle	1
	#define INCLUDE_xTaskGetIdleTaskHandle      	1
	#define INCLUDE_pcTaskGetTaskName				1
	#define INCLUDE_uxTaskPriorityGet 				1
#endif




// Be ENORMOUSLY careful if you want to modify these two values and make sure
// you read http://www.freertos.org/a00110.html#kernel_priority and
// http://www.freertos.org/RTOS-Cortex-M3-M4.html first!

#define configKERNEL_INTERRUPT_PRIORITY         	priority_KERNEL   // Priority 7, This is the lowest priority.
#define configMAX_SYSCALL_INTERRUPT_PRIORITY   	 	priority_MAX_API_CALL_INTERRUPT   // Priority 1


#endif // __FREERTOS_CONFIG_H__

