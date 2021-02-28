/*===================================================================================================*/
/*  command_task.c                                                                                   */
/*===================================================================================================*/

/*
*   file   command_task.c
*
*   brief  The command task waits for a queue of functions and performs them every 100ms
*
*   details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>13/04/2016      <td>Tobias Walter       <td>Implementation & Last modification of MAs
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Added USB functionality
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*   -
*/
/*====================================================================================================*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

// Standard libraries
#include <stdbool.h>
#include <stdint.h>

// Setup
#include "prioritys.h"

// Application
#include "command_task.h"

// Drivers
#include "debug_interface.h"
#include "display_driver.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

// Utils
#include "qc_math.h"
#include "workload.h"
#include "fault.h"
#include "link_functions.h"


/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Defines                                                 */
/* ---------------------------------------------------------------------------------------------------*/

#define COMMAND_TASK_STACK_SIZE     150                         ///< Task stack size in words
#define COMMAND_QUEUE_LENGTH        3                           ///< Max Length of commands in the queue
#define COMMAND_TASK_PERIOD			portMAX_DELAY               ///< Allows to block the task indefinitely

#define UPDATE_TIMER_MS				100/portTICK_PERIOD_MS      ///< Timer load value is 100ms

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Type Definitions                                        */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Forward Declarations                                          */
/* ---------------------------------------------------------------------------------------------------*/

static void CommandTask(void *pvParameters);
static void UpdateStuff(void* p_parameter);
static void UpdateTimerCallback(TimerHandle_t xTimer);
/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Variables                                               */
/* ---------------------------------------------------------------------------------------------------*/

static QueueHandle_t x_commandQueue;        ///< FreeRTOS handle for the command queue

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Procedure Definitions                                         */
/* ---------------------------------------------------------------------------------------------------*/

/**
 * @brief	The command task waits for a queue of functions and performs them.
 *
 * @param	pvParameters --> not used
 *
 * @return  void
 */
static void CommandTask(void *pvParameters)
{
	command_handle_s s_commandExecute;

    // Loop forever
    while(1)
    {
    	if(xQueueReceive(x_commandQueue, &s_commandExecute, COMMAND_TASK_PERIOD) != 0)
    		s_commandExecute.fp_funcntion(s_commandExecute.p_parameter);
    }
}

/**
 * @brief	This function will be performed from the Command Task
 *			to update non critical stuff every 100ms.
 *
 * @param	p_parameter --> not used
 *
 * @return  void
 */
static void UpdateStuff(void* p_parameter)
{
	HIDE_Display_Redraw();
	HIDE_Debug_USB_Com();
}

/**
 * @brief	This Callback function will be performed from the RTOS Deamon Task
 *
 * @param	xTimer --> not used
 *
 * @return  void
 */
static void UpdateTimerCallback(TimerHandle_t xTimer)
{
	// RTOS Deamon Task has a high priority, so it es better to
	// do non critical things at a lower priority in the command Task
	CommandTask_Insert( UpdateStuff,	// inserted function
						(void *) 0,		// no parameters
						0);				// non blocking
}

/**
 * @brief	Use drivers to initialize peripherals for the
 *			command task and start the task.
 *
 * @return	false --> Task creation was successful\n
 *			true --> Task creation was not successful
 */
uint32_t CommandTask_Init(void)
{
    // Initializes Display
	HIDE_Display_Init();

	//Initializes UART debug interface
	HIDE_Debug_InterfaceInit();

	// Initializes USB debug interface
	HIDE_Debug_USB_InterfaceInit();

	// Inits development display
	#if ( setup_DEV_DISPLAY )
		HIDE_Display_InsertDrawFun(HIDE_Workload_DrawDisplay);
		HIDE_Display_InsertDrawFun(HIDE_Fault_DrawDisplay);
	#endif


	// Create a timer, which inserts every 100 ms a update funktion into the command queue
	#if ( setup_DISPLAY_NONE!=(setup_DISPLAY&setup_MASK_OPT1) || setup_DEV_PID_TUNE )

		TimerHandle_t xTimer = xTimerCreate(		// Create a timer to update stuff every x ms
						   "",						// Timer name (not used)
		                   UPDATE_TIMER_MS,			// Period of the timer
						   pdTRUE,					// Use auto reload
						   ( void * ) 0,			// Timer ID
						   UpdateTimerCallback );	// Timer callback function
		if( xTimer == math_NULL )
			return true;
		if( xTimerStart( xTimer, 0 ) != pdPASS )
			return true;
	#endif

	// Create the queue
	x_commandQueue = xQueueCreate ( COMMAND_QUEUE_LENGTH, sizeof(command_handle_s) );
	if (x_commandQueue == 0)
        return(true);


	TaskHandle_t  x_TaskHandle;

    // Create the command Task.
    if(xTaskCreate(CommandTask,"CommandTask", COMMAND_TASK_STACK_SIZE, math_NULL,
                   tskIDLE_PRIORITY + priority_COMMAND_TASK, &x_TaskHandle) != pdTRUE)
    {
        return(true);
    }

    // Success
    HIDE_Workload_StoreTaskHandle(priotity_NUM_COMMAND_TASK,x_TaskHandle);
    return(false);
}

/**
 * @brief	Writes a command (function) to the command queue.
 *
 * @param	fp_handle --> Pointer to the function which is inserted in the command queue.
 * @param	p_parameters --> Pointer of the parameter of the function which is inserted in the command queue.
 * @param	xTicksToWait --> portMAX_DELAY for Blocking, 0 for non-Blocking
 *
 * @return  void
 *
 */
void CommandTask_Insert(command_function_fp fp_handle,void *p_parameters,TickType_t xTicksToWait)
{
	command_handle_s s_commandInsert = (command_handle_s){fp_handle, p_parameters};
	xQueueSend(x_commandQueue, &s_commandInsert , xTicksToWait);

}

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
