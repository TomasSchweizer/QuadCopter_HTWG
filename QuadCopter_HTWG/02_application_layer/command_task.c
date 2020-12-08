/**
 * 		@file 	command_task.c
 * 		@brief	The command Task waits for a Queue of functions and performs them.
 *//*	@author Tobias Walter
 * 		@date 	13.04.2016	(last modified)
 */

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <stdbool.h>
#include <stdint.h>

// freeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

// drivers
#include "debug_interface.h"
#include "display_driver.h"
#include "sensor_driver.h"

// application
#include "command_task.h"
#include "receiver_task.h"		// for drawDisplay
#include "fault.h"
#include "flight_control.h"		// for PID Tune over debug interface

// utils
#include "qc_math.h"
#include "workload.h"
#include "link_functions.h"

// setup
#include "prioritys.h"

/* ------------------------------------------------------------ */
/*				Local Defines									*/
/* ------------------------------------------------------------ */

#define COMMAND_TASK_STACK_SIZE     150         // Stack size in words
#define COMMAND_QUEUE_LENGTH        3
#define COMMAND_TASK_PERIOD			portMAX_DELAY

#define UPDATE_TIMER_MS				100/portTICK_PERIOD_MS

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

static void CommandTask(void *pvParameters);
uint32_t CommandTask_Init(void);
void CommandTask_Insert(command_function_fp fp_handle,void *p_parameters,TickType_t xTicksToWait);

/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

static QueueHandle_t x_commandQueue;

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

/**
 * \brief	The command Task waits for a Queue of functions and performs them.
 * \param	pvParameters	not used
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
 * \brief	This function will be performed from the Command Task
 *			to update non critical stuff every x ms.
 * \param	p_parameter		not used
 */
static void UpdateStuff(void* p_parameter)
{
	HIDE_Display_Redraw();
	HIDE_Control_DebugGetPid();
	HIDE_Sensor_SendDataOverUSB();

	// TODO insert USB HIDE_FUnction
}

/**
 * \brief	This Callback function will be performed from the RTOS Deamon Task
 * \param	xTimer		not used
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
 * \brief	use drivers to initialize peripherals for the
 *			Commandask und start the Task
 * \return	false if Task creation was successful,
 *			true  else
 */
uint32_t CommandTask_Init(void)
{
	HIDE_Display_Init();
	HIDE_Debug_InterfaceInit();
	HIDE_Debug_USB_InterfaceInit();
	// TODO add HIDE_Debug_USB_Interface

	#if ( setup_DEV_DISPLAY )
		HIDE_Display_InsertDrawFun(HIDE_Workload_DrawDisplay);
		HIDE_Display_InsertDrawFun(HIDE_Fault_DrawDisplay);
		HIDE_Display_InsertDrawFun(HIDE_Receive_DrawDisplay);
	#endif

//	if ( setup_DEV_DEBUG_USB)
//	    HIDE_

	//
	// Create a timer, which inserts every x ms a UpdateFunktion into the Command Queue
	// TODO add USB as alternative why the timer is used
	#if ( setup_DISPLAY_NONE!=(setup_DISPLAY&setup_MASK_OPT1) || setup_DEV_PID_TUNE )

		TimerHandle_t xTimer = xTimerCreate(		// create a timer to update stuff every x ms
						   "",						// timer name (not used)
		                   UPDATE_TIMER_MS,			// period of the timer
						   pdTRUE,					// use auto reload
						   ( void * ) 0,			// timer ID
						   UpdateTimerCallback );	// timer callback function
		if( xTimer == math_NULL )
			return true;
		if( xTimerStart( xTimer, 0 ) != pdPASS )
			return true;
	#endif

	//
	// Create the Queue
	//
	x_commandQueue = xQueueCreate ( COMMAND_QUEUE_LENGTH, sizeof(command_handle_s) );
	if (x_commandQueue == 0)
        return(true);


	TaskHandle_t  x_TaskHandle;
    //
    // Create the command Task.
    //
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
 * \brief	Writtes a command(function) in to the command queue.
 * \param	fp_handle       Pointer to the function which is inserted in the command queue.
 * \param	p_parameters    Pointer of the parameter of the function which is inserted in the command queue.
 * \param	xTicksToWait	portMAX_DELAY for Blocking, 0 for non-Blocking
 */
void CommandTask_Insert(command_function_fp fp_handle,void *p_parameters,TickType_t xTicksToWait)
{
	command_handle_s s_commandInsert = (command_handle_s){fp_handle, p_parameters};
	xQueueSend(x_commandQueue, &s_commandInsert , xTicksToWait);

}
