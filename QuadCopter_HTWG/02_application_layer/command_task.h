/**
 * 		@file 	command_task.h
 * 		@brief	The command Task waits for a Queue of functions and performs them.
 *//*	@author Tobias Walter
 * 		@date 	13.04.2016	(last modified)
 */

#ifndef __COMMAND_TASK_H__
#define	__COMMAND_TASK_H__

/* ------------------------------------------------------------ */
/*					Include File Definitions					*/
/* ------------------------------------------------------------ */

#include <stdint.h>

// setup
#include "qc_setup.h"

/* ------------------------------------------------------------ */
/*				   	Type Definitions			    			*/
/* ------------------------------------------------------------ */

/**
 * \brief	function pointer and the pointer to the handover parameters of this function.
 *
 *			If there are more then one parameter the parameter pointer points to a struct of the parameters.(therefore implemented as void pointer).
 *			(make sure that fp_function has the right typecast for p_parameters)
 */
typedef void (*command_function_fp)(void * p_parameters);

/**
 * \brief	Structure that contains the pointer to a function and the pointer to the handover parameters of this function.
 *
 *			If there are more then one parameter the parameter pointer points to a structer of the parameters.
 *			(therefore implemented as void pointer).
 */
typedef struct command_handle_s
{
	command_function_fp     fp_funcntion;		/**< pointer to funktion */
	void   		         *  p_parameter;		/**< parameter pointer (make shure that fp_funcntion has the right typecast for this parameter) */

} command_handle_s;

/* ------------------------------------------------------------ */
/*				   	Defines			    						*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*					API Procedure Declarations					*/
/* ------------------------------------------------------------ */

extern uint32_t CommandTask_Init(void);
extern void CommandTask_Insert(command_function_fp fp_handle,void *p_parameters,TickType_t xTicksToWait);

#if	( setup_DEV_COMMANDS ) || DOXYGEN
	/**
	 * \brief	Writtes a command(function) in to the command queue.
	 * \note	this function can be HIDE (see qc_setup.h)
	 * \param	fp_handle       Pointer to the function which is inserted in the command queue.
	 * \param	p_parameters    Pointer of the parameter of the function which is inserted in the command queue.
	 * \param	xTicksToWait	portMAX_DELAY for Blocking, 0 for non-Blocking
	 * \note	to enable this HIDE function set setup_DEV_COMMANDS in qc_setup.h
	 */
	#define HIDE_CommandTask_Insert(fp_handle,p_parameters,xTicksToWait)		CommandTask_Insert(fp_handle,p_parameters,xTicksToWait)
#else
	#define HIDE_CommandTask_Insert(fp_handle,p_parameters,xTicksToWait)		// this define will be kicked off from the preprocessor
#endif
/* ------------------------------------------------------------ */
/*					Global Variables							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */

#endif // __COMMAND_TASK_H__
