/*===================================================================================================*/
/*  command_task.h                                                                                   */
/*===================================================================================================*/

/**
*   @file   command_task.h
*
*   @brief  API for the command task
*
*   @details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>13/05/2016      <td>Tobias Walter       <td>Implementation & Last modification of MAs
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*   -
*/
/*====================================================================================================*/

#ifndef __COMMAND_TASK_H__
#define	__COMMAND_TASK_H__

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

// Standard libraries
#include <stdint.h>

// Setup
#include "qc_setup.h"

// FreeRTOS
#include "FreeRTOS.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/
/**
 * @brief	Function pointer and the pointer to the handover parameters of this function.
 *
 * @details If there are more then one parameter the parameter pointer points to a struct of the parameters.
 *          Therefore implemented as void pointer.Make sure that fp_function has the right typecast for p_parameters.
 */
typedef void (*command_function_fp)(void * p_parameters);

/**
 * @brief	Structure that contains the pointer to a function and the pointer to the handover parameters of this function.
 *
 * @details	If there are more then one parameter the parameter pointer points to a struct of the parameters.
 *			Therefore implemented as void pointer.
 */
typedef struct command_handle_s
{
	command_function_fp     fp_funcntion;		/**< pointer to funktion */
	void   		         *  p_parameter;		/**< parameter pointer (make shure that fp_funcntion has the right typecast for this parameter) */

} command_handle_s;

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/

extern uint32_t CommandTask_Init(void);
extern void CommandTask_Insert(command_function_fp fp_handle,void *p_parameters,TickType_t xTicksToWait);

#if	( setup_DEV_COMMANDS ) || DOXYGEN
	/**
	 * @brief	Writes a command(function) in to the command queue.
	 *
	 * @note	This function can be HIDE (see qc_setup.h). To enable this HIDE function set setup_DEV_COMMANDS in qc_setup.h
	 *
	 * @param	fp_handle --> Pointer to the function which is inserted in the command queue.
	 * @param	p_parameters --> Pointer of the parameter of the function which is inserted in the command queue.
	 * @param	xTicksToWait --> portMAX_DELAY for Blocking, 0 for non-Blocking
	 *
	 */
	#define HIDE_CommandTask_Insert(fp_handle,p_parameters,xTicksToWait)		CommandTask_Insert(fp_handle,p_parameters,xTicksToWait)
#else
	#define HIDE_CommandTask_Insert(fp_handle,p_parameters,xTicksToWait)		// this define will be kicked off from the preprocessor
#endif


#endif // __COMMAND_TASK_H__

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
