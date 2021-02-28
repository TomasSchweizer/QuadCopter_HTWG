/*===================================================================================================*/
/*  workload.h                                                                                       */
/*===================================================================================================*/

/**
*   @file   workload.h
*
*   @brief  API for workload estimation
*
*   @details
*   Used to create workload instances to estimate desired workloads
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>21/05/2016      <td>Tobias Grimm        <td>Implementation & last modifications through MAs
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*
*/
/*====================================================================================================*/

#ifndef __WORKLOAD_H__
#define	__WORKLOAD_H__

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

// Standard library
#include <stdint.h>

// Setup
#include "prioritys.h"
#include "qc_setup.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/

 /**
  * @brief	Handle to the workload estimator instance
  */
typedef void* workload_handle_p;

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

extern volatile uint16_t gui16_tasksTimeShareMemory[priority_NUM_COUNT];
extern  TaskHandle_t  g_workload_taskHandles[priority_NUM_COUNT];

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/

#if	( setup_DEV_WORKLOAD_CALC || setup_DEV_WORKLOAD_LED ) || DOXYGEN
	extern void HIDE_Workload_Init(void);
#else
	#define HIDE_Workload_Init()										// this define will be kicked off from the preprocessor
#endif

#if	( setup_DEV_WORKLOAD_CALC ) || DOXYGEN
	extern void HIDE_Workload_StoreTaskHandle(uint8_t ui8_numTask,TaskHandle_t x_taskHandle);
	extern void HIDE_Workload_DrawDisplay(void);
	extern void HIDE_Workload_EstimateCreate(workload_handle_p* p_handle, const char* pc_name);
	extern void HIDE_Workload_EstimateStart(workload_handle_p p_handle);
	extern void HIDE_Workload_EstimateStop(workload_handle_p p_handle);
#else
	#define HIDE_Workload_StoreTaskHandle(ui8_numTask,x_taskHandle) 	// this define will be kicked off from the preprocessor
	#define HIDE_Workload_DrawDisplay								0	// 0 Pointer
	#define HIDE_Workload_EstimateCreate(p_handle, pc_name)				// this define will be kicked off from the preprocessor
	#define HIDE_Workload_EstimateStart(p_handle)						// this define will be kicked off from the preprocessor
	#define HIDE_Workload_EstimateStop(p_handle)						// this define will be kicked off from the preprocessor
#endif

#endif // __WORKLOAD_H__

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/

