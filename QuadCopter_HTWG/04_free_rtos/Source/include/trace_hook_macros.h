/**
 * 		@file 	trace_hook_macros.h
 * 		@brief	define freeRTOS trace hook macros
 *//*	@author Tobias Walter
 * 		@date 	04.04.2016	(last modified)
 */

#ifndef __TRACE_HOOK_MAKROS_H__
#define __TRACE_HOOK_MAKROS_H__

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include "qc_setup.h"

/* ------------------------------------------------------------ */
/*				Trace Hook Functions							*/
/* ------------------------------------------------------------ */

extern void Workload_TaskSwitchNotify( void );


/* ------------------------------------------------------------ */
/*				Trace Hook Macros								*/
/* ------------------------------------------------------------ */

#if   ( setup_DEV_WORKLOAD_LED || setup_DEV_WORKLOAD_CALC )
	#define traceTASK_SWITCHED_OUT() Workload_TaskSwitchNotify()
#endif

#endif // __TRACE_HOOK_MAKROS_H__
