/**
 * 		@file 	link_functions.h
 * 		@brief	link functions together (linked list)
 *
 *  			and run all linked funktions
	 *//*
	 * 		@author Tobias Grimm
 * 		@date 	15.05.2016	(last modified)
 */

#ifndef __LINKED_FUNCTIONS_H__
#define	__LINKED_FUNCTIONS_H__

/* ------------------------------------------------------------ */
/*					Include File Definitions					*/
/* ------------------------------------------------------------ */

#include <stdint.h>

/* ------------------------------------------------------------ */
/*				   	Defines			    						*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				   	Type Definitions			    			*/
/* ------------------------------------------------------------ */

/**
 * @brief 	handle to the LinkFunctions instance
 *
 *			(to create a new instance, just initialize a
 *			linkFun_handle_p variable with 0)
 */
typedef void* linkFun_handle_p;

/**
 * @brief	prototype function pointer, which can be stored
 *			in linkFun_handle_p variables.
 */
typedef void (*linkFun_fp)(void);

/* ------------------------------------------------------------ */
/*					API Procedure Declarations					*/
/* ------------------------------------------------------------ */

extern void LinkFun_Insert(linkFun_handle_p* pp_handle, linkFun_fp fp_fun);
extern void LinkFun_RunAllFun(linkFun_handle_p p_handle);

/* ------------------------------------------------------------ */
/*					Global Variables							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */

#endif // __LINKED_FUNCTIONS_H__
