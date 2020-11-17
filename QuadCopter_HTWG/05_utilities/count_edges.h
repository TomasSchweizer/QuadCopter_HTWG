/**
 * 		@file 	count_edges.h
 * 		@brief	count the edges in different signals.
 *
 *  			counting method can be rising-, falling-, or both edges.
 *  			every signal can have a name.
 *//*
 * 		     -------
 *	sig0:	0|1 1 1|0 0 0		countEdges_METHOD_RISING:  1
 *         ---     ------
 *
 * 		     ------- -----
 *	sig1:	0|1 1 1|0|1 1		countEdges_METHOD_FALLING: 1
 *         ---     ---
 *
 * 		     ------- ---
 *	sig2:	0|1 1 1|0|1|0		countEdges_METHOD_BOTH:    4
 *         ---     --- ---
 *  ...
 *
 * 		@author Tobias Grimm
 * 		@date 	23.04.2016	(last modified)
 */

#ifndef __COUNT_EDGES_H__
#define	__COUNT_EDGES_H__

/* ------------------------------------------------------------ */
/*					Include File Definitions					*/
/* ------------------------------------------------------------ */

#include <stdint.h>

/* ------------------------------------------------------------ */
/*				   	Defines			    						*/
/* ------------------------------------------------------------ */

#define countEdges_METHOD_RISING	(1<<0)
#define countEdges_METHOD_FALLING	(1<<1)
#define countEdges_METHOD_BOTH		(countEdges_METHOD_RISING|countEdges_METHOD_FALLING)

/* ------------------------------------------------------------ */
/*				   	Type Definitions			    			*/
/* ------------------------------------------------------------ */


/**
 * @brief handle to the CoundEdge instance
 */
typedef void* countEdges_handle_p;

/* ------------------------------------------------------------ */
/*					API Procedure Declarations					*/
/* ------------------------------------------------------------ */

extern countEdges_handle_p CountEdges_Create(uint8_t ui8_length);
extern void 			   CountEdges_Reset(countEdges_handle_p p_handle);
extern void 			   CountEdges_Update(countEdges_handle_p p_handle, uint8_t ui8_sigNum, uint8_t ui8_sigValue, uint8_t ui8_method);
extern void 			   CountEdges_Increment(countEdges_handle_p p_handle, uint8_t ui8_sigNum);
extern uint8_t			   CountEdges_Get(countEdges_handle_p p_handle, uint8_t ui8_sigNum);
extern void 			   CountEdges_SetName(countEdges_handle_p p_handle, uint8_t ui8_sigNum,const char* pc_name);
extern const char* 		   CountEdges_GetName(countEdges_handle_p p_handle, uint8_t ui8_sigNum);
extern uint8_t 			   CountEdges_Bit2Num(uint32_t ui32_bit);

/* ------------------------------------------------------------ */
/*					Global Variables							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */

#endif // __COUNT_EDGES_H__
