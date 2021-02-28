/*===================================================================================================*/
/*  count_edges.h                                                                                    */
/*===================================================================================================*/

/**
*   @file   count_edges.h
*
*   @brief  API to count edges of different signals.
*
*   @details
*   Counting method can be rising, falling, or both edges.
*   Every signal can have a name.\n
*
*   sig0:   0|1 1 1|0 0 0       countEdges_METHOD_RISING:  1\n
*
*   sig1:   0|1 1 1|0|1 1       countEdges_METHOD_FALLING: 1\n

*   sig2:   0|1 1 1|0|1|0       countEdges_METHOD_BOTH:    4\n
*
*   \n
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>23/04/2016      <td>Tobias Grimm        <td>Implementation & last modifications through MAs
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*/
/*====================================================================================================*/

#ifndef __COUNT_EDGES_H__
#define	__COUNT_EDGES_H__

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

// Standard library
#include <stdint.h>

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/

#define countEdges_METHOD_RISING	(1<<0)
#define countEdges_METHOD_FALLING	(1<<1)
#define countEdges_METHOD_BOTH		(countEdges_METHOD_RISING|countEdges_METHOD_FALLING)

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/

/**
 * @brief handle to the CoundEdge instance
 */
typedef void* countEdges_handle_p;

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/

extern countEdges_handle_p CountEdges_Create(uint8_t ui8_length);
extern void 			   CountEdges_Reset(countEdges_handle_p p_handle);
extern void 			   CountEdges_Update(countEdges_handle_p p_handle, uint8_t ui8_sigNum, uint8_t ui8_sigValue, uint8_t ui8_method);
extern void 			   CountEdges_Increment(countEdges_handle_p p_handle, uint8_t ui8_sigNum);
extern uint8_t			   CountEdges_Get(countEdges_handle_p p_handle, uint8_t ui8_sigNum);
extern void 			   CountEdges_SetName(countEdges_handle_p p_handle, uint8_t ui8_sigNum,const char* pc_name);
extern const char* 		   CountEdges_GetName(countEdges_handle_p p_handle, uint8_t ui8_sigNum);
extern uint8_t 			   CountEdges_Bit2Num(uint32_t ui32_bit);

#endif // __COUNT_EDGES_H__

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
