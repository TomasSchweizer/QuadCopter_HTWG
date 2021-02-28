/*===================================================================================================*/
/*  link_functions.h                                                                                 */
/*===================================================================================================*/

/**
*   @file   link_functions.h
*
*   @brief  API to link functions together (Linked list)
*
*   @details
*   Multiple functions can be linked together and run all at once
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>15/05/2016      <td>Tobias Grimm        <td>Implementation & last modifications through MAs
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*/
/*====================================================================================================*/

#ifndef __LINKED_FUNCTIONS_H__
#define	__LINKED_FUNCTIONS_H__

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

// Standard library
#include <stdint.h>

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/

/**
 * @brief 	Handle to the LinkFunctions instance
 *
 * @details
 * To create a new instance, just initialize a linkFun_handle_p variable with 0
 *
 */
typedef void* linkFun_handle_p;

/**
 * @brief	Prototype function pointer, which can be stored in linkFun_handle_p variables.
 */
typedef void (*linkFun_fp)(void);

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/

extern void LinkFun_Insert(linkFun_handle_p* pp_handle, linkFun_fp fp_fun);
extern void LinkFun_RunAllFun(linkFun_handle_p p_handle);


#endif // __LINKED_FUNCTIONS_H__

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
