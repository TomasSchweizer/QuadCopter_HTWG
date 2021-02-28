/*===================================================================================================*/
/*  link_functions.c                                                                                 */
/*===================================================================================================*/

/*
*   file   link_functions.c
*
*   brief  Implementation of functions to link functions together (Linked list)
*
*   details
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

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

// Standard library
#include <stdint.h>

// FreeRTOS
#include "FreeRTOS.h"		// for pvPortMalloc

// Utilities
#include "qc_math.h"
#include "link_functions.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Defines                                                 */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Type Definitions                                        */
/* ---------------------------------------------------------------------------------------------------*/

typedef struct linkFun_s
 {
     linkFun_fp 		fp_fun;		/**< function pointer to desired function */
     struct linkFun_s*  ps_next; 	/**< pointer to the next element (linked List) */
 }linkFun_s;

 /* ---------------------------------------------------------------------------------------------------*/
 /*                                      Forward Declarations                                          */
 /* ---------------------------------------------------------------------------------------------------*/

 /* ---------------------------------------------------------------------------------------------------*/
 /*                                      Global Variables                                              */
 /* ---------------------------------------------------------------------------------------------------*/

 /* ---------------------------------------------------------------------------------------------------*/
 /*                                      Local Variables                                               */
 /* ---------------------------------------------------------------------------------------------------*/

 /* ---------------------------------------------------------------------------------------------------*/
 /*                                      Procedure Definitions                                         */
 /* ---------------------------------------------------------------------------------------------------*/

 /**
  * @brief  Insert a function into the linkFun_handle_p instance (linked list)
  *
  * @param	pp_handle --> Pointer to handle of the LinkFunction instance
  *	@param	fp_fun --> Function to insert
  *
  *	@return void
  */
void LinkFun_Insert(linkFun_handle_p* pp_handle, linkFun_fp fp_fun)
{
	linkFun_s ** pps_linkFun = (linkFun_s **) pp_handle;
	linkFun_s *  ps_newLinkFun;

	// Walk to the end of the linked List
    while( *pps_linkFun != math_NULL )
        pps_linkFun = &(*pps_linkFun)->ps_next;

    ps_newLinkFun = (linkFun_s *) pvPortMalloc(sizeof(*ps_newLinkFun));
    ps_newLinkFun->fp_fun = fp_fun;
    ps_newLinkFun->ps_next = math_NULL;

    *pps_linkFun = ps_newLinkFun;
}

/**
 * @brief	Run all linked functions
 *
 * @param	p_handle --> Handle to the LinkFunction instance
 *
 * @return  void
 */
void LinkFun_RunAllFun(linkFun_handle_p p_handle)
{
	linkFun_s * ps_linkFun = (linkFun_s *) p_handle;
    for( ; ps_linkFun != math_NULL ; ps_linkFun = ps_linkFun->ps_next )
    	ps_linkFun->fp_fun();
}

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/

