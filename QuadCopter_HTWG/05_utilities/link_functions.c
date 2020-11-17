/**
 * 		@file 	link_functions.c
 * 		@brief	link functions together (linked list)
 *
 *  			and run all linked funktions
 *//*	@author Tobias Grimm
 * 		@date 	15.05.2016	(last modified)
 */


/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */


#include <stdint.h>

// freeRTOS
#include "FreeRTOS.h"		// for pvPortMalloc

// utils
#include "qc_math.h"
#include "link_functions.h"

/* ------------------------------------------------------------ */
/*				Local Defines									*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */

typedef struct linkFun_s
 {
     linkFun_fp 		fp_fun;		/**< function pointer to desired function */
     struct linkFun_s*  ps_next; 	/**< pointer to the next element (linked List) */
 }linkFun_s;

/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

 /**
  * \brief	insert a function into the linkFun_handle_p instance (linked List)
  * \param	pp_handle	pointer to handle of the LinkFunction instance
  *	\param	fp_fun		function to insert
  */
void LinkFun_Insert(linkFun_handle_p* pp_handle, linkFun_fp fp_fun)
{
	linkFun_s ** pps_linkFun = (linkFun_s **) pp_handle;
	linkFun_s *  ps_newLinkFun;

	// walk to the end of the linked List
    while( *pps_linkFun != math_NULL )
        pps_linkFun = &(*pps_linkFun)->ps_next;

    ps_newLinkFun = (linkFun_s *) pvPortMalloc(sizeof(*ps_newLinkFun));
    ps_newLinkFun->fp_fun = fp_fun;
    ps_newLinkFun->ps_next = math_NULL;

    *pps_linkFun = ps_newLinkFun;
}

/**
 * \brief	run all linked functions
 * \param	p_handle	handle to the LinkFunction instance
 */
void LinkFun_RunAllFun(linkFun_handle_p p_handle)
{
	linkFun_s * ps_linkFun = (linkFun_s *) p_handle;
    for( ; ps_linkFun != math_NULL ; ps_linkFun = ps_linkFun->ps_next )
    	ps_linkFun->fp_fun();
}
