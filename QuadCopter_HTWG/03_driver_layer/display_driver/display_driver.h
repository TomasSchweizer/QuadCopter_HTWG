/*===================================================================================================*/
/*  display_driver.h                                                                                 */
/*===================================================================================================*/

/**
*   @file   display_driver.h
*
*   @brief  Functions to interact with the display.
*
*   @details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>15/05/2016      <td>Tobias Grimm        <td>Implementation & lasz modification MAs
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*   -
*/
/*====================================================================================================*/

#ifndef __DISPLAY_WRAPPER_H__
#define	__DISPLAY_WRAPPER_H__

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

#include <u8g_port.h>

// setup
#include "qc_setup.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/

/// function pointer to link the desired draw functions
typedef void (*display_draw_fp)(void);

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/
extern u8g_t gs_display;    // Display instance

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/

#if	( setup_DISPLAY_NONE != (setup_DISPLAY&setup_MASK_OPT1) ) || DOXYGEN
	extern void HIDE_Display_Init(void);
	extern void HIDE_Display_InsertDrawFun(display_draw_fp fp_draw);
	extern void HIDE_Display_Redraw(void);
#else
	#define HIDE_Display_Init()					// this define will be kicked off from the preprocessor
	#define HIDE_Display_InsertDrawFun(fp_draw)	// this define will be kicked off from the preprocessor
	#define HIDE_Display_Redraw()				// this define will be kicked off from the preprocessor
#endif

#endif // __DISPLAY_WRAPPER_H__

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
