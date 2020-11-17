/**
 * 		@file 	display_driver.h
 * 		@brief	Functions to interact with the display.
 *
 * 				It is a wrapper interface declaration for u8g_port.c
 *//*	@author Tobias Grimm
 * 		@date 	15.05.2016	(last modified)
 */

#ifndef __DISPLAY_WRAPPER_H__
#define	__DISPLAY_WRAPPER_H__

/* ------------------------------------------------------------ */
/*					Include File Definitions					*/
/* ------------------------------------------------------------ */

#include <u8g_port.h>

// setup
#include "qc_setup.h"

/* ------------------------------------------------------------ */
/*				   	Defines			    						*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				   	Type Definitions			    			*/
/* ------------------------------------------------------------ */

/**
 * \brief	function pointer to draw the desired stuff
 */
typedef void (*display_draw_fp)(void);

/* ------------------------------------------------------------ */
/*					API Procedure Declarations					*/
/* ------------------------------------------------------------ */

#if	( setup_DISPLAY_NONE != (setup_DISPLAY&setup_MASK_OPT1) ) || DOXYGEN
	extern void HIDE_Display_Init(void);
	extern void HIDE_Display_InsertDrawFun(display_draw_fp fp_draw);
	extern void HIDE_Display_Redraw(void);
#else
	#define HIDE_Display_Init()					// this define will be kicked off from the preprocessor
	#define HIDE_Display_InsertDrawFun(fp_draw)	// this define will be kicked off from the preprocessor
	#define HIDE_Display_Redraw()				// this define will be kicked off from the preprocessor
#endif

/* ------------------------------------------------------------ */
/*					Global Variables							*/
/* ------------------------------------------------------------ */

extern u8g_t gs_display;

/* ------------------------------------------------------------ */

#endif // __DISPLAY_WRAPPER_H__
