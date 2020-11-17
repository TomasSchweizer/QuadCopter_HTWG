/************************************************************************
 *
 *	port_expander.h		Interface Declarations for port_expander.c
 *
 ************************************************************************
 *	Author: 			Tobias Grimm
 *
 ************************************************************************
 *  File Description:
 *
 ************************************************************************
 *  Revision History:
 *
 *	09/10/2015(GrimmT): created
 *
 ************************************************************************/

#ifndef __PORT_EXPANDER_H__
#define	__PORT_EXPANDER_H__

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <stdint.h>


/* ------------------------------------------------------------ */
/*				   	Defines			    						*/
/* ------------------------------------------------------------ */

/* Expander */
#define expander_PORT_COUNT                  1  // Number of used additional ports for memory allocation


#define expander_PORT_A                      0
/*
#define expander_PORT_B                      1
#define expander_PORT_C                      2
*/

/* Expander Port A */
#define expander_LED_RIGHT_FRONT           0x00000001
#define expander_LED_LEFT_FRONT            0x00000002
#define expander_LED_RIGHT_BACK            0x00000004
#define expander_LED_LEFT_BACK             0x00000008

/* ------------------------------------------------------------ */
/*				   Type Definitions			    				*/
/* ------------------------------------------------------------ */



/* Expander Port B */

/* ------------------------------------------------------------ */
/*					API Procedure Declarations					*/
/* ------------------------------------------------------------ */

extern void ExpanderInit();
extern void ExpanderPinWrite(uint32_t Port, uint8_t Pins, uint8_t Value);
extern void ExpanderPinToggel(uint32_t Port, uint8_t Pins);
extern int32_t ExpanderPinRead(uint32_t Port, uint8_t Pins);

/* ------------------------------------------------------------ */
/*					Global Variables							*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */

#endif // __PORT_EXPANDER_H__
