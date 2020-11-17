/**
 * 		@file 	debug_interface.h
 * 		@brief	The debug interface can write or read Bytes
 *  			from or to e.g. a computer.
 *//*	@author Tobias Grimm
 * 		@date 	08.04.2016	(last modified)
 */

#ifndef __DEBUG_INTERFACE_H__
#define	__DEBUG_INTERFACE_H__

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <stdint.h>
#include <stdbool.h>

// Hardware Specific
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"

// setup
#include "qc_setup.h"
#include "peripheral_setup.h"



/* ------------------------------------------------------------ */
/*				   	Defines			    						*/
/* ------------------------------------------------------------ */

#define debug_PIN1					periph_SPLIT_PORT_PIN(periph_DEBUG_PIN1)
#define debug_PIN2					periph_SPLIT_PORT_PIN(periph_DEBUG_PIN2)
#define debug_CLEAR					0x00
#define debug_SET					0xFF

/* ------------------------------------------------------------ */
/*				   Type Definitions			    				*/
/* ------------------------------------------------------------ */

/**
 * \brief	converter between float and char Array
 */
union {
	float f;	/**< the float number */
	char c[4];	/**< the char array number */
} f2c_u;

/* ------------------------------------------------------------ */
/*					Variable Declarations						*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*					Procedure Declarations						*/
/* ------------------------------------------------------------ */

#if	( setup_DEBUG_NONE != (setup_DEBUG&setup_MASK_OPT1) ) || DOXYGEN

	extern void HIDE_Debug_InterfaceInit(void);
	extern void HIDE_Debug_InterfaceGet(int32_t* i32_buff);
	extern void HIDE_Debug_InterfaceSend(const uint8_t *pui8Buffer, uint32_t ui32Count);

#else

	#define HIDE_Debug_InterfaceInit()						// this define will be kicked off from the preprocessor
	#define HIDE_Debug_InterfaceGet(i32_buff)				// this define will be kicked off from the preprocessor
	#define HIDE_Debug_InterfaceSend(pui8Buffer,ui32Count)	// this define will be kicked off from the preprocessor

#endif

#if	( setup_DEV_DEBUG_PINS ) || DOXYGEN

	extern void HIDE_Debug_PinsInit(void);
	extern void HIDE_Debug_PinWrite(uint32_t ui32_port,uint8_t ui8_pin,uint8_t ui8_val);

#else

	#define HIDE_Debug_PinsInit()								// this define will be kicked off from the preprocessor
	#define HIDE_Debug_PinWrite(ui32_port,ui8_pin,ui8_val)		// this define will be kicked off from the preprocessor

#endif

/* ------------------------------------------------------------ */

#endif // __DEBUG_INTERFACE_H__
