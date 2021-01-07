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

#define debug_INT16                 1
#define debug_FLOAT                 2


/* ------------------------------------------------------------ */
/*				   Type Definitions			    				*/
/* ------------------------------------------------------------ */

/**
 * \brief	converter between float/int16_t and uint8t Array
 */
#if ( (setup_DEBUG_USB == (setup_DEBUG&setup_MASK_OPT1))  || (setup_DEBUG_UART_USB == (setup_DEBUG&setup_MASK_OPT1)) )

    typedef union f_ui8 {

        float f;
        uint8_t uint8[4];

    }f_ui8_union;

    typedef union ui16_ui8 {

        int16_t int16;
        uint8_t uint8[2];

    }i16_ui8_union;

#endif
/* ------------------------------------------------------------ */
/*					Variable Declarations						*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*					Procedure Declarations						*/
/* ------------------------------------------------------------ */

#if	( (setup_DEBUG_UART == (setup_DEBUG&setup_MASK_OPT1)) || (setup_DEBUG_UART_USB == (setup_DEBUG&setup_MASK_OPT1)) ) || DOXYGEN

	extern void HIDE_Debug_InterfaceInit(void);
	extern void HIDE_Debug_InterfaceGet(int32_t* i32_buff);
	extern void HIDE_Debug_InterfaceSend(const uint8_t *pui8Buffer, uint32_t ui32Count);

#else

	#define HIDE_Debug_InterfaceInit()						// this define will be kicked off from the preprocessor
	#define HIDE_Debug_InterfaceGet(i32_buff)				// this define will be kicked off from the preprocessor
	#define HIDE_Debug_InterfaceSend(pui8Buffer,ui32Count)	// this define will be kicked off from the preprocessor

#endif

// TODO new implementation of USB test and comment
#if	( (setup_DEBUG_USB == (setup_DEBUG&setup_MASK_OPT1)) || (setup_DEBUG_UART_USB == (setup_DEBUG&setup_MASK_OPT1)) ) || DOXYGEN

	extern void HIDE_Debug_USB_InterfaceInit(void);
	extern void HIDE_Debug_USB_InterfaceSend(void* pv_txBuffer, uint32_t ui32_count, uint8_t ui8_txDataType);
	extern uint32_t TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData);
	extern uint32_t RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData);
	extern void HIDE_Debug_USB_InterfaceReceive(uint8_t pid_values_buffer[12]);

#else

    #define HIDE_Debug_USB_InterfaceInit()                                          // this define will be kicked off from the preprocessor
    #define HIDE_Debug_USB_InterfaceSend(pv_txBuffer, ui32_count, ui8_txDataType)   // this define will be kicked off from the preprocessor
    #define HIDE_Debug_USB_InterfaceReceive(pid_values_buffer)
	#define TxHandler(pvCBData, ui32Event, ui32MsgValue, pvMsgData);
    #define RxHandler(pvCBData, ui32Event, ui32MsgValue, pvMsgData);                // this define will be kicked off from the preprocessor


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
