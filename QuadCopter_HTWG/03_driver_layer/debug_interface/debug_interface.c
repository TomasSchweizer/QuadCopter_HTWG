/**
 * 		@file 	debug_interface.c
 * 		@brief	The debug interface can write or read Bytes
 *  			from or to e.g. a computer.
 *//*	@author Tobias Grimm
 * 		@date 	22.03.2016	(last modified)
 */

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <stdint.h>
#include <stdbool.h>

//  Hardware Specific
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "utils/uartstdio.h"

// setup
#include "qc_setup.h"
#include "peripheral_setup.h"

// drivers
#include "debug_interface.h"

/* ------------------------------------------------------------ */
/*				Select the Mode									*/
/* ------------------------------------------------------------ */

#if	( setup_DEBUG_UART == (setup_DEBUG&setup_MASK_OPT1) ) || DOXYGEN

	/* ------------------------------------------------------------ */
	/*				Local Defines									*/
	/* ------------------------------------------------------------ */

	#if ( UART0_BASE == periph_DEBUG_UART_BASE )
		#define  TRACE_SYSCTL_PERIPH_UART	SYSCTL_PERIPH_UART0
		#define  TRACE_SYSCTL_PERIPH_GPIO	SYSCTL_PERIPH_GPIOA
		#define  TRACE_GPIO_PA_URX			GPIO_PA0_U0RX
		#define  TRACE_GPIO_PA_UTX			GPIO_PA1_U0TX
		#define  TRACE_GPIO_PORT_BASE		GPIO_PORTA_BASE
		#define  TRACE_GPIO_PIN_RX			GPIO_PIN_0
		#define  TRACE_GPIO_PIN_TX			GPIO_PIN_1
	#else
		#error	ERROR implement the defines above here
	#endif


	/* ------------------------------------------------------------ */
	/*				Procedure Definitions							*/
	/* ------------------------------------------------------------ */

	/**
	 * \brief	Init peripheral for debug interface
	 * \note	to enable this HIDE function define setup_DEBUG in qc_setup.h
	 */
	void HIDE_Debug_InterfaceInit(void)
	{
		// Enable peripheral for UART
		SysCtlPeripheralEnable(TRACE_SYSCTL_PERIPH_UART);
		SysCtlPeripheralEnable(TRACE_SYSCTL_PERIPH_GPIO);

		// Set GPIO A0 and A1 as UART pins.
		GPIOPinConfigure(TRACE_GPIO_PA_URX);
		GPIOPinConfigure(TRACE_GPIO_PA_UTX);
		GPIOPinTypeUART(TRACE_GPIO_PORT_BASE, TRACE_GPIO_PIN_RX | TRACE_GPIO_PIN_TX);

	   // Use the internal 16MHz oscillator as the UART clock source.
		UARTClockSourceSet(periph_DEBUG_UART_BASE, UART_CLOCK_PIOSC);

		// Init des UART  115200
		UARTStdioConfig(0, 9600, 16000000);

		// Enable 16x8Bit FIFO
		UARTFIFOEnable(periph_DEBUG_UART_BASE);
	}

	/**
	 * \brief	get one byte (non blocking) from the debug peripheral
	 *			and write it into i32_buff.
	 *
	 *			if debug peripheral is empty write -1 into i32_buff.
	 * \param	i32_buff	buffer to write in
	 * \note	to enable this HIDE function define setup_DEBUG in qc_setup.h
	 */
	void HIDE_Debug_InterfaceGet(int32_t* i32_buff)
	{
		*i32_buff = UARTCharGetNonBlocking(periph_DEBUG_UART_BASE);
	}

	/**
	 * \brief	write (non blocking) all elements in the array
	 *			into the debug peripheral
	 * \param	pui8_buff	pointer to the uint8_t array
	 * \param	ui32_count  count of elements in the array
	 * \note	to enable this HIDE function define setup_DEBUG in qc_setup.h
	 */
	void HIDE_Debug_InterfaceSend(const uint8_t *pui8_buff, uint32_t ui32_count)
	{

//	    int p = 0;
//	    if(!UARTBusy(periph_DEBUG_UART_BASE) || p == 0){
//	        p++;
		// Loop while there are more characters to send.
	        while(ui32_count--)
	        {

	            // Write the next character to the UART.
	            UARTCharPutNonBlocking(periph_DEBUG_UART_BASE, *pui8_buff++);
	        }
//	    }
	}

#elif ( setup_DEBUG_USB == (setup_DEBUG&setup_MASK_OPT1) )
	#error ERROR: define setup_DEBUG (in qc_setup.h)
#elif ( setup_DEBUG_NONE == (setup_DEBUG&setup_MASK_OPT1) )
#else
	#error ERROR: define setup_DEBUG (in qc_setup.h)
#endif

#if	( setup_DEV_DEBUG_PINS ) || DOXYGEN

	/* ------------------------------------------------------------ */
	/*				Local Defines									*/
	/* ------------------------------------------------------------ */

	#define DEBUG_PIN1 						(periph_DEBUG_PIN1 & periph_MASK_PIN)
	#define DEBUG_PORT1						(periph_DEBUG_PIN1 & periph_MASK_PORT)
	#if ( GPIO_PORTA_BASE == DEBUG_PORT1 )
		#define  DEBUG_SYSCTL_PERIPH_GPIO1	SYSCTL_PERIPH_GPIOA
	#elif ( GPIO_PORTF_BASE == DEBUG_PORT1 )
		#define  DEBUG_SYSCTL_PERIPH_GPIO1	SYSCTL_PERIPH_GPIOF
	#elif(periph_NONE == periph_DEBUG_PIN2)

	#else
		#error	ERROR implement the defines above here
	#endif

	#define DEBUG_PIN2 						(periph_DEBUG_PIN2 & periph_MASK_PIN)
	#define DEBUG_PORT2						(periph_DEBUG_PIN2 & periph_MASK_PORT)
	#if ( GPIO_PORTA_BASE == DEBUG_PORT2 )
		#define  DEBUG_SYSCTL_PERIPH_GPIO2	SYSCTL_PERIPH_GPIOA
	#elif ( GPIO_PORTF_BASE == DEBUG_PORT2 )
		#define  DEBUG_SYSCTL_PERIPH_GPIO2	SYSCTL_PERIPH_GPIOF
	#elif(periph_NONE == periph_DEBUG_PIN2)

	#else
		#error	ERROR implement the defines above here
	#endif

	/* ------------------------------------------------------------ */
	/*				Procedure Definitions							*/
	/* ------------------------------------------------------------ */

	/**
	 * \brief	Init peripheral for the debug pins
	 * \note	to enable this HIDE function set setup_DEV_DEBUG_PINS in qc_setup.h
	 */
	void HIDE_Debug_PinsInit(void)
	{
		// init debug Pin 1
		#if(periph_NONE != periph_DEBUG_PIN1)
			ROM_SysCtlPeripheralEnable(DEBUG_SYSCTL_PERIPH_GPIO1);
			ROM_GPIOPinWrite(DEBUG_PORT1, DEBUG_PIN1, 0x00);
			ROM_GPIOPinTypeGPIOOutput(DEBUG_PORT1, DEBUG_PIN1);
			ROM_GPIOPinWrite(DEBUG_PORT1, DEBUG_PIN1, 0x00);
		#endif

		// init debug Pin 2
		#if(periph_NONE != periph_DEBUG_PIN2)
			ROM_SysCtlPeripheralEnable(DEBUG_SYSCTL_PERIPH_GPIO2);
			ROM_GPIOPinWrite(DEBUG_PORT2, DEBUG_PIN2, 0x00);
			ROM_GPIOPinTypeGPIOOutput(DEBUG_PORT2, DEBUG_PIN2);
			ROM_GPIOPinWrite(DEBUG_PORT2, DEBUG_PIN2, 0x00);
		#endif
	}

	/**
	 * \brief	write on the desired debug pin
	 * \param	ui32_port	use the define debug_PIN1 or debug_PIN2 here
	 * \param	ui8_pin		use the define debug_PIN1 or debug_PIN2 here
	 * \param	ui8_val		use the define debug_CLEAR or debug_SET here
	 * \note	to enable this HIDE function set setup_DEV_DEBUG_PINS in qc_setup.h
	 */
	void HIDE_Debug_PinWrite(uint32_t ui32_port,uint8_t ui8_pin,uint8_t ui8_val)
	{
		if(ui8_pin!=(periph_NONE&periph_MASK_PIN))
			ROM_GPIOPinWrite(ui32_port, ui8_pin, ui8_val);
	}

#endif
