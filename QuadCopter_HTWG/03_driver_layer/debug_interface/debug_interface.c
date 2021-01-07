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
// TODO USB new includes test

// USBlib
#include "usblib/usblib.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdbulk.h"
//structs for bulk transfer
#include "usb_bulk_structs.h"
#include "utils/ustdlib.h"

// setup
#include "qc_setup.h"
#include "peripheral_setup.h"
#include "prioritys.h"

// drivers
#include "debug_interface.h"

// utils
#include "qc_math.h"



/* ------------------------------------------------------------ */
/*				Select the Mode									*/
/* ------------------------------------------------------------ */

#if	( (setup_DEBUG_UART == (setup_DEBUG&setup_MASK_OPT1)) || (setup_DEBUG_UART_USB == (setup_DEBUG&setup_MASK_OPT1)) ) || DOXYGEN

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

		// Init des UART  9600
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
#endif
// new implementation fo USB Debug TODO implement test comment
#if ( (setup_DEBUG_USB == (setup_DEBUG&setup_MASK_OPT1)) || (setup_DEBUG_UART_USB == (setup_DEBUG&setup_MASK_OPT1)) )


    /* ------------------------------------------------------------ */
    /*              Local Defines                                   */
    /* ------------------------------------------------------------ */

    #if (USB0_BASE == periph_DEBUG_USB_BASE)
	    #define TRACE_SYSCTL_PERIPH_GPIO_USB    SYSCTL_PERIPH_GPIOD
        #define TRACE_GPIO_PORT_BASE_USB        GPIO_PORTD_BASE
        #define TRACE_GPIO_PINS_USB_ANALOG   (GPIO_PIN_4 | GPIO_PIN_5)
        #define periph_USB_INT                  INT_USB0
    #else
        #error ERROR: define setup_DEBUG (in qc_setup.h)
    #endif

	static bool b_USBDeviceConnected = false;
	static volatile uint32_t ui32_RXTransmitCounter = 0;


	/* ------------------------------------------------------------ */
    /*              Procedure Definitions                           */
    /* ------------------------------------------------------------ */

    /**
     * \brief   Init peripheral for USB debug interface in Bulk transfer mode
     * \note    to enable this HIDE function define setup_DEBUG in qc_setup.h
     */
	void HIDE_Debug_USB_InterfaceInit(void)
	{


	    // Enable the GPIO peripheral used for USB, and configure the USB pins.
        SysCtlPeripheralEnable(TRACE_SYSCTL_PERIPH_GPIO_USB);
        GPIOPinTypeUSBAnalog(TRACE_GPIO_PORT_BASE_USB, TRACE_GPIO_PINS_USB_ANALOG);

        // Initialize the transmit and receive buffers.
        USBBufferInit(&g_sTxBuffer);
        USBBufferInit(&g_sRxBuffer);

        // Set the USB stack mode to Device mode with VBUS monitoring.
        USBStackModeSet(0, eUSBModeForceDevice, 0);

        // Pass our device information to the USB library and place the device on the bus.
        USBDBulkInit(0, &g_sBulkDevice);

        // Important to set USB interrupt priority lower than I2C (Motor/Sensor interrupt)
        ROM_IntPrioritySet(periph_USB_INT,    priority_USB_ISR);

	}

	//*****************************************************************************
	    //
	    // Function to send data over USB to PC application
	    //
	    // \param pv_txBuffer is the a void pointer to the array of data which should be send
	    // \param ui32_count is the lenght/ size of the data
	    // \param ui8_txDataType is type of data which is send
	    //
	    //
	//*****************************************************************************
	void HIDE_Debug_USB_InterfaceSend(void* pv_txBuffer, uint32_t ui32_count, uint8_t ui8_txDataType){

	    tUSBRingBufObject sTxRing;
	    uint8_t ui8_txArraySend[49];
	    uint32_t ui32_WriteIndex;

	    uint8_t index_n, index_m, index_k;
	    int j, k, n, m;

	    if(b_USBDeviceConnected){



	        // first byte is a indicator which datatype was send
            k = 0;
            ui8_txArraySend[k] = ui8_txDataType;
            k++;

            switch(ui8_txDataType){

            // Values to send are uint16
            case 1: // UINT16

                index_n = 24; // 24 uint16 values
                index_m = 2;  // 2 uint8 values
                index_k = 49; // max 49 bytes

                i16_ui8_union i16_ui8_array[24];

                // fill array with the values given to the function
                for(j= 0; j < index_n; j++){

                if (j < ui32_count){
                    i16_ui8_array[j].int16 = *((uint16_t*)pv_txBuffer + j);
                } else {
                    i16_ui8_array[j].int16 = 0;
                }
                }


                // fill the send array with uint8 data
                for(n = 0; n < index_n; n++){
                    for(m = 0; m < index_m; m++){

                        ui8_txArraySend[k] = i16_ui8_array[n].uint8[m];

                        if(k >= index_k){
                            k = 0;
                        } else{
                            k++;
                        }

                    }
                }
                break;

            // values to send are floats
            case 2: // FLOAT

                index_n = 12; // 12 float values
                index_m = 4;  // 4 uint8 values
                index_k = 49; // max 49 bytes

                f_ui8_union f_ui8_array[12];

                // fill array with the values given to the function
                for(j= 0; j < index_n; j++){

                    if (j < ui32_count){
                        f_ui8_array[j].f = *((float*)pv_txBuffer + j);
                    } else {
                        f_ui8_array[j].f = 0.0f;
                    }
                }

                // fill the send array with uint8 data
                for(n = 0; n < index_n; n++){
                        for(m = 0; m < index_m; m++){

                            ui8_txArraySend[k] = f_ui8_array[n].uint8[m];

                            if(k >= index_k){
                                k = 0;
                            } else{
                                k++;
                            }

                        }
                }
                break;
            }
	    }


	    // flush the buffer so no faulty bytes are there
	    USBBufferFlush(&g_sTxBuffer);

	    // set the write index
	    USBBufferInfoGet(&g_sTxBuffer, &sTxRing);
	    ui32_WriteIndex = sTxRing.ui32WriteIndex;

	    // fill the TX buffer with data and increment the write index after every byte
	    int i;
	    for(i = 0; i < sizeof(ui8_txArraySend); i++){
	       g_pui8USBTxBuffer[ui32_WriteIndex] = ui8_txArraySend[i];
	       ui32_WriteIndex = increment2Limit(ui32_WriteIndex, BULK_BUFFER_SIZE);
	    }

	    // indicates that the client has written data in the transmit buffer and wants to transmit it
	    USBBufferDataWritten(&g_sTxBuffer, sizeof(ui8_txArraySend));




	}

	// TODO Receive PID values
	void HIDE_Debug_USB_InterfaceReceive(uint8_t pid_values_buffer[14]){

	    uint32_t ui32_readIndex;
	    tUSBRingBufObject sRxRing;


	    if(g_pui8USBRxBuffer[0] == 115 && ui32_RXTransmitCounter == 3)
	    {


	        ui32_RXTransmitCounter = 0;

	        USBBufferInfoGet(&g_sRxBuffer, &sRxRing);
	        ui32_readIndex = sRxRing.ui32ReadIndex;

	        int i;
            for(i = 0; i < 14; i++){

                pid_values_buffer[i] = g_pui8USBRxBuffer[ui32_readIndex];
                ui32_readIndex = increment2Limit(ui32_readIndex, BULK_BUFFER_SIZE);
            }
            USBBufferDataRemoved(&g_sRxBuffer, 14);

         }

	}

	//*****************************************************************************
	//
	// Handles bulk driver notifications related to the transmit channel (data to
	// the USB host).
	//
	// \param pvCBData is the client-supplied callback pointer for this channel.
	// \param ui32Event identifies the event we are being notified about.
	// \param ui32MsgValue is an event-specific value.
	// \param pvMsgData is an event-specific pointer.
	//
	// This function is called by the bulk driver to notify us of any events
	// related to operation of the transmit data channel (the IN channel carrying
	// data to the USB host).
	//
	// \return The return value is event-specific.
	//
	//*****************************************************************************
	uint32_t TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
	          void *pvMsgData)
	{
	    return(0);
	}

	//*****************************************************************************
	//
	// Handles bulk driver notifications related to the receive channel (data from
	// the USB host).
	//
	// \param pvCBData is the client-supplied callback pointer for this channel.
	// \param ui32Event identifies the event we are being notified about.
	// \param ui32MsgValue is an event-specific value.
	// \param pvMsgData is an event-specific pointer.
	//
	// This function is called by the bulk driver to notify us of any events
	// related to operation of the receive data channel (the OUT channel carrying
	// data from the USB host).
	//
	// \return The return value is event-specific.
	//
	//*****************************************************************************
	uint32_t RxHandler(void *pvCBData, uint32_t ui32Event,
	               uint32_t ui32MsgValue, void *pvMsgData)
	{
	    //
	    // Which event are we being sent?
	    //
	    switch(ui32Event)
	    {
	        //
	        // We are connected to a host and communication is now possible.
	        //
	        case USB_EVENT_CONNECTED:
	        {
	            //
	            // Flush our buffers.
	            //
	            USBBufferFlush(&g_sTxBuffer);
	            USBBufferFlush(&g_sRxBuffer);

	            b_USBDeviceConnected = true;

	            break;
	        }

	        //
	        // The host has disconnected.
	        //
	        case USB_EVENT_DISCONNECTED:
	        {

	            b_USBDeviceConnected = false;

	            break;
	        }

	        //
	        // A new packet has been received.
	        //
	        case USB_EVENT_RX_AVAILABLE:
	        {
	            ui32_RXTransmitCounter++;
	            if(ui32_RXTransmitCounter > 3)
	                USBBufferFlush(&g_sRxBuffer);
	            break;
	        }

	        //
	        // Ignore SUSPEND and RESUME for now.
	        //
	        case USB_EVENT_SUSPEND:
	        case USB_EVENT_RESUME:
	        {
	            break;
	        }

	        //
	        // Ignore all other events and return 0.
	        //
	        default:
	        {
	            break;
	        }
	    }

	    return(0);
	}

#endif

#if ( setup_DEBUG_NONE == (setup_DEBUG&setup_MASK_OPT1) )
#endif

//	#error ERROR: define setup_DEBUG (in qc_setup.h)
//#endif

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
