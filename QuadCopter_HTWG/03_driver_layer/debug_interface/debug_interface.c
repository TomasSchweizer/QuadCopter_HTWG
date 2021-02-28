/*===================================================================================================*/
/*  debug_interface.c                                                                                */
/*===================================================================================================*/

/*
*   file   debug_interface.c
*
*   brief  The debug interface allows to read and write bytes over UART or USB
*
*   details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>08/04/2016      <td>Tobias Grimm        <td>Implementation & Last modification of MAs
*   <tr><td>20/11/2020      <td>Tomas Schweizer     <td>Implementation of USB
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*   - TivaWare USB Bulk example
*/
/*====================================================================================================*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

// Standard libraries
#include <stdint.h>
#include <stdbool.h>

//  Hardware specific libraries
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
// USBlib
#include "usblib/usblib.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdbulk.h"

// Setup
#include "qc_setup.h"
#include "peripheral_setup.h"
#include "prioritys.h"

// Drivers
#include "debug_interface.h"
#include "usb_utilities/usb_bulk_structs.h"

// Utilities
#include "qc_math.h"
#include "link_functions.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Select Mode                                                   */
/* ---------------------------------------------------------------------------------------------------*/

#if	 ( setup_DEBUG_UART == (setup_DEBUG&setup_MASK_OPT1) )  || DOXYGEN

	/*------------------------------------------------------------------------------------------------*/
    /*                                     Local defines UART mode                                    */
    /* -----------------------------------------------------------------------------------------------*/

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


    /* -----------------------------------------------------------------------------------------------*/
    /*                                      Procedure Definitions UART mode                           */
    /* -----------------------------------------------------------------------------------------------*/

	/**
	 * @brief	Init peripheral for debug interface
	 *
	 * @return void
	 *
	 * @note	To enable this HIDE function define setup_DEBUG in qc_setup.h
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

		// Init UART at baudrate 9600
		UARTStdioConfig(0, 9600, 16000000);

		// Enable 16x8Bit FIFO
		UARTFIFOEnable(periph_DEBUG_UART_BASE);
	}

	/**
	 * @brief	Get one byte (non blocking) from the debug peripheral
	 *			and write it into i32_buff. If debug peripheral is empty write -1 into i32_buff.
	 *
	 *
	 * @param	i32_buff --> Buffer to write in
	 *
	 * @return  void
	 *
	 * @note	To enable this HIDE function define setup_DEBUG in qc_setup.h
	 */
	void HIDE_Debug_InterfaceGet(int32_t* i32_buff)
	{


        *i32_buff = UARTCharGetNonBlocking(periph_DEBUG_UART_BASE);


	}

	/**
	 * @brief	Write (non blocking) all elements in the array
	 *			into the debug peripheral
	 *
	 * @param	pui8_buff --> Pointer to the uint8_t array
	 * @param	ui32_count -->  Count of elements in the array
	 *
	 * @return  void
	 *
	 * @note	To enable this HIDE function define setup_DEBUG in qc_setup.h
	 */
	void HIDE_Debug_InterfaceSend(const uint8_t *pui8_buff, uint32_t ui32_count)
	{

		// Loop while there are more characters to send.
	        while(ui32_count--)
	        {

	            // Write the next character to the UART.
	            UARTCharPutNonBlocking(periph_DEBUG_UART_BASE, *pui8_buff++);
	        }
	}
#endif


#if ( setup_DEBUG_USB == (setup_DEBUG&setup_MASK_OPT1) )  || DOXYGEN


	/*------------------------------------------------------------------------------------------------*/
	/*                                     Local defines USB mode                                     */
	/* -----------------------------------------------------------------------------------------------*/

    #if (USB0_BASE == periph_DEBUG_USB_BASE)
	    #define TRACE_SYSCTL_PERIPH_GPIO_USB    SYSCTL_PERIPH_GPIOD
        #define TRACE_GPIO_PORT_BASE_USB        GPIO_PORTD_BASE
        #define TRACE_GPIO_PINS_USB_ANALOG      (GPIO_PIN_4 | GPIO_PIN_5)
        #define periph_USB_INT                  INT_USB0
    #else
        #error ERROR: define setup_DEBUG (in qc_setup.h)
    #endif

	/* ------------------------------------------------------------------------------------------------*/
    /*                                      Forward Declarations USB mode                              */
    /* ------------------------------------------------------------------------------------------------*/
	uint32_t TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData); ///< USB transmit handler
	uint32_t RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData); ///< USB receive handler

	/* ---------------------------------------------------------------------------------------------------*/
    /*                                      Global Variables USB mode                                     */
    /* ---------------------------------------------------------------------------------------------------*/

    /* ---------------------------------------------------------------------------------------------------*/
    /*                                      Local Variables USB mode                                      */
    /* ---------------------------------------------------------------------------------------------------*/
    static linkFun_handle_p p_debugUSBLinkFunHandle = 0;    ///< Handle for USB linked functions
    static bool b_USBDeviceConnected = false;               ///< Flag for USB connection
    static volatile uint32_t ui32_RXCounter = 0;            ///< Count variable for USB receive events

	/* -----------------------------------------------------------------------------------------------*/
    /*                                      Procedure Definitions USB mode                            */
    /* -----------------------------------------------------------------------------------------------*/

    /**
     * @brief   Init peripheral for USB debug interface in bulk transfer mode
     *
     * @return  void
     *
     * @note    To enable this HIDE function define setup_DEBUG in qc_setup.h
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

	 /**
     * @brief   Run all USB functions in the function handle
     *
     * @return  void
     *
     * @note    To enable this HIDE function define setup_DEBUG in qc_setup.h
     */
	void HIDE_Debug_USB_Com(void){

	    LinkFun_RunAllFun(p_debugUSBLinkFunHandle);
	}

	/**
     * @brief   Insert a USB communication function into the function handle
     *
     * @param   fp_com --> An USB function pointer
     * @param   insert --> Flag if function should be inserted or not
     *
     * @return  void
     *
     * @note    To enable this HIDE function define setup_DEBUG in qc_setup.h
     */
	void HIDE_Debug_USB_InsertComFun(usb_com_fp fp_com, uint8_t insert){


	    if(fp_com!=0)
	    {
	        if(insert == 1)
	        {
	            LinkFun_Insert(&p_debugUSBLinkFunHandle, fp_com); // link into usb com list
	        }
	    }


	}


	/**
     * @brief   Function to send data over USB to PC application
     *
     * @param   pv_txBuffer --> Void pointer to the array of data which should be send
     * @param   ui32_count --> The lenghth/ size of the data
     * @param   ui8_txDataType --> Type of send data
     *
     * @return  void
     *
     * @note    To enable this HIDE function define setup_DEBUG in qc_setup.h
     */
	void HIDE_Debug_USB_InterfaceSend(void* pv_txBuffer, uint32_t ui32_count, uint8_t ui8_txDataType){

	    tUSBRingBufObject sTxRing;
	    uint8_t ui8_txArraySend[49];
	    uint32_t ui32_WriteIndex;

	    uint8_t index_n, index_m, index_k;
	    int j, k, n, m;

	    if(b_USBDeviceConnected){

	        // First byte is a indicator which data type is send
            k = 0;
            ui8_txArraySend[k] = ui8_txDataType;
            k++;

            switch(ui8_txDataType){

            // Values to send are uint16
            case 1: // UINT16

                index_n = 24; // 24 uint16 values
                index_m = 2;  // 2 uint8 values
                index_k = 49; // Max 49 bytes

                i16_ui8_union i16_ui8_array[24];

                // Fill array with the values given to the function
                for(j= 0; j < index_n; j++){

                if (j < ui32_count){
                    i16_ui8_array[j].int16 = *((uint16_t*)pv_txBuffer + j);
                } else {
                    i16_ui8_array[j].int16 = 0;
                }
                }


                // Fill the send array with uint8 data
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

            // Values to send are floats
            case 2: // FLOAT

                index_n = 12; // 12 float values
                index_m = 4;  // 4 uint8 values
                index_k = 49; // Max 49 bytes

                f_ui8_union f_ui8_array[12];

                // Fill array with the values given to the function
                for(j= 0; j < index_n; j++){

                    if (j < ui32_count){
                        f_ui8_array[j].f = *((float*)pv_txBuffer + j);
                    } else {
                        f_ui8_array[j].f = 0.0f;
                    }
                }

                // Fill the send array with uint8 data
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


	    // Flush the buffer so no faulty bytes are there
	    USBBufferFlush(&g_sTxBuffer);

	    // Set the write index
	    USBBufferInfoGet(&g_sTxBuffer, &sTxRing);
	    ui32_WriteIndex = sTxRing.ui32WriteIndex;

	    // Fill the TX buffer with data and increment the write index after every byte
	    int i;
	    for(i = 0; i < sizeof(ui8_txArraySend); i++){
	       g_pui8USBTxBuffer[ui32_WriteIndex] = ui8_txArraySend[i];
	       ui32_WriteIndex = increment2Limit(ui32_WriteIndex, BULK_BUFFER_SIZE);
	    }

	    // Indicates that the client has written data in the transmit buffer and wants to transmit it
	    USBBufferDataWritten(&g_sTxBuffer, sizeof(ui8_txArraySend));




	}

    /**
     * @brief   Function to receive data over USB from PC application
     *
     * @param   ui8_buffer --> Array to copy the received data into
     *
     * @return  void
     *
     * @note    To enable this HIDE function define setup_DEBUG in qc_setup.h
     */
	void HIDE_Debug_USB_InterfaceReceive(uint8_t ui8_buffer[14]){

	    uint32_t ui32_readIndex;
	    tUSBRingBufObject sRxRing;


	    // Check if the first byte is an 's'/115 and if 3 USB receive events have occured
	    if(g_pui8USBRxBuffer[0] == 115 && ui32_RXCounter == 3)
	    {
	        // Reset RX Counter
	        ui32_RXCounter = 0;

	        // Set read index
	        USBBufferInfoGet(&g_sRxBuffer, &sRxRing);
	        ui32_readIndex = sRxRing.ui32ReadIndex;

	        // Copy the receive buffer into other array
	        int i;
            for(i = 0; i < 14; i++){

                ui8_buffer[i] = g_pui8USBRxBuffer[ui32_readIndex];
                ui32_readIndex = increment2Limit(ui32_readIndex, BULK_BUFFER_SIZE);
            }

            // Delete 14 bytes out of the receive buffer, so that no faulty bytes remain
            USBBufferDataRemoved(&g_sRxBuffer, 14);

         }

	}

    /**
     * @brief   Handles bulk driver notifications related to the transmit channel (data to the USB host).
     *
     * @param   pvCBData --> Is the client-supplied callback pointer for this channel.
     * @param   ui32Event  --> Identifies the event we are being notified about.
     * @param   ui32MsgValue --> Is an event-specific value.
     * @param   pvMsgData --> Is an event-specific pointer.
     *
     * @return  The return value is event-specific.
     *
     */
    uint32_t TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
              void *pvMsgData)
    {
        return(0);
    }



    /**
     * @brief   Handles bulk driver notifications related to the receive channel (data from the USB host).
     *
     * @param   pvCBData --> Is the client-supplied callback pointer for this channel.
     * @param   ui32Event --> Identifies the event we are being notified about.
     * @param   ui32MsgValue --> Is an event-specific value.
     * @param   pvMsgData --> Is an event-specific pointer.
     *
     * @return The return value is event-specific.
     *
     */
	uint32_t RxHandler(void *pvCBData, uint32_t ui32Event,
	               uint32_t ui32MsgValue, void *pvMsgData)
	{
	    // Which event is being sent
	    switch(ui32Event)
	    {
	        // We are connected to a host and communication is now possible.
	        case USB_EVENT_CONNECTED:
	        {

	            // Flush our buffers.
	            USBBufferFlush(&g_sTxBuffer);
	            USBBufferFlush(&g_sRxBuffer);

	            // Set USB device connected flag
	            b_USBDeviceConnected = true;

	            break;
	        }

	        // The host has disconnected.
	        case USB_EVENT_DISCONNECTED:
	        {
	            // Set USB device connected flag
	            b_USBDeviceConnected = false;

	            break;
	        }

	        // A new packet has been received.
	        case USB_EVENT_RX_AVAILABLE:
	        {
	            // increase RX counter
	            ui32_RXCounter++;

	            // if counter over 3 packets flush the receive buffer
	            if(ui32_RXCounter > 3)
	                USBBufferFlush(&g_sRxBuffer);
	            break;
	        }

	        // Ignore SUSPEND and RESUME
	        case USB_EVENT_SUSPEND:
	        case USB_EVENT_RESUME:
	        {
	            break;
	        }

	        // Ignore all other events and return 0.
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

#if	( setup_DEV_DEBUG_PINS ) || DOXYGEN

    /*------------------------------------------------------------------------------------------------*/
    /*                                     Local defines debug pins                                   */
    /* -----------------------------------------------------------------------------------------------*/

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

	/*------------------------------------------------------------------------------------------------*/
	/*                                     Procedure definitions debug pins                           */
    /* -----------------------------------------------------------------------------------------------*/

	/**
	 * @brief	Init peripheral for the debug pins
	 *
	 * @return  void
	 *
	 * @note	To enable this HIDE function set setup_DEV_DEBUG_PINS in qc_setup.h
	 */
	void HIDE_Debug_PinsInit(void)
	{
		// Init debug Pin 1
		#if(periph_NONE != periph_DEBUG_PIN1)
			ROM_SysCtlPeripheralEnable(DEBUG_SYSCTL_PERIPH_GPIO1);
			ROM_GPIOPinWrite(DEBUG_PORT1, DEBUG_PIN1, 0x00);
			ROM_GPIOPinTypeGPIOOutput(DEBUG_PORT1, DEBUG_PIN1);
			ROM_GPIOPinWrite(DEBUG_PORT1, DEBUG_PIN1, 0x00);
		#endif

		// Init debug Pin 2
		#if(periph_NONE != periph_DEBUG_PIN2)
			ROM_SysCtlPeripheralEnable(DEBUG_SYSCTL_PERIPH_GPIO2);
			ROM_GPIOPinWrite(DEBUG_PORT2, DEBUG_PIN2, 0x00);
			ROM_GPIOPinTypeGPIOOutput(DEBUG_PORT2, DEBUG_PIN2);
			ROM_GPIOPinWrite(DEBUG_PORT2, DEBUG_PIN2, 0x00);
		#endif
	}

	/**
	 * @brief	Write the desired debug pin
	 *
	 * @param	ui32_port --> Use the define debug_PIN1 or debug_PIN2 here
	 * @param	ui8_pin --> Use the define debug_PIN1 or debug_PIN2 here
	 * @param	ui8_val	--> Use the define debug_CLEAR or debug_SET here
	 *
	 * @return  void
	 *
	 * @note	to enable this HIDE function set setup_DEV_DEBUG_PINS in qc_setup.h
	 */
	void HIDE_Debug_PinWrite(uint32_t ui32_port,uint8_t ui8_pin,uint8_t ui8_val)
	{
		if(ui8_pin!=(periph_NONE&periph_MASK_PIN))
			ROM_GPIOPinWrite(ui32_port, ui8_pin, ui8_val);
	}

#endif
