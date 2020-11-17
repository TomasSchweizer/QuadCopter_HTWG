/**
 * 		@file 	u8g_port.c
 * 		@brief	portet stuff to use the ui8glib for a lot of displays
 *//*	@author Tobias Grimm
 * 		@date 	15.05.2016	(last modified)
 */

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <stdint.h>
#include <stdbool.h>
#include <u8g_port.h>

// utils
#include "busy_delay.h"
#include "link_functions.h"

//drivers
#include "display_driver.h"

// setup
#include "qc_setup.h"
#include "peripheral_setup.h"

/* ------------------------------------------------------------ */
/*				Local Defines									*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

void u8g_Delay(uint16_t val);
void u8g_MicroDelay(void);
void u8g_10MicroDelay(void);

/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */

/**
 * \brief	info and storage for the display driver
 */
u8g_t gs_display;

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

linkFun_handle_p p_displayLinkFunHandle=0;

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

void u8g_Delay(uint16_t val)
{
	BusyDelay_Ms(val);
}

void u8g_MicroDelay(void)
{
	BusyDelay_Us(1);
}

void u8g_10MicroDelay(void)
{
	BusyDelay_Us(10);
}

/* ------------------------------------------------------------ */
/*				Select the Mode									*/
/* ------------------------------------------------------------ */

#if   ( setup_DISPLAY_SPI == (setup_DISPLAY&setup_MASK_OPT1) ) || DOXYGEN

	/* ------------------------------------------------------------ */
	/*				Include File Definitions						*/
	/* ------------------------------------------------------------ */

	//  Hardware Specific
	#include "inc/hw_memmap.h"
	#include "inc/hw_types.h"
	#include "inc/hw_ints.h"
	#include "inc/hw_gpio.h"
	#include "inc/hw_ssi.h"
	#include "driverlib/sysctl.h"
	#include "driverlib/pin_map.h"
	#include "driverlib/gpio.h"
	#include "driverlib/rom.h"
	#include "driverlib/ssi.h"
	#include "driverlib/udma.h"

	/* ------------------------------------------------------------ */
	/*				Local Defines									*/
	/* ------------------------------------------------------------ */

	#define 	PIN_DC 			( 1 )	// display or command select
	#define 	PIN_CS 			( 2 )	// Chip Select
	#define 	PIN_RST 		( 3 )	// Reset

	//
	// define the desired peripheral setup (see peripheral_setup.h)
	//
	#if( periph_DISPLAY_SSI_BASE == SSI3_BASE)
		#define DISPLAY_SSI_SYSCTL_PERIPH					SYSCTL_PERIPH_GPIOD
		#define DISPLAY_SSI_PORT_BASE						GPIO_PORTD_BASE
		#define DISPLAY_SSI_CLK_PIN							GPIO_PIN_0			// clock
		#define DISPLAY_SSI_CLK_PIN_CONFIG					GPIO_PD0_SSI3CLK
		#define DISPLAY_SSI_TX_PIN							GPIO_PIN_3			// transmit
		#define DISPLAY_SSI_TX_PIN_CONFIG					GPIO_PD3_SSI3TX
	#else
		#error	ERROR implement the defines above here
	#endif

	#define DISPLAY_FSS_PIN									(periph_DISPLAY_PIN_FSS & periph_MASK_PIN)		// frame signal (chip select)
	#define DISPLAY_FSS_PORT_BASE							(periph_DISPLAY_PIN_FSS & periph_MASK_PORT)
	#if( DISPLAY_FSS_PORT_BASE == GPIO_PORTD_BASE)
		#define DISPLAY_FSS_SYSCTL_PERIPH					SYSCTL_PERIPH_GPIOD
	#elif( periph_DISPLAY_PIN_FSS == periph_NONE )

	#else
		#error	ERROR implement the defines above here
	#endif

	#define DISPLAY_RST_PIN									(periph_DISPLAY_PIN_RST & periph_MASK_PIN)
	#define DISPLAY_RST_PORT_BASE							(periph_DISPLAY_PIN_RST & periph_MASK_PORT)			// Reset (low aktive)
	#if ( DISPLAY_RST_PORT_BASE == GPIO_PORTE_BASE )
		#define DISPLAY_RST_SYSCTL_PERIPH					SYSCTL_PERIPH_GPIOE
	#elif( DISPLAY_RST_PORT_BASE == GPIO_PORTD_BASE )
		#define DISPLAY_RST_SYSCTL_PERIPH					SYSCTL_PERIPH_GPIOD
	#else
		#error	ERROR implement the defines above here
	#endif

	#define DISPLAY_DC_PIN									(periph_DISPLAY_PIN_DC & periph_MASK_PIN)			// display or command select (high for display bufferaccess, low for command access)
	#define DISPLAY_DC_PORT_BASE							(periph_DISPLAY_PIN_DC & periph_MASK_PORT)
	#if ( DISPLAY_DC_PORT_BASE == GPIO_PORTD_BASE )
		#define DISPLAY_DC_SYSCTL_PERIPH					SYSCTL_PERIPH_GPIOD
	#else
		#error	ERROR implement the defines above here
	#endif


	#define DISPLAY_PWR_SCREEN_PIN							(periph_DISPLAY_PIN_PWR_SCREEN & periph_MASK_PIN)			// turn on/off power to the OLED display itself (low aktive)
	#define DISPLAY_PWR_SCREEN_PORT_BASE					(periph_DISPLAY_PIN_PWR_SCREEN & periph_MASK_PORT)
	#if ( DISPLAY_PWR_SCREEN_PORT_BASE == GPIO_PORTF_BASE )
		#define DISPLAY_PWR_SCREEN_SYSCTL_PERIPH			SYSCTL_PERIPH_GPIOF
	#elif( periph_DISPLAY_PIN_PWR_SCREEN == periph_NONE )

	#else
		#error	ERROR implement the defines above here
	#endif

	#define DISPLAY_PWR_LOGIC_PIN							(periph_DISPLAY_PIN_PWR_LOGIC & periph_MASK_PIN)
	#define DISPLAY_PWR_LOGIC_PORT_BASE						(periph_DISPLAY_PIN_PWR_LOGIC & periph_MASK_PORT)			// turn on/off the power to the logic of the display (low aktive)
	#if ( DISPLAY_PWR_LOGIC_PORT_BASE == GPIO_PORTE_BASE )
		#define DISPLAY_PWR_LOGIC_SYSCTL_PERIPH				SYSCTL_PERIPH_GPIOE
	#elif( periph_DISPLAY_PIN_PWR_LOGIC == periph_NONE )

	#else
		#error	ERROR implement the defines above here
	#endif

	/* ------------------------------------------------------------ */
	/*				Local Type Definitions							*/
	/* ------------------------------------------------------------ */

	/* ------------------------------------------------------------ */
	/*				Forward Declarations							*/
	/* ------------------------------------------------------------ */

	void Display_Update(display_draw_fp fp_draw);
	static void set_gpio_level(uint16_t pin, uint8_t level);
	static void spi_init(uint32_t ns);
	static void spi_out(uint8_t data);
	uint8_t u8g_com_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);

	/* ------------------------------------------------------------ */
	/*				Local Variables									*/
	/* ------------------------------------------------------------ */

	/* ------------------------------------------------------------ */
	/*				Procedure Definitions							*/
	/* ------------------------------------------------------------ */

	/**
	 * \brief	Redraw all inserted functions on the display.
	 *
	 * 			Add a function with HIDE_Display_InsertDrawFun
	 *			into the display draw functionLinkedList (do this only once)
	 * \note	to enable this HIDE function define setup_DISPLAY in qc_setup.h
	 */
	void HIDE_Display_Redraw(void)
	{
		// picture loop
		u8g_FirstPage(&gs_display);
		do
		{
			LinkFun_RunAllFun(p_displayLinkFunHandle);
		} while ( u8g_NextPage(&gs_display) );
	}

	/**
	 * \brief	Insert a function into the display draw
	 *			functionLinkedList.
	 *
	 *			(this should be performed before scheduler starts)
	 *			e.g. in the init of a driver or task
	 *			(0 pointers won't be inserted)
	 * \param	fp_draw		the function with the draw commands
	 * \note	to enable this HIDE function define setup_DISPLAY in qc_setup.h
	 */
	void HIDE_Display_InsertDrawFun(display_draw_fp fp_draw)
	{
		if(fp_draw!=0)
			LinkFun_Insert(&p_displayLinkFunHandle,fp_draw);	//  link into draw list
	}

	/**
	 * \brief	initializes the peripheral for the display
	 * \note	to enable this HIDE function define setup_DISPLAY in qc_setup.h
	 */
	void HIDE_Display_Init(void)
	{
		// Init display stuff
		u8g_InitComFn(&gs_display, &periph_DISPLAY_DRIVER, u8g_com_hw_spi_fn);
	}

	static void set_gpio_level(uint16_t pin, uint8_t level)
	{
		if( PIN_DC == pin )
			if(level==0)
			{
				GPIOPinWrite(DISPLAY_DC_PORT_BASE, DISPLAY_DC_PIN, 0);
			}
			else
			{
				GPIOPinWrite(DISPLAY_DC_PORT_BASE, DISPLAY_DC_PIN, DISPLAY_DC_PIN);
			}
		else if( PIN_CS == pin )
		{
			#if( periph_DISPLAY_PIN_FSS != periph_NONE )
				if(level==0)
					GPIOPinWrite(DISPLAY_FSS_PORT_BASE, DISPLAY_FSS_PIN, 0);
				else
				{
					GPIOPinWrite(DISPLAY_FSS_PORT_BASE, DISPLAY_FSS_PIN, DISPLAY_FSS_PIN);
				}
			#endif
		}
		else if(PIN_RST == pin )
			if(level==0)
				GPIOPinWrite(DISPLAY_RST_PORT_BASE, DISPLAY_RST_PIN, 0);
			else
				GPIOPinWrite(DISPLAY_RST_PORT_BASE, DISPLAY_RST_PIN, DISPLAY_RST_PIN);
	}

	// ns is not used at the moment! (SPI speed)
	static void spi_init(uint32_t ns)
	{
		BusyDelay_Init();

		// Pin RST
		ROM_SysCtlPeripheralEnable( DISPLAY_RST_SYSCTL_PERIPH);
		ROM_GPIOPinWrite(DISPLAY_RST_PORT_BASE, DISPLAY_RST_PIN, DISPLAY_RST_PIN);
		ROM_GPIOPinTypeGPIOOutput(DISPLAY_RST_PORT_BASE, DISPLAY_RST_PIN);

		// Pin DC
		ROM_SysCtlPeripheralEnable( DISPLAY_DC_SYSCTL_PERIPH);
		#if(periph_DISPLAY_PIN_DC == (GPIO_PORTD_BASE | GPIO_PIN_7) )
			// Make the Data/Command select, Reset, and SSI CS pins be outputs.
			// The nDC_OLED pin is PD7 an is a special GPIO (it is an NMI pin)
			// Therefore, we must unlock it first:
			// 1. Write 0x4C4F434B to GPIOLOCK register to unlock the GPIO Commit register
			// 2. Write to appropriate bit in the Commit Register (bit 7)
			// 3. Re-lock the GPIOLOCK register
			HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0x4C4F434B;	// unlock
			HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 1 << 7; 		// allow writes
			HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0x0;			// re-lock
		#endif
		ROM_GPIOPinWrite(DISPLAY_DC_PORT_BASE, DISPLAY_DC_PIN, DISPLAY_DC_PIN);
		ROM_GPIOPinTypeGPIOOutput(DISPLAY_DC_PORT_BASE, DISPLAY_DC_PIN);
		ROM_GPIOPinWrite(DISPLAY_DC_PORT_BASE, DISPLAY_DC_PIN, DISPLAY_DC_PIN);

		// Pin FSS
		#if( periph_DISPLAY_PIN_FSS != periph_NONE )
			ROM_SysCtlPeripheralEnable( DISPLAY_FSS_SYSCTL_PERIPH);
			ROM_GPIOPinWrite(DISPLAY_FSS_PORT_BASE, DISPLAY_FSS_PIN, DISPLAY_FSS_PIN);
			ROM_GPIOPinTypeGPIOOutput(DISPLAY_FSS_PORT_BASE, DISPLAY_FSS_PIN);
		#endif

		// Pin PWR_SCREEN
		#if( periph_DISPLAY_PIN_PWR_SCREEN != periph_NONE )
			ROM_SysCtlPeripheralEnable( DISPLAY_PWR_SCREEN_SYSCTL_PERIPH);
			ROM_GPIOPinWrite(DISPLAY_PWR_SCREEN_PORT_BASE, DISPLAY_PWR_SCREEN_PIN, DISPLAY_PWR_SCREEN_PIN);
			ROM_GPIOPinTypeGPIOOutput(DISPLAY_PWR_SCREEN_PORT_BASE, DISPLAY_PWR_SCREEN_PIN);	//VDD power control (1=off)
		#endif

		// Pin PWR_LOGIC
		#if( periph_DISPLAY_PIN_PWR_LOGIC != periph_NONE )
			ROM_SysCtlPeripheralEnable( DISPLAY_PWR_LOGIC_SYSCTL_PERIPH);
			ROM_GPIOPinWrite(DISPLAY_PWR_LOGIC_PORT_BASE, DISPLAY_PWR_LOGIC_PIN, DISPLAY_PWR_LOGIC_PIN);
			ROM_GPIOPinTypeGPIOOutput(DISPLAY_PWR_LOGIC_PORT_BASE, DISPLAY_PWR_LOGIC_PIN);		//VBAT power control (1=off)
			ROM_GPIOPinWrite(DISPLAY_PWR_LOGIC_PORT_BASE, DISPLAY_PWR_LOGIC_PIN, 0);
		#endif

		// Initialize SSI
		ROM_SysCtlPeripheralEnable( DISPLAY_SSI_SYSCTL_PERIPH);
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
		ROM_GPIOPinTypeSSI(DISPLAY_SSI_PORT_BASE, DISPLAY_SSI_CLK_PIN);
		ROM_GPIOPinTypeSSI(DISPLAY_SSI_PORT_BASE, DISPLAY_SSI_TX_PIN);
		ROM_GPIOPinConfigure(DISPLAY_SSI_TX_PIN_CONFIG);
		ROM_GPIOPinConfigure(DISPLAY_SSI_CLK_PIN_CONFIG);
		ROM_SSIClockSourceSet(periph_DISPLAY_SSI_BASE, SSI_CLOCK_SYSTEM);
		ROM_SSIConfigSetExpClk(periph_DISPLAY_SSI_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 30000000, 8);
		ROM_SSIEnable(periph_DISPLAY_SSI_BASE);
	}

	static void spi_out(uint8_t data)
	{
		// Wait for transmitter to be ready (FIFO empty)
		while (SSIBusy(SSI3_BASE));

		// Write the next transmit byte.
		SSIDataPut(SSI3_BASE, (unsigned long)data);
	}

	uint8_t u8g_com_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
	{
	  switch(msg)
	  {
		case U8G_COM_MSG_STOP:
		  break;

		case U8G_COM_MSG_INIT:

		   if ( arg_val <= U8G_SPI_CLK_CYCLE_50NS )
		  {
		spi_init(50);
		  }
		  else if ( arg_val <= U8G_SPI_CLK_CYCLE_300NS )
		  {
		spi_init(300);
		  }
		  else if ( arg_val <= U8G_SPI_CLK_CYCLE_400NS )
		  {
		spi_init(400);
		  }
		  else
		  {
		spi_init(1200);
		  }

		  u8g_MicroDelay();
		  break;

		case U8G_COM_MSG_ADDRESS:                     // define cmd (arg_val = 0) or data mode (arg_val = 1)
		  u8g_10MicroDelay();
		  set_gpio_level(PIN_DC, arg_val);
		  u8g_10MicroDelay();
		 break;

		case U8G_COM_MSG_CHIP_SELECT:
		  if ( arg_val == 0 )
		  {
			// disable
		uint8_t i;
		// this delay is required to avoid that the display is switched off too early --> DOGS102 with LPC1114
		for( i = 0; i < 5; i++ )
		  u8g_10MicroDelay();
		set_gpio_level(PIN_CS, 1);
		  }
		  else
		  {
			/* enable */
		set_gpio_level(PIN_CS, 0);
		  }
		  u8g_MicroDelay();
		  break;

		case U8G_COM_MSG_RESET:
		  set_gpio_level(PIN_RST, arg_val);
		  u8g_10MicroDelay();
		  break;

		case U8G_COM_MSG_WRITE_BYTE:
		  spi_out(arg_val);
		  u8g_MicroDelay();
		  break;

		case U8G_COM_MSG_WRITE_SEQ:
		case U8G_COM_MSG_WRITE_SEQ_P:
		  {
			register uint8_t *ptr = arg_ptr;
			while( arg_val > 0 )
			{
			  spi_out(*ptr++);
			  arg_val--;
			}
		  }
		  break;
	  }
	  return 1;
	}

/*
	// The use of DMA for the Display is not really needed
	// the overhead for DMA and EventBits are equal to stupid waiting
	static void InitDMA()
	{
		// all dma channels must be unlocked first
		Dma_ChannelUnlock(dma_CH15_EB);
	   	HIDE_Receive_SetEventName(dma_CH15_EB,"D15");	// set name for the eventBit

	    // Enable SSI DMA_TX
	    ROM_SSIDMAEnable(DISPLAY_SSI_BASE,SSI_DMA_TX);

	    //
	    // Enable the UART peripheral interrupts.  Note that no UART interrupts
	    // were enabled, but the uDMA controller will cause an interrupt on the
	    // UART interrupt signal when a uDMA transfer is complete.
	    //
	    ROM_IntEnable(INT_SSI3);

	}

	// The use of DMA for the Display is not really needed
	// the overhead for DMA and EventBits are equal to stupid waiting
	static void StartDMA(void *pvSrcAddr,uint32_t ui32TransferSize)
	{
	    // Map DMA Channel to SSI
	    ROM_uDMAChannelAssign(UDMA_CH15_SSI3TX);

	    // Put the attributes in a known state for the uDMA software channel.
	    // These should already be disabled by default.
	    ROM_uDMAChannelAttributeDisable(dma_CH15,
	                                    UDMA_ATTR_USEBURST | 		// is used to restrict transfers to use only burst mode.
										UDMA_ATTR_ALTSELECT |		// is used to select the alternate control structure for this channel.
	                                    (UDMA_ATTR_HIGH_PRIORITY |	// is used to set this channel to high priority.
	                                    UDMA_ATTR_REQMASK));		// used to mask the hardware request signal from the peripheral for this channel.

	    ROM_uDMAChannelAttributeEnable(	dma_CH15,
                						UDMA_ATTR_USEBURST); 		// is used to restrict transfers to use only burst mode.

	    // Configure the control parameters for the Channel
	    ROM_uDMAChannelControlSet(	dma_CH15 | 						// Channel for SSI
	    							UDMA_PRI_SELECT,				// because of simple Transfers only PRI_SELECT is used (no Ping-Pong,...)
									UDMA_SIZE_8 | 					// 8 Bits at one transfer
									UDMA_SRC_INC_8 | 				// adress increment source
									UDMA_DST_INC_NONE |				// adress increment destination
									UDMA_ARB_4);					// rearbitrates for the bus after 4 items

	    // Set up the transfer parameters for the channel
	    ROM_uDMAChannelTransferSet(	dma_CH15 | 						// Channel for SSI
	    							UDMA_PRI_SELECT,				// simple Transfer
									UDMA_MODE_BASIC,				// Because peripheral performs requests
									pvSrcAddr, 						// Pointer to source
									(void *)(SSI3_BASE+SSI_O_DR),	// Pointer to destination
									ui32TransferSize);				// Count of items to transfer

	    // Now the channel is primed to start a transfer.
	    ROM_uDMAChannelEnable(dma_CH15);
	}

	// The use of DMA for the Display is not really needed
	// the overhead for DMA and EventBits are equal to stupid waiting
	void Display_SsiIntHandler(void)
	{
		ROM_SSIIntClear(SSI3_BASE,SSI_DMATX);
		Dma_ChannelUnlockFromISR(dma_CH15_EB);
	}
*/
#elif ( setup_DISPLAY_I2C == (setup_DISPLAY&setup_MASK_OPT1) )
	#error ERROR: not implemented. port it here ( like SPI was ported above ) or select SPI
#elif ( setup_DISPLAY_NONE == (setup_DISPLAY&setup_MASK_OPT1) )

#else
	#error ERROR: define setup_DISPLAY (in qc_setup.h)
#endif
