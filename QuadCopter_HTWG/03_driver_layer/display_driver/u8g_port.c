/*===================================================================================================*/
/*  u8g_port.c                                                                                       */
/*===================================================================================================*/

/**
*   file   u8g_port.c
*
*   @brief  Port to use the ui8glib for display
*
*   @details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>15/05/2016      <td>Tobias Grimm        <td>Implementation & last modification MAs
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*   -
*/
/*====================================================================================================*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

// Standard libraries
#include <stdint.h>
#include <stdbool.h>

// Setup
#include "qc_setup.h"
#include "peripheral_setup.h"


// Drivers
#include "display_driver.h"
#include "u8g_port.h"

// Utilities
#include "busy_delay.h"
#include "link_functions.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Defines                                                 */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Type Definitions                                        */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Forward Declarations                                          */
/* ---------------------------------------------------------------------------------------------------*/
void u8g_Delay(uint16_t val);
void u8g_MicroDelay(void);
void u8g_10MicroDelay(void);

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/


u8g_t gs_display;           ///<  Info and storage for the display driver

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Variables                                               */
/* ---------------------------------------------------------------------------------------------------*/

linkFun_handle_p p_displayLinkFunHandle = 0; ///< Handle for the linked function list for the display

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Procedure Definitions                                         */
/* ---------------------------------------------------------------------------------------------------*/

/**
 * @brief   Delay in milli seconds
 */
void u8g_Delay(uint16_t val)
{
	BusyDelay_Ms(val);
}

/**
 * @brief   Delay one micro second
 */
void u8g_MicroDelay(void)
{
	BusyDelay_Us(1);
}
/**
 * @brief   Delay 10 micro seconds
 */
void u8g_10MicroDelay(void)
{
	BusyDelay_Us(10);
}

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Select Mode                                                   */
/* ---------------------------------------------------------------------------------------------------*/

#if   ( setup_DISPLAY_SPI == (setup_DISPLAY&setup_MASK_OPT1) ) || DOXYGEN

    /* -----------------------------------------------------------------------------------------------*/
    /*                                     Include File Definitions SPI Mode                          */
    /* -----------------------------------------------------------------------------------------------*/

	//  Hardware specific libraries
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

    /*------------------------------------------------------------------------------------------------*/
    /*                                     Local defines SPI Mode                                     */
    /* -----------------------------------------------------------------------------------------------*/

	#define 	PIN_DC 			( 1 )	///< Display or command select
	#define 	PIN_CS 			( 2 )	///< Chip Select
	#define 	PIN_RST 		( 3 )	///< Reset

	///< Define the desired peripheral setup (see peripheral_setup.h)
	#if( periph_DISPLAY_SSI_BASE == SSI3_BASE)
		#define DISPLAY_SSI_SYSCTL_PERIPH					SYSCTL_PERIPH_GPIOD
		#define DISPLAY_SSI_PORT_BASE						GPIO_PORTD_BASE
		#define DISPLAY_SSI_CLK_PIN							GPIO_PIN_0			///< Clock
		#define DISPLAY_SSI_CLK_PIN_CONFIG					GPIO_PD0_SSI3CLK
		#define DISPLAY_SSI_TX_PIN							GPIO_PIN_3			///< Transmit
		#define DISPLAY_SSI_TX_PIN_CONFIG					GPIO_PD3_SSI3TX
	#else
		#error	ERROR implement the defines above here
	#endif

	#define DISPLAY_FSS_PIN									(periph_DISPLAY_PIN_FSS & periph_MASK_PIN)		///< Frame signal (chip select)
	#define DISPLAY_FSS_PORT_BASE							(periph_DISPLAY_PIN_FSS & periph_MASK_PORT)
	#if( DISPLAY_FSS_PORT_BASE == GPIO_PORTD_BASE)
		#define DISPLAY_FSS_SYSCTL_PERIPH					SYSCTL_PERIPH_GPIOD
	#elif( periph_DISPLAY_PIN_FSS == periph_NONE )

	#else
		#error	ERROR implement the defines above here
	#endif

	#define DISPLAY_RST_PIN									(periph_DISPLAY_PIN_RST & periph_MASK_PIN)
	#define DISPLAY_RST_PORT_BASE							(periph_DISPLAY_PIN_RST & periph_MASK_PORT)			///< Reset (low active)
	#if ( DISPLAY_RST_PORT_BASE == GPIO_PORTE_BASE )
		#define DISPLAY_RST_SYSCTL_PERIPH					SYSCTL_PERIPH_GPIOE
	#elif( DISPLAY_RST_PORT_BASE == GPIO_PORTD_BASE )
		#define DISPLAY_RST_SYSCTL_PERIPH					SYSCTL_PERIPH_GPIOD
	#else
		#error	ERROR implement the defines above here
	#endif

	#define DISPLAY_DC_PIN									(periph_DISPLAY_PIN_DC & periph_MASK_PIN)			///< Display or command select (high for display buffer access, low for command access)
	#define DISPLAY_DC_PORT_BASE							(periph_DISPLAY_PIN_DC & periph_MASK_PORT)
	#if ( DISPLAY_DC_PORT_BASE == GPIO_PORTD_BASE )
		#define DISPLAY_DC_SYSCTL_PERIPH					SYSCTL_PERIPH_GPIOD
	#else
		#error	ERROR implement the defines above here
	#endif


	#define DISPLAY_PWR_SCREEN_PIN							(periph_DISPLAY_PIN_PWR_SCREEN & periph_MASK_PIN)			///< turn on/off power to the OLED display itself (low active)
	#define DISPLAY_PWR_SCREEN_PORT_BASE					(periph_DISPLAY_PIN_PWR_SCREEN & periph_MASK_PORT)
	#if ( DISPLAY_PWR_SCREEN_PORT_BASE == GPIO_PORTF_BASE )
		#define DISPLAY_PWR_SCREEN_SYSCTL_PERIPH			SYSCTL_PERIPH_GPIOF
	#elif( periph_DISPLAY_PIN_PWR_SCREEN == periph_NONE )

	#else
		#error	ERROR implement the defines above here
	#endif

	#define DISPLAY_PWR_LOGIC_PIN							(periph_DISPLAY_PIN_PWR_LOGIC & periph_MASK_PIN)
	#define DISPLAY_PWR_LOGIC_PORT_BASE						(periph_DISPLAY_PIN_PWR_LOGIC & periph_MASK_PORT)			///< turn on/off the power to the logic of the display (low active)
	#if ( DISPLAY_PWR_LOGIC_PORT_BASE == GPIO_PORTE_BASE )
		#define DISPLAY_PWR_LOGIC_SYSCTL_PERIPH				SYSCTL_PERIPH_GPIOE
	#elif( periph_DISPLAY_PIN_PWR_LOGIC == periph_NONE )

	#else
		#error	ERROR implement the defines above here
	#endif

    /*------------------------------------------------------------------------------------------------*/
    /*                                     Local Type Definitions SPI Mode                            */
    /* -----------------------------------------------------------------------------------------------*/

    /* -----------------------------------------------------------------------------------------------*/
    /*                                      Forward Declarations SPI Mode                             */
    /* -----------------------------------------------------------------------------------------------*/

	void Display_Update(display_draw_fp fp_draw);
	static void set_gpio_level(uint16_t pin, uint8_t level);
	static void spi_init(uint32_t ns);
	static void spi_out(uint8_t data);
	uint8_t u8g_com_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);

    /* -----------------------------------------------------------------------------------------------*/
    /*                                      Local Variables SPI Mode                                  */
    /* -----------------------------------------------------------------------------------------------*/

    /* -----------------------------------------------------------------------------------------------*/
    /*                                      Procedure Definitions SPI mode                            */
    /* -----------------------------------------------------------------------------------------------*/

	/**
	 * @brief	Redraw all inserted functions on the display.
	 *
	 * @details	Add a function with HIDE_Display_InsertDrawFun
	 *			into the display draw functionLinkedList (do this only once)
	 *
	 * @note	To enable this HIDE function define setup_DISPLAY in qc_setup.h
	 */
	void HIDE_Display_Redraw(void)
	{
		// Picture loop
		u8g_FirstPage(&gs_display);
		do
		{
			LinkFun_RunAllFun(p_displayLinkFunHandle);
		} while ( u8g_NextPage(&gs_display) );
	}

	/**
	 * @brief	Insert a function into the display draw function linked list.
	 *
	 * @details This should be performed before scheduler starts, e.g. in the init of a driver or task.
	 *			null pointers won't be inserted.
	 *
	 * @param	fp_draw	The function with the draw commands
	 *
	 * @note	To enable this HIDE function define setup_DISPLAY in qc_setup.h
	 */
	void HIDE_Display_InsertDrawFun(display_draw_fp fp_draw)
	{
		if(fp_draw!=0)
			LinkFun_Insert(&p_displayLinkFunHandle,fp_draw);	//  link into draw list
	}

	/**
	 * @brief	Initializes the peripheral for the display
	 *
	 * @note	To enable this HIDE function define setup_DISPLAY in qc_setup.h
	 */
	void HIDE_Display_Init(void)
	{
		// Init display stuff
		u8g_InitComFn(&gs_display, &periph_DISPLAY_DRIVER, u8g_com_hw_spi_fn);
	}

	/**
     * @brief   Set GPIO level for display pins
     */
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


	/**
     * @brief   Initialize SPI
     */
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

	/**
     * @brief   Send a byte over SPI
     *
     * @param data uint8_t value to transmit
     */
	static void spi_out(uint8_t data)
	{
		// Wait for transmitter to be ready (FIFO empty)
		while (SSIBusy(SSI3_BASE));

		// Write the next transmit byte.
		SSIDataPut(SSI3_BASE, (unsigned long)data);
	}

	/*
	 * @brief u8g spi handler
	 */
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

#elif ( setup_DISPLAY_I2C == (setup_DISPLAY&setup_MASK_OPT1) )
	#error ERROR: not implemented. port it here ( like SPI was ported above ) or select SPI
#elif ( setup_DISPLAY_NONE == (setup_DISPLAY&setup_MASK_OPT1) )

#else
	#error ERROR: define setup_DISPLAY (in qc_setup.h)
#endif

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
