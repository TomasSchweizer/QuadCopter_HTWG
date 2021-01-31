/**
 * 		@file 	peripheral_setup.h
 * 		@brief	Setup the peripherals (GPIOs, I2Cs, Timers, ISRs, ... )
 *  	@note	Note, that the desired driver and INT vector table
 *  			has to support this setup.
 *//*	@author Tobias Grimm
 * 		@date 	01.06.2016	(last modified)
 */

#ifndef __PERIPHERAL_SETUP_H__
#define	__PERIPHERAL_SETUP_H__

/* ------------------------------------------------------------ */
/*					Include File Definitions					*/
/* ------------------------------------------------------------ */

//  Hardware Specific
#include "inc/hw_ints.h"

// setup
#include "qc_setup.h"

/* ------------------------------------------------------------ */
/*				   	Defines			    						*/
/* ------------------------------------------------------------ */

// extract Port and Pin from a Macro
#define periph_MASK_PORT				 	0xFFFFFF00
#define periph_MASK_PIN					 	0x000000FF
#define periph_SPLIT_PORT_PIN(debug_PIN) 	debug_PIN&periph_MASK_PORT,debug_PIN&periph_MASK_PIN
#define periph_NONE			0xFFFFFFFF


//
// Watchdog peripherals
//
#if ( setup_WATCHDOG == (setup_WATCHDOG_ACTIVE&setup_MASK_OPT1) )

    #define periph_WATCHDOG_INT                 INT_WATCHDOG // interrupt for wachdog

#endif

//
//  Motor Peripherals
//
#if ( setup_MOTOR_I2C == (setup_MOTOR&setup_MASK_OPT1) )

	#define periph_MOTOR_INT 					INT_I2C1		// motor driver can share I2C with sensor

#endif

//
//  Sensor Peripherals
//
#if   ( setup_SENSOR_I2C == (setup_SENSOR&setup_MASK_OPT1) )

	#define periph_SENSOR_INT 					INT_I2C1 		// motor driver can share I2C with sensor

    #if ( setup_ALT_BARO || setup_ALT_LIDAR )

        #define periph_SENSOR_ALT_INT           INT_I2C2
    #endif

#endif

//
//  Remote Control Peripherals
//
#if( setup_REMODE_CPPM == (setup_REMOTE&setup_MASK_OPT1) )

	#define periph_REMOTE_INT 					INT_GPIOE
	#define periph_REMOTE_PIN_CPPM 				(GPIO_PORTE_BASE| GPIO_PIN_2)

	#define periph_REMOTE_TIMER_BASE			TIMER2_BASE			// free down running Timer 16 Bit 1us ticks
	#define periph_REMOTE_TIMER_MODULE 			TIMER_A

#endif

//
//  Display Peripherals
//
#if( setup_DISPLAY_SPI == (setup_DISPLAY&setup_MASK_OPT1) )

	#define periph_DISPLAY_SSI_BASE				SSI3_BASE
																				// Defines for Orbit BoosterPack Display:
	#define periph_DISPLAY_PIN_FSS				(periph_NONE)					//(GPIO_PORTD_BASE | GPIO_PIN_1)		// frame signal (chip select)
	#define periph_DISPLAY_PIN_RST				(GPIO_PORTD_BASE | GPIO_PIN_2)	//(GPIO_PORTE_BASE | GPIO_PIN_5)		// Reset (low aktive)
	#define periph_DISPLAY_PIN_DC				(GPIO_PORTD_BASE | GPIO_PIN_1)	//(GPIO_PORTD_BASE | GPIO_PIN_7)		// display or command select (high for display bufferaccess, low for command access)
	#define periph_DISPLAY_PIN_PWR_SCREEN		(periph_NONE)					//(GPIO_PORTF_BASE | GPIO_PIN_1)		// turn on/off power to the OLED display itself (low aktive)
	#define periph_DISPLAY_PIN_PWR_LOGIC		(periph_NONE)					//(GPIO_PORTE_BASE | GPIO_PIN_1)		// turn on/off the power to the logic of the display (low aktive)

	#define periph_DISPLAY_DRIVER				u8g_dev_ssd1306_128x64_hw_spi	//u8g_dev_ssd1306_128x32_hw_spi			// see 05_third_party/display_u8glib/src_options for more drivers
#endif

//
//  Debug Peripherals
//
#if( setup_DEBUG_UART == (setup_DEBUG&setup_MASK_OPT1) )

	#define periph_DEBUG_UART_BASE 				UART0_BASE

#endif

//
// New implementation USB Debug Peripherals TODO check if correct edit comment
#if( setup_DEBUG_USB == (setup_DEBUG&setup_MASK_OPT1) )

    #define periph_DEBUG_USB_BASE              USB0_BASE

#endif


#if( setup_DEV_DEBUG_PINS )

	#define periph_DEBUG_PIN1 					(GPIO_PORTA_BASE | GPIO_PIN_2)
	#define periph_DEBUG_PIN2 					(periph_NONE)

#endif

//
//  Workload Peripherals
//
#if( setup_DEV_WORKLOAD_LED )

	#define periph_WORKLOAD_ALIVE_LED 			(GPIO_PORTF_BASE | GPIO_PIN_3)

#endif
#if( setup_DEV_WORKLOAD_LED || setup_DEV_WORKLOAD_CALC )

	#define periph_WORKLOAD_TIMER_BASE			TIMER2_BASE			   // free down running Timer 16 Bit 1us ticks
	#define periph_WORKLOAD_TIMER_MODULE 		TIMER_A

    #define periph_WORKLOAD_TIMER_INT       	INT_TIMER2A

#endif

#endif // __PERIPHERAL_SETUP_H__
