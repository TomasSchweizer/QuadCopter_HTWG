/*===================================================================================================*/
/*  peripheral_setup.h                                                                               */
/*===================================================================================================*/

/**
*   @file   peripheral_setup.h
*
*   @brief  Setup the peripherals (GPIOs, I2Cs, Timers, ISRs, ... )
*
*   @details
*   The desired driver and INT vector table has to support this setup.\n
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>21/03/2016      <td>Tobias Grimm        <td>Implementation & Last modification of MAs
*   <tr><td>18/11/2020      <td>Tomas Schweizer     <td>Added USB and I2C2 for extra sensors
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*   -
*/
/*====================================================================================================*/

#ifndef __PERIPHERAL_SETUP_H__
#define	__PERIPHERAL_SETUP_H__

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/


// Hardware specific libraries
#include "inc/hw_ints.h"

// Setup
#include "qc_setup.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/

// extract Port and Pin from a Macro
#define periph_MASK_PORT				 	( 0xFFFFFF00 )                  ///< Mask to get the port of a TivaWare macro
#define periph_MASK_PIN					 	( 0x000000FF )                 ///< Mask to get the pin of a TivaWare macro
#define periph_SPLIT_PORT_PIN(debug_PIN) 	( debug_PIN&periph_MASK_PORT,debug_PIN&periph_MASK_PIN ) ///< get Port and Pin split of TivaWare macro
#define periph_NONE			                ( 0xFFFFFFFF )                  ///< No peripheral is activated


// Watchdog peripherals
#if ( setup_WATCHDOG == (setup_WATCHDOG_ACTIVE&setup_MASK_OPT1) )

    #define periph_WATCHDOG_INT                 ( INT_WATCHDOG )            ///< macro for wachtdog interrupt

#endif

// Motor Peripherals
#if ( setup_MOTOR_I2C == (setup_MOTOR&setup_MASK_OPT1) )

	#define periph_MOTOR_INT 					( INT_I2C1 )		        ///< I2C1 interrupt for motors (shared I2C with mpu)

#endif


// Sensor Peripherals
#if   ( setup_SENSOR_I2C == (setup_SENSOR&setup_MASK_OPT1) )

	#define periph_SENSOR_INT 					( INT_I2C1 ) 		        ///< I2C1 interrupt for mpu, (share I2C with motors)

    #if ( setup_ALT_BARO || setup_ALT_LIDAR )

        #define periph_SENSOR_ALT_INT           ( INT_I2C2 )                ///< I2C2 interrupt for Lidar and Barometer
    #endif

#endif


// Remote Control Peripherals
#if( setup_REMODE_CPPM == (setup_REMOTE&setup_MASK_OPT1) )

	#define periph_REMOTE_INT 					( INT_GPIOE )               ///< Pin interrupt for port E
	#define periph_REMOTE_PIN_CPPM 				( GPIO_PORTE_BASE| GPIO_PIN_2 )   ///< Pin E2 is pin for receiver communication

	#define periph_REMOTE_TIMER_BASE			( TIMER2_BASE )			    ///< Free down running Timer 16 Bit 1us ticks
	#define periph_REMOTE_TIMER_MODULE 			( TIMER_A )                 ///< TimerA module

#endif


// Display Peripherals
#if( setup_DISPLAY_SPI == (setup_DISPLAY&setup_MASK_OPT1) )

	#define periph_DISPLAY_SSI_BASE				( SSI3_BASE )

	#define periph_DISPLAY_PIN_FSS				( periph_NONE )		    ///< Frame signal (chip select)
	#define periph_DISPLAY_PIN_RST				( GPIO_PORTD_BASE | GPIO_PIN_2 )  ///< Reset (low active)
	#define periph_DISPLAY_PIN_DC				( GPIO_PORTD_BASE | GPIO_PIN_1 )	///< Display or command select (high for display buffer access, low for command access)
	#define periph_DISPLAY_PIN_PWR_SCREEN		( periph_NONE )		    ///< Turn on/off power to the OLED display itself (low active)
	#define periph_DISPLAY_PIN_PWR_LOGIC		( periph_NONE )		    ///< Turn on/off the power to the logic of the display (low active)

	#define periph_DISPLAY_DRIVER				( u8g_dev_ssd1306_128x64_hw_spi )   ///< see 05_third_party/display_u8glib/src_options for more drivers
#endif


// UART-Debug Peripherals
#if( setup_DEBUG_UART == (setup_DEBUG&setup_MASK_OPT1) )

	#define periph_DEBUG_UART_BASE 				( UART0_BASE )              ///< Macro for UART-Base for debugging

#endif

// USB-Debug Peripherals
#if( setup_DEBUG_USB == (setup_DEBUG&setup_MASK_OPT1) )

    #define periph_DEBUG_USB_BASE              ( USB0_BASE )                ///< Macro foe USB-Base for debugging / data transfer

#endif


#if( setup_DEV_DEBUG_PINS )

	#define periph_DEBUG_PIN1 					( GPIO_PORTA_BASE | GPIO_PIN_2 )  ///< Debug pin 1 is pin A2
	#define periph_DEBUG_PIN2 					( periph_NONE )

#endif

//
//  Workload Peripherals
//
#if( setup_DEV_WORKLOAD_LED )

	#define periph_WORKLOAD_ALIVE_LED 			( GPIO_PORTF_BASE | GPIO_PIN_3 )  ///< Macro for alive LED, blinks if the software is running

#endif
#if( setup_DEV_WORKLOAD_LED || setup_DEV_WORKLOAD_CALC )

	#define periph_WORKLOAD_TIMER_BASE			( TIMER2_BASE )			    ///< Free down running Timer 16 Bit 1us ticks, same as the receiver uses
	#define periph_WORKLOAD_TIMER_MODULE 		( TIMER_A )                 ///< TimerA module

    #define periph_WORKLOAD_TIMER_INT       	( INT_TIMER2A )             ///< Workload uses the interrupt of TimerA2 receiver only uses the count values

#endif

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/


#endif // __PERIPHERAL_SETUP_H__

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
