/*===================================================================================================*/
/*  remote_control.h                                                                                 */
/*===================================================================================================*/

/*
*   file   remote_control.h
*
*   brief  Implementation of functions to receive and interpret values from the remote control
*
*   details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>12/03/2016      <td>Tobias Walter       <td>Implementation & last modifications through MAs
*   <tr><td>12/06/2021      <td>Tomas Schweizer     <td>Added features (QC axes, throttle rate slope)
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*/
/*====================================================================================================*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

// Standard libraries
#include <stdint.h>
#include <stdbool.h>

// Hardware specific libraries
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "peripheral_setup.h"

// Setup
#include "qc_setup.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "event_groups.h"


// Application
#include "receiver_task.h"

// Drivers
#include "remote_control.h"
#include "debug_interface.h"

// utils
#include "qc_math.h"
#include "fault.h"




/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Defines                                                 */
/* ---------------------------------------------------------------------------------------------------*/

// Maximum values remote control
#define ROLL_MAX		    (15.0f*math_PI/180.0f)		    ///< Roll max angle commands [-max...+max] [rad] (At the moment 15° ~ 0.262 rad)
#define PITCH_MAX		    (15.0f*math_PI/180.0f)		    ///< Pitch max angle commands[-max...+max] [rad] (At the moment 15° ~ 0.262 rad)
#define YAW_MAX			    (15.0f*math_PI/180.0f)		    ///< Yaw max rate commands [-max...+max] [rad/s] (At the moment 25°/s ~ 0.436 rad/s)
#define THROTTLE_MAX	    (1.0f)							///< Throttle max command [-max...+max] [] (At the moment -1.0 -> 1.0)
#define THROTTLE_RC_EXPO    (0.4f)                          ///< Value to make the throttle around 0.0 (hover point) less sensitive

/// Number of Channles in the CPPM signal
#define remote_CPPM_CHANNELS 		  ( 6 )

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Type Definitions                                        */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Forward Declarations                                          */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

extern volatile EventGroupHandle_t gx_receiver_eventGroup;      // Extern global variable from receiver task for receiver events

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

const float f_maxValues[] = {ROLL_MAX, PITCH_MAX, YAW_MAX, THROTTLE_MAX};

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Variables                                               */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Select Mode                                                   */
/* ---------------------------------------------------------------------------------------------------*/

#if ( setup_REMODE_CPPM == (setup_REMOTE&setup_MASK_OPT1)) || DOXYGEN

    /* -----------------------------------------------------------------------------------------------*/
    /*                                     Include File Definitions CPPM Mode                         */
    /* -----------------------------------------------------------------------------------------------*/

    /*------------------------------------------------------------------------------------------------*/
    /*                                     Local defines CPPM Mode                                    */
    /* -----------------------------------------------------------------------------------------------*/

	// Time Constants for the CPPM Signal [µs]
	#define CPPM_SYNC_TIME_MIN	  		( 8000 )            ///< mimimum frame sync time
	#define CPPM_SYNC_TIME_MAX	  		( 18000 )           ///< maximum frame sync time


    #define CPPM_MIN_MAX_RANGE_THROTTEL ( 900.0f )          ///< Average min -> max range of the throttle channel [µs]

    #define CPPM_MIN_MAX_RANGE			( 780.0f )          ///< Average min -> max range of the roll, pitch, yaw channels
	#define CPPM_MIN_MAX_RANGE_HALF		(CPPM_MIN_MAX_RANGE/2.0f)   ///< Half range for roll; pitch, yaw channels (because - to +)


	// possible position of the AUX stick from the remote control [µs]
	#define IS_AUX_POS1(aux)   			( aux > 600   && aux < 1200 )
	#define IS_AUX_POS2(aux)   			( aux > 1200  && aux < 1700 )
	#define IS_AUX_POS3(aux)   			( aux > 1700  && aux < 2000 )
	#define IS_AUX_LOST(aux)	  		(IS_AUX_POS2(aux) || IS_AUX_POS3(aux) )
	#define IS_AUX_CALIBRATE(aux)	 	IS_AUX_POS3(aux)
	#define AUX_LOST				 	1300

	// Order of the Channels in the CPPM Signal	(different for left and right hand)
	#if   ( setup_REMOTE_RIGHT_HAND == (setup_REMOTE&setup_MASK_OPT2) )
		#define CPPM_ORDER			{ remote_YAW , remote_PITCH , remote_THROTTLE , remote_ROLL , remote_AUX1 , remote_RESERVE}
	#elif ( setup_REMOTE_LEFT_HAND == (setup_REMOTE&setup_MASK_OPT2) )
		#define CPPM_ORDER			{ remote_YAW , remote_PITCH , remote_THROTTLE , remote_ROLL , remote_AUX1 , remote_RESERVE} // TODO not implemented anymore
	#else
		#error	ERROR define the hand in qc_setup.h
	#endif


	// define the desired peripheral setup (see peripheral_setup.h)
	#define REMOTE_PIN_CPPM 							(periph_REMOTE_PIN_CPPM & periph_MASK_PIN)
	#define REMOTE_GPIO_PORT_BASE						(periph_REMOTE_PIN_CPPM & periph_MASK_PORT)

    #if ( REMOTE_GPIO_PORT_BASE == GPIO_PORTE_BASE )
		#define REMOTE_SYSCTL_PERIPH_GPIO				SYSCTL_PERIPH_GPIOE
	#else
		#error	ERROR implement the defines above here
	#endif
	#if ( periph_REMOTE_TIMER_BASE == TIMER2_BASE )
		#define REMOTE_SYSCTL_PERIPH_TIMER				SYSCTL_PERIPH_TIMER2
	#else
		#error	ERROR implement the defines above here
	#endif

    /*------------------------------------------------------------------------------------------------*/
    /*                                     Local Type Definitions CPPM Mode                           */
    /* -----------------------------------------------------------------------------------------------*/

    /// States of the receiver state machine
	enum receiverState_e
	{
		STATE_SYNC,
		STATE_NORMAL
	};

    /* ------------------------------------------------------------------------------------------------*/
    /*                                      Forward Declarations CPPM Mode                             */
    /* ------------------------------------------------------------------------------------------------*/

	/// Interrupt handler for CPPM
	void RemoteControl_CppmIntHandler(void);

    /* ---------------------------------------------------------------------------------------------------*/
    /*                                      Global Variables CPPM Mode                                    */
    /* ---------------------------------------------------------------------------------------------------*/

    /* ---------------------------------------------------------------------------------------------------*/
    /*                                      Local Variables CPPM Mode                                     */
    /* ---------------------------------------------------------------------------------------------------*/

	static int32_t i32_receiverData[remote_CPPM_CHANNELS];      ///< Store received time frames of CPPM pulses
	static int32_t i32_offsetData[4];                           ///< Store offset of CPPM pulses

	static const uint16_t ui16_cppmMap[remote_CPPM_CHANNELS] = CPPM_ORDER;  ///< Map of the signals in the CPPM


    /* -----------------------------------------------------------------------------------------------*/
    /*                                      Procedure Definitions CPPM mode                           */
    /* -----------------------------------------------------------------------------------------------*/

	/**
	 * @brief	Interrupt handler for CPPM-Mode.
	 *
	 * @details
	 * State machine is used for synchronization. After a data frame is collected the eventBit receiver_REMOTE_DATA
	 * is set in gx_receiver_eventGroup.IF synchronization is lost -> ui32_receiverData[remote_AUX1]=AUX_LOST
	 *
	 * @return  void
	 */
	void RemoteControl_CppmIntHandler(void)
	{

		static enum receiverState_e e_receiverState = STATE_SYNC;

		// 16 Bit to use Inter Overflow at subtraction from (16Bit)time Values
		// -> no "bad jumps" when timer is overflowing
		static uint16_t		ui16_lastTime;
		static uint16_t		ui16_loopCounter=0;
		uint16_t			ui16_currentTime;
		uint16_t			ui16_diffTime;

		GPIOIntClear(REMOTE_GPIO_PORT_BASE, REMOTE_PIN_CPPM);

		ui16_currentTime = ROM_TimerValueGet(periph_REMOTE_TIMER_BASE, periph_REMOTE_TIMER_MODULE);
		ui16_diffTime    = ui16_lastTime-ui16_currentTime; // Because the counter counts down

		switch(e_receiverState)
		{
			case STATE_NORMAL:
			{
				// Not the end of frame?
				if (ui16_loopCounter<remote_CPPM_CHANNELS-1)
				{
					// Save the time value for the current Channel
					i32_receiverData[ui16_cppmMap[ui16_loopCounter] ] = ui16_diffTime;
					ui16_loopCounter++;
				}
				// Last element of the Frame
				else if (ui16_loopCounter == remote_CPPM_CHANNELS-1)
				{
					// Save the time value for the current Channel
					i32_receiverData[ ui16_cppmMap[ui16_loopCounter] ] = ui16_diffTime;
					ui16_loopCounter++;

					// Send a notification to ReceiverTask, that frame is ready
					// and can be read during sync time of the frame
					BaseType_t x_higherPriorityTaskWoken = pdFALSE;
					if(pdFAIL==xEventGroupSetBitsFromISR(gx_receiver_eventGroup,receiver_REMOTE_DATA, &x_higherPriorityTaskWoken))
					{
						// this should never happen!
						// you will come here, when configTIMER_QUEUE_LENGTH is full
						// This often happens, when you run into a while(1) loop and ISRs still try to insert!
						while(1);
					}
					else
					{
						portYIELD_FROM_ISR( x_higherPriorityTaskWoken );
					}
				}
				// End of frame (sync time)
				else
				{
					// End of frame time value should be sync time
					if( ui16_diffTime > CPPM_SYNC_TIME_MIN  && ui16_diffTime < CPPM_SYNC_TIME_MAX )
					{
						// All is good, start new frame
						ui16_loopCounter=0;
					}
					else
					{
						// Something bad happened, synchronization lost -> try to sync again
						e_receiverState=STATE_SYNC;

						// Store in aux the information, that sync is lost
						i32_receiverData[remote_AUX1]=AUX_LOST;
					}
				}

				break;
			}
			case STATE_SYNC:
			{
				/// Wait for the sync time in the cppm signal
				if( ui16_diffTime > CPPM_SYNC_TIME_MIN  && ui16_diffTime < CPPM_SYNC_TIME_MAX )
				{
					ui16_loopCounter=0;
					e_receiverState=STATE_NORMAL;
				}
				break;
			}
		}
		ui16_lastTime=ui16_currentTime;
	}

	/**
	 * @brief	Interpret CPPM data and copy it into f_receiverSetPoint
	 *
	 * @details
	 * To interpret the data first the offset is subtracted from the received data.
	 * For roll, pitch and yaw the signal is just scaled and limited to the min/max values.
	 * For throttle the signal is scaled and limited to min/max values and then send through
	 * a cubic function to get a smoother flight feeling around the hover point.
	 * Fire eventBit fault_REMOTE_CONTROL if there is a fault, else clear the bit.
	 * If the remote control requests a calibration, return true.
	 *
	 * @param	f_receiverSetPoint --> Array to copy the setpoints into in (see remote_control.h for the order)
	 *
	 * @return	true --> If Calibrate is required\n
	 *          false --> else
	 *
	 * @note    Events: EventBit fault_REMOTE_CONTROL will be set or cleared in gx_fault_EventGroup
	 */
	uint8_t  RemoteControl_GetData(volatile float f_receiverSetPoint[])
	{

		//	Interpret Data
	    int16_t i;
		float help, help_3;
		for( i=0 ; i<4 ; ++i )
		{
			help=(float) (i32_receiverData[i]-i32_offsetData[i]);		// subtract offset
			if(i==remote_THROTTLE)
			{
				help=help/CPPM_MIN_MAX_RANGE_THROTTEL*f_maxValues[i];		// scale to [0...1]
				help = (2.0 * help) - 1;                                    // move it to [-1...1]
				help=math_LIMIT(help, -f_maxValues[i], f_maxValues[i]);	    // limit to [-1...1]
				help_3 = help*help*help;                                    // help variable x^3
				help= (1.0 - THROTTLE_RC_EXPO) * help + help_3 * THROTTLE_RC_EXPO; // Cubic function applied to throttle
			}
			else
			{
				help=help/CPPM_MIN_MAX_RANGE_HALF*f_maxValues[i];		// scale to [-max...max]
				help=math_LIMIT(help, -f_maxValues[i], f_maxValues[i]); // limit to [-max...max]
			}
			f_receiverSetPoint[i]=help;
		}

		// Fit remote control input to quadcopter rotational axes
		if( setup_REMOTE_FIT_TO_QC == (setup_REMOTE&setup_MASK_OPT3) ){

		f_receiverSetPoint[remote_YAW]  =     -f_receiverSetPoint[remote_YAW];	 // invert YAW
		f_receiverSetPoint[remote_PITCH]=   -f_receiverSetPoint[remote_PITCH];   // invert PITCH
		f_receiverSetPoint[remote_ROLL] =    -f_receiverSetPoint[remote_ROLL];   // invert ROLL
		}

		// check if remote control has a fault
		if(IS_AUX_LOST(i32_receiverData[remote_AUX1]))
		{
			// Set Bit in EventGroup to say that the remote control has a fault.
			xEventGroupSetBits(gx_fault_EventGroup,fault_REMOTE_CONTROL);
		}
		else
		{
			// Clear Bit in EventGroup to say that the remote control fault is over.
			xEventGroupClearBits(gx_fault_EventGroup,fault_REMOTE_CONTROL);
		}
		HIDE_Fault_Increment(fault_REMOTE_CONTROL,IS_AUX_LOST(i32_receiverData[remote_AUX1]));

		// check if calibration es required
		if(IS_AUX_CALIBRATE(i32_receiverData[remote_AUX1]))
		{
		    return true;
		}

		return false;
	}

	/**
	 * @brief	Initializes GPIOs for the CPPPM receiver
	 */
	void RemoteControl_Init(void)
	{

		ROM_IntPrioritySet(periph_REMOTE_INT, priority_REMOTE_ISR);
		ROM_SysCtlPeripheralEnable(REMOTE_SYSCTL_PERIPH_GPIO);
		ROM_GPIOPinTypeGPIOInput(REMOTE_GPIO_PORT_BASE,REMOTE_PIN_CPPM );
		GPIOIntClear(REMOTE_GPIO_PORT_BASE, REMOTE_PIN_CPPM );				// Pin is reset

		// Pins is rising edge
		ROM_GPIOIntTypeSet(REMOTE_GPIO_PORT_BASE, REMOTE_PIN_CPPM , GPIO_RISING_EDGE);

		// Pin interrups enabled
		GPIOIntEnable(REMOTE_GPIO_PORT_BASE,    REMOTE_PIN_CPPM );

		// Port interrupts enabled
		ROM_IntEnable(periph_REMOTE_INT);

		// Extra Timer for CPPM pulse measurments
		// Timer2 with Timer A, as 16 bit down counter, because prescale doesn't work with up counter ( TI problem )

		// Enable Timer 2 peripheral
		ROM_SysCtlPeripheralEnable(REMOTE_SYSCTL_PERIPH_TIMER);
		// Diasble Timer
		ROM_TimerDisable(periph_REMOTE_TIMER_BASE,periph_REMOTE_TIMER_MODULE);

		// Configure Timer 2A to count down from 0xffff to 0x000 and start again
		ROM_TimerConfigure(periph_REMOTE_TIMER_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);

		// prescaler 80 bei 80Mhz -> 1us
		ROM_TimerPrescaleSet(periph_REMOTE_TIMER_BASE,periph_REMOTE_TIMER_MODULE,80);

		// Start timer module
		ROM_TimerEnable(periph_REMOTE_TIMER_BASE, periph_REMOTE_TIMER_MODULE);

		// By default the signal is lost
		i32_receiverData[remote_AUX1]=AUX_LOST;

		// Set optional eventBits name
		HIDE_Fault_SetEventName(fault_REMOTE_CONTROL,"Rem");
		HIDE_Receive_SetEventName(receiver_REMOTE_DATA,"Rem");
	}

	/**
	 * @brief	Calibrate Roll, Pitch, Yaw to mid and Throttle to min
	 *
	 * @details
	 * To calibrate the remote control the received pulses in the default position
	 * are measured. For roll, pitch and yaw the default position is in the middle,
	 * so that you can command + and - . For throttle it's bottom/min so that
	 * only bigger commands can be send
	 *
	 * @return  void
	 */
	void RemoteControl_Calibrate(void)
	{
		int16_t i;
		for( i=0 ; i<=3 ; ++i )
		{
			i32_offsetData[i]= i32_receiverData[i];
		}

	}

#elif ( setup_REMOTE_SIMULATE == (setup_REMOTE&setup_MASK_OPT1) )

	#define EVENT_FIRE_TIME_MS		20

	/***	UpdateTimerCallback
	**
	**	Parameters:		not used
	**
	**	Description:
	**		This Callback function will be performed from the RTOS Deamon Task
	*/
	static void UpdateTimerCallback(TimerHandle_t xTimer)
	{
		xEventGroupSetBits(gx_receiver_eventGroup,receiver_REMOTE_DATA);
	}

	void RemoteControl_Init(void)
	{
		//
		// Create a timer, which inserts every x ms a UpdateFunktion into the Command Queue
		//
		TimerHandle_t xTimer = xTimerCreate(		// create a timer to update stuff every x ms
						   "",						// timer name (not used)
						   EVENT_FIRE_TIME_MS,		// period of the timer
						   pdTRUE,					// use auto reload
						   ( void * ) 0,			// timer ID
						   UpdateTimerCallback );	// timer callback function
		if( xTimer == math_NULL )
			while(1);
		if( xTimerStart( xTimer, 0 ) != pdPASS )
			while(1);

		// set optional eventBits name
		HIDE_Fault_SetEventName(fault_REMOTE_CONTROL,"Rem");
		HIDE_Receive_SetEventName(receiver_REMOTE_DATA,"Rem");
	}
	uint8_t RemoteControl_GetData(volatile float f_receiverSetPoint[])
	{
		f_receiverSetPoint[remote_ROLL] 	= 0.0f;
		f_receiverSetPoint[remote_PITCH] 	= 0.0f;
		f_receiverSetPoint[remote_YAW] 		= 0.0f;
		f_receiverSetPoint[remote_THROTTLE] = 0.5f;
		return false;
	}
	void RemoteControl_Calibrate(){}

#elif ( setup_REMOTE_NONE == (setup_REMOTE&setup_MASK_OPT1) )

	void 	 RemoteControl_Init(void){}
	uint8_t  RemoteControl_GetData(volatile float f_receiverSetPoint[]){ return false;}
	void 	 RemoteControl_Calibrate(){}
#else

	#error ERROR: define setup_REMOTE (in qc_setup.h)

#endif

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
