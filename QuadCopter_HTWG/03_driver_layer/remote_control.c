/**
 * 		@file 	remote_control.c
 * 		@brief	Driver to read values over a remote control
 *//*	@author Tobias Walter
 * 		@date 	12.03.2016	(last modified)
 */

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <stdint.h>
#include <stdbool.h>

//  Hardware Specific
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "peripheral_setup.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "event_groups.h"
#include "receiver_task.h"
#include "remote_control.h"

// utils
#include "qc_math.h"
#include "fault.h"

// setup
#include "qc_setup.h"

/* ------------------------------------------------------------ */
/*				Local Defines									*/
/* ------------------------------------------------------------ */

// Maximum values remote control
#define ROLL_MAX		    (45.0f*math_PI/180.0f)				// [-max...+max] [rad/s]
#define PITCH_MAX		    (45.0f*math_PI/180.0f)				// [-max...+max] [rad/s]
#define YAW_MAX			    (45.0f*math_PI/180.0f)				// [-max...+max] [rad/s] // TODO make max higher
#define THROTTLE_MAX	    (1.0f)							// [-max   ...+max] [throttle]
#define THROTTLE_RC_EXPO    (0.3f)                           // makes the stick input around throttle less sensitive

/** \brief	Number of Channles in the CPPM signal */
#define remote_CPPM_CHANNELS 		  ( 6 )

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */

extern volatile EventGroupHandle_t gx_receiver_eventGroup;

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

const float f_maxValues[] = {ROLL_MAX, PITCH_MAX, YAW_MAX, THROTTLE_MAX};

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Select the Mode									*/
/* ------------------------------------------------------------ */

#if ( setup_REMODE_CPPM == (setup_REMOTE&setup_MASK_OPT1)) || DOXYGEN

	/* ------------------------------------------------------------ */
	/*				Include File Definitions						*/
	/* ------------------------------------------------------------ */

	/* ------------------------------------------------------------ */
	/*				Local Defines									*/
	/* ------------------------------------------------------------ */

	// Time Constants for the CPPM Signal [µs]
	#define CPPM_SYNC_TIME_MIN	  		( 8000 )
	#define CPPM_SYNC_TIME_MAX	  		( 18000 )

	// average max-min value from a channel [µs]
    #define CPPM_MIN_MAX_RANGE_THROTTEL ( 950.0f )

    #define CPPM_MIN_MAX_RANGE			( 816.0f ) // TODO remember 760 as value from master students was wrong 800 much better
	#define CPPM_MIN_MAX_RANGE_HALF		(CPPM_MIN_MAX_RANGE/2.0f)


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

	//
	// define the desired peripheral setup (see peripheral_setup.h)
	//
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

	/* ------------------------------------------------------------ */
	/*				Local Type Definitions							*/
	/* ------------------------------------------------------------ */

	enum receiverState_e
	{
		STATE_SYNC,
		STATE_NORMAL
	};

	/* ------------------------------------------------------------ */
	/*				Forward Declarations							*/
	/* ------------------------------------------------------------ */

	void RemoteControl_CppmIntHandler(void);

	/* ------------------------------------------------------------ */
	/*				Global Variables								*/
	/* ------------------------------------------------------------ */

	/* ------------------------------------------------------------ */
	/*				Local Variables									*/
	/* ------------------------------------------------------------ */

	static int32_t i32_receiverData[remote_CPPM_CHANNELS];
	static int32_t i32_offsetData[4];
	static const uint16_t ui16_cppmMap[remote_CPPM_CHANNELS] = CPPM_ORDER;

	/* ------------------------------------------------------------ */
	/*				Procedure Definitions							*/
	/* ------------------------------------------------------------ */

	/**
	 * \brief	Interrupt handler for CPPM-Mode.
	 *
	 *			state machine for syncronisation.
	 *			after a data frame is collected the eventBit receiver_REMOTE_DATA is set in gx_receiver_eventGroup
	 *			if syncronisation is lost -> ui32_receiverData[remote_AUX1]=AUX_LOST;
	 */
	void RemoteControl_CppmIntHandler(void)
	{
		// Es könnte ggf. Möglich direkt über die Capture/Compare-Einheit
		// eines Timers genau das gleiche zu tun -> weniger ISR aufwand

		static enum receiverState_e e_receiverState=STATE_SYNC;

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
				// not the end of frame?
				if (ui16_loopCounter<remote_CPPM_CHANNELS-1)
				{
					// save the time value for the current Channel
					i32_receiverData[ui16_cppmMap[ui16_loopCounter] ] = ui16_diffTime;
					ui16_loopCounter++;
				}
				// last element of the Frame
				else if (ui16_loopCounter == remote_CPPM_CHANNELS-1)
				{
					// save the time value for the current Channel
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
				// end of frame (sync time)
				else
				{
					// end of frame time value shoult be Sync time
					if( ui16_diffTime > CPPM_SYNC_TIME_MIN  && ui16_diffTime < CPPM_SYNC_TIME_MAX )
					{
						// all is good, start new frame
						ui16_loopCounter=0;
					}
					else
					{
						// something bad happend, syncronisation lost -> try to sync again
						e_receiverState=STATE_SYNC;

						// store in aux the information, that sync is lost
						i32_receiverData[remote_AUX1]=AUX_LOST;
					}
				}

				break;
			}
			case STATE_SYNC:
			{
				// wait for the sync time in the cppm signal
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
	 * \brief	prepare data and copy it into f_receiverSetPoint
	 *
	 *			fire eventBit fault_REMOTE_CONTROL if there is a fault,
	 *			else clear the bit.
	 *			If the remote controle orders a calibration, return true.
	 * \param	f_receiverSetPoint Array to copy in (see remote_control.h for the order)
	 * \return	true if Calibrate is required, false else
	 * \note    Events: EventBit fault_REMOTE_CONTROL will be set or cleared in gx_fault_EventGroup
	 */
	uint8_t  RemoteControl_GetData(volatile float f_receiverSetPoint[])
	{
		//
		//	Prepare Data
		//
		int16_t i;
		float help, help_3;
		for( i=0 ; i<4 ; ++i )
		{
			help=(float) (i32_receiverData[i]-i32_offsetData[i]);		// sub offset
			if(i==remote_THROTTLE)
			{
				help=help/CPPM_MIN_MAX_RANGE_THROTTEL*f_maxValues[i];			// scale to [0...1]
				help = (2.0 * help) - 1;                                    // move it to [-1..1]
				help=math_LIMIT(help, -f_maxValues[i], f_maxValues[i]);	// limit to [-1...1]
				help_3 = help*help*help;
				help=(1.0 - THROTTLE_RC_EXPO) * help + help_3 * THROTTLE_RC_EXPO;
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

		f_receiverSetPoint[remote_YAW]=     -f_receiverSetPoint[remote_YAW];	// invert YAW
		f_receiverSetPoint[remote_PITCH]=   -f_receiverSetPoint[remote_PITCH];  // invert PITCH
		f_receiverSetPoint[remote_ROLL]=    -f_receiverSetPoint[remote_ROLL];   // invert ROLL
		}

		// check if remote control has a fault
		//
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

		//
		// check if calibration es required
		//
		if(IS_AUX_CALIBRATE(i32_receiverData[remote_AUX1]))
			return true;
		return false;
	}

	/**
	 * \brief	initializes GPIOs for the receiver
	 */
	void RemoteControl_Init(void)
	{

		ROM_IntPrioritySet(periph_REMOTE_INT, priority_REMOTE_ISR);
		ROM_SysCtlPeripheralEnable(REMOTE_SYSCTL_PERIPH_GPIO);
		ROM_GPIOPinTypeGPIOInput(REMOTE_GPIO_PORT_BASE,REMOTE_PIN_CPPM );
		GPIOIntClear(REMOTE_GPIO_PORT_BASE, REMOTE_PIN_CPPM );				// P isr reset

		// Pins is rising edge
		ROM_GPIOIntTypeSet(REMOTE_GPIO_PORT_BASE, REMOTE_PIN_CPPM , GPIO_RISING_EDGE);

		// Pin ints frei
		GPIOIntEnable(REMOTE_GPIO_PORT_BASE,    REMOTE_PIN_CPPM );

		// Port ints frei
		ROM_IntEnable(periph_REMOTE_INT);

		// Extra Timer für CPPM Pulsmessung
		// Timer2 und dort Timer A
		// als 16 bit runter zählen, da beim hochzählen das Prescale nicht funktioniert ( TI abhängig )

		// timer 1 aktivieren
		ROM_SysCtlPeripheralEnable(REMOTE_SYSCTL_PERIPH_TIMER);
		// zuerst Timer disable
		ROM_TimerDisable(periph_REMOTE_TIMER_BASE,periph_REMOTE_TIMER_MODULE);

		// timer 1A zählt hoch von 0xffff bis 0x000 und fängt von vorne an
		ROM_TimerConfigure(periph_REMOTE_TIMER_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);

		// von 0xffff  bis 0 zählen und dann von vorne

		// prescaler 80 bei 80Mhz -> 1us
		// prescaler 50 bei 50Mhz  -> 1us

		ROM_TimerPrescaleSet(periph_REMOTE_TIMER_BASE,periph_REMOTE_TIMER_MODULE,80);

		// timer 1A starten
		ROM_TimerEnable(periph_REMOTE_TIMER_BASE, periph_REMOTE_TIMER_MODULE);

		// by default the signal is lost
		i32_receiverData[remote_AUX1]=AUX_LOST;

		// set optional eventBits name
		HIDE_Fault_SetEventName(fault_REMOTE_CONTROL,"Rem");
		HIDE_Receive_SetEventName(receiver_REMOTE_DATA,"Rem");
	}

	/**
	 * \brief	Calibrate Roll, Pitch, Yaw to mid and Throttle to min
	 */
	void RemoteControl_Calibrate(void)
	{
		int16_t i;
		for( i=0 ; i<=3 ; ++i )
		{
			i32_offsetData[i]=i32_receiverData[i];
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
