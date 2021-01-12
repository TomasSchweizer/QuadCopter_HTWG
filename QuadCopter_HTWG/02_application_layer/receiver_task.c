/**
 * 		@file 	receiver_task.c
 * 		@brief	Receiver Task waits for asynchronous events
 *  			and handles/answers them
 *  			remote control, telemetrie, ...
 *//*	@author Tobias Grimm
 * 		@date 	31.05.2016	(last modified)
 */

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <stdint.h>
#include <stdbool.h>

//  FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

// setup
#include <prioritys.h>

// drivers
#include "remote_control.h"
#include "sensor_driver.h"		// to require sensor calibration
#include "display_driver.h"
#include "debug_interface.h"

// application
#include "flight_task.h"
#include "receiver_task.h"
#include "fault.h"

// utils
#include "qc_math.h"
#include "workload.h"
#include "count_edges.h"

/* ------------------------------------------------------------ */
/*				Local Defines									*/
/* ------------------------------------------------------------ */

#define RECEIVER_TASK_STACK_SIZE        100        // Stack size in words

#define INPUT_REMOTE_CONTROL			receiver_REMOTE_DATA
#define INPUT_AUTOPILOT					receiver_AUTOPILOT_DATA
#define INPUT_TELEMETRIE				receiver_TELEMETRIE_DATA

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */


static void ReceiverTask(void *pvParameters);
static uint32_t ChangeFlightInput(uint32_t ui32_inputDefine);

/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */

// used global variables
extern volatile EventGroupHandle_t   gx_fault_EventGroup;
extern volatile uint32_t 		     gui32_flight_setPoint[4];

/**
 * \brief	event bits to indicate if new
 *			receiver data is there
 *
 *			(see receiver_task.h for bit order)
 */
volatile EventGroupHandle_t gx_receiver_eventGroup = 0;

/**
 * \brief	Current input mode for flightStabilisation (e.g. receiver, thelemetrie, ...)
 *
 * 			only one bit is set to indicate the current
 *			input mode for the flight-Task
 *			(bits are the same as in fault.h)
 * \note	Write access:	receiver_task
 */
volatile uint32_t 		   gui32_receiver_flightStabInput;

/**
 * \brief	set point values for flight stabilisation
 *
 *			(see flight_task.h for signal order)
 * \note	Write access:	receiver_task
 */
float gf_receiver_setPoint[4];

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

countEdges_handle_p p_receive_coundEdges;
static uint8_t      ui8_calibrateRemoteAtStartFlag = 1;

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

/**
 * \brief	Draw info about ReceiverTask on the Display
 */
static void ReceiverTask_DrawDisplay(void)
{
	const uint8_t yOffset 		= 64;
	u8g_SetFont(&gs_display, u8g_font_04b_03r);		// u8g_font_unifont
	u8g_DrawStr(&gs_display,0,  	yOffset + 0,"Ro");
	u8g_DrawStr(&gs_display,0+12,  	yOffset + 0,u8g_8toa((int8_t)math_RAD2DEC(gf_receiver_setPoint[flight_ROLL]),3));
	u8g_DrawStr(&gs_display,36,  	yOffset + 0,"Pi");
	u8g_DrawStr(&gs_display,36+12,  yOffset + 0,u8g_8toa((int8_t)math_RAD2DEC(gf_receiver_setPoint[flight_PITCH]),3));
	u8g_DrawStr(&gs_display,72,  	yOffset + 0,"Ya");
	u8g_DrawStr(&gs_display,72+12,  yOffset + 0,u8g_8toa((int8_t)math_RAD2DEC(gf_receiver_setPoint[flight_YAW]),3));
	u8g_DrawStr(&gs_display,108,  	yOffset + 0,"T");
	u8g_DrawStr(&gs_display,108+5,  yOffset + 0,u8g_u8toa((uint8_t)(gf_receiver_setPoint[flight_THROTTLE]*100.0f),3));
}

/**
 * \brief	use drivers to initialize GPIOs for the
 *			ReceiverTask und start the Task
 * \return 	false if Task creation was successful,
 *			true else
 */
uint32_t ReceiverTask_Init(void)
{
	HIDE_Display_InsertDrawFun(ReceiverTask_DrawDisplay);

	#if	( setup_DEV_SUM_RECEIVS )
	    HIDE_Display_InsertDrawFun(HIDE_Receive_DrawDisplay);
		p_receive_coundEdges = CountEdges_Create(receiver_COUNT);
		if( p_receive_coundEdges == math_NULL )
			return(true);
	#endif

	// init input for FlyStab-Task is the receiver
	gui32_receiver_flightStabInput = INPUT_REMOTE_CONTROL;

	RemoteControl_Init();    // Initialisierung des Empfängers in PPM

    // Attempt to create the event group
	gx_receiver_eventGroup = xEventGroupCreate();

    // Was the event group created successfully?
    if( gx_receiver_eventGroup == math_NULL )
    	return(true);

	TaskHandle_t  x_TaskHandle;
	// Create the receiver task.
    if(xTaskCreate(ReceiverTask,"ReceivTask", RECEIVER_TASK_STACK_SIZE, math_NULL,tskIDLE_PRIORITY + priority_RECEIVER_TASK, &x_TaskHandle ) != pdTRUE)
        return(true);

    // Success
    HIDE_Workload_StoreTaskHandle(priority_NUM_RECIVER_TASK,x_TaskHandle);
    return(false);
}

/**
 * \brief	copy current setPoints from the ReceiverTask
 * \param	f_setPoint 		Array to copy in
 * \note	a race condition can happen, but this is not critical
 */
void ReceiverTask_GetSetPoints(float f_setPoint[])
{
	uint8_t i;
	for(i=0;i<4;++i)
		f_setPoint[i]=gf_receiver_setPoint[i];
}

/**
 * \brief	ReceiverTask waits for asynchronous
 *  		Events. (handles/answers them)
 *  		remote control, telemetrie, ...
 */
static void ReceiverTask(void *pvParameters)
{
	EventBits_t x_receiverEventBits;

	// Loop forever
	while(1)
	{
	    //  Wait for any receiver event
		x_receiverEventBits = xEventGroupWaitBits(gx_receiver_eventGroup,
				receiver_REMOTE_DATA | receiver_AUTOPILOT_DATA | receiver_TELEMETRIE_DATA,
				pdTRUE,          // clear Bits before returning.
				pdFALSE,         // wait for any Bits
				portMAX_DELAY ); // Wait the maximum time.

		//
		//  remote control data is there
		//
	    if( receiver_REMOTE_DATA & x_receiverEventBits )
	    {
	    	HIDE_Receive_Increment(receiver_REMOTE_DATA);
	    	// is the current input mode for flight stabilisation the remote control?
	    	if( INPUT_REMOTE_CONTROL & gui32_receiver_flightStabInput )
	    	{
	    		// get data, is calibration required? & is the flight_state RESTING?
	    		if((RemoteControl_GetData(&gf_receiver_setPoint[0]) && ge_flight_state==RESTING) || ui8_calibrateRemoteAtStartFlag)
	    		{
	    		    ui8_calibrateRemoteAtStartFlag = 0;
	    			RemoteControl_Calibrate();		// calibrate remote control
	    			Sensor_CalibrateRequire();		// require calibration for sensor (flight_task will do calibration algorithm)
	    		}
	    	}
	    }

		//
		// autopilot data is there
		//
	    if( receiver_AUTOPILOT_DATA & x_receiverEventBits )
	    {
	    	HIDE_Receive_Increment(receiver_AUTOPILOT_DATA);
	    }

		//
		// telemetrie data is there
		//
	    if( receiver_TELEMETRIE_DATA & x_receiverEventBits )
	    {
	    	HIDE_Receive_Increment(receiver_TELEMETRIE_DATA);
	    	uint32_t ui32_inputDefine;
	    	// read telemetrie and change e.g. the input mode for flight stabilisation
	    	ui32_inputDefine = INPUT_REMOTE_CONTROL ;
	    	if( ChangeFlightInput(ui32_inputDefine))
	    	{
	    		// Change isn't allowed
	    	}

	    	// is the current input mode for flight stabilisation the telemetrie?
	    	if( INPUT_TELEMETRIE & gui32_receiver_flightStabInput )
	    	{
	    		// jetzt darf ich daten aufbereiten und rein kopieren
	    	}
	    }
	}
}

/**
 * \brief	change the input signal for flight-Task
 *			but only when it es allowed to change
 * \param	ui32_inputDefine  	e.g. INPUT_REMOTE_CONTROL, ...
 * \return	false if change was allowed,
 *			true  else
 */
static uint32_t ChangeFlightInput(uint32_t ui32_inputDefine)
{
	EventBits_t x_faultEventBits;
	x_faultEventBits = xEventGroupGetBits( gx_fault_EventGroup );

	// if desired input has no fault
	if (( ui32_inputDefine & x_faultEventBits ) == 0 )
	{
		// Change is allowed
		gui32_receiver_flightStabInput = ui32_inputDefine;
		return(false);
	}

	// Change isn't allowed
	return(true);
}

#if	( setup_DEV_SUM_RECEIVS )

	/**
	 * \brief	Draw info about receive Counts on the Display
	 * \note	to enable this HIDE function set setup_DEV_SUM_RECEIVS in qc_setup.h
	 */
	void HIDE_Receive_DrawDisplay(void)
	{
		EventBits_t x_faultEventBits = xEventGroupGetBits( gx_receiver_eventGroup );
		const uint8_t drawHight 	= 6;
		const uint8_t xOffset 		= 55;
		u8g_SetFont(&gs_display, u8g_font_04b_03r);		// u8g_font_unifont
		uint8_t i;
		uint16_t hight=drawHight-1;
		const char* p_name;
		for(i=0;i<receiver_COUNT;++i)
		{
			p_name=CountEdges_GetName(p_receive_coundEdges,i);
			if(p_name!=0)
			{
				u8g_DrawStr(&gs_display, xOffset + 0,  hight,p_name);						// Name
				u8g_DrawStr(&gs_display, xOffset + 15+1, hight, u8g_u16toa(CountEdges_Get(p_receive_coundEdges,i),3));		// How often fired
				hight+=drawHight;
			}
		}
	}

	/**
	 * \brief	store the name of the receive eventBit into p_receive_coundEdges
	 *
	 *			(this should be performed before scheduler starts)
	 *			e.g. in the init of the driver who fires the eventBit
	 * \param	ui32_eventBit		the receive eventBit (see receiver_task.h)
	 * \param	pc_name				const string name
	 * \note	to enable this HIDE function set setup_DEV_SUM_RECEIVS in qc_setup.h
	 */
	void HIDE_Receive_SetEventName(uint32_t ui32_eventBit,const char* pc_name)
	{
		CountEdges_SetName(p_receive_coundEdges,CountEdges_Bit2Num(ui32_eventBit),pc_name);
	}

	/**
	 * \brief	increment at ui32_receiveEventBit in p_receive_coundEdges
	 * \param	ui32_receiveEventBit	the receive eventBit (see receiver_task.h)
	 * \note	to enable this HIDE function set setup_DEV_SUM_RECEIVS in qc_setup.h
	 */
	void HIDE_Receive_Increment(uint32_t ui32_receiveEventBit)
	{
		CountEdges_Increment(p_receive_coundEdges,CountEdges_Bit2Num(ui32_receiveEventBit));
	}

#endif

