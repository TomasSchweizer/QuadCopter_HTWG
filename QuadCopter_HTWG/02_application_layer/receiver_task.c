/*===================================================================================================*/
/*  receiver_task.c                                                                                  */
/*===================================================================================================*/

/*
*   file   receiver_task.c
*
*   brief  Receiver task waits for asynchronous events and handles them (remote control, telemetry, ...)
*
*   details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>31/05/2016      <td>Tobias Grimm        <td>Implementation & Last modification of MAs
*   <tr><td>20/01/2020      <td>Tomas Schweizer     <td>Changes to remote control start-up
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
#include "prioritys.h"

// Application
#include "flight_task.h"
#include "receiver_task.h"
#include "fault.h"

// Drivers
#include "remote_control.h"
#include "sensor_driver.h"      // to require sensor calibration
#include "display_driver.h"
#include "debug_interface.h"

//  FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

// Utilities
#include "qc_math.h"
#include "workload.h"
#include "count_edges.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Defines                                                 */
/* ---------------------------------------------------------------------------------------------------*/

#define RECEIVER_TASK_STACK_SIZE        100                         ///< Task stack size in words

#define INPUT_REMOTE_CONTROL			receiver_REMOTE_DATA        ///< Define for input mode RC
#define INPUT_AUTOPILOT					receiver_AUTOPILOT_DATA     ///< Define for input mode UAV !!!NOT IMPLEMENTED!!!
#define INPUT_TELEMETRIE				receiver_TELEMETRIE_DATA    ///< Define for input mode telemetry !!!NOT IMPLEMENTED!!!

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Type Definitions                                        */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Forward Declarations                                          */
/* ---------------------------------------------------------------------------------------------------*/
static void ReceiverTask(void *pvParameters);
static uint32_t ChangeFlightInput(uint32_t ui32_inputDefine);

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

// Extern global variables
extern volatile EventGroupHandle_t   gx_fault_EventGroup;           //Fault event group to detect receiver communication faults
extern volatile uint32_t 		     gui32_flight_setPoint[4];      // Flight task setpoints (copy from receiver to flight task)

// Global variables
volatile EventGroupHandle_t gx_receiver_eventGroup = 0;             ///< Event bits to indicate new receiver data has arrived
volatile uint32_t 		   gui32_receiver_flightStabInput;          ///< Current input mode (receiver, telemetry)
float gf_receiver_setPoint[4];                                      ///< Receiver task setpoint values

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Variables                                               */
/* ---------------------------------------------------------------------------------------------------*/

static countEdges_handle_p p_receive_coundEdges;                    ///< To count received RC signals
static uint8_t      ui8_calibrateRemoteAtStartFlag = 1;             ///< Flag to trigger sensor calibration sequence at start-up

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Procedure Definitions                                         */
/* ---------------------------------------------------------------------------------------------------*/

/**
 * @brief	Draw the setpoints of the receiver task on the display
 *
 * @return  void
 */
static void ReceiverTask_DrawDisplay(void)
{
	const uint8_t yOffset 		= 64;
	u8g_SetFont(&gs_display, u8g_font_04b_03r);		// u8g_font_unifont
	// Roll setpoint in degrees
	u8g_DrawStr(&gs_display,0,  	yOffset + 0,"Ro");
	u8g_DrawStr(&gs_display,0+12,  	yOffset + 0,u8g_8toa((int8_t)math_RAD2DEC(gf_receiver_setPoint[flight_ROLL]),3));
	// Pitch setpoint in degrees
	u8g_DrawStr(&gs_display,36,  	yOffset + 0,"Pi");
	u8g_DrawStr(&gs_display,36+12,  yOffset + 0,u8g_8toa((int8_t)math_RAD2DEC(gf_receiver_setPoint[flight_PITCH]),3));
	// Yaw setpoint in degrees
	u8g_DrawStr(&gs_display,72,  	yOffset + 0,"Ya");
	u8g_DrawStr(&gs_display,72+12,  yOffset + 0,u8g_8toa((int8_t)math_RAD2DEC(gf_receiver_setPoint[flight_YAW]),3));
	// Throttle setpoint but scaled to 0 to 100
	u8g_DrawStr(&gs_display,108,  	yOffset + 0,"T");
	u8g_DrawStr(&gs_display,108+5,  yOffset + 0,u8g_u8toa((uint8_t)(((gf_receiver_setPoint[flight_THROTTLE]+1.0)/2.0)*100.0f),3));
}

/**
 * @brief	Uses drivers to initialize GPIOs for the receiver task and starts the task
 *
 * @return 	false --> Task creation was successful\n
 *			true --> Task creation was not succesful
 */
uint32_t ReceiverTask_Init(void)
{
	HIDE_Display_InsertDrawFun(ReceiverTask_DrawDisplay);

	// Display sum of received RC signals
	#if	( setup_DEV_SUM_RECEIVS )
	    HIDE_Display_InsertDrawFun(HIDE_Receive_DrawDisplay);
		p_receive_coundEdges = CountEdges_Create(receiver_COUNT);
		if( p_receive_coundEdges == math_NULL )
			return(true);
	#endif

	// Init input mode as remote control
	gui32_receiver_flightStabInput = INPUT_REMOTE_CONTROL;

	// Init the divers needed for remote control
	RemoteControl_Init();

    // Attempt to create the event group
	gx_receiver_eventGroup = xEventGroupCreate();

    // Check if the event group was created successfully
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
 * @brief	Copy current setpoints of the receiver task
 *
 * @param	f_setPoint --> array to copy the setpoints into
 *
 * @return  void
 *
 * @note	Race condition can occur between fligth and receiver
 *          task but this is not critical
 *          because one older setpoint is no problem.
 */
void ReceiverTask_GetSetPoints(float f_setPoint[])
{
	uint8_t i;
	for(i=0;i<4;++i)
		f_setPoint[i]=gf_receiver_setPoint[i];
}

/**
 * @brief	Receiver task waits for asynchronous events. and handles them
 *
 * @details
 *
 * Procedure:
 *
 * 1. Wait for event bits
 * 2. Check over event bits the input mode
 * 3. Get remote control data
 * 4. Check if calibration is required
 * 5. If yes calibrate remote control and sensors
 *
 * @param   pvParameters --> not used
 *
 * @return  void
 *
 * @note    Autopilot and telemetry are not implemented, only remote control
 *
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
				pdTRUE,          // Clear bits before returning.
				pdFALSE,         // Wait for any bits
				portMAX_DELAY ); // Wait the maximum time.

		//  Remote control data is received
	    if( receiver_REMOTE_DATA & x_receiverEventBits )
	    {
	        // Increment receive counter for remote control data
	    	HIDE_Receive_Increment(receiver_REMOTE_DATA);

	    	// Check if the current input mode is the remote control
	    	if( INPUT_REMOTE_CONTROL & gui32_receiver_flightStabInput )
	    	{
	    		// Get remote control data. Check if calibration is required & if the flight state is RESTING or if first startup
	    		if((RemoteControl_GetData(&gf_receiver_setPoint[0]) && ge_flight_state==RESTING) || ui8_calibrateRemoteAtStartFlag)
	    		{
	    		    ui8_calibrateRemoteAtStartFlag = 0; // Set startup flag to 0
	    			RemoteControl_Calibrate();		    // Calibrate remote control
	    			Sensor_CalibrateRequire();		    // Require calibration for sensor (flight task will do calibration algorithm)
	    		}
	    	}
	    }

		// Autopilot data is received
	    if( receiver_AUTOPILOT_DATA & x_receiverEventBits )
	    {
	    	HIDE_Receive_Increment(receiver_AUTOPILOT_DATA);
	    }

		// Telemetry data is received
	    if( receiver_TELEMETRIE_DATA & x_receiverEventBits )
	    {
	    	HIDE_Receive_Increment(receiver_TELEMETRIE_DATA);
	    	uint32_t ui32_inputDefine;

	    	// Read telemetrie and change e.g. the input mode
	    	ui32_inputDefine = INPUT_REMOTE_CONTROL ;
	    	if( ChangeFlightInput(ui32_inputDefine) )
	    	{
	    		// Change isn't allowed
	    	}

	    	// Check if the current input mode is telemetry
	    	if( INPUT_TELEMETRIE & gui32_receiver_flightStabInput )
	    	{
	    		// use telemetry data
	    	}
	    }
	}
}

/**
 * @brief	Change the input mode but only when it's allowed to change
 *
 * @param	ui32_inputDefineT --> The input mode to change to
 *
 * @return	false --> Change was allowed
 *			true --> Change was not allowed
 */
static uint32_t ChangeFlightInput(uint32_t ui32_inputDefine)
{
	EventBits_t x_faultEventBits;
	x_faultEventBits = xEventGroupGetBits( gx_fault_EventGroup );

	// If desired input has no fault
	if (( ui32_inputDefine & x_faultEventBits ) == 0 )
	{
		// Change is allowed
		gui32_receiver_flightStabInput = ui32_inputDefine;
		return(false);
	}

	// Change isn't allowed
	return(true);
}

#if	( setup_DEV_SUM_RECEIVS ) || DOXYGEN

	/**
	 * @brief	Draw info about receive counts on the display
	 *
	 * @return  void
	 *
	 * @note	To enable this HIDE function set setup_DEV_SUM_RECEIVS in qc_setup.h
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
	 * @brief	Store the name of the receive eventBit into p_receive_coundEdges
	 *
	 * @param	ui32_eventBit --> The receive eventBit (see receiver_task.h)
	 * @param	pc_name --> Const string name
	 *
	 * @return  void
	 *
	 * @note    This should be performed before scheduler starts, e.g. in the init of the driver who fires the eventBit.
	 *      	To enable this HIDE function set setup_DEV_SUM_RECEIVS in qc_setup.h.
	 */
	void HIDE_Receive_SetEventName(uint32_t ui32_eventBit,const char* pc_name)
	{
		CountEdges_SetName(p_receive_coundEdges,CountEdges_Bit2Num(ui32_eventBit),pc_name);
	}

	/**
	 * @brief	Increment a ui32_receiveEventBit in p_receive_coundEdges
	 *
	 * @param	ui32_receiveEventBit --> The receive eventBit (see receiver_task.h)
	 *
	 * @return  void
	 *
	 * @note	To enable this HIDE function set setup_DEV_SUM_RECEIVS in qc_setup.h
	 */
	void HIDE_Receive_Increment(uint32_t ui32_receiveEventBit)
	{
		CountEdges_Increment(p_receive_coundEdges,CountEdges_Bit2Num(ui32_receiveEventBit));
	}

#endif

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
