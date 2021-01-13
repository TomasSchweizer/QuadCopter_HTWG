/**
 * 		@file 	flight_task.c
 * 		@brief	Task for flight stabilization
 *
 *  			sensorfusion, control algorithm, motor output,
 *  			state machine for critical events
 *//*	@author Tobias Grimm
 * 		@date 	01.06.2016	(last modified)
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

// drivers
#include "motor_driver.h"
#include "sensor_driver.h"
#include "display_driver.h"
#include "debug_interface.h"

// application
#include "flight_control.h"
#include "flight_task.h"
#include "receiver_task.h"		// GetSetPoints


// utils
#include "workload.h"
#include "fault.h"
#include "qc_math.h"

// setup
#include "prioritys.h"

/* ------------------------------------------------------------ */
/*				Local Defines									*/
/* ------------------------------------------------------------ */

#define TASK_STACK_SIZE        	 		( 250 )       // Stack size in words
#define TASK_PERIOD_MS			 		( 2 )

#define LANDING_TIME_MS					( 5000 )	  // time for the transition of the states LANDING -> RESTING

#define MOTOR_OVERCURRENT_MAX_THROTTLE	( 0.5f )	  // throttle goes from [0...1]
#define THROTTLE_LANDING				( 0.25f)	  // throttle goes from [0...1]
#define THRESHOLD_FLYING_REQUIRED		( 0.2f )	  // throttle goes from [0...1]
#define THRESHOLD_FLYING_NOT_REQUIRED	( 0.1f )	  // throttle goes from [0...1]
#define MOTOR_SET_POINT_SENSOR_FAULT	((uint16_t)(0.25f*0xFFFF))

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */

enum transitionState_e
{
	ENTRY,		// transition entry for comming state, transition exit for current state
	DURING		// during state execution
};

/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

static void    FlightTask(void *pvParameters);
static void    StateFlying(EventBits_t x_faultEventBits);
static void    StateLanding(EventBits_t x_faultEventBits);
static uint8_t IsFlyingRequired(void);
static uint8_t IsFlyingNotRequired(void);
static uint8_t IsFlyingPossible(EventBits_t x_faultEventBits);
static void    StateResting(void);

/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */

// used global variables
extern volatile uint32_t 		    gui32_receiver_flightStabInput;
extern volatile EventGroupHandle_t  gx_fault_EventGroup;
//extern volatile EventGroupHandle_t  gx_sensor_EventGroup;


/**
 * \brief	set point values for flight stabilisation.
 *
 * 			flight_ROLL 	[rad],
 * 			flight_PITCH	[rad],
 * 			flight_YAW		[rad/s],
 * 			flight_THROTTLE [0 ... 1]
 * \note	Write access:	flight_task
 */
float gf_flight_setPoint[4];

/**
 * \brief	current state of the flight_task (see flight_task.h)
 * \note	Write access:	flight_task
 */
enum flight_state_e ge_flight_state=RESTING;


countEdges_handle_p p_flight_countEdges;

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

/**
 * \brief	Draw info about FlightTask on the Display
 */
static void FlightTaskDrawDisplay(void)
{
	const uint8_t yOffset 		= 52;
	const uint8_t xOffset 		= 55;
	u8g_SetFont(&gs_display, u8g_font_04b_03r);		// u8g_font_unifont
	if(ge_flight_state == RESTING){

		if(Sensor_IsCalibrateRequired()){
		    u8g_DrawStr(&gs_display,xOffset, yOffset,"CALIBRATE");
		}
		else
		{
		    u8g_DrawStr(&gs_display,xOffset,yOffset,"RESTING");
		}
	}
	else if(ge_flight_state == LANDING)
		u8g_DrawStr(&gs_display,xOffset,yOffset,"LANDING");
	else if(ge_flight_state == FLYING)
		u8g_DrawStr(&gs_display,xOffset,yOffset,"FLYING");
	else
		u8g_DrawStr(&gs_display,xOffset,yOffset,"STATE?");
}

/**
 * \brief	use drivers to initialize peripherals for the
 *			FlightTask und start the Task
 * \return	false if Task creation was successful,
 *			true  else
 */
uint32_t FlightTask_Init(void)
{
	HIDE_Display_InsertDrawFun(FlightTaskDrawDisplay);
	Sensor_InitPeriph();
	HIDE_Display_InsertDrawFun(Sensor_DrawDisplay);
	HIDE_Debug_USB_InsertComFun(HIDE_Sensor_SendDataOverUSB, 1);
    Motor_InitPeriph();
	HIDE_Display_InsertDrawFun(Motor_DrawDisplay);
	HIDE_Debug_USB_InsertComFun(HIDE_Motor_SendDataOverUSB, 0);

	// insert pid
    #if( setup_DEV_PID_TUNE )
	HIDE_Debug_USB_InsertComFun(HIDE_Control_Debug_USB_GetPID, 0);
	HIDE_Display_InsertDrawFun(HIDE_Control_PID_TUNE_DrawDisplay);
    #endif


    TaskHandle_t  x_TaskHandle;
	// Create the flight task.
    if(xTaskCreate(FlightTask,"FlightTask", TASK_STACK_SIZE, math_NULL,
                   tskIDLE_PRIORITY + priority_FLIGHT_TASK, &x_TaskHandle ) != pdTRUE)
    {
        return(true);
    }

    uint8_t i;
    for(i=0;i<4;++i)
    	gf_flight_setPoint[i]=0;

    // Success
    HIDE_Workload_StoreTaskHandle(priority_NUM_FLIGHT_TASK,x_TaskHandle);
    return(false);
}

/**
 * \brief	flight stabilization
 *  		sensorfusion, control algorithm, motor output
 *  		state machine for critical events
 * \param	pvParameters	not used
 */
static void FlightTask(void *pvParameters)
{

	Motor_InitMotor();
	// Delay 1ms just for safety that motor is initalized before sensors get initialized not necessary but nicer
	vTaskDelay( 1/ portTICK_PERIOD_MS );
	Sensor_InitSensor();


	TickType_t  x_lastWakeTime;
	EventBits_t x_faultEventBits;

	//  Initialise the x_lastWakeTime variable with the current time.
	x_lastWakeTime = xTaskGetTickCount();

	// timer for the transition of the states LANDING -> RESTING
	int32_t i32_landingTimer;

	enum transitionState_e e_state=ENTRY;

	// Loop forever
	while(1)
	{
		//  Wait for the next cycle.
		vTaskDelayUntil( &x_lastWakeTime, TASK_PERIOD_MS / portTICK_PERIOD_MS );

		// get the faultEventBits to decide the emergency mode
		x_faultEventBits = xEventGroupGetBits( gx_fault_EventGroup );

		switch(ge_flight_state)
		{
			case RESTING:
			{
				// ENTRY
				if(e_state==ENTRY)
				{
					Control_Reset();
					e_state=DURING;
				}
				// DURING
				StateResting();



				// EXIT
				if(IsFlyingRequired() && Sensor_IsCalibrateReady() && IsFlyingPossible(x_faultEventBits))
				{
					Sensor_CalibrateStop();
					ge_flight_state=FLYING;
					e_state=ENTRY;
				}
				break;
			}
			case FLYING:
			{
				// ENTRY
				if(e_state==ENTRY)
				{
					e_state=DURING;
				}
				// DURING
				StateFlying(x_faultEventBits);

				// EXIT
				if( 0 )//###!IsFlyingPossible(x_faultEventBits))
				{
					ge_flight_state=LANDING;
					e_state=ENTRY;
				}
				else if (IsFlyingNotRequired())
				{
					ge_flight_state=RESTING;
					e_state=ENTRY;
				}
				break;
			}
			case LANDING:
			{
				// ENTRY
				if(e_state==ENTRY)
				{
					i32_landingTimer=-TASK_PERIOD_MS;
					e_state=DURING;
				}
				// DURING
				i32_landingTimer+=TASK_PERIOD_MS;
				StateLanding(x_faultEventBits);

				// EXIT
				if(i32_landingTimer>=LANDING_TIME_MS)
				{
					ge_flight_state=RESTING;
					e_state=ENTRY;
				}
				else if(IsFlyingRequired() && IsFlyingPossible(x_faultEventBits))
				{
					ge_flight_state=FLYING;
					e_state=ENTRY;
				}
				break;
			}
		}
	}
}

/**
 * \brief	executed in state FLYING at DURING time.
 * \param	x_faultEventBits	the fault eventBits
 */
static void StateFlying(EventBits_t x_faultEventBits)
{
	ReceiverTask_GetSetPoints(&gf_flight_setPoint[0]);

	// throttle back but with flight stabilization
	if ( fault_MOTOR & x_faultEventBits )
		if(gf_flight_setPoint[flight_THROTTLE]>MOTOR_OVERCURRENT_MAX_THROTTLE)
			gf_flight_setPoint[flight_THROTTLE]=MOTOR_OVERCURRENT_MAX_THROTTLE;

	Sensor_ReadAndFusion();

	Control_FlightStabilisation();
	Control_Mixer();

	Motor_OutputAll();



}

/**
 * \brief	executed in state LANDUNG at DURING time.
 * \param	x_faultEventBits	the fault eventBits
 */
static void StateLanding(EventBits_t x_faultEventBits)
{
	ReceiverTask_GetSetPoints(&gf_flight_setPoint[0]);
	gf_flight_setPoint[flight_THROTTLE]=THROTTLE_LANDING;	// const throttle while landing

	//
	// !!! elseif statements are sorted by the degree of emergency, so any combination of emergency will be handled correctly
	//

	// stop all motors
	if ( fault_MOTOR & x_faultEventBits )
	{
		Motor_StopAll();
	}

	// blind landing without flight stabilization
	else if ( fault_SENSOR & x_faultEventBits )
	{
		Control_MotorSameSetPoint(MOTOR_SET_POINT_SENSOR_FAULT);
		Motor_OutputAll();
	}

	// blind landing with flight stabilization
	else // when landing is required or when the current input signal for flight stab. has a fault
	{
		gf_flight_setPoint[flight_ROLL]		= 0.0f;
		gf_flight_setPoint[flight_PITCH]	= 0.0f;
		gf_flight_setPoint[flight_YAW]		= 0.0f;

		Sensor_ReadAndFusion();
		Control_FlightStabilisation();
		Control_Mixer();
		Motor_OutputAll();

	}

}

/**
 * \brief	executed in state RESTING at DURING time.
 */
static void StateResting(void)
{
    // TODO check if added part in if works
    if(Sensor_IsCalibrateRequired()){

       Sensor_Calibrate(TASK_PERIOD_MS);

       if(Sensor_IsCalibrateReady()){
           Sensor_CalibrateStop();
       }
    }
    else
    {
        //TODO delete later besides Sensor_ReadFusion just for pID lib tests
        //ReceiverTask_GetSetPoints(&gf_flight_setPoint[0]);
        Sensor_ReadAndFusion();

        //Control_FlightStabilisation();
        //Control_Mixer();
    }
    vTaskDelay(0.4 / portTICK_PERIOD_MS);
	Motor_StopAll();
}

/**
 * \brief	Test if transition to FLYING is required
 *
 *			(hysterese effect with IsFlyingNotRequired)
 * \return	true if flying is required,
 *			false else
 */
static uint8_t IsFlyingRequired(void)
{
	return ( gf_receiver_setPoint[flight_THROTTLE] >= THRESHOLD_FLYING_REQUIRED );
}

/**
 * \brief	Test if flying is not required
 *
 *			(hysterese effect with IsFlyingRequired)
 * \return	true if flying is not required,
 *			false else
 */
static uint8_t IsFlyingNotRequired(void)
{
	return ( gf_receiver_setPoint[flight_THROTTLE] <= THRESHOLD_FLYING_NOT_REQUIRED );
}

/**
 * \brief	Test if transition to FLYING is possible
 * \param	x_faultEventBits	the fault eventBits
 * \return	true if flying is possible,
 *			false else
 */
static uint8_t IsFlyingPossible(EventBits_t x_faultEventBits)
{
	return (((fault_MOTOR|fault_SENSOR|gui32_receiver_flightStabInput) & x_faultEventBits) == 0 );
}



