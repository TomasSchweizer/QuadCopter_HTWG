/*===================================================================================================*/
/*  flight_task.c                                                                                    */
/*===================================================================================================*/

/*
*   file flight_task.c
*
*   brief Task for the flight stabilization. Implemented as state machine
*          with the states RESTING, FLYING and LANDING
*
*   details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>01/06/2016      <td>Tobias Grimm        <td>Implementation & Last modification of MAs
*   <tr><td>28/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   <tr><td>29/01/2021      <td>Tomas Schweizer     <td>Implementation of control algorithm
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
#include "flight_control.h"
#include "flight_task.h"
#include "receiver_task.h"      // GetSetPoints

// Drivers
#include "motor_driver.h"
#include "sensor_driver.h"
#include "display_driver.h"
#include "debug_interface.h"

//  FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

// Utilities
#include "workload.h"
#include "fault.h"
#include "qc_math.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Defines                                                 */
/* ---------------------------------------------------------------------------------------------------*/

#define TASK_STACK_SIZE        	 		( 250 )       ///< Task stack size in words
#define TASK_PERIOD_MS			 		( 2 )         ///< Task period in ms

#define LANDING_TIME_MS					( 5000 )	  ///< Time for the transition of the states LANDING -> RESTING

// Throttle from receiver goes from -1 to 1
#define MOTOR_OVERCURRENT_MAX_THROTTLE	(  0.0f )	  ///< If motor overcurrent is detected throttle is set to hover !!!NOT IMPLEMENTED!!!
#define THROTTLE_LANDING				( -0.6f)	  ///< If state landing is chosen throttle is set to ~20% !!!NOT IMPLEMENTED!!!
#define THRESHOLD_FLYING_REQUIRED		( -0.6f )	  ///< If throttle is over -0.6  flying is required
#define THRESHOLD_FLYING_NOT_REQUIRED	( -0.7f )	  ///< If throttle is under -0.7 flying is not longer required
#define MOTOR_SET_POINT_SENSOR_FAULT	((uint16_t)(0.25f*0xFFFF)) ///< If a sensor fault happened while flying the setpoint for the motors is set to 0.25 for all motors

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Type Definitions                                        */
/* ---------------------------------------------------------------------------------------------------*/
/// Transition states for flight state machine
enum transitionState_e
{
	ENTRY,		///< Transition entry for comming state, transition exit for current state
	DURING		///< During state execution
};

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Forward Declarations                                          */
/* ---------------------------------------------------------------------------------------------------*/
static void    FlightTask(void *pvParameters);
static void    StateFlying(EventBits_t x_faultEventBits);
static void    StateLanding(EventBits_t x_faultEventBits);
static uint8_t IsFlyingRequired(void);
static uint8_t IsFlyingNotRequired(void);
static uint8_t IsFlyingPossible(EventBits_t x_faultEventBits);
static void    StateResting(void);

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

// Extern global variables
extern volatile uint32_t 		    gui32_receiver_flightStabInput;         // Input mode (receiver/telemetrie...) !!!Only receiver implemented!!!
extern volatile EventGroupHandle_t  gx_fault_EventGroup;                    // Event group for faults

/**
 * @brief	Set point values for flight stabilization.
 *
 * @details
 * 			flight_ROLL 	[rad],\n
 * 			flight_PITCH	[rad],\n
 * 			flight_YAW		[rad/s],\n
 * 			flight_THROTTLE [-1 ... 1]
 *
 * @note	Write access:	flight_task
 */
float gf_flight_setPoint[4];

/**
 * @brief	Current state of the flight_task (see flight_task.h)
 * @note	Write access:	flight_task
 */
enum flight_state_e ge_flight_state=RESTING;

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Variables                                               */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Procedure Definitions                                         */
/* ---------------------------------------------------------------------------------------------------*/

/**
 * @brief	Draw info about flight task on the Display
 *
 * @return  void
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
 * @brief	Initializes peripherals needed for flight task and starts the task
 *
 * @return	0 --> Task creation was successful\n
 *			1 --> Task creation was not successful
 */
uint32_t FlightTask_Init(void)
{
	HIDE_Display_InsertDrawFun(FlightTaskDrawDisplay);
	Sensor_InitPeriph();
	HIDE_Display_InsertDrawFun(Sensor_DrawDisplay);
	HIDE_Debug_USB_InsertComFun(HIDE_Sensor_SendDataOverUSB, 0);
    Motor_InitPeriph();
	HIDE_Display_InsertDrawFun(Motor_DrawDisplay);


	// Insert PID tune functions in command task
    #if( setup_DEV_PID_TUNE )
	HIDE_Debug_USB_InsertComFun(HIDE_Control_Debug_USB_GetPID, 0);
	HIDE_Debug_USB_InsertComFun(HIDE_Control_SendDataOverUSB, 0);
	//HIDE_Display_InsertDrawFun(HIDE_Control_PID_TUNE_DrawDisplay);
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
 * @brief	Initializes motors/sensors, Then goes through state machine.
 *
 * @details
 *
 * Procedure:
 *
 * The flight task first initializes Motor and Sensor peripherals, after that it is called every
 * 2ms. If the flight state is RESTING the function StateResting is executed, if the state is Flying
 * the function StateFlying is executed. And in the state LANDING the function StateLanding is executed.
 *
 * @param	pvParameters --> Not used
 *
 * @return  void
 */
static void FlightTask(void *pvParameters)
{
    // Delay that the ESCs can power up
    vTaskDelay( 50 / portTICK_PERIOD_MS );
	Motor_InitMotor();
	// Delay 1ms just for safety that motor is initialized before the sensors get initialized
	vTaskDelay( 1/ portTICK_PERIOD_MS );
	Sensor_InitSensor();

	TickType_t  x_lastWakeTime;
	EventBits_t x_faultEventBits;

	//  Initialize the x_lastWakeTime variable with the current time.
	x_lastWakeTime = xTaskGetTickCount();

	// Timer for the transition of the states LANDING -> RESTING
	int32_t i32_landingTimer;

	enum transitionState_e e_state=ENTRY;

	// Loop forever
	while(1)
	{
		//  Wait for the next cycle.
		vTaskDelayUntil( &x_lastWakeTime, TASK_PERIOD_MS / portTICK_PERIOD_MS );

		// Get the faultEventBits to decide the emergency mode
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
 * @brief	Executed in state FLYING at DURING time. Is responsible for flight control
 *
 * @details
 *
 * Procedure:
 *
 * 1. Copies the setpoint from receiver
 * 2. Reads sensors and calculate values for control algorithm
 * 3. Calculate control algorithm
 * 4. Caculate motor setpoints
 * 5. Send motor setpoints to the ESCs
 *
 * @param	x_faultEventBits --> Fault eventBits
 *
 * @return  void
 */
static void StateFlying(EventBits_t x_faultEventBits)
{
    // Copies the receiver setpoints
	ReceiverTask_GetSetPoints(&gf_flight_setPoint[0]);

	// Throttle back but with flight stabilization if there is an overcurrent !!!NOT IMPLEMENTED!!!
	if ( fault_MOTOR & x_faultEventBits )
		if(gf_flight_setPoint[flight_THROTTLE]>MOTOR_OVERCURRENT_MAX_THROTTLE)
			gf_flight_setPoint[flight_THROTTLE]=MOTOR_OVERCURRENT_MAX_THROTTLE;

	// Reads all Sensors and calculates the necessary variables for the control algorithm
	Sensor_ReadAndFusion();

	// Calculates the control algorithm
	Control_FlightStabilisation();
	// Calculates the motor setpoints
	Control_Mixer();
	// Sends the motor setpoints to the ESCs
	Motor_OutputAll();
}

/**
 * @brief	Executed in state LANDING at DURING time.
 *
 * @note !!!NOT IMPLEMENTED!!!
 *
 * @param	x_faultEventBits --> Fault eventBits
 *
 * @return  void
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
 * @brief	Executed in state RESTING at DURING time. Sensor calibration is applied if needed
 *
 * @details
 *
 * Procedure:
 *
 * 1. Check if sensor calibration is required
 * 2. If yes then calibrate sensors for 5s
 * 3. If no then just read sensors and apply sensor fusion
 * 4. Send 0 as setpoint to all ESCs/motors
 *
 * @return void
 */
static void StateResting(void)
{
    // Check if sensor calibration is required
    if(Sensor_IsCalibrateRequired()){

       Sensor_Calibrate(TASK_PERIOD_MS);

       if(Sensor_IsCalibrateReady()){
           Sensor_CalibrateStop();
       }
    }
    else
    {
        //TODO delete later besides Sensor_ReadFusion just for tests
        ReceiverTask_GetSetPoints(&gf_flight_setPoint[0]);
        Sensor_ReadAndFusion();

        Control_FlightStabilisation();
        Control_Mixer();
    }

	Motor_StopAll();
}

/**
 * @brief	Test if transition to FLYING is required
 *
 * @note    Hysterese effect with IsFlyingNotRequired
 *
 * @return	true --> Flying is required\n
 *			false --> Flying is not required
 */
static uint8_t IsFlyingRequired(void)
{
	return ( gf_receiver_setPoint[flight_THROTTLE] >= THRESHOLD_FLYING_REQUIRED );
}

/**
 * @brief	Test if flying is not required
 *
 * @note	Hysterese effect with IsFlyingRequired
 *
 * @return	true --> Flying is not required\n
 *			false --> Flying is required
 */
static uint8_t IsFlyingNotRequired(void)
{
	return ( gf_receiver_setPoint[flight_THROTTLE] <= THRESHOLD_FLYING_NOT_REQUIRED );
}

/**
 * @brief	Test if transition to FLYING is possible
 *
 * @param	x_faultEventBits	Fault eventBits
 *
 * @return	true --> Flying is possible\n
 *			false --> Flying is not possible\n
 */
static uint8_t IsFlyingPossible(EventBits_t x_faultEventBits)
{
	return (((fault_MOTOR|fault_SENSOR|gui32_receiver_flightStabInput) & x_faultEventBits) == 0 );
}

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
