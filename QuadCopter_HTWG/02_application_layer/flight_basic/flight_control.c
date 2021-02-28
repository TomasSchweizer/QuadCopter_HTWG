/*===================================================================================================*/
/*  flight_control.c                                                                                 */
/*===================================================================================================*/

/*
*   file   flight_control.c
*
*   brief  Flight control algorithm and motor mixing.
*
*   details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>13/05/2016      <td>Tobias Grimm        <td>Implementation & Last modification of MAs
*   <tr><td>20/01/2020      <td>Tomas Schweizer     <td>Complete change of flight control algorithm
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

// Application
#include "flight_task.h"        // used for gi32_flight_setPoint

// Drivers
#include "sensor_driver.h"
#include "motor_driver.h"
#include "debug_interface.h"
#include "display_driver.h"

// Utilities
#include "qc_math.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Defines                                                 */
/* ---------------------------------------------------------------------------------------------------*/

// pid values for pitch&roll rate controller
#define PID_P_RATE 							0.241f              ///< P value for roll & pitch rate controller
#define PID_I_RATE 							0.0f                ///< I value for roll & pitch rate controller
#define PID_D_RATE 							0.0f                ///< D value for roll & pitch rate controller

// pid values for yaw rate controller
#define PID_P_YRATE                         0.5f                ///< P value for yaw rate controller
#define PID_I_YRATE                         0.001f             ///< I value for yaw rate controller
#define PID_D_YRATE                         0.0f                ///< D value for yaw rate controller


// pid values for angle controller
#define PID_P                               2.5f                ///< P value for roll & pitch angle controller
#define PID_I                               0.0001f             ///< I value for roll & pitch angle controller
#define PID_D                               0.0f                ///< D value for roll & pitch angle controller

// variables for thrust calculation
#define THR_SCALE						    1.0f		        ///< Thrust scale factor, can be used to limited max throttle [%]
#define THR_BASE                            0.5f                ///< Thrust base is used to get hoverpoint at around 50% thrust
#define dt   						        0.002f		        ///< Loop time [s]

// Motor mix takes the thrust base and the rate PIDs output values and then adds them up for the motor commands
#define MIX(X,Y,Z) 						    (f_thrustBase + f_pidRateOut[flight_ROLL]*X + f_pidRateOut[flight_PITCH]*Y + f_pidRateOut[flight_YAW]*Z)    ///< Motor mixing macro

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Type Definitions                                        */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Forward Declarations                                          */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/
extern float gf_flight_setPoint[4];                         // Extern global variable from flight_task used to get set point
extern float gf_sensor_fusedAngles[3];                      // Extern global variable from sensor_driver used to get angles of QC
extern float gf_sensor_angularVelocity[3];                  // Extern global variable from sensor_driver used to get angular velocity (quaternion) of QC
extern float gf_sensor_gyroAngularVelocity[3];              // Extern global variable from sensor_driver used to get angular velocity (gyro) of QC
extern volatile motor_Data_s gs_motor[motor_COUNT];         // Extern global struct from motor_driver used to set the motor setpoints

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Variables                                               */
/* ---------------------------------------------------------------------------------------------------*/
static float f_pidLastInputs[4] = {0.0, 0.0, 0.0, 0.0};     ///< For PID angle controller calculation of D-Term
static float f_pidOutputSum[4] = {0.0, 0.0, 0.0, 0.0};      ///< For PID angle controller calculation of I-Term
static float f_pidOut[4] = {0.0, 0.0, 0.0, 0.0};            ///< Output for PID angle controllers is saved here

static float f_pidRateLastInputs[3] = {0.0, 0.0, 0.0};      ///< For PID rate controller calculation of D-Term
static float f_pidRateOutputSum[3] = {0.0, 0.0, 0.0};       ///< For PID rate controller calculation of I-Term
static float f_pidRateOut[3] = {0.0, 0.0, 0.0} ;            ///< Output for PID rate controller is saved here goes into the motor mixing algorithm


static float f_thrustBase = 0.0;                            ///< Thrust base for motor mixing

/// Rate controller for roll axis acts as D-Term of roll angle controller
static math_pidController_s s_pidRateRoll = {

    .pf_input       =   &gf_sensor_gyroAngularVelocity[flight_ROLL],                    ///< Input for roll rate controller is the angular velocity of the gyro
    .pf_output      =   &f_pidRateOut[flight_ROLL],                                     ///< Output gets saved and then is used in motor mixing algorithm
    .pf_setPoint    =   /*&gf_flight_setPoint[flight_ROLL],*/&f_pidOut[flight_ROLL],    ///< Setpoint is the output of the roll angle controller
    .pf_lastInput   =   &f_pidRateLastInputs[flight_ROLL],                              ///< Saves last input for D-Term calculation
    .pf_outputSum   =   &f_pidRateOutputSum[flight_ROLL],                               ///< Saves the output sum for I-Term calculation
    .f_kP           =   PID_P_RATE,                                                     ///< P value for roll rate controller
    .f_kI           =   PID_I_RATE,                                                     ///< I value for roll rate controller -> normally zero
    .f_kD           =   PID_D_RATE,                                                     ///< D value for roll rate controller -> normally zero
    .cf_maxOut      =   0.2f,                                                           ///< Max limit for PID controller output [max +20% of motor power]
    .cf_minOut      =   -0.2f,                                                          ///< Min limit for PID controller output [min -20% of motor power]
    .cf_sampleTime  =   dt                                                              ///< Sample time [s]
};

/// Rate controller for pitch axis acts as D-Term of pitch angle controller
static math_pidController_s s_pidRatePitch = {

    .pf_input       =   &gf_sensor_gyroAngularVelocity[flight_PITCH],                   ///< Input for pitch rate controller is the angular velocity of the gyro
    .pf_output      =   &f_pidRateOut[flight_PITCH],                                    ///< Output gets saved and then is used in motor mixing algorithm
    .pf_setPoint    =   /*&gf_flight_setPoint[flight_PITCH],*/&f_pidOut[flight_PITCH],  ///< Setpoint is the output of the pitch angle controller
    .pf_lastInput   =   &f_pidRateLastInputs[flight_PITCH],                             ///< Saves last input for D-Term calculation
    .pf_outputSum   =   &f_pidRateOutputSum[flight_PITCH],                              ///< Saves the output sum for I-Term calculation
    .f_kP           =   PID_P_RATE,                                                     ///< P value for pitch rate controller
    .f_kI           =   PID_I_RATE,                                                     ///< I value for pitch rate controller -> normally zero
    .f_kD           =   PID_D_RATE,                                                     ///< D value for pitch rate controller -> normally zero
    .cf_maxOut      =   0.2f,                                                           ///< Max limit for PID controller output [max +20% of motor power]
    .cf_minOut      =   -0.2f,                                                          ///< Min limit for PID controller output [min -20% of motor power]
    .cf_sampleTime  =   dt                                                              ///< Sample time [s]
};

/// Rate controller for yaw axis has no outer PID acts directly from stick inputs
static math_pidController_s s_pidRateYaw = {

    .pf_input       =   &gf_sensor_gyroAngularVelocity[flight_YAW],                     ///< Input for yaw rate controller is the angular velocity of the gyro
    .pf_output      =   &f_pidRateOut[flight_YAW],                                      ///< Output gets saved and then is used in motor mixing algorithm
    .pf_setPoint    =   &gf_flight_setPoint[flight_YAW],                                ///< Setpoint is yaw rate command from the receiver
    .pf_lastInput   =   &f_pidRateLastInputs[flight_YAW],                               ///< Saves last input for D-Term calculation
    .pf_outputSum   =   &f_pidRateOutputSum[flight_YAW],                                ///< Saves the output sum for I-Term calculation
    .f_kP           =   PID_P_YRATE,                                                    ///< P value for yaw rate controller
    .f_kI           =   PID_I_YRATE,                                                    ///< I value for yaw rate controller -> normally zero
    .f_kD           =   PID_D_YRATE,                                                    ///< D value for yaw rate controller -> normally zero
    .cf_maxOut      =   0.2f,                                                           ///< Max limit for PID controller output [max +20% of motor power]
    .cf_minOut      =   -0.2f,                                                          ///< Min limit for PID controller output [min -20% of motor power]
    .cf_sampleTime  =   dt                                                              ///< Sample time [s]
};

/// Angle controller for the roll axis
static math_pidController_s s_pidRoll = {

    .pf_input       =   &gf_sensor_fusedAngles[flight_ROLL],                            ///< Input for the roll angle controller is the fused roll angle of the QC
    .pf_output      =   &f_pidOut[flight_ROLL],                                         ///< Output is saved and then used as input for the roll rate controller
    .pf_setPoint    =   &gf_flight_setPoint[flight_ROLL],                               ///< Setpoint is the roll angle command from the receiver
    .pf_lastInput   =   &f_pidLastInputs[flight_ROLL],                                  ///< Saves last input for D-Term calculation
    .pf_outputSum   =   &f_pidOutputSum[flight_ROLL],                                   ///< Saves the output sum for I-Term calculation
    .f_kP           =   PID_P,                                                          ///< P value for roll angle controller
    .f_kI           =   PID_I,                                                          ///< I value for roll angle controller -> really small x < 0.002 better for stable flight
    .f_kD           =   PID_D,                                                          ///< D value for roll angle controller -> normally zero because rate controllers "act as D-Term"
    .cf_maxOut      =   2.0f,                                                           ///< Max limit for PID controller output [max +2 rad/s]
    .cf_minOut      =   -2.0f,                                                          ///< Min limit for PID controller output [max -2 rad/s]
    .cf_sampleTime  =   dt                                                              ///< Sample time [s]
};

/// Angle controller for the pitch axis
static math_pidController_s s_pidPitch = {

    .pf_input       =   &gf_sensor_fusedAngles[flight_PITCH],                           ///< Input for the pitch angle controller is the fused pitch angle of the QC
    .pf_output      =   &f_pidOut[flight_PITCH],                                        ///< Output is saved and then used as input for the pitch rate controller
    .pf_setPoint    =   &gf_flight_setPoint[flight_PITCH],                              ///< Setpoint is the pitch angle command from the receiver
    .pf_lastInput   =   &f_pidLastInputs[flight_PITCH],                                 ///< Saves last input for D-Term calculation
    .pf_outputSum   =   &f_pidOutputSum[flight_PITCH],                                  ///< Saves the output sum for I-Term calculation
    .f_kP           =   PID_P,                                                          ///< P value for pitch angle controller
    .f_kI           =   PID_I,                                                          ///< I value for pitch angle controller -> really small x < 0.002 better for stable flight
    .f_kD           =   PID_D,                                                          ///< D value for pitch angle controller -> normally zero because rate controllers "act as D-Term"
    .cf_maxOut      =   2.0f,                                                           ///< Max limit for PID controller output [max +2 rad/s]
    .cf_minOut      =   -2.0f,                                                          ///< Min limit for PID controller output [max -2 rad/s]
    .cf_sampleTime  =   dt                                                              ///< Sample time [s]
};

//static math_pidController_s s_pidYaw = {
//
//    .pf_input       =   &gf_sensor_fusedAngles[flight_YAW],
//    .pf_output      =   &f_pidOut[flight_YAW],
//    .pf_setPoint    =   &gf_flight_setPoint[flight_YAW],
//    .pf_lastInput   =   &f_pidLastInputs[flight_YAW],
//    .pf_outputSum   =   &f_pidOutputSum[flight_YAW],
//    .f_kP           =   PID_P,
//    .f_kI           =   PID_I * dt,
//    .f_kD           =   PID_D / dt,
//    .cf_maxOut      =   0.1f,
//    .cf_minOut      =   -0.1f,
//    .cf_sampleTime  =   dt
//};

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Procedure Definitions                                         */
/* ---------------------------------------------------------------------------------------------------*/

// Only included if setup_DEV_PID_TUNE is activated in qc_setup.h
#if( setup_DEV_PID_TUNE ) || DOXYGEN

    /// Union to receive pid values over USB
    static union {
		float f[3];
		uint8_t ui8[12];
	} pid_values;

    /**
     * @brief	Function for receiving PID parameters via UART
     *
     * @details
     * Waits till 12 uint8_t values are received from the UART debug interface
     * then saves them into the union pid values and then sets the gains of the
     * controllers. Can be changed to the wanted controllers
     *
     * @return  void
     *
     * @note	!!!OUTDATED!!! new implementation over USB
     */
    void HIDE_Control_DebugGetPid(void)
    {
		int32_t temp_received=0;
		volatile uint32_t i = 0;

		// wait till 12 uint8_t values were received over uART
		while(i<12)
		{
			HIDE_Debug_InterfaceGet(& temp_received);
			if (temp_received != -1)
			{
				pid_values.ui8[i] = (uint8_t)temp_received;
				i++;
			}
			else
				break;

		}

		// Then use union to convert 12 uint8_t values to 3 float values and write them to the gain values of the controller
		if (i==12){
			i = 0;
			s_pidRoll.f_kP 	= math_RAD2NORM(pid_values.f[flight_ROLL]);
			s_pidPitch.f_kP = math_RAD2NORM(pid_values.f[flight_PITCH]);
			s_pidRoll.f_kI 	= pid_values.f[flight_ROLL];
			s_pidPitch.f_kI = pid_values.f[flight_PITCH];
			s_pidRoll.f_kD 	= pid_values.f[flight_ROLL];
			s_pidPitch.f_kD = pid_values.f[flight_PITCH];
		}
	}

    /**
    * @brief   Function for receiving PID parameters via USB
    *
    * @details
    * Uses the USB debug interface, checks if a 's'/115 is received which means the beginning of a transaction
    * Then saves the uint8_t values into the union, next it limits to big PID values (but just really high values
    * better check self). Then checks the second received value if it's a 'r' the roll and pitch controllers get
    * changed if its an 'y' the yaw controller gets changed. If its neither an 'r' or 'y' all gains get reset.
    *
    * @return   void
    */
	void HIDE_Control_Debug_USB_GetPID(void)
	{

	    uint8_t pid_temp[14];

	    // Check if something is received
	    HIDE_Debug_USB_InterfaceReceive(pid_temp);

	    // If value 0 equals 's'/115
	    if(pid_temp[0] == 115){

            int i;
            for(i = 0; i < 12; i++){

                pid_values.ui8[i] = pid_temp[i+2];
            }

            // To Limit insane PID tune values TODO maybe adjust later
            for(i = 0; i < 3; i++){

                pid_values.f[i] = math_LIMIT(pid_values.f[i], 0.0, 5.0);

            }

            // Change roll and pitch controller
            if(pid_temp[1] == 'r'){
                s_pidRoll.f_kP = pid_values.f[0];
                s_pidRoll.f_kI  = pid_values.f[1];
                s_pidRoll.f_kD  = pid_values.f[2];
                s_pidPitch.f_kP = pid_values.f[0];
                s_pidPitch.f_kI  = pid_values.f[1];
                s_pidPitch.f_kD  = pid_values.f[2];
            }
            // Change yaw controller
            else if(pid_temp[1] == 'y'){
                s_pidRateYaw.f_kP = pid_values.f[0];
                s_pidRateYaw.f_kI  = pid_values.f[1];
                s_pidRateYaw.f_kD  = pid_values.f[2];
            }
            // Mistake in USB reset controllers
            else{

                s_pidRoll.f_kP = 0.0;
                s_pidRoll.f_kI  = 0.0;
                s_pidRoll.f_kD  = 0.0;
                s_pidPitch.f_kP = 0.0;
                s_pidPitch.f_kI  = 0.0;
                s_pidPitch.f_kD  = 0.0;
                s_pidRateYaw.f_kP = 0.0;
                s_pidRateYaw.f_kI  = 0.0;
                s_pidRateYaw.f_kD  = 0.0;

            }
	    }

	}

	/**
    * @brief   Function to display PID values on the display for tuning
    *
    * @details
    * Must be changed to the controllers you want to see at the moment.
    *
    * @return   void
    */
    void HIDE_Control_PID_TUNE_DrawDisplay(void)
    {
        const uint8_t yOffset   = 32;
        const uint8_t xOffset   = 58;

        u8g_SetFont(&gs_display, u8g_font_04b_03r);     // u8g_font_unifont

        // Roll controller
        u8g_DrawStr(&gs_display,xOffset,      yOffset + 0,"Ro:");
        u8g_DrawStr(&gs_display,xOffset+14,      yOffset + 0,"P:");
        u8g_DrawStr(&gs_display,xOffset+18,   yOffset + 0,u8g_8toa((int8_t) (s_pidRateRoll.f_kP*100.0),2));
        u8g_DrawStr(&gs_display,xOffset+32,      yOffset + 0,"I:");
        u8g_DrawStr(&gs_display,xOffset+36,   yOffset + 0,u8g_8toa((int8_t) (s_pidRateRoll.f_kI*100.0),2));
        u8g_DrawStr(&gs_display,xOffset+52,      yOffset + 0,"D:");
        u8g_DrawStr(&gs_display,xOffset+56,   yOffset + 0,u8g_8toa((int8_t) (s_pidRateRoll.f_kD*100.0),2));

        // Pitch controller
        u8g_DrawStr(&gs_display,xOffset,      yOffset + 6,"Pi:");
        u8g_DrawStr(&gs_display,xOffset+14,      yOffset + 6,"P:");
        u8g_DrawStr(&gs_display,xOffset+18,   yOffset + 6,u8g_8toa((int8_t) (s_pidRatePitch.f_kP*100.0),2));
        u8g_DrawStr(&gs_display,xOffset+32,      yOffset + 6,"I:");
        u8g_DrawStr(&gs_display,xOffset+36,   yOffset + 6,u8g_8toa((int8_t) (s_pidRatePitch.f_kI*100.0),2));
        u8g_DrawStr(&gs_display,xOffset+52,      yOffset + 6,"D:");
        u8g_DrawStr(&gs_display,xOffset+56,   yOffset + 6,u8g_8toa((int8_t) (s_pidRatePitch.f_kD*100.0),2));

        // Yaw controller
        u8g_DrawStr(&gs_display,xOffset,      yOffset + 12,"Ya:");
        u8g_DrawStr(&gs_display,xOffset+14,      yOffset + 12,"P:");
        u8g_DrawStr(&gs_display,xOffset+18,   yOffset + 12,u8g_8toa((int8_t) (s_pidRateYaw.f_kP*100.0),2));
        u8g_DrawStr(&gs_display,xOffset+32,      yOffset + 12,"I:");
        u8g_DrawStr(&gs_display,xOffset+36,   yOffset + 12,u8g_8toa((int8_t) (s_pidRateYaw.f_kI*100.0),2));
        u8g_DrawStr(&gs_display,xOffset+52,      yOffset + 12,"D:");
        u8g_DrawStr(&gs_display,xOffset+56,   yOffset + 12,u8g_8toa((int8_t) (s_pidRateYaw.f_kD*100.0),2));

    }
#endif

// If setup_debug_USB is activated data can be send every 100ms over USB to PC application
#if (setup_DEBUG_USB) || DOXYGEN

    // TODO delete later
    static float f_usb_debug[9];

    /**
    * @brief   Function to send values to the PC-Application uses USB debug interface
    *
    * @details
    * Must be inserted into the update function of the command task
    *
    * @return   void
    */
    void HIDE_Control_SendDataOverUSB(void)
    {

        float f_usb_debug[9];

        f_usb_debug[0] = gf_flight_setPoint[0];
        f_usb_debug[1] = gf_flight_setPoint[1];
        f_usb_debug[2] = gf_flight_setPoint[2];
        f_usb_debug[3] = gf_flight_setPoint[3];
        f_usb_debug[4] = f_pidRateOut[2];
        f_usb_debug[5] = (float) gs_motor[0].ui16_setPoint;
        f_usb_debug[6] = (float) gs_motor[1].ui16_setPoint;
        f_usb_debug[7] = (float) gs_motor[2].ui16_setPoint;
        f_usb_debug[8] = (float) gs_motor[3].ui16_setPoint;

        HIDE_Debug_USB_InterfaceSend(f_usb_debug, sizeof(f_usb_debug)/ sizeof(f_usb_debug[0]), debug_FLOAT);
      }

#endif

/**
 * @brief	Reset output & internal states of the flight controllers.
 *
 * @details
 * Just a safety measure. Normally the control outputs should be initialized as with zeros.
 *
 * @return  void
 *
 */
void Control_Reset(void)
{
    uint8_t i;
	for(i=0; i<4; i++)
	{
	    f_pidLastInputs[i] = 0.0;
	    f_pidOutputSum [i] = 0.0;
	    f_pidOut[i] = 0.0;

	    f_pidRateLastInputs[i] = 0.0;
        f_pidRateOutputSum [i] = 0.0;
        f_pidRateOut[i] = 0.0;
	}
}



/**
 * @brief   Function which calculates the control algorithm to stabilize attitude of the QC
 *
 * @details
 *
 * Procedure:
 * 1. Calculates the thrust base based on the throttle setpoints
 * 2. Runs the PI angle controllers for roll and pitch axis
 * 3. Runs the P rate controller for roll, pitch and yaw axis
 *
 * @return  void
 *
 * @note    Math_StepPidController definition can be found under utilities/qc_math
 *
 */
void Control_FlightStabilisation(void)
{

    // Calculate the thrust base [(0.5 + {-1..1} * 0.5) * 1]
    f_thrustBase = (THR_BASE + gf_flight_setPoint[flight_THROTTLE] * THR_BASE) * THR_SCALE;

    // Calculate PI angle controllers for roll and pitch axis
    Math_StepPidController(&s_pidRoll);
    Math_StepPidController(&s_pidPitch);

    // Calculate P rate controllers for roll, pitch and yaw axis
    Math_StepPidController(&s_pidRateRoll);
    Math_StepPidController(&s_pidRatePitch);
    Math_StepPidController(&s_pidRateYaw);
}

/**
 * @brief   Motor mixing algorithm for 4X-QC with saturation limits
 *
 * @details
 *
 * Procedure:
 *
 * 1. Calculates motor mixing algorithm:\n
 *    M1 = Thrust + Roll + Pitch - Yaw\n
 *    M2 = Thrust + Roll - Pitch + Yaw\n
 *    M3 = Thrust - Roll + Pitch + Yaw\n
 *    M4 = Thrust - Roll - Pitch - Yaw \n
 * 2. Check if motor is in over or under saturation
 * 3. Subtract/Add highest/lowest saturation from all motors so that ratio
 *    betweem them stays the same
 * 4. Check limit of motors and write the setpoint for each motor in global variable
 *
 * @return  void
 *
 */
void Control_Mixer(void)
{
	uint8_t i;
	float motor[4];
	float motorSaturationValue[4] = {0.0, 0.0, 0.0, 0.0};
	float motorPositiveSaturationMaxValue = 0.0;
	float motorNegativeSaturationMaxValue = 0.0;

	// Apply motor mix see local defines
	//				R      P     Y
	motor[0] = MIX(+1.0f,+1.0f,-1.0f); //Motor VL
	motor[1] = MIX(+1.0f,-1.0f,+1.0f); //Motor HL
	motor[2] = MIX(-1.0f,+1.0f,+1.0f); //Motor VR
	motor[3] = MIX(-1.0f,-1.0f,-1.0f); //Motor HR

	// Check for saturation
	for(i=0;i<motor_COUNT;++i)
	{
	    if(motor[i] > 1.0)
	    {
	        motorSaturationValue[i] = motor[i] - 1.0;
	        motorPositiveSaturationMaxValue = math_MAX(motorSaturationValue[i], motorPositiveSaturationMaxValue);

	    } else if (motor[i] < 0.15)
	    {
	        motorSaturationValue[i] = 0.15 - motor[i];
	        motorNegativeSaturationMaxValue = math_MAX(motorSaturationValue[i], motorNegativeSaturationMaxValue);
	    }

	}
	// Write motor setpoints to global variable
	for(i=0;i<motor_COUNT;++i)
	{


        motor[i] = motor[i] - motorPositiveSaturationMaxValue + motorNegativeSaturationMaxValue;
		motor[i]=math_LIMIT(motor[i],0.1f,1.0f);
		gs_motor[i].ui16_setPoint=(uint16_t)(motor[i]*0xFFFF);
	}

//	// Copy data to send over usb into array
//    f_usb_debug[0] = gf_flight_setPoint[0];
//    f_usb_debug[1] = gf_sensor_fusedAngles[0];
//    f_usb_debug[2] = gf_sensor_gyroAngularVelocity[0];
//    f_usb_debug[3] = f_pidOut[0];
//    f_usb_debug[4] = f_pidRateOut[0];
//    f_usb_debug[5] = (float) gs_motor[0].ui16_setPoint;
//    f_usb_debug[6] = (float) gs_motor[1].ui16_setPoint;
//    f_usb_debug[7] = (float) gs_motor[2].ui16_setPoint;
//    f_usb_debug[8] = (float) gs_motor[3].ui16_setPoint;
//
//    // Send data per USB
//    HIDE_Debug_USB_InterfaceSend(f_usb_debug, sizeof(f_usb_debug)/ sizeof(f_usb_debug[0]), debug_FLOAT);
}

/**
 * @brief   Function to just write throttle to setpoints for the motors
 *
 * @return  void
 *
 * @note    Only development function, not used
 */
void Control_MixerPassThrottle(void)
{
	uint8_t i;

	for(i=0;i<4;++i)
		gs_motor[i].ui16_setPoint=(uint16_t)((gf_flight_setPoint[flight_THROTTLE]*THR_SCALE)*0xFFFF);
}

/**
 * @brief   Function to write the same setpoint to the motors
 *
 * @param	ui16_setPoint --> set point value for motors
 *
 * @return void
 */
void Control_MotorSameSetPoint(uint16_t ui16_setPoint)
{
	uint16_t i=0;
	for(i = 0; i< motor_COUNT; i++)
		gs_motor[i].ui16_setPoint = ui16_setPoint;

}

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
