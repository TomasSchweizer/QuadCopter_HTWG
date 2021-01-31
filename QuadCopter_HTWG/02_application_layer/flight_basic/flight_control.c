/**
 * 		@file 	flight_control.c
 * 		@brief	flight control algorithm, and mixer functions
 *//*	@author Tobias Grimm
 * 		@date 	13.05.2016	(last modified)
 */

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <stdint.h>
#include <stdbool.h>

// drivers
#include "sensor_driver.h"
#include "motor_driver.h"
#include "debug_interface.h"
#include "display_driver.h"

// utils
#include "qc_math.h"

// application
#include "flight_task.h"		// used for gi32_flight_setPoint

// setup
#include "qc_setup.h"

/* ------------------------------------------------------------ */
/*				Local Defines									*/
/* ------------------------------------------------------------ */

// pid values for rate controller
#define PID_P_RATE 							0.25f // 0.18 good behaviour now sweetspot between 0.17 -> 0.23
#define PID_I_RATE 							0.0f
#define PID_D_RATE 							0.0f

// pid values for angle controller
#define PID_P                               1.0f
#define PID_I                               0.0f
#define PID_D                               0.0f

// variables for thrust calculation
#define THR_SCALE						    1.0f		// [%]
#define THR_BASE                            0.5f
#define dt   						        0.002f		// [s]
#define YAW_MAX_INF						    (1.0f/(15.0f*M_PI/180.0f))
#define MIX(X,Y,Z) 						    (f_thrustBase + f_pidRateOut[flight_ROLL]*X + f_pidRateOut[flight_PITCH]*Y + f_pidRateOut[flight_YAW]*Z)

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */
extern float gf_flight_setPoint[4];
extern float gf_sensor_fusedAngles[3];
extern float gf_sensor_angularVelocity[3];
extern float gf_sensor_gyroAngularVelocity[3];
extern volatile motor_Data_s gs_motor[motor_COUNT];

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */
static float f_pidLastInputs[4] = {0.0, 0.0, 0.0, 0.0};
static float f_pidOutputSum[4] = {0.0, 0.0, 0.0, 0.0};
static float f_pidOut[4] = {0.0, 0.0, 0.0, 0.0} ;

static float f_pidRateLastInputs[3] = {0.0, 0.0, 0.0};
static float f_pidRateOutputSum[3] = {0.0, 0.0, 0.0};
static float f_pidRateOut[3] = {0.0, 0.0, 0.0} ;


static float f_thrustBase = 0.0;

static math_pidController_s s_pidRateRoll = {

    .pf_input       =   &gf_sensor_gyroAngularVelocity[flight_ROLL],
    .pf_output      =   &f_pidRateOut[flight_ROLL],
    .pf_setPoint    =   &gf_flight_setPoint[flight_ROLL], /*&f_pidOut[flight_ROLL],*/
    .pf_lastInput   =   &f_pidRateLastInputs[flight_ROLL],
    .pf_outputSum   =   &f_pidRateOutputSum[flight_ROLL],
    .f_kP           =   PID_P_RATE,
    .f_kI           =   PID_I_RATE * dt,
    .f_kD           =   PID_D_RATE / dt,
    .cf_maxOut      =   0.2f,
    .cf_minOut      =   -0.2f,
    .cf_sampleTime  =   dt
};

static math_pidController_s s_pidRatePitch = {

    .pf_input       =   &gf_sensor_gyroAngularVelocity[flight_PITCH],
    .pf_output      =   &f_pidRateOut[flight_PITCH],
    .pf_setPoint    =   &gf_flight_setPoint[flight_PITCH], /*&f_pidOut[flight_PITCH],*/
    .pf_lastInput   =   &f_pidRateLastInputs[flight_PITCH],
    .pf_outputSum   =   &f_pidRateOutputSum[flight_PITCH],
    .f_kP           =   PID_P_RATE,
    .f_kI           =   PID_I_RATE * dt,
    .f_kD           =   PID_D_RATE / dt,
    .cf_maxOut      =   0.2f,
    .cf_minOut      =   -0.2f,
    .cf_sampleTime  =   dt
};

static math_pidController_s s_pidRateYaw = {

    .pf_input       =   &gf_sensor_gyroAngularVelocity[flight_YAW],
    .pf_output      =   &f_pidRateOut[flight_YAW],
    .pf_setPoint    =   &gf_flight_setPoint[flight_YAW],
    .pf_lastInput   =   &f_pidRateLastInputs[flight_YAW],
    .pf_outputSum   =   &f_pidRateOutputSum[flight_YAW],
    .f_kP           =   PID_P_RATE,
    .f_kI           =   PID_I_RATE * dt,
    .f_kD           =   PID_D_RATE / dt,
    .cf_maxOut      =   0.2f,
    .cf_minOut      =   -0.2f,
    .cf_sampleTime  =   dt
};

static math_pidController_s s_pidRoll = {

    .pf_input       =   &gf_sensor_fusedAngles[flight_ROLL],
    .pf_output      =   &f_pidOut[flight_ROLL],
    .pf_setPoint    =   &gf_flight_setPoint[flight_ROLL],
    .pf_lastInput   =   &f_pidLastInputs[flight_ROLL],
    .pf_outputSum   =   &f_pidOutputSum[flight_ROLL],
    .f_kP           =   PID_P,
    .f_kI           =   PID_I * dt,
    .f_kD           =   PID_D / dt,
    .cf_maxOut      =   2.0f,
    .cf_minOut      =   -2.0f,
    .cf_sampleTime  =   dt
};


static math_pidController_s s_pidPitch = {

    .pf_input       =   &gf_sensor_fusedAngles[flight_PITCH],
    .pf_output      =   &f_pidOut[flight_PITCH],
    .pf_setPoint    =   &gf_flight_setPoint[flight_PITCH],
    .pf_lastInput   =   &f_pidLastInputs[flight_PITCH],
    .pf_outputSum   =   &f_pidOutputSum[flight_PITCH],
    .f_kP           =   PID_P,
    .f_kI           =   PID_I * dt,
    .f_kD           =   PID_D / dt,
    .cf_maxOut      =   2.0f,
    .cf_minOut      =   -2.0f,
    .cf_sampleTime  =   dt
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

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

#if( setup_DEV_PID_TUNE ) || DOXYGEN
	union {
		float f[3];
		uint8_t ui8[12];
	} pid_values;

	/** TODO outdated
	 * \brief	Function for receiving pid parameters via uart
	 * \note	to enable this HIDE function set setup_DEV_PID_TUNE in qc_setup.h
	 */
	void HIDE_Control_DebugGetPid(void)
	{
		int32_t temp_received=0;
		volatile uint32_t i = 0;


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

		// when all is received, write values
		if (i==12){
			i = 0;
			s_pidRoll.f_kP 	= math_RAD2NORM(pid_values.f[flight_ROLL]);
			s_pidPitch.f_kP = math_RAD2NORM(pid_values.f[flight_PITCH]);
			s_pidRoll.f_kI 	= pid_values.f[flight_ROLL] * dt;
			s_pidPitch.f_kI = pid_values.f[flight_PITCH] * dt;
			s_pidRoll.f_kD 	= pid_values.f[flight_ROLL] / dt;
			s_pidPitch.f_kD = pid_values.f[flight_PITCH] / dt;
		}
	}

	void HIDE_Control_Debug_USB_GetPID(void)
	{

	    uint8_t pid_temp[14];

	    HIDE_Debug_USB_InterfaceReceive(pid_temp);

	    if(pid_temp[0] == 115){

            int i;
            for(i = 0; i < 12; i++){

                pid_values.ui8[i] = pid_temp[i+2];
            }

            // To Limit insane PID tune values TODO maybe adjust later
            for(i = 0; i < 3; i++){

                pid_values.f[i] = math_LIMIT(pid_values.f[i], 0.0, 1.0);

            }

            if(pid_temp[1] == 'r'){
                s_pidRateRoll.f_kP = pid_values.f[0];
                s_pidRateRoll.f_kI  = pid_values.f[1] * dt;
                s_pidRateRoll.f_kD  = pid_values.f[2] / dt;
                s_pidRatePitch.f_kP = pid_values.f[0];
                s_pidRatePitch.f_kI  = pid_values.f[1] * dt;
                s_pidRatePitch.f_kD  = pid_values.f[2] / dt;
            }
            else if(pid_temp[1] == 'y'){
                s_pidRateYaw.f_kP = pid_values.f[0];
                s_pidRateYaw.f_kI  = pid_values.f[1] * dt;
                s_pidRateYaw.f_kD  = pid_values.f[2] / dt;
            }
            else{

                // wrong USB values
                s_pidRateRoll.f_kP = 0.0;
                s_pidRateRoll.f_kI  = 0.0;
                s_pidRateRoll.f_kD  = 0.0;
                s_pidRatePitch.f_kP = 0.0;
                s_pidRatePitch.f_kI  = 0.0;
                s_pidRatePitch.f_kD  = 0.0;
                s_pidRateYaw.f_kP = 0.0;
                s_pidRateYaw.f_kI  = 0.0;
                s_pidRateYaw.f_kD  = 0.0;

            }
	    }

	}

    void HIDE_Control_PID_TUNE_DrawDisplay(void)
    {
        const uint8_t yOffset   = 32;
        const uint8_t xOffset   = 58;

        u8g_SetFont(&gs_display, u8g_font_04b_03r);     // u8g_font_unifont

        // Roll PID controller
        u8g_DrawStr(&gs_display,xOffset,      yOffset + 0,"Ro:");
        u8g_DrawStr(&gs_display,xOffset+14,      yOffset + 0,"P:");
        u8g_DrawStr(&gs_display,xOffset+18,   yOffset + 0,u8g_8toa((int8_t) (s_pidRateRoll.f_kP*100.0),2));
        u8g_DrawStr(&gs_display,xOffset+32,      yOffset + 0,"I:");
        u8g_DrawStr(&gs_display,xOffset+36,   yOffset + 0,u8g_8toa((int8_t) (s_pidRateRoll.f_kI*100.0),2));
        u8g_DrawStr(&gs_display,xOffset+52,      yOffset + 0,"D:");
        u8g_DrawStr(&gs_display,xOffset+56,   yOffset + 0,u8g_8toa((int8_t) (s_pidRateRoll.f_kD*100.0),2));

        // Pitch PID controller
        u8g_DrawStr(&gs_display,xOffset,      yOffset + 6,"Pi:");
        u8g_DrawStr(&gs_display,xOffset+14,      yOffset + 6,"P:");
        u8g_DrawStr(&gs_display,xOffset+18,   yOffset + 6,u8g_8toa((int8_t) (s_pidRatePitch.f_kP*100.0),2));
        u8g_DrawStr(&gs_display,xOffset+32,      yOffset + 6,"I:");
        u8g_DrawStr(&gs_display,xOffset+36,   yOffset + 6,u8g_8toa((int8_t) (s_pidRatePitch.f_kI*100.0),2));
        u8g_DrawStr(&gs_display,xOffset+52,      yOffset + 6,"D:");
        u8g_DrawStr(&gs_display,xOffset+56,   yOffset + 6,u8g_8toa((int8_t) (s_pidRatePitch.f_kD*100.0),2));

        // Yaw PID controller
        u8g_DrawStr(&gs_display,xOffset,      yOffset + 12,"Ya:");
        u8g_DrawStr(&gs_display,xOffset+14,      yOffset + 12,"P:");
        u8g_DrawStr(&gs_display,xOffset+18,   yOffset + 12,u8g_8toa((int8_t) (s_pidRateYaw.f_kP*100.0),2));
        u8g_DrawStr(&gs_display,xOffset+32,      yOffset + 12,"I:");
        u8g_DrawStr(&gs_display,xOffset+36,   yOffset + 12,u8g_8toa((int8_t) (s_pidRateYaw.f_kI*100.0),2));
        u8g_DrawStr(&gs_display,xOffset+52,      yOffset + 12,"D:");
        u8g_DrawStr(&gs_display,xOffset+56,   yOffset + 12,u8g_8toa((int8_t) (s_pidRateYaw.f_kD*100.0),2));

    }
#endif

#if (setup_DEBUG_USB)
    static float f_usb_debug[9];

    void HIDE_Control_SendDataOverUSB(void)
    {
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
 * \brief	Reset output & internal states of the flight Stabilisation.
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
 * \brief	Do control algorithm to stabilisate the QC
 */


void Control_FlightStabilisation(void)
{


    //HIDE_Control_Debug_USB_GetPID();

    // gf_flight_setPoint[flight_THROTTLE] = [0 -> 1]
    f_thrustBase = (THR_BASE + gf_flight_setPoint[flight_THROTTLE] * THR_BASE) * THR_SCALE;

    //Math_StepPidController(&s_pidRoll);
    //Math_StepPidController(&s_pidPitch);

    Math_StepPidController(&s_pidRateRoll);
    Math_StepPidController(&s_pidRatePitch);
    Math_StepPidController(&s_pidRateYaw);



}

/**
 * \brief	Mix for 4 Motor X-QuadCopter
 *
 * 			and write set points into gs_motor
 */
void Control_Mixer(void)
{
	uint8_t i;
	float motor[4];
	float motorSaturationValue[4] = {0.0, 0.0, 0.0, 0.0};
	float motorPositiveSaturationMaxValue = 0.0;
	float motorNegativeSaturationMaxValue = 0.0;

	//				R      P     Y
	motor[0] = MIX(+1.0f,+1.0f,-1.0f); //Motor VL
	motor[1] = MIX(+1.0f,-1.0f,+1.0f); //Motor HL
	motor[2] = MIX(-1.0f,+1.0f,+1.0f); //Motor VR
	motor[3] = MIX(-1.0f,-1.0f,-1.0f); //Motor HR

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
	for(i=0;i<motor_COUNT;++i)
	{


        motor[i] = motor[i] - motorPositiveSaturationMaxValue + motorNegativeSaturationMaxValue;
		motor[i]=math_LIMIT(motor[i],0.1f,1.0f);
		gs_motor[i].ui16_setPoint=(uint16_t)(motor[i]*0xFFFF);
	}


	// TODO delete later just for debug
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
//
//
//    HIDE_Debug_USB_InterfaceSend(f_usb_debug, sizeof(f_usb_debug)/ sizeof(f_usb_debug[0]), debug_FLOAT);



}

/**
 * \brief	Mix for 4 Motor X-QuadCopter
 *			(only give the throttle out)
 *
 *			and write set points into gs_motor
 */
void Control_MixerPassThrottle(void)
{
	uint8_t i;

	for(i=0;i<4;++i)
		gs_motor[i].ui16_setPoint=(uint16_t)((gf_flight_setPoint[flight_THROTTLE]*THR_SCALE)*0xFFFF);
}

/**
 * \brief	write the same set point for all motors (in gs_motor)
 * \param	ui16_setPoint	set point value
 */
void Control_MotorSameSetPoint(uint16_t ui16_setPoint)
{
	uint16_t i=0;
	for(i = 0; i< motor_COUNT; i++)
		gs_motor[i].ui16_setPoint = ui16_setPoint;

}

