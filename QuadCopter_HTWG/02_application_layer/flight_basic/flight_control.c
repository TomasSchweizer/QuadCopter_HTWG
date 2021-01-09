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

#define PID_P 							1.0f
#define PID_I 							0.0f
#define PID_D 							0.0f
#define THR_SCALE						0.8f		// [%]
#define dt   						    0.002f		// [s]
#define YAW_MAX_INF						(1.0f/(15.0f*M_PI/180.0f))
#define MIX(X,Y,Z) 						(f_thrustBase + f_pidRateOut[flight_ROLL]*X + f_pidRateOut[flight_PITCH]*Y + f_pidRateOut[flight_YAW]*Z)

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
extern volatile motor_Data_s gs_motor[motor_COUNT];

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */
static float f_pidLastInputs[4] = {0.0, 0.0, 0.0, 0.0};
static float f_pidOutputSum[4] = {0.0, 0.0, 0.0, 0.0};
static float f_pidOut[4] = {0.0, 0.0, 0.0, 0.0} ;

static float f_pidRateLastInputs[4] = {0.0, 0.0, 0.0, 0.0};
static float f_pidRateOutputSum[4] = {0.0, 0.0, 0.0, 0.0};
static float f_pidRateOut[4] = {0.0, 0.0, 0.0, 0.0} ;


static float f_thrustBase = 0.0;

static math_pidController_s s_pidRateRoll = {

    .pf_input       =   &gf_sensor_angularVelocity[flight_ROLL],
    .pf_output      =   &f_pidRateOut[flight_ROLL],
    .pf_setPoint    =   &gf_flight_setPoint[flight_ROLL],
    .pf_lastInput   =   &f_pidRateLastInputs[flight_ROLL],
    .pf_outputSum   =   &f_pidRateOutputSum[flight_ROLL],
    .f_kP           =   math_RAD2NORM(PID_P),
    .f_kI           =   PID_I * dt,
    .f_kD           =   PID_D / dt,
    .cf_maxOut      =   0.3f,
    .cf_minOut      =   -0.3f,
    .cf_sampleTime  =   dt
};

static math_pidController_s s_pidRatePitch = {

    .pf_input       =   &gf_sensor_angularVelocity[flight_PITCH],
    .pf_output      =   &f_pidRateOut[flight_PITCH],
    .pf_setPoint    =   &gf_flight_setPoint[flight_PITCH],
    .pf_lastInput   =   &f_pidRateLastInputs[flight_PITCH],
    .pf_outputSum   =   &f_pidRateOutputSum[flight_PITCH],
    .f_kP           =   math_RAD2NORM(PID_P),
    .f_kI           =   PID_I * dt,
    .f_kD           =   PID_D / dt,
    .cf_maxOut      =   0.3f,
    .cf_minOut      =   -0.3f,
    .cf_sampleTime  =   dt
};

static math_pidController_s s_pidRateYaw = {

    .pf_input       =   &gf_sensor_angularVelocity[flight_YAW],
    .pf_output      =   &f_pidRateOut[flight_YAW],
    .pf_setPoint    =   &gf_flight_setPoint[flight_YAW],
    .pf_lastInput   =   &f_pidRateLastInputs[flight_YAW],
    .pf_outputSum   =   &f_pidRateOutputSum[flight_YAW],
    .f_kP           =   math_RAD2NORM(PID_P),
    .f_kI           =   PID_I * dt,
    .f_kD           =   PID_D / dt,
    .cf_maxOut      =   0.3f,
    .cf_minOut      =   -0.3f,
    .cf_sampleTime  =   dt
};

static math_pidController_s s_pidRoll = {

    .pf_input       =   &gf_sensor_fusedAngles[flight_ROLL],
    .pf_output      =   &f_pidOut[flight_ROLL],
    .pf_setPoint    =   &gf_flight_setPoint[flight_ROLL],
    .pf_lastInput   =   &f_pidLastInputs[flight_ROLL],
    .pf_outputSum   =   &f_pidOutputSum[flight_ROLL],
    .f_kP           =   math_RAD2NORM(PID_P),
    .f_kI           =   PID_I * dt,
    .f_kD           =   PID_D / dt,
    .cf_maxOut      =   0.3f,
    .cf_minOut      =   -0.3f,
    .cf_sampleTime  =   dt
};


static math_pidController_s s_pidPitch = {

    .pf_input       =   &gf_sensor_fusedAngles[flight_PITCH],
    .pf_output      =   &f_pidOut[flight_PITCH],
    .pf_setPoint    =   &gf_flight_setPoint[flight_PITCH],
    .pf_lastInput   =   &f_pidLastInputs[flight_PITCH],
    .pf_outputSum   =   &f_pidOutputSum[flight_PITCH],
    .f_kP           =   math_RAD2NORM(PID_P),
    .f_kI           =   PID_I * dt,
    .f_kD           =   PID_D / dt,
    .cf_maxOut      =   0.3f,
    .cf_minOut      =   -0.3f,
    .cf_sampleTime  =   dt
};

static math_pidController_s s_pidYaw = {

    .pf_input       =   &gf_sensor_fusedAngles[flight_YAW],
    .pf_output      =   &f_pidOut[flight_YAW],
    .pf_setPoint    =   &gf_flight_setPoint[flight_YAW],
    .pf_lastInput   =   &f_pidLastInputs[flight_YAW],
    .pf_outputSum   =   &f_pidOutputSum[flight_YAW],
    .f_kP           =   math_RAD2NORM(PID_P),
    .f_kI           =   PID_I * dt,
    .f_kD           =   PID_D / dt,
    .cf_maxOut      =   0.3f,
    .cf_minOut      =   -0.3f,
    .cf_sampleTime  =   dt
};

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

                pid_values.f[i] = math_LIMIT(pid_values.f[i], 0.0, 10.0);

            }

            if(pid_temp[1] == 'r'){
                s_pidRoll.f_kP = math_RAD2NORM(pid_values.f[0]);
                s_pidRoll.f_kI  = pid_values.f[1] * dt;
                s_pidRoll.f_kD  = pid_values.f[2] / dt;
            }
            else if(pid_temp[1] == 'p'){
                s_pidPitch.f_kP = math_RAD2NORM(pid_values.f[0]);
                s_pidPitch.f_kI  = pid_values.f[1] * dt;
                s_pidPitch.f_kD  = pid_values.f[2] / dt;
            }
            else if(pid_temp[1] == 'y'){
                s_pidYaw.f_kP = math_RAD2NORM(pid_values.f[0]);
                s_pidYaw.f_kI  = pid_values.f[1] * dt;
                s_pidYaw.f_kD  = pid_values.f[2] / dt;
            }
            else{

                // wrong USB values
                s_pidRoll.f_kP = math_RAD2NORM(1.0);
                s_pidRoll.f_kI  = 0.0;
                s_pidRoll.f_kD  = 0.0;
                s_pidPitch.f_kP = math_RAD2NORM(1.0);
                s_pidPitch.f_kI  = 0.0;
                s_pidPitch.f_kD  = 0.0;
                s_pidYaw.f_kP = math_RAD2NORM(1.0);
                s_pidYaw.f_kI  = 0.0;
                s_pidYaw.f_kD  = 0.0;

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
        u8g_DrawStr(&gs_display,xOffset+18,   yOffset + 0,u8g_8toa((int8_t) math_NORM2RAD(s_pidRoll.f_kP),2));
        u8g_DrawStr(&gs_display,xOffset+32,      yOffset + 0,"I:");
        u8g_DrawStr(&gs_display,xOffset+36,   yOffset + 0,u8g_8toa((int8_t) s_pidRoll.f_kI,2));
        u8g_DrawStr(&gs_display,xOffset+52,      yOffset + 0,"D:");
        u8g_DrawStr(&gs_display,xOffset+56,   yOffset + 0,u8g_8toa((int8_t) s_pidRoll.f_kD,2));

        // Pitch PID controller
        u8g_DrawStr(&gs_display,xOffset,      yOffset + 6,"Pi:");
        u8g_DrawStr(&gs_display,xOffset+14,      yOffset + 6,"P:");
        u8g_DrawStr(&gs_display,xOffset+18,   yOffset + 6,u8g_8toa((int8_t) math_NORM2RAD(s_pidPitch.f_kP),2));
        u8g_DrawStr(&gs_display,xOffset+32,      yOffset + 6,"I:");
        u8g_DrawStr(&gs_display,xOffset+36,   yOffset + 6,u8g_8toa((int8_t) s_pidPitch.f_kI,2));
        u8g_DrawStr(&gs_display,xOffset+52,      yOffset + 6,"D:");
        u8g_DrawStr(&gs_display,xOffset+56,   yOffset + 6,u8g_8toa((int8_t) s_pidPitch.f_kD,2));

        // Yaw PID controller
        u8g_DrawStr(&gs_display,xOffset,      yOffset + 12,"Ya:");
        u8g_DrawStr(&gs_display,xOffset+14,      yOffset + 12,"P:");
        u8g_DrawStr(&gs_display,xOffset+18,   yOffset + 12,u8g_8toa((int8_t) math_NORM2RAD(s_pidYaw.f_kP),2));
        u8g_DrawStr(&gs_display,xOffset+32,      yOffset + 12,"I:");
        u8g_DrawStr(&gs_display,xOffset+36,   yOffset + 12,u8g_8toa((int8_t) s_pidYaw.f_kI,2));
        u8g_DrawStr(&gs_display,xOffset+52,      yOffset + 12,"D:");
        u8g_DrawStr(&gs_display,xOffset+56,   yOffset + 12,u8g_8toa((int8_t) s_pidYaw.f_kD,2));

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
    //float w_yaw;

    //w_yaw = (gf_sensor_fusedAngles[flight_YAW] - fused_yaw_angle_old) / dt;
    //fused_yaw_angle_old = gf_sensor_fusedAngles[flight_YAW];


//
//    s_pidRoll.f_error =gf_flight_setPoint[flight_ROLL]  - gf_sensor_fusedAngles[flight_ROLL];
//    s_pidPitch.f_error=gf_flight_setPoint[flight_PITCH] - gf_sensor_fusedAngles[flight_PITCH];
//    //s_pidYaw.f_error=gf_flight_setPoint[flight_YAW] - w_yaw;
//
//    pid_out[flight_ROLL]   = Math_StepPidController(&s_pidRoll);
//    pid_out[flight_PITCH]  = Math_StepPidController(&s_pidPitch);
//    //pid_out[flight_YAW] =   Math_StepPidController(&s_pidYaw);
//    pid_out[flight_YAW] = 0.0;

    // gf_flight_setPoint[flight_THROTTLE] = [0 -> 1]
    f_thrustBase = gf_flight_setPoint[flight_THROTTLE] * THR_SCALE;
    Math_StepPidController(&s_pidRateRoll);
    Math_StepPidController(&s_pidRatePitch);

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

	//				R      P     Y
	motor[0] = MIX(+1.0f,+1.0f,-1.0f); //Motor VL
	motor[1] = MIX(+1.0f,-1.0f,+1.0f); //Motor HL
	motor[2] = MIX(-1.0f,+1.0f,+1.0f); //Motor VR
	motor[3] = MIX(-1.0f,-1.0f,-1.0f); //Motor HR

	for(i=0;i<4;++i)
	{
		motor[i]=math_LIMIT(motor[i],0.1f,1.0f);
		gs_motor[i].ui16_setPoint=(uint16_t)(motor[i]*0xFFFF);
	}
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

