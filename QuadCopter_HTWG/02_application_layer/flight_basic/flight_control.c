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
#define dt 								0.002f		// [s]
#define YAW_MAX_INF						(1.0f/(15.0f*M_PI/180.0f))
#define MIX(X,Y,Z) 						(gf_flight_setPoint[flight_THROTTLE]*THR_SCALE + pid_out[flight_ROLL]*X + pid_out[flight_PITCH]*Y + pid_out[flight_YAW]*Z)

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

void Control_FlightStabilisation(void);
void Control_Mixer(void);
void Control_MotorSameSetPoint(uint16_t ui16_setPoint);

/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */

math_pidController_s s_pidRoll = {
		.f_sampleTime = dt,
		.f_kP		  =math_RAD2NORM(PID_P),
		.f_kI		  =PID_I,
		.f_kD		  =PID_D,
		.f_maxOut	  =0.3f,
		.f_minOut	  =-0.3f,
		.f_intSigOld  =0.0f,
		.i8_clamping  =0
};

math_pidController_s s_pidPitch = {
		.f_sampleTime = dt,
		.f_kP		  =math_RAD2NORM(PID_P),
		.f_kI		  =PID_I,
		.f_kD		  =PID_D,
		.f_maxOut	  =0.3f,
		.f_minOut	  =-0.3f,
		.f_intSigOld  =0.0f,
		.i8_clamping  =0
};

math_pidController_s s_pidYaw = {
		.f_sampleTime = dt,
		.f_kP		  =0.0f,
		.f_kI		  =0.0f,
		.f_kD		  =0.0f,
		.f_maxOut	  =0.3f,
		.f_minOut	  =-0.3f,
		.f_intSigOld  =0.0f,
		.i8_clamping  =0
};

// used global variables
extern volatile uint32_t 	gui32_flight_setPoint[4];
extern float gf_sensor_fusedAngles[3];
extern volatile motor_Data_s gs_motor[motor_COUNT];

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

static float pid_out[3];

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

#if( setup_DEV_PID_TUNE ) || DOXYGEN
	union {
		float f[3];
		char c[12];
	} pid_values;

	/**
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
				pid_values.c[i] = (uint8_t)temp_received;
				i++;
			}
			else
				break;

		}

		// when all is received, write values
		if (i==12){
			i = 0;
			s_pidRoll.f_kP 	= math_RAD2NORM(pid_values.f[0]);
			s_pidPitch.f_kP = math_RAD2NORM(pid_values.f[0]);
			s_pidRoll.f_kI 	= pid_values.f[1];
			s_pidPitch.f_kI = pid_values.f[1];
			s_pidRoll.f_kD 	= pid_values.f[2];
			s_pidPitch.f_kD = pid_values.f[2];
		}
	}
#endif

/**
 * \brief	Reset output & internal states of the flight Stabilisation.
 */
void Control_Reset(void)
{
	s_pidRoll. f_intSigOld=0.0f;
	s_pidPitch.f_intSigOld=0.0f;
	s_pidYaw.  f_intSigOld=0.0f;
	s_pidRoll. f_output   =0.0f;
	s_pidPitch.f_output   =0.0f;
	s_pidYaw.  f_output   =0.0f;
}

/**
 * \brief	Do control algorithm to stabilisate the QC
 */
void Control_FlightStabilisation(void)
{
	s_pidRoll.f_error =gf_flight_setPoint[flight_ROLL]  - gf_sensor_fusedAngles[flight_ROLL];
	s_pidPitch.f_error=gf_flight_setPoint[flight_PITCH] - gf_sensor_fusedAngles[flight_PITCH];

	pid_out[flight_ROLL]   = Math_StepPidController(&s_pidRoll);
	pid_out[flight_PITCH]  = Math_StepPidController(&s_pidPitch);
	pid_out[flight_YAW]	= 0.0f;//-gf_flight_setPoint[flight_YAW]*YAW_MAX_INF/2.0f;	###
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
	motor[0] = MIX(+1.0f,-1.0f,-1.0f); //Motor VL
	motor[1] = MIX(+1.0f,+1.0f,+1.0f); //Motor HL
	motor[2] = MIX(-1.0f,-1.0f,+1.0f); //Motor VR
	motor[3] = MIX(-1.0f,+1.0f,-1.0f); //Motor HR

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
