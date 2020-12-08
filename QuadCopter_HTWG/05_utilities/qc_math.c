/**
 * 		@file 	qc_math.c
 * 		@brief	Math functions, macros and mathematical objects for quadcopter
 *//*	@author Tobias Grimm
 * 		@date 	28.05.2016	(last modified)
 */

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <stdint.h>

// freeRTOS
#include "FreeRTOS.h"		// for pvPortMalloc

// utils
#include "qc_math.h"

/* ------------------------------------------------------------ */
/*				Local Defines									*/
/* ------------------------------------------------------------ */

#define CLAMPING_OFF 		( 0 )
#define CLAMPING_POS_SAT 	( 1 )
#define CLAMPING_NEG_SAT 	( 2 )

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

static void PidControllerCalcOutput(math_pidController_s *ps_pidC);

/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */
/*
 * increment the given value by one but reset on given limit
 * eg: limit is 5 then value ranges from 0...4
 */
uint32_t increment2Limit(uint32_t value, uint32_t limit) {
    value++;
    value = (value == limit) ? 0 : value;

    return value;
}






/**
 * \brief	calculate one time step for the desired PID Controller.
 *
 *			Make sure that this function is called periodically
 *			every f_sampleTime [s] seconds.
 * \param	ps_pidC   		Pointer to the PID Controller
 * \return	PID output signal
 */
float Math_StepPidController(math_pidController_s *ps_pidC)
{
	//  Check if Controller is in Saturation
	if (ps_pidC->i8_clamping != CLAMPING_OFF)
	{
		//  If Controller is in positive Saturation & Input Signal<0 -> we can now go out of Saturation
		if(ps_pidC->i8_clamping == CLAMPING_POS_SAT && ps_pidC->f_error < 0 )
		{
			ps_pidC->i8_clamping = CLAMPING_OFF;
			PidControllerCalcOutput( ps_pidC );
		}
		//  If Controller is in negative Saturation & Input Signal>0 -> we can now go out of Saturation
		else if(ps_pidC->i8_clamping == CLAMPING_NEG_SAT && ps_pidC->f_error > 0 )
		{
			ps_pidC->i8_clamping = CLAMPING_OFF;
			PidControllerCalcOutput( ps_pidC );
		}
		else
		{
			PidControllerCalcOutput( ps_pidC );
		}
	}
	//  Controller is not in Saturation
	else
	{
		PidControllerCalcOutput( ps_pidC );
	}
	return ps_pidC->f_output;
}

/**
 * \brief	calculate PID Controller output Signal and Check if
 *			ontroller has to go into Saturation.
 * \param	ps_pidC   		Pointer to the PID Controller
 */
static void PidControllerCalcOutput(math_pidController_s *ps_pidC)
{
	//  Internal Signal to store Integration Value
	float intSig;

	//  Integrate
	if(ps_pidC->i8_clamping!= CLAMPING_OFF)
		intSig=ps_pidC->f_intSigOld;			// don't integrate
	else
	{
		intSig = ps_pidC->f_error * ps_pidC->f_sampleTime * ps_pidC->f_kI + ps_pidC->f_intSigOld;
		ps_pidC->f_intSigOld = intSig;
	}

	//  Calc new Output Signal
	ps_pidC->f_output = ps_pidC->f_kP * (ps_pidC->f_error + intSig + ps_pidC->f_kD*ps_pidC->f_errorDt);

	//  Check if Output is to large
	if (ps_pidC->f_output > ps_pidC->f_maxOut)
	{
		//  Saturate Output
		ps_pidC->f_output = ps_pidC->f_maxOut;
		ps_pidC->i8_clamping = CLAMPING_POS_SAT;
	}
	//  Check if Output is to small
	else if (ps_pidC->f_output < ps_pidC->f_minOut)
	{
		//  Saturate Output
		ps_pidC->f_output = ps_pidC->f_minOut;
		ps_pidC->i8_clamping = CLAMPING_NEG_SAT;
	}
}

/**
 * \brief	calculate one time step for the desired Low Pass Filter.
 *
 *			Make sure that this function is called periodically
 *			every fSampleTime [s] seconds.
 * \param	ps_lpf  		Pointer to the Low Pass Filter
 */
void Math_StepLowPassFilter(math_lowPassFilter_s *ps_lpf)
{
	ps_lpf->f_output = *ps_lpf->pf_input * (ps_lpf->f_sampleTime / ps_lpf->f_timeConstant) - ps_lpf->f_outputOld * (ps_lpf->f_sampleTime / ps_lpf->f_timeConstant - 1);
	ps_lpf->f_outputOld = ps_lpf->f_output;
}
