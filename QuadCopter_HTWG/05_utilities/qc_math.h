/**
 * 		@file 	qc_math.h
 * 		@brief	Math functions, macros and mathematical objects for quadcopter
 *//*	@author Tobias Grimm
 * 		@date 	28.05.2016	(last modified)
 */

#ifndef __QC_MATH_H__
#define	__QC_MATH_H__

/* ------------------------------------------------------------ */
/*					Include File Definitions					*/
/* ------------------------------------------------------------ */

#include <stdint.h>

/* ------------------------------------------------------------ */
/*				   	Defines			    						*/
/* ------------------------------------------------------------ */

#define math_PI 						3.14159265358979323846f
 /**
  * \brief	convert radiant to degree (floating point)
  * \param	f_rad	the floating point number to convert
  */
#define math_RAD2DEC(f_rad)				(f_rad*360.0f/(2.0f*math_PI))
 /**
  * \brief	convert radiant to normaliced (floating point)
  * \param	f_rad	the floating point number to convert
  */
#define math_RAD2NORM(f_rad)			(f_rad*  1.0f/(2.0f*math_PI))
 /**
  * \brief	convert degree to radiant (floating point)
  * \param	f_dec	the floating point number to convert
  */
#define math_DEC2RAD(f_dec)				(f_dec/360.0f*(2.0f*math_PI))
 /**
  * \brief	convert degree to normaliced (floating point)
  * \param	f_dec	the floating point number to convert
  */
#define math_DEC2NORM(f_dec)			(f_dec/360.0f)
 /**
  * \brief	saturate a desired value when it reachs the low or high limit
  * \param	value	the number to saturate
  *	\param	low		the lower limit
  *	\param	high	the higher limit
  */
#define math_LIMIT(value, low, high) 	((value)<(low)?(low):((value)>(high)?(high):(value)))

#define math_NULL						0

/* ------------------------------------------------------------ */
/*				   	Type Definitions			    			*/
/* ------------------------------------------------------------ */

/**
 * \brief	structure to make an instance of a 1. Order Lowpass Filter.
 *
 *			An IIR-Filter Structure is used to approximate a continuous
 *			1. Order Lowpass Filter.
 *			When the ratio between timeConstant/sampleTime get's really
 *			small (for example smaller 5) the approximation gets really bad.
 */
typedef struct math_lowPassFilter_s
{
	float   		* pf_input;			/**< Pointer to Input Signal */
	float   		  f_output;			/**< Output Signal */
	const float		  f_sampleTime;		/**< Sample Time [s] */
	const float	      f_timeConstant;	/**< 1. Order Low-Pass Time Constant [s] */

	float   		  f_outputOld;		/**< Internal Signal -> set it to 0 */

} math_lowPassFilter_s;

/**
 * @brief structure to make a instance of a PID Controller.
 *
 *		(Ideal Form, not Parallel)
 *		The Output can be Saturated to a desired value.
 *		And the Anti-Windup Method "clamping" is used
 *		to stop the Integral Part when the Output is in
 *		saturation.
 */
typedef struct math_pidController_s
{
	float   		  f_error;			/**< Error Signal */
	float   		  f_errorDt;		/**< Error Signal differentiated to time */
	float   		  f_output;			/**< Output Signal */
	const float		  f_sampleTime;		/**< Sample Time [s] */
	float	      	  f_kP;				/**< Proportional Gain  [] */
	float	          f_kI;				/**< Integral Gain  [1/s] */
	float	          f_kD;				/**< Diff Gain  [s] */
	const float	      f_maxOut;			/**< Maximal Output  [] */
	const float	      f_minOut;			/**< Minimal Output  [] */

	float   		  f_intSigOld;		/**< Internal Signal -> set it to 0 */
	int8_t   		  i8_clamping;		/**< Internal Signal -> set it to 0 */

} math_pidController_s;

/* ------------------------------------------------------------ */
/*					API Procedure Declarations					*/
/* ------------------------------------------------------------ */

extern float Math_StepPidController(math_pidController_s *ps_pidC);
extern void  Math_StepLowPassFilter(math_lowPassFilter_s *ps_lpf);

/* ------------------------------------------------------------ */
/*					Global Variables							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */

#endif // __QC_MATH_H__
