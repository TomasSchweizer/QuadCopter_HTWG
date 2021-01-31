//=====================================================================================================
// @file qc_math.c
//=====================================================================================================
//
// @brief Implementation of math functions, macros and mathematical objects for quadcopter.
//        For example PID, Limit, ...
//
// Date                 Author                      Notes
// @date 28/05/2016     @author Tobias Grimm        PID, Limit
// @date 28/12/2020     @author Tomas Schweizer     Quaternion2Euler, PID implementation improvement
//
//
// Source:
// https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h [PID algorithm is based on this]
//
//=====================================================================================================

#ifndef __QC_MATH_H__
#define	__QC_MATH_H__

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/
#include <stdint.h>

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/
/**
  * \brief  PI as define
  */
#define math_PI                         3.14159265358979323846f
/**
  * \brief  Null as define
  */
#define math_NULL                       0
 /**
  * \brief  convert radiant to degree (floating point)
  *
  * \param  f_rad   the floating point number to convert
  */
#define math_RAD2DEC(f_rad)             (f_rad*360.0f/(2.0f*math_PI))
 /**
  * \brief  convert radiant to normalized (floating point)
  *
  * \param  f_rad   the floating point number to convert
  */
#define math_RAD2NORM(f_rad)            (f_rad*1.0f/(2.0f*math_PI))
 /**
  * \brief  convert normalized to radiant (floating point)
  *
  * \param  f_rad   the floating point number to convert
  */
#define math_NORM2RAD(f_rad)            (f_rad*(2.0f*math_PI))
 /**
  * \brief  convert degree to radiant (floating point)
  *
  * \param  f_dec   the floating point number to convert
  */
#define math_DEC2RAD(f_dec)             (f_dec/360.0f*(2.0f*math_PI))
 /**
  * \brief  convert degree to normalized (floating point)
  *
  * \param  f_dec   the floating point number to convert
  */
#define math_DEC2NORM(f_dec)            (f_dec/360.0f)
 /**
  * \brief  saturate a desired value when it reaches the low or high limit
  *
  * \param  value   the number to saturate
  * \param  low     the lower limit
  * \param  high    the higher limit
  */
#define math_LIMIT(value, low, high)    ((value)<(low)?(low):((value)>(high)?(high):(value)))

#define math_MAX(value, max)            ( (value) > (max) ? (value) : (max) )

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/
 /**
 * @brief structure to make a instance of a PID Controller.
 *
 */
typedef struct math_pidController_s
{
    float              *pf_input;       /**< Pointer to PID input variable */
    float              *pf_output;      /**< Pointer to PID output variable */
    float              *pf_setPoint;    /**< Pointer to PID setPoint variable */
    float              *pf_lastInput;   /**< Pointer to PID lastInput value */
    float              *pf_outputSum;   /**< Pointer to PID sum of output values */
    float               f_kP;           /**< Proportional gain [] */
    float               f_kI;           /**< Integral gain [1/s] */
    float               f_kD;           /**< Differential gain [s] */
    const float         cf_maxOut;      /**< Maximal Output [] */
    const float         cf_minOut;      /**< Minimal Output [] */
    const float         cf_sampleTime;   /**< Sample time PID [] */

} math_pidController_s;
/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/
extern uint32_t increment2Limit(uint32_t value, uint32_t limit);
extern void Math_StepPidController(math_pidController_s *ps_pidC);
extern void Math_QuatToEuler(volatile float q[], float fusedAngles[]);

#endif // __QC_MATH_H__

//=====================================================================================================
// End of file
//=====================================================================================================
