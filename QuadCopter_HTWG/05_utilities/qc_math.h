/*===================================================================================================*/
/*  qc_math.h                                                                                        */
/*===================================================================================================*/

/**
*   @file   qc_math.h
*
*   @brief  API for math functions, macros and mathematical objects for quadcopter.
*
*   @details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>28/05/2016      <td>Tobias Grimm        <td>Implementation & last modifications through MAs
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Completly new implementation (Quaternion2Euler, PID)
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*   - https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h [PID algorithm is based on this]
*   - Quaternion2Euler based on paper "Orientation, Rotation, Velocity and Acceleration,and the SRM"
*/
/*====================================================================================================*/

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
  * @brief  PI as define
  */
#define math_PI                         3.14159265358979323846f
/**
  * @brief  Null as define
  */
#define math_NULL                       0
 /**
  * @brief  Convert radiant to degree (floating point)
  *
  * @param  f_rad --> The floating point number to convert
  */
#define math_RAD2DEC(f_rad)             (f_rad*360.0f/(2.0f*math_PI))
 /**
  * @brief  Convert radiant to normalized (floating point)
  *
  * @param  f_rad --> The floating point number to convert
  */
#define math_RAD2NORM(f_rad)            (f_rad*1.0f/(2.0f*math_PI))
 /**
  * @brief  Convert normalized to radiant (floating point)
  *
  * @param  f_rad --> The floating point number to convert
  */
#define math_NORM2RAD(f_rad)            (f_rad*(2.0f*math_PI))
 /**
  * @brief  Convert degree to radiant (floating point)
  *
  * @param  f_dec --> The floating point number to convert
  */
#define math_DEC2RAD(f_dec)             (f_dec/360.0f*(2.0f*math_PI))
 /**
  * @brief  Convert degree to normalized (floating point)
  *
  * @param  f_dec --> The floating point number to convert
  */
#define math_DEC2NORM(f_dec)            (f_dec/360.0f)
 /**
  * @brief  Saturate a desired value when it reaches the low or high limit
  *
  * @param  value --> The number to saturate
  * @param  low --> The lower limit
  * @param  high --> The higher limit
  */
#define math_LIMIT(value, low, high)    ((value)<(low)?(low):((value)>(high)?(high):(value)))

/**
 * @brief Compare two values and find maximum
 *
 * @param   value --> The number to compare with max
 * @param   max --> max value
 */
#define math_MAX(value, max)            ( (value) > (max) ? (value) : (max) )

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/
 /**
 * @brief Structure to make a instance of a PID Controller.
 *
 */
typedef struct math_pidController_s
{
    float              *pf_input;       ///< Pointer to PID input variable
    float              *pf_output;      ///< Pointer to PID output variable
    float              *pf_setPoint;    ///< Pointer to PID setPoint variable
    float              *pf_lastInput;   ///< Pointer to PID lastInput value
    float              *pf_outputSum;   ///< Pointer to PID sum of output values
    float               f_kP;           ///< Proportional gain []
    float               f_kI;           ///< Integral gain [1/s]
    float               f_kD;           ///< Differential gain [s]
    const float         cf_maxOut;      ///< Maximal Output []
    const float         cf_minOut;      ///< Minimal Output []
    const float         cf_sampleTime;  ///< Sample time PID []

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

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
