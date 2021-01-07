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

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/
// standard libaries
#include <stdint.h>
#include <math.h>
#include <float.h>

// own header file
#include "qc_math.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Defines                                                 */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Type Definitions                                        */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Forward Declarations                                          */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Variables                                               */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Procedure Definitions                                         */
/* ---------------------------------------------------------------------------------------------------*/

/**
 * \brief   increment the given value by one but reset on given limit
 *          eg: limit is 5 then value ranges from 0...4
 *
 * \param   value       ->      ui32 value to limit
 * \param   limit       ->      ui32 limit for the value
 */
uint32_t increment2Limit(uint32_t value, uint32_t limit) {

    value++;
    value = (value == limit) ? 0 : value;

    return value;
}

/**
 * \brief	calculate one time step for the desired PID Controller.
 *			Make sure that this function is called periodically
 *			every sample time in this application they should be called every 2ms (flight_task).
 *
 * \param	ps_pidC         ->      Pointer to the PID Controller
 *
 */
void Math_StepPidController(math_pidController_s *ps_pidC)
{

    float f_input = *ps_pidC->pf_input;
    float f_error = *ps_pidC->pf_setPoint - f_input;
    float f_dInput = (f_input - *ps_pidC->pf_lastInput);
    *ps_pidC->pf_outputSum += (ps_pidC->f_kI * f_error);

    float f_output = ps_pidC->f_kP *f_error;

    f_output += *ps_pidC->pf_outputSum - ps_pidC->f_kP * f_dInput;

    if(f_output > ps_pidC->cf_maxOut)
    {
        f_output = ps_pidC->cf_maxOut;
    }
    else if(f_output < ps_pidC->cf_minOut)
    {
        f_output = ps_pidC->cf_minOut;
    }
    else
    {
        *ps_pidC->pf_output = f_output;
    }

    *ps_pidC->pf_lastInput = f_input;
}

/**
 * \brief   calculate Euler/Tait-Bryan angles out of quaternions
 *
 */
// TODO change so that it is applicable also without the global variable
void Math_QuatToEuler(volatile float q[], float fusedAngles[]){

    float yaw, pitch, roll;

    float sq0 =  (q[0]*q[0]);
    float sq1 =  (q[1]*q[1]);
    float sq2 =  (q[2]*q[2]);
    float sq3 =  (q[3]*q[3]);

    float test = 2.0 * (q[2]*q[0] - q[1]*q[3]);

    if(test > 0.999 && test < 1.001){

        yaw = (float) (-2.0*atan2f(q[1], q[0]));
        roll = 0.0;
        pitch = (float) (math_PI/2.0);
    }
    else if(test > -1.001 && test < -0.999){

        yaw = (float) (2.0*atan2f(q[1], q[0]));
        roll = 0.0;
        pitch = (float) (math_PI/-2.0);
    } else {

        yaw = (float) atan2f(2.0 * (q[1]*q[2] + q[3]*q[0]), (sq1 - sq2 - sq3 + sq0));
        roll = (float) atan2f(2.0 * (q[2]*q[3] + q[1]*q[0]), (-sq1 + sq2 + sq3 + sq0));
        pitch = (float) asinf(math_LIMIT(test, -1.0, 1.0));
    }


    fusedAngles[0] = roll;
    fusedAngles[1] = pitch;
    fusedAngles[2] = yaw;


}
