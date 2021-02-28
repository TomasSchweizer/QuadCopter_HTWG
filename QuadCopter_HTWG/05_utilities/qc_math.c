/*===================================================================================================*/
/*  qc_math.c                                                                                        */
/*===================================================================================================*/

/*
*   file   qc_math.c
*
*   @brief  Implementation of math functions, macros and mathematical objects for quadcopter.
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

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

// Standard libraries
#include <stdint.h>
#include <math.h>
#include <float.h>

// Utilities
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
 * @brief   Increment the given value by one but reset on given limit
 *
 * @details
 * eg: limit is 5 then value ranges from 0...4
 *
 * @param   value --> ui32 value to limit
 * @param   limit --> ui32 limit for the value
 *
 * @return uint32_t --> incremented and/or limited value
 */
uint32_t increment2Limit(uint32_t value, uint32_t limit) {

    value++;
    value = (value == limit) ? 0 : value;

    return value;
}

/**
 * @brief	Calculate one time step for the desired PID Controller.
 *
 * @details
 * Make sure that this function is called periodically every sample time.
 * In this application this should be called every 2ms (flight_task).
 *
 * @param	ps_pidC --> Pointer to a PID Controller instance
 *
 * @return  void
 *
 */
void Math_StepPidController(math_pidController_s *ps_pidC)
{

    // copy input variable in local variable
    float f_input = *ps_pidC->pf_input;
    // calculate error from setpoint and input
    float f_error = *ps_pidC->pf_setPoint - f_input;

    // Anti derivative kick: derivative of error is equal to -derivative of input so instead
    // + kD * derivative of error --> -kD * derivative of input
    float f_dInput = (f_input - *ps_pidC->pf_lastInput);

    // Anti tuning changes: calculate ITerm = outputSum and take kI into the integral because of this
    // the effect of the old errors doesn't get changed if kI is changed
    *ps_pidC->pf_outputSum += (ps_pidC->f_kI * f_error);

    // Anti wind-up 1 (saturation): The pid stops integrating if max value is reached
    if(*ps_pidC->pf_outputSum > ps_pidC->cf_maxOut)
    {
        *ps_pidC->pf_outputSum = ps_pidC->cf_maxOut;
    }
    else if(*ps_pidC->pf_outputSum < ps_pidC->cf_minOut)
    {
        *ps_pidC->pf_outputSum = ps_pidC->cf_minOut;
    }

    // Calculate output based on proportional gain part of PID and then add the intergral and
    // derivative gain part
    float f_output = ps_pidC->f_kP *f_error;
    f_output += *ps_pidC->pf_outputSum - ps_pidC->f_kD * f_dInput;

    // Anti wind-up 2: Also if the controller output is saturated eliminate
    // the effect of P and D gain
    if(f_output > ps_pidC->cf_maxOut)
    {
        f_output = ps_pidC->cf_maxOut;
    }
    else if(f_output < ps_pidC->cf_minOut)
    {
        f_output = ps_pidC->cf_minOut;
    }

    // Set the output value
    *ps_pidC->pf_output = f_output;

    // save last input value for derivative calculation
    *ps_pidC->pf_lastInput = f_input;
}

/**
 * @brief   Calculate Euler/Tait-Bryan angles out of quaternions
 *
 * @param   q* --> pointer/array of the quaternion
 * @param   fusedAngles --> Array to copy the fused angles into
 *
 * @return  void
 */
void Math_QuatToEuler(volatile float q[], float fusedAngles[]){

    float yaw, pitch, roll;

    float sq0 =  (q[0]*q[0]);
    float sq1 =  (q[1]*q[1]);
    float sq2 =  (q[2]*q[2]);
    float sq3 =  (q[3]*q[3]);


    // TODO improve test if time
    float test = 2.0 * (q[1]*q[3] + q[0]*q[2]);

    if(test > 0.999 && test < 1.001){

        yaw = (-2.0*atan2f(q[1], q[0]));
        pitch = (math_PI/2.0);
        roll = 0.0;

    }
    else if(test > -1.001 && test < -0.999){

        yaw = (2.0*atan2f(q[1], q[0]));
        pitch = (math_PI/-2.0);
        roll = 0.0;
    }
    else{

        roll = atan2f(  q[2]*q[3] + q[0]*q[1]   ,   0.5 - (sq1 + sq2)     );
        pitch = asinf(  -2.0 * (q[1]*q[3] - q[0]*q[2])  );
        yaw = atan2f(   q[1]*q[2] + q[0]*q[3]  ,   0.5 - (sq2 + sq3)    );

    }


    fusedAngles[0] = roll;
    fusedAngles[1] = pitch;
    fusedAngles[2] = yaw;


}

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
