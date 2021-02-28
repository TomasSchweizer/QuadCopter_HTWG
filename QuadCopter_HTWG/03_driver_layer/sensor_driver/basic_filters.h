/*===================================================================================================*/
/*  basic_filters.h                                                                                  */
/*===================================================================================================*/

/**
*   @file   basic_filters.h
*
*   @brief  API for basic LP, HP and notch filter implementations
*
*   @details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>06/12/2016      <td>Daniel Eckstein     <td>Implementation
*   <tr><td>20/01/2021      <td>Tomas Schweizer     <td>Added Notch-Filter
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*   -
*/
/*====================================================================================================*/

#ifndef _BASIC_FILTERS_H_
#define _BASIC_FILTERS_H_

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/
#include "qc_math.h"
/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/

// calculate alpha from frequency and sample time
#define LP_FREQ2ALPHA(FREQ, DT) ((2.0f*math_PI*(DT)*(FREQ))/(2.0f*math_PI*(DT)*(FREQ)+1))   ///< alpha for lowpass
#define HP_FREQ2ALPHA(FREQ, DT) ((1.0f)/(2.0f*math_PI*(DT)*(FREQ)+1))                       ///< alpha for highpass

#define LP_ARRAY_SIZE 3         ///< LP array size 3 because only used for either gyro or accelerometer with each 3 axis
#define HP_ARRAY_SIZE 3         ///< HP array size 3 because only used for either gyro or accelerometer with each 3 axis
#define NF_ARRAY_SIZE 3         ///< Notch filter array size 3 because only used for either gyro or accelerometer with each 3 axis

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/

/// Struct for first order highpass filter, with one value
struct hp_struct {
	float x;		            ///< Input value
	float x_old;	            ///< Last input value
	float y_old;	            ///< Last output value
	const float alpha;          ///< Filter coefficient
};

/// Struct for first order highpass filter, with an array of values
struct hp_a_struct {
    float x[HP_ARRAY_SIZE];     ///< Array of input values
    float x_old[HP_ARRAY_SIZE]; ///< Array of last input values
    float y[HP_ARRAY_SIZE];     ///< Array of last output values
    const float alpha;          ///< Filter coefficient
};

/// Struct for first order lowpass filter, with one value
struct lp_struct {
	float x;		            ///< input value
	float y_old;	            ///< last output value
	const float alpha;          ///< Filter coefficient
};

/// Struct for first order lowpass filter, with an array of values
struct lp_a_struct {
	float x[LP_ARRAY_SIZE];	    ///< Array of input values
	float y[LP_ARRAY_SIZE];	    ///< Array of last output values
	const float alpha;		    ///< Filter coefficient
};

/// Struct for second order IIR notch filter, with an array of values
struct notch_a_s {

    float x[NF_ARRAY_SIZE];     ///< Input signal array
    float x1[NF_ARRAY_SIZE];    ///< Array of states to buffer older input signal
    float x2[NF_ARRAY_SIZE];    ///< Array of states to buffer older input signal

    float y[NF_ARRAY_SIZE];     ///< Output signal array (filtered)
    float y1[NF_ARRAY_SIZE];    ///< Array of states to buffer older output signal
    float y2[NF_ARRAY_SIZE];    ///< Array of states to buffer older output signal

    // filter coefficients input
    const float b0;             ///< Coefficient for x
    const float b1;             ///< Coefficient for x1
    const float b2;             ///< Coefficient for x2

    // filter coefficients output
    const float a0;             ///< Coefficient for y
    const float a1;             ///< Coefficient for y1
    const float a2;             ///< Coefficient for y2

};

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/
extern float highPass(struct hp_struct *hp);
extern float lowPass(struct lp_struct *lp);
extern void lowPassArray(struct lp_a_struct *lp);
extern void highPassArray(struct hp_a_struct *hp);
extern void notchFilterArray(float *pf_sensorData, struct notch_a_s *nf);

#endif /* _BASIC_FILTERS_H_ */

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
