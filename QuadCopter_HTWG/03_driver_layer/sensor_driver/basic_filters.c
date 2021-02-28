/*===================================================================================================*/
/*  basic_filters.c                                                                                  */
/*===================================================================================================*/

/*
*   file   basic_filters.c
*
*   brief  Functions for basic LP, HP and notch filter implementations
*
*   details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>09/01/2016      <td>Daniel Eckstein     <td>Implementation
*   <tr><td>20/01/2021      <td>Tomas Schweizer     <td>Added Notch-Filter
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*   -
*/
/*====================================================================================================*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

#include "basic_filters.h"

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
 * @brief   Applies a first order highpass filter to a signal
 *
 * @param   hp --> A pointer to an highpass structure
 *
 * @return  y --> Highpass filtered signal
 */
float highPass(struct hp_struct *hp){

	float y = hp->alpha*(hp->y_old + hp->x - hp->x_old);

	hp->x_old = hp->x;
	hp->y_old = y;

	return y;
}

/**
 * @brief   Applies a first order highpass filter to an array of signals.
 *          And saves the filtered signals in the hp structure.
 *
 * @param   hp --> A pointer to an highpass structure
 *
 * @return  void
 *
 */
void highPassArray(struct hp_a_struct *hp){

    int i;
    float y_new[HP_ARRAY_SIZE];
    for(i=0; i<HP_ARRAY_SIZE; i++){
        y_new[i] = hp->alpha*(hp->y[i] + hp->x[i] - hp->x_old[i]);

        hp->x_old[i] = hp->x[i];
        hp->y[i] = y_new[i];
    }

}

/**
 * @brief   Applies a first order lowpass filter to a signal
 *
 * @param   lp --> A pointer to an lowpass structure
 *
 * @return  y --> Lowpass filtered signal
 */
float lowPass(struct lp_struct *lp){

	float y = lp->y_old + lp->alpha * (lp->x - lp->y_old);

	lp->y_old = y;

	return y;
}

/**
 * @brief   Applies a first order lowpass filter to an array of signals.
 *          And saves the filtered signals in the lp structure.
 *
 * @param   lp --> A pointer to an lowpass structure
 *
 * @return  void
 */
void lowPassArray(struct lp_a_struct *lp){

	int i;
	float y_new[LP_ARRAY_SIZE];
	for(i=0; i<LP_ARRAY_SIZE; i++){
		y_new[i] = lp->y[i] + lp->alpha * (lp->x[i] - lp->y[i]);

		lp->y[i] = y_new[i];
	}

}

/**
 * @brief   Applies a second order IIR notch filter to an array of signals.
 *
 * @param   pf_sensorData --> pointer to an array of float sensor data
 * @param   nf -->  A pointer to an notch filter structure
 *
 * @return  void
 */
void notchFilterArray(float *pf_sensorData, struct notch_a_s *nf)
{

    int i;
    for(i=0; i<NF_ARRAY_SIZE; i++)
    {
        // get the sensor data
        nf->x[i] = pf_sensorData[i];

        // Calculate filtered output signal
        nf->y[i] = (nf->b0 * nf->x[i]) + (nf->b1 * nf->x1[i]) + (nf->b2 * nf->x2[i])
                        - (nf->a1 * nf->y1[i]) - (nf->a2 * nf->y2[i]);

        // save output in states
        nf->y2[i] = nf->y1[i];
        nf->y1[i] = nf->y[i];

        // save input in states
        nf->x2[i] = nf->x1[i];
        nf->x1[i] = nf->x[i];

        // write the output in the sensor data array
        pf_sensorData[i] = nf->y[i];

    }
}

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
