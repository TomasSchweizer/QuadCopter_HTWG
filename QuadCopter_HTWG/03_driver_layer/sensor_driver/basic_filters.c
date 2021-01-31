/*
 * basic_filters.c
 *
 *  Created on: 09.01.2016
 *      Author: Daniel Eckstein
 */
/* Includes
=============================================================================*/
#include "basic_filters.h"


/* Functions
=============================================================================*/

float highPass(struct hp_struct *hp){

	float y = hp->alpha*(hp->y_old + hp->x - hp->x_old);

	hp->x_old = hp->x;
	hp->y_old = y;

	return y;
}

float lowPass(struct lp_struct *lp){

	float y = lp->y_old + lp->alpha * (lp->x - lp->y_old);

	lp->y_old = y;

	return y;
}

void lowPassArray(struct lp_a_struct *lp){

	int i;
	float y_new[LP_ARRAY_SIZE];
	for(i=0; i<LP_ARRAY_SIZE; i++){
		y_new[i] = lp->y[i] + lp->alpha * (lp->x[i] - lp->y[i]);

		lp->y[i] = y_new[i];
	}

}

void highPassArray(struct hp_a_struct *hp){

	int i;
	float y_new[HP_ARRAY_SIZE];
	for(i=0; i<HP_ARRAY_SIZE; i++){
		y_new[i] = hp->alpha*(hp->y[i] + hp->x[i] - hp->x_old[i]);

		hp->x_old[i] = hp->x[i];
		hp->y[i] = y_new[i];
	}

}




void notchFilterArray(float *pf_sensorData, struct notch_a_s *nf)
{

    int i;

    for(i=0; i<NF_ARRAY_SIZE; i++)
    {

        nf->x[i] = pf_sensorData[i+3];

        nf->y[i] = (nf->b0 * nf->x[i]) + (nf->b1 * nf->x1[i]) + (nf->b2 * nf->x2[i])
                        - (nf->a1 * nf->y1[i]) - (nf->a2 * nf->y2[i]);

        nf->y2[i] = nf->y1[i];
        nf->y1[i] = nf->y[i];

        nf->x2[i] = nf->x1[i];
        nf->x1[i] = nf->x[i];

        pf_sensorData[i+3] = nf->y[i];

    }




}



