/*
 * basic_filters.h
 *
 *  Created on: 09.01.2016
 *      Author: Daniel Eckstein
 */

#ifndef _SRC_BASIC_FILTERS_H_
#define _SRC_BASIC_FILTERS_H_

/* Includes
=============================================================================*/


/* Defines
=============================================================================*/
// define Pi
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// calculate alpha from freqency and timestep
#define LP_FREQ2ALPHA(FREQ, DT) ((2.0f*M_PI*(DT)*(FREQ))/(2.0f*M_PI*(DT)*(FREQ)+1))
#define HP_FREQ2ALPHA(FREQ, DT) ((1.0f)/(2.0f*M_PI*(DT)*(FREQ)+1))

//// define a lopass array structure
//#define LP_A_STRUCT(name, alpha_val, size) struct lp_a_struct { float x[size]; float y[size]; const float alpha = alpha_val; } name;
//// define a highpass array structure
//#define HP_A_STRUCT(name, alpha_val, size) struct hp_a_struct { float x[size]; float x_old[size]; float y[size]; const float alpha = alpha_val; } name;

#define LP_ARRAY_SIZE 3
#define HP_ARRAY_SIZE 3
#define NF_ARRAY_SIZE 3

/* Structs
=============================================================================*/
struct hp_struct {
	float x;		// input value
	float x_old;	// last input value
	float y_old;	// last output value
	const float alpha; // filter coefficient
};

struct lp_struct {
	float x;		// input value
	float y_old;	// last output value
	const float alpha; // filter coefficient
};

struct lp_a_struct {
	float x[LP_ARRAY_SIZE];	// input value
	float y[LP_ARRAY_SIZE];	// last output value
	const float alpha;		// filter coefficient
};

struct hp_a_struct {
	float x[HP_ARRAY_SIZE];		// input value
	float x_old[HP_ARRAY_SIZE];	// last input value
	float y[HP_ARRAY_SIZE];		// last output value
	const float alpha;		// filter coefficient
};

struct notch_s {

    // input signal
    float x;
    // states to buffer older input signals
    float x1;
    float x2;
    // output signal (filtered)
    float y;
    // states to buffer older output signals
    float y1;
    float y2;

    // filter coefficients input
    const float b0;
    const float b1;
    const float b2;

    // filter coefficients output
    const float a0;
    const float a1;
    const float a2;

};

struct notch_a_s {

    // input signal
    float x[NF_ARRAY_SIZE];
    // states to buffer older input signals
    float x1[NF_ARRAY_SIZE];
    float x2[NF_ARRAY_SIZE];
    // output signal (filtered)
    float y[NF_ARRAY_SIZE];
    // states to buffer older output signals
    float y1[NF_ARRAY_SIZE];
    float y2[NF_ARRAY_SIZE];

    // filter coefficients input
    const float b0;
    const float b1;
    const float b2;

    // filter coefficients output
    const float a0;
    const float a1;
    const float a2;

};



/* Function Prototypes
=============================================================================*/
extern float highPass(struct hp_struct *hp);
extern float lowPass(struct lp_struct *lp);
extern void lowPassArray(struct lp_a_struct *lp);
extern void highPassArray(struct hp_a_struct *hp);
extern void notchFilterArray(float *pf_sensorData, struct notch_a_s *nf);

#endif /* _SRC_BASIC_FILTERS_H_ */
