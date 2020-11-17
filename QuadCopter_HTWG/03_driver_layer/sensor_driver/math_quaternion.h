/*
 * math_quaternion.h
 *
 *  Created on: 01.09.2015
 *      Author: Ecki
 */

#ifndef MY_MPU9250_SIMPLE_COMP_FILTER2_SRC_MATH_QUATERNION_H_
#define MY_MPU9250_SIMPLE_COMP_FILTER2_SRC_MATH_QUATERNION_H_

// choose precision for quaternion equations
#ifndef PREC_T_
#define PREC_T_
typedef float prec_t;
#endif


/* Contants
 */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
//#define M_PI 3.14159265f
#endif


// choose precision for equations
#ifndef PREC_T_
#define PREC_T_
typedef float prec_t;
#endif


/*
 * Function prototypes
 */
extern uint32_t increment(uint32_t value, uint32_t limit);
extern void normVector(prec_t *vector3D);
prec_t deg2rad(prec_t degree_angle);
prec_t rad2deg(prec_t radian_angle);

/*
 * Stuctures
 */
// quaternion struct
struct quat {
	float w; // q0 (real value)
	float x; // q1
	float y; // q2
	float z; // q3
};

/*
 * Function prototypes
 */
extern struct quat quatFromVectors(prec_t *vectA, prec_t *vectB);
extern void normQuaternion(struct quat *q);
extern struct quat createUnitQuaternion(prec_t *vector3D, prec_t phi);
extern struct quat euler2Quaternion(prec_t *euler_angles);
extern struct quat halfEuler2Quaternion(prec_t *half_euler_angles);
extern struct quat mulQuaternions(struct quat *q0, struct quat *q1);
extern void quaternion2Euler(struct quat *q, prec_t *euler_angles);


#endif /* MY_MPU9250_SIMPLE_COMP_FILTER2_SRC_MATH_QUATERNION_H_ */
