/*
 * math_quaternion.c
 *
 *  Created on: 01.09.2015
 *      Author: Ecki
 */

/*
 * Includes
 */
#include <stdint.h>
#include <stdbool.h>
#include "stdio.h"
#include "math.h"
#include "math_quaternion.h"




//*****************************************************************************
// Functions
//*****************************************************************************

/*
 * increment the given value by one but reset on given limit
 * eg: limit is 5 then value ranges from 0...4
 */
uint32_t increment(uint32_t value, uint32_t limit) {
    value++;
    value = (value == limit) ? 0 : value;

    return value;
}



struct quat quatFromVectors(prec_t *vectA, prec_t *vectB){
	/*
	 * returns the rotation quaternion from two given vectors
	 * caution: vectors have to be normalized!
	 */
	struct quat q;

	// calculate x,y,z of quaternion via cross product of vectors
	q.x = vectA[1]*vectB[2] - vectA[2]*vectB[1];
	q.y = vectA[2]*vectB[0] - vectA[0]*vectB[2];
	q.z = vectA[0]*vectB[1] - vectA[1]*vectB[0];

	// calculate w via dot product
	q.w = 1.0f + vectA[0]*vectB[0] + vectA[1]*vectB[1] + vectA[2]*vectB[2];

	// check for 180 degrees roation
	if ((q.x == 0.0f) && (q.y == 0.0f) && (q.z == 0.0f) && (q.w == 0.0f)){
		// if so, generate a perpendicular vector for rotation
		prec_t perp_vect[3] = {-vectA[1], vectA[0], 0.0f};
		normVector(perp_vect);
		q = createUnitQuaternion(perp_vect, M_PI);
	}
	else{
		normQuaternion(&q);
	}

	return q;
}


/*
 * Normalizes the given 3D vector
 */
void normVector(prec_t *vector3D){

	prec_t length = sqrtf(vector3D[0]*vector3D[0] + vector3D[1]*vector3D[1] + vector3D[2]*vector3D[2]);

	vector3D[0] /= length;
	vector3D[1] /= length;
	vector3D[2] /= length;
}

prec_t deg2rad(prec_t degree_angle){
	/*
	 * convert degree angle to radiant angle
	 */
	return degree_angle * M_PI / 180.0;
}

prec_t rad2deg(prec_t radian_angle){
	/*
	 * convert radian angle to degree angle
	 */
	return radian_angle * 180.0 / M_PI;
}


struct quat createUnitQuaternion(prec_t *vector3D, prec_t phi){
	/* creates a normalized unit quaternion from given vector and rotation (in rad)
	 * caution: vector has to be normalized!
	 */
	struct quat q;
	q.w = cosf(phi/2.0f);
	q.x = vector3D[0] * sinf(phi/2.0f);
	q.y = vector3D[1] * sinf(phi/2.0f);
	q.z = vector3D[2] * sinf(phi/2.0f);

	return q;
}


void normQuaternion(struct quat *q){
	/*
	 * Calculates the normalized Quaternion
	 * w is the real part
	 * x, y, z are the complex elements
	 * Source: Buchholz, J. J. (2013). Vorlesungsmanuskript Regelungstechnik und Flugregler.
	 * GRIN Verlag. Retrieved from http://www.grin.com/de/e-book/82818/regelungstechnik-und-flugregler
	 */

	prec_t length = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);

	if (length != 0.0){
		q->w /= length;
		q->x /= length;
		q->y /= length;
		q->z /= length;
	}
}

void quaternion2Euler(struct quat *q, prec_t *euler_angles){
	/*
	 * converts the given rotation quaternion (unit quaternion) into euler angles (must be 3D)
	 * !!!Not Tested
	 */

	euler_angles[0] = atan2f(2*(q->w * q->x + q->y * q->z), 1-2*(q->x*q->x + q->y*q->y));
	euler_angles[1] = asinf(2*(q->w * q->y - q->z*q->x));
	euler_angles[2] = atan2f(2*(q->w * q->z + q->x * q->y), 1-2*(q->y*q->y + q->z*q->z));

}

struct quat euler2Quaternion(prec_t *euler_angles){
	/*
	 * converts the given euler angles (must be 3D) into one rotation quaternion (unit quaternion)
	 */
	struct quat q;

	prec_t c1 = cosf(euler_angles[0]/2.0);
	prec_t c2 = cosf(euler_angles[1]/2.0);
	prec_t c3 = cosf(euler_angles[2]/2.0);
	prec_t s1 = sinf(euler_angles[0]/2.0);
	prec_t s2 = sinf(euler_angles[1]/2.0);
	prec_t s3 = sinf(euler_angles[2]/2.0);

	q.w = c1*c2*c3 - s1*s2*s3;
	q.x = s1*c2*c3 - c1*s2*s3;
	q.y = c1*s2*c3 + s1*c2*s3;
	q.z = c1*c2*s3 - s1*s2*c3;

	// normQuaternion(&q);

	return q;
}

struct quat halfEuler2Quaternion(prec_t *half_euler_angles){
	/*
	 * converts the given euler angles (must be 3D and divided by 2) into one rotation quaternion (unit quaternion)
	 */
	struct quat q;

	prec_t c1 = cosf(half_euler_angles[0]);
	prec_t c2 = cosf(half_euler_angles[1]);
	prec_t c3 = cosf(half_euler_angles[2]);
	prec_t s1 = sinf(half_euler_angles[0]);
	prec_t s2 = sinf(half_euler_angles[1]);
	prec_t s3 = sinf(half_euler_angles[2]);

	q.w = c1*c2*c3 - s1*s2*s3;
	q.x = s1*c2*c3 - c1*s2*s3;
	q.y = c1*s2*c3 + s1*c2*s3;
	q.z = c1*c2*s3 - s1*s2*c3;

	// normQuaternion(&q);

	return q;
}


struct quat mulQuaternions(struct quat *q0, struct quat *q1){
	/*
	 * multiplies the two given quaternions
	 * result will be a normalized quaternion
	 */
	struct quat q_r;

	q_r.w = q0->w*q1->w - q0->x*q1->x - q0->y*q1->y - q0->z*q1->z;
	q_r.x = q0->w*q1->x + q1->w*q0->x + q0->y*q1->z - q0->z*q1->y;
	q_r.y = q0->w*q1->y + q1->w*q0->y + q0->z*q1->x - q0->x*q1->z;
	q_r.z = q0->w*q1->z + q1->w*q0->z + q0->x*q1->y - q0->y*q1->x;

	normQuaternion(&q_r);

	return q_r;

}

