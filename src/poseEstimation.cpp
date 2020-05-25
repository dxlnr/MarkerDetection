#define _USE_MATH_DEFINES

#include <math.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include "poseEstimation.h"


using namespace std;


namespace
{
	//! @brief Use only this constant for accessing a quaternion's X component
	const unsigned char QX = 0;

	//! @brief Use only this constant for accessing a quaternion's Y component
	const unsigned char QY = 1;

	//! @brief Use only this constant for accessing a quaternion's Z component
	const unsigned char QZ = 2;

	//! @brief Use only this constant for accessing a quaternion's scalar component
	const unsigned char QW = 3;
}



float* normalizeQuaternion(float *q) {
  /**
    normalizes a quaternion (makes it a unit quaternion)
    */

	float norm = 0.0f;

	for (int i = 0; i < 4; i++) {
    norm += q[i] * q[i];
  }

	norm = sqrtf(1.0f / norm);

	for (int i = 0; i < 4; i++) {
    q[i] *= norm;
  }

	return q;
}


float* matrixToQuaternion(const CvMat *pMat, float *q) {

	// shortcuts to the matrix data

	const float* m0 = (float*)(pMat->data.ptr);
	const float* m1 = (float*)(pMat->data.ptr + pMat->step);
	const float* m2 = (float*)(pMat->data.ptr + 2 * pMat->step);


	// get entry of q with largest absolute value

	float tmp[4];

	tmp[QW] = m0[0] + m1[1] + m2[2];
	tmp[QX] = m0[0] - m1[1] - m2[2];
	tmp[QY] = -m0[0] + m1[1] - m2[2];
	tmp[QZ] = -m0[0] - m1[1] + m2[2];

	int max = QW;

	if (tmp[QX] > tmp[max]) {
    max = QX;
  }

	if (tmp[QY] > tmp[max]) {
    max = QY;
  }

	if (tmp[QZ] > tmp[max]) {
    max = QZ;
  }

	// depending on largest entry compute the other values
	// note: these formulae can be derived very simply from the
	//       matrix representation computed in quaternionToMatrix

	switch (max) {

	case QW:

		q[QW] = sqrtf(tmp[QW] + 1) * 0.5f;
		q[QX] = (m2[1] - m1[2]) / (4 * q[QW]);
		q[QY] = (m0[2] - m2[0]) / (4 * q[QW]);
		q[QZ] = (m1[0] - m0[1]) / (4 * q[QW]);

		break;


	case QX:

		q[QX] = sqrtf(tmp[QX] + 1) * 0.5f;
		q[QW] = (m2[1] - m1[2]) / (4 * q[QX]);
		q[QY] = (m1[0] + m0[1]) / (4 * q[QX]);
		q[QZ] = (m0[2] + m2[0]) / (4 * q[QX]);

		break;


	case QY:

		q[QY] = sqrtf(tmp[QY] + 1) * 0.5f;
		q[QW] = (m0[2] - m2[0]) / (4 * q[QY]);
		q[QX] = (m1[0] + m0[1]) / (4 * q[QY]);
		q[QZ] = (m2[1] + m1[2]) / (4 * q[QY]);

		break;


	case QZ:

		q[QZ] = sqrtf(tmp[QZ] + 1) * 0.5f;
		q[QW] = (m1[0] - m0[1]) / (4 * q[QZ]);
		q[QX] = (m0[2] + m2[0]) / (4 * q[QZ]);
		q[QY] = (m2[1] + m1[2]) / (4 * q[QZ]);

		break;

	}

	normalizeQuaternion(q);

	return q;
}
