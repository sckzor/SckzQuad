#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "gyro.h"
#include "madgwick.h"

/*
 * Get an euler heading angle derived from the accelerometer, magnetometer and gyroscope readings
 */
struct vec3 get_angle(struct vec3 w, struct vec3 a, struct vec3 m, double deltat) {
	static struct quaternion q = { 1, 0, 0, 0 };
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;
	/* Variables to avoid repeated arithmetic */
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q.q1;
	float _2q2 = 2.0f * q.q2;
	float _2q3 = 2.0f * q.q3;
	float _2q4 = 2.0f * q.q4;
	float _2q1q3 = 2.0f * q.q1 * q.q3;
	float _2q3q4 = 2.0f * q.q3 * q.q4;
	float q1q1 = q.q1 * q.q1;
	float q1q2 = q.q1 * q.q2;
	float q1q3 = q.q1 * q.q3;
	float q1q4 = q.q1 * q.q4;
	float q2q2 = q.q2 * q.q2;
	float q2q3 = q.q2 * q.q3;
	float q2q4 = q.q2 * q.q4;
	float q3q3 = q.q3 * q.q3;
	float q3q4 = q.q3 * q.q4;
	float q4q4 = q.q4 * q.q4;

	/* Convert degrees to radians */
	w.x *= 0.017453;
	w.y *= 0.017453;
	w.z *= 0.017453;

	/* Normalise accelerometer measurement */
	norm = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
	if (norm == 0.0f) {
		return a; /* Handle a possible NaN (a will always equal [0, 0, 0]) */
	}
	norm = 1.0f/norm;
	a.x *= norm;
	a.y *= norm;
	a.z *= norm;

	/* Normalise magnetometer measurementa */
	norm = sqrt(m.x * m.x + m.y * m.y + m.z * m.z);
	if (norm == 0.0f) {
		return m; /* Handle a possible NaN (m will always equal [0, 0, 0]) */ 
	}
	norm = 1.0f/norm;
	m.x *= norm;
	m.y *= norm;
	m.z *= norm;

	/*
	 * WARNING: DARK SORCERY AHEAD!
	 */

	/* Reference direction of Earth's magnetic field */
	_2q1mx = 2.0f * q.q1 * m.x;
	_2q1my = 2.0f * q.q1 * m.y;
	_2q1mz = 2.0f * q.q1 * m.z;
	_2q2mx = 2.0f * q.q2 * m.x;
	hx = m.x * q1q1 - _2q1my * q.q4 + _2q1mz * q.q3 + m.x * q2q2 + _2q2 * m.y * q.q3 + _2q2 * m.z * q.q4 - m.x * q3q3 - m.x * q4q4;
	hy = _2q1mx * q.q4 + m.y * q1q1 - _2q1mz * q.q2 + _2q2mx * q.q3 - m.y * q2q2 + m.y * q3q3 + _2q3 * m.z * q.q4 - m.y * q4q4;
	
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q.q3 + _2q1my * q.q2 + m.z * q1q1 + _2q2mx * q.q4 - m.z * q2q2 + _2q3 * m.y * q.q4 - m.z * q3q3 + m.z * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	/* Gradient decent algorithm corrective step */
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - a.x) + _2q2 * (2.0f * q1q2 + _2q3q4 - a.y) - _2bz * q.q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - m.x) + (-_2bx * q.q4 + _2bz * q.q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - m.y) + _2bx * q.q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - m.z);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - a.x) + _2q1 * (2.0f * q1q2 + _2q3q4 - a.y) - 4.0f * q.q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - a.z) + _2bz * q.q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - m.x) + (_2bx * q.q3 + _2bz * q.q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - m.y) + (_2bx * q.q4 - _4bz * q.q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - m.z);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - a.x) + _2q4 * (2.0f * q1q2 + _2q3q4 - a.y) - 4.0f * q.q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - a.z) + (-_4bx * q.q3 - _2bz * q.q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - m.x) + (_2bx * q.q2 + _2bz * q.q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - m.y) + (_2bx * q.q1 - _4bz * q.q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - m.z);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - a.x) + _2q3 * (2.0f * q1q2 + _2q3q4 - a.y) + (-_4bx * q.q4 + _2bz * q.q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - m.x) + (-_2bx * q.q1 + _2bz * q.q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - m.y) + _2bx * q.q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - m.z);

	/* Normalise step magnitude */
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	/* Compute rate of change of quaternion */
	qDot1 = 0.5f * (-q.q2 * w.x - q.q3 * w.y - q.q4 * w.z) - BETA * s1;
	qDot2 = 0.5f * (q.q1 * w.x + q.q3 * w.z - q.q4 * w.y) - BETA * s2;
	qDot3 = 0.5f * (q.q1 * w.y - q.q2 * w.z + q.q4 * w.x) - BETA * s3;
	qDot4 = 0.5f * (q.q1 * w.z + q.q2 * w.y - q.q3 * w.x) - BETA * s4;

	/* Integrate to yield quaternion */
	q.q1 += qDot1 * deltat;
	q.q2 += qDot2 * deltat;
	q.q3 += qDot3 * deltat;
	q.q4 += qDot4 * deltat;

	/* Normalise quaternion */
	norm = sqrt(q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3 + q.q4 * q.q4);
	norm = 1.0f/norm;

	q.q1 *= norm;
	q.q2 *= norm;
	q.q3 *= norm;
	q.q4 *= norm;

	/*
	 * End of black magic zone
	 */

	return to_euler(q);
}

/*
 * Take a quaternion angle and convert it to a 3D heading euler angle
 */
static struct vec3 to_euler(struct quaternion q) {
	struct vec3 dir;
	double temp1, temp2;

	/* Modified from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles */

	temp1 = 2 * (q.q4 * q.q1 + q.q2 * q.q3);
	temp2 = 1 - 2 * (q.q1 * q.q1 + q.q2 * q.q2);
	dir.x = atan2(temp1, temp2);

	temp1 = 2 * (q.q4 * q.q2 - q.q3 * q.q1);
	if(abs(temp1) >= 1) {
		dir.y = copysign(M_PI / 2, temp1);
	} else { 
		dir.y = asin(temp1);
	}

	temp1 = 2 * (q.q4 * q.q3 + q.q1 * q.q2);
	temp2 = 1 - 2 * (q.q2 * q.q2 + q.q3 * q.q3);
	dir.z = atan2(temp1, temp2);

	/* Convert back to degrees */

	dir.x *= 57.29577951;
	dir.y *= 57.29577951;
	dir.z *= 57.29577951;

	return dir;
}
