/****************************************************************************
 *
 *  Copyright (C) 2018-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file test_controlmath.cpp
 * Tests for the controls calculations.
 */

#include <unit_test.h>
#include <mc_pos_control/Utility/ControlMath.hpp>
#include <mathlib/mathlib.h>
#include <float.h>

#define SIGMA_SINGLE_OP			0.000001f

class ControlMathTest : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool testThrAttMapping();
	bool testPrioritizeVector();
	bool crossSphereLineTest();
};

bool ControlMathTest::run_tests()
{
	ut_run_test(testThrAttMapping);
	ut_run_test(testPrioritizeVector);
	ut_run_test(crossSphereLineTest);

	return (_tests_failed == 0);
}

bool ControlMathTest::testThrAttMapping()
{

	/* expected: zero roll, zero pitch, zero yaw, full thr mag
	 * reasone: thrust pointing full upward
	 */
	matrix::Vector3f thr{0.0f, 0.0f, -1.0f};
	float yaw = 0.0f;
	vehicle_attitude_setpoint_s att = ControlMath::thrustToAttitude(thr, yaw);
	ut_assert_true(att.roll_body < SIGMA_SINGLE_OP);
	ut_assert_true(att.pitch_body < SIGMA_SINGLE_OP);
	ut_assert_true(att.yaw_body < SIGMA_SINGLE_OP);
	ut_assert_true(-att.thrust_body[2] - 1.0f < SIGMA_SINGLE_OP);

	/* expected: same as before but with 90 yaw
	 * reason: only yaw changed
	 */
	yaw = M_PI_2_F;
	att = ControlMath::thrustToAttitude(thr, yaw);
	ut_assert_true(att.roll_body < SIGMA_SINGLE_OP);
	ut_assert_true(att.pitch_body < SIGMA_SINGLE_OP);
	ut_assert_true(att.yaw_body - M_PI_2_F < SIGMA_SINGLE_OP);
	ut_assert_true(-att.thrust_body[2] - 1.0f < SIGMA_SINGLE_OP);

	/* expected: same as before but roll 180
	 * reason: thrust points straight down and order Euler
	 * order is: 1. roll, 2. pitch, 3. yaw
	 */
	thr = matrix::Vector3f(0.0f, 0.0f, 1.0f);
	att = ControlMath::thrustToAttitude(thr, yaw);
	ut_assert_true(fabsf(att.roll_body) - M_PI_F < SIGMA_SINGLE_OP);
	ut_assert_true(fabsf(att.pitch_body) < SIGMA_SINGLE_OP);
	ut_assert_true(att.yaw_body - M_PI_2_F < SIGMA_SINGLE_OP);
	ut_assert_true(-att.thrust_body[2] - 1.0f < SIGMA_SINGLE_OP);

	/* TODO: find a good way to test it */


	return true;
}

bool ControlMathTest::testPrioritizeVector()
{
	float max = 5.0f;

	// v0 already at max
	matrix::Vector2f v0(max, 0);
	matrix::Vector2f v1(v0(1), -v0(0));

	// the static keywork is a workaround for an internal bug of GCC
	// "internal compiler error: in trunc_int_for_mode, at explow.c:55"
	matrix::Vector2f v_r = ControlMath::constrainXY(v0, v1, max);
	ut_assert_true(fabsf(v_r(0)) - max < SIGMA_SINGLE_OP && v_r(0) > 0.0f);
	ut_assert_true(fabsf(v_r(1) - 0.0f) < SIGMA_SINGLE_OP);

	// v1 exceeds max but v0 is zero
	v0.zero();
	v_r = ControlMath::constrainXY(v0, v1, max);
	ut_assert_true(fabsf(v_r(1)) - max < SIGMA_SINGLE_OP && v_r(1) < 0.0f);
	ut_assert_true(fabsf(v_r(0) - 0.0f) < SIGMA_SINGLE_OP);

	// v0 and v1 are below max
	v0 = matrix::Vector2f(0.5f, 0.5f);
	v1(0) = v0(1); v1(1) = -v0(0);
	v_r = ControlMath::constrainXY(v0, v1, max);
	float diff = matrix::Vector2f(v_r - (v0 + v1)).length();
	ut_assert_true(diff < SIGMA_SINGLE_OP);

	// v0 and v1 exceed max and are perpendicular
	v0 = matrix::Vector2f(4.0f, 0.0f);
	v1 = matrix::Vector2f(0.0f, -4.0f);
	v_r = ControlMath::constrainXY(v0, v1, max);
	ut_assert_true(v_r(0) - v0(0) < SIGMA_SINGLE_OP && v_r(0) > 0.0f);
	float remaining = sqrtf(max * max - (v0(0) * v0(0)));
	ut_assert_true(fabsf(v_r(1)) - remaining  < SIGMA_SINGLE_OP && v_r(1) < SIGMA_SINGLE_OP);

	//TODO: add more tests with vectors not perpendicular

	return true;
}

bool ControlMathTest::crossSphereLineTest()
{
	matrix::Vector3f prev = matrix::Vector3f(0.0f, 0.0f, 0.0f);
	matrix::Vector3f curr = matrix::Vector3f(0.0f, 0.0f, 2.0f);
	matrix::Vector3f res;
	bool retval = false;

	/*
	 * Testing 9 positions (+) around waypoints (o):
	 *
	 * Far             +              +              +
	 *
	 * Near            +              +              +
	 * On trajectory --+----o---------+---------o----+--
	 *                    prev                curr
	 *
	 * Expected targets (1, 2, 3):
	 * Far             +              +              +
	 *
	 *
	 * On trajectory -------1---------2---------3-------
	 *
	 *
	 * Near            +              +              +
	 * On trajectory -------o---1---------2-----3-------
	 *
	 *
	 * On trajectory --+----o----1----+--------2/3---+--
	 */

	// on line, near, before previous waypoint
	retval = ControlMath::cross_sphere_line(matrix::Vector3f(0.0f, 0.0f, -0.5f), 1.0f, prev, curr, res);
	PX4_WARN("result %.2f, %.2f, %.2f", (double)res(0), (double)res(1), (double)res(2));
	ut_assert_true(retval);
	ut_compare_float("target A 0", res(0), 0.0f, 2);
	ut_compare_float("target A 1", res(1), 0.0f, 2);
	ut_compare_float("target A 2", res(2), 0.5f, 2);

	// on line, near, before target waypoint
	retval = ControlMath::cross_sphere_line(matrix::Vector3f(0.0f, 0.0f, 1.0f), 1.0f, prev, curr, res);
	PX4_WARN("result %.2f, %.2f, %.2f", (double)res(0), (double)res(1), (double)res(2));
	ut_assert_true(retval);
	ut_compare_float("target B 0", res(0), 0.0f, 2);
	ut_compare_float("target B 1", res(1), 0.0f, 2);
	ut_compare_float("target B 2", res(2), 2.0f, 2);

	// on line, near, after target waypoint
	retval = ControlMath::cross_sphere_line(matrix::Vector3f(0.0f, 0.0f, 2.5f), 1.0f, prev, curr, res);
	PX4_WARN("result %.2f, %.2f, %.2f", (double)res(0), (double)res(1), (double)res(2));
	ut_assert_true(retval);
	ut_compare_float("target C 0", res(0), 0.0f, 2);
	ut_compare_float("target C 1", res(1), 0.0f, 2);
	ut_compare_float("target C 2", res(2), 2.0f, 2);

	// near, before previous waypoint
	retval = ControlMath::cross_sphere_line(matrix::Vector3f(0.0f, 0.5f, -0.5f), 1.0f, prev, curr, res);
	PX4_WARN("result %.2f, %.2f, %.2f", (double)res(0), (double)res(1), (double)res(2));
	ut_assert_true(retval);
	ut_compare_float("target D 0", res(0), 0.0f, 2);
	ut_compare_float("target D 1", res(1), 0.0f, 2);
	ut_compare_float("target D 2", res(2), 0.37f, 2);

	// near, before target waypoint
	retval = ControlMath::cross_sphere_line(matrix::Vector3f(0.0f, 0.5f, 1.0f), 1.0f, prev, curr, res);
	PX4_WARN("result %.2f, %.2f, %.2f", (double)res(0), (double)res(1), (double)res(2));
	ut_assert_true(retval);
	ut_compare_float("target E 0", res(0), 0.0f, 2);
	ut_compare_float("target E 1", res(1), 0.0f, 2);
	ut_compare_float("target E 2", res(2), 1.87f, 2);

	// near, after target waypoint
	retval = ControlMath::cross_sphere_line(matrix::Vector3f(0.0f, 0.5f, 2.5f), 1.0f, prev, curr, res);
	PX4_WARN("result %.2f, %.2f, %.2f", (double)res(0), (double)res(1), (double)res(2));
	ut_assert_true(retval);
	ut_compare_float("target F 0", res(0), 0.0f, 2);
	ut_compare_float("target F 1", res(1), 0.0f, 2);
	ut_compare_float("target F 2", res(2), 2.0f, 2);

	// far, before previous waypoint
	retval = ControlMath::cross_sphere_line(matrix::Vector3f(0.0f, 2.0f, -0.5f), 1.0f, prev, curr, res);
	PX4_WARN("result %.2f, %.2f, %.2f", (double)res(0), (double)res(1), (double)res(2));
	ut_assert_false(retval);
	ut_compare_float("target G 0", res(0), 0.0f, 2);
	ut_compare_float("target G 1", res(1), 0.0f, 2);
	ut_compare_float("target G 2", res(2), 0.0f, 2);

	// far, before target waypoint
	retval = ControlMath::cross_sphere_line(matrix::Vector3f(0.0f, 2.0f, 1.0f), 1.0f, prev, curr, res);
	PX4_WARN("result %.2f, %.2f, %.2f", (double)res(0), (double)res(1), (double)res(2));
	ut_assert_false(retval);
	ut_compare_float("target H 0", res(0), 0.0f, 2);
	ut_compare_float("target H 1", res(1), 0.0f, 2);
	ut_compare_float("target H 2", res(2), 1.0f, 2);

	// far, after target waypoint
	retval = ControlMath::cross_sphere_line(matrix::Vector3f(0.0f, 2.0f, 2.5f), 1.0f, prev, curr, res);
	PX4_WARN("result %.2f, %.2f, %.2f", (double)res(0), (double)res(1), (double)res(2));
	ut_assert_false(retval);
	ut_compare_float("target I 0", res(0), 0.0f, 2);
	ut_compare_float("target I 1", res(1), 0.0f, 2);
	ut_compare_float("target I 2", res(2), 2.0f, 2);

	return true;
}

ut_declare_test_c(test_controlmath, ControlMathTest)
