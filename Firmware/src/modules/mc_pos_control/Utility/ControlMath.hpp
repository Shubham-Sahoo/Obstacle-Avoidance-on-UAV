/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
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
 * @file ControlMath.hpp
 *
 * Simple functions for vector manipulation that do not fit into matrix lib.
 * These functions are specific for controls.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>

namespace ControlMath
{
/**
 * Converts thrust vector and yaw set-point to a desired attitude.
 * @param thr_sp a 3D vector
 * @param yaw_sp the desired yaw
 * @return vehicle_attitude_setpoints_s structure
 */
vehicle_attitude_setpoint_s thrustToAttitude(const matrix::Vector3f &thr_sp, const float yaw_sp);

/**
 * Outputs the sum of two vectors but respecting the limits and priority.
 * The sum of two vectors are constraint such that v0 has priority over v1.
 * This means that if the length of (v0+v1) exceeds max, then it is constraint such
 * that v0 has priority.
 *
 * @param v0 a 2D vector that has priority given the maximum available magnitude.
 * @param v1 a 2D vector that less priority given the maximum available magnitude.
 * @return 2D vector
 */
matrix::Vector2f constrainXY(const matrix::Vector2f &v0, const matrix::Vector2f &v1, const float &max);

/**
 * This method was used for smoothing the corners along two lines.
 *
 * @param sphere_c
 * @param sphere_r
 * @param line_a
 * @param line_b
 * @param res
 * return boolean
 *
 * Note: this method is not used anywhere and first requires review before usage.
 */
bool cross_sphere_line(const matrix::Vector3f &sphere_c, const float sphere_r, const matrix::Vector3f &line_a,
		       const matrix::Vector3f &line_b, matrix::Vector3f &res);
}
