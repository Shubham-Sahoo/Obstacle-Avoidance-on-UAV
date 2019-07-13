/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskManualPositionSmoothVel.hpp
 *
 * Flight task for smooth manual controlled position.
 */

#pragma once

#include "FlightTaskManualPosition.hpp"
#include "VelocitySmoothing.hpp"

class FlightTaskManualPositionSmoothVel : public FlightTaskManualPosition
{
public:
	FlightTaskManualPositionSmoothVel() = default;

	virtual ~FlightTaskManualPositionSmoothVel() = default;

	bool activate() override;
	void reActivate() override;

protected:

	virtual void _updateSetpoints() override;

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskManualPosition,
					(ParamFloat<px4::params::MPC_JERK_MAX>) _param_mpc_jerk_max,
					(ParamFloat<px4::params::MPC_ACC_UP_MAX>) _param_mpc_acc_up_max,
					(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _param_mpc_acc_down_max
				       )
private:

	enum class Axes {XY, XYZ};

	/**
	 * Reset the required axes. when force_z_zero is set to true, the z derivatives are set to sero and not to the estimated states
	 */
	void reset(Axes axes, bool force_z_zero = false);
	void _checkEkfResetCounters(); /**< Reset the trajectories when the ekf resets velocity or position */

	VelocitySmoothing _smoothing[3]; ///< Smoothing in x, y and z directions
	matrix::Vector3f _vel_sp_smooth;
	bool _position_lock_xy_active{false};
	bool _position_lock_z_active{false};
	matrix::Vector2f _position_setpoint_xy_locked;
	float _position_setpoint_z_locked{NAN};

	/* counters for estimator local position resets */
	struct {
		uint8_t xy;
		uint8_t vxy;
		uint8_t z;
		uint8_t vz;
	} _reset_counters{0, 0, 0, 0};
};
