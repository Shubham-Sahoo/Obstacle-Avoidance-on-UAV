/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file FlightTask.hpp
 *
 * Abstract base class for different advanced flight tasks like orbit, follow me, ...
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include <px4_module_params.h>
#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>
#include <uORB/SubscriptionPollable.hpp>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <lib/WeatherVane/WeatherVane.hpp>
#include "SubscriptionArray.hpp"

class FlightTask : public ModuleParams
{
public:
	FlightTask() :
		ModuleParams(nullptr)
	{
		_resetSetpoints();
		_constraints = empty_constraints;
	}

	virtual ~FlightTask() = default;

	/**
	 * Initialize the uORB subscriptions using an array
	 * @param subscription_array handling uORB subscribtions externally across task switches
	 * @return true on success, false on error
	 */
	virtual bool initializeSubscriptions(SubscriptionArray &subscription_array);

	/**
	 * Call once on the event where you switch to the task
	 * @return true on success, false on error
	 */
	virtual bool activate();

	/**
	 * Call this to reset an active Flight Task
	 */
	virtual void reActivate();

	/**
	 * To be called to adopt parameters from an arrived vehicle command
	 * @param command received command message containing the parameters
	 * @return true if accepted, false if declined
	 */
	virtual bool applyCommandParameters(const vehicle_command_s &command) { return false; }

	/**
	 * Call before activate() or update()
	 * to initialize time and input data
	 * @return true on success, false on error
	 */
	virtual bool updateInitialize();

	/**
	 * To be called regularly in the control loop cycle to execute the task
	 * @return true on success, false on error
	 */
	virtual bool update() = 0;

	/**
	 * Call after update()
	 * to constrain the generated setpoints in order to comply
	 * with the constraints of the current mode
	 * @return true on success, false on error
	 */
	virtual bool updateFinalize() { return true; };

	/**
	 * Get the output data
	 * @return task output setpoints that get executed by the positon controller
	 */
	const vehicle_local_position_setpoint_s getPositionSetpoint();

	/**
	 * Get vehicle constraints.
	 * The constraints can vary with task.
	 * @return constraints
	 */
	const vehicle_constraints_s &getConstraints() { return _constraints; }

	/**
	 * Get landing gear position.
	 * The constraints can vary with task.
	 * @return landing gear
	 */
	const landing_gear_s &getGear() { return _gear; }

	/**
	 * Get avoidance desired waypoint
	 * @return desired waypoints
	 */
	const vehicle_trajectory_waypoint_s &getAvoidanceWaypoint() { return _desired_waypoint; }

	/**
	 * Empty setpoint.
	 * All setpoints are set to NAN.
	 */
	static const vehicle_local_position_setpoint_s empty_setpoint;

	/**
	 * Empty constraints.
	 * All constraints are set to NAN.
	 */
	static const vehicle_constraints_s empty_constraints;

	/**
	 * default landing gear state
	 */
	static const landing_gear_s empty_landing_gear_default_keep;

	/**
	 * Empty desired waypoints.
	 * All waypoints are set to NAN.
	 */
	static const vehicle_trajectory_waypoint_s empty_trajectory_waypoint;

	/**
	 * Call this whenever a parameter update notification is received (parameter_update uORB message)
	 */
	void handleParameterUpdate()
	{
		updateParams();
	}

	/**
	 * Sets an external yaw handler which can be used by any flight task to implement a different yaw control strategy.
	 * This method does nothing, each flighttask which wants to use the yaw handler needs to override this method.
	 */
	virtual void setYawHandler(WeatherVane *ext_yaw_handler) {}

	void updateVelocityControllerIO(const matrix::Vector3f &vel_sp,
					const matrix::Vector3f &thrust_sp) {_velocity_setpoint_feedback = vel_sp; _thrust_setpoint_feedback = thrust_sp; }

protected:

	uORB::SubscriptionPollable<vehicle_local_position_s> *_sub_vehicle_local_position{nullptr};
	uORB::SubscriptionPollable<vehicle_attitude_s> *_sub_attitude{nullptr};
	uint8_t _heading_reset_counter{0}; /**< estimator heading reset */

	/** Reset all setpoints to NAN */
	void _resetSetpoints();

	/** Check and update local position */
	void _evaluateVehicleLocalPosition();

	/** Set constraints to default values */
	virtual void _setDefaultConstraints();

	/** determines when to trigger a takeoff (ignored in flight) */
	virtual bool _checkTakeoff();

	/* Time abstraction */
	static constexpr uint64_t _timeout = 500000; /**< maximal time in us before a loop or data times out */
	float _time = 0; /**< passed time in seconds since the task was activated */
	float _deltatime = 0; /**< passed time in seconds since the task was last updated */
	hrt_abstime _time_stamp_activate = 0; /**< time stamp when task was activated */
	hrt_abstime _time_stamp_current = 0; /**< time stamp at the beginning of the current task update */
	hrt_abstime _time_stamp_last = 0; /**< time stamp when task was last updated */

	/* Current vehicle state */
	matrix::Vector3f _position; /**< current vehicle position */
	matrix::Vector3f _velocity; /**< current vehicle velocity */
	float _yaw = 0.f; /**< current vehicle yaw heading */
	float _dist_to_bottom = 0.0f; /**< current height above ground level */

	/**
	 * Setpoints which the position controller has to execute.
	 * Setpoints that are set to NAN are not controlled. Not all setpoints can be set at the same time.
	 * If more than one type of setpoint is set, then order of control is a as follow: position, velocity,
	 * acceleration, thrust. The exception is _position_setpoint together with _velocity_setpoint, where the
	 * _velocity_setpoint is used as feedforward.
	 * _acceleration_setpoint and _jerk_setpoint are currently not supported.
	 */
	matrix::Vector3f _position_setpoint;
	matrix::Vector3f _velocity_setpoint;
	matrix::Vector3f _acceleration_setpoint;
	matrix::Vector3f _jerk_setpoint;
	matrix::Vector3f _thrust_setpoint;
	float _yaw_setpoint;
	float _yawspeed_setpoint;

	matrix::Vector3f _velocity_setpoint_feedback;
	matrix::Vector3f _thrust_setpoint_feedback;

	/**
	 * Vehicle constraints.
	 * The constraints can vary with tasks.
	 */
	vehicle_constraints_s _constraints{};

	landing_gear_s _gear{};

	/**
	 * Desired waypoints.
	 * Goals set by the FCU to be sent to the obstacle avoidance system.
	 */
	vehicle_trajectory_waypoint_s _desired_waypoint{};

	DEFINE_PARAMETERS_CUSTOM_PARENT(ModuleParams,
					(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max,
					(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,
					(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
					(ParamFloat<px4::params::MPC_TILTMAX_AIR>) _param_mpc_tiltmax_air
				       )
};
