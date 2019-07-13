/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 *
 */

#include "mc_att_control.hpp"

#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

#define TPA_RATE_LOWER_LIMIT 0.05f

#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3

using namespace matrix;

MulticopterAttitudeControl::MulticopterAttitudeControl() :
	ModuleParams(nullptr),
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control"))
{
	for (uint8_t i = 0; i < MAX_GYRO_COUNT; i++) {
		_sensor_gyro_sub[i] = -1;
	}

	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	/* initialize quaternions in messages to be valid */
	_v_att.q[0] = 1.f;
	_v_att_sp.q_d[0] = 1.f;

	_rates_prev.zero();
	_rates_prev_filtered.zero();
	_rates_sp.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	/* initialize thermal corrections as we might not immediately get a topic update (only non-zero values) */
	for (unsigned i = 0; i < 3; i++) {
		// used scale factors to unity
		_sensor_correction.gyro_scale_0[i] = 1.0f;
		_sensor_correction.gyro_scale_1[i] = 1.0f;
		_sensor_correction.gyro_scale_2[i] = 1.0f;
	}

	parameters_updated();
}

void
MulticopterAttitudeControl::parameters_updated()
{
	// Store some of the parameters in a more convenient way & precompute often-used values
	_attitude_control.setProportionalGain(Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(), _param_mc_yaw_p.get()));

	// rate gains
	_rate_p = Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get());
	_rate_i = Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get());
	_rate_int_lim = Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get());
	_rate_d = Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get());
	_rate_ff = Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get());

	if (fabsf(_lp_filters_d.get_cutoff_freq() - _param_mc_dterm_cutoff.get()) > 0.01f) {
		_lp_filters_d.set_cutoff_frequency(_loop_update_rate_hz, _param_mc_dterm_cutoff.get());
		_lp_filters_d.reset(_rates_prev);
	}

	// angular rate limits
	using math::radians;
	_attitude_control.setRateLimit(Vector3f(radians(_param_mc_rollrate_max.get()), radians(_param_mc_pitchrate_max.get()),
						radians(_param_mc_yawrate_max.get())));

	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_man_tilt_max = math::radians(_param_mpc_man_tilt_max.get());

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	/* get transformation matrix from sensor/board to body frame */
	_board_rotation = get_rot_matrix((enum Rotation)_param_sens_board_rot.get());

	/* fine tune the rotation */
	Dcmf board_rotation_offset(Eulerf(
					   M_DEG_TO_RAD_F * _param_sens_board_x_off.get(),
					   M_DEG_TO_RAD_F * _param_sens_board_y_off.get(),
					   M_DEG_TO_RAD_F * _param_sens_board_z_off.get()));
	_board_rotation = board_rotation_offset * _board_rotation;
}

void
MulticopterAttitudeControl::parameter_update_poll()
{
	/* Check if parameters have changed */
	parameter_update_s param_update;

	if (_params_sub.update(&param_update)) {
		updateParams();
		parameters_updated();
	}
}

void
MulticopterAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	if (_vehicle_status_sub.update(&_vehicle_status)) {
		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (_actuators_id == nullptr) {
			if (_vehicle_status.is_vtol) {
				_actuators_id = ORB_ID(actuator_controls_virtual_mc);
				_attitude_sp_id = ORB_ID(mc_virtual_attitude_setpoint);

				int32_t vt_type = -1;

				if (param_get(param_find("VT_TYPE"), &vt_type) == PX4_OK) {
					_is_tailsitter = (static_cast<vtol_type>(vt_type) == vtol_type::TAILSITTER);
				}

			} else {
				_actuators_id = ORB_ID(actuator_controls_0);
				_attitude_sp_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}
}

void
MulticopterAttitudeControl::vehicle_motor_limits_poll()
{
	/* check if there is a new message */
	multirotor_motor_limits_s motor_limits{};

	if (_motor_limits_sub.update(&motor_limits)) {
		_saturation_status.value = motor_limits.saturation_status;
	}
}

bool
MulticopterAttitudeControl::vehicle_attitude_poll()
{
	/* check if there is a new message */
	const uint8_t prev_quat_reset_counter = _v_att.quat_reset_counter;

	if (_v_att_sub.update(&_v_att)) {
		// Check for a heading reset
		if (prev_quat_reset_counter != _v_att.quat_reset_counter) {
			// we only extract the heading change from the delta quaternion
			_man_yaw_sp += Eulerf(Quatf(_v_att.delta_q_reset)).psi();
		}

		return true;
	}

	return false;
}

float
MulticopterAttitudeControl::throttle_curve(float throttle_stick_input)
{
	// throttle_stick_input is in range [0, 1]
	switch (_param_mpc_thr_curve.get()) {
	case 1: // no rescaling to hover throttle
		return _param_mpc_manthr_min.get() + throttle_stick_input * (_param_mpc_thr_max.get() - _param_mpc_manthr_min.get());

	default: // 0 or other: rescale to hover throttle at 0.5 stick
		if (throttle_stick_input < 0.5f) {
			return (_param_mpc_thr_hover.get() - _param_mpc_manthr_min.get()) / 0.5f * throttle_stick_input +
			       _param_mpc_manthr_min.get();

		} else {
			return (_param_mpc_thr_max.get() - _param_mpc_thr_hover.get()) / 0.5f * (throttle_stick_input - 1.0f) +
			       _param_mpc_thr_max.get();
		}
	}
}

float
MulticopterAttitudeControl::get_landing_gear_state()
{
	// Only switch the landing gear up if we are not landed and if
	// the user switched from gear down to gear up.
	// If the user had the switch in the gear up position and took off ignore it
	// until he toggles the switch to avoid retracting the gear immediately on takeoff.
	if (_vehicle_land_detected.landed) {
		_gear_state_initialized = false;
	}

	float landing_gear = landing_gear_s::GEAR_DOWN; // default to down

	if (_manual_control_sp.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON && _gear_state_initialized) {
		landing_gear = landing_gear_s::GEAR_UP;

	} else if (_manual_control_sp.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
		// Switching the gear off does put it into a safe defined state
		_gear_state_initialized = true;
	}

	return landing_gear;
}

void
MulticopterAttitudeControl::generate_attitude_setpoint(float dt, bool reset_yaw_sp)
{
	vehicle_attitude_setpoint_s attitude_setpoint{};
	const float yaw = Eulerf(Quatf(_v_att.q)).psi();

	/* reset yaw setpoint to current position if needed */
	if (reset_yaw_sp) {
		_man_yaw_sp = yaw;

	} else if (_manual_control_sp.z > 0.05f || _param_mc_airmode.get() == (int32_t)Mixer::Airmode::roll_pitch_yaw) {

		const float yaw_rate = math::radians(_param_mpc_man_y_max.get());
		attitude_setpoint.yaw_sp_move_rate = _manual_control_sp.r * yaw_rate;
		_man_yaw_sp = wrap_pi(_man_yaw_sp + attitude_setpoint.yaw_sp_move_rate * dt);
	}

	/*
	 * Input mapping for roll & pitch setpoints
	 * ----------------------------------------
	 * We control the following 2 angles:
	 * - tilt angle, given by sqrt(x*x + y*y)
	 * - the direction of the maximum tilt in the XY-plane, which also defines the direction of the motion
	 *
	 * This allows a simple limitation of the tilt angle, the vehicle flies towards the direction that the stick
	 * points to, and changes of the stick input are linear.
	 */
	const float x = _manual_control_sp.x * _man_tilt_max;
	const float y = _manual_control_sp.y * _man_tilt_max;

	// we want to fly towards the direction of (x, y), so we use a perpendicular axis angle vector in the XY-plane
	Vector2f v = Vector2f(y, -x);
	float v_norm = v.norm(); // the norm of v defines the tilt angle

	if (v_norm > _man_tilt_max) { // limit to the configured maximum tilt angle
		v *= _man_tilt_max / v_norm;
	}

	Quatf q_sp_rpy = AxisAnglef(v(0), v(1), 0.f);
	Eulerf euler_sp = q_sp_rpy;
	attitude_setpoint.roll_body = euler_sp(0);
	attitude_setpoint.pitch_body = euler_sp(1);
	// The axis angle can change the yaw as well (noticeable at higher tilt angles).
	// This is the formula by how much the yaw changes:
	//   let a := tilt angle, b := atan(y/x) (direction of maximum tilt)
	//   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
	attitude_setpoint.yaw_body = _man_yaw_sp + euler_sp(2);

	/* modify roll/pitch only if we're a VTOL */
	if (_vehicle_status.is_vtol) {
		// Construct attitude setpoint rotation matrix. Modify the setpoints for roll
		// and pitch such that they reflect the user's intention even if a large yaw error
		// (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
		// from the pure euler angle setpoints will lead to unexpected attitude behaviour from
		// the user's view as the euler angle sequence uses the  yaw setpoint and not the current
		// heading of the vehicle.
		// However there's also a coupling effect that causes oscillations for fast roll/pitch changes
		// at higher tilt angles, so we want to avoid using this on multicopters.
		// The effect of that can be seen with:
		// - roll/pitch into one direction, keep it fixed (at high angle)
		// - apply a fast yaw rotation
		// - look at the roll and pitch angles: they should stay pretty much the same as when not yawing

		// calculate our current yaw error
		float yaw_error = wrap_pi(attitude_setpoint.yaw_body - yaw);

		// compute the vector obtained by rotating a z unit vector by the rotation
		// given by the roll and pitch commands of the user
		Vector3f zB = {0.0f, 0.0f, 1.0f};
		Dcmf R_sp_roll_pitch = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, 0.0f);
		Vector3f z_roll_pitch_sp = R_sp_roll_pitch * zB;

		// transform the vector into a new frame which is rotated around the z axis
		// by the current yaw error. this vector defines the desired tilt when we look
		// into the direction of the desired heading
		Dcmf R_yaw_correction = Eulerf(0.0f, 0.0f, -yaw_error);
		z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;

		// use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
		// R_tilt is computed from_euler; only true if cos(roll) not equal zero
		// -> valid if roll is not +-pi/2;
		attitude_setpoint.roll_body = -asinf(z_roll_pitch_sp(1));
		attitude_setpoint.pitch_body = atan2f(z_roll_pitch_sp(0), z_roll_pitch_sp(2));
	}

	/* copy quaternion setpoint to attitude setpoint topic */
	Quatf q_sp = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, attitude_setpoint.yaw_body);
	q_sp.copyTo(attitude_setpoint.q_d);
	attitude_setpoint.q_d_valid = true;

	attitude_setpoint.thrust_body[2] = -throttle_curve(_manual_control_sp.z);
	attitude_setpoint.timestamp = hrt_absolute_time();

	if (_attitude_sp_id != nullptr) {
		orb_publish_auto(_attitude_sp_id, &_vehicle_attitude_setpoint_pub, &attitude_setpoint, nullptr, ORB_PRIO_DEFAULT);
	}

	_landing_gear.landing_gear = get_landing_gear_state();
	_landing_gear.timestamp = hrt_absolute_time();
	_landing_gear_pub.publish(_landing_gear);
}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
void
MulticopterAttitudeControl::control_attitude()
{
	_v_att_sp_sub.update(&_v_att_sp);

	// reinitialize the setpoint while not armed to make sure no value from the last mode or flight is still kept
	if (!_v_control_mode.flag_armed) {
		Quatf().copyTo(_v_att_sp.q_d);
		Vector3f().copyTo(_v_att_sp.thrust_body);
	}

	// physical thrust axis is the negative of body z axis
	_thrust_sp = -_v_att_sp.thrust_body[2];

	_rates_sp = _attitude_control.update(Quatf(_v_att.q), Quatf(_v_att_sp.q_d), _v_att_sp.yaw_sp_move_rate);
}

/*
 * Throttle PID attenuation
 * Function visualization available here https://www.desmos.com/calculator/gn4mfoddje
 * Input: 'tpa_breakpoint', 'tpa_rate', '_thrust_sp'
 * Output: 'pidAttenuationPerAxis' vector
 */
Vector3f
MulticopterAttitudeControl::pid_attenuations(float tpa_breakpoint, float tpa_rate)
{
	/* throttle pid attenuation factor */
	float tpa = 1.0f - tpa_rate * (fabsf(_thrust_sp) - tpa_breakpoint) / (1.0f - tpa_breakpoint);
	tpa = fmaxf(TPA_RATE_LOWER_LIMIT, fminf(1.0f, tpa));

	Vector3f pidAttenuationPerAxis;
	pidAttenuationPerAxis(AXIS_INDEX_ROLL) = tpa;
	pidAttenuationPerAxis(AXIS_INDEX_PITCH) = tpa;
	pidAttenuationPerAxis(AXIS_INDEX_YAW) = 1.0;

	return pidAttenuationPerAxis;
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
MulticopterAttitudeControl::control_attitude_rates(float dt)
{
	/* reset integral if disarmed */
	if (!_v_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		_rates_int.zero();
	}

	// get the raw gyro data and correct for thermal errors
	Vector3f rates;

	if (_selected_gyro == 0) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_0[0]) * _sensor_correction.gyro_scale_0[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_0[1]) * _sensor_correction.gyro_scale_0[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_0[2]) * _sensor_correction.gyro_scale_0[2];

	} else if (_selected_gyro == 1) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_1[0]) * _sensor_correction.gyro_scale_1[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_1[1]) * _sensor_correction.gyro_scale_1[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_1[2]) * _sensor_correction.gyro_scale_1[2];

	} else if (_selected_gyro == 2) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_2[0]) * _sensor_correction.gyro_scale_2[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_2[1]) * _sensor_correction.gyro_scale_2[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_2[2]) * _sensor_correction.gyro_scale_2[2];

	} else {
		rates(0) = _sensor_gyro.x;
		rates(1) = _sensor_gyro.y;
		rates(2) = _sensor_gyro.z;
	}

	// rotate corrected measurements from sensor to body frame
	rates = _board_rotation * rates;

	// correct for in-run bias errors
	rates(0) -= _sensor_bias.gyro_x_bias;
	rates(1) -= _sensor_bias.gyro_y_bias;
	rates(2) -= _sensor_bias.gyro_z_bias;

	Vector3f rates_p_scaled = _rate_p.emult(pid_attenuations(_param_mc_tpa_break_p.get(), _param_mc_tpa_rate_p.get()));
	Vector3f rates_i_scaled = _rate_i.emult(pid_attenuations(_param_mc_tpa_break_i.get(), _param_mc_tpa_rate_i.get()));
	Vector3f rates_d_scaled = _rate_d.emult(pid_attenuations(_param_mc_tpa_break_d.get(), _param_mc_tpa_rate_d.get()));

	/* angular rates error */
	Vector3f rates_err = _rates_sp - rates;

	/* apply low-pass filtering to the rates for D-term */
	Vector3f rates_filtered(_lp_filters_d.apply(rates));

	_att_control = rates_p_scaled.emult(rates_err) +
		       _rates_int -
		       rates_d_scaled.emult(rates_filtered - _rates_prev_filtered) / dt +
		       _rate_ff.emult(_rates_sp);

	_rates_prev = rates;
	_rates_prev_filtered = rates_filtered;

	/* update integral only if we are not landed */
	if (!_vehicle_land_detected.maybe_landed && !_vehicle_land_detected.landed) {
		for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
			// Check for positive control saturation
			bool positive_saturation =
				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_pos) ||
				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_pos) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_pos);

			// Check for negative control saturation
			bool negative_saturation =
				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_neg) ||
				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_neg) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_neg);

			// prevent further positive control saturation
			if (positive_saturation) {
				rates_err(i) = math::min(rates_err(i), 0.0f);

			}

			// prevent further negative control saturation
			if (negative_saturation) {
				rates_err(i) = math::max(rates_err(i), 0.0f);

			}

			// I term factor: reduce the I gain with increasing rate error.
			// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
			// change (noticeable in a bounce-back effect after a flip).
			// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
			// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
			// and up to 200 deg error leads to <25% reduction of I.
			float i_factor = rates_err(i) / math::radians(400.f);
			i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

			// Perform the integration using a first order method and do not propagate the result if out of range or invalid
			float rate_i = _rates_int(i) + i_factor * rates_i_scaled(i) * rates_err(i) * dt;

			if (PX4_ISFINITE(rate_i) && rate_i > -_rate_int_lim(i) && rate_i < _rate_int_lim(i)) {
				_rates_int(i) = rate_i;

			}
		}
	}

	/* explicitly limit the integrator state */
	for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
		_rates_int(i) = math::constrain(_rates_int(i), -_rate_int_lim(i), _rate_int_lim(i));

	}
}

void
MulticopterAttitudeControl::publish_rates_setpoint()
{
	_v_rates_sp.roll = _rates_sp(0);
	_v_rates_sp.pitch = _rates_sp(1);
	_v_rates_sp.yaw = _rates_sp(2);
	_v_rates_sp.thrust_body[0] = 0.0f;
	_v_rates_sp.thrust_body[1] = 0.0f;
	_v_rates_sp.thrust_body[2] = -_thrust_sp;
	_v_rates_sp.timestamp = hrt_absolute_time();

	_v_rates_sp_pub.publish(_v_rates_sp);
}

void
MulticopterAttitudeControl::publish_rate_controller_status()
{
	rate_ctrl_status_s rate_ctrl_status = {};
	rate_ctrl_status.timestamp = hrt_absolute_time();
	rate_ctrl_status.rollspeed = _rates_prev(0);
	rate_ctrl_status.pitchspeed = _rates_prev(1);
	rate_ctrl_status.yawspeed = _rates_prev(2);
	rate_ctrl_status.rollspeed_integ = _rates_int(0);
	rate_ctrl_status.pitchspeed_integ = _rates_int(1);
	rate_ctrl_status.yawspeed_integ = _rates_int(2);

	_controller_status_pub.publish(rate_ctrl_status);
}

void
MulticopterAttitudeControl::publish_actuator_controls()
{
	_actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
	_actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
	_actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
	_actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
	_actuators.control[7] = (float)_landing_gear.landing_gear;
	_actuators.timestamp = hrt_absolute_time();
	_actuators.timestamp_sample = _sensor_gyro.timestamp;

	/* scale effort by battery status */
	if (_param_mc_bat_scale_en.get() && _battery_status.scale > 0.0f) {
		for (int i = 0; i < 4; i++) {
			_actuators.control[i] *= _battery_status.scale;
		}
	}

	if (!_actuators_0_circuit_breaker_enabled) {
		orb_publish_auto(_actuators_id, &_actuators_0_pub, &_actuators, nullptr, ORB_PRIO_DEFAULT);
	}
}

void
MulticopterAttitudeControl::run()
{
	_gyro_count = math::constrain(orb_group_count(ORB_ID(sensor_gyro)), 1, MAX_GYRO_COUNT);

	for (unsigned s = 0; s < _gyro_count; s++) {
		_sensor_gyro_sub[s] = orb_subscribe_multi(ORB_ID(sensor_gyro), s);
	}

	/* wakeup source: gyro data from sensor selected by the sensor app */
	px4_pollfd_struct_t poll_fds = {};
	poll_fds.events = POLLIN;

	const hrt_abstime task_start = hrt_absolute_time();
	hrt_abstime last_run = task_start;
	float dt_accumulator = 0.f;
	int loop_counter = 0;

	bool reset_yaw_sp = true;
	float attitude_dt = 0.f;

	while (!should_exit()) {

		// check if the selected gyro has updated first
		_sensor_correction_sub.update(&_sensor_correction);

		/* update the latest gyro selection */
		if (_sensor_correction.selected_gyro_instance < _gyro_count) {
			_selected_gyro = _sensor_correction.selected_gyro_instance;
		}

		poll_fds.fd = _sensor_gyro_sub[_selected_gyro];

		/* wait for up to 100ms for data */
		int pret = px4_poll(&poll_fds, 1, 100);

		/* timed out - periodic check for should_exit() */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			PX4_ERR("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			px4_usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);

		/* run controller on gyro changes */
		if (poll_fds.revents & POLLIN) {
			const hrt_abstime now = hrt_absolute_time();

			// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
			const float dt = math::constrain(((now - last_run) / 1e6f), 0.0002f, 0.02f);
			last_run = now;

			/* copy gyro data */
			orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub[_selected_gyro], &_sensor_gyro);

			/* run the rate controller immediately after a gyro update */
			if (_v_control_mode.flag_control_rates_enabled) {
				control_attitude_rates(dt);

				publish_actuator_controls();
				publish_rate_controller_status();
			}

			/* check for updates in other topics */
			_v_control_mode_sub.update(&_v_control_mode);
			_battery_status_sub.update(&_battery_status);
			_sensor_bias_sub.update(&_sensor_bias);
			_vehicle_land_detected_sub.update(&_vehicle_land_detected);
			_landing_gear_sub.update(&_landing_gear);
			vehicle_status_poll();
			vehicle_motor_limits_poll();
			const bool manual_control_updated = _manual_control_sp_sub.update(&_manual_control_sp);
			const bool attitude_updated = vehicle_attitude_poll();

			attitude_dt += dt;

			/* Check if we are in rattitude mode and the pilot is above the threshold on pitch
			 * or roll (yaw can rotate 360 in normal att control). If both are true don't
			 * even bother running the attitude controllers */
			if (_v_control_mode.flag_control_rattitude_enabled) {
				_v_control_mode.flag_control_attitude_enabled =
					fabsf(_manual_control_sp.y) <= _param_mc_ratt_th.get() &&
					fabsf(_manual_control_sp.x) <= _param_mc_ratt_th.get();
			}

			bool attitude_setpoint_generated = false;

			const bool is_hovering = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
						 && !_vehicle_status.in_transition_mode;

			// vehicle is a tailsitter in transition mode
			const bool is_tailsitter_transition = _vehicle_status.in_transition_mode && _is_tailsitter;

			bool run_att_ctrl = _v_control_mode.flag_control_attitude_enabled && (is_hovering || is_tailsitter_transition);


			if (run_att_ctrl) {
				if (attitude_updated) {
					// Generate the attitude setpoint from stick inputs if we are in Manual/Stabilized mode
					if (_v_control_mode.flag_control_manual_enabled &&
					    !_v_control_mode.flag_control_altitude_enabled &&
					    !_v_control_mode.flag_control_velocity_enabled &&
					    !_v_control_mode.flag_control_position_enabled) {
						generate_attitude_setpoint(attitude_dt, reset_yaw_sp);
						attitude_setpoint_generated = true;
					}

					control_attitude();

					if (_v_control_mode.flag_control_yawrate_override_enabled) {
						/* Yaw rate override enabled, overwrite the yaw setpoint */
						_v_rates_sp_sub.update(&_v_rates_sp);
						const auto yawrate_reference = _v_rates_sp.yaw;
						_rates_sp(2) = yawrate_reference;
					}

					publish_rates_setpoint();
				}

			} else {
				/* attitude controller disabled, poll rates setpoint topic */
				if (_v_control_mode.flag_control_manual_enabled && is_hovering) {

					if (manual_control_updated) {
						/* manual rates control - ACRO mode */
						Vector3f man_rate_sp(
							math::superexpo(_manual_control_sp.y, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
							math::superexpo(-_manual_control_sp.x, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
							math::superexpo(_manual_control_sp.r, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get()));
						_rates_sp = man_rate_sp.emult(_acro_rate_max);
						_thrust_sp = _manual_control_sp.z;
						publish_rates_setpoint();
					}

				} else {
					/* attitude controller disabled, poll rates setpoint topic */
					if (_v_rates_sp_sub.update(&_v_rates_sp)) {
						_rates_sp(0) = _v_rates_sp.roll;
						_rates_sp(1) = _v_rates_sp.pitch;
						_rates_sp(2) = _v_rates_sp.yaw;
						_thrust_sp = -_v_rates_sp.thrust_body[2];
					}
				}
			}

			if (_v_control_mode.flag_control_termination_enabled) {
				if (!_vehicle_status.is_vtol) {
					_rates_sp.zero();
					_rates_int.zero();
					_thrust_sp = 0.0f;
					_att_control.zero();
					publish_actuator_controls();
				}
			}

			if (attitude_updated) {
				// reset yaw setpoint during transitions, tailsitter.cpp generates
				// attitude setpoint for the transition
				reset_yaw_sp = (!attitude_setpoint_generated && !_v_control_mode.flag_control_rattitude_enabled) ||
					       _vehicle_land_detected.landed ||
					       (_vehicle_status.is_vtol && _vehicle_status.in_transition_mode);

				attitude_dt = 0.f;
			}

			/* calculate loop update rate while disarmed or at least a few times (updating the filter is expensive) */
			if (!_v_control_mode.flag_armed || (now - task_start) < 3300000) {
				dt_accumulator += dt;
				++loop_counter;

				if (dt_accumulator > 1.f) {
					const float loop_update_rate = (float)loop_counter / dt_accumulator;
					_loop_update_rate_hz = _loop_update_rate_hz * 0.5f + loop_update_rate * 0.5f;
					dt_accumulator = 0;
					loop_counter = 0;
					_lp_filters_d.set_cutoff_frequency(_loop_update_rate_hz, _param_mc_dterm_cutoff.get());
				}
			}

			parameter_update_poll();
		}

		perf_end(_loop_perf);
	}

	for (unsigned s = 0; s < _gyro_count; s++) {
		orb_unsubscribe(_sensor_gyro_sub[s]);
	}
}

int MulticopterAttitudeControl::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("mc_att_control",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_ATTITUDE_CONTROL,
				      1700,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

MulticopterAttitudeControl *MulticopterAttitudeControl::instantiate(int argc, char *argv[])
{
	return new MulticopterAttitudeControl();
}

int MulticopterAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter attitude and rate controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) or rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has two loops: a P loop for angular error and a PID loop for angular rate error.

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

### Implementation
To reduce control latency, the module directly polls on the gyro topic published by the IMU driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int mc_att_control_main(int argc, char *argv[])
{
	return MulticopterAttitudeControl::main(argc, argv);
}
