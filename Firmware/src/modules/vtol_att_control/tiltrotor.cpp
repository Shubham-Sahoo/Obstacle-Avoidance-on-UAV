/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file tiltrotor.cpp
 *
 * @author Roman Bapst 		<bapstroman@gmail.com>
 * @author Andreas Antener 	<andreas@uaventure.com>
 *
*/

#include "tiltrotor.h"
#include "vtol_att_control_main.h"

#define ARSP_YAW_CTRL_DISABLE 7.0f	// airspeed at which we stop controlling yaw during a front transition

Tiltrotor::Tiltrotor(VtolAttitudeControl *attc) :
	VtolType(attc)
{
	_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
	_vtol_schedule.transition_start = 0;

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;

	_flag_was_in_trans_mode = false;

	_params_handles_tiltrotor.tilt_mc = param_find("VT_TILT_MC");
	_params_handles_tiltrotor.tilt_transition = param_find("VT_TILT_TRANS");
	_params_handles_tiltrotor.tilt_fw = param_find("VT_TILT_FW");
	_params_handles_tiltrotor.front_trans_dur_p2 = param_find("VT_TRANS_P2_DUR");
}

void
Tiltrotor::parameters_update()
{
	float v;

	/* vtol tilt mechanism position in mc mode */
	param_get(_params_handles_tiltrotor.tilt_mc, &v);
	_params_tiltrotor.tilt_mc = v;

	/* vtol tilt mechanism position in transition mode */
	param_get(_params_handles_tiltrotor.tilt_transition, &v);
	_params_tiltrotor.tilt_transition = v;

	/* vtol tilt mechanism position in fw mode */
	param_get(_params_handles_tiltrotor.tilt_fw, &v);
	_params_tiltrotor.tilt_fw = v;

	/* vtol front transition phase 2 duration */
	param_get(_params_handles_tiltrotor.front_trans_dur_p2, &v);
	_params_tiltrotor.front_trans_dur_p2 = v;
}

void Tiltrotor::update_vtol_state()
{
	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting rotors, picking up
	 * forward speed. After the vehicle has picked up enough speed the rotors are tilted
	 * forward completely. For the backtransition the motors simply rotate back.
	*/

	if (!_attc->is_fixed_wing_requested()) {

		// plane is in multicopter mode
		switch (_vtol_schedule.flight_mode) {
		case vtol_mode::MC_MODE:
			break;

		case vtol_mode::FW_MODE:
			_vtol_schedule.flight_mode = vtol_mode::TRANSITION_BACK;
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case vtol_mode::TRANSITION_FRONT_P1:
			// failsafe into multicopter mode
			_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
			break;

		case vtol_mode::TRANSITION_FRONT_P2:
			// failsafe into multicopter mode
			_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
			break;

		case vtol_mode::TRANSITION_BACK:
			float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

			if (_tilt_control <= _params_tiltrotor.tilt_mc && time_since_trans_start > _params->back_trans_duration) {
				_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
			}

			break;
		}

	} else {

		switch (_vtol_schedule.flight_mode) {
		case vtol_mode::MC_MODE:
			// initialise a front transition
			_vtol_schedule.flight_mode = vtol_mode::TRANSITION_FRONT_P1;
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case vtol_mode::FW_MODE:
			break;

		case vtol_mode::TRANSITION_FRONT_P1: {
				// allow switch if we are not armed for the sake of bench testing
				bool transition_to_p2 = can_transition_on_ground();

				float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

				// check if we have reached airspeed to switch to fw mode
				transition_to_p2 |= !_params->airspeed_disabled &&
						    _airspeed->indicated_airspeed_m_s >= _params->transition_airspeed &&
						    time_since_trans_start > _params->front_trans_time_min;

				// check if airspeed is invalid and transition by time
				transition_to_p2 |= _params->airspeed_disabled &&
						    _tilt_control >= _params_tiltrotor.tilt_transition &&
						    time_since_trans_start > _params->front_trans_time_openloop;

				if (transition_to_p2) {
					_vtol_schedule.flight_mode = vtol_mode::TRANSITION_FRONT_P2;
					_vtol_schedule.transition_start = hrt_absolute_time();
				}

				break;
			}

		case vtol_mode::TRANSITION_FRONT_P2:

			// if the rotors have been tilted completely we switch to fw mode
			if (_tilt_control >= _params_tiltrotor.tilt_fw) {
				_vtol_schedule.flight_mode = vtol_mode::FW_MODE;
				_tilt_control = _params_tiltrotor.tilt_fw;
			}

			break;

		case vtol_mode::TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_schedule.flight_mode = vtol_mode::FW_MODE;
			break;
		}
	}

	// map tiltrotor specific control phases to simple control modes
	switch (_vtol_schedule.flight_mode) {
	case vtol_mode::MC_MODE:
		_vtol_mode = mode::ROTARY_WING;
		break;

	case vtol_mode::FW_MODE:
		_vtol_mode = mode::FIXED_WING;
		break;

	case vtol_mode::TRANSITION_FRONT_P1:
	case vtol_mode::TRANSITION_FRONT_P2:
		_vtol_mode = mode::TRANSITION_TO_FW;
		break;

	case vtol_mode::TRANSITION_BACK:
		_vtol_mode = mode::TRANSITION_TO_MC;
		break;
	}
}

void Tiltrotor::update_mc_state()
{
	VtolType::update_mc_state();

	// make sure motors are not tilted
	_tilt_control = _params_tiltrotor.tilt_mc;
}

void Tiltrotor::update_fw_state()
{
	VtolType::update_fw_state();

	// make sure motors are tilted forward
	_tilt_control = _params_tiltrotor.tilt_fw;
}

void Tiltrotor::update_transition_state()
{
	VtolType::update_transition_state();

	float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

	if (!_flag_was_in_trans_mode) {
		// save desired heading for transition and last thrust value
		_flag_was_in_trans_mode = true;
	}

	if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_FRONT_P1) {
		// for the first part of the transition the rear rotors are enabled
		if (_motor_state != motor_state::ENABLED) {
			_motor_state = set_motor_state(_motor_state, motor_state::ENABLED);
		}

		// tilt rotors forward up to certain angle
		if (_tilt_control <= _params_tiltrotor.tilt_transition) {
			_tilt_control = _params_tiltrotor.tilt_mc +
					fabsf(_params_tiltrotor.tilt_transition - _params_tiltrotor.tilt_mc) * time_since_trans_start /
					_params->front_trans_duration;
		}


		// at low speeds give full weight to MC
		_mc_roll_weight = 1.0f;
		_mc_yaw_weight = 1.0f;

		// reduce MC controls once the plane has picked up speed
		if (!_params->airspeed_disabled && _airspeed->indicated_airspeed_m_s > ARSP_YAW_CTRL_DISABLE) {
			_mc_yaw_weight = 0.0f;
		}

		if (!_params->airspeed_disabled && _airspeed->indicated_airspeed_m_s >= _params->airspeed_blend) {
			_mc_roll_weight = 1.0f - (_airspeed->indicated_airspeed_m_s - _params->airspeed_blend) /
					  (_params->transition_airspeed - _params->airspeed_blend);
		}

		// without airspeed do timed weight changes
		if (_params->airspeed_disabled
		    && time_since_trans_start > _params->front_trans_time_min) {
			_mc_roll_weight = 1.0f - (time_since_trans_start - _params->front_trans_time_min) /
					  (_params->front_trans_time_openloop - _params->front_trans_time_min);
			_mc_yaw_weight = _mc_roll_weight;
		}

		_thrust_transition = -_mc_virtual_att_sp->thrust_body[2];

	} else if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_FRONT_P2) {
		// the plane is ready to go into fixed wing mode, tilt the rotors forward completely
		_tilt_control = _params_tiltrotor.tilt_transition +
				fabsf(_params_tiltrotor.tilt_fw - _params_tiltrotor.tilt_transition) * time_since_trans_start /
				_params_tiltrotor.front_trans_dur_p2;

		_mc_roll_weight = 0.0f;
		_mc_yaw_weight = 0.0f;

		// ramp down rear motors (setting MAX_PWM down scales the given output into the new range)
		int rear_value = (1.0f - time_since_trans_start / _params_tiltrotor.front_trans_dur_p2) *
				 (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) + PWM_DEFAULT_MIN;


		_motor_state = set_motor_state(_motor_state, motor_state::VALUE, rear_value);


		_thrust_transition = -_mc_virtual_att_sp->thrust_body[2];

	} else if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_BACK) {
		if (_motor_state != motor_state::ENABLED) {
			_motor_state = set_motor_state(_motor_state, motor_state::ENABLED);
		}


		// set idle speed for rotary wing mode
		if (!flag_idle_mc) {
			flag_idle_mc = set_idle_mc();
		}

		// tilt rotors back
		if (_tilt_control > _params_tiltrotor.tilt_mc) {
			_tilt_control = _params_tiltrotor.tilt_fw -
					fabsf(_params_tiltrotor.tilt_fw - _params_tiltrotor.tilt_mc) * time_since_trans_start / 1.0f;
		}

		_mc_yaw_weight = 1.0f;

		// while we quickly rotate back the motors keep throttle at idle
		if (time_since_trans_start < 1.0f) {
			_mc_throttle_weight = 0.0f;
			_mc_roll_weight = 0.0f;
			_mc_pitch_weight = 0.0f;

		} else {
			_mc_roll_weight = 1.0f;
			_mc_pitch_weight = 1.0f;
			// slowly ramp up throttle to avoid step inputs
			_mc_throttle_weight = (time_since_trans_start - 1.0f) / 1.0f;
		}
	}

	_mc_roll_weight = math::constrain(_mc_roll_weight, 0.0f, 1.0f);
	_mc_yaw_weight = math::constrain(_mc_yaw_weight, 0.0f, 1.0f);
	_mc_throttle_weight = math::constrain(_mc_throttle_weight, 0.0f, 1.0f);

	// copy virtual attitude setpoint to real attitude setpoint (we use multicopter att sp)
	memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));
}

void Tiltrotor::waiting_on_tecs()
{
	// keep multicopter thrust until we get data from TECS
	_v_att_sp->thrust_body[0] = _thrust_transition;
}

/**
* Write data to actuator output topic.
*/
void Tiltrotor::fill_actuator_outputs()
{
	// Multirotor output
	_actuators_out_0->timestamp = hrt_absolute_time();
	_actuators_out_0->timestamp_sample = _actuators_mc_in->timestamp_sample;

	_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_ROLL] * _mc_roll_weight;
	_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight;
	_actuators_out_0->control[actuator_controls_s::INDEX_YAW] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_YAW] * _mc_yaw_weight;

	if (_vtol_schedule.flight_mode == vtol_mode::FW_MODE) {
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];

		/* allow differential thrust if enabled */
		if (_params->diff_thrust == 1) {
			_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
				_actuators_fw_in->control[actuator_controls_s::INDEX_YAW] * _params->diff_thrust_scale;
		}

	} else {
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE] * _mc_throttle_weight;
	}

	// Fixed wing output
	_actuators_out_1->timestamp = hrt_absolute_time();
	_actuators_out_1->timestamp_sample = _actuators_fw_in->timestamp_sample;

	_actuators_out_1->control[4] = _tilt_control;

	if (_params->elevons_mc_lock && _vtol_schedule.flight_mode == vtol_mode::MC_MODE) {
		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] = 0.0f;
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] = 0.0f;
		_actuators_out_1->control[actuator_controls_s::INDEX_YAW] = 0.0f;

	} else {
		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL];
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH];
		_actuators_out_1->control[actuator_controls_s::INDEX_YAW] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_YAW];
	}
}
