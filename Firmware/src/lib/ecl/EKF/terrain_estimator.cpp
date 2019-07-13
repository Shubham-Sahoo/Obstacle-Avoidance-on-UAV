/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file terrain_estimator.cpp
 * Function for fusing rangefinder measurements to estimate terrain vertical position/
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"
#include <ecl.h>
#include <mathlib/mathlib.h>

bool Ekf::initHagl()
{
	// get most recent range measurement from buffer
	const rangeSample &latest_measurement = _range_buffer.get_newest();

	if ((_time_last_imu - latest_measurement.time_us) < (uint64_t)2e5 && _R_rng_to_earth_2_2 > _params.range_cos_max_tilt) {
		// if we have a fresh measurement, use it to initialise the terrain estimator
		_terrain_vpos = _state.pos(2) + latest_measurement.rng * _R_rng_to_earth_2_2;
		// initialise state variance to variance of measurement
		_terrain_var = sq(_params.range_noise);
		// success
		return true;
	} else if (_flow_for_terrain_data_ready) {
		// initialise terrain vertical position to origin as this is the best guess we have
		_terrain_vpos = fmaxf(0.0f,  _state.pos(2));
		_terrain_var = 100.0f;
		return true;
	} else if (!_control_status.flags.in_air) {
		// if on ground we assume a ground clearance
		_terrain_vpos = _state.pos(2) + _params.rng_gnd_clearance;
		// Use the ground clearance value as our uncertainty
		_terrain_var = sq(_params.rng_gnd_clearance);
		// this is a guess
		return false;

	} else {
		// no information - cannot initialise
		return false;
	}
}

void Ekf::runTerrainEstimator()
{
	// Perform a continuity check on range finder data
	checkRangeDataContinuity();

	// Perform initialisation check
	if (!_terrain_initialised) {
		_terrain_initialised = initHagl();

	} else {

		// predict the state variance growth where the state is the vertical position of the terrain underneath the vehicle

		// process noise due to errors in vehicle height estimate
		_terrain_var += sq(_imu_sample_delayed.delta_vel_dt * _params.terrain_p_noise);

		// process noise due to terrain gradient
		_terrain_var += sq(_imu_sample_delayed.delta_vel_dt * _params.terrain_gradient) * (sq(_state.vel(0)) + sq(_state.vel(
					1)));

		// limit the variance to prevent it becoming badly conditioned
		_terrain_var = math::constrain(_terrain_var, 0.0f, 1e4f);

		// Fuse range finder data if available
		if (_range_data_ready && !_rng_hgt_faulty) {
			fuseHagl();

			// update range sensor angle parameters in case they have changed
			// we do this here to avoid doing those calculations at a high rate
			_sin_tilt_rng = sinf(_params.rng_sens_pitch);
			_cos_tilt_rng = cosf(_params.rng_sens_pitch);

		}

		if (_flow_for_terrain_data_ready) {
			fuseFlowForTerrain();
			_flow_for_terrain_data_ready = false;
		}

		// constrain _terrain_vpos to be a minimum of _params.rng_gnd_clearance larger than _state.pos(2)
		if (_terrain_vpos - _state.pos(2) < _params.rng_gnd_clearance) {
			_terrain_vpos = _params.rng_gnd_clearance + _state.pos(2);
		}
	}

	// Update terrain validity
	update_terrain_valid();
}

void Ekf::fuseHagl()
{
	// If the vehicle is excessively tilted, do not try to fuse range finder observations
	if (_R_rng_to_earth_2_2 > _params.range_cos_max_tilt) {
		// get a height above ground measurement from the range finder assuming a flat earth
		float meas_hagl = _range_sample_delayed.rng * _R_rng_to_earth_2_2;

		// predict the hagl from the vehicle position and terrain height
		float pred_hagl = _terrain_vpos - _state.pos(2);

		// calculate the innovation
		_hagl_innov = pred_hagl - meas_hagl;

		// calculate the observation variance adding the variance of the vehicles own height uncertainty
		float obs_variance = fmaxf(P[9][9] * _params.vehicle_variance_scaler, 0.0f) + sq(_params.range_noise) + sq(_params.range_noise_scaler * _range_sample_delayed.rng);

		// calculate the innovation variance - limiting it to prevent a badly conditioned fusion
		_hagl_innov_var = fmaxf(_terrain_var + obs_variance, obs_variance);

		// perform an innovation consistency check and only fuse data if it passes
		float gate_size = fmaxf(_params.range_innov_gate, 1.0f);
		_terr_test_ratio = sq(_hagl_innov) / (sq(gate_size) * _hagl_innov_var);

		if (_terr_test_ratio <= 1.0f) {
			// calculate the Kalman gain
			float gain = _terrain_var / _hagl_innov_var;
			// correct the state
			_terrain_vpos -= gain * _hagl_innov;
			// correct the variance
			_terrain_var = fmaxf(_terrain_var * (1.0f - gain), 0.0f);
			// record last successful fusion event
			_time_last_hagl_fuse = _time_last_imu;
			_innov_check_fail_status.flags.reject_hagl = false;
		} else {
			// If we have been rejecting range data for too long, reset to measurement
			if ((_time_last_imu - _time_last_hagl_fuse) > (uint64_t)10E6) {
				_terrain_vpos = _state.pos(2) + meas_hagl;
				_terrain_var = obs_variance;
			} else {
				_innov_check_fail_status.flags.reject_hagl = true;
			}
		}
	} else {
		_innov_check_fail_status.flags.reject_hagl = true;
		return;
	}
}

void Ekf::fuseFlowForTerrain()
{
	// calculate optical LOS rates using optical flow rates that have had the body angular rate contribution removed
	// correct for gyro bias errors in the data used to do the motion compensation
	// Note the sign convention used: A positive LOS rate is a RH rotation of the scene about that axis.
	Vector2f opt_flow_rate;
	opt_flow_rate(0) = _flowRadXYcomp(0) / _flow_sample_delayed.dt + _flow_gyro_bias(0);
	opt_flow_rate(1) = _flowRadXYcomp(1) / _flow_sample_delayed.dt + _flow_gyro_bias(1);

	// get latest estimated orientation
	float q0 = _state.quat_nominal(0);
	float q1 = _state.quat_nominal(1);
	float q2 = _state.quat_nominal(2);
	float q3 = _state.quat_nominal(3);

	// calculate the optical flow observation variance
	float R_LOS = calcOptFlowMeasVar();

	// get rotation matrix from earth to body
	Dcmf earth_to_body(_state.quat_nominal);
	earth_to_body = earth_to_body.transpose();

	// calculate the sensor position relative to the IMU
	Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

	// calculate the velocity of the sensor relative to the imu in body frame
	// Note: _flow_sample_delayed.gyroXYZ is the negative of the body angular velocity, thus use minus sign
	Vector3f vel_rel_imu_body = cross_product(-_flow_sample_delayed.gyroXYZ / _flow_sample_delayed.dt, pos_offset_body);

	// calculate the velocity of the sensor in the earth frame
	Vector3f vel_rel_earth = _state.vel + _R_to_earth * vel_rel_imu_body;

	// rotate into body frame
	Vector3f vel_body = earth_to_body * vel_rel_earth;

	float t0 = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	// constrain terrain to minimum allowed value and predict height above ground
	_terrain_vpos = fmaxf(_terrain_vpos, _params.rng_gnd_clearance + _state.pos(2));
	float pred_hagl = _terrain_vpos - _state.pos(2);

	// Calculate observation matrix for flow around the vehicle x axis
	float Hx = vel_body(1) * t0 /(pred_hagl * pred_hagl);

	// Constrain terrain variance to be non-negative
	_terrain_var = fmaxf(_terrain_var, 0.0f);

	// Cacluate innovation variance
	_flow_innov_var[0] = Hx * Hx * _terrain_var + R_LOS;

	// calculate the kalman gain for the flow x measurement
	float Kx = _terrain_var * Hx / _flow_innov_var[0];

	// calculate prediced optical flow about x axis
	float pred_flow_x = vel_body(1) * earth_to_body(2,2) / pred_hagl;

	// calculate flow innovation (x axis)
	_flow_innov[0] = pred_flow_x - opt_flow_rate(0);

	// calculate correction term for terrain variance
	float KxHxP =  Kx * Hx * _terrain_var;

	// innovation consistency check
	float gate_size = fmaxf(_params.flow_innov_gate, 1.0f);
	float flow_test_ratio = sq(_flow_innov[0]) / (sq(gate_size) * _flow_innov_var[0]);

	// do not perform measurement update if badly conditioned
	if (flow_test_ratio <= 1.0f) {
		_terrain_vpos += Kx * _flow_innov[0];
		// guard against negative variance
		_terrain_var = fmaxf(_terrain_var - KxHxP, 0.0f);
		_time_last_of_fuse = _time_last_imu;
	}

	// Calculate observation matrix for flow around the vehicle y axis
	float Hy = -vel_body(0) * t0 /(pred_hagl * pred_hagl);

	// Calculuate innovation variance
	_flow_innov_var[1] = Hy * Hy * _terrain_var + R_LOS;

	// calculate the kalman gain for the flow y measurement
	float Ky = _terrain_var * Hy / _flow_innov_var[1];

	// calculate prediced optical flow about y axis
	float pred_flow_y = -vel_body(0) * earth_to_body(2,2) / pred_hagl;

	// calculate flow innovation (y axis)
	_flow_innov[1] = pred_flow_y - opt_flow_rate(1);

	// calculate correction term for terrain variance
	float KyHyP =  Ky * Hy * _terrain_var;

	// innovation consistency check
	flow_test_ratio = sq(_flow_innov[1]) / (sq(gate_size) * _flow_innov_var[1]);

	if (flow_test_ratio <= 1.0f) {
		_terrain_vpos += Ky * _flow_innov[1];
		// guard against negative variance
		_terrain_var = fmaxf(_terrain_var - KyHyP, 0.0f);
		_time_last_of_fuse = _time_last_imu;
	}
}

// return true if the terrain height estimate is valid
bool Ekf::get_terrain_valid()
{
	return _hagl_valid;
}

// determine terrain validity
void Ekf::update_terrain_valid()
{
	// we have been fusing range finder measurements in the last 5 seconds
	bool recent_range_fusion = (_time_last_imu - _time_last_hagl_fuse) < 5*1000*1000;

	// we have been fusing optical flow measurements for terrain estimation within the last 5 seconds
	// this can only be the case if the main filter does not fuse optical flow
	bool recent_flow_for_terrain_fusion = ((_time_last_imu - _time_last_of_fuse) < 5*1000*1000) && !_control_status.flags.opt_flow;


	if (_terrain_initialised && (recent_range_fusion || recent_flow_for_terrain_fusion)) {

		_hagl_valid = true;

	} else {
		_hagl_valid = false;
	}
}

// get the estimated vertical position of the terrain relative to the NED origin
void Ekf::get_terrain_vert_pos(float *ret)
{
	memcpy(ret, &_terrain_vpos, sizeof(float));
}

void Ekf::get_hagl_innov(float *hagl_innov)
{
	memcpy(hagl_innov, &_hagl_innov, sizeof(_hagl_innov));
}


void Ekf::get_hagl_innov_var(float *hagl_innov_var)
{
	memcpy(hagl_innov_var, &_hagl_innov_var, sizeof(_hagl_innov_var));
}

// check that the range finder data is continuous
void Ekf::checkRangeDataContinuity()
{
	// update range data continuous flag (1Hz ie 2000 ms)
	/* Timing in micro seconds */

	/* Apply a 2.0 sec low pass filter to the time delta from the last range finder updates */
	float alpha = 0.5f * _dt_update;
	_dt_last_range_update_filt_us = _dt_last_range_update_filt_us * (1.0f - alpha) + alpha *
					(_imu_sample_delayed.time_us - _range_sample_delayed.time_us);

	_dt_last_range_update_filt_us = fminf(_dt_last_range_update_filt_us, 4e6f);

	if (_dt_last_range_update_filt_us < 2e6f) {
		_range_data_continuous = true;

	} else {
		_range_data_continuous = false;
	}
}
