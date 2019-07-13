/****************************************************************************
 *
 *   Copyright (c) 2015-2018 PX4 Development Team. All rights reserved.
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
 * @file mavlink_high_latency2.cpp
 *
 * @author Achermann Florian <acfloria@ethz.ch>
 */

#include "mavlink_high_latency2.h"

#include <commander/px4_custom_mode.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/uORB.h>

using matrix::wrap_2pi;

MavlinkStreamHighLatency2::MavlinkStreamHighLatency2(Mavlink *mavlink) : MavlinkStream(mavlink),
	_actuator_sub_0(_mavlink->add_orb_subscription(ORB_ID(actuator_controls_0))),
	_actuator_time_0(0),
	_actuator_sub_1(_mavlink->add_orb_subscription(ORB_ID(actuator_controls_1))),
	_actuator_time_1(0),
	_airspeed_sub(_mavlink->add_orb_subscription(ORB_ID(airspeed))),
	_airspeed_time(0),
	_attitude_sp_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_attitude_setpoint))),
	_attitude_sp_time(0),
	_battery_sub(_mavlink->add_orb_subscription(ORB_ID(battery_status))),
	_battery_time(0),
	_estimator_status_sub(_mavlink->add_orb_subscription(ORB_ID(estimator_status))),
	_estimator_status_time(0),
	_pos_ctrl_status_sub(_mavlink->add_orb_subscription(ORB_ID(position_controller_status))),
	_pos_ctrl_status_time(0),
	_geofence_sub(_mavlink->add_orb_subscription(ORB_ID(geofence_result))),
	_geofence_time(0),
	_global_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_global_position))),
	_global_pos_time(0),
	_gps_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_gps_position))),
	_gps_time(0),
	_mission_result_sub(_mavlink->add_orb_subscription(ORB_ID(mission_result))),
	_mission_result_time(0),
	_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status))),
	_status_time(0),
	_status_flags_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status_flags))),
	_status_flags_time(0),
	_tecs_status_sub(_mavlink->add_orb_subscription(ORB_ID(tecs_status))),
	_tecs_time(0),
	_wind_sub(_mavlink->add_orb_subscription(ORB_ID(wind_estimate))),
	_wind_time(0),
	_airspeed(SimpleAnalyzer::AVERAGE),
	_airspeed_sp(SimpleAnalyzer::AVERAGE),
	_battery(SimpleAnalyzer::AVERAGE),
	_climb_rate(SimpleAnalyzer::MAX),
	_eph(SimpleAnalyzer::MAX),
	_epv(SimpleAnalyzer::MAX),
	_groundspeed(SimpleAnalyzer::AVERAGE),
	_temperature(SimpleAnalyzer::AVERAGE),
	_throttle(SimpleAnalyzer::AVERAGE),
	_windspeed(SimpleAnalyzer::AVERAGE)
{
	reset_last_sent();
}

bool MavlinkStreamHighLatency2::send(const hrt_abstime t)
{
	// only send the struct if transmitting is allowed
	// this assures that the stream timer is only reset when actually a message is transmitted
	if (_mavlink->should_transmit()) {
		mavlink_high_latency2_t msg = {};
		set_default_values(msg);

		bool updated = _airspeed.valid();
		updated |= _airspeed_sp.valid();
		updated |= _battery.valid();
		updated |= _climb_rate.valid();
		updated |= _eph.valid();
		updated |= _epv.valid();
		updated |= _groundspeed.valid();
		updated |= _temperature.valid();
		updated |= _throttle.valid();
		updated |= _windspeed.valid();
		updated |= write_airspeed(&msg);
		updated |= write_attitude_sp(&msg);
		updated |= write_battery_status(&msg);
		updated |= write_estimator_status(&msg);
		updated |= write_fw_ctrl_status(&msg);
		updated |= write_geofence_result(&msg);
		updated |= write_global_position(&msg);
		updated |= write_mission_result(&msg);
		updated |= write_tecs_status(&msg);
		updated |= write_vehicle_status(&msg);
		updated |= write_vehicle_status_flags(&msg);
		updated |= write_wind_estimate(&msg);

		if (updated) {
			msg.timestamp = t / 1000;

			msg.type = _mavlink->get_system_type();
			msg.autopilot = MAV_AUTOPILOT_PX4;

			if (_airspeed.valid()) {
				_airspeed.get_scaled(msg.airspeed, 5.0f);
			}

			if (_airspeed_sp.valid()) {
				_airspeed_sp.get_scaled(msg.airspeed_sp, 5.0f);
			}

			if (_battery.valid()) {
				_battery.get_scaled(msg.battery, 100.0f);
			}

			if (_climb_rate.valid()) {
				_climb_rate.get_scaled(msg.climb_rate, 10.0f);
			}

			if (_eph.valid()) {
				_eph.get_scaled(msg.eph, 10.0f);
			}

			if (_epv.valid()) {
				_epv.get_scaled(msg.epv, 10.0f);
			}

			if (_groundspeed.valid()) {
				_groundspeed.get_scaled(msg.groundspeed, 5.0f);
			}

			if (_temperature.valid()) {
				_temperature.get_scaled(msg.temperature_air, 1.0f);
			}

			if (_throttle.valid()) {
				_throttle.get_scaled(msg.throttle, 100.0f);
			}

			if (_windspeed.valid()) {
				_windspeed.get_scaled(msg.windspeed, 5.0f);
			}

			reset_analysers(t);

			mavlink_msg_high_latency2_send_struct(_mavlink->get_channel(), &msg);
		}

		return updated;

	} else {
		// reset the analysers every 60 seconds if nothing should be transmitted
		if ((t - _last_reset_time) > 60000000) {
			reset_analysers(t);
			PX4_DEBUG("Resetting HIGH_LATENCY2 analysers");
		}

		return false;
	}
}

void MavlinkStreamHighLatency2::reset_analysers(const hrt_abstime t)
{
	// reset the analyzers
	_airspeed.reset();
	_airspeed_sp.reset();
	_battery.reset();
	_climb_rate.reset();
	_eph.reset();
	_epv.reset();
	_groundspeed.reset();
	_temperature.reset();
	_throttle.reset();
	_windspeed.reset();

	_last_reset_time = t;
}

bool MavlinkStreamHighLatency2::write_airspeed(mavlink_high_latency2_t *msg)
{
	struct airspeed_s airspeed;

	const bool updated = _airspeed_sub->update(&_airspeed_time, &airspeed);

	if (_airspeed_time > 0) {
		if (airspeed.confidence < 0.95f) { // the same threshold as for the commander
			msg->failure_flags |= HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE;
		}
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_attitude_sp(mavlink_high_latency2_t *msg)
{
	struct vehicle_attitude_setpoint_s attitude_sp;

	const bool updated = _attitude_sp_sub->update(&_attitude_sp_time, &attitude_sp);

	if (_attitude_sp_time > 0) {
		msg->target_heading = static_cast<uint8_t>(math::degrees(wrap_2pi(attitude_sp.yaw_body)) * 0.5f);
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_battery_status(mavlink_high_latency2_t *msg)
{
	struct battery_status_s battery;

	const bool updated = _battery_sub->update(&_battery_time, &battery);

	if (_battery_time > 0) {
		if (battery.warning > battery_status_s::BATTERY_WARNING_LOW) {
			msg->failure_flags |= HL_FAILURE_FLAG_BATTERY;
		}
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_estimator_status(mavlink_high_latency2_t *msg)
{
	struct estimator_status_s estimator_status;

	const bool updated = _estimator_status_sub->update(&_estimator_status_time, &estimator_status);

	if (_estimator_status_time > 0) {
		if (estimator_status.gps_check_fail_flags > 0 ||
		    estimator_status.filter_fault_flags > 0 ||
		    estimator_status.innovation_check_flags > 0) {
			msg->failure_flags |= HL_FAILURE_FLAG_ESTIMATOR;
		}
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_fw_ctrl_status(mavlink_high_latency2_t *msg)
{
	position_controller_status_s pos_ctrl_status = {};

	const bool updated = _pos_ctrl_status_sub->update(&_pos_ctrl_status_time, &pos_ctrl_status);

	if (_pos_ctrl_status_time > 0) {
		uint16_t target_distance;
		convert_limit_safe(pos_ctrl_status.wp_dist * 0.1f, target_distance);
		msg->target_distance = target_distance;
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_geofence_result(mavlink_high_latency2_t *msg)
{
	struct geofence_result_s geofence;

	const bool updated = _geofence_sub->update(&_geofence_time, &geofence);

	if (_geofence_time > 0) {
		if (geofence.geofence_violated) {
			msg->failure_flags |= HL_FAILURE_FLAG_GEOFENCE;
		}
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_global_position(mavlink_high_latency2_t *msg)
{
	struct vehicle_global_position_s global_pos;

	const bool updated = _global_pos_sub->update(&_global_pos_time, &global_pos);

	if (_global_pos_time > 0) {
		msg->latitude = global_pos.lat * 1e7;
		msg->longitude = global_pos.lon * 1e7;

		int16_t altitude = 0;

		if (global_pos.alt > 0) {
			convert_limit_safe(global_pos.alt + 0.5f, altitude);

		} else {
			convert_limit_safe(global_pos.alt - 0.5f, altitude);
		}

		msg->altitude = altitude;

		msg->heading = static_cast<uint8_t>(math::degrees(wrap_2pi(global_pos.yaw)) * 0.5f);
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_mission_result(mavlink_high_latency2_t *msg)
{
	struct mission_result_s mission_result;

	const bool updated = _mission_result_sub->update(&_mission_result_time, &mission_result);

	if (_mission_result_time > 0) {
		msg->wp_num = mission_result.seq_current;
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_tecs_status(mavlink_high_latency2_t *msg)
{
	struct tecs_status_s tecs_status;

	const bool updated = _tecs_status_sub->update(&_tecs_time, &tecs_status);

	if (_tecs_time > 0) {
		int16_t target_altitude;
		convert_limit_safe(tecs_status.altitude_sp, target_altitude);
		msg->target_altitude = target_altitude;
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_vehicle_status(mavlink_high_latency2_t *msg)
{
	struct vehicle_status_s status;

	const bool updated = _status_sub->update(&_status_time, &status);

	if (_status_time > 0) {
		if ((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE)
		    && !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE)) {
			msg->failure_flags |= HL_FAILURE_FLAG_ABSOLUTE_PRESSURE;
		}

		if (((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_ACCEL)
		     && !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_ACCEL)) ||
		    ((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_ACCEL2) &&
		     !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_ACCEL2))) {
			msg->failure_flags |= HL_FAILURE_FLAG_3D_ACCEL;
		}

		if (((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_GYRO)
		     && !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_GYRO)) ||
		    ((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_GYRO2) &&
		     !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_GYRO2))) {
			msg->failure_flags |= HL_FAILURE_FLAG_3D_GYRO;
		}

		if (((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_MAG)
		     && !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_MAG)) ||
		    ((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_MAG2) &&
		     !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_MAG2))) {
			msg->failure_flags |= HL_FAILURE_FLAG_3D_MAG;
		}

		if ((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_TERRAIN)
		    && !(status.onboard_control_sensors_health & MAV_SYS_STATUS_TERRAIN)) {
			msg->failure_flags |= HL_FAILURE_FLAG_TERRAIN;
		}

		if (status.rc_signal_lost) {
			msg->failure_flags |= HL_FAILURE_FLAG_RC_RECEIVER;
		}

		if (status.engine_failure) {
			msg->failure_flags |= HL_FAILURE_FLAG_ENGINE;
		}

		if (status.mission_failure) {
			msg->failure_flags |= HL_FAILURE_FLAG_MISSION;
		}

		// flight mode
		union px4_custom_mode custom_mode;
		uint8_t mavlink_base_mode;
		get_mavlink_navigation_mode(&status, &mavlink_base_mode, &custom_mode);
		msg->custom_mode = custom_mode.custom_mode_hl;
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_vehicle_status_flags(mavlink_high_latency2_t *msg)
{
	struct vehicle_status_flags_s status_flags;

	const bool updated = _status_flags_sub->update(&_status_flags_time, &status_flags);

	if (_status_flags_time > 0) {
		if (!status_flags.condition_global_position_valid) { //TODO check if there is a better way to get only GPS failure
			msg->failure_flags |= HL_FAILURE_FLAG_GPS;
		}

		if (status_flags.offboard_control_signal_lost && status_flags.offboard_control_signal_found_once) {
			msg->failure_flags |= HL_FAILURE_FLAG_OFFBOARD_LINK;
		}
	}

	return updated;
}

bool MavlinkStreamHighLatency2::write_wind_estimate(mavlink_high_latency2_t *msg)
{
	struct wind_estimate_s wind;

	const bool updated = _wind_sub->update(&_wind_time, &wind);

	if (_wind_time > 0) {
		msg->wind_heading = static_cast<uint8_t>(
					    math::degrees(wrap_2pi(atan2f(wind.windspeed_east, wind.windspeed_north))) * 0.5f);
	}

	return updated;
}

void MavlinkStreamHighLatency2::update_data()
{
	const hrt_abstime t = hrt_absolute_time();

	if (t > _last_update_time) {
		// first order low pass filter for the update rate
		_update_rate_filtered = 0.97f * _update_rate_filtered + 0.03f / ((t - _last_update_time) * 1e-6f);
		_last_update_time = t;
	}

	update_airspeed();

	update_tecs_status();

	update_battery_status();

	update_global_position();

	update_gps();

	update_vehicle_status();

	update_wind_estimate();
}

void MavlinkStreamHighLatency2::update_airspeed()
{
	airspeed_s airspeed;

	if (_airspeed_sub->update(&airspeed)) {
		_airspeed.add_value(airspeed.indicated_airspeed_m_s, _update_rate_filtered);
		_temperature.add_value(airspeed.air_temperature_celsius, _update_rate_filtered);
	}
}

void MavlinkStreamHighLatency2::update_tecs_status()
{
	tecs_status_s tecs_status;

	if (_tecs_status_sub->update(&tecs_status)) {
		_airspeed_sp.add_value(tecs_status.airspeed_sp, _update_rate_filtered);
	}
}

void MavlinkStreamHighLatency2::update_battery_status()
{
	battery_status_s battery;

	if (_battery_sub->update(&battery)) {
		_battery.add_value(battery.remaining, _update_rate_filtered);
	}
}

void MavlinkStreamHighLatency2::update_global_position()
{
	vehicle_global_position_s global_pos;

	if (_global_pos_sub->update(&global_pos)) {
		_climb_rate.add_value(fabsf(global_pos.vel_d), _update_rate_filtered);
		_groundspeed.add_value(sqrtf(global_pos.vel_n * global_pos.vel_n + global_pos.vel_e * global_pos.vel_e),
				       _update_rate_filtered);
	}
}

void MavlinkStreamHighLatency2::update_gps()
{
	vehicle_gps_position_s gps;

	if (_gps_sub->update(&gps)) {
		_eph.add_value(gps.eph, _update_rate_filtered);
		_epv.add_value(gps.epv, _update_rate_filtered);
	}
}

void MavlinkStreamHighLatency2::update_vehicle_status()
{
	vehicle_status_s status;

	if (_status_sub->update(&status)) {
		if (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			struct actuator_controls_s actuator = {};

			if (status.is_vtol && status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
				if (_actuator_sub_1->update(&actuator)) {
					_throttle.add_value(actuator.control[actuator_controls_s::INDEX_THROTTLE], _update_rate_filtered);
				}

			} else {
				if (_actuator_sub_0->update(&actuator)) {
					_throttle.add_value(actuator.control[actuator_controls_s::INDEX_THROTTLE], _update_rate_filtered);
				}
			}

		} else {
			_throttle.add_value(0.0f, _update_rate_filtered);
		}
	}
}

void MavlinkStreamHighLatency2::update_wind_estimate()
{
	wind_estimate_s wind;

	if (_wind_sub->update(&wind)) {
		_windspeed.add_value(sqrtf(wind.windspeed_north * wind.windspeed_north + wind.windspeed_east * wind.windspeed_east),
				     _update_rate_filtered);
	}
}

void MavlinkStreamHighLatency2::set_default_values(mavlink_high_latency2_t &msg) const
{
	msg.airspeed = 0;
	msg.airspeed_sp = 0;
	msg.altitude = 0;
	msg.autopilot = MAV_AUTOPILOT_ENUM_END;
	msg.battery = -1;
	msg.climb_rate = 0;
	msg.custom0 = INT8_MIN;
	msg.custom1 = INT8_MIN;
	msg.custom2 = INT8_MIN;
	msg.eph = UINT8_MAX;
	msg.epv = UINT8_MAX;
	msg.failure_flags = 0;
	msg.custom_mode = 0;
	msg.groundspeed = 0;
	msg.heading = 0;
	msg.latitude = 0;
	msg.longitude = 0;
	msg.target_altitude = 0;
	msg.target_distance = 0;
	msg.target_heading = 0;
	msg.temperature_air = INT8_MIN;
	msg.throttle = 0;
	msg.timestamp = 0;
	msg.type = MAV_TYPE_ENUM_END;
	msg.wind_heading = 0;
	msg.windspeed = 0;
	msg.wp_num = UINT16_MAX;
}
