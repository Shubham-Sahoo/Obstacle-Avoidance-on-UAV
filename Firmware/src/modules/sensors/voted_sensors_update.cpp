/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file voted_sensors_update.cpp
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */

#include "voted_sensors_update.h"

#include <systemlib/mavlink_log.h>

#include <uORB/Subscription.hpp>
#include <conversion/rotation.h>
#include <ecl/geo/geo.h>

#define MAG_ROT_VAL_INTERNAL		-1
#define CAL_ERROR_APPLY_CAL_MSG "FAILED APPLYING %s CAL #%u"

using namespace sensors;
using namespace DriverFramework;

VotedSensorsUpdate::VotedSensorsUpdate(const Parameters &parameters, bool hil_enabled)
	: _parameters(parameters), _hil_enabled(hil_enabled)
{
	memset(&_last_sensor_data, 0, sizeof(_last_sensor_data));
	memset(&_last_magnetometer, 0, sizeof(_last_magnetometer));
	memset(&_last_airdata, 0, sizeof(_last_airdata));
	memset(&_last_accel_timestamp, 0, sizeof(_last_accel_timestamp));
	memset(&_accel_diff, 0, sizeof(_accel_diff));
	memset(&_gyro_diff, 0, sizeof(_gyro_diff));
	memset(&_mag_diff, 0, sizeof(_mag_diff));

	// initialise the publication variables
	memset(&_corrections, 0, sizeof(_corrections));
	memset(&_info, 0, sizeof(_info));

	for (unsigned i = 0; i < 3; i++) {
		_corrections.gyro_scale_0[i] = 1.0f;
		_corrections.accel_scale_0[i] = 1.0f;
		_corrections.gyro_scale_1[i] = 1.0f;
		_corrections.accel_scale_1[i] = 1.0f;
		_corrections.gyro_scale_2[i] = 1.0f;
		_corrections.accel_scale_2[i] = 1.0f;
	}

	_corrections.baro_scale_0 = 1.0f;
	_corrections.baro_scale_1 = 1.0f;
	_corrections.baro_scale_2 = 1.0f;

	_baro.voter.set_timeout(300000);
	_mag.voter.set_timeout(300000);
	_mag.voter.set_equal_value_threshold(1000);

	if (_hil_enabled) { // HIL has less accurate timing so increase the timeouts a bit
		_gyro.voter.set_timeout(500000);
		_accel.voter.set_timeout(500000);
	}
}

int VotedSensorsUpdate::init(sensor_combined_s &raw)
{
	raw.accelerometer_timestamp_relative = sensor_combined_s::RELATIVE_TIMESTAMP_INVALID;
	raw.timestamp = 0;

	initialize_sensors();

	_corrections_changed = true; //make sure to initially publish the corrections topic
	_selection_changed = true;

	return 0;
}

void VotedSensorsUpdate::initialize_sensors()
{
	init_sensor_class(ORB_ID(sensor_gyro), _gyro, GYRO_COUNT_MAX);
	init_sensor_class(ORB_ID(sensor_mag), _mag, MAG_COUNT_MAX);
	init_sensor_class(ORB_ID(sensor_accel), _accel, ACCEL_COUNT_MAX);
	init_sensor_class(ORB_ID(sensor_baro), _baro, BARO_COUNT_MAX);
}

void VotedSensorsUpdate::deinit()
{
	for (int i = 0; i < _gyro.subscription_count; i++) {
		orb_unsubscribe(_gyro.subscription[i]);
	}

	for (int i = 0; i < _accel.subscription_count; i++) {
		orb_unsubscribe(_accel.subscription[i]);
	}

	for (int i = 0; i < _mag.subscription_count; i++) {
		orb_unsubscribe(_mag.subscription[i]);
	}

	for (int i = 0; i < _baro.subscription_count; i++) {
		orb_unsubscribe(_baro.subscription[i]);
	}
}

void VotedSensorsUpdate::parameters_update()
{
	/* fine tune board offset */
	matrix::Dcmf board_rotation_offset = matrix::Eulerf(
			M_DEG_TO_RAD_F * _parameters.board_offset[0],
			M_DEG_TO_RAD_F * _parameters.board_offset[1],
			M_DEG_TO_RAD_F * _parameters.board_offset[2]);

	_board_rotation = board_rotation_offset * get_rot_matrix((enum Rotation)_parameters.board_rotation);

	// initialze all mag rotations with the board rotation in case there is no calibration data available
	for (int topic_instance = 0; topic_instance < MAG_COUNT_MAX; ++topic_instance) {
		_mag_rotation[topic_instance] = _board_rotation;
	}

	/* Load & apply the sensor calibrations.
	 * IMPORTANT: we assume all sensor drivers are running and published sensor data at this point
	 */

	/* temperature compensation */
	_temperature_compensation.parameters_update(_hil_enabled);

	/* gyro */
	for (uint8_t topic_instance = 0; topic_instance < _gyro.subscription_count; ++topic_instance) {

		uORB::SubscriptionData<sensor_gyro_s> report{ORB_ID(sensor_gyro), topic_instance};

		int temp = _temperature_compensation.set_sensor_id_gyro(report.get().device_id, topic_instance);

		if (temp < 0) {
			PX4_ERR("%s temp compensation init: failed to find device ID %u for instance %i", "gyro", report.get().device_id,
				topic_instance);

			_corrections.gyro_mapping[topic_instance] = 0;

		} else {
			_corrections.gyro_mapping[topic_instance] = temp;

		}
	}


	/* accel */
	for (uint8_t topic_instance = 0; topic_instance < _accel.subscription_count; ++topic_instance) {

		uORB::SubscriptionData<sensor_accel_s> report{ORB_ID(sensor_accel), topic_instance};

		int temp = _temperature_compensation.set_sensor_id_accel(report.get().device_id, topic_instance);

		if (temp < 0) {
			PX4_ERR("%s temp compensation init: failed to find device ID %u for instance %i", "accel", report.get().device_id,
				topic_instance);

			_corrections.accel_mapping[topic_instance] = 0;

		} else {
			_corrections.accel_mapping[topic_instance] = temp;

		}
	}


	/* baro */
	for (uint8_t topic_instance = 0; topic_instance < _baro.subscription_count; ++topic_instance) {

		uORB::SubscriptionData<sensor_baro_s> report{ORB_ID(sensor_baro), topic_instance};

		int temp = _temperature_compensation.set_sensor_id_baro(report.get().device_id, topic_instance);

		if (temp < 0) {
			PX4_ERR("%s temp compensation init: failed to find device ID %u for instance %i", "baro", report.get().device_id,
				topic_instance);

			_corrections.baro_mapping[topic_instance] = 0;

		} else {
			_corrections.baro_mapping[topic_instance] = temp;

		}
	}


	/* set offset parameters to new values */
	bool failed = false;
	char str[30] {};
	unsigned gyro_count = 0;
	unsigned accel_count = 0;
	unsigned gyro_cal_found_count = 0;
	unsigned accel_cal_found_count = 0;

	/* run through all gyro sensors */
	for (unsigned driver_index = 0; driver_index < GYRO_COUNT_MAX; driver_index++) {

		(void)sprintf(str, "%s%u", GYRO_BASE_DEVICE_PATH, driver_index);

		DevHandle h;
		DevMgr::getHandle(str, h);

		if (!h.isValid()) {
			continue;
		}

		uint32_t driver_device_id = h.ioctl(DEVIOCGDEVICEID, 0);
		bool config_ok = false;

		/* run through all stored calibrations that are applied at the driver level*/
		for (unsigned i = 0; i < GYRO_COUNT_MAX; i++) {
			/* initially status is ok per config */
			failed = false;

			(void)sprintf(str, "CAL_GYRO%u_ID", i);
			int32_t device_id = 0;
			failed = failed || (OK != param_get(param_find(str), &device_id));

			(void)sprintf(str, "CAL_GYRO%u_EN", i);
			int32_t device_enabled = 1;
			failed = failed || (OK != param_get(param_find(str), &device_enabled));

			_gyro.enabled[i] = (device_enabled == 1);

			if (failed) {
				continue;
			}

			if (driver_index == 0 && device_id > 0) {
				gyro_cal_found_count++;
			}

			/* if the calibration is for this device, apply it */
			if ((uint32_t)device_id == driver_device_id) {
				struct gyro_calibration_s gscale = {};
				(void)sprintf(str, "CAL_GYRO%u_XOFF", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.x_offset));
				(void)sprintf(str, "CAL_GYRO%u_YOFF", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.y_offset));
				(void)sprintf(str, "CAL_GYRO%u_ZOFF", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.z_offset));
				(void)sprintf(str, "CAL_GYRO%u_XSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.x_scale));
				(void)sprintf(str, "CAL_GYRO%u_YSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.y_scale));
				(void)sprintf(str, "CAL_GYRO%u_ZSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &gscale.z_scale));

				if (failed) {
					PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "gyro", i);

				} else {
					/* apply new scaling and offsets */
					config_ok = apply_gyro_calibration(h, &gscale, device_id);

					if (!config_ok) {
						PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "gyro ", i);
					}
				}

				break;
			}
		}

		if (config_ok) {
			gyro_count++;
		}
	}

	// There are less gyros than calibrations
	// reset the board calibration and fail the initial load
	if (gyro_count < gyro_cal_found_count) {

		// run through all stored calibrations and reset them
		for (unsigned i = 0; i < GYRO_COUNT_MAX; i++) {

			int32_t device_id = 0;
			(void)sprintf(str, "CAL_GYRO%u_ID", i);
			(void)param_set(param_find(str), &device_id);
		}
	}

	/* run through all accel sensors */
	for (unsigned driver_index = 0; driver_index < ACCEL_COUNT_MAX; driver_index++) {

		(void)sprintf(str, "%s%u", ACCEL_BASE_DEVICE_PATH, driver_index);

		DevHandle h;
		DevMgr::getHandle(str, h);

		if (!h.isValid()) {
			continue;
		}

		uint32_t driver_device_id = h.ioctl(DEVIOCGDEVICEID, 0);
		bool config_ok = false;

		/* run through all stored calibrations */
		for (unsigned i = 0; i < ACCEL_COUNT_MAX; i++) {
			/* initially status is ok per config */
			failed = false;

			(void)sprintf(str, "CAL_ACC%u_ID", i);
			int32_t device_id = 0;
			failed = failed || (OK != param_get(param_find(str), &device_id));

			(void)sprintf(str, "CAL_ACC%u_EN", i);
			int32_t device_enabled = 1;
			failed = failed || (OK != param_get(param_find(str), &device_enabled));

			_accel.enabled[i] = (device_enabled == 1);

			if (failed) {
				continue;
			}

			if (driver_index == 0 && device_id > 0) {
				accel_cal_found_count++;
			}

			/* if the calibration is for this device, apply it */
			if ((uint32_t)device_id == driver_device_id) {
				struct accel_calibration_s ascale = {};
				(void)sprintf(str, "CAL_ACC%u_XOFF", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.x_offset));
				(void)sprintf(str, "CAL_ACC%u_YOFF", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.y_offset));
				(void)sprintf(str, "CAL_ACC%u_ZOFF", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.z_offset));
				(void)sprintf(str, "CAL_ACC%u_XSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.x_scale));
				(void)sprintf(str, "CAL_ACC%u_YSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.y_scale));
				(void)sprintf(str, "CAL_ACC%u_ZSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &ascale.z_scale));

				if (failed) {
					PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "accel", i);

				} else {
					/* apply new scaling and offsets */
					config_ok = apply_accel_calibration(h, &ascale, device_id);

					if (!config_ok) {
						PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "accel ", i);
					}
				}

				break;
			}
		}

		if (config_ok) {
			accel_count++;
		}
	}

	// There are less accels than calibrations
	// reset the board calibration and fail the initial load
	if (accel_count < accel_cal_found_count) {

		// run through all stored calibrations and reset them
		for (unsigned i = 0; i < ACCEL_COUNT_MAX; i++) {

			int32_t device_id = 0;
			(void)sprintf(str, "CAL_ACC%u_ID", i);
			(void)param_set(param_find(str), &device_id);
		}
	}

	/* run through all mag sensors
	 * Because we store the device id in _mag_device_id, we need to get the id via uorb topic since
	 * the DevHandle method does not work on POSIX.
	 */
	for (int topic_instance = 0; topic_instance < MAG_COUNT_MAX
	     && topic_instance < _mag.subscription_count; ++topic_instance) {

		struct mag_report report;

		if (orb_copy(ORB_ID(sensor_mag), _mag.subscription[topic_instance], &report) != 0) {
			continue;
		}

		int topic_device_id = report.device_id;
		bool is_external = report.is_external;
		_mag_device_id[topic_instance] = topic_device_id;

		// find the driver handle that matches the topic_device_id
		DevHandle h;

		for (unsigned driver_index = 0; driver_index < MAG_COUNT_MAX; ++driver_index) {

			(void)sprintf(str, "%s%u", MAG_BASE_DEVICE_PATH, driver_index);

			DevMgr::getHandle(str, h);

			if (!h.isValid()) {
				/* the driver is not running, continue with the next */
				continue;
			}

			int driver_device_id = h.ioctl(DEVIOCGDEVICEID, 0);

			if (driver_device_id == topic_device_id) {
				break; // we found the matching driver

			} else {
				DevMgr::releaseHandle(h);
			}
		}

		bool config_ok = false;

		/* run through all stored calibrations */
		for (unsigned i = 0; i < MAG_COUNT_MAX; i++) {
			/* initially status is ok per config */
			failed = false;

			(void)sprintf(str, "CAL_MAG%u_ID", i);
			int32_t device_id = 0;
			failed = failed || (OK != param_get(param_find(str), &device_id));

			(void)sprintf(str, "CAL_MAG%u_EN", i);
			int32_t device_enabled = 1;
			failed = failed || (OK != param_get(param_find(str), &device_enabled));

			_mag.enabled[i] = (device_enabled == 1);

			if (failed) {
				continue;
			}

			/* if the calibration is for this device, apply it */
			if ((uint32_t)device_id == _mag_device_id[topic_instance]) {
				struct mag_calibration_s mscale = {};
				(void)sprintf(str, "CAL_MAG%u_XOFF", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.x_offset));
				(void)sprintf(str, "CAL_MAG%u_YOFF", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.y_offset));
				(void)sprintf(str, "CAL_MAG%u_ZOFF", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.z_offset));
				(void)sprintf(str, "CAL_MAG%u_XSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.x_scale));
				(void)sprintf(str, "CAL_MAG%u_YSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.y_scale));
				(void)sprintf(str, "CAL_MAG%u_ZSCALE", i);
				failed = failed || (OK != param_get(param_find(str), &mscale.z_scale));

				(void)sprintf(str, "CAL_MAG%u_ROT", i);

				int32_t mag_rot;
				param_get(param_find(str), &mag_rot);

				if (is_external) {

					/* check if this mag is still set as internal, otherwise leave untouched */
					if (mag_rot < 0) {
						/* it was marked as internal, change to external with no rotation */
						mag_rot = 0;
						param_set_no_notification(param_find(str), &mag_rot);
					}

				} else {
					/* mag is internal - reset param to -1 to indicate internal mag */
					if (mag_rot != MAG_ROT_VAL_INTERNAL) {
						mag_rot = MAG_ROT_VAL_INTERNAL;
						param_set_no_notification(param_find(str), &mag_rot);
					}
				}

				/* now get the mag rotation */
				if (mag_rot >= 0) {
					// Set external magnetometers to use the parameter value
					_mag_rotation[topic_instance] = get_rot_matrix((enum Rotation)mag_rot);

				} else {
					// Set internal magnetometers to use the board rotation
					_mag_rotation[topic_instance] = _board_rotation;
				}

				if (failed) {
					PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "mag", i);

				} else {

					/* apply new scaling and offsets */
					config_ok = apply_mag_calibration(h, &mscale, device_id);

					if (!config_ok) {
						PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "mag ", i);
					}
				}

				break;
			}
		}
	}

}

void VotedSensorsUpdate::accel_poll(struct sensor_combined_s &raw)
{
	float *offsets[] = {_corrections.accel_offset_0, _corrections.accel_offset_1, _corrections.accel_offset_2 };
	float *scales[] = {_corrections.accel_scale_0, _corrections.accel_scale_1, _corrections.accel_scale_2 };

	for (int uorb_index = 0; uorb_index < _accel.subscription_count; uorb_index++) {
		bool accel_updated;
		orb_check(_accel.subscription[uorb_index], &accel_updated);

		if (accel_updated) {
			sensor_accel_s accel_report;

			int ret = orb_copy(ORB_ID(sensor_accel), _accel.subscription[uorb_index], &accel_report);

			if (ret != PX4_OK || accel_report.timestamp == 0) {
				continue; //ignore invalid data
			}

			if (!_accel.enabled[uorb_index]) {
				continue;
			}

			// First publication with data
			if (_accel.priority[uorb_index] == 0) {
				int32_t priority = 0;
				orb_priority(_accel.subscription[uorb_index], &priority);
				_accel.priority[uorb_index] = (uint8_t)priority;
			}

			_accel_device_id[uorb_index] = accel_report.device_id;

			matrix::Vector3f accel_data;

			if (accel_report.integral_dt != 0) {
				/*
				 * Using data that has been integrated in the driver before downsampling is preferred
				 * becasue it reduces aliasing errors. Correct the raw sensor data for scale factor errors
				 * and offsets due to temperature variation. It is assumed that any filtering of input
				 * data required is performed in the sensor driver, preferably before downsampling.
				*/

				// convert the delta velocities to an equivalent acceleration before application of corrections
				float dt_inv = 1.e6f / accel_report.integral_dt;
				accel_data = matrix::Vector3f(accel_report.x_integral * dt_inv,
							      accel_report.y_integral * dt_inv,
							      accel_report.z_integral * dt_inv);

				_last_sensor_data[uorb_index].accelerometer_integral_dt = accel_report.integral_dt;

			} else {
				// using the value instead of the integral (the integral is the prefered choice)

				// Correct each sensor for temperature effects
				// Filtering and/or downsampling of temperature should be performed in the driver layer
				accel_data = matrix::Vector3f(accel_report.x, accel_report.y, accel_report.z);

				// handle the cse where this is our first output
				if (_last_accel_timestamp[uorb_index] == 0) {
					_last_accel_timestamp[uorb_index] = accel_report.timestamp - 1000;
				}

				// approximate the  delta time using the difference in accel data time stamps
				_last_sensor_data[uorb_index].accelerometer_integral_dt =
					(accel_report.timestamp - _last_accel_timestamp[uorb_index]);
			}

			// handle temperature compensation
			if (_temperature_compensation.apply_corrections_accel(uorb_index, accel_data, accel_report.temperature,
					offsets[uorb_index], scales[uorb_index]) == 2) {
				_corrections_changed = true;
			}

			// rotate corrected measurements from sensor to body frame
			accel_data = _board_rotation * accel_data;

			_last_sensor_data[uorb_index].accelerometer_m_s2[0] = accel_data(0);
			_last_sensor_data[uorb_index].accelerometer_m_s2[1] = accel_data(1);
			_last_sensor_data[uorb_index].accelerometer_m_s2[2] = accel_data(2);

			_last_accel_timestamp[uorb_index] = accel_report.timestamp;
			_accel.voter.put(uorb_index, accel_report.timestamp, _last_sensor_data[uorb_index].accelerometer_m_s2,
					 accel_report.error_count, _accel.priority[uorb_index]);
		}
	}

	// find the best sensor
	int best_index;
	_accel.voter.get_best(hrt_absolute_time(), &best_index);

	// write the best sensor data to the output variables
	if (best_index >= 0) {
		raw.accelerometer_integral_dt = _last_sensor_data[best_index].accelerometer_integral_dt;
		memcpy(&raw.accelerometer_m_s2, &_last_sensor_data[best_index].accelerometer_m_s2, sizeof(raw.accelerometer_m_s2));

		if (best_index != _accel.last_best_vote) {
			_accel.last_best_vote = (uint8_t)best_index;
			_corrections.selected_accel_instance = (uint8_t)best_index;
			_corrections_changed = true;
		}

		if (_selection.accel_device_id != _accel_device_id[best_index]) {
			_selection_changed = true;
			_selection.accel_device_id = _accel_device_id[best_index];
		}
	}
}

void VotedSensorsUpdate::gyro_poll(struct sensor_combined_s &raw)
{
	float *offsets[] = {_corrections.gyro_offset_0, _corrections.gyro_offset_1, _corrections.gyro_offset_2 };
	float *scales[] = {_corrections.gyro_scale_0, _corrections.gyro_scale_1, _corrections.gyro_scale_2 };

	for (int uorb_index = 0; uorb_index < _gyro.subscription_count; uorb_index++) {
		bool gyro_updated;
		orb_check(_gyro.subscription[uorb_index], &gyro_updated);

		if (gyro_updated) {
			sensor_gyro_s gyro_report;

			int ret = orb_copy(ORB_ID(sensor_gyro), _gyro.subscription[uorb_index], &gyro_report);

			if (ret != PX4_OK || gyro_report.timestamp == 0) {
				continue; //ignore invalid data
			}

			if (!_gyro.enabled[uorb_index]) {
				continue;
			}

			// First publication with data
			if (_gyro.priority[uorb_index] == 0) {
				int32_t priority = 0;
				orb_priority(_gyro.subscription[uorb_index], &priority);
				_gyro.priority[uorb_index] = (uint8_t)priority;
			}

			_gyro_device_id[uorb_index] = gyro_report.device_id;

			matrix::Vector3f gyro_rate;

			if (gyro_report.integral_dt != 0) {
				/*
				 * Using data that has been integrated in the driver before downsampling is preferred
				 * becasue it reduces aliasing errors. Correct the raw sensor data for scale factor errors
				 * and offsets due to temperature variation. It is assumed that any filtering of input
				 * data required is performed in the sensor driver, preferably before downsampling.
				*/

				// convert the delta angles to an equivalent angular rate before application of corrections
				float dt_inv = 1.e6f / gyro_report.integral_dt;
				gyro_rate = matrix::Vector3f(gyro_report.x_integral * dt_inv,
							     gyro_report.y_integral * dt_inv,
							     gyro_report.z_integral * dt_inv);

				_last_sensor_data[uorb_index].gyro_integral_dt = gyro_report.integral_dt;

			} else {
				//using the value instead of the integral (the integral is the prefered choice)

				// Correct each sensor for temperature effects
				// Filtering and/or downsampling of temperature should be performed in the driver layer
				gyro_rate = matrix::Vector3f(gyro_report.x, gyro_report.y, gyro_report.z);

				// handle the case where this is our first output
				if (_last_sensor_data[uorb_index].timestamp == 0) {
					_last_sensor_data[uorb_index].timestamp = gyro_report.timestamp - 1000;
				}

				// approximate the delta time using the difference in gyro data time stamps
				_last_sensor_data[uorb_index].gyro_integral_dt =
					(gyro_report.timestamp - _last_sensor_data[uorb_index].timestamp);
			}

			// handle temperature compensation
			if (_temperature_compensation.apply_corrections_gyro(uorb_index, gyro_rate, gyro_report.temperature,
					offsets[uorb_index], scales[uorb_index]) == 2) {
				_corrections_changed = true;
			}

			// rotate corrected measurements from sensor to body frame
			gyro_rate = _board_rotation * gyro_rate;

			_last_sensor_data[uorb_index].gyro_rad[0] = gyro_rate(0);
			_last_sensor_data[uorb_index].gyro_rad[1] = gyro_rate(1);
			_last_sensor_data[uorb_index].gyro_rad[2] = gyro_rate(2);

			_last_sensor_data[uorb_index].timestamp = gyro_report.timestamp;
			_gyro.voter.put(uorb_index, gyro_report.timestamp, _last_sensor_data[uorb_index].gyro_rad,
					gyro_report.error_count, _gyro.priority[uorb_index]);
		}
	}

	// find the best sensor
	int best_index;
	_gyro.voter.get_best(hrt_absolute_time(), &best_index);

	// write data for the best sensor to output variables
	if (best_index >= 0) {
		raw.timestamp = _last_sensor_data[best_index].timestamp;
		raw.gyro_integral_dt = _last_sensor_data[best_index].gyro_integral_dt;
		memcpy(&raw.gyro_rad, &_last_sensor_data[best_index].gyro_rad, sizeof(raw.gyro_rad));

		if (_gyro.last_best_vote != best_index) {
			_gyro.last_best_vote = (uint8_t)best_index;
			_corrections.selected_gyro_instance = (uint8_t)best_index;
			_corrections_changed = true;
		}

		if (_selection.gyro_device_id != _gyro_device_id[best_index]) {
			_selection_changed = true;
			_selection.gyro_device_id = _gyro_device_id[best_index];
		}
	}
}

void VotedSensorsUpdate::mag_poll(vehicle_magnetometer_s &magnetometer)
{
	for (int uorb_index = 0; uorb_index < _mag.subscription_count; uorb_index++) {
		bool mag_updated;
		orb_check(_mag.subscription[uorb_index], &mag_updated);

		if (mag_updated) {
			struct mag_report mag_report;

			int ret = orb_copy(ORB_ID(sensor_mag), _mag.subscription[uorb_index], &mag_report);

			if (ret != PX4_OK || mag_report.timestamp == 0) {
				continue; //ignore invalid data
			}

			if (!_mag.enabled[uorb_index]) {
				continue;
			}

			// First publication with data
			if (_mag.priority[uorb_index] == 0) {
				int32_t priority = 0;
				orb_priority(_mag.subscription[uorb_index], &priority);
				_mag.priority[uorb_index] = (uint8_t)priority;
			}

			matrix::Vector3f vect(mag_report.x, mag_report.y, mag_report.z);
			vect = _mag_rotation[uorb_index] * vect;

			_last_magnetometer[uorb_index].timestamp = mag_report.timestamp;
			_last_magnetometer[uorb_index].magnetometer_ga[0] = vect(0);
			_last_magnetometer[uorb_index].magnetometer_ga[1] = vect(1);
			_last_magnetometer[uorb_index].magnetometer_ga[2] = vect(2);

			_mag.voter.put(uorb_index, mag_report.timestamp, vect.data(), mag_report.error_count, _mag.priority[uorb_index]);
		}
	}

	int best_index;
	_mag.voter.get_best(hrt_absolute_time(), &best_index);

	if (best_index >= 0) {
		magnetometer = _last_magnetometer[best_index];
		_mag.last_best_vote = (uint8_t)best_index;

		if (_selection.mag_device_id != _mag_device_id[best_index]) {
			_selection_changed = true;
			_selection.mag_device_id = _mag_device_id[best_index];
		}
	}
}

void VotedSensorsUpdate::baro_poll(vehicle_air_data_s &airdata)
{
	bool got_update = false;
	float *offsets[] = {&_corrections.baro_offset_0, &_corrections.baro_offset_1, &_corrections.baro_offset_2 };
	float *scales[] = {&_corrections.baro_scale_0, &_corrections.baro_scale_1, &_corrections.baro_scale_2 };

	for (int uorb_index = 0; uorb_index < _baro.subscription_count; uorb_index++) {
		bool baro_updated;
		orb_check(_baro.subscription[uorb_index], &baro_updated);

		if (baro_updated) {
			sensor_baro_s baro_report;

			int ret = orb_copy(ORB_ID(sensor_baro), _baro.subscription[uorb_index], &baro_report);

			if (ret != PX4_OK || baro_report.timestamp == 0) {
				continue; //ignore invalid data
			}

			// Convert from millibar to Pa
			float corrected_pressure = 100.0f * baro_report.pressure;

			// handle temperature compensation
			if (_temperature_compensation.apply_corrections_baro(uorb_index, corrected_pressure, baro_report.temperature,
					offsets[uorb_index], scales[uorb_index]) == 2) {
				_corrections_changed = true;
			}

			// First publication with data
			if (_baro.priority[uorb_index] == 0) {
				int32_t priority = 0;
				orb_priority(_baro.subscription[uorb_index], &priority);
				_baro.priority[uorb_index] = (uint8_t)priority;
			}

			_baro_device_id[uorb_index] = baro_report.device_id;

			got_update = true;
			matrix::Vector3f vect(baro_report.pressure, baro_report.temperature, 0.f);

			_last_airdata[uorb_index].timestamp = baro_report.timestamp;
			_last_airdata[uorb_index].baro_temp_celcius = baro_report.temperature;
			_last_airdata[uorb_index].baro_pressure_pa = corrected_pressure;

			_baro.voter.put(uorb_index, baro_report.timestamp, vect.data(), baro_report.error_count, _baro.priority[uorb_index]);
		}
	}

	if (got_update) {
		int best_index;
		_baro.voter.get_best(hrt_absolute_time(), &best_index);

		if (best_index >= 0) {
			airdata = _last_airdata[best_index];

			if (_baro.last_best_vote != best_index) {
				_baro.last_best_vote = (uint8_t)best_index;
				_corrections.selected_baro_instance = (uint8_t)best_index;
				_corrections_changed = true;
			}

			if (_selection.baro_device_id != _baro_device_id[best_index]) {
				_selection_changed = true;
				_selection.baro_device_id = _baro_device_id[best_index];
			}

			// calculate altitude using the hypsometric equation

			static constexpr float T1 = 15.0f - CONSTANTS_ABSOLUTE_NULL_CELSIUS;	/* temperature at base height in Kelvin */
			static constexpr float a  = -6.5f / 1000.0f;	/* temperature gradient in degrees per metre */

			/* current pressure at MSL in kPa (QNH in hPa)*/
			const float p1 = _parameters.baro_qnh * 0.1f;

			/* measured pressure in kPa */
			const float p = airdata.baro_pressure_pa * 0.001f;

			/*
			 * Solve:
			 *
			 *     /        -(aR / g)     \
			 *    | (p / p1)          . T1 | - T1
			 *     \                      /
			 * h = -------------------------------  + h1
			 *                   a
			 */
			airdata.baro_alt_meter = (((powf((p / p1), (-(a * CONSTANTS_AIR_GAS_CONST) / CONSTANTS_ONE_G))) * T1) - T1) / a;


			// calculate air density
			// estimate air density assuming typical 20degC ambient temperature
			// TODO: use air temperature if available (differential pressure sensors)
			static constexpr float pressure_to_density = 1.0f / (CONSTANTS_AIR_GAS_CONST * (20.0f -
					CONSTANTS_ABSOLUTE_NULL_CELSIUS));
			airdata.rho = pressure_to_density * airdata.baro_pressure_pa;
		}
	}
}

bool VotedSensorsUpdate::check_failover(SensorData &sensor, const char *sensor_name, const uint64_t type)
{
	if (sensor.last_failover_count != sensor.voter.failover_count() && !_hil_enabled) {

		uint32_t flags = sensor.voter.failover_state();
		int failover_index = sensor.voter.failover_index();

		if (flags == DataValidator::ERROR_FLAG_NO_ERROR) {
			if (failover_index != -1) {
				//we switched due to a non-critical reason. No need to panic.
				PX4_INFO("%s sensor switch from #%i", sensor_name, failover_index);
			}

		} else {
			if (failover_index != -1) {
				mavlink_log_emergency(&_mavlink_log_pub, "%s #%i fail: %s%s%s%s%s!",
						      sensor_name,
						      failover_index,
						      ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " OFF" : ""),
						      ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " STALE" : ""),
						      ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " TIMEOUT" : ""),
						      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " ERR CNT" : ""),
						      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " ERR DNST" : ""));

				// reduce priority of failed sensor to the minimum
				sensor.priority[failover_index] = 1;

				PX4_ERR("Sensor %s #%i failed. Reconfiguring sensor priorities.", sensor_name, failover_index);

				int ctr_valid = 0;

				for (uint8_t i = 0; i < sensor.subscription_count; i++) {
					if (sensor.priority[i] > 1) { ctr_valid++; }

					PX4_WARN("Remaining sensors after failover event %u: %s #%u priority: %u", failover_index, sensor_name, i,
						 sensor.priority[i]);
				}

				if (ctr_valid < 2) {
					if (ctr_valid == 0) {
						// Zero valid sensors remain! Set even the primary sensor health to false
						_info.subsystem_type = type;

					} else if (ctr_valid == 1) {
						// One valid sensor remains, set secondary sensor health to false
						if (type == subsystem_info_s::SUBSYSTEM_TYPE_GYRO) { _info.subsystem_type = subsystem_info_s::SUBSYSTEM_TYPE_GYRO2; }

						if (type == subsystem_info_s::SUBSYSTEM_TYPE_ACC) { _info.subsystem_type = subsystem_info_s::SUBSYSTEM_TYPE_ACC2; }

						if (type == subsystem_info_s::SUBSYSTEM_TYPE_MAG) { _info.subsystem_type = subsystem_info_s::SUBSYSTEM_TYPE_MAG2; }
					}

					_info.timestamp = hrt_absolute_time();
					_info.present = true;
					_info.enabled = true;
					_info.ok = false;

					if (_info_pub == nullptr) {
						_info_pub = orb_advertise_queue(ORB_ID(subsystem_info), &_info, subsystem_info_s::ORB_QUEUE_LENGTH);

					} else {
						orb_publish(ORB_ID(subsystem_info), _info_pub, &_info);
					}
				}
			}
		}

		sensor.last_failover_count = sensor.voter.failover_count();
		return true;
	}

	return false;
}

void VotedSensorsUpdate::init_sensor_class(const struct orb_metadata *meta, SensorData &sensor_data,
		uint8_t sensor_count_max)
{
	int max_sensor_index = -1;

	for (unsigned i = 0; i < sensor_count_max; i++) {
		if (orb_exists(meta, i) != 0) {
			continue;
		}

		max_sensor_index = i;

		if (sensor_data.subscription[i] < 0) {
			sensor_data.subscription[i] = orb_subscribe_multi(meta, i);

			if (i > 0) {
				/* the first always exists, but for each further sensor, add a new validator */
				if (!sensor_data.voter.add_new_validator()) {
					PX4_ERR("failed to add validator for sensor %s %i", meta->o_name, i);
				}
			}
		}
	}

	// never decrease the sensor count, as we could end up with mismatching validators
	if (max_sensor_index + 1 > sensor_data.subscription_count) {
		sensor_data.subscription_count = max_sensor_index + 1;
	}
}

void VotedSensorsUpdate::print_status()
{
	PX4_INFO("gyro status:");
	_gyro.voter.print();
	PX4_INFO("accel status:");
	_accel.voter.print();
	PX4_INFO("mag status:");
	_mag.voter.print();
	PX4_INFO("baro status:");
	_baro.voter.print();

	_temperature_compensation.print_status();
}

bool
VotedSensorsUpdate::apply_gyro_calibration(DevHandle &h, const struct gyro_calibration_s *gcal, const int device_id)
{
#if defined(__PX4_NUTTX)

	/* On most systems, we can just use the IOCTL call to set the calibration params. */
	return !h.ioctl(GYROIOCSSCALE, (long unsigned int)gcal);

#else
	/* On QURT, the params are read directly in the respective wrappers. */
	return true;
#endif
}

bool
VotedSensorsUpdate::apply_accel_calibration(DevHandle &h, const struct accel_calibration_s *acal, const int device_id)
{
#if defined(__PX4_NUTTX)

	/* On most systems, we can just use the IOCTL call to set the calibration params. */
	return !h.ioctl(ACCELIOCSSCALE, (long unsigned int)acal);

#else
	/* On QURT, the params are read directly in the respective wrappers. */
	return true;
#endif
}

bool
VotedSensorsUpdate::apply_mag_calibration(DevHandle &h, const struct mag_calibration_s *mcal, const int device_id)
{
#if defined(__PX4_NUTTX)

	if (!h.isValid()) {
		return false;
	}

	/* On most systems, we can just use the IOCTL call to set the calibration params. */
	return !h.ioctl(MAGIOCSSCALE, (long unsigned int)mcal);

#else
	/* On QURT & POSIX, the params are read directly in the respective wrappers. */
	return true;
#endif
}

void VotedSensorsUpdate::sensors_poll(sensor_combined_s &raw, vehicle_air_data_s &airdata,
				      vehicle_magnetometer_s &magnetometer)
{
	accel_poll(raw);
	gyro_poll(raw);
	mag_poll(magnetometer);
	baro_poll(airdata);

	// publish sensor corrections if necessary
	if (_corrections_changed) {
		_corrections.timestamp = hrt_absolute_time();

		if (_sensor_correction_pub == nullptr) {
			_sensor_correction_pub = orb_advertise(ORB_ID(sensor_correction), &_corrections);

		} else {
			orb_publish(ORB_ID(sensor_correction), _sensor_correction_pub, &_corrections);
		}

		_corrections_changed = false;
	}

	// publish sensor selection if changed
	if (_selection_changed) {
		_selection.timestamp = hrt_absolute_time();

		if (_sensor_selection_pub == nullptr) {
			_sensor_selection_pub = orb_advertise(ORB_ID(sensor_selection), &_selection);

		} else {
			orb_publish(ORB_ID(sensor_selection), _sensor_selection_pub, &_selection);
		}

		_selection_changed = false;
	}
}

void VotedSensorsUpdate::check_failover()
{
	check_failover(_accel, "Accel", subsystem_info_s::SUBSYSTEM_TYPE_ACC);
	check_failover(_gyro, "Gyro", subsystem_info_s::SUBSYSTEM_TYPE_GYRO);
	check_failover(_mag, "Mag", subsystem_info_s::SUBSYSTEM_TYPE_MAG);
	check_failover(_baro, "Baro", subsystem_info_s::SUBSYSTEM_TYPE_ABSPRESSURE);
}

void VotedSensorsUpdate::set_relative_timestamps(sensor_combined_s &raw)
{
	if (_last_accel_timestamp[_accel.last_best_vote]) {
		raw.accelerometer_timestamp_relative = (int32_t)((int64_t)_last_accel_timestamp[_accel.last_best_vote] -
						       (int64_t)raw.timestamp);
	}
}

void
VotedSensorsUpdate::calc_accel_inconsistency(sensor_preflight_s &preflt)
{
	float accel_diff_sum_max_sq = 0.0f; // the maximum sum of axis differences squared
	unsigned check_index = 0; // the number of sensors the primary has been checked against

	// Check each sensor against the primary
	for (int sensor_index = 0; sensor_index < _accel.subscription_count; sensor_index++) {

		// check that the sensor we are checking against is not the same as the primary
		if ((_accel.priority[sensor_index] > 0) && (sensor_index != _accel.last_best_vote)) {

			float accel_diff_sum_sq = 0.0f; // sum of differences squared for a single sensor comparison agains the primary

			// calculate accel_diff_sum_sq for the specified sensor against the primary
			for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
				_accel_diff[axis_index][check_index] = 0.95f * _accel_diff[axis_index][check_index] + 0.05f *
								       (_last_sensor_data[_accel.last_best_vote].accelerometer_m_s2[axis_index] -
									_last_sensor_data[sensor_index].accelerometer_m_s2[axis_index]);
				accel_diff_sum_sq += _accel_diff[axis_index][check_index] * _accel_diff[axis_index][check_index];

			}

			// capture the largest sum value
			if (accel_diff_sum_sq > accel_diff_sum_max_sq) {
				accel_diff_sum_max_sq = accel_diff_sum_sq;

			}

			// increment the check index
			check_index++;
		}

		// check to see if the maximum number of checks has been reached and break
		if (check_index >= 2) {
			break;

		}
	}

	// skip check if less than 2 sensors
	if (check_index < 1) {
		preflt.accel_inconsistency_m_s_s = 0.0f;

	} else {
		// get the vector length of the largest difference and write to the combined sensor struct
		preflt.accel_inconsistency_m_s_s = sqrtf(accel_diff_sum_max_sq);
	}
}

void VotedSensorsUpdate::calc_gyro_inconsistency(sensor_preflight_s &preflt)
{
	float gyro_diff_sum_max_sq = 0.0f; // the maximum sum of axis differences squared
	unsigned check_index = 0; // the number of sensors the primary has been checked against

	// Check each sensor against the primary
	for (int sensor_index = 0; sensor_index < _gyro.subscription_count; sensor_index++) {

		// check that the sensor we are checking against is not the same as the primary
		if ((_gyro.priority[sensor_index] > 0) && (sensor_index != _gyro.last_best_vote)) {

			float gyro_diff_sum_sq = 0.0f; // sum of differences squared for a single sensor comparison against the primary

			// calculate gyro_diff_sum_sq for the specified sensor against the primary
			for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
				_gyro_diff[axis_index][check_index] = 0.95f * _gyro_diff[axis_index][check_index] + 0.05f *
								      (_last_sensor_data[_gyro.last_best_vote].gyro_rad[axis_index] -
								       _last_sensor_data[sensor_index].gyro_rad[axis_index]);
				gyro_diff_sum_sq += _gyro_diff[axis_index][check_index] * _gyro_diff[axis_index][check_index];

			}

			// capture the largest sum value
			if (gyro_diff_sum_sq > gyro_diff_sum_max_sq) {
				gyro_diff_sum_max_sq = gyro_diff_sum_sq;

			}

			// increment the check index
			check_index++;
		}

		// check to see if the maximum number of checks has been reached and break
		if (check_index >= 2) {
			break;

		}
	}

	// skip check if less than 2 sensors
	if (check_index < 1) {
		preflt.gyro_inconsistency_rad_s = 0.0f;

	} else {
		// get the vector length of the largest difference and write to the combined sensor struct
		preflt.gyro_inconsistency_rad_s = sqrtf(gyro_diff_sum_max_sq);
	}
}

void VotedSensorsUpdate::calc_mag_inconsistency(sensor_preflight_s &preflt)
{
	float mag_diff_sum_max_sq = 0.0f; // the maximum sum of axis differences squared
	unsigned check_index = 0; // the number of sensors the primary has been checked against

	// Check each sensor against the primary
	for (int sensor_index = 0; sensor_index < _mag.subscription_count; sensor_index++) {

		// check that the sensor we are checking against is not the same as the primary
		if ((_mag.priority[sensor_index] > 0) && (sensor_index != _mag.last_best_vote)) {

			float mag_diff_sum_sq = 0.0f; // sum of differences squared for a single sensor comparison against the primary

			// calculate mag_diff_sum_sq for the specified sensor against the primary
			for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
				_mag_diff[axis_index][check_index] = 0.95f * _mag_diff[axis_index][check_index] + 0.05f *
								     (_last_magnetometer[_mag.last_best_vote].magnetometer_ga[axis_index] -
								      _last_magnetometer[sensor_index].magnetometer_ga[axis_index]);

				mag_diff_sum_sq += _mag_diff[axis_index][check_index] * _mag_diff[axis_index][check_index];

			}

			// capture the largest sum value
			if (mag_diff_sum_sq > mag_diff_sum_max_sq) {
				mag_diff_sum_max_sq = mag_diff_sum_sq;

			}

			// increment the check index
			check_index++;
		}

		// check to see if the maximum number of checks has been reached and break
		if (check_index >= 2) {
			break;

		}
	}

	// skip check if less than 2 sensors
	if (check_index < 1) {
		preflt.mag_inconsistency_ga = 0.0f;

	} else {
		// get the vector length of the largest difference and write to the combined sensor struct
		preflt.mag_inconsistency_ga = sqrtf(mag_diff_sum_max_sq);
	}
}
