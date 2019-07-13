/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <drivers/drv_hrt.h>
#include "baro.hpp"
#include <cmath>

const char *const UavcanBarometerBridge::NAME = "baro";

UavcanBarometerBridge::UavcanBarometerBridge(uavcan::INode &node) :
	UavcanCDevSensorBridgeBase("uavcan_baro", "/dev/uavcan/baro", BARO_BASE_DEVICE_PATH, ORB_ID(sensor_baro)),
	_sub_air_pressure_data(node),
	_sub_air_temperature_data(node),
	_reports(2, sizeof(sensor_baro_s))
{ }

int UavcanBarometerBridge::init()
{
	int res = device::CDev::init();

	if (res < 0) {
		return res;
	}

	res = _sub_air_pressure_data.start(AirPressureCbBinder(this, &UavcanBarometerBridge::air_pressure_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	res = _sub_air_temperature_data.start(AirTemperatureCbBinder(this, &UavcanBarometerBridge::air_temperature_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

ssize_t UavcanBarometerBridge::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_baro_s);
	sensor_baro_s *baro_buf = reinterpret_cast<sensor_baro_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	while (count--) {
		if (_reports.get(baro_buf)) {
			ret += sizeof(*baro_buf);
			baro_buf++;
		}
	}

	/* if there was no data, warn the caller */
	return ret ? ret : -EAGAIN;
}

int UavcanBarometerBridge::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case SENSORIOCSPOLLRATE: {
			// not supported yet, pretend that everything is ok
			return OK;
		}

	default: {
			return CDev::ioctl(filp, cmd, arg);
		}
	}
}

void UavcanBarometerBridge::air_temperature_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticTemperature> &msg)
{
	last_temperature_kelvin = msg.static_temperature;
}

void UavcanBarometerBridge::air_pressure_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticPressure> &msg)
{
	sensor_baro_s report{};

	/*
	 * FIXME HACK
	 * This code used to rely on msg.getMonotonicTimestamp().toUSec() instead of HRT.
	 * It stopped working when the time sync feature has been introduced, because it caused libuavcan
	 * to use an independent time source (based on hardware TIM5) instead of HRT.
	 * The proper solution is to be developed.
	 */
	report.timestamp   = hrt_absolute_time();
	report.temperature = last_temperature_kelvin - 273.15F;
	report.pressure    = msg.static_pressure / 100.0F;  // Convert to millibar
	report.error_count = 0;

	/* TODO get device ID for sensor */
	report.device_id = 0;

	// add to the ring buffer
	_reports.force(&report);

	publish(msg.getSrcNodeID().get(), &report);
}
