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
 * @file board_config.h
 *
 * BBBLUE internal definitions
 */

#pragma once

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#define BOARD_OVERRIDE_UUID "BBBLUEID00000000" // must be of length 16
#define PX4_SOC_ARCH_ID     PX4_SOC_ARCH_ID_BBBLUE

#define BOARD_BATTERY1_V_DIV   (11.0f)
//#define BOARD_BATTERY1_A_PER_V (15.391030303f)

// Battery ADC channels
#define ADC_BATTERY_VOLTAGE_CHANNEL     5
#define ADC_BATTERY_CURRENT_CHANNEL     ((uint8_t)(-1))
#define ADC_AIRSPEED_VOLTAGE_CHANNEL    ((uint8_t)(-1))

#define BOARD_HAS_NO_BOOTLOADER

#define BOARD_MAX_LEDS 4 // Number external of LED's this board has

/*
 * I2C busses
 */
#define PX4_I2C_BUS_EXPANSION	1
#define PX4_NUMBER_I2C_BUSES    1

#include <system_config.h>
#include <drivers/boards/common/board_common.h>

#ifdef  __cplusplus
extern "C" {
#endif

int  rc_init(void);
void rc_cleaning(void);

#ifdef __cplusplus
}
#endif

#ifdef __RC_V0_3
#define rc_i2c_lock_bus 	rc_i2c_claim_bus
#define rc_i2c_unlock_bus	rc_i2c_release_bus
#define rc_i2c_get_lock		rc_i2c_get_in_use_state

#define rc_bmp_init			rc_initialize_barometer

#define rc_adc_read_raw		rc_adc_raw

#define rc_servo_send_pulse_us			rc_send_servo_pulse_us

#define rc_filter_empty					rc_empty_filter
#define rc_filter_march					rc_march_filter
#define rc_filter_prefill_inputs		rc_prefill_filter_inputs
#define rc_filter_prefill_outputs		rc_prefill_filter_outputs
#define rc_filter_butterworth_lowpass	rc_butterworth_lowpass

/**
 * struct to hold the data retreived during one read of the barometer.
 */
typedef struct rc_bmp_data_t {
	float temp_c;		///< temperature in degrees celcius
	float alt_m;		///< altitude in meters
	float pressure_pa;	///< current pressure in pascals
} rc_bmp_data_t;

#ifdef  __cplusplus
extern "C" {
#endif

int rc_bmp_read(rc_bmp_data_t *data);

#ifdef __cplusplus
}
#endif

#endif

#endif // BOARD_CONFIG_H
