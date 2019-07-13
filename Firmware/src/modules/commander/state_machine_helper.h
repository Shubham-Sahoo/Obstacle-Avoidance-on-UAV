/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file state_machine_helper.h
 * State machine helper functions definitions
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 */

#ifndef STATE_MACHINE_HELPER_H_
#define STATE_MACHINE_HELPER_H_

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/commander_state.h>
#include <uORB/topics/vehicle_status_flags.h>

typedef enum {
	TRANSITION_DENIED = -1,
	TRANSITION_NOT_CHANGED = 0,
	TRANSITION_CHANGED
} transition_result_t;

enum class link_loss_actions_t {
	DISABLED = 0,
	AUTO_LOITER = 1,	// Hold mode
	AUTO_RTL = 2,		// Return mode
	AUTO_LAND = 3,		// Land mode
	AUTO_RECOVER = 4,	// Data Link Auto Recovery (CASA Outback Challenge rules)
	TERMINATE = 5,		// Turn off all controllers and set PWM outputs to failsafe value
	LOCKDOWN = 6,		// Kill the motors, same result as kill switch
};

typedef enum {
	ARM_REQ_NONE = 0,
	ARM_REQ_MISSION_BIT = (1 << 0),
	ARM_REQ_ARM_AUTH_BIT = (1 << 1),
	ARM_REQ_GPS_BIT = (1 << 2),
} arm_requirements_t;

extern const char *const arming_state_names[];

bool is_safe(const safety_s &safety, const actuator_armed_s &armed);

transition_result_t
arming_state_transition(vehicle_status_s *status, const safety_s &safety, const arming_state_t new_arming_state,
			actuator_armed_s *armed, const bool fRunPreArmChecks, orb_advert_t *mavlink_log_pub,
			vehicle_status_flags_s *status_flags, const uint8_t arm_requirements, const hrt_abstime &time_since_boot);

transition_result_t
main_state_transition(const vehicle_status_s &status, const main_state_t new_main_state,
		      const vehicle_status_flags_s &status_flags, commander_state_s *internal_state);

void enable_failsafe(vehicle_status_s *status, bool old_failsafe, orb_advert_t *mavlink_log_pub, const char *reason);

bool set_nav_state(vehicle_status_s *status, actuator_armed_s *armed, commander_state_s *internal_state,
		   orb_advert_t *mavlink_log_pub, const link_loss_actions_t data_link_loss_act, const bool mission_finished,
		   const bool stay_in_failsafe, const vehicle_status_flags_s &status_flags, bool landed,
		   const link_loss_actions_t rc_loss_act, const int offb_loss_act, const int offb_loss_rc_act,
		   const int posctl_nav_loss_act);

/*
 * Checks the validty of position data aaainst the requirements of the current navigation
 * mode and switches mode if position data required is not available.
 */
bool check_invalid_pos_nav_state(vehicle_status_s *status, bool old_failsafe, orb_advert_t *mavlink_log_pub,
				 const vehicle_status_flags_s &status_flags, const bool use_rc, const bool using_global_pos);

bool prearm_check(orb_advert_t *mavlink_log_pub, const vehicle_status_flags_s &status_flags, const safety_s &safety,
		  const uint8_t arm_requirements);


// COM_LOW_BAT_ACT parameter values
typedef enum LOW_BAT_ACTION {
	WARNING = 0,		// Warning
	RETURN = 1,			// Return mode
	LAND = 2,			// Land mode
	RETURN_OR_LAND = 3	// Return mode at critically low level, Land mode at current position if reaching dangerously low levels
} low_battery_action_t;

void battery_failsafe(orb_advert_t *mavlink_log_pub, const vehicle_status_s &status,
		      const vehicle_status_flags_s &status_flags, commander_state_s *internal_state, const uint8_t battery_warning,
		      const low_battery_action_t low_bat_action);

#endif /* STATE_MACHINE_HELPER_H_ */
