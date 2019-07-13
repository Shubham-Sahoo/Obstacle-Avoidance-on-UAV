/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file state_machine_helper.cpp
 * State machine helper functions implementations
 *
 * @author Thomas Gubler	<thomas@px4.io>
 * @author Julian Oes		<julian@oes.ch>
 * @author Sander Smeets	<sander@droneslab.com>
 */
#include <px4_config.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_status.h>
#include <systemlib/mavlink_log.h>
#include <drivers/drv_hrt.h>


#include "state_machine_helper.h"
#include "commander_helper.h"
#include "PreflightCheck.h"
#include "arm_auth.h"

static constexpr const char reason_no_rc[] = "no RC";
static constexpr const char reason_no_offboard[] = "no offboard";
static constexpr const char reason_no_rc_and_no_offboard[] = "no RC and no offboard";
static constexpr const char reason_no_local_position[] = "no local position";
static constexpr const char reason_no_global_position[] = "no global position";
static constexpr const char reason_no_datalink[] = "no datalink";

// This array defines the arming state transitions. The rows are the new state, and the columns
// are the current state. Using new state and current state you can index into the array which
// will be true for a valid transition or false for a invalid transition. In some cases even
// though the transition is marked as true additional checks must be made. See arming_state_transition
// code for those checks.
static constexpr const bool arming_transitions[vehicle_status_s::ARMING_STATE_MAX][vehicle_status_s::ARMING_STATE_MAX]
= {
	//                                                    INIT,  STANDBY, ARMED, STANDBY_ERROR, REBOOT, IN_AIR_RESTORE
	{ /* vehicle_status_s::ARMING_STATE_INIT */           true,  true,    false, true,          false,  false },
	{ /* vehicle_status_s::ARMING_STATE_STANDBY */        true,  true,    true,  false,         false,  false },
	{ /* vehicle_status_s::ARMING_STATE_ARMED */          false, true,    true,  false,         false,  true },
	{ /* vehicle_status_s::ARMING_STATE_STANDBY_ERROR */  true,  true,    true,  true,          false,  false },
	{ /* vehicle_status_s::ARMING_STATE_REBOOT */         true,  true,    false, true,          true,   true },
	{ /* vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE */ false, false,   false, false,         false,  false }, // NYI
};

// You can index into the array with an arming_state_t in order to get its textual representation
const char *const arming_state_names[vehicle_status_s::ARMING_STATE_MAX] = {
	"INIT",
	"STANDBY",
	"ARMED",
	"STANDBY_ERROR",
	"REBOOT",
	"IN_AIR_RESTORE",
};

static hrt_abstime last_preflight_check = 0;	///< initialize so it gets checked immediately

void set_link_loss_nav_state(vehicle_status_s *status, actuator_armed_s *armed,
			     const vehicle_status_flags_s &status_flags, commander_state_s *internal_state, const link_loss_actions_t link_loss_act,
			     uint8_t auto_recovery_nav_state);

void reset_link_loss_globals(actuator_armed_s *armed, const bool old_failsafe, const link_loss_actions_t link_loss_act);

transition_result_t arming_state_transition(vehicle_status_s *status, const safety_s &safety,
		const arming_state_t new_arming_state, actuator_armed_s *armed, const bool fRunPreArmChecks,
		orb_advert_t *mavlink_log_pub, vehicle_status_flags_s *status_flags, const uint8_t arm_requirements,
		const hrt_abstime &time_since_boot)
{
	// Double check that our static arrays are still valid
	static_assert(vehicle_status_s::ARMING_STATE_INIT == 0, "ARMING_STATE_INIT == 0");
	static_assert(vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE == vehicle_status_s::ARMING_STATE_MAX - 1,
		      "ARMING_STATE_IN_AIR_RESTORE == ARMING_STATE_MAX - 1");

	transition_result_t ret = TRANSITION_DENIED;
	arming_state_t current_arming_state = status->arming_state;
	bool feedback_provided = false;

	const bool hil_enabled = (status->hil_state == vehicle_status_s::HIL_STATE_ON);

	/* only check transition if the new state is actually different from the current one */
	if (new_arming_state == current_arming_state) {
		ret = TRANSITION_NOT_CHANGED;

	} else {

		/*
		 * Get sensing state if necessary
		 */
		bool preflight_check_ret = true;
		bool prearm_check_ret = true;

		const bool checkGNSS = (arm_requirements & ARM_REQ_GPS_BIT);

		/* only perform the pre-arm check if we have to */
		if (fRunPreArmChecks && (new_arming_state == vehicle_status_s::ARMING_STATE_ARMED)
		    && !hil_enabled) {

			preflight_check_ret = Preflight::preflightCheck(mavlink_log_pub, *status, *status_flags, checkGNSS, true, true,
					      time_since_boot);

			if (preflight_check_ret) {
				status_flags->condition_system_sensors_initialized = true;
			}

			feedback_provided = true;
		}

		/* re-run the pre-flight check as long as sensors are failing */
		if (!status_flags->condition_system_sensors_initialized
		    && fRunPreArmChecks
		    && ((new_arming_state == vehicle_status_s::ARMING_STATE_ARMED)
			|| (new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY))
		    && !hil_enabled) {

			if ((last_preflight_check == 0) || (hrt_elapsed_time(&last_preflight_check) > 1000 * 1000)) {

				status_flags->condition_system_sensors_initialized = Preflight::preflightCheck(mavlink_log_pub, *status, *status_flags,
						checkGNSS, false, false, time_since_boot);

				last_preflight_check = hrt_absolute_time();
			}
		}

		// Check that we have a valid state transition
		bool valid_transition = arming_transitions[new_arming_state][status->arming_state];

		if (valid_transition) {
			// We have a good transition. Now perform any secondary validation.
			if (new_arming_state == vehicle_status_s::ARMING_STATE_ARMED) {

				//      Do not perform pre-arm checks if coming from in air restore
				//      Allow if vehicle_status_s::HIL_STATE_ON
				if (status->arming_state != vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE &&
				    !hil_enabled) {

					if (preflight_check_ret) {
						// only bother running prearm if preflight was successful
						prearm_check_ret = prearm_check(mavlink_log_pub, *status_flags, safety, arm_requirements);
					}

					if (!(preflight_check_ret && prearm_check_ret)) {
						// the prearm and preflight checks already print the rejection reason
						feedback_provided = true;
						valid_transition = false;
					}
				}
			}
		}

		if (hil_enabled) {
			/* enforce lockdown in HIL */
			armed->lockdown = true;
			status_flags->condition_system_sensors_initialized = true;

			/* recover from a prearm fail */
			if (status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY_ERROR) {
				status->arming_state = vehicle_status_s::ARMING_STATE_STANDBY;
			}

			// HIL can always go to standby
			if (new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
				valid_transition = true;
			}
		}

		if (!hil_enabled &&
		    (new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY) &&
		    (status->arming_state != vehicle_status_s::ARMING_STATE_STANDBY_ERROR)) {

			// Sensors need to be initialized for STANDBY state, except for HIL
			if (!status_flags->condition_system_sensors_initialized) {
				feedback_provided = true;
				valid_transition = false;
			}
		}

		// Finish up the state transition
		if (valid_transition) {
			armed->armed = (new_arming_state == vehicle_status_s::ARMING_STATE_ARMED);
			armed->ready_to_arm = (new_arming_state == vehicle_status_s::ARMING_STATE_ARMED)
					      || (new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY);
			ret = TRANSITION_CHANGED;
			status->arming_state = new_arming_state;

			if (new_arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
				armed->armed_time_ms = hrt_absolute_time() / 1000;

			} else {
				armed->armed_time_ms = 0;
			}
		}
	}

	if (ret == TRANSITION_DENIED) {
		/* print to MAVLink and console if we didn't provide any feedback yet */
		if (!feedback_provided) {
			mavlink_log_critical(mavlink_log_pub, "Transition denied: %s to %s",
					     arming_state_names[status->arming_state], arming_state_names[new_arming_state]);
		}
	}

	return ret;
}

bool is_safe(const safety_s &safety, const actuator_armed_s &armed)
{
	// System is safe if:
	// 1) Not armed
	// 2) Armed, but in software lockdown (HIL)
	// 3) Safety switch is present AND engaged -> actuators locked
	const bool lockdown = (armed.lockdown || armed.manual_lockdown);

	return !armed.armed || (armed.armed && lockdown) || (safety.safety_switch_available && !safety.safety_off);
}

transition_result_t
main_state_transition(const vehicle_status_s &status, const main_state_t new_main_state,
		      const vehicle_status_flags_s &status_flags, commander_state_s *internal_state)
{
	// IMPORTANT: The assumption of callers of this function is that the execution of
	// this check is essentially "free". Therefore any runtime checking in here has to be
	// kept super lightweight. No complex logic or calls on external function should be
	// implemented here.

	transition_result_t ret = TRANSITION_DENIED;

	/* transition may be denied even if the same state is requested because conditions may have changed */
	switch (new_main_state) {
	case commander_state_s::MAIN_STATE_MANUAL:
	case commander_state_s::MAIN_STATE_STAB:
	case commander_state_s::MAIN_STATE_ACRO:
	case commander_state_s::MAIN_STATE_RATTITUDE:
		ret = TRANSITION_CHANGED;
		break;

	case commander_state_s::MAIN_STATE_ALTCTL:

		/* need at minimum altitude estimate */
		if (status_flags.condition_local_altitude_valid ||
		    status_flags.condition_global_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_POSCTL:

		/* need at minimum local position estimate */
		if (status_flags.condition_local_position_valid ||
		    status_flags.condition_global_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_LOITER:

		/* need global position estimate */
		if (status_flags.condition_global_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET:
	case commander_state_s::MAIN_STATE_ORBIT:

		/* Follow and orbit only implemented for multicopter */
		if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_MISSION:

		/* need global position, home position, and a valid mission */
		if (status_flags.condition_global_position_valid &&
		    status_flags.condition_auto_mission_available) {

			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_RTL:

		/* need global position and home position */
		if (status_flags.condition_global_position_valid && status_flags.condition_home_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_TAKEOFF:
	case commander_state_s::MAIN_STATE_AUTO_LAND:

		/* need local position */
		if (status_flags.condition_local_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_PRECLAND:

		/* need local and global position, and precision land only implemented for multicopters */
		if (status_flags.condition_local_position_valid
		    && status_flags.condition_global_position_valid
		    && status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_OFFBOARD:

		/* need offboard signal */
		if (!status_flags.offboard_control_signal_lost) {

			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_MAX:
	default:
		break;
	}

	if (ret == TRANSITION_CHANGED) {
		if (internal_state->main_state != new_main_state) {
			internal_state->main_state = new_main_state;
			internal_state->timestamp = hrt_absolute_time();

		} else {
			ret = TRANSITION_NOT_CHANGED;
		}
	}

	return ret;
}

/**
 * Enable failsafe and report to user
 */
void enable_failsafe(vehicle_status_s *status, bool old_failsafe, orb_advert_t *mavlink_log_pub, const char *reason)
{
	if (!old_failsafe && status->arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		mavlink_log_critical(mavlink_log_pub, "Failsafe enabled: %s", reason);
	}

	status->failsafe = true;
}

/**
 * Check failsafe and main status and set navigation status for navigator accordingly
 */
bool set_nav_state(vehicle_status_s *status, actuator_armed_s *armed, commander_state_s *internal_state,
		   orb_advert_t *mavlink_log_pub, const link_loss_actions_t data_link_loss_act, const bool mission_finished,
		   const bool stay_in_failsafe, const vehicle_status_flags_s &status_flags, bool landed,
		   const link_loss_actions_t rc_loss_act, const int offb_loss_act, const int offb_loss_rc_act,
		   const int posctl_nav_loss_act)
{
	navigation_state_t nav_state_old = status->nav_state;

	const bool data_link_loss_act_configured = data_link_loss_act > link_loss_actions_t::DISABLED;
	const bool rc_loss_act_configured = rc_loss_act > link_loss_actions_t::DISABLED;
	const bool rc_lost = rc_loss_act_configured && (status->rc_signal_lost);

	bool is_armed = (status->arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	bool old_failsafe = status->failsafe;
	status->failsafe = false;

	// Safe to do reset flags here, as if loss state persists flags will be restored in the code below
	reset_link_loss_globals(armed, old_failsafe, rc_loss_act);
	reset_link_loss_globals(armed, old_failsafe, data_link_loss_act);

	/* evaluate main state to decide in normal (non-failsafe) mode */
	switch (internal_state->main_state) {
	case commander_state_s::MAIN_STATE_ACRO:
	case commander_state_s::MAIN_STATE_MANUAL:
	case commander_state_s::MAIN_STATE_RATTITUDE:
	case commander_state_s::MAIN_STATE_STAB:
	case commander_state_s::MAIN_STATE_ALTCTL:

		/* require RC for all manual modes */
		if (rc_lost && is_armed) {
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_rc);

			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act,
						vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER);

		} else {
			switch (internal_state->main_state) {
			case commander_state_s::MAIN_STATE_ACRO:
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_ACRO;
				break;

			case commander_state_s::MAIN_STATE_MANUAL:
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
				break;

			case commander_state_s::MAIN_STATE_RATTITUDE:
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_RATTITUDE;
				break;

			case commander_state_s::MAIN_STATE_STAB:
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_STAB;
				break;

			case commander_state_s::MAIN_STATE_ALTCTL:
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_ALTCTL;
				break;

			default:
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
				break;
			}
		}

		break;

	case commander_state_s::MAIN_STATE_POSCTL: {

			if (rc_lost && is_armed) {
				enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_rc);

				set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act,
							vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER);

				/* As long as there is RC, we can fallback to ALTCTL, or STAB. */
				/* A local position estimate is enough for POSCTL for multirotors,
				 * this enables POSCTL using e.g. flow.
				 * For fixedwing, a global position is needed. */

			} else if (is_armed
				   && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, !(posctl_nav_loss_act == 1),
						   status->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING)) {
				// nothing to do - everything done in check_invalid_pos_nav_state

			} else {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;
			}
		}
		break;

	case commander_state_s::MAIN_STATE_AUTO_MISSION:

		/* go into failsafe
		 * - if commanded to do so
		 * - if we have an engine failure
		 * - if we have vtol transition failure
		 * - depending on datalink, RC and if the mission is finished */

		if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, true)) {
			// nothing to do - everything done in check_invalid_pos_nav_state
		} else if (status->engine_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (status_flags.vtol_transition_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

		} else if (status->mission_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

		} else if (data_link_loss_act_configured && status->data_link_lost && is_armed) {
			/* datalink loss enabled:
			 * check for datalink lost: this should always trigger RTGS */
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_datalink);

			set_link_loss_nav_state(status, armed, status_flags, internal_state, data_link_loss_act,
						vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS);

		} else if (!data_link_loss_act_configured && status->rc_signal_lost && status->data_link_lost && !landed
			   && mission_finished && is_armed) {
			/* datalink loss DISABLED:
			 * check if both, RC and datalink are lost during the mission
			 * or all links are lost after the mission finishes in air: this should always trigger RCRECOVER */
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_datalink);

			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act,
						vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER);

		} else if (!stay_in_failsafe) {
			/* stay where you are if you should stay in failsafe, otherwise everything is perfect */
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_LOITER:

		/* go into failsafe on a engine failure */
		if (status->engine_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, true)) {
			// nothing to do - everything done in check_invalid_pos_nav_state
		} else if (status->data_link_lost && data_link_loss_act_configured && !landed && is_armed) {
			/* also go into failsafe if just datalink is lost, and we're actually in air */
			set_link_loss_nav_state(status, armed, status_flags, internal_state, data_link_loss_act,
						vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS);

			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_datalink);

		} else if (rc_lost && !data_link_loss_act_configured && is_armed) {
			/* go into failsafe if RC is lost and datalink loss is not set up and rc loss is not DISABLED */
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_rc);

			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act,
						vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER);

		} else if (status->rc_signal_lost) {
			/* don't bother if RC is lost if datalink is connected */

			/* this mode is ok, we don't need RC for LOITERing */
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;

		} else {
			/* everything is perfect */
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_RTL:

		/* require global position and home, also go into failsafe on an engine failure */

		if (status->engine_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, true)) {
			// nothing to do - everything done in check_invalid_pos_nav_state
		} else {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET:

		/* require global position and home */

		if (status->engine_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, true)) {
			// nothing to do - everything done in check_invalid_pos_nav_state

		} else {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET;
		}

		break;

	case commander_state_s::MAIN_STATE_ORBIT:
		if (status->engine_failure) {
			// failsafe: on engine failure
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, true)) {
			// failsafe: necessary position estimate lost (nothing to do - everything done in check_invalid_pos_nav_state)
		} else if (status->data_link_lost && data_link_loss_act_configured && !landed && is_armed) {
			// failsafe: just datalink is lost and we're in air
			set_link_loss_nav_state(status, armed, status_flags, internal_state, data_link_loss_act,
						vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS);

			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_datalink);

		} else if (rc_lost && !data_link_loss_act_configured && is_armed) {
			// failsafe: RC is lost, datalink loss is not set up and rc loss is not disabled
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_rc);

			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act,
						vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER);

		} else {
			// no failsafe, RC is not mandatory for orbit
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_ORBIT;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_TAKEOFF:

		/* require local position */

		if (status->engine_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, false)) {
			// nothing to do - everything done in check_invalid_pos_nav_state

		} else {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_LAND:

		/* require local position */

		if (status->engine_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, false)) {
			// nothing to do - everything done in check_invalid_pos_nav_state

		} else {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_PRECLAND:

		/* must be rotary wing plus same requirements as normal landing */

		if (status->engine_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, false)) {
			// nothing to do - everything done in check_invalid_pos_nav_state

		} else {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND;
		}

		break;

	case commander_state_s::MAIN_STATE_OFFBOARD:

		/* require offboard control, otherwise stay where you are */
		if (status_flags.offboard_control_signal_lost && !status->rc_signal_lost) {
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_offboard);

			if (status_flags.offboard_control_loss_timeout && offb_loss_rc_act < 6 && offb_loss_rc_act >= 0) {
				if (offb_loss_rc_act == 3 && status_flags.condition_global_position_valid
				    && status_flags.condition_home_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

				} else if (offb_loss_rc_act == 0 && status_flags.condition_global_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;

				} else if (offb_loss_rc_act == 1 && status_flags.condition_local_altitude_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_ALTCTL;

				} else if (offb_loss_rc_act == 2) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;

				} else if (offb_loss_rc_act == 4 && status_flags.condition_global_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

				} else if (offb_loss_rc_act == 5 && status_flags.condition_global_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;

				} else if (status_flags.condition_local_altitude_valid) {
					//TODO: Add case for rover
					if (status->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
						status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

					} else {
						// TODO: FW position controller doesn't run without condition_global_position_valid
						status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL;
					}

				} else {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
				}

			} else {
				if (status_flags.condition_global_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;

				} else if (status_flags.condition_local_altitude_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_ALTCTL;

				} else {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
				}
			}

		} else if (status_flags.offboard_control_signal_lost && status->rc_signal_lost) {
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_rc_and_no_offboard);

			if (status_flags.offboard_control_loss_timeout && offb_loss_act < 3 && offb_loss_act >= 0) {
				if (offb_loss_act == 2 && status_flags.condition_global_position_valid
				    && status_flags.condition_home_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

				} else if (offb_loss_act == 1 && status_flags.condition_global_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;

				} else if (offb_loss_act == 0 && status_flags.condition_global_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

				} else if (status_flags.condition_local_altitude_valid) {
					if (status->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
						status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

					} else {
						// TODO: FW position controller doesn't run without condition_global_position_valid
						status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL;
					}

				} else {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
				}

			} else {
				if (status_flags.condition_global_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;

				} else if (status_flags.condition_local_altitude_valid) {
					if (status->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
						status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

					} else {
						// TODO: FW position controller doesn't run without condition_global_position_valid
						status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL;
					}

				} else {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
				}
			}

		} else {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
		}

	default:
		break;
	}

	return status->nav_state != nav_state_old;
}

bool check_invalid_pos_nav_state(vehicle_status_s *status, bool old_failsafe, orb_advert_t *mavlink_log_pub,
				 const vehicle_status_flags_s &status_flags, const bool use_rc, const bool using_global_pos)
{
	bool fallback_required = false;

	if (using_global_pos && !status_flags.condition_global_position_valid) {
		fallback_required = true;

	} else if (!using_global_pos
		   && (!status_flags.condition_local_position_valid || !status_flags.condition_local_velocity_valid)) {

		fallback_required = true;
	}

	if (fallback_required) {
		if (use_rc) {
			// fallback to a mode that gives the operator stick control
			if (status->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
			    && status_flags.condition_local_position_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;

			} else if (status_flags.condition_local_altitude_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_ALTCTL;

			} else {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_STAB;
			}

		} else {
			// go into a descent that does not require stick control
			if (status_flags.condition_local_position_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

			} else  if (status_flags.condition_local_altitude_valid) {
				if (status->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

				} else {
					// TODO: FW position controller doesn't run without condition_global_position_valid
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL;
				}

			} else {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
			}
		}

		if (using_global_pos) {
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_global_position);

		} else {
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_local_position);
		}

	}

	return fallback_required;

}

void set_link_loss_nav_state(vehicle_status_s *status, actuator_armed_s *armed,
			     const vehicle_status_flags_s &status_flags, commander_state_s *internal_state, const link_loss_actions_t link_loss_act,
			     uint8_t auto_recovery_nav_state)
{
	// do the best you can according to the action set

	switch (link_loss_act) {
	case link_loss_actions_t::DISABLED:
		// If datalink loss failsafe is disabled then no action must be taken.
		break;

	case link_loss_actions_t::AUTO_RECOVER:
		if (status_flags.condition_global_position_valid && status_flags.condition_home_position_valid) {
			status->nav_state = auto_recovery_nav_state;
			return;
		}

	// FALLTHROUGH
	case link_loss_actions_t::AUTO_RTL:
		if (status_flags.condition_global_position_valid && status_flags.condition_home_position_valid) {
			main_state_transition(*status, commander_state_s::MAIN_STATE_AUTO_RTL, status_flags, internal_state);
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
			return;
		}

	// FALLTHROUGH
	case link_loss_actions_t::AUTO_LOITER:
		if (status_flags.condition_global_position_valid) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
			return;
		}

	// FALLTHROUGH
	case link_loss_actions_t::AUTO_LAND:
		if (status_flags.condition_global_position_valid) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
			return;

		} else {
			if (status->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				if (status_flags.condition_local_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
					return;

				} else if (status_flags.condition_local_altitude_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;
					return;
				}

			} else {
				// TODO: FW position controller doesn't run without condition_global_position_valid
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL;
				return;
			}
		}

	// FALLTHROUGH
	case link_loss_actions_t::TERMINATE:
		status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
		armed->force_failsafe = true;
		break;

	case link_loss_actions_t::LOCKDOWN:
		armed->lockdown = true;
		break;
	}
}

void reset_link_loss_globals(actuator_armed_s *armed, const bool old_failsafe, const link_loss_actions_t link_loss_act)
{
	if (old_failsafe) {
		if (link_loss_act == link_loss_actions_t::TERMINATE) {
			armed->force_failsafe = false;

		} else if (link_loss_act == link_loss_actions_t::LOCKDOWN) {
			armed->lockdown = false;
		}
	}
}

bool prearm_check(orb_advert_t *mavlink_log_pub, const vehicle_status_flags_s &status_flags, const safety_s &safety,
		  const uint8_t arm_requirements)
{
	bool reportFailures = true;
	bool prearm_ok = true;


	// USB not connected
	if (!status_flags.circuit_breaker_engaged_usb_check && status_flags.usb_connected) {
		if (reportFailures) {
			mavlink_log_critical(mavlink_log_pub, "Arming denied! Flying with USB is not safe");
		}

		prearm_ok = false;
	}

	// battery and system power status
	if (!status_flags.circuit_breaker_engaged_power_check) {

		// Fail transition if power is not good
		if (!status_flags.condition_power_input_valid) {
			if (reportFailures) {
				mavlink_log_critical(mavlink_log_pub, "Arming denied! Connect power module");
			}

			prearm_ok = false;
		}

		// main battery level
		if (!status_flags.condition_battery_healthy) {
			if (prearm_ok && reportFailures) {
				mavlink_log_critical(mavlink_log_pub, "Arming denied! Check battery");
			}

			prearm_ok = false;
		}
	}

	// Arm Requirements: mission
	if (arm_requirements & ARM_REQ_MISSION_BIT) {

		if (!status_flags.condition_auto_mission_available) {
			if (prearm_ok && reportFailures) {
				mavlink_log_critical(mavlink_log_pub, "Arming denied! No valid mission");
			}

			prearm_ok = false;
		}

		if (!status_flags.condition_global_position_valid) {
			if (prearm_ok && reportFailures) {
				mavlink_log_critical(mavlink_log_pub, "Arming denied! Missions require a global position");
			}

			prearm_ok = false;
		}
	}

	// Arm Requirements: global position
	if (arm_requirements & ARM_REQ_GPS_BIT) {

		if (!status_flags.condition_global_position_valid) {
			if (prearm_ok && reportFailures) {
				mavlink_log_critical(mavlink_log_pub, "Arming denied! Global position required");
			}

			prearm_ok = false;
		}
	}

	// safety button
	if (safety.safety_switch_available && !safety.safety_off) {
		// Fail transition if we need safety switch press
		if (prearm_ok && reportFailures) {
			mavlink_log_critical(mavlink_log_pub, "Arming denied! Press safety switch first");
		}

		prearm_ok = false;
	}

	// Arm Requirements: authorization
	// check last, and only if everything else has passed
	if ((arm_requirements & ARM_REQ_ARM_AUTH_BIT) && prearm_ok) {
		if (arm_auth_check() != vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED) {
			// feedback provided in arm_auth_check
			prearm_ok = false;
		}
	}

	if (status_flags.avoidance_system_required && !status_flags.avoidance_system_valid) {
		if (prearm_ok && reportFailures) {
			mavlink_log_critical(mavlink_log_pub, "Arming denied! Avoidance system not ready");
		}

		prearm_ok = false;

	}

	return prearm_ok;
}


void battery_failsafe(orb_advert_t *mavlink_log_pub, const vehicle_status_s &status,
		      const vehicle_status_flags_s &status_flags, commander_state_s *internal_state, const uint8_t battery_warning,
		      const low_battery_action_t low_battery_action)
{
	switch (battery_warning) {
	case battery_status_s::BATTERY_WARNING_NONE:
		break;

	case battery_status_s::BATTERY_WARNING_LOW:
		mavlink_log_critical(mavlink_log_pub, "Low battery level! Return advised");
		break;

	case battery_status_s::BATTERY_WARNING_CRITICAL:

		static constexpr char battery_critical[] = "Critical battery level!";

		switch (low_battery_action) {
		case LOW_BAT_ACTION::WARNING:
			mavlink_log_critical(mavlink_log_pub, "%s, Landing advised", battery_critical);
			break;

		case LOW_BAT_ACTION::RETURN:

		// FALLTHROUGH
		case LOW_BAT_ACTION::RETURN_OR_LAND:

			// let us send the critical message even if already in RTL
			if (TRANSITION_DENIED != main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_RTL, status_flags,
					internal_state)) {
				mavlink_log_critical(mavlink_log_pub, "%s, Executing RTL", battery_critical);

			} else {
				mavlink_log_emergency(mavlink_log_pub, "%s, Can't execute RTL", battery_critical);
			}

			break;

		case LOW_BAT_ACTION::LAND:
			if (TRANSITION_DENIED != main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_LAND, status_flags,
					internal_state)) {
				mavlink_log_critical(mavlink_log_pub, "%s, Landing immediately", battery_critical);

			} else {
				mavlink_log_emergency(mavlink_log_pub, "%s, Can't execute landing", battery_critical);
			}

			break;
		}

		break;

	case battery_status_s::BATTERY_WARNING_EMERGENCY:

		static constexpr char battery_dangerous[] = "Dangerous battery level!";

		switch (low_battery_action) {
		case LOW_BAT_ACTION::WARNING:
			mavlink_log_emergency(mavlink_log_pub, "%s, Please land!", battery_dangerous);
			break;

		case LOW_BAT_ACTION::RETURN:
			if (TRANSITION_DENIED != main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_RTL, status_flags,
					internal_state)) {
				mavlink_log_emergency(mavlink_log_pub, "%s, Executing RTL", battery_dangerous);

			} else {
				mavlink_log_emergency(mavlink_log_pub, "%s, Can't execute RTL", battery_dangerous);
			}

			break;

		case LOW_BAT_ACTION::RETURN_OR_LAND:

		// FALLTHROUGH
		case LOW_BAT_ACTION::LAND:
			if (TRANSITION_DENIED != main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_LAND, status_flags,
					internal_state)) {
				mavlink_log_emergency(mavlink_log_pub, "%s, Landing immediately", battery_dangerous);

			} else {
				mavlink_log_emergency(mavlink_log_pub, "%s, Can't execute landing", battery_dangerous);
			}

			break;
		}

		break;

	case battery_status_s::BATTERY_WARNING_FAILED:
		mavlink_log_emergency(mavlink_log_pub, "Battery failure detected");
		break;
	}
}
