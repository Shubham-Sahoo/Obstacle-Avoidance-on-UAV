#include "FlightTask.hpp"
#include <mathlib/mathlib.h>
#include <lib/ecl/geo/geo.h>

constexpr uint64_t FlightTask::_timeout;
// First index of empty_setpoint corresponds to time-stamp and requires a finite number.
const vehicle_local_position_setpoint_s FlightTask::empty_setpoint = {0, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, {NAN, NAN, NAN}};

const vehicle_constraints_s FlightTask::empty_constraints = {0, NAN, NAN, NAN, NAN, NAN, NAN, NAN, false, {}};
const landing_gear_s FlightTask::empty_landing_gear_default_keep = {0, landing_gear_s::GEAR_KEEP, {}};

bool FlightTask::initializeSubscriptions(SubscriptionArray &subscription_array)
{
	if (!subscription_array.get(ORB_ID(vehicle_local_position), _sub_vehicle_local_position)) {
		return false;
	}

	if (!subscription_array.get(ORB_ID(vehicle_attitude), _sub_attitude)) {
		return false;
	}

	return true;
}

bool FlightTask::activate()
{
	_resetSetpoints();
	_setDefaultConstraints();
	_time_stamp_activate = hrt_absolute_time();
	_heading_reset_counter = _sub_attitude->get().quat_reset_counter;
	_gear = empty_landing_gear_default_keep;
	return true;
}

void FlightTask::reActivate()
{
	activate();
}

bool FlightTask::updateInitialize()
{
	_time_stamp_current = hrt_absolute_time();
	_time = (_time_stamp_current - _time_stamp_activate) / 1e6f;
	_deltatime  = math::min((_time_stamp_current - _time_stamp_last), _timeout) / 1e6f;
	_time_stamp_last = _time_stamp_current;
	_evaluateVehicleLocalPosition();
	return true;
}

const vehicle_local_position_setpoint_s FlightTask::getPositionSetpoint()
{
	/* fill position setpoint message */
	vehicle_local_position_setpoint_s vehicle_local_position_setpoint;
	vehicle_local_position_setpoint.timestamp = hrt_absolute_time();

	vehicle_local_position_setpoint.x = _position_setpoint(0);
	vehicle_local_position_setpoint.y = _position_setpoint(1);
	vehicle_local_position_setpoint.z = _position_setpoint(2);

	vehicle_local_position_setpoint.vx = _velocity_setpoint(0);
	vehicle_local_position_setpoint.vy = _velocity_setpoint(1);
	vehicle_local_position_setpoint.vz = _velocity_setpoint(2);

	vehicle_local_position_setpoint.acc_x = _acceleration_setpoint(0);
	vehicle_local_position_setpoint.acc_y = _acceleration_setpoint(1);
	vehicle_local_position_setpoint.acc_z = _acceleration_setpoint(2);

	vehicle_local_position_setpoint.jerk_x = _jerk_setpoint(0);
	vehicle_local_position_setpoint.jerk_y = _jerk_setpoint(1);
	vehicle_local_position_setpoint.jerk_z = _jerk_setpoint(2);

	_thrust_setpoint.copyTo(vehicle_local_position_setpoint.thrust);
	vehicle_local_position_setpoint.yaw = _yaw_setpoint;
	vehicle_local_position_setpoint.yawspeed = _yawspeed_setpoint;

	return vehicle_local_position_setpoint;
}

void FlightTask::_resetSetpoints()
{
	_position_setpoint.setAll(NAN);
	_velocity_setpoint.setAll(NAN);
	_acceleration_setpoint.setAll(NAN);
	_jerk_setpoint.setAll(NAN);
	_thrust_setpoint.setAll(NAN);
	_yaw_setpoint = _yawspeed_setpoint = NAN;
}

void FlightTask::_evaluateVehicleLocalPosition()
{
	_position.setAll(NAN);
	_velocity.setAll(NAN);
	_yaw = NAN;
	_dist_to_bottom = NAN;

	if ((_time_stamp_current - _sub_attitude->get().timestamp) < _timeout) {
		// yaw
		_yaw = matrix::Eulerf(matrix::Quatf(_sub_attitude->get().q)).psi();
	}

	// Only use vehicle-local-position topic fields if the topic is received within a certain timestamp
	if ((_time_stamp_current - _sub_vehicle_local_position->get().timestamp) < _timeout) {

		// position
		if (_sub_vehicle_local_position->get().xy_valid) {
			_position(0) = _sub_vehicle_local_position->get().x;
			_position(1) = _sub_vehicle_local_position->get().y;
		}

		if (_sub_vehicle_local_position->get().z_valid) {
			_position(2) = _sub_vehicle_local_position->get().z;
		}

		// velocity
		if (_sub_vehicle_local_position->get().v_xy_valid) {
			_velocity(0) = _sub_vehicle_local_position->get().vx;
			_velocity(1) = _sub_vehicle_local_position->get().vy;
		}

		if (_sub_vehicle_local_position->get().v_z_valid) {
			_velocity(2) = _sub_vehicle_local_position->get().vz;
		}

		// distance to bottom
		if (_sub_vehicle_local_position->get().dist_bottom_valid
		    && PX4_ISFINITE(_sub_vehicle_local_position->get().dist_bottom)) {
			_dist_to_bottom =  _sub_vehicle_local_position->get().dist_bottom;
		}

		// global frame reference coordinates to enable conversions
		if (_sub_vehicle_local_position->get().xy_global && _sub_vehicle_local_position->get().z_global) {
			globallocalconverter_init(_sub_vehicle_local_position->get().ref_lat, _sub_vehicle_local_position->get().ref_lon,
						  _sub_vehicle_local_position->get().ref_alt, _sub_vehicle_local_position->get().ref_timestamp);
		}
	}
}

void FlightTask::_setDefaultConstraints()
{
	_constraints.speed_xy = _param_mpc_xy_vel_max.get();
	_constraints.speed_up = _param_mpc_z_vel_max_up.get();
	_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
	_constraints.tilt = math::radians(_param_mpc_tiltmax_air.get());
	_constraints.min_distance_to_ground = NAN;
	_constraints.max_distance_to_ground = NAN;
	_constraints.want_takeoff = false;
}

bool FlightTask::_checkTakeoff()
{
	// position setpoint above the minimum altitude
	bool position_triggered_takeoff = false;

	if (PX4_ISFINITE(_position_setpoint(2))) {
		// minimal altitude either 20cm or what is necessary for correct estimation e.g. optical flow
		float min_altitude = 0.2f;
		const float min_distance_to_ground = _sub_vehicle_local_position->get().hagl_min;

		if (PX4_ISFINITE(min_distance_to_ground)) {
			min_altitude = min_distance_to_ground + 0.05f;
		}

		position_triggered_takeoff = _position_setpoint(2) < (_position(2) - min_altitude);
	}

	// upwards velocity setpoint
	bool velocity_triggered_takeoff = false;

	if (PX4_ISFINITE(_velocity_setpoint(2))) {
		velocity_triggered_takeoff = _velocity_setpoint(2) < -0.3f;
	}

	return position_triggered_takeoff || velocity_triggered_takeoff;
}
