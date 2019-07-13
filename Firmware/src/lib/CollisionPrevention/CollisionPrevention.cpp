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
 * @file CollisionPrevention.cpp
 * CollisionPrevention controller.
 *
 */

#include <CollisionPrevention/CollisionPrevention.hpp>
using namespace matrix;
using namespace time_literals;


CollisionPrevention::CollisionPrevention(ModuleParams *parent) :
	ModuleParams(parent)
{

}

CollisionPrevention::~CollisionPrevention()
{
	//unadvertise publishers
	if (_mavlink_log_pub != nullptr) {
		orb_unadvertise(_mavlink_log_pub);
	}
}

void CollisionPrevention::publishConstrainedSetpoint(const Vector2f &original_setpoint,
		const Vector2f &adapted_setpoint)
{
	collision_constraints_s	constraints{};	/**< collision constraints message */

	//fill in values
	constraints.timestamp = hrt_absolute_time();

	constraints.original_setpoint[0] = original_setpoint(0);
	constraints.original_setpoint[1] = original_setpoint(1);
	constraints.adapted_setpoint[0] = adapted_setpoint(0);
	constraints.adapted_setpoint[1] = adapted_setpoint(1);

	// publish constraints
	if (_constraints_pub != nullptr) {
		orb_publish(ORB_ID(collision_constraints), _constraints_pub, &constraints);

	} else {
		_constraints_pub = orb_advertise(ORB_ID(collision_constraints), &constraints);
	}
}

void CollisionPrevention::calculateConstrainedSetpoint(Vector2f &setpoint,
		const Vector2f &curr_pos, const Vector2f &curr_vel)
{
	_sub_obstacle_distance.update();
	const obstacle_distance_s &obstacle_distance = _sub_obstacle_distance.get();

	//The maximum velocity formula contains a square root, therefore the whole calculation is done with squared norms.
	//that way the root does not have to be calculated for every range bin but once at the end.
	float setpoint_length = setpoint.norm();
	Vector2f setpoint_sqrd = setpoint * setpoint_length;

	//Limit the deviation of the adapted setpoint from the originally given joystick input (slightly less than 90 degrees)
	float max_slide_angle_rad = 0.5f;

	if (hrt_elapsed_time(&obstacle_distance.timestamp) < RANGE_STREAM_TIMEOUT_US) {
		if (setpoint_length > 0.001f) {

			int distances_array_size = sizeof(obstacle_distance.distances) / sizeof(obstacle_distance.distances[0]);

			for (int i = 0; i < distances_array_size; i++) {

				//determine if distance bin is valid and contains a valid distance measurement
				if (obstacle_distance.distances[i] < obstacle_distance.max_distance &&
				    obstacle_distance.distances[i] > obstacle_distance.min_distance && i * obstacle_distance.increment < 360) {
					float distance = obstacle_distance.distances[i] / 100.0f; //convert to meters
					float angle = math::radians((float)i * obstacle_distance.increment);

					//split current setpoint into parallel and orthogonal components with respect to the current bin direction
					Vector2f bin_direction = {cos(angle), sin(angle)};
					Vector2f orth_direction = {-bin_direction(1), bin_direction(0)};
					float sp_parallel = setpoint_sqrd.dot(bin_direction);
					float sp_orth = setpoint_sqrd.dot(orth_direction);
					float curr_vel_parallel = math::max(0.f, curr_vel.dot(bin_direction));

					//calculate max allowed velocity with a P-controller (same gain as in the position controller)
					float delay_distance = curr_vel_parallel * _param_mpc_col_prev_dly.get();
					float vel_max_posctrl = math::max(0.f,
									  _param_mpc_xy_p.get() * (distance - _param_mpc_col_prev_d.get() - delay_distance));
					float vel_max_sqrd = vel_max_posctrl * vel_max_posctrl;

					//limit the setpoint to respect vel_max by subtracting from the parallel component
					if (sp_parallel > vel_max_sqrd) {
						Vector2f setpoint_temp = setpoint_sqrd - (sp_parallel - vel_max_sqrd) * bin_direction;
						float setpoint_temp_length = setpoint_temp.norm();

						//limit sliding angle
						float angle_diff_temp_orig = acos(setpoint_temp.dot(setpoint) / (setpoint_temp_length * setpoint_length));
						float angle_diff_temp_bin = acos(setpoint_temp.dot(bin_direction) / setpoint_temp_length);

						if (angle_diff_temp_orig > max_slide_angle_rad && setpoint_temp_length > 0.001f) {
							float angle_temp_bin_cropped = angle_diff_temp_bin - (angle_diff_temp_orig - max_slide_angle_rad);
							float orth_len = vel_max_sqrd * tan(angle_temp_bin_cropped);

							if (sp_orth > 0) {
								setpoint_temp = vel_max_sqrd * bin_direction + orth_len * orth_direction;

							} else {
								setpoint_temp = vel_max_sqrd * bin_direction - orth_len * orth_direction;
							}
						}

						setpoint_sqrd = setpoint_temp;
					}
				}
			}

			//take the squared root
			if (setpoint_sqrd.norm() > 0.001f) {
				setpoint = setpoint_sqrd / std::sqrt(setpoint_sqrd.norm());

			} else {
				setpoint.zero();
			}
		}

	} else if (_last_message + MESSAGE_THROTTLE_US < hrt_absolute_time()) {
		mavlink_log_critical(&_mavlink_log_pub, "No range data received");
		_last_message = hrt_absolute_time();
	}
}

void CollisionPrevention::modifySetpoint(Vector2f &original_setpoint, const float max_speed,
		const Vector2f &curr_pos, const Vector2f &curr_vel)
{
	//calculate movement constraints based on range data
	Vector2f new_setpoint = original_setpoint;
	calculateConstrainedSetpoint(new_setpoint, curr_pos, curr_vel);

	//warn user if collision prevention starts to interfere
	bool currently_interfering = (new_setpoint(0) < original_setpoint(0) - 0.05f * max_speed
				      || new_setpoint(0) > original_setpoint(0) + 0.05f * max_speed
				      || new_setpoint(1) < original_setpoint(1) - 0.05f * max_speed
				      || new_setpoint(1) > original_setpoint(1) + 0.05f * max_speed);

	if (currently_interfering && (currently_interfering != _interfering)) {
		mavlink_log_critical(&_mavlink_log_pub, "Collision Warning");
	}

	_interfering = currently_interfering;
	publishConstrainedSetpoint(original_setpoint, new_setpoint);
	original_setpoint = new_setpoint;
}
