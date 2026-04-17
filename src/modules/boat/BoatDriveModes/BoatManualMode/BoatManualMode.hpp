/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#pragma once

// uORB includes
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <lib/sticks/Sticks.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_angle_setpoints.h>

class BoatManualMode: public ModuleParams
{
public:
	/**
	 * @brief Constructor for BoatManualMode.
	 * @param parent The parent ModuleParams object.
	 */
	BoatManualMode(ModuleParams *parent);;
	~BoatManualMode() = default;

	/**
	 * @brief Publish boatThrottleSetpoint and boatRudderSetpoint from manualControlSetpoint.
	 */
	void manual();

	/**
	 * @brief Stop the vehicle by publishing zero throttle and steering.
	 */
	void stop();


	/**
	 * @brief Reset manual mode variables.
	 */
	void reset() {};

private:
	/**
	 * @brief Publish actuator values for throttle and steering.
	 * @param throttle_setpoint The desired throttle setpoint.
	 * @param steering_setpoint The desired steering setpoint.
	 */
	void publishActuatorValues(float throttle_setpoint, float steering_setpoint);

	// uORB publications
	uORB::Publication<actuator_motors_s> _boat_throttle_setpoint_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_angle_setpoints_s> _boat_rudder_setpoint_pub{ORB_ID(steering_angle_setpoint)};

	// Manage stick inputs
	Sticks _sticks{this};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BO_STEER_MAX>) _param_boat_max_steer_angle
	)
};
