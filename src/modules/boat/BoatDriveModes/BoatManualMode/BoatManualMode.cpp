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

#include "BoatManualMode.hpp"

using namespace time_literals;

BoatManualMode::BoatManualMode(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
	_boat_throttle_setpoint_pub.advertise();
	_boat_rudder_setpoint_pub.advertise();
}

void BoatManualMode::publishActuatorValues(float throttle_setpoint, float steering_setpoint)
{
	hrt_abstime now = hrt_absolute_time();

	actuator_angle_setpoints_s boat_rudder_setpoint{};
	boat_rudder_setpoint.angle_setpoint[0] = steering_setpoint;
	boat_rudder_setpoint.timestamp = now;
	_boat_rudder_setpoint_pub.publish(boat_rudder_setpoint);

	actuator_motors_s boat_throttle_setpoint{};
	boat_throttle_setpoint.reversible_flags = true;
	boat_throttle_setpoint.control[0] = throttle_setpoint;
	boat_throttle_setpoint.timestamp = now;
	_boat_throttle_setpoint_pub.publish(boat_throttle_setpoint);
}

void BoatManualMode::manual()
{
	_sticks.checkAndUpdateStickInputs();

	float throttle_setpoint = _sticks.getThrottleZeroCenteredExpo();
	float steering_setpoint = _sticks.getRollExpo(0.0) * _param_boat_max_steer_angle.get();
	publishActuatorValues(throttle_setpoint, steering_setpoint);
}

void BoatManualMode::stop()
{
	publishActuatorValues(0.f, 0.f);
}
