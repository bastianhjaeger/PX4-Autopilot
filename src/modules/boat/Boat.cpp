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

#include "Boat.hpp"

using namespace time_literals;

ModuleBase::Descriptor Boat::desc{task_spawn, custom_command, print_usage};

Boat::Boat() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	updateParams();
}

bool Boat::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void Boat::Run()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update{};
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	if (_vehicle_control_mode_sub.updated()) {
		vehicle_control_mode_s new_vehicle_control_mode{};
		_vehicle_control_mode_sub.copy(&new_vehicle_control_mode);

		// Run sanity checks if the control mode changes (Note: This has to be done this way, because the topic is periodically updated at 2 Hz)
		if (_vehicle_control_mode.flag_control_position_enabled != new_vehicle_control_mode.flag_control_position_enabled ||
		    _vehicle_control_mode.flag_control_velocity_enabled != new_vehicle_control_mode.flag_control_velocity_enabled ||
		    _vehicle_control_mode.flag_control_attitude_enabled != new_vehicle_control_mode.flag_control_attitude_enabled ||
		    _vehicle_control_mode.flag_control_rates_enabled != new_vehicle_control_mode.flag_control_rates_enabled ||
		    _vehicle_control_mode.flag_control_allocation_enabled != new_vehicle_control_mode.flag_control_allocation_enabled ||
		    _vehicle_control_mode.flag_armed != new_vehicle_control_mode.flag_armed) {
			reset();
		}

		// Save the new control mode as current control mode
		_vehicle_control_mode = new_vehicle_control_mode;

	}

	if (_vehicle_control_mode.flag_armed) {
		generateSetpoints();

	} else {
		_manual_mode.stop();
	}
}

void Boat::generateSetpoints()
{
	vehicle_status_s vehicle_status{};
	_vehicle_status_sub.copy(&vehicle_status);

	switch (vehicle_status.nav_state) {
	default:
		if (!_logged_once) {
			PX4_WARN("Boat module: Unsupported navigation state %i. Only manual mode is currently supported.", vehicle_status.nav_state);
			_logged_once = true;
		}

		[[fallthrough]];

	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		_manual_mode.manual(); // For now, position control mode behaves the same as manual mode
		break;
	}

}

void Boat::reset()
{
	_manual_mode.reset();
	_logged_once = false;
}

int Boat::task_spawn(int argc, char *argv[])
{
	Boat *instance = new Boat();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int Boat::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Boat::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Boat module.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("boat", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int boat_main(int argc, char *argv[])
{
	return ModuleBase::main(Boat::desc, argc, argv);
}
