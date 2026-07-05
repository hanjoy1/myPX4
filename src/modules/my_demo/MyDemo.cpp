#include "MyDemo.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/events.h>


using namespace matrix;

MyDemo::MyDemo() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_sample_interval_s.update(0.01f); // 100 Hz default
	parameters_update(true);
}

MyDemo::~MyDemo()
{
	perf_free(_cycle_perf);
}

bool MyDemo::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	return true;
}

void MyDemo::parameters_update(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();

		_demo_bool = _param_my_demo_bool.get();
		_demo_float = _param_my_demo_float.get();
	}

}

void MyDemo::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	// main
	if (_demo_bool) {
		printf("Hello from my_demo module!\n");
		printf("Demo float parameter value: %.2f\n", (double)_demo_float);
	}

	// reschedule backup
	ScheduleDelayed(5000_ms);

	parameters_update(false);

	perf_end(_cycle_perf);
}

int MyDemo::task_spawn(int argc, char *argv[])
{
	MyDemo *instance = new MyDemo();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MyDemo::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MyDemo::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is a demonstration of a custom PX4 module. It showcases how to implement
a basic control loop and interact with the PX4 middleware.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("my_demo", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int my_demo_main(int argc, char *argv[])
{
	return MyDemo::main(argc, argv);
}
