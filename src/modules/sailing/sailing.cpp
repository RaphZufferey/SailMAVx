/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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


#include "sailing.h"

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

// #include <fcntl.h>
// #include <errno.h>
// #include <poll.h>
// #include <time.h>
// #include <drivers/drv_hrt.h>



#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>


// system libraries
#include <parameters/param.h>
#include <systemlib/err.h>
#include <perf/perf_counter.h>

// internal libraries
#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <lib/ecl/geo/geo.h>

// Include uORB and the required topics for this app
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>                // this topics hold the acceleration data
#include <uORB/topics/actuator_controls.h>              // this topic gives the actuators control input
#include <uORB/topics/vehicle_attitude.h>                  // this topic holds the orientation of the hippocampus
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/manual_control_setpoint.h> 

//px4_module_params.h to get the DEFINE_PARAMETERS macro
#include <px4_module_params.h>
#include "params.c"

float myPi = (float)M_PI;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template <typename T> float wrapToPi(T val) {
    
    val = fmod(val + myPi,2*myPi);
    if (val < 0)
        val += 2*myPi;
    return val - myPi;

}

extern "C" __EXPORT int sailing_main(int argc, char *argv[]);

int Sailing::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.
This is a template for a module running as a task in the background with start/stop/status functionality.
### Implementation
Section describing the high-level implementation of this module.
### Examples
CLI usage example:
$ module start -f -p 42
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sailing", "module");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int Sailing::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int Sailing::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}
	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int Sailing::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("sailing",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Sailing *Sailing::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	Sailing *instance = new Sailing(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

Sailing::Sailing(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void Sailing::run()
{

	float sail_angle_max = 60*(float)M_PI/180;


	// 	/* subscribe to control_state topic */
	int vehicle_attitude_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	/* limit the update rate to X ms */
	orb_set_interval(vehicle_attitude_sub_fd, 500); //200

	// subscribe to manual control setpoint
	struct manual_control_setpoint_s manual_sp;
	memset(&manual_sp, 0, sizeof(manual_sp));	
	int manual_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
 
	/* advertise to actuator_control topic */
	struct actuator_controls_s act;
	memset(&act, 0, sizeof(act));
	orb_advert_t act_pub = orb_advertise(ORB_ID(actuator_controls_0), &act);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = vehicle_attitude_sub_fd;
	fds[0].events = POLLIN;



	bool updated = false;
	int wnd_angle_to_n = 179;
	param_t param_wnd_angle_to_n = param_find("WND_ANGLE_TO_N");
	param_get(param_wnd_angle_to_n, &wnd_angle_to_n);
	
	struct parameter_update_s update;
	memset(&update, 0, sizeof(update));
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	// Get parameter updates

	// orb_check(parameter_update_sub, &updated);
	// if (updated) {
	// 	// copy global position
	// 	orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);

	// 	// update all parameters
	// 	param_get(param_wnd_angle_to_n, &wnd_angle_to_n);
	// }

	// initialize parameters
	//int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	parameters_update(parameter_update_sub, true);

	//int i = 0;
	while (!should_exit()) {

		orb_check(parameter_update_sub, &updated);
		if (updated) {
			// copy global position
			orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);

			// update all parameters
			param_get(param_wnd_angle_to_n, &wnd_angle_to_n);
			PX4_INFO("Wind angle parameter updated to : %d", wnd_angle_to_n);
			parameters_update(parameter_update_sub, true);
		}


		//if(++i > 10) should_exit = TRUE;
		//	PX4_INFO("Parameter %d", twindangle);

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {


			bool manual_sp_updated;
			orb_check(manual_sp_sub, &manual_sp_updated);
			if (manual_sp_updated)
			/* get the RC (or otherwise user based) input */
			{
				orb_copy(ORB_ID(manual_control_setpoint), manual_sp_sub, &manual_sp);
				PX4_INFO("manual sp: %f ", (double)(manual_sp.r));
	//				,(int)(manual_sp.x*100.0f)
	//				,(int)(manual_sp.y*100.0f));
			}




			/* obtained data for the third file descriptor */
			struct vehicle_attitude_s raw_att;
			memset(&raw_att, 0, sizeof(raw_att));
			/* copy sensors raw data into local buffer */
			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub_fd, &raw_att);
			float current_yaw =  matrix::Eulerf(matrix::Quatf(raw_att.q)).psi();
			//PX4_INFO("Yaw: \t%d", (int)(current_yaw*100));

			float wnd_angle_to_n_rad = wnd_angle_to_n*myPi/180;
			float wnd_to_boat = wrapToPi(wnd_angle_to_n_rad - current_yaw);
			float sail_angle = -sgn(wnd_to_boat)*M_PI/4*(cos(wnd_to_boat)+1);
			
			float cmd_sail_angle = sail_angle/sail_angle_max;
			// PX4_INFO("Yaw  \t%d, actuators: \t%d", 
			// 	(int)((current_yaw*180.0f/myPi)),
			// 	(int)((cmd_sail_angle*180.0f/myPi)));

			static int i = 0;
			if(++i >= 99) i = -99;

	 		//Control 
			//act.control[0] = current_yaw/3.1415f;   // roll = SAILS
			act.control[0] = cmd_sail_angle;   // roll = SAILS
			// act.control[1] = 1.0f;   // pitch
			act.control[2] = i/100.0f;	 // yaw = RUDDER (in SailMAV : servo line 5)
			// act.control[3] = 0.0f;	 // thrust
			//act.timestamp = hrt_absolute_time();

			//PX4_INFO("Rudder: %d", (int)(act.control[2]*1000.0f));
			// Write to actuators
			orb_publish(ORB_ID(actuator_controls_0), act_pub, &act);

		}

 
		
		//parameters_update(parameter_update_sub);
	}

	orb_unsubscribe(vehicle_attitude_sub_fd);
	orb_unsubscribe(parameter_update_sub);
	orb_unsubscribe(manual_sp_sub);
}

void Sailing::parameters_update(int parameter_update_sub, bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(parameter_update_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
	}

	if (force || updated) {
		updateParams();
	}
}


int sailing_main(int argc, char *argv[])
{
	return Sailing::main(argc, argv);
}




