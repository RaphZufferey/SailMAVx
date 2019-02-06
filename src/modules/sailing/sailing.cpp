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

/**
 * @file uuv_example_app.cpp
 *
 * This file let the hippocampus drive in a circle and prints the orientation as well as the acceleration data.
 * The HippoCampus is an autonomous underwater vehicle (AUV) designed by the Technical University Hamburg-Harburg (TUHH).
 * https://www.tuhh.de/mum/forschung/forschungsgebiete-und-projekte/flow-field-estimation-with-a-swarm-of-auvs.html
 *
 * @author Nils Rottann <Nils.Rottmann@tuhh.de>
 */
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

 //px4_module_params.h to get the DEFINE_PARAMETERS macro
#include <px4_module_params.h>

//extern "C" __EXPORT int sailing_main(int argc, char *argv[]);

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
	// Example: run the loop synchronized to the sensor_combined topic publication
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = sensor_combined_sub;
	fds[0].events = POLLIN;

	// initialize parameters
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	parameters_update(parameter_update_sub, true);

	while (!should_exit()) {

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

			struct sensor_combined_s sensor_combined;
			orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);
			// TODO: do something with the data...

		}


		parameters_update(parameter_update_sub);
	}

	orb_unsubscribe(sensor_combined_sub);
	orb_unsubscribe(parameter_update_sub);
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



// int sailing_main(int argc, char *argv[])
// {
// 	PX4_INFO("Sailing has started !");
// 	bool _task_should_exit = FALSE;


// 	// DEFINE_PARAMETERS(
// 	// 	(ParamFloat<px4::params::TRUE_WIND_ANGLE>) _true_wind_angle
// 	// )

// 	bool updated = false;
// 	float twindangle = 0;
// 	param_t param_twindangle = param_find("TRUE_WIND_ANGLE");
// 	param_get(param_twindangle, &twindangle);
// 	struct parameter_update_s update;
// 	memset(&update, 0, sizeof(update));
// 	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
// 	// Get parameter updates

// 	orb_check(parameter_update_sub, &updated);
// 	if (updated) {
// 		// copy global position
// 		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);

// 		// update all parameters
// 		param_get(param_twindangle, &twindangle);
// 	}


// 	/* subscribe to sensor_combined topic */
// 	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
// 	/* limit the update rate to 5 Hz */
// 	orb_set_interval(sensor_sub_fd, 200);

// 	/* subscribe to control_state topic */
// 	int vehicle_attitude_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
// 	/* limit the update rate to 5 Hz */
// 	orb_set_interval(vehicle_attitude_sub_fd, 200);

// 	/* advertise to actuator_control topic */
// 	struct actuator_controls_s act;
// 	memset(&act, 0, sizeof(act));
// 	orb_advert_t act_pub = orb_advertise(ORB_ID(actuator_controls_0), &act);

// 	/* one could wait for multiple topics with this technique, just using one here */
// 	px4_pollfd_struct_t fds[2] = {};
// 	fds[0].fd = sensor_sub_fd;
// 	fds[0].events = POLLIN;
// 	fds[1].fd = vehicle_attitude_sub_fd;
// 	fds[1].events = POLLIN;

// 	int error_counter = 0;
// 	int i = 0;

// 	while (!_task_should_exit) {
// 		if(++i > 10) _task_should_exit = TRUE;
// 		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
// 		int poll_ret = px4_poll(fds, 1, 1000);

// 		/* handle the poll result */
// 		if (poll_ret == 0) {
// 			/* this means none of our providers is giving us data */
// 			PX4_ERR("Got no data within a second");

// 		} else if (poll_ret < 0) {
// 			/* this is seriously bad - should be an emergency */
// 			if (error_counter < 10 || error_counter % 50 == 0) {
// 				/* use a counter to prevent flooding (and slowing us down) */
// 				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
// 			}

// 			error_counter++;

// 		} else {

// 			if (fds[0].revents & POLLIN) {
// 				/* obtained data for the first file descriptor */
// 				struct sensor_combined_s raw_sensor;
// 				/* copy sensors raw data into local buffer */
// 				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw_sensor);
// 				// printing the sensor data into the terminal
// 				PX4_INFO("Acc:\t%8.4f\t%8.4f\t%8.4f",
// 					 (double)raw_sensor.accelerometer_m_s2[0],
// 					 (double)raw_sensor.accelerometer_m_s2[1],
// 					 (double)raw_sensor.accelerometer_m_s2[2]);

// 				/* obtained data for the third file descriptor */
// 				struct vehicle_attitude_s raw_ctrl_state;
// 				/* copy sensors raw data into local buffer */
// 				orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub_fd, &raw_ctrl_state);

// 				// get current rotation matrix from control state quaternions, the quaternions are generated by the
// 				// attitude_estimator_q application using the sensor data
// 				matrix::Quatf q_att(raw_ctrl_state.q);     // control_state is frequently updated
// 				matrix::Dcmf R = q_att; // create rotation matrix for the quaternion when post multiplying with a column vector

// 				// orientation vectors
// 				matrix::Vector3f x_B(R(0, 0), R(1, 0), R(2, 0));     // orientation body x-axis (in world coordinates)
// 				matrix::Vector3f y_B(R(0, 1), R(1, 1), R(2, 1));     // orientation body y-axis (in world coordinates)
// 				matrix::Vector3f z_B(R(0, 2), R(1, 2), R(2, 2));     // orientation body z-axis (in world coordinates)

// 				PX4_INFO("x_B:\t%8.4f\t%8.4f\t%8.4f",
// 					 (double)x_B(0),
// 					 (double)x_B(1),
// 					 (double)x_B(2));

// 				PX4_INFO("y_B:\t%8.4f\t%8.4f\t%8.4f",
// 					 (double)y_B(0),
// 					 (double)y_B(1),
// 					 (double)y_B(2));

// 				PX4_INFO("z_B:\t%8.4f\t%8.4f\t%8.4f \n",
// 					 (double)z_B(0),
// 					 (double)z_B(1),
// 					 (double)z_B(2));
// 			}
// 		}


// 		//float yaw_att = matrix::Eulerf(matrix::Quatf(q_att->q)).psi();


// 		// Control 
// 		act.control[0] = 0.0f;      // roll
// 		act.control[1] = 0.0f;      // pitch
// 		act.control[2] = 1.0f;		// yaw
// 		act.control[3] = 1.0f;		// thrust


// 		// Write to actuators
// 		orb_publish(ORB_ID(actuator_controls_0), act_pub, &act);

// 	}


// 	PX4_INFO("Exiting sailing!");
// 	fflush(stdout);

// 	return 0;
// }


