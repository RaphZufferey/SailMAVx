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
//push

//px4_module_params.h to get the DEFINE_PARAMETERS macro
#include <px4_module_params.h>
#include "params.c"

#define DOWNWARDS 0
#define UPWARDS 1

float myPi = (float)M_PI;

template <typename T> int sgn(T val) { 
    return (T(0) < val) - (val < T(0));
}

template <typename T> float wrapToPi(T val) {
    val = fmod(val + myPi,2*myPi);
    if (val < 0) val += 2*myPi;
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

int Sailing::print_status(){
	PX4_INFO("Running");
	return 0;
}

int Sailing::custom_command(int argc, char *argv[]){
	return print_usage("unknown command");
}


int Sailing::task_spawn(int argc, char *argv[]){
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
	memset(&manual_sp, 0, sizeof(manual_sp));	
	memset(&act, 0, sizeof(act));
	memset(&param_upd, 0, sizeof(param_upd));
	memset(&raw_att, 0, sizeof(raw_att));
}

void Sailing::vehicle_poll()
{
    bool updated;
    /* check if vehicle control mode has changed */
    orb_check(vehicle_control_mode_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_control_mode), vehicle_control_mode_sub, &vehicle_control_mode);
    }

    orb_check(vehicle_status_sub, &updated);
	if (updated) {
	    orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status);
	}
}

void Sailing::parameters_update(bool force)
{
	bool updated;
	orb_check(param_update_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(parameter_update), param_update_sub, &param_upd);
	}
	if (force || updated) {
		updateParams();
		//param_get(param_wnd_angle_to_n, &wnd_angle_to_n);
		//PX4_INFO("Wind angle parameter updated to : %d", wnd_angle_to_n);
	}
}

void Sailing::fold_sails(int direction){
	if (direction == UPWARDS) {
		/* code */
	}
	else if (direction == DOWNWARDS) {

	}
}

void Sailing::run()
{
	px4_usleep(50000);
	PX4_INFO("sailing running");

	// Subscriptions and publications
	parameter_update_sub = 		orb_subscribe(ORB_ID(parameter_update));
	vehicle_attitude_sub = 		orb_subscribe(ORB_ID(vehicle_attitude));// 	/* subscribe to control_state topic */ 
	manual_sp_sub = 			orb_subscribe(ORB_ID(manual_control_setpoint));// subscribe to manual control setpoint
 	vehicle_control_mode_sub = 	orb_subscribe(ORB_ID(vehicle_control_mode));// subscribe and advertise to vehicle control mode
	vehicle_status_sub = 		orb_subscribe(ORB_ID(vehicle_status));


	vehicle_control_mode_pub = 	orb_advertise(ORB_ID(vehicle_control_mode), &vehicle_control_mode);
	act_pub = 					orb_advertise(ORB_ID(actuator_controls_0), &act);/* advertise to actuator_control topic */

	// Options
	orb_set_interval(vehicle_attitude_sub, 100); //200 /* limit the update rate to X ms */
	
	vehicle_poll(); // checks for navigation state changes and flags changes to exit this loop


	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = vehicle_attitude_sub;
	fds[0].events = POLLIN;

	
	bool updated = false;
	param_t param_wnd_angle_to_n = param_find("WND_ANGLE_TO_N");
	param_get(param_wnd_angle_to_n, &wnd_angle_to_n);
		
	// Get parameter updates
	parameters_update(true);

	//int i = 0;
	PX4_INFO("sail controller loop starting");

	while (!should_exit()) {
		// IS THIS CONDITION IN THE RIGHT PLACE?? 
		vehicle_poll();

		// Not checking for flags at this point, doesnt seem to be required
		while((vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_SAIL))
				// && (vehicle_control_mode.flag_control_sail_enabled))
			{

			vehicle_poll(); // checks for navigation state changes and flags changes to exit this loop

			/* CODE THAT RUNS ONCE WHEN WE ENTER THIS LOOP. SHOULD FOLD THE SAIL UP */
			if (sails_are_down){
				PX4_INFO("Folding sails up");
				fold_sails(UPWARDS);
				//act.control[?] = UP;
				//orb_publish(ORB_ID(actuator_controls_0), act_pub, &act);
				px4_usleep(1000000);
				sails_are_down = false;
			}

			// wait for up to 1000ms for data
			int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);
			if (pret == 0) {
				// Timeout: let the loop run anyway, don't do `continue` here
			} else if (pret < 0) {
				// this is undesirable but not much we can do
				PX4_ERR("poll error %d, %d", pret, errno);
				px4_usleep(50000);
				continue;

			} else if (fds[0].revents & POLLIN) { // SLOW RUNNING LOOP
			
				orb_copy(ORB_ID(manual_control_setpoint), manual_sp_sub, &manual_sp);
				//PX4_INFO("manual sp: %f ", (double)(manual_sp.r)*100); //,(int)(manual_sp.x*100.0f),(int)(manual_sp.y*100.0f));

				orb_check(vehicle_attitude_sub, &updated);
				if(updated){/* copy sensors raw data into local buffer */
					orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &raw_att);
				}

				float current_yaw =  matrix::Eulerf(matrix::Quatf(raw_att.q)).psi();
				float wnd_angle_to_n_rad = wnd_angle_to_n*myPi/180;
				float wnd_to_boat = wrapToPi(wnd_angle_to_n_rad - current_yaw);
				float sail_angle = -sgn(wnd_to_boat)*M_PI/4*(cos(wnd_to_boat)+1);
				float cmd_sail_angle = sail_angle/sail_angle_max;
				// PX4_INFO("Yaw  \t%d, actuators: \t%d", (int)((current_yaw*180.0f/myPi)), (int)((cmd_sail_angle*180.0f/myPi)));

		 		//Control 
				act.control[actuator_controls_s::INDEX_ROLL] = cmd_sail_angle;   // roll = SAILS
				//act.control[actuator_controls_s::INDEX_PITCH] DEACTIVATED 
				act.control[actuator_controls_s::INDEX_YAW] = manual_sp.r;	 // yaw = RUDDER (in SailMAV : servo line 5)
				// act.control[actuator_controls_s::INDEX_THROTTLE] = manual_sp.z;	 // thrust
				//act.timestamp = hrt_absolute_time();
				//PX4_INFO("Rudder: %d", (int)(act.control[2]*100.0f));
				// Write to actuators
				orb_publish(ORB_ID(actuator_controls_0), act_pub, &act);
			}
		}

		if(!sails_are_down){
			//bring sails down
			fold_sails(DOWNWARDS);
			sails_are_down = true;

		}
		// if we are not in the right mode, go to sleep
		px4_usleep(50000);		
	}
	orb_unsubscribe(vehicle_attitude_sub);
	orb_unsubscribe(parameter_update_sub);
	orb_unsubscribe(manual_sp_sub);
	orb_unsubscribe(vehicle_control_mode_sub);
	orb_unsubscribe(vehicle_status_sub);
}

int sailing_main(int argc, char *argv[])
{
	return Sailing::main(argc, argv);
}




