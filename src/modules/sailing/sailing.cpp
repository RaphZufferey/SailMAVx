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


#define DOWNWARDS (0)
#define UPWARDS (1)
#define ACT_CTRL_SAIL_UP (1)
#define ACT_CTRL_SAIL_DOWN (-1)

#define ACT_CTRL_PITCH_TRIM (0)

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
	memset(&act, 100, sizeof(act));
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

	/* check if vehicle status has changed */
	orb_check(vehicle_status_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status);
	}

	/* check if vehicle odometry has changed */
	orb_check(vehicle_odometry_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_odometry), vehicle_odometry_sub, &vehicle_odometry);
	}

	/* check if vehicle global position has changed */
	orb_check(vehicle_global_position_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_global_position), vehicle_global_position_sub, &vehicle_global_position);
	}

	/* check if vehicle status has changed */
	orb_check(sensor_wind_angle_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(sensor_wind_angle), sensor_wind_angle_sub, &sensor_wind_angle);
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
		PX4_INFO("Folding sails up, 3 sec");
		act.control[actuator_controls_s::INDEX_LANDING_GEAR] = ACT_CTRL_SAIL_UP;
		orb_publish(ORB_ID(actuator_controls_0), act_pub, &act);
		px4_sleep(3);
	}
	else if (direction == DOWNWARDS) {
		PX4_INFO("Folding sails down, 3 sec");
		act.control[actuator_controls_s::INDEX_LANDING_GEAR] = ACT_CTRL_SAIL_DOWN;
		orb_publish(ORB_ID(actuator_controls_0), act_pub, &act);
		px4_sleep(3);
	}
}

void Sailing::run()
{
	px4_usleep(50000);
	PX4_INFO("sailing running");

	// Subscriptions and publications
	parameter_update_sub = 		orb_subscribe(ORB_ID(parameter_update));
	vehicle_attitude_sub = 		orb_subscribe(ORB_ID(vehicle_attitude));// 	/* subscribe to control_state topic */
	manual_sp_sub = 		orb_subscribe(ORB_ID(manual_control_setpoint));// subscribe to manual control setpoint
 	vehicle_control_mode_sub = 	orb_subscribe(ORB_ID(vehicle_control_mode));// subscribe and advertise to vehicle control mode
	vehicle_status_sub = 		orb_subscribe(ORB_ID(vehicle_status));
	sensor_wind_angle_sub = 	orb_subscribe(ORB_ID(sensor_wind_angle));
	vehicle_global_position_sub = 	orb_subscribe(ORB_ID(vehicle_global_position));



	//vehicle odometry subscription
	vehicle_odometry_sub = 		orb_subscribe(ORB_ID(vehicle_odometry));

	vehicle_control_mode_pub = 	orb_advertise(ORB_ID(vehicle_control_mode), &vehicle_control_mode);
	act_pub = 			orb_advertise(ORB_ID(actuator_controls_0), &act);/* advertise to actuator_control topic */


	if(act_pub != nullptr){
		PX4_WARN("act_pub is null");
	}

	// Options
	orb_set_interval(vehicle_attitude_sub, 100); //200 /* limit the update rate to X ms */

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
	{.fd = vehicle_attitude_sub, .events = POLLIN},
	};

	float min_wnd = 40* M_PI/180; // This is the tolerance angle between boat and wind. It should represent the minumum orientation in which the boat can not be push by the wind anymore
	double max_rudder_angle = M_PI/4; // hypothesis: 45 degree max rudder angle
	//float Theta; //This is the (upper)Theta angle in the reference
	double rudder;
	double rudder_PI;
	float course_angle;
	float cmd_rudder_angle;

	float tmp_max = 1;
	float tmp = tmp_max;
	float wind_angle_actual = 0;
	float last_wind_angle = 0;
	float cmd_sail_throttle = 0; // from 0 to 1

	float Kp = .1; // PI parameters
	float Ki = .05; // PI parameters
	float P_error;
	float I_error;

	bool updated = false;
	//param_t param_wnd_angle_to_n = param_find("WND_ANGLE_TO_N");
	//param_get(param_wnd_angle_to_n, &wnd_angle_to_n);
	param_t param_heading_strategy = param_find("HEADING_STRATEGY");
	param_get(param_heading_strategy, &heading_strategy);
	//param_t param_heading_latitude = param_find("HEADING_LATITUDE");
	//param_get(param_heading_latitude, &heading_latitude);
	//param_t param_heading_longitude = param_find("HEADING_LONGITUDE");
	//param_get(param_heading_longitude, &heading_longitude);
	param_t param_heading_set = param_find("HEADING_SET");
	param_get(param_heading_set, &heading_set);
	param_t param_wind_strategy = param_find("WIND_STRATEGY");
	param_get(param_wind_strategy, &wind_strategy);
	param_t param_rudder_strategy = param_find("RUDDER_STRATEGY");
	param_get(param_rudder_strategy, &rudder_strategy);

	// Get parameter updates
	parameters_update(true);

	//int i = 0;
	PX4_INFO("sail controller loop starting");

	orb_check(sensor_wind_angle_sub, &updated); // instead of odometry_sub we can use vehicle_gps_position
	if(updated){/* copy sensors raw data into local buffer */
		orb_copy(ORB_ID(sensor_wind_angle), sensor_wind_angle_sub, &sensor_wind_angle);
		}

	float scale_wind_angle = sensor_wind_angle.wind_magnetic_angle;

	while (!should_exit()) {

		vehicle_poll(); // checks for navigation state changes and flags changes to exit this loop
		PX4_INFO("sail controller: vehicle_status.nav_state %d, wind_angle %f", vehicle_status.nav_state, sensor_wind_angle.wind_magnetic_angle);

		// Not checking for flags at this point, doesnt seem to be required
		if((vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_SAIL)
				&& (vehicle_control_mode.flag_control_sail_enabled)) //REMEMBER CHANGE
		//if((vehicle_status.nav_state == 0))
		{
			/* CODE THAT RUNS ONCE WHEN WE ENTER THIS LOOP. SHOULD FOLD THE SAIL UP */
			/* This piece just fold the sails up without checking*/
			if (sails_are_down){
				fold_sails(UPWARDS); // 3 sec blocking
				sails_are_down = false;
			}

			PX4_INFO("sail controller: polling vehicle_attitude_sub");

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
			}

				//float wnd_angle_to_n_rad = wnd_angle_to_n*myPi/180.0f;
				//float wnd_to_boat = wrapToPi(wnd_angle_to_n_rad - current_yaw)
				float cmd_sail_angle;

				float wnd_to_boat = (sensor_wind_angle.wind_magnetic_angle - scale_wind_angle)*M_PI/180; // from sensor_wind_angle.msg

				// To have -180 to 180, compliant with the controller
				if (wnd_to_boat > M_PI){
					wnd_to_boat = wnd_to_boat -2*M_PI;
				}
				else if (wnd_to_boat < -M_PI){
					wnd_to_boat = wnd_to_boat +2*M_PI;
				}
				float sail_angle = sgn(wnd_to_boat)*M_PI/6*(-cos(wnd_to_boat)+1);
				cmd_sail_angle = sail_angle/sail_angle_max; // from -1 to 1
				//PX4_INFO("wind 1 %f", cmd_sail_angle);

				param_get(param_wind_strategy, &wind_strategy); //applying another control strategy sqrt(1-cos)
				if (wind_strategy == 1){
					//if (sail_angle > M_PI/2 || sail_angle < -M_PI/2) { // only for downwind
						float sail_angle_sqrt = sqrt(sgn(sail_angle)*sail_angle);//*sqrt(2);
						cmd_sail_angle = sgn(sail_angle)*sail_angle_sqrt/sail_angle_max;
						//PX4_INFO("wind 2 %f", cmd_sail_angle);
					//}
				}

				//PX4_INFO("Yaw  \t%d, actuators: \t%d", (int)((current_yaw*180.0f/myPi)), (int)((cmd_sail_angle*180.0f/myPi)));

				// rudder, reference for the rudder control: Position keeping control of an autonomous sailboat Par 3.1
				float current_yaw =  matrix::Eulerf(matrix::Quatf(raw_att.q)).psi(); //psi corresponds to Yaw (heading)
				float wnd_angle_to_n = wnd_to_boat - M_PI + current_yaw; // just for the tester, to check the wind angle and decide the heading direction. This can be compared to the station one
				wnd_angle_to_n = wnd_angle_to_n > 0 ? wnd_angle_to_n : wnd_angle_to_n + 2*M_PI ; // normalize from 0 to 360

				float velocity_x = vehicle_odometry.vx;
				float velocity_y = vehicle_odometry.vy;

				// check tolerance for velocities
				if (velocity_x > 0.01 || velocity_x < -0.01 || velocity_y > 0.01 || velocity_y < -0.01){ // tolerance to check if SailMAV is moving
					course_angle = current_yaw + atan2(velocity_y , velocity_x); //actual angle of the boat trajectory  check
				}
				else{
					course_angle = current_yaw;
				}
				//PX4_INFO("course_angle %f", course_angle);

				float heading_setpoint;
				switch(heading_strategy)
				{
					case 1: // waypoints to reach known a priori
						heading_setpoint = (float)atan2(heading_latitude - vehicle_global_position.lat, heading_longitude - vehicle_global_position.lon); // online heading angle computation
						break;
					case 2: // waypoints to reach taken from PX4 parameter
						param_get(param_heading_longitude, &heading_longitude);
						param_get(param_heading_latitude, &heading_latitude);
						heading_setpoint = (float)atan2(heading_latitude - vehicle_global_position.lat, heading_longitude - vehicle_global_position.lon); // online heading angle computation
						break;
					default: // heading setpoint, just the angle
						param_get(param_heading_set, &heading_set);
						heading_setpoint = (float)heading_set*M_PI/180; // North (reference frame) setpoint in heading, tester/developer decision (put on top of the file?). 0 obviously means go straight
						// float error_heading = Theta - heading_setpoint; // /epsilon_{theta}

				}

				float error_heading = heading_setpoint - current_yaw; // /epsilon_{theta}

				// construct to decide the value of (upper)Theta to decide
				/*if (cos(current_yaw - course_angle) - cos(error_heading) >= 0){
					Theta = course_angle;
				}
				else{
					Theta = current_yaw;
				}*/

				// construct to apply the control
				if(cos(error_heading) >= 0){
					rudder = max_rudder_angle*sin(error_heading);
				}
				else{
					rudder = max_rudder_angle*sgn(error_heading);
				}

				cmd_rudder_angle = rudder/max_rudder_angle; // It goes from -1 to 1

				/*
				// check if the robot can make the operation (in the case of strong upwind)
				if (wnd_to_boat < min_wnd && wnd_to_boat > -min_wnd){
					sail_angle = 0;
				  	cmd_sail_angle = sail_angle/wnd_to_boat;
				}
				*/

				// give power to the throttle
				if (manual_sp.z < 0.1f){ // manual_sp.z is thrust setpoint
					cmd_sail_throttle = 0;
				} else {
					cmd_sail_throttle = min(manual_sp.z, 1);
				}

				//PI rudder for comparison

				P_error = Kp*sin(error_heading);
				I_error+= Ki*sin(error_heading);

				param_get(param_rudder_strategy, &rudder_strategy);
				if (rudder_strategy == 1){
					PX4_INFO("PI rudder");
					cmd_rudder_angle = (P_error + I_error < max_rudder_angle && (P_error + I_error) > - max_rudder_angle) ? P_error + I_error : sgn(error_heading); //rudder is computed by PI controller until it reaches the max threshold. The if-else limits to -1/1
				}

				PX4_INFO("cmd_sail: %f wnd_angle %f yaw %f course_ang %f  cmd_rudder %f head_set %f wind_to_n %f", (double)cmd_sail_angle, (double)wnd_to_boat*180/M_PI, (double)current_yaw*180/M_PI, (double)course_angle*180/M_PI, (double)cmd_rudder_angle, (double)heading_set, (double)wnd_angle_to_n*180/M_PI); // (double)Theta);

		 		//Control
				act.control[actuator_controls_s::INDEX_ROLL] = cmd_sail_angle;   // roll = SAILS
				//act.control[actuator_controls_s::INDEX_PITCH] = ACT_CTRL_PITCH_TRIM; // not control
				//act.control[actuator_controls_s::INDEX_YAW] = manual_sp.r;	 // yaw = RUDDER (in SailMAV : servo line 5)
				act.control[actuator_controls_s::INDEX_YAW] = cmd_rudder_angle;  // yaw = RUDDER
				act.control[actuator_controls_s::INDEX_THROTTLE] = cmd_sail_throttle;	 // thrust
				act.timestamp = hrt_absolute_time();
				//PX4_INFO("Rudder: %d", (int)(act.control[actuator_controls_s::INDEX_YAW]*100.0f));
				// Write to actuators
				orb_publish(ORB_ID(actuator_controls_0), act_pub, &act);
				px4_usleep(500);
			}

		else { // not in sailing
			if(!sails_are_down){
				fold_sails(DOWNWARDS); // 3 sec blocking
				sails_are_down = true;
			}
			px4_usleep(50000);
		}


	}
	orb_unsubscribe(vehicle_attitude_sub);
	orb_unsubscribe(parameter_update_sub);
	orb_unsubscribe(manual_sp_sub);
	orb_unsubscribe(vehicle_control_mode_sub);
	orb_unsubscribe(vehicle_status_sub);
	orb_unsubscribe(sensor_wind_angle_sub);
}

int sailing_main(int argc, char *argv[])
{
	return Sailing::main(argc, argv);
}
