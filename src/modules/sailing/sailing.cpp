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
#include <lib/ecl/geo/geo.h>


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
	//memset(&act, 100, sizeof(act));
	memset(&param_upd, 0, sizeof(param_upd));
	memset(&raw_att, 0, sizeof(raw_att));
	memset(&_outputs, 100, sizeof(_outputs));
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


	/* check if vehicle global position has changed*/
	orb_check(vehicle_gps_position_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_position_sub, &vehicle_gps_position);
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
		//act.control[actuator_controls_s::INDEX_LANDING_GEAR] = ACT_CTRL_SAIL_UP;
		//orb_publish(ORB_ID(actuator_controls_0), act_pub, &act);
		px4_sleep(3);
	}
	else if (direction == DOWNWARDS) {
		PX4_INFO("Folding sails down, 3 sec");
		//act.control[actuator_controls_s::INDEX_LANDING_GEAR] = ACT_CTRL_SAIL_DOWN;
		//orb_publish(ORB_ID(actuator_controls_0), act_pub, &act);
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
	vehicle_gps_position_sub = 	orb_subscribe(ORB_ID(vehicle_gps_position));

	//vehicle odometry subscription
	//vehicle_odometry_sub = 		orb_subscribe(ORB_ID(vehicle_odometry));

	vehicle_control_mode_pub = 	orb_advertise(ORB_ID(vehicle_control_mode), &vehicle_control_mode);
	act_pub = 			orb_advertise(ORB_ID(actuator_controls_0), &act);/* advertise to actuator_control topic */

	//_outputs_pub = 			orb_advertise(ORB_ID(actuator_outputs), &_outputs); /* advertise to actuator_control topic */

	if(_outputs_pub != nullptr){
		PX4_WARN("act_pub is null");
	}

	//if(act_pub != nullptr){
	//	PX4_WARN("act_pub is null");
	//}

	// Options
	orb_set_interval(vehicle_attitude_sub, 100); //200 /* limit the update rate to X ms */

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
	{.fd = vehicle_attitude_sub, .events = POLLIN},
	};

	double max_rudder_angle = M_PI/4; // hypothesis: 45 degree max rudder angle
	double rudder;
	float cmd_rudder_angle;
	float lon;
	float lat;
	int number_points = 4; //CHECKKKKKKKKKKK

	int SAILING_FREEZE = 400;
	int STOP_STRATEGY = 500;
	int WAYPOINT_QGC = 700;
	float lat_next_point = 43.92817837;
	float lon_next_point = 15.51208952;
	float lat_second = 43.93602850;
	float lon_second = 15.52084425;
	float lat_third = 43.93602850;
	float lon_third = 15.54642179;
	float lat_fourth = 43.91074367;
	float lon_fourth = 15.57388761;
	float lat_fifth = 43.88984003;
	float lon_fifth = 15.56762197;

	int stop_index = 0;
	int wind_passed = 0;
	int tacking = 0;

	int i = 0;
	int head_sign = -1;
	float tolerance_radius = 100.0;
	float cmd_sail_throttle = 0.0; // from 0 to 1

	float heading_setpoint = 0.0;
	float heading_setpoint_print = 0.0;
	float distance_waypoint = 0.0;
	float heading_initial = 0.0;
	float heading_wind = -130.0*M_PI/180;
	float heading_tolerance = 90.0*M_PI/180;
	float wnd_contribution = 20*M_PI/180;
	//int tacking;
	float upper_limit = heading_initial + heading_tolerance < 2*heading_tolerance ? heading_initial + heading_tolerance : heading_initial - 3*heading_tolerance;
	float lower_limit = heading_initial - heading_tolerance > -2*heading_tolerance ? heading_initial - heading_tolerance : heading_initial + 3*heading_tolerance;
	float wind_tolerance = 55*M_PI/180;
	float wind_triangle = 40*M_PI/180;
	//float drift_compensation = 15*M_PI/180;
	float drift_factor = 0.0;
	float upper_wind = heading_wind + wind_triangle < 2*heading_tolerance ? heading_wind + wind_triangle : heading_wind + wind_triangle - 4*heading_tolerance;
	float lower_wind = heading_wind - wind_triangle > -2*heading_tolerance ? heading_wind - wind_triangle : heading_wind - wind_triangle + 4*heading_tolerance;
	float cmd_sail_angle_left;
	float cmd_sail_angle_right;

	act.control[actuator_controls_s::INDEX_PITCH] = 0;

	bool updated = false;
	param_t param_heading_set = param_find("HEADING_SET");
	param_get(param_heading_set, &heading_set);
	param_t param_heading_lat = param_find("HEADING_LAT");
	param_get(param_heading_lat, &heading_latitude);
	param_t param_heading_lon = param_find("HEADING_LON");
	param_get(param_heading_lon, &heading_longitude);

	// Get parameter updatesheading_set
	parameters_update(true);

	//int i = 0;
	PX4_INFO("sail controller loop starting");

	orb_check(sensor_wind_angle_sub, &updated); // instead of odometry_sub we can use vehicle_gps_position
	if(updated){/* copy sensors raw data into local buffer */
		orb_copy(ORB_ID(sensor_wind_angle), sensor_wind_angle_sub, &sensor_wind_angle);
	}

	float scale_wind_angle = sensor_wind_angle.wind_magnetic_angle;

	orb_check(vehicle_gps_position_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_position_sub, &vehicle_gps_position);
	}

	lon = (double)((vehicle_gps_position.lon) * 1e-7d);
	lat = (double)((vehicle_gps_position.lat) * 1e-7d);

	heading_initial = (double)get_bearing_to_next_waypoint_sailing((double)lat, (double)lon, lat_next_point, (double)lon_next_point);

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

			//PX4_INFO("sail controller: polling vehicle_attitude_sub");

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

				double wnd_to_boat = (sensor_wind_angle.wind_magnetic_angle - scale_wind_angle)*M_PI/180; // from sensor_wind_angle.msg
				//float wnd_to_boat = heading_wind + current_yaw;
				wnd_to_boat = wnd_to_boat < M_PI ? wnd_to_boat : wnd_to_boat - 2*M_PI;
				wnd_to_boat = wnd_to_boat > - M_PI ? wnd_to_boat : wnd_to_boat + 2*M_PI;

				/* To have -180 to 180, compliant with the controller
				if (wnd_to_boat > M_PI){
					wnd_to_boat = wnd_to_boat -2*M_PI;
				}
				else if (wnd_to_boat < -M_PI){
					wnd_to_boat = wnd_to_boat +2*M_PI;
				}*/

				float cmd_sail_angle = sgn(wnd_to_boat)*M_PI/4*(-cos(wnd_to_boat)+1)/sail_angle_max; // from -PI/2 to PI/2
				cmd_sail_angle = sgn(cmd_sail_angle)*cmd_sail_angle <= 1 ? cmd_sail_angle : sgn(cmd_sail_angle); // saturation of the sail
				cmd_sail_angle_left = cmd_sail_angle; // from -1 to 1
				cmd_sail_angle_right = cmd_sail_angle; // from -1 to 1

				//if (wnd_to_boat > 8.0f*M_PI/180 && wnd_to_boat < 69.0f*M_PI/180)
				//param_get(param_sail_strategy, &sail_strategy); //applying another control strategy sqrt(1-cos)
				px4_usleep(5000);
				//if (sail_strategy == 1){
					if (wnd_to_boat > 8.0*M_PI/180 && wnd_to_boat < 69.0*M_PI/180){
						cmd_sail_angle = 0.9775*wnd_to_boat - 0.133; // Andre's strategy
						cmd_sail_angle_right = cmd_sail_angle/sail_angle_max;
						//PX4_INFO("wind 2 %f", cmd_sail_angle);
					}
					if (wnd_to_boat < -8.0*M_PI/180 && wnd_to_boat > -69.0*M_PI/180){
						cmd_sail_angle = 0.9775*wnd_to_boat + 0.133; // Andre's strategy
						cmd_sail_angle_left = cmd_sail_angle/sail_angle_max;
						//PX4_INFO("wind 2 %f", cmd_sail_angle);
					}
				//}


				// rudder, reference for the rudder control: Position keeping control of an autonomous sailboat Par 3.1
				float current_yaw =  matrix::Eulerf(matrix::Quatf(raw_att.q)).psi(); //psi corresponds to Yaw (heading)

				param_get(param_heading_set, &heading_set);


				lon = (double)((vehicle_gps_position.lon) * 1e-7d);
				lat = (double)((vehicle_gps_position.lat) * 1e-7d);

				// something more reliable can be: get to know the wind angle to N, follow 45 degree to it fixing heading angles and check the tolerance adjusting this heading angle
				px4_usleep(50000);
				if (heading_set == WAYPOINT_QGC){ // new waypoint communicated from QGC
					param_get(param_heading_longitude, &heading_longitude);
					param_get(param_heading_latitude, &heading_latitude);
					lon_next_point = (double)heading_longitude;
					lat_next_point = (double)heading_latitude;
				}

				distance_waypoint = (double)get_distance_to_next_waypoint_sailing((double)lat, (double)lon, lat_next_point, lon_next_point); // with 0 works

				if (distance_waypoint > tolerance_radius){
					heading_setpoint = (double)get_bearing_to_next_waypoint_sailing((double)lat, (double)lon, lat_next_point, (double)lon_next_point);
					heading_setpoint_print = heading_setpoint;
					px4_usleep(2000);
					//TACKING, online checkings
					if (tacking == 1){ //if at the beginning the next waypoint's heading is close to the wind speed
						if (heading_setpoint < heading_wind + 5*M_PI/180 && heading_setpoint > heading_wind - 5*M_PI/180){ // around the heading wind,
						wind_passed = 1;
					}
					// this is way to understand if the the robot is going to the right direction
					//if ((heading_setpoint - heading_wind)*sgn(heading_setpoint - heading_wind) < 50*M_PI/180 || (heading_setpoint - heading_wind)*sgn(heading_setpoint - heading_wind) > 310*M_PI/180){
					if (heading_setpoint > upper_wind && wind_passed == 1){ //if SailMAV overcame the upper wind triangle threshold
						wind_passed = 0;
						head_sign = 1; //sign(heading_wind)
					}
					else if (heading_setpoint < lower_wind && wind_passed == 1){ //if SailMAV overcame the lower wind triangle threshold
						wind_passed = 0; //if we passed the line to the wind. It is a way to not make the code write all the time
						head_sign = -1;
					}
					heading_setpoint = heading_wind + head_sign*45*M_PI/180;
					}

					if (heading_setpoint > upper_limit || heading_setpoint < lower_limit && (i < number_points - 1)){
						i++;
						if (i == 1){
							lat_next_point = lat_second;
							lon_next_point = lon_second;
						}
						if (i == 2){
							lat_next_point = lat_third;
							lon_next_point = lon_third;
						}
						if (i == 3){
							lat_next_point = lat_fourth;
							lon_next_point = lon_fourth;
						}
						if (i == 4){
							lat_next_point = lat_fifth;
							lon_next_point = lon_fifth;
						}							}
						heading_initial = (double)get_bearing_to_next_waypoint_sailing((double)lat, (double)lon, lat_next_point, (double)lon_next_point);

						}

					}
				}
				else if (i < number_points - 1){
					//if (distance_waypoint < tolerance_radius && i < number_points - 1){
					i++;
					tacking = 0;
					px4_usleep(2000);
					if (i == 1){
						lat_next_point = lat_second;
						lon_next_point = lon_second;
					}						}
					if (i == 2){
						lat_next_point = lat_third;
						lon_next_point = lon_third;
						}
					if (i == 3){
						lat_next_point = lat_fourth;
						lon_next_point = lon_fourth;
						}
					if (i == 4){
						lat_next_point = lat_fifth;
						lon_next_point = lon_fifth;
					}
					heading_initial = (double)get_bearing_to_next_waypoint_sailing((double)lat, (double)lon, lat_next_point, (double)lon_next_point);
					//TACKING, offline checkings
					if ((heading_initial - heading_wind)*sgn(heading_initial - heading_wind) < wind_tolerance || (heading_initial - heading_wind)*sgn(heading_initial - heading_wind) > 4*heading_tolerance - wind_tolerance){ // decide if it is needed to tack
						tacking = 1;
					}
					if (tacking == 1 && sgn(heading_initial - heading_wind)){ // to decide first heading
						head_sign = 1;
					}
					else if (tacking == 1 && !sgn(heading_initial - heading_wind)) {
						head_sign = -1;
					}
				}

				//to check in case this value exceeds -180 to 180
				heading_setpoint = heading_setpoint < M_PI ? heading_setpoint : heading_setpoint - M_PI*2;
				heading_setpoint = heading_setpoint > -M_PI ? heading_setpoint : heading_setpoint + M_PI*2;

				float error_heading = heading_setpoint - current_yaw; // /epsilon_{theta}
				px4_usleep(20000);

				// construct to apply the rudder control
				if(cos(error_heading) >= 0){
					rudder = max_rudder_angle*sin(error_heading);
				}
				else{
					rudder = max_rudder_angle*sgn(error_heading);
				}

				cmd_rudder_angle = rudder/max_rudder_angle; // It goes from -1 to 1

				// give power to the throttle
				if (manual_sp.z < 0.1f){ // manual_sp.z is thrust setpoint
					cmd_sail_throttle = 0;
				} else {
					cmd_sail_throttle = min(manual_sp.z, 1);
				}


				//if (stop_strategy != 1){ // if stop strategy is not active
				if (heading_set != STOP_STRATEGY){
					//param_get(param_sailing_freeze, &sailing_freeze);
					if (heading_set != SAILING_FREEZE){ // if sailing are not freezed
						act.control[actuator_controls_s::INDEX_ROLL] = cmd_sail_angle_left;   // roll = SAILS
						act.control[actuator_controls_s::INDEX_FLAPS] = cmd_sail_angle_right;
					}

					act.control[actuator_controls_s::INDEX_YAW] = cmd_rudder_angle;  // yaw = RUDDER
					act.control[actuator_controls_s::INDEX_THROTTLE] = cmd_sail_throttle;	 // thrust

					act.control[5] = wnd_to_boat; //made just to save this values in the log of px4
					act.control[6] = distance_waypoint;
					act.control[7] = heading_setpoint;
					stop_index = 0;
					PX4_INFO("n.point %f wnd_ang %f yaw %f head_p %f rud %f head_set %f dist %f sail %f", (double)i, (double)wnd_to_boat*180/M_PI, (double)current_yaw*180/M_PI, (double)heading_setpoint_print*180/M_PI, (double)cmd_rudder_angle, (double)heading_setpoint*180/M_PI, (double)distance_waypoint, (double)cmd_sail_angle_right); // (double)Theta);

				}
				//}
				else if (stop_index != 1){ //put everything to 0
					stop_index = 1;
					act.control[actuator_controls_s::INDEX_YAW] = 0;  // yaw = RUDDER
					act.control[actuator_controls_s::INDEX_THROTTLE] = 0;	 // thrust
					act.control[actuator_controls_s::INDEX_ROLL] = 0;   // left SAIL
					act.control[actuator_controls_s::INDEX_FLAPS] = 0;  // RIGHT SAIL
					//act.control[actuator_controls_s::INDEX_PITCH] = 0; // not control
				}
				act.timestamp = hrt_absolute_time();
				//PX4_INFO("Rudder: %d", (int)(act.control[actuator_controls_s::INDEX_YAW]*100.0f));
				// Write to actuators
				orb_publish(ORB_ID(actuator_controls_0), act_pub, &act);
				px4_usleep(20000);
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
	orb_unsubscribe(vehicle_gps_position_sub);
}

int sailing_main(int argc, char *argv[])
{
	return Sailing::main(argc, argv);
}
