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
	vehicle_gps_position_sub = 	orb_subscribe(ORB_ID(vehicle_gps_position));



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
	//double rudder_PI;
	float course_angle;
	float cmd_rudder_angle;
	float lon;
	float lat;
	int number_points = 3;
	//float lat_next[number_points] = {515069853.0 * 1e-7d, 515055023.0 * 1e-7d, 515064603.0 * 1e-7d}; //51506297.0 * 1e-7d, 51505502.0 * 1e-7d, 51506460.0 * 1e-7d
	//float lon_next[number_points] = {-1843449.0 * 1e-7d, -1822099.0 * 1e-7d, -1821509.0 * 1e-7d}; //-184344.0 * 1e-7d, -182209.0 * 1e-7d, -182150.0 * 1e-7d
	//float lat_next_point = lat_next[0];
	//float lon_next_point = lon_next[0];
	float lat_next_point = 515062979.0 * 1e-7d;
	float lon_next_point = -1843443.0 * 1e-7d;
	float lat_second = 515055023.0 * 1e-7d;
	float lon_second = -1843449.0 * 1e-7d;
	float lat_third = 515064603.0 * 1e-7d;
	float lon_third = -1821509.0 * 1e-7d;
	//float lat_next = 518486985.0 * 1e-7d;
	//float lon_next = 1751629.0 * 1e-7d;

	int i = 0;
	//Waypoint lake; // create lake list
	//lake.latitude[0] = 51.8486985; // 514986985
	//lake.longitude[0] = .1751629; //-1751629
	//lake.number_points = 1;
	float tolerance_radius = 10;
	//int number_points = 1;

	//float tmp_max = 1;
	//float tmp = tmp_max;
	float wind_angle_actual = 0.0;
	float last_wind_angle = 0.0;
	float cmd_sail_throttle = 0.0; // from 0 to 1

	//float Kp = .1; // PI parameters
	//float Ki = .05; // PI parameters
	//float P_error;
	//float I_error;
	float heading_setpoint;
	float distance_waypoint;

	bool updated = false;
	//param_t param_wnd_angle_to_n = param_find("WND_ANGLE_TO_N");
	//param_get(param_wnd_angle_to_n, &wnd_angle_to_n);
	param_t param_heading_strategy = param_find("HEADING_STRATEGY");
	param_get(param_heading_strategy, &heading_strategy);
	param_t param_heading_latitude = param_find("HEADING_LAT");
	param_get(param_heading_latitude, &heading_latitude);
	param_t param_heading_longitude = param_find("HEADING_LONG");
	param_get(param_heading_longitude, &heading_longitude);
	param_t param_heading_set = param_find("HEADING_SET");
	param_get(param_heading_set, &heading_set);
	/*param_t param_wind_strategy = param_find("WIND_STRATEGY");
	param_get(param_wind_strategy, &wind_strategy);
	param_t param_rudder_strategy = param_find("RUDDER_STRATEGY");
	param_get(param_rudder_strategy, &rudder_strategy);*/

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

				//param_get(param_heading_longitude, &heading_longitude);

				/*param_get(param_wind_strategy, &wind_strategy); //applying another control strategy sqrt(1-cos)
				if (wind_strategy == 1){
					//if (sail_angle > M_PI/2 || sail_angle < -M_PI/2) { // only for downwind
						float sail_angle_sqrt = sqrt(sgn(sail_angle)*sail_angle);//*sqrt(2);
						cmd_sail_angle = sgn(sail_angle)*sail_angle_sqrt/sail_angle_max;
						//PX4_INFO("wind 2 %f", cmd_sail_angle);
					//}
				}*/

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
				PX4_INFO("course_angle %f",course_angle);

				param_get(param_heading_strategy, &heading_strategy); //applying another control strategy sqrt(1-cos)
				//float distance_waypoint = sqrtf((heading_longitude - _global_pos.lat) * (heading_longitude - _global_pos.lat) + (heading_longitude - _global_pos.lat) * (heading_longitude - _global_pos.lat));

				//PX4_INFO("latitude: %f longitude: %f", (double)vehicle_gps_position.lat, (double)vehicle_gps_position.lon); // (double)Theta);
				//PX4_INFO("%"PRIu32 , (float)latitude1); // (double)Theta);

				lon = (double)((vehicle_gps_position.lon) * 1e-7d);
				lat = (double)((vehicle_gps_position.lat) * 1e-7d);

				/*double lat_next_rad = math::radians(lat_next);

				double d_lon = math::radians(lon_next) - math::radians(lon);

				double cos_lat_next = cos(lat_next_rad);

				double d_lat = lat_next_rad - math::radians(lat);

				double a = sin(d_lat / 2.0) * sin(d_lat / 2.0) + sin(d_lon / 2.0) * sin(d_lon / 2.0) * cos(lat) * cos(lat_next_rad);

				double c = atan2(sqrt(a), sqrt(1.0 - a));

				float y = static_cast<float>(sin(d_lon) * cos_lat_next);
				float x = static_cast<float>(cos(lat) * sin(lat_next_rad) - sin(lat) * cos_lat_next * cos(d_lon));*/




				//const double lat_now_rad = math::radians(lat);
				//const double lat_next_rad = math::radians(lat_next);

				//lon = get_distance_to_next_waypoint_sailing((double)vehicle_gps_position.lat, (double)vehicle_gps_position.lon, (double) latitude1, (double)longitude1);

				//lat = get_bearing_to_next_waypoint_sailing((double)vehicle_gps_position.lat, (double)vehicle_gps_position.lon, (double) latitude1, (double)longitude1);
				//float heading_setpoint = (float)(atan2f(y, x));
				//float heading_setpoint= get_bearing_to_next_waypoint_sailing((double)lat, (double)lon, (double)lat_next, (double)lon_next);

				//get_distance_to_next_waypoint_sailing


				//PX4_INFO("course_angle %f", course_angle);

				//PX4_INFO("lat: %f lon: %f", wind_angle_actual, wind_angle_actual); // (double)Theta);


				//float heading_setpoint;
				/* if (heading_strategy == 1){
					float distance_waypoint = get_distance_to_next_waypoint_sailing(vehicle_gps_position.lat, vehicle_gps_position.lon, latitude1, longitude1);
					if (distance_waypoint > tolerance_radius){
						heading_setpoint = get_bearing_to_next_waypoint(vehicle_gps_position.lat, vehicle_gps_position.lon, latitude1, longitude1);
					}
					else if (i < lake.number_points - 1){
						i++;
					}
					//PX4_INFO("distance waypoint: %f heading direction: %f", (double)distance_waypoint, (double)heading_setpoint); // (double)Theta);
				}
				else {*/
					//param_get(param_heading_set, &heading_set);
					//heading_setpoint = (float64)heading_set*M_PI/180; // North (reference frame) setpoint in heading, tester/developer decision (put on top of the file?). 0 obviously means go straight
				//}
				/*switch(heading_strategy)
				{
					case 1: // waypoints to reach known a priori
						float distance_waypoint = get_distance_to_next_waypoint(vehicle_gps_position.lat, vehicle_gps_position.lon, lake.latitude[i], lake.longitude[i]);
						if (distance_waypoint > M_PI*tolerance_radius^2){
							heading_setpoint = get_bearing_to_next_waypoint(vehicle_gps_position.lat, vehicle_gps_position.lon, lake.latitude[i], lake.longitude[i]);
						}
						else if (i < lake.number_points - 1){
							i++;
						}
						PX4_INFO("distance waypoint: %f heading direction: %f", (double)distance_waypoint, (double)heading_setpoint); // (double)Theta);
						//heading_setpoint = lake.setpoint_heading(i, vehicle_gps_position.lat, vehicle_gps_position.lon);
						//heading_setpoint = (float)atan2(lake.latitude[i] - vehicle_gps_position.lat, lake.longitude[i] - vehicle_gps_position.lon); // online heading angle computation
						//if(lake.tolerance_circle(i, vehicle_gps_position.lat, vehicle_gps_position.lon) < M_PI*tolerance_radius^2){i++;}
						//if((lake.latitude[i] - vehicle_gps_position.lat)^2 + (lake.longitude[i] - vehicle_gps_position.lon)^2 < M_PI*tolerance_radius^2 && i < lake.number_points - 1){
						//	i++;
						//}
						//if(i < number_points - 1){point_reached = 1;}
						break;
					//case 2: // waypoints to reach taken from PX4 parameter
						//param_get(param_heading_longitude, &heading_longitude);
						//param_get(param_heading_latitude, &heading_latitude);
						//heading_setpoint = (float)atan2(heading_latitude - vehicle_gps_position.lat, heading_longitude - vehicle_gps_position.lon); // online heading angle computation
						//if(pow(heading_latitude - vehicle_gps_position.lat,2) + pow(heading_longitude - vehicle_gps_position.lon,2) > M_PI*pow(tolerance_radius,2)){
						//	heading_setpoint = (float64)atan2(heading_latitude - vehicle_gps_position.lat, heading_longitude - vehicle_gps_position.lon); // online heading angle computation
						//};
						//break;
					default: // heading setpoint, just the angle
						param_get(param_heading_set, &heading_set);
						heading_setpoint = (float64)heading_set*M_PI/180; // North (reference frame) setpoint in heading, tester/developer decision (put on top of the file?). 0 obviously means go straight
						// float error_heading = Theta - heading_setpoint; // /epsilon_{theta}

				}*/

				//param_get(param_heading_longitude, &heading_longitude);
				//param_get(param_heading_latitude, &heading_latitude);

				if (heading_strategy == 1){
					distance_waypoint = (double)get_distance_to_next_waypoint_sailing((double)lat, (double)lon, (double)lat_next_point, (double)lon_next_point); // with 0 works
					if (distance_waypoint > tolerance_radius){
					heading_setpoint = (double)get_bearing_to_next_waypoint_sailing((double)lat, (double)lon, (double)lat_next_point, (double)lon_next_point);
					}
					else if (i < number_points - 1){
					//if (distance_waypoint < tolerance_radius && i < number_points - 1){
						i++;
						if (i == 1){
							lat_next_point = lat_second;
							lon_next_point = lon_second;
						}
						if (i == 2){
							lat_next_point = lat_third;
							lon_next_point = lon_third;
						}
						//lat_next_point = lat_next[i];
						//lon_next_point = lon_next[i];
					}
				}
				//else if (heading_strategy == 2){
					//param_get(param_heading_longitude, &heading_longitude);
					//param_get(param_heading_latitude, &heading_latitude);
					//heading_longitude = (double)heading_longitude;
					//heading_latitude = (double)heading_latitude;
					//heading_setpoint = get_bearing_to_next_waypoint_sailing((double)lat, (double)lon, (double)heading_latitude, (double)heading_longitude);
				//}
				else {
					param_get(param_heading_set, &heading_set);
					heading_setpoint = (double)heading_set*M_PI/180; // North (reference frame) setpoint in heading, tester/developer decision (put on top of the file?). 0 obviously means go straight
				}

				//param_get(param_heading_latitude, &heading_latitude);

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

				/*P_error = Kp*sin(error_heading);
				I_error+= Ki*sin(error_heading);

				param_get(param_rudder_strategy, &rudder_strategy);
				if (rudder_strategy == 1){
					PX4_INFO("PI rudder");
					cmd_rudder_angle = (P_error + I_error < max_rudder_angle && (P_error + I_error) > - max_rudder_angle) ? P_error + I_error : sgn(error_heading); //rudder is computed by PI controller until it reaches the max threshold. The if-else limits to -1/1
				}
				wind_angle_actual = 5456378329.0;*/
				//latitude1 = 5456378329.0;

				PX4_INFO("cmd_sail: %f wnd_angle %f yaw %f course_ang %f  cmd_rudder %f head_set %f wind_to_n %f", (double)cmd_sail_angle, (double)wnd_to_boat, (double)current_yaw*180/M_PI, (double)course_angle*180/M_PI, (double)cmd_rudder_angle, (double)heading_setpoint, (double)distance_waypoint); // (double)Theta);

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
	orb_unsubscribe(vehicle_gps_position_sub);
}

int sailing_main(int argc, char *argv[])
{
	return Sailing::main(argc, argv);
}
