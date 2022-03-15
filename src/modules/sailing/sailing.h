/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <cmath>

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


#include <px4_module.h>
#include <px4_module_params.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>                // this topics hold the acceleration data
#include <uORB/topics/actuator_controls.h>              // this topic gives the actuators control input
#include <uORB/topics/vehicle_attitude.h>                  // this topic holds the orientation of the hippocampus
#include <uORB/topics/vehicle_odometry.h>                // this msg structure holds for odometry
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <drivers/drv_hrt.h>
// additions for navigation modes
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_wind_angle.h>
#include <uORB/topics/vehicle_gps_position.h>


using matrix::wrap_pi;

float get_bearing_to_next_waypoint_sailing(double lat_now, double lon_now, double lat_next, double lon_next){

	double lat_next_rad = math::radians(lat_next);
	double lat_now_rad = math::radians(lat_now);

	double d_lon = math::radians(lon_next) - math::radians(lon_now);

	double d_lat = lat_next_rad - math::radians(lat_now);

	/* conscious mix of double and float trig function to maximize speed and efficiency */

	double a = sin(d_lat / 2.0) * sin(d_lat / 2.0) + sin(d_lon / 2.0) * sin(d_lon / 2.0) * cos(lat_now_rad) * cos(lat_next_rad);

	double c = atan2(sqrt(a), sqrt(1.0 - a));

	float y = (double)(sin(d_lon) * cos(lat_next_rad));
	float x = (double)(cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos(lat_next_rad) * cos(d_lon));

	return (double)wrap_pi(atan2f(y, x));
}

float get_distance_to_next_waypoint_sailing(double lat_now, double lon_now, double lat_next, double lon_next){

	double lat_next_rad = math::radians(lat_next);
	double lat_now_rad = math::radians(lat_now);

	double d_lon = math::radians(lon_next) - math::radians(lon_now);

	double d_lat = lat_next_rad - lat_now_rad;

	/* conscious mix of double and float trig function to maximize speed and efficiency */

	double a = sin(d_lat / 2.0) * sin(d_lat / 2.0) + sin(d_lon / 2.0) * sin(d_lon / 2.0) * cos(lat_now_rad) * cos(lat_next_rad);

	//double c = atan2(sqrt(a), sqrt(1.0 - a));
	double c = asin(sqrt(a));

	//float y = (double)(sin(d_lon) * cos(lat_next_rad));
	//float x = (double)(cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos(lat_next_rad) * cos(d_lon));

	return (double)(6371000.0 * 2.0 * c);
}

float scaling(float val, float inMin, float inMax, float outMin, float outMax)
{
	float retVal = 0.0;

	retVal = (val - inMin) / (inMax - inMin) * (outMax - outMin) + outMin;

	if (retVal < outMin) {
		retVal = outMin;

	} else if (retVal > outMax) {
		retVal = outMax;
	}

	return retVal;
}
/*class Waypoint{
	public:
		size_t number_points;
		float latitude[];
		float longitude[];
		float setpoint_heading(int i, float vehicle_latitude, float vehicle_longitude){
			return (float)atan2(latitude[i] - vehicle_latitude, this->longitude[i] - vehicle_longitude); // online heading angle computation
		}
		float tolerance_circle(int i, float vehicle_latitude, float vehicle_longitude){
			return pow(latitude[i] - vehicle_latitude,2) + pow(this->longitude[i] - vehicle_longitude,2);
		}
};*/

extern "C" __EXPORT int sailing_main(int argc, char *argv[]);

#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

class Sailing : public ModuleBase<Sailing>, public ModuleParams
{
public:
	Sailing(int example_param, bool example_flag);

	virtual ~Sailing() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Sailing *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/* */
	void vehicle_poll();

	void fold_sails(int direction);

private:

	bool sails_are_down = true;
	float sail_angle_max = 60*(float)M_PI/180;
	//int wnd_angle_to_n = 179;
	int heading_set = 0;
	//int wind_strategy = 0;
	//int rudder_strategy = 0;
	int heading_strategy = 0;
	//int sailing_freeze = 0;
	//int stop_strategy = 0;
	int heading_latitude = 0;
	int heading_longitude = 0;

	// Publications
	orb_advert_t vehicle_control_mode_pub;
	orb_advert_t act_pub;
	orb_advert_t _outputs_pub;

	// Subscriptionheading_set
	int vehicle_control_mode_sub;
	int vehicle_attitude_sub;
	//int vehicle_odometry_sub;
	int parameter_update_sub;
	int manual_sp_sub;
	int param_update_sub;
	int vehicle_status_sub;
	int sensor_wind_angle_sub;
	int vehicle_gps_position_sub;

	struct manual_control_setpoint_s manual_sp; 		// RC input
	struct actuator_controls_s act;						// actuator outputs manual
	struct vehicle_attitude_s raw_att;					// attitude

	struct parameter_update_s param_upd;				// parameter handling (with QGC)
	struct vehicle_control_mode_s vehicle_control_mode;	// flags
	struct vehicle_status_s vehicle_status;			// navigation state
	//struct vehicle_odometry_s vehicle_odometry;		// vehicle odometry
	struct sensor_wind_angle_s sensor_wind_angle;				// wind sensor
	struct vehicle_gps_position_s vehicle_gps_position{};		// global position
	struct actuator_outputs_s  _outputs;      // actuator output


	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::HEADING_SET>) _heading_set,   /**< example parameter */
		//(ParamInt<px4::params::WIND_STRATEGY>) _wind_strategy,   /**< example parameter */
		//(ParamInt<px4::params::RUDDER_STRATEGY>) _rudder_strategy,   /**< example parameter */
		//(ParamInt<px4::params::WND_ANGLE_TO_N>) _wnd_angle_to_n,   /**< example parameter */
		(ParamFloat<px4::params::SMV_AIR_T>) _smv_air_t,   /**< example parameter */
		(ParamFloat<px4::params::SMV_H2O_T>) _smv_h2o_t,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _sys_autoconfig,  /**< another parameter */
		(ParamInt<px4::params::HEADING_STRATEGY>) _heading_strategy,   /**< example parameter */
		//(ParamInt<px4::params::SAILING_FREEZE>) _sailing_freeze,   /**< example parameter */
		//(ParamInt<px4::params::STOP_STRATEGY>) _stop_strategy,   /**< example parameter */
		(ParamInt<px4::params::HEADING_LAT>) _heading_latitude,   /**< example parameter */
		(ParamInt<px4::params::HEADING_LON>) _heading_longitude   /**< example parameter */
	)
};
