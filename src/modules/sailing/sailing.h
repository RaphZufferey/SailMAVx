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
#include <uORB/topics/manual_control_setpoint.h> 
#include <drivers/drv_hrt.h>
// additions for navigation modes
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>


extern "C" __EXPORT int sailing_main(int argc, char *argv[]);


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
	int wnd_angle_to_n = 179;


	// Publications
	orb_advert_t vehicle_control_mode_pub;
	orb_advert_t act_pub;

	// Subscription
	int vehicle_control_mode_sub;
	int vehicle_attitude_sub;
	int vehicle_odometry_sub;
	int parameter_update_sub;
	int manual_sp_sub;
	int param_update_sub;
	int vehicle_status_sub;

	struct manual_control_setpoint_s manual_sp; 		// RC input
	struct actuator_controls_s act;						// actuator outputs manual
	struct vehicle_attitude_s raw_att;					// attitude
	struct vehicle_odometry_s raw_odom;
	struct parameter_update_s param_upd;				// parameter handling (with QGC)
	struct vehicle_control_mode_s vehicle_control_mode;	// flags
	struct vehicle_status_s vehicle_status;				// navigation state

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::NEW>) _new,   /**< example parameter */
		(ParamInt<px4::params::WND_ANGLE_TO_N>) _wnd_angle_to_n,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _sys_autoconfig  /**< another parameter */
	)
};
