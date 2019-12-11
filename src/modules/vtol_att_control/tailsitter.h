/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
* @file tailsitter.h
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author David Vorsin     <davidvorsin@gmail.com>
*
*/

#ifndef TAILSITTER_H
#define TAILSITTER_H

#define POINT_NUM (5)

#include "vtol_type.h"
#include "ILC_DATA.h"
#include "euler_zxy.h"
#include "Quaternion_zxy.hpp"
#include <perf/perf_counter.h>  /** is it necsacery? **/
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>
#include <mathlib/math/EulerFromQuat.hpp>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <uORB/topics/vehicle_local_position.h>

class Tailsitter : public VtolType
{

public:
	Tailsitter(VtolAttitudeControl *_att_controller);
	~Tailsitter() override = default;

	void update_vtol_state() override;
	void update_transition_state() override;
	void update_fw_state() override;
	void fill_actuator_outputs() override;
	void waiting_on_tecs() override;

	virtual float calc_roll_sp();
	virtual void send_atti_sp();
	virtual void PID_Initialize();
	virtual void State_Machine_Initialize();

	float cal_sysidt_pitch();

	//virtual float control_vertical_speed(float vz, float vz_cmd);
	virtual float control_sideslip(float dt);
	virtual void  reset_trans_start_state();
	virtual float ILC_in(float time_since_trans_start);
	virtual float calc_vz_cmd(float time_since_trans_start);
	virtual float control_altitude(float time_since_trans_start, float alt_cmd, int control_loop_mode);
	virtual float control_vertical_acc(float time_since_trans_start, float vert_acc_cmd, float vert_vel_cmd);
	virtual float calc_pitch_rot(float time_since_trans_start);
	virtual float get_theta_cmd();
	virtual float calc_pitch_b_trans(float dt);
	virtual float calc_roll_b_trans(float dt);
	float lift_d_theta(float horiz_vel, float theta);
	bool is_ground_speed_satisfied();

private:

	struct {
		float front_trans_dur_p2;
		float fw_pitch_sp_offset;
		float sys_ident_input;
		int   sys_ident_num;
	} _params_tailsitter{};	

	struct {
		param_t front_trans_dur_p2;
		param_t fw_pitch_sp_offset;
		param_t sys_ident_input;
		param_t sys_ident_num;
	} _params_handles_tailsitter{};

	enum vtol_mode 
	{
		MC_MODE = 0,			/**< vtol is in multicopter mode */
		TRANSITION_FRONT_P1,	/**< vtol is in front transition part 1 mode */
		TRANSITION_BACK,		/**< vtol is in back transition mode */
		FW_MODE					/**< vtol is in fixed wing mode */
	};

	enum sweep_type 
	{
		NO_SWEEP = 0,
		PITCH_RATE,
		ROLL_RATE,
		YAW_RATE,
		THRUST
	};

	enum sysidt_state
	{
		SYSIDT_LOCK = 0,
		TRIM_FLIGHT,
		TURN_FLIGHT
	};

	enum control_mode
	{
		CONTROL_POS = 0,
		CONTROL_VEL,
		CONTROL_VEL_WITHOUT_ACC,
		CONTROL_ACC
	};

	struct {
		vtol_mode   flight_mode;	    /**< vtol flight mode, defined by enum vtol_mode */
		float       ctrl_out_trans_end; /**< MC controller output at the end of front transition */
		hrt_abstime fw_start;           /**< absoulte time at which fw mode started, this time will be used to smooth the controller output */
		hrt_abstime sweep_start;
		hrt_abstime _trans_start_t;	/**< absoulte time at which front transition started */
		bool 	    vz_mission_finished = false;
	} _vtol_schedule;

	struct {
		uint8_t state = 0;
		uint8_t global_counter = 0;		
		uint8_t trim_counter = 0;
		uint8_t turn_counter = 0;
		float trim_timer = 0.0f;
		float angle_start = 0.0f;
		bool  is_accelerated = false;
	} _vtol_sysidt;

	struct _PID_Control{
	 	bool is_saturated = false;
	 	float last_D_state = 0;
	 	float last_I_state = 0;
	 	float last_run;
	} _VZ_PID_Control,_VY_PID_Control,_VX_PID_Control;

	matrix::Quatf _q_trans_start;
	matrix::Quatf _q_trans_sp;
	matrix::Vector3f _trans_pitch_axis;
	matrix::Vector3f _trans_roll_axis;
	matrix::Vector3f _trans_yaw_axis;
	math::LowPassFilter2p	_accel_filter_x;
	math::LowPassFilter2p	_accel_filter_y;
	math::LowPassFilter2p	_accel_filter_z;


	const float POINT_ACTION[2][POINT_NUM] = {
	{0.0f, 2.5f, 3.2f, 5.1f, 7.0f},
	{0.0f, 75.0f, 75.0f, 54.88f, 75.00f}
	};

/***
	const float POINT_ACTION[2][POINT_NUM] = {
	{0.0f, 2.5f, 3.2f, 5.1f, 7.0f},
	{0.0f, 75.0f, 75.0f, 27.44f, 75.00f}
	};
***/

	float _alt_sp;
	float _last_run_time;
	float _vert_i_term;
	float _mc_hover_thrust;
	float _trans_end_thrust;
	float _trans_pitch_rot;
	float _trans_roll_rot;
	float _trans_start_x;
	float _trans_start_y;
	float _trans_start_yaw;
	float _trans_start_pitch;
	float _trans_start_roll;
	float _target_alt;
	float _yaw;
	float _pitch;
	float _roll;

	void parameters_update() override;

};
#endif
