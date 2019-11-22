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
* @file tailsitter.cpp
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author David Vorsin     <davidvorsin@gmail.com>
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include "vtol_att_control_main.h"
#include <systemlib/mavlink_log.h>
#include <math.h>
#include "tailsitter.h"

#ifndef M_PI
#define M_PI (3.14159265f)
#endif

//#define SYSIDT

#define RAD_TO_DEG(x) ((x) / 3.1416f * 180.0f)
#define DEG_TO_RAD(x) ((x) / 180.0f * 3.1416f)
#define CORRECT_ANG(x) (((x)>DEG_TO_RAD(359.9f))? (x)-DEG_TO_RAD(359.9f) : (((x)<DEG_TO_RAD(-359.9f))? (x)+DEG_TO_RAD(359.9f) : (x)))


#define ARSP_YAW_CTRL_DISABLE     (4.0f)	// airspeed at which we stop controlling yaw during a front transition
#define THROTTLE_TRANSITION_MAX   (0.25f)	// maximum added thrust above last value in transition
#define PITCH_TRANSITION_FRONT_P1 (-_params->front_trans_pitch_sp_p1)	// pitch angle to switch to TRANSITION_P2
#define PITCH_TRANSITION_BACK     (0.2f)	// pitch angle to switch to MC
#define Max_Thrust_cmd 0.9f
#define	Min_Thrust_cmd 0.1f
#define VERT_CONTROL_MODE  (CONTROL_POS) // modes: CONTROL_POS, CONTROL_VEL, CONTROL_VEL_WITHOUT_ACC

#define CTRL_FREQ (250.0f)

static  orb_advert_t mavlink_log_pub = nullptr;

using namespace matrix;

Tailsitter::Tailsitter(VtolAttitudeControl *attc) :
	VtolType(attc),
	_accel_filter_x(CTRL_FREQ, 12.0f),
	_accel_filter_y(CTRL_FREQ, 12.0f),
	_accel_filter_z(CTRL_FREQ, 12.0f)
{
	_vtol_schedule.flight_mode = MC_MODE;
	_vtol_schedule._trans_start_t = 0.0f;

	_flag_was_in_trans_mode = false;

	_params_handles_tailsitter.sys_ident_input    = param_find("SYS_IDENT_INPUT");
	_params_handles_tailsitter.sys_ident_num      = param_find("SYS_IDENT_NUM");
}

void Tailsitter::PID_Initialize(){
	float now = float(hrt_absolute_time()) * 1e-6f;
	_VY_PID_Control.last_run = now;
	_VY_PID_Control.is_saturated = false;	
	_VY_PID_Control.last_I_state = 0;
	_VY_PID_Control.last_D_state = 0;	
	_VX_PID_Control.last_run = now;
	_VX_PID_Control.is_saturated = false;
	_VX_PID_Control.last_I_state = 0;
	_VX_PID_Control.last_D_state = 0;		
	_VZ_PID_Control.last_run = now;
	_VZ_PID_Control.is_saturated = false;
	_VZ_PID_Control.last_I_state = 0;	
	_VZ_PID_Control.last_D_state = 0;	
}

void Tailsitter::parameters_update()
{
	float v;

	/* vtol front transition phase 2 duration */
	param_get(_params_handles_tailsitter.sys_ident_input, &v);
	_params_tailsitter.sys_ident_input = v;

	param_get(_params_handles_tailsitter.sys_ident_num, &v);
	_params_tailsitter.sys_ident_num = v;

	/* update the CL points */
	int iden_num = 0;
	if ((_params_tailsitter.sys_ident_num >= 9) && (_params_tailsitter.sys_ident_num >= 0)) {
		iden_num = _params_tailsitter.sys_ident_num;
	}

	memcpy(_CL_Degree, CL_SYS_ID[iden_num], sizeof(_CL_Degree));

	//mavlink_log_critical(&mavlink_log_pub, "sys_ident_cl_point:%.5f inttest:%d", (double)(_CL_Degree[19]), int(16.99f * 1));

}

void Tailsitter::update_vtol_state()
{
	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting in MC control mode, picking up
	 * forward speed. After the vehicle has picked up enough and sufficient pitch angle the uav will go into FW mode.
	 * For the backtransition the pitch is controlled in MC mode again and switches to full MC control reaching the sufficient pitch angle.
	*/

	Eulerf_zxy q_zxy(Quatf(_v_att->q));
	float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule._trans_start_t) * 1e-6f;
	float pitch = q_zxy.theta();
	float roll = q_zxy.phi();

	if (!_attc->is_fixed_wing_requested()) {

		switch (_vtol_schedule.flight_mode) { // user switchig to MC mode
		case MC_MODE:
			break;

		case FW_MODE:
			_vtol_schedule.flight_mode 	= TRANSITION_BACK;
			break;

		case TRANSITION_FRONT_P1:
			// failsafe into multicopter mode
			_vtol_schedule.flight_mode = MC_MODE;
			break;

		case TRANSITION_BACK:
			// check if we have reached pitch angle to switch to MC mode
			if (fabsf(pitch) <= PITCH_TRANSITION_BACK || time_since_trans_start >= _params->back_trans_duration)
			{
				_vtol_schedule.flight_mode = MC_MODE;
			}
			else
			{
				_vtol_schedule.flight_mode = TRANSITION_BACK;
			}

			break;
		}

	} else {  // user switchig to FW mode

		switch (_vtol_schedule.flight_mode) {
		case MC_MODE:
			// initialise a front transition
		if (_local_pos->z < (- _params->vt_safe_alt)){
			_vtol_schedule.flight_mode 	= TRANSITION_FRONT_P1;

		}
			break;

		case FW_MODE:
			
			if ((fabsf(pitch) >= DEG_TO_RAD(115.0f)) || (fabsf(roll) >= DEG_TO_RAD(85.0f)))
			{
				_attc->abort_front_transition("dangerous attitude");
				_vtol_schedule.flight_mode = TRANSITION_BACK;
			}

			if ((_local_pos->z > (- _params->vt_safe_alt)) || (_local_pos->vz > 10.0f))
			{
				_attc->abort_front_transition("dangerous height");
				_vtol_schedule.flight_mode = MC_MODE;
			}
			break;

		case TRANSITION_FRONT_P1: {
				bool airspeed_condition_satisfied = _airspeed->indicated_airspeed_m_s >= _params->transition_airspeed;
				airspeed_condition_satisfied |= _params->airspeed_disabled;

				_vtol_schedule.fw_start = hrt_absolute_time();

				// check if we have reached airspeed  and the transition time is over the setpoint to switch to TRANSITION P2 mode
				
				if ((airspeed_condition_satisfied && (time_since_trans_start >= _params->front_trans_duration)) || can_transition_on_ground()) {
					_vtol_schedule.flight_mode = FW_MODE;
					//_vtol_schedule.flight_mode = MC_MODE;
				}
				else if (time_since_trans_start >= _params->front_trans_duration + 2.0f){
					_attc->abort_front_transition("Transition timeout");
				}

				break;
			}

		case TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_schedule.flight_mode = FW_MODE;
			break;
		}
	}
	
	
	// map tailsitter specific control phases to simple control modes
	switch (_vtol_schedule.flight_mode) {
	case MC_MODE:
		reset_trans_start_state();
		_vtol_mode = ROTARY_WING;
		_vtol_vehicle_status->vtol_in_trans_mode = false;
		_flag_was_in_trans_mode = false;
		break;

	case FW_MODE:
		reset_trans_start_state();
		_vtol_mode = FIXED_WING;
		_vtol_vehicle_status->vtol_in_trans_mode = false;
		break;

	case TRANSITION_FRONT_P1:
		if (_vtol_mode != TRANSITION_TO_FW)
		{
			reset_trans_start_state();
		}
		_vtol_mode = TRANSITION_TO_FW;
		_vtol_vehicle_status->vtol_in_trans_mode = true;
		break;

	case TRANSITION_BACK:
		if (_vtol_mode != TRANSITION_TO_MC)
		{
			reset_trans_start_state();
		}
		_vtol_mode = TRANSITION_TO_MC;
		_vtol_vehicle_status->vtol_in_trans_mode = true;
		break;
	}
}

void Tailsitter::reset_trans_start_state()
{
	_vtol_schedule._trans_start_t = hrt_absolute_time();
	_trans_start_yaw = Eulerf_zxy(Quatf(_v_att->q)).psi();
	_trans_start_pitch = Eulerf_zxy(Quatf(_v_att->q)).theta();
	_trans_start_roll = Eulerf_zxy(Quatf(_v_att->q)).phi();

	_trans_roll_rot  = _trans_start_roll;
	_trans_pitch_rot = _trans_start_pitch;
	_yaw             = _trans_start_yaw;

	_mc_hover_thrust = _v_att_sp->thrust_body[2];
	_alt_sp          = _local_pos->z;
	_trans_start_y   = _local_pos->y;
	_trans_start_x   = _local_pos->x;
}


/***
 *	calculate the thrust feedforward cmd based on the vertical acceleration cmd, horizontal velocity, pitch angle and ang-of-attack
 *	@input: vertical acceleration cmd (up is positive)
 *			horizontal velocity
 *			pitch angle
 *			ang-of-attack
 *	@output: thrust feedforward cmd
 ***/
float Tailsitter::control_vertical_acc(float time_since_trans_start, float vert_acc_cmd, float vert_vel_cmd)
{
	float dt = 1.0f / 250.0f;
	//float ILC_input    = ILC_in(time_since_trans_start);
	float bx_acc_filt  = _accel_filter_x.apply(_sensor_acc->x);
	float bz_acc_filt  = _accel_filter_z.apply(_sensor_acc->z);
	float pitchrate_filt = _accel_filter_y.apply(_v_att->pitchspeed);
	//float bx_acc_cmd   = 0.0f;
	//float bx_acc_err   = 0.0f;
	//float bx_acc_err_i = 0.0f;

	/* bx_acc_kp and bx_acc_ki are from loopshaping */
	//float bx_acc_kp    = 0.006f;
	//float bx_acc_ki    = 0.003f;
	float thrust_cmd   = 0.0f;
	//float cos_pitch    = 0.0f;

	matrix::EulerFromQuatf euler = matrix::Quatf(_v_att->q);
	float pitch      = math::constrain(- euler.theta(), DEG_TO_RAD(0.001f), DEG_TO_RAD(89.99f)); // theta is minus zero
	float roll       = math::constrain(  euler.phi(), DEG_TO_RAD(0.001f), DEG_TO_RAD(89.99f));

	/** incremental control **/	
	float horiz_vel  = sqrtf((_local_pos->vx * _local_pos->vx) + (_local_pos->vy * _local_pos->vy));
	float thrust_f   = _v_att_sp->thrust_body[2] / _mc_hover_thrust * 9.8f;
	float thrust_to_acc = -_mc_hover_thrust / 9.8f;
	float acc_iz_fdb = (bz_acc_filt * sinf(pitch) - bx_acc_filt * cosf(pitch))*cosf(roll) + 9.8f;
	float acc_iz_err = vert_acc_cmd- acc_iz_fdb;
	float delt_thrust = (-acc_iz_err - (thrust_f * sinf(-pitch) + lift_d_theta(horiz_vel, -pitch)) * pitchrate_filt * dt) / cos(pitch) * thrust_to_acc;

	if ((fabsf(pitch) < DEG_TO_RAD(89.999f)) && (fabsf(pitch) >= DEG_TO_RAD(0.001f)))
	{
		/***
		cos_pitch     = math::constrain(cosf(pitch), 0.2f, 1.0f);
		bx_acc_cmd    = (9.8f + _sensor_acc->z * sinf(pitch) - acc_iz_err) / (cos_pitch - 2.6f * sinf(pitch) * (1 - cosf(pitch)));
		bx_acc_cmd    = math::constrain(bx_acc_cmd, -2.0f * 9.8f, 2.0f * 9.8f);
		bx_acc_err    = bx_acc_cmd - _sensor_acc->x;
		bx_acc_err_i  = _vtol_vehicle_status->bx_acc_i + bx_acc_ki * bx_acc_err * 0.004f;
		***/
		
		thrust_cmd    = _v_att_sp->thrust_body[2] + delt_thrust;
	}
	else
	{
		//lift_weight_ratio = 0.0f;
		thrust_cmd = -_mc_hover_thrust;
	}

	_vtol_vehicle_status->acc_iz_fdb = acc_iz_fdb;
	_vtol_vehicle_status->delt_thrust     = delt_thrust;

	return thrust_cmd;
}

float Tailsitter::lift_d_theta(float horiz_vel, float theta)
{
	float L_d_theta = 0.0f;
	if (horiz_vel <= 12.0f)
	{
		if (theta > DEG_TO_RAD(-40.f))
		{
			L_d_theta = 0.0f;
		}
		else if (theta > DEG_TO_RAD(-80.f))
		{
			L_d_theta = 24.f * (theta + DEG_TO_RAD(40.f)) / DEG_TO_RAD(40.f);
		}
		else
		{
			L_d_theta = 1.0f;
		}
	}
	if (horiz_vel > 12.0f)
	{
		L_d_theta = -(horiz_vel - 8.5f) * 6.88f;
	}
	return L_d_theta;
}

/* 
*  Calculate the vz command according to the predesigned trajectory  
*  Vz increase from a certain minimum speed Min_Speed to a maximum speed Max_Speed.
*  Every 1m/s there will be a fixed speed flight interval lasting for KeepTime secs.
*  AccTime secs will be cost to accelerate the vehicle to increase 1m/s
*  To smooth the step trajectory, the acceleration interval will be generated from a 
*  sigmoid function.
*/
float Tailsitter::calc_vz_cmd(float time_since_trans_start)
{
	int vz_cmd_index;
	float vz_change_period, current_vz_cmd, time_in_perioid;
	float k_sigmoid = 20 / _params->vt_vz_acctime, time_in_sigmoid = 0, sigmoid_value = 0;

	vz_change_period = _params->vt_vz_acctime + _params->vt_vz_keeptime;
	vz_cmd_index = floor(time_since_trans_start / vz_change_period);
	time_in_perioid = time_since_trans_start - vz_cmd_index * vz_change_period;
	current_vz_cmd = _params->vt_vz_minspeed + vz_cmd_index * _params->vt_vz_interval;

	if (time_in_perioid > _params->vt_vz_keeptime ){

		time_in_sigmoid = time_in_perioid - _params->vt_vz_keeptime - 0.5f * _params->vt_vz_acctime;
		sigmoid_value = _params->vt_vz_interval/(1 + exp(-k_sigmoid * time_in_sigmoid));
		current_vz_cmd += sigmoid_value;
	}

	if (current_vz_cmd > _params->vt_vz_maxspeed + 0.01f) {
		current_vz_cmd = -0.0f;
	}

	/* To avoid the vehicle from flying too high */
	if (_local_pos->z < (-_params->vt_max_height)){
		current_vz_cmd = -0.0f;
	}

	return -current_vz_cmd;
}

float Tailsitter::control_sideslip(float dt)
{
	if(_params->vt_sideslip_ctrl_en)
	{
		if(_yaw >= DEG_TO_RAD(180.0f))
		{
			_yaw -= DEG_TO_RAD(360.f);
		}
	}
	return _yaw;
}

/***
 *	calculate the acceleration cmd using feedforward and feedback controller
 *	@input: 
 *	@output:
 ***/
float Tailsitter::control_altitude(float time_since_trans_start, float alt_cmd, int control_loop_mode)
{
	/* state input */
	
	
	/* position loop */
	float alt_kp = _params->vt_x_dist_kp;
	float vz_cmd = (control_loop_mode == CONTROL_POS) ? ((alt_cmd - _local_pos->z) * alt_kp) : calc_vz_cmd(time_since_trans_start);

	/* velocity loop  */
	float vel_kp = _params->vt_vz_control_kp;
	float vel_ki = _params->vt_vz_control_ki;
	float vel_kd = _params->vt_vz_control_kd;

	float now = float(hrt_absolute_time()) * 1e-6f;
	float dt  = now - _VZ_PID_Control.last_run;
	float vel_error  = vz_cmd - _local_pos->vz;
	float v_P_output = -vel_kp * vel_error;
	float v_I_output =  (-vel_ki) * vel_error * dt + _VZ_PID_Control.last_I_state;
	v_I_output = math::constrain(v_I_output, -0.8f, 0.8f);
	float v_D_output = (-vel_kd) * (vel_error - _VZ_PID_Control.last_D_state);
	float vert_acc_cmd = (v_P_output + v_I_output + v_D_output) * 9.8f;

	_VZ_PID_Control.last_run     = now;	
	_VZ_PID_Control.last_I_state = v_I_output;
	_VZ_PID_Control.last_D_state = vel_error; 
	_VZ_PID_Control.is_saturated = (vert_acc_cmd < 0.1f || vert_acc_cmd > 9.5f) ? true : false;//between 0.1G to 1G

	/* acc loop*/
	float thrust_cmd = 0.0f;
	switch(control_loop_mode)
	{
		case CONTROL_ACC:
			thrust_cmd = math::constrain(control_vertical_acc(time_since_trans_start, -vert_acc_cmd, vz_cmd), 0.10f, 0.95f);
			break;
		case CONTROL_VEL_WITHOUT_ACC:
		case CONTROL_POS:
		default:
			thrust_cmd = math::constrain(vert_acc_cmd / 9.8f+ (- _mc_hover_thrust), 0.10f,0.95f);
			break;
	}
	

	/* record data */
	_vtol_vehicle_status->vz_cmd       = vz_cmd;
	//_vtol_vehicle_status->ilc_input    = ILC_input;
	_vtol_vehicle_status->vert_acc_cmd = vert_acc_cmd;
	_vtol_vehicle_status->thrust_cmd   = thrust_cmd;
	_vtol_vehicle_status->ticks_since_trans ++;

	/* send back command and feedback data */
	//mavlink_log_critical(&mavlink_log_pub, "thr_cmd:%.5f", (double)(thrust_cmd));

	return (-1.0f * thrust_cmd);
}

float Tailsitter::calc_pitch_rot(float time_since_trans_start) {
	float angle = 0.0;

	for (int i = 0; i < (POINT_NUM - 1); i ++) {
		if (time_since_trans_start <= POINT_ACTION[0][i+1]) {
			angle = POINT_ACTION[1][i] + (time_since_trans_start - POINT_ACTION[0][i]) / (POINT_ACTION[0][i+1] - POINT_ACTION[0][i]) * (POINT_ACTION[1][i+1] - POINT_ACTION[1][i]);
			angle = DEG_TO_RAD(angle);
			break;
		}
	}

	if (time_since_trans_start >= POINT_ACTION[0][POINT_NUM - 1]) 
	{
		angle = DEG_TO_RAD(POINT_ACTION[1][POINT_NUM - 1]);
	}

	mavlink_log_critical(&mavlink_log_pub, "pitchsp:%.5f", (double)(-RAD_TO_DEG(angle)));
	return -1.0f * angle;
}

float Tailsitter::calc_roll_sp()
{
	float lateral_dist, lateral_v;
	//float longitudinal_dist, longitudinal_v, pitchrot;
	float rollrot;
	float Kp, Kvp, Kvi;
	float v_cmd, v_error, P_output, I_output;
	float delt_x, delt_y;
	float vx, vy;
	float now, dt;

	now = float(hrt_absolute_time()) * 1e-6f;
	dt  = now - _VY_PID_Control.last_run;
	_VY_PID_Control.last_run = now;
	_VX_PID_Control.last_run = now;

	delt_x = _local_pos->x - _trans_start_x;
	delt_y = _local_pos->y - _trans_start_y;
	vx = _local_pos->vx;
	vy = _local_pos->vy;
	lateral_dist = sqrtf(delt_x * delt_x + delt_y * delt_y) * sinf(atan2f(delt_y, delt_x) - _trans_start_yaw);
	// longitudinal_dist = sqrtf(delt_x * delt_x + delt_y * delt_y) * cosf((atan2f(delt_y, delt_x) - _mc_virtual_att_sp->yaw_body));
	lateral_v = sqrtf(vx * vx + vy * vy) * sinf(atan2f(vy, vx) - _trans_start_yaw);
	// longitudinal_v = sqrtf(vx * vx + vy * vy) * cosf(atan2f(vy, vx) - _mc_virtual_att_sp-> yaw_body);

	// PI controller of lateral dist
	Kp = _params->vt_y_dist_kp;
	Kvp = _params->vt_vy_kp;
	Kvi = _params->vt_vy_ki;

	if (_VY_PID_Control.is_saturated){
		Kvi = 0;
	}

	v_cmd = -Kp * lateral_dist;
	_vtol_vehicle_status->vy_cmd = v_cmd;
	v_error = (v_cmd - lateral_v);
	P_output = Kvp * v_error;
	I_output = _VY_PID_Control.last_I_state + (-Kvi) * v_error * dt;;
	rollrot = P_output + I_output;
	if (fabsf(rollrot) > 0.3f){
		_VY_PID_Control.is_saturated = true;
	} else {
		_VY_PID_Control.is_saturated = false;
	}

	_vtol_vehicle_status->lat_dist      = lateral_dist;
	_vtol_vehicle_status->lateral_v 	= lateral_v;
	return math::constrain(rollrot, -0.3f, 0.3f);
}

void Tailsitter::State_Machine_Initialize(){
	_vtol_sysidt.state = TRIM_FLIGHT;
	_vtol_sysidt.global_counter = 0;
	_vtol_sysidt.trim_counter = 0;
	_vtol_sysidt.turn_counter = 0;
	_vtol_sysidt.trim_timer = float(hrt_absolute_time()) * 1e-6f;
	_vtol_sysidt.angle_start = atan2f(_local_pos->vx, _local_pos->vy);
	_vtol_sysidt.is_accelerated = false;
	return;
}

bool Tailsitter::is_ground_speed_satisfied(){
	float vx = _local_pos->vx;
	float vy = _local_pos->vy;
	float angle = atan2f(vy, vx);
	float delta_angle = 180.0f - RAD_TO_DEG(fabsf(angle - _trans_start_yaw));

	if (fabsf(delta_angle) < 1.0f)
	{
		_trans_start_x   = _local_pos->x;
		_trans_start_y   = _local_pos->y;
		_trans_start_yaw = (_trans_start_yaw >= DEG_TO_RAD(180.0f)) ? (_trans_start_yaw - DEG_TO_RAD(180.0f)) : (_trans_start_yaw + DEG_TO_RAD(180.0f));
		_yaw             = Eulerf_zxy(Quatf(_v_att->q)).psi();
		return (true);
	}
	else return (false);
	return false;
}

float Tailsitter::get_theta_cmd(){
	float theta;
	theta = _params->sysidt_minaoa + _vtol_sysidt.global_counter * _params->sysidt_interval;
	if (theta > _params->sysidt_maxaoa){
		theta = _params->sysidt_maxaoa;
	}
	return (90.0f - theta); 
}

float Tailsitter::cal_sysidt_pitch()
{
	float pitch_sp = DEG_TO_RAD(-POINT_ACTION[1][POINT_NUM - 1]);
	if (_mission_result->seq_current > 0)
	{
		pitch_sp = - 90.0f + (_params->sysidt_minaoa + (_mission_result->instance_count - 1) * _params->sysidt_interval);
		pitch_sp = DEG_TO_RAD(math::constrain(pitch_sp, -90.0f, 0.0f));
	}

	return pitch_sp;
}

float Tailsitter::calc_pitch_b_trans(float dt)
{
	float cmd = 0.0f;
	cmd = _trans_pitch_rot - dt * _trans_start_pitch / math::constrain(_params->back_trans_duration, 0.5f, 100.0f);

	if (_trans_pitch_rot >= 0.0f)
	{
		return math::constrain(cmd, -0.01f, _trans_pitch_rot);
	}
	else
	{
		return math::constrain(cmd, _trans_pitch_rot, 0.01f);
	}
}

float Tailsitter::calc_roll_b_trans(float dt)
{
	float cmd = 0.0f;
	cmd = _trans_roll_rot - dt * _trans_start_roll / math::constrain(_params->back_trans_duration, 0.5f, 100.0f);

	if (_trans_roll_rot >= 0.0f)
	{
		return math::constrain(cmd, -0.01f, _trans_roll_rot);
	}
	else
	{
		return math::constrain(cmd, _trans_roll_rot, 0.01f);
	}
}


void Tailsitter::update_transition_state()
{
	float dt = (float)(hrt_absolute_time()) * 1e-6f - _last_run_time;
	_last_run_time = (float)(hrt_absolute_time()) * 1e-6f;

	/** check mode **/
	if((_vtol_mode != TRANSITION_TO_FW) && (_vtol_mode != TRANSITION_TO_MC) && (_vtol_mode != FIXED_WING))
	{
		_vtol_mode = ROTARY_WING;
		return;
	}

	/** initialization **/
	if (!_flag_was_in_trans_mode) 
	{
		_flag_was_in_trans_mode = true;
		PID_Initialize();
		State_Machine_Initialize();
		reset_trans_start_state();
		_vert_i_term = 0.0f;
		//POINT_ACTION[1][0] = RAD_TO_DEG(_trans_start_pitch);
	}

	float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule._trans_start_t) * 1e-6f;

	_v_att_sp->thrust_body[2] = _mc_virtual_att_sp->thrust_body[2];

	/** Front Trans Control **/
	switch (_vtol_schedule.flight_mode)
	{
		case TRANSITION_FRONT_P1:
		{
			_trans_pitch_rot = calc_pitch_rot(time_since_trans_start);
			/* lateral control */
			_trans_roll_rot  = calc_roll_sp();

			/* sideslip control */
			_v_att_sp->sideslip_ctrl_en = false;
			_v_att_sp->yaw_sp_move_rate = 0.0f;
			_yaw = _trans_start_yaw;

			/* Altitude control */
			_v_att_sp->thrust_body[2] = control_altitude(time_since_trans_start, _alt_sp, CONTROL_ACC);

			/* save the thrust value at the end of the transition */
			_trans_end_thrust = _actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];
			break;
		}

		case TRANSITION_BACK:
		{
			_trans_roll_rot = calc_roll_b_trans(dt);

			_trans_pitch_rot = calc_pitch_b_trans(dt);

			_yaw = _trans_start_yaw;

			_v_att_sp->sideslip_ctrl_en = false;

			_v_att_sp->thrust_body[2] = math::constrain(control_altitude(time_since_trans_start, _alt_sp, VERT_CONTROL_MODE), -0.7f, -0.30f);
			break;
		}

		case FW_MODE:
		{
			/* vertical control */
			#ifdef SYSIDT
			_trans_pitch_rot = cal_sysidt_pitch();
			#else
			_trans_pitch_rot = DEG_TO_RAD(-90.0f)+_fw_virtual_att_sp->pitch_body;
			#endif

			/* lateral control */
			_trans_roll_rot = _fw_virtual_att_sp->roll_body;

			/* sideslip control */
			_v_att_sp->sideslip_ctrl_en = true;
			_yaw = _fw_virtual_att_sp->yaw_body;//control_sideslip(dt);

			/* Altitude control */
			#ifdef SYSIDT
			_v_att_sp->thrust_body[2] = control_altitude(time_since_trans_start, _alt_sp, VERT_CONTROL_MODE);
			#else
			_alt_sp = _local_pos->z;
			_v_att_sp->thrust_body[2] = -_fw_virtual_att_sp->thrust_body[0];
			PID_Initialize();
			#endif
			
			break;
		}
		default:
			break;
	}

	send_atti_sp();
/***
	static int ii = 0;
	ii ++;
	if (ii % 25 == 1)
	{
		mavlink_log_critical(&mavlink_log_pub, "wp_item: %d calulated pitch sp: %.4f %.4f", _mission_result->seq_current, double(RAD_TO_DEG(_trans_pitch_rot)), double(RAD_TO_DEG(_fw_virtual_att_sp->pitch_body)));
	}
***/
}

void Tailsitter::send_atti_sp()
{
	_v_att_sp->roll_body  = _trans_roll_rot;
	_v_att_sp->pitch_body = _trans_pitch_rot;
	_v_att_sp->yaw_body   = _yaw;

	Quatf_zxy q_sp(Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body));
	q_sp.copyTo(_v_att_sp->q_d);

	_v_att_sp->q_d_valid = true;
	_v_att_sp->timestamp = hrt_absolute_time();

	_vtol_vehicle_status->pitch_sp = _trans_pitch_rot;
}

void Tailsitter::waiting_on_tecs()
{
	// copy the last trust value from the front transition
	_v_att_sp->thrust_body[2] = _thrust_transition;
}

void Tailsitter::update_fw_state()
{
	VtolType::update_fw_state();
}

/**
* Write data to actuator output topic.
*/
void Tailsitter::fill_actuator_outputs()
{
	float time_since_sweep = 0.0f;
	float sweep_signal_phase = 0.0f;
	float sweep_signal = 0.0f;
	float sweep_min_frequency = 0.5f * 6.2831f;
	float sweep_max_frequency = 80.0f * 6.2831f ;
	float overall_time = 150.0f;
	//float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.f_trans_start_t) * 1e-6f;

	_actuators_out_0->timestamp = hrt_absolute_time();
	_actuators_out_0->timestamp_sample = _actuators_mc_in->timestamp_sample;

	_actuators_out_1->timestamp = hrt_absolute_time();
	_actuators_out_1->timestamp_sample = _actuators_fw_in->timestamp_sample;

	switch (_vtol_mode) {
	case ROTARY_WING:
		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL];
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] = _actuators_mc_in->control[actuator_controls_s::INDEX_PITCH];
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = _actuators_mc_in->control[actuator_controls_s::INDEX_YAW];
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] = _actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];

		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] = 0.0f;	// roll elevon
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] = 0.0f;

		/* Used for sweep experiment's input signal */
		if(_attc->is_sweep_requested()) {
			switch (_params->vt_sweep_type){
			case NO_SWEEP:
				break;
			case PITCH_RATE:
				time_since_sweep = (float)(hrt_absolute_time() - _vtol_schedule.sweep_start) * 1e-6f;
				// Exponantial Chirp
				sweep_signal_phase = 8.0f * 6.2831f * time_since_sweep;//sweep_min_frequency * time_since_sweep + 0.0187f * (sweep_max_frequency - sweep_min_frequency) * (overall_time / 4.0f * powf(2.7183f, (4.0f * time_since_sweep / overall_time)) - time_since_sweep);
				// Linear Chirp
				//sweep_signal_phase = sweep_min_frequency * time_since_sweep + 0.5f * (sweep_max_frequency - sweep_min_frequency) * (time_since_sweep * time_since_sweep / overall_time);
				sweep_signal = (float)(_params->vt_sweep_amp) * sinf(sweep_signal_phase);
				_actuators_out_0->sweep_input = sweep_signal;
				_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] = _actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] + sweep_signal;
				break;
		    case ROLL_RATE:
		    		time_since_sweep = (float)(hrt_absolute_time() - _vtol_schedule.sweep_start) * 1e-6f;
		    		// Exponantial Chirp
				sweep_signal_phase = 8.0f * 6.2831f * time_since_sweep;//sweep_min_frequency * time_since_sweep + 0.0187f * (sweep_max_frequency - sweep_min_frequency) * (overall_time / 4.0f * powf(2.7183f, (4.0f * time_since_sweep / overall_time)) - time_since_sweep);
				// Linear Chirp
				// sweep_signal_phase = sweep_min_frequency  * time_since_sweep + 0.5f * (sweep_max_frequency - sweep_min_frequency) * (time_since_sweep * time_since_sweep / overall_time);
				sweep_signal = (float)(_params->vt_sweep_amp) * sinf(sweep_signal_phase);
				_actuators_out_0->sweep_input = sweep_signal;
				_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL] + sweep_signal;
				break;
			case THRUST:
				time_since_sweep = (float)(hrt_absolute_time() - _vtol_schedule.sweep_start) * 1e-6f;
		    	// Exponantial Chirp
				sweep_signal_phase = sweep_min_frequency * time_since_sweep + 0.0187f * (sweep_max_frequency - sweep_min_frequency) * (overall_time / 4.0f * powf(2.7183f, (4.0f * time_since_sweep / overall_time)) - time_since_sweep);
				// Linear Chirp
				// sweep_signal_phase = sweep_min_frequency  * time_since_sweep + 0.5f * (sweep_max_frequency - sweep_min_frequency) * (time_since_sweep * time_since_sweep / overall_time);
				sweep_signal = (float)(_params->vt_sweep_amp) * sinf(sweep_signal_phase);
				_actuators_out_0->sweep_input = sweep_signal;
				_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] = _actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE] + sweep_signal;
				break;
			}
		} else {
			/* record the start time */
			_vtol_schedule.sweep_start = hrt_absolute_time();
		}
		break;

	case FIXED_WING:
		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL];
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] = _actuators_mc_in->control[actuator_controls_s::INDEX_PITCH];
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = _actuators_mc_in->control[actuator_controls_s::INDEX_YAW];
		#ifdef SYSIDT
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] = _actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];
		#else
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] = _fw_virtual_att_sp->thrust_body[0];//_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];
		#endif

		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] = -_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL];	// roll elevon
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] = -_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH];	// pitch elevon


	case TRANSITION_TO_FW:
	case TRANSITION_TO_MC:
		/**
		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_YAW] * smooth_fw_start + _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL] * (1.0f - smooth_fw_start);
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH] * smooth_fw_start + _actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * (1.0f - smooth_fw_start);
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW] =
			-_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL] * smooth_fw_start + _actuators_mc_in->control[actuator_controls_s::INDEX_YAW] * (1.0f - smooth_fw_start);
		**/

		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL];
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] = _actuators_mc_in->control[actuator_controls_s::INDEX_PITCH];
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = _actuators_mc_in->control[actuator_controls_s::INDEX_YAW];
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] = math::constrain(_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE], 0.3f, 1.0f);

		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] = -_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL];	// roll elevon
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] = -_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH];	// pitch elevon
	}
}
