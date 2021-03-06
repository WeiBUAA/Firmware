/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file vtol_att_control_params.c
 * Parameters for vtol attitude controller.
 *
 * @author Roman Bapst <roman@px4.io>
 * @author Sander Smeets <sander@droneslab.com>
 */

/**
 * VTOL number of engines
 *
 * @min 0
 * @max 8
 * @increment 1
 * @decimal 0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_MOT_COUNT, 0);

/**
 * Idle speed of VTOL when in multicopter mode
 *
 * @unit us
 * @min 900
 * @max 2000
 * @increment 1
 * @decimal 0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_IDLE_PWM_MC, 900);

/**
 * Permanent stabilization in fw mode
 *
 * If set to one this parameter will cause permanent attitude stabilization in fw mode.
 * This parameter has been introduced for pure convenience sake.
 *
 * @boolean
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_PERM_STAB, 0);

/**
 * VTOL Type (Tailsitter=0, Tiltrotor=1, Standard=2)
 *
 * @value 0 Tailsitter
 * @value 1 Tiltrotor
 * @value 2 Standard
 * @min 0
 * @max 2
 * @decimal 0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_TYPE, 0);

/**
 * Lock elevons in multicopter mode
 *
 * If set to 1 the elevons are locked in multicopter mode
 *
 * @boolean
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_ELEV_MC_LOCK, 1);

/**
 * Choose the channel put in the sweep signal
 *
 * 0: no sweep signal
 * 1: pitch rate
 * 2: roll rate
 * 3: yaw rate
 *
 * @boolean
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_SWEEP_TYPE, 0);

/**
 * Set the amlitute of the sweep signal
 *
 *
 * @boolean
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_SWEEP_AMP, 0.2f);

/*
* The parameters of the vertical velocity controller
* Kp 
*/

PARAM_DEFINE_FLOAT(VT_VZ_CONTROL_KP, 1.0f);

/*
* The parameters of the vertical velocity controller
* Ki 
*/

PARAM_DEFINE_FLOAT(VT_VZ_CONTROL_KI, 0.0f);

/*
* The parameters of the vertical velocity controller
* Kd 
*/

PARAM_DEFINE_FLOAT(VT_VZ_CONTROL_KD, 0.0f);

/*
* The parameters of the vertical velocity command generation
* Define the acceleration time between two stablized vz
*/

PARAM_DEFINE_FLOAT(VT_VZ_ACCTIME, 1.0f);

/*
* The parameters of the vertical velocity command generation
* Define the maintain time for certain vz
*/

PARAM_DEFINE_FLOAT(VT_VZ_KEEPTIME, 2.0f);

/*
* The parameters of the vertical velocity command generation
* Define the start speed of vz;
*/

PARAM_DEFINE_FLOAT(VT_VZ_MINSPEED, 1.0f);

/*
* The parameters of the vertical velocity command generation
* Define the end speed of vz;
*/

PARAM_DEFINE_FLOAT(VT_VZ_MAXSPEED, 3.0f);


/*
* The parameters of the vertical velocity command generation
* Define the speed increasing interval of vz;
*/

PARAM_DEFINE_FLOAT(VT_VZ_INTERVAL,1.0f);

/*
* The parameters of the lateral dist controller
* Kp 
*/


PARAM_DEFINE_FLOAT(VT_Y_DIST_KP, 0.1f);

/*
* The parameters of the lateral velocity controller
* Kp 
*/

PARAM_DEFINE_FLOAT(VT_VY_KP, 0.1f);

/*
* The parameters of the lateral velocity controller
* Ki 
*/

PARAM_DEFINE_FLOAT(VT_VY_KI, 0.01f);

/*
* The parameters of the longitudinal dist controller
* Kp 
*/


PARAM_DEFINE_FLOAT(VT_X_DIST_KP, 0.1f);


/*
* The parameters of the vertical dist controller
* Kp 
*/

PARAM_DEFINE_FLOAT(VT_Z_DIST_KP, 0.5f);

/*
* The parameters of the longitudinal velocity controller
* Kp
*/

PARAM_DEFINE_FLOAT(VT_VX_KP, 0.1f);

/*
* The parameters of the longitudinal velocity controller
* Ki
*/

PARAM_DEFINE_FLOAT(VT_VX_KI, 0.01f);

/*
* The maximum height of thrust test
* 
*/

PARAM_DEFINE_FLOAT(VT_MAX_HEIGHT, 200.00f);

/**
 * Duration of a front transition
 *
 * Time in seconds used for a transition
 *
 * @unit s
 * @min 0.00
 * @max 20.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_SAFE_ALT, 10.0f);

/*
*    For system identification.
*    Define the max aoa of system identification.
*
*/

PARAM_DEFINE_FLOAT(SYSIDT_MAXAOA, 70.0f);

/*
*    For system identification.
*    Define the aoa interval of system identification.
*
*/

PARAM_DEFINE_FLOAT(SYSIDT_INTERVAL, 5.0f);

/*
*    For system identification.
*    Define the min aoa of system identification.
*
*/

PARAM_DEFINE_FLOAT(SYSIDT_MINAOA, 5.0f);

/*
*    For system identification.
*    Define the time to keep aoa of system identification.
*
*/

PARAM_DEFINE_FLOAT(SYSIDT_PITCHTIME, 1.5f);

/*
*    For system identification.
*    Define the trim flight speed of system identification.
*
*/

PARAM_DEFINE_FLOAT(SYSIDT_ACCTIME, 4.0f);

/*
*    For system identification.
*    Define the flight circles of system identification.
*
*/

PARAM_DEFINE_FLOAT(SYSIDT_COUNTER, 14.0f);

/*
*    For system identification.
*    Define the roll angle when turning around of system identification.
*
*/

PARAM_DEFINE_FLOAT(SYSIDT_ROLL, 5.0f);


/**
 * Duration of a front transition
 *
 * Time in seconds used for a transition
 *
 * @unit s
 * @min 0.00
 * @max 20.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(F_TRANS_DUR, 2.5f);

PARAM_DEFINE_INT32(VT_SIDESLIP_EN, 0);

PARAM_DEFINE_FLOAT(VT_SIDESLIP_GAIN, 1.0f);

PARAM_DEFINE_FLOAT(SYS_IDENT_INPUT, 0.05f);

PARAM_DEFINE_INT32(SYS_IDENT_NUM, 0);

/**
 * the target pitch at the end of front trans
 *
 * @unit rad
 */
PARAM_DEFINE_FLOAT(F_TRANS_PIT_SP, 1.45f);

/**
 * the flight time when pitch setpoint get the MAX (level flight) in the front transition stage
 *
 * @unit second
 */
PARAM_DEFINE_FLOAT(F_TRANS_LEVEL_T, 0.5f);

/**
 * Duration of a back transition
 *
 * Time in seconds used for a back transition
 *
 * @unit s
 * @min 0.00
 * @max 20.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_TRANS_DUR, 4.0f);

/**
 * Fixed wing pitch trim
 *
 * This parameter allows to adjust the neutral elevon position in fixed wing mode.
 *
 * @min -1.0
 * @max 1.0
 * @increment 0.01
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FW_PITCH_TRIM, 0.0f);

/**
 * Target throttle value for the transition to fixed wing flight.
 * standard vtol: pusher
 * tailsitter, tiltrotor: main throttle
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(F_TRANS_THR, 0.65f);

/**
 * Target throttle value for the transition to hover flight.
 * standard vtol: pusher
 * tailsitter, tiltrotor: main throttle
 *
 * Note for standard vtol:
 * For ESCs and mixers that support reverse thrust on low PWM values set this to a negative value to apply active breaking
 * For ESCs that support thrust reversal with a control channel please set VT_B_REV_OUT and set this to a positive value to apply active breaking
 *
 * @min -1
 * @max 1
 * @increment 0.01
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_TRANS_THR, 0.5f);

/**
 * Approximate deceleration during back transition
 *
 * The approximate deceleration during a back transition in m/s/s
 * Used to calculate back transition distance in mission mode. A lower value will make the VTOL transition further from the destination waypoint.
 *
 * @unit m/s/s
 * @min 0.00
 * @max 20.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_DEC_MSS, 2.0f);

/**
 * Transition blending airspeed
 *
 * Airspeed at which we can start blending both fw and mc controls. Set to 0 to disable.
 *
 * @unit m/s
 * @min 0.00
 * @max 30.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_ARSP_BLEND, 8.0f);

/**
 * Transition airspeed
 *
 * Airspeed at which we can switch to fw mode
 *
 * @unit m/s
 * @min 0.00
 * @max 30.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_ARSP_TRANS, 13.0f);

/**
 * Front transition timeout
 *
 * Time in seconds after which transition will be cancelled. Disabled if set to 0.
 *
 * @unit s
 * @min 0.00
 * @max 30.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TRANS_TIMEOUT, 15.0f);

/**
 * Front transition minimum time
 *
 * Minimum time in seconds for front transition.
 *
 * @unit s
 * @min 0.0
 * @max 20.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TRANS_MIN_TM, 2.0f);

/**
 * QuadChute Altitude
 *
 * Minimum altitude for fixed wing flight, when in fixed wing the altitude drops below this altitude
 * the vehicle will transition back to MC mode and enter failsafe RTL
 * @min 0.0
 * @max 200.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FW_MIN_ALT, 0.0f);

/**
 * Adaptive QuadChute
 *
 * Maximum negative altitude error for fixed wing flight. If the altitude drops below this value below the altitude setpoint
 * the vehicle will transition back to MC mode and enter failsafe RTL.
 * @min 0.0
 * @max 200.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FW_ALT_ERR, 0.0f);

/**
 * QuadChute Max Pitch
 *
 * Maximum pitch angle before QuadChute engages
 * Above this the vehicle will transition back to MC mode and enter failsafe RTL
 * @min 0
 * @max 180
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_QC_P, 0);

/**
 * QuadChute Max Roll
 *
 * Maximum roll angle before QuadChute engages
 * Above this the vehicle will transition back to MC mode and enter failsafe RTL
 * @min 0
 * @max 180
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_QC_R, 0);

/**
 * Airspeed less front transition time (open loop)
 *
 * The duration of the front transition when there is no airspeed feedback available.
 *
 * @unit seconds
 * @min 1.0
 * @max 30.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_F_TR_OL_TM, 6.0f);

/**
 * The channel number of motors that must be turned off in fixed wing mode.
 *
 * @min 0
 * @max 12345678
 * @increment 1
 * @decimal 0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_MOT_OFFID, 0);

/**
 * Differential thrust in forwards flight.
 *
 * Set to 1 to enable differential thrust in fixed-wing flight.
 *
 * @min 0
 * @max 1
 * @decimal 0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_DIFTHR_EN, 0);

/**
 * Differential thrust scaling factor
 *
 * This factor specifies how the yaw input gets mapped to differential thrust in forwards flight.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FW_DIFTHR_SC, 0.1f);
