/*
 * Copyright (C) 2021 EPFL-REHAssist (Rehabilitation and Assistive Robotics Group).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "haptic_controller.h"
#include "communication.h"
#include "drivers/adc.h"
#include "drivers/incr_encoder.h"
#include "drivers/hall.h"
#include "drivers/callback_timers.h"
#include "lib/utils.h"
#include "torque_regulator.h"
#include "lib/pid.h"
#include "stdio.h"
#include "math.h"
#include "drivers/led.h"

//#include "MatLib.h"


#define DEFAULT_HAPTIC_CONTROLLER_PERIOD 350 // Default control loop period [us].
//#define DEFAULT_HAPTIC_CONTROLLER_PERIOD 10000 // 10ms control loop period [us]. //LAB0

volatile uint32_t  hapt_timestamp; // Time base of the controller, also used to timestamp the samples sent by streaming [us].
volatile float32_t hapt_hallVoltage; // Hall sensor output voltage [V].
volatile float32_t hapt_encoderPaddleAngle; // Paddle angle measured by the incremental encoder [deg].
volatile float32_t hapt_motorTorque; // Motor torque [N.m].

/* LAB0 - BEGIN*/
volatile float32_t hapt_RestAngle; // Rest Angle of the Paddle angle measured by the incremental encoder [deg].
volatile float32_t hapt_springStiffness; // Stiffness of the modeled Spring [N.m/deg].
volatile float32_t hapt_dampingCoeff; // Coefficient of the modeled Dampimg [N.m/deg].
volatile float32_t hapt_encoderPrevPaddleAngle; // save previous Paddle angle [deg].
volatile float32_t hapt_encoderPaddleSpeed; // Paddle angular speed calculated by differentiation of the angle [deg/s].
/* LAB0 - END*/

/* LAB1 - BEGIN*/
#define LUCA
#ifndef YVES
#define HALL_POS_SLOPE 47.3328f	    // for linear relationship between hapt_hallVoltage and hapt_hallPaddleAngle
#define HALL_POS_INTERCEPT -124.2214f    // for linear relationship between hapt_hallVoltage and hapt_hallPaddleAngle
#else
#define HALL_POS_SLOPE 19.5464f	    // for linear relationship between hapt_hallVoltage and hapt_hallPaddleAngle
#define HALL_POS_INTERCEPT -49.7716f    // for linear relationship between hapt_hallVoltage and hapt_hallPaddleAngle
#endif

// HALL
volatile float32_t hapt_hallPaddleAngle; // Paddle angle measured by the hall sensor [deg].
volatile float32_t hapt_hallPrevPaddleAngle; // save previous Paddle angle [deg].
volatile float32_t hapt_hallPrevPrevPaddleAngle; // save 2nd previous Paddle angle [deg].
volatile float32_t hapt_hallPaddlePrevSpeed; // save previous Paddle speed [deg/s].
volatile float32_t hapt_hallPaddlePrevPrevSpeed; // save 2nd previous Paddle speed [deg/s].
volatile float32_t hapt_hallPaddleSpeed_over_1_p; // Paddle angular speed calculated by differentiation of the angle over 1 period [deg/s].
volatile float32_t hapt_hallPaddleSpeed_over_2_p; // Paddle angular speed calculated by differentiation of the angle over 2 periods [deg/s].
volatile float32_t hapt_hallPaddleAccel_over_1_p; // Paddle angular acceleration calculated by differentiation of the speed over 1 period [deg/s^2].
volatile float32_t hapt_hallPaddleAccel_over_2_p; // Paddle angular acceleration calculated by differentiation of the speed over 2 period [deg/s^2].
volatile float32_t hapt_hallPaddleSpeedFilt; // Filtered paddle speed [deg/s].
volatile float32_t hapt_hallPaddleAccelFilt; // Filtered paddle acceleration [deg/s].
// ENCODER
volatile float32_t hapt_encoderPrevPaddleAngle; // save previous Paddle angle [deg].
volatile float32_t hapt_encoderPrevPrevPaddleAngle; // save 2nd previous Paddle angle [deg].
volatile float32_t hapt_encoderPaddlePrevSpeed; // save previous Paddle speed [deg/s].
volatile float32_t hapt_encoderPaddlePrevPrevSpeed; // save 2nd previous Paddle speed [deg/s].
volatile float32_t hapt_encoderPaddleSpeed_over_1_p; // Paddle angular speed calculated by differentiation of the angle over 1 period [deg/s].
volatile float32_t hapt_encoderPaddleSpeed_over_2_p; // Paddle angular speed calculated by differentiation of the angle over 2 periods [deg/s].
volatile float32_t hapt_encoderPaddleAccel_over_1_p; // Paddle angular acceleration calculated by differentiation of the speed over 1 period [deg/s^2].
volatile float32_t hapt_encoderPaddleAccel_over_2_p; // Paddle angular acceleration calculated by differentiation of the speed over 2 period [deg/s^2].
volatile float32_t hapt_encoderPaddleSpeedFilt; // Filtered paddle speed [deg/s].
volatile float32_t hapt_encoderPaddleAccelFilt; // Filtered paddle acceleration [deg/s].

const float32_t filterCutoffFreqSpeed = 100.0f; // Cutoff frequency for the speed filter [Hz].
const float32_t filterCutoffFreqAccel = 10.0f; // Cutoff frequency for the acceleration filter [Hz].

float32_t hapt_LowPassFilter(float32_t previousFilteredValue, float32_t input, float32_t dt, float32_t tau);
/* LAB1 - END*/

/* LAB2 - BEGIN*/
volatile float32_t timing;
volatile float32_t paddleTorque_forFrictionMeasurement; // keep the value of the torque to increase ... it gradually

#define KP_ANGLE_DEFAULT_VAL 0.005f // [Nm/deg].
#define KI_ANGLE_DEFAULT_VAL 0.0012f // [Nm/(deg.s)].
#define KD_ANGLE_DEFAULT_VAL 0.00009f // [Nm/(deg/s)].
#define FF_ANGLE_DEFAULT_VAL 0.0f // [Nm/deg].
#define ANGLE_INTEGRATOR_SAT_DEFAULT_VAL 0.032f // [Nm].

//volatile bool enable_position_regulator;
volatile uint8_t enable_position_regulator;
volatile float32_t hapt_TargetAngle; // Target Angle the position controller should reach [deg].
volatile pid_Pid hapt_anglePid;
volatile uint8_t hapt_PID_uses_encoder;
volatile uint8_t hapt_use_PID;

volatile float32_t hapt_hallPaddleAngleFilt; // Filtered hall paddle angle [deg].

/* LAB2 - END*/

/* LAB 3 - BEGIN */
volatile float32_t paddleTorque; // [N.m]. //LAB0

// Compensation
volatile uint8_t enable_compensation=0;
volatile uint8_t enable_paddle_dry_friction_compensation=1;
volatile uint8_t enable_motor_viscous_friction_compensation=0;
volatile uint8_t enable_gravity_compensation=1;

volatile float32_t hapt_dyn_dry_fric_coeff_pos;
volatile float32_t hapt_dyn_dry_fric_coeff_neg;
volatile float32_t hapt_stat_dry_fric_coeff_pos;
volatile float32_t hapt_stat_dry_fric_coeff_neg;
volatile float32_t hapt_added_viscous_coeff;
volatile float32_t hapt_viscous_fric_coeff;

const float32_t epsilon_speed_noise_encoder = 1.0f;
const float32_t epsilon_speed_noise_hall = 1.0f;

// Virtual Wall
volatile float32_t filterCutoffFreqAngle = 50.0f; // Cutoff frequency for the speed filter [Hz].
volatile float32_t hapt_K = 4.0f; // the virtual spring stiffness [N.m/deg]
volatile float32_t hapt_B = 0.08f; //  the virtual damping factor [N.m/(deg/s)]
static float32_t phi_wall = 15.0f; //  the virtual wall angle [deg]
#define FOAW_MAX_N 20
#define NOISE_MAX 0.2
volatile float32_t foaw_buffer[FOAW_MAX_N];

/* LAB 3 - END */

/* PROJECT BEGINS  */

volatile float32_t hapt_foaw_position;
volatile float32_t hapt_foaw_filtered_position;
volatile float32_t hapt_foaw_speed;
volatile float32_t hapt_foaw_filtered_speed;
volatile float32_t hapt_noise_foaw;

volatile float32_t hapt_levant_filtered_speed;
volatile float32_t hapt_C_levant;
volatile float32_t hapt_alpha_levant;
volatile float32_t hapt_lambda_levant;
volatile float32_t hapt_levant_error;
volatile int8_t hapt_levant_error_sign;
volatile float32_t hapt_levant_u1;
volatile float32_t hapt_levant_x_dot;
volatile float32_t hapt_levant_x;

volatile float32_t kalman_states[3];
volatile float32_t hapt_q_kalman;
volatile float32_t forgetting_factor;
const float32_t R = 0.000048f;

volatile uint8_t speed_selector = 1; // fdm by default

/* PROJECT ENDS   */

void hapt_Update(void);
void add_value_to_buffer(float32_t yk);
float32_t get_FOAWFilter(float32_t dt, uint8_t pos_or_speed);

/**
 * @brief Initializes the haptic controller.
 */
void hapt_Init(void)
{
	hapt_timestamp = 0;
	hapt_motorTorque = 0.0f;
	//    enable_position_regulator = false; // disable at the start // LAB2
	enable_position_regulator = 0; // disable at the start // LAB2
	hapt_PID_uses_encoder = 1; // Use encoder by default // LAB2
	hapt_use_PID = 0; //Use PID by defaut //LAB2

	hapt_C_levant = 50000;
//	hapt_alpha_levant = 55000;
//	hapt_lambda_levant = 223;
	hapt_levant_u1 = 0;
	hapt_levant_x = 0;
	kalman_states[0] = 0;
	kalman_states[1] = 0;
	kalman_states[2] = 0; // LAB specialization
	hapt_q_kalman = 100000;
	forgetting_factor = 1;


	// Make the timers call the update function periodically.
	cbt_SetHapticControllerTimer(hapt_Update, DEFAULT_HAPTIC_CONTROLLER_PERIOD);

	// Share some variables with the computer.
	comm_monitorUint32Func("timestep [us]", cbt_GetHapticControllerPeriod,
			cbt_SetHapticControllerPeriod);
	comm_monitorFloat("motor_torque [N.m]", (float32_t*)&hapt_motorTorque, READWRITE);
	comm_monitorFloat("encoder_paddle_pos [deg]", (float32_t*)&hapt_encoderPaddleAngle, READONLY);
	comm_monitorFloat("hall_voltage [V]", (float32_t*)&hapt_hallVoltage, READONLY);

	/* LAB0 - BEGIN*/
	hapt_RestAngle = 0.0f;
	hapt_springStiffness = 0.008f;
	hapt_dampingCoeff = 0.0f;//0.0005f;
	hapt_encoderPrevPaddleAngle = 0.0f;
	/* comm_monitorFloat("Rest Angle [deg]", (float32_t*)&hapt_RestAngle, READWRITE);
    comm_monitorFloat("Spring Stiffness [N.m/deg]", (float32_t*)&hapt_springStiffness, READWRITE);
    comm_monitorFloat("Damping Coefficient [N.m.s/deg]", (float32_t*)&hapt_dampingCoeff, READWRITE);
    comm_monitorFloat("encoder_paddle_speed [deg/s]", (float32_t*)&hapt_encoderPaddleSpeed, READONLY); //LAB0*/
	/* LAB0 - END*/


	/* LAB1 - BEGIN*/
	// HALL
	hapt_hallPrevPaddleAngle = 0.0f;
	hapt_hallPaddlePrevSpeed = 0.0f;
	hapt_hallPrevPrevPaddleAngle = 0.0f;
	hapt_hallPaddlePrevPrevSpeed = 0.0f;
	comm_monitorFloat("hall_paddle_pos [deg]", (float32_t*)&hapt_hallPaddleAngle, READONLY);
	/*    comm_monitorFloat("hall_paddle_speed_over_1_p [deg/s]", (float32_t*)&hapt_hallPaddleSpeed_over_1_p, READONLY);
    comm_monitorFloat("hall_paddle_speed_over_2_p [deg/s]", (float32_t*)&hapt_hallPaddleSpeed_over_2_p, READONLY);
    comm_monitorFloat("hall_paddle_accel_over_1_p [deg/s^2]", (float32_t*)&hapt_hallPaddleAccel_over_1_p, READONLY);
    comm_monitorFloat("hall_paddle_accel_over_2_p [deg/s^2]", (float32_t*)&hapt_hallPaddleAccel_over_2_p, READONLY);*/
	comm_monitorFloat("hall_paddle_angle_filt [deg]", (float32_t*)&hapt_hallPaddleAngleFilt, READONLY);
	comm_monitorFloat("hall_paddle_speed_filt [deg/s]", (float32_t*)&hapt_hallPaddleSpeedFilt, READONLY);
	comm_monitorFloat("hall_paddle_accel_filt [deg/s^2]", (float32_t*)&hapt_hallPaddleAccelFilt, READONLY);
	// ENCODER
	hapt_encoderPrevPaddleAngle = 0.0f;
	hapt_encoderPaddlePrevSpeed = 0.0f;
	hapt_encoderPrevPrevPaddleAngle = 0.0f;
	hapt_encoderPaddlePrevPrevSpeed = 0.0f;
	    comm_monitorFloat("encoder_paddle_speed_over_1_p [deg/s]", (float32_t*)&hapt_encoderPaddleSpeed_over_1_p, READONLY);
    comm_monitorFloat("encoder_paddle_speed_over_2_p [deg/s]", (float32_t*)&hapt_encoderPaddleSpeed_over_2_p, READONLY);
    comm_monitorFloat("encoder_paddle_accel_over_1_p [deg/s^2]", (float32_t*)&hapt_encoderPaddleAccel_over_1_p, READONLY);
    comm_monitorFloat("encoder_paddle_accel_over_2_p [deg/s^2]", (float32_t*)&hapt_encoderPaddleAccel_over_2_p, READONLY);
    comm_monitorFloat("encoder_paddle_speed_filt [deg/s]", (float32_t*)&hapt_encoderPaddleSpeedFilt, READONLY);
    comm_monitorFloat("encoder_paddle_accel_filt [deg/s^2]", (float32_t*)&hapt_encoderPaddleAccelFilt, READONLY);
/* LAB1 - END*/


/* LAB2 - BEGIN*/
	timing = 0.0f;
	paddleTorque_forFrictionMeasurement = 0.0f; // start with torque = 0

	//    comm_monitorBoolFunc("Position Regulator [0/1]", hapt_Get_enable_position_regulator, hapt_Set_enable_position_regulator);
//	comm_monitorUint8("Position Regulator [0/1]", (uint8_t*)&enable_position_regulator, READWRITE);
//    comm_monitorFloat("Target Angle [deg]", (float32_t*)&hapt_TargetAngle, READWRITE);
//    comm_monitorUint8("hapt_PID_uses_encoder [bool]", (uint8_t*)&hapt_PID_uses_encoder, READWRITE);
//    comm_monitorUint8("hapt_use_PID [bool]", (uint8_t*)&hapt_use_PID, READWRITE);

    // tuning parameters
//    comm_monitorFloatFunc("KP", hapt_GetKp, hapt_SetKp);
//    comm_monitorFloatFunc("KI", hapt_GetKi, hapt_SetKi);
//    comm_monitorFloatFunc("KD", hapt_GetKd, hapt_SetKd);
//    comm_monitorFloatFunc("ARW", hapt_GetARW, hapt_SetARW);

    pid_Init((pid_Pid*)&hapt_anglePid, KP_ANGLE_DEFAULT_VAL, KI_ANGLE_DEFAULT_VAL, KD_ANGLE_DEFAULT_VAL, ANGLE_INTEGRATOR_SAT_DEFAULT_VAL, FF_ANGLE_DEFAULT_VAL);

/* LAB2 - END*/


/* LAB 3 - BEGIN */

	// Compensation

	comm_monitorUint8("Compensation [0/1]", (uint8_t*)&enable_compensation, READWRITE);
	comm_monitorUint8("Paddle Dry Friction Compensation [0/1]", (uint8_t*)&enable_paddle_dry_friction_compensation, READWRITE);
	comm_monitorUint8("Gravity Compensation [0/1]", (uint8_t*)&enable_gravity_compensation, READWRITE);
	comm_monitorUint8("Motor Viscous Friction Compensation [0/1]", (uint8_t*)&enable_motor_viscous_friction_compensation, READWRITE);
	hapt_dyn_dry_fric_coeff_pos = 0.0012; //0.00104
	hapt_dyn_dry_fric_coeff_neg = 0.001; //0.00066
	hapt_stat_dry_fric_coeff_pos = 0.0018;
	hapt_stat_dry_fric_coeff_neg = 0.0015;

	hapt_added_viscous_coeff = 0.0; //0.00003
	comm_monitorFloat("Dynamic Dry Friction Coeff +", (float32_t*)&hapt_dyn_dry_fric_coeff_pos, READWRITE);
	comm_monitorFloat("Dynamic Dry Friction Coeff -", (float32_t*)&hapt_dyn_dry_fric_coeff_neg, READWRITE);
	comm_monitorFloat("Static Dry Friction Coeff +", (float32_t*)&hapt_stat_dry_fric_coeff_pos, READWRITE);
	comm_monitorFloat("Static Dry Friction Coeff -", (float32_t*)&hapt_stat_dry_fric_coeff_neg, READWRITE);
	comm_monitorFloat("Add Viscous Friction", (float32_t*)&hapt_added_viscous_coeff, READWRITE);

	hapt_viscous_fric_coeff = 0.00000079;
	comm_monitorFloat("Viscous Friction Coeff", (float32_t*)&hapt_viscous_fric_coeff, READWRITE);



	// Virtual Wall
	comm_monitorFloat("filterCutoffFreqAngle [Hz]", (float32_t*)&filterCutoffFreqAngle, READWRITE);
	comm_monitorFloat("hapt_K  [N.m/deg]", (float32_t*)&hapt_K, READWRITE);
	comm_monitorFloat("hapt_B  [N.m/(deg/s)]", (float32_t*)&hapt_B, READWRITE);
	comm_monitorUint8("speed_selector [fdm 1, foaw 2, levant 3, kalman 4]", (uint8_t*)&speed_selector, READWRITE);
/* LAB 3 - END */

/* Specialization Project - BEGIN */
	comm_monitorFloat("hapt_foaw_position [deg]", (float32_t*)&hapt_foaw_position, READONLY);
	comm_monitorFloat("hapt_foaw_filtered_position [deg]", (float32_t*)&hapt_foaw_filtered_position, READONLY);
	comm_monitorFloat("hapt_foaw_speed [deg/s]", (float32_t*)&hapt_foaw_speed, READONLY);
	comm_monitorFloat("hapt_foaw_filtered_speed [deg/s]", (float32_t*)&hapt_foaw_filtered_speed, READONLY);
	comm_monitorFloat("hapt_noise_foaw [deg]", (float32_t*)&hapt_noise_foaw, READWRITE);

	comm_monitorFloat("levant position [deg]", (float32_t*)&hapt_levant_x, READONLY);
	comm_monitorFloat("levant speed [deg/s]", (float32_t*)&hapt_levant_x_dot, READONLY);
	comm_monitorFloat("hapt_levant_filtered_speed [deg/s]", (float32_t*)&hapt_levant_filtered_speed, READONLY);
	comm_monitorFloat("C (levant) [-]", (float32_t*)&hapt_C_levant, READWRITE);
	comm_monitorFloat("alpha (levant) [-]", (float32_t*)&hapt_alpha_levant, READWRITE);
	comm_monitorFloat("lambda (levant) [-]", (float32_t*)&hapt_lambda_levant, READWRITE);

	comm_monitorFloat("kalman position [deg]", (float32_t*)&kalman_states[0], READONLY);
	comm_monitorFloat("kalman speed [deg/s]", (float32_t*)&kalman_states[1], READONLY);
	comm_monitorFloat("q (kalman) [-]", (float32_t*)&hapt_q_kalman, READWRITE);
	comm_monitorFloat("forgetting factor (kalman) [-]", (float32_t*)&forgetting_factor, READWRITE);
/* Specialization Project - END */

}

/**
 * @brief Updates the haptic controller state.
 */
void hapt_Update()
{
	float32_t motorShaftAngle; // [deg].

	// Compute the dt (uncomment if you need it).
	float32_t dt = ((float32_t)cbt_GetHapticControllerPeriod()) / 1000000.0f; // [s].

	// Increment the timestamp.
	hapt_timestamp += cbt_GetHapticControllerPeriod();

	// Get the Hall sensor voltage.
	hapt_hallVoltage = hall_GetVoltage();

	// Get the encoder position.
	motorShaftAngle = enc_GetPosition();
	hapt_encoderPaddleAngle = motorShaftAngle / REDUCTION_RATIO;
	/*
    // Compute the motor torque, and apply it.
//    hapt_motorTorque = 0.0f;
//    torq_SetTorque(hapt_motorTorque);

/* LAB0 - BEGIN */


/* LAB1 - BEGIN*/

	// HALL
	hapt_hallPaddleAngle = HALL_POS_SLOPE * hapt_hallVoltage + HALL_POS_INTERCEPT;
	hapt_hallPaddleSpeed_over_1_p = (hapt_hallPaddleAngle - hapt_hallPrevPaddleAngle) / dt;
	hapt_hallPaddleSpeed_over_2_p = (hapt_hallPaddleAngle - hapt_hallPrevPrevPaddleAngle) / (2.0f*dt);
	hapt_hallPaddleAccel_over_1_p = (hapt_hallPaddleSpeed_over_1_p - hapt_hallPaddlePrevSpeed)/dt;
	hapt_hallPaddleAccel_over_2_p = (hapt_hallPaddleSpeed_over_1_p - hapt_hallPaddlePrevPrevSpeed)/(2.0f*dt);
	// Filter the speed and acceleration
	hapt_hallPaddleAngleFilt = hapt_LowPassFilter(hapt_hallPaddleAngleFilt, hapt_hallPrevPaddleAngle, dt, filterCutoffFreqSpeed);
	hapt_hallPaddleSpeedFilt = hapt_LowPassFilter(hapt_hallPaddleSpeedFilt, hapt_hallPaddleSpeed_over_1_p, dt, filterCutoffFreqSpeed);
	hapt_hallPaddleAccelFilt = hapt_LowPassFilter(hapt_hallPaddleAccelFilt, hapt_hallPaddleAccel_over_1_p, dt, filterCutoffFreqAccel);
	// save previous angles and speeds
	hapt_hallPrevPrevPaddleAngle = hapt_hallPrevPaddleAngle; //save 2nd previous angle.
	hapt_hallPrevPaddleAngle = hapt_hallPaddleAngle; //save previous angle
	hapt_hallPaddlePrevPrevSpeed = hapt_hallPaddlePrevSpeed; //save 2nd previous speed.
	hapt_hallPaddlePrevSpeed = hapt_hallPaddleSpeed_over_1_p; //save previous speed

	// ENCODER
	hapt_encoderPaddleSpeed_over_1_p = (hapt_encoderPaddleAngle - hapt_encoderPrevPaddleAngle) / dt;
	hapt_encoderPaddleSpeed_over_2_p = (hapt_encoderPaddleAngle - hapt_encoderPrevPrevPaddleAngle) / (2.0f*dt);
	hapt_encoderPaddleAccel_over_1_p = (hapt_encoderPaddleSpeed_over_1_p - hapt_encoderPaddlePrevSpeed)/dt;
	hapt_encoderPaddleAccel_over_2_p = (hapt_encoderPaddleSpeed_over_1_p - hapt_encoderPaddlePrevPrevSpeed)/(2.0f*dt);
	// save previous angles and speeds
	hapt_encoderPrevPrevPaddleAngle = hapt_encoderPrevPaddleAngle; //save 2nd previous angle.
	hapt_encoderPrevPaddleAngle = hapt_encoderPaddleAngle; //save previous angle
	hapt_encoderPaddlePrevPrevSpeed = hapt_encoderPaddlePrevSpeed; //save 2nd previous speed.
	hapt_encoderPaddlePrevSpeed = hapt_encoderPaddleSpeed_over_1_p; //save previous speed
	// Filter the speed and acceleration.
//	hapt_encoderPaddleSpeedFilt = hapt_LowPassFilter(hapt_encoderPaddleSpeedFilt, hapt_encoderPaddleSpeed_over_1_p, dt, filterCutoffFreqSpeed);
	hapt_encoderPaddleAccelFilt = hapt_LowPassFilter(hapt_encoderPaddleAccelFilt, hapt_encoderPaddleAccel_over_1_p, dt, filterCutoffFreqAccel);
	/* LAB1 - END*/


/* Specialization Project - BEGIN */
	hapt_encoderPaddleSpeedFilt = hapt_LowPassFilter(hapt_encoderPaddleSpeedFilt, hapt_encoderPaddleSpeed_over_1_p, dt, filterCutoffFreqAngle);
// FOAW
	add_value_to_buffer(hapt_encoderPaddleAngle);
	hapt_foaw_position = get_FOAWFilter(dt,1);
	hapt_foaw_speed = get_FOAWFilter(dt,0);
	hapt_foaw_filtered_speed = hapt_LowPassFilter(hapt_foaw_filtered_speed,hapt_foaw_speed,dt,filterCutoffFreqAngle);
// LEVANT
	hapt_alpha_levant = 1.1 * hapt_C_levant;
	hapt_lambda_levant = sqrt(hapt_C_levant);

	hapt_levant_error = hapt_levant_x - hapt_encoderPaddleAngle;
	hapt_levant_error_sign = hapt_levant_error/fabs(hapt_levant_error);
	hapt_levant_u1 -= hapt_alpha_levant * hapt_levant_error_sign * dt;
	hapt_levant_x_dot = hapt_levant_u1 - hapt_lambda_levant * sqrt(fabs(hapt_levant_error)) * hapt_levant_error_sign;
	hapt_levant_x += hapt_levant_x_dot * dt;
	hapt_levant_filtered_speed = hapt_LowPassFilter(hapt_levant_filtered_speed,hapt_levant_x_dot,dt,filterCutoffFreqAngle);
// KALMAN
	// initialization
	float32_t A[3][3] = {{1, dt, dt*dt/2},{0, 1, dt},{0, 0, 1}}; // 1x1
	float32_t Q[3][3] = {{0, 0, 0},{0, 0, 0},{0, 0, hapt_q_kalman}}; // adjust 1
	static float32_t H[3] = {1, 0, 0};  // 1x3
	static float32_t P[3][3] = {{R, 0, 0},{0, 2*R/(dt*dt), 0},{0, 0, 2*R/(dt*dt*dt*dt)}}; // 3x3

	// prediction
	kalman_states[0] = A[0][0]*kalman_states[0] + A[0][1]*kalman_states[1] + A[0][2]*kalman_states[2];
	kalman_states[1] = A[1][0]*kalman_states[0] + A[1][1]*kalman_states[1] + A[1][2]*kalman_states[2];
	kalman_states[2] = A[2][0]*kalman_states[0] + A[2][1]*kalman_states[1] + A[2][2]*kalman_states[2];
	P[0][0] = forgetting_factor*(Q[0][0]+A[0][0]*(A[0][0]*P[0][0]+A[0][1]*P[1][0]+A[0][2]*P[2][0])+A[0][1]*(A[0][0]*P[0][1]+A[0][1]*P[1][1]+A[0][2]*P[2][1])+A[0][2]*(A[0][0]*P[0][2]+A[0][1]*P[1][2]+A[0][2]*P[2][2]));
    P[0][1] = forgetting_factor*(Q[0][1]+A[1][0]*(A[0][0]*P[0][0]+A[0][1]*P[1][0]+A[0][2]*P[2][0])+A[1][1]*(A[0][0]*P[0][1]+A[0][1]*P[1][1]+A[0][2]*P[2][1])+A[1][2]*(A[0][0]*P[0][2]+A[0][1]*P[1][2]+A[0][2]*P[2][2]));
    P[0][2] = forgetting_factor*(Q[0][2]+A[2][0]*(A[0][0]*P[0][0]+A[0][1]*P[1][0]+A[0][2]*P[2][0])+A[2][1]*(A[0][0]*P[0][1]+A[0][1]*P[1][1]+A[0][2]*P[2][1])+A[2][2]*(A[0][0]*P[0][2]+A[0][1]*P[1][2]+A[0][2]*P[2][2]));
    P[1][0] = forgetting_factor*(Q[1][0]+A[0][0]*(A[1][0]*P[0][0]+A[1][1]*P[1][0]+A[1][2]*P[2][0])+A[0][1]*(A[1][0]*P[0][1]+A[1][1]*P[1][1]+A[1][2]*P[2][1])+A[0][2]*(A[1][0]*P[0][2]+A[1][1]*P[1][2]+A[1][2]*P[2][2]));
    P[1][0] = forgetting_factor*(Q[1][0]+A[0][0]*(A[1][0]*P[0][0]+A[1][1]*P[1][0]+A[1][2]*P[2][0])+A[0][1]*(A[1][0]*P[0][1]+A[1][1]*P[1][1]+A[1][2]*P[2][1])+A[0][2]*(A[1][0]*P[0][2]+A[1][1]*P[1][2]+A[1][2]*P[2][2]));
    P[1][1] = forgetting_factor*(Q[1][1]+A[1][0]*(A[1][0]*P[0][0]+A[1][1]*P[1][0]+A[1][2]*P[2][0])+A[1][1]*(A[1][0]*P[0][1]+A[1][1]*P[1][1]+A[1][2]*P[2][1])+A[1][2]*(A[1][0]*P[0][2]+A[1][1]*P[1][2]+A[1][2]*P[2][2]));
    P[1][2] = forgetting_factor*(Q[1][2]+A[2][0]*(A[1][0]*P[0][0]+A[1][1]*P[1][0]+A[1][2]*P[2][0])+A[2][1]*(A[1][0]*P[0][1]+A[1][1]*P[1][1]+A[1][2]*P[2][1])+A[2][2]*(A[1][0]*P[0][2]+A[1][1]*P[1][2]+A[1][2]*P[2][2]));
    P[2][0] = forgetting_factor*(Q[2][0]+A[0][0]*(A[2][0]*P[0][0]+A[2][1]*P[1][0]+A[2][2]*P[2][0])+A[0][1]*(A[2][0]*P[0][1]+A[2][1]*P[1][1]+A[2][2]*P[2][1])+A[0][2]*(A[2][0]*P[0][2]+A[2][1]*P[1][2]+A[2][2]*P[2][2]));
    P[2][1] = forgetting_factor*(Q[2][1]+A[1][0]*(A[2][0]*P[0][0]+A[2][1]*P[1][0]+A[2][2]*P[2][0])+A[1][1]*(A[2][0]*P[0][1]+A[2][1]*P[1][1]+A[2][2]*P[2][1])+A[1][2]*(A[2][0]*P[0][2]+A[2][1]*P[1][2]+A[2][2]*P[2][2]));
    P[2][2] = forgetting_factor*(Q[2][2]+A[2][0]*(A[2][0]*P[0][0]+A[2][1]*P[1][0]+A[2][2]*P[2][0])+A[2][1]*(A[2][0]*P[0][1]+A[2][1]*P[1][1]+A[2][2]*P[2][1])+A[2][2]*(A[2][0]*P[0][2]+A[2][1]*P[1][2]+A[2][2]*P[2][2]));

    // gain
    float32_t K_denom;
    K_denom = R+H[0]*(H[0]*P[0][0]+H[1]*P[1][0]+H[2]*P[2][0])+H[1]*(H[0]*P[0][1]+H[1]*P[1][1]+H[2]*P[2][1])+H[2]*(H[0]*P[0][2]+H[1]*P[1][2]+H[2]*P[2][2]);
    float32_t K[3];
    K[0] = (H[0]*P[0][0]+H[1]*P[0][1]+H[2]*P[0][2])/K_denom;
    K[1] = (H[0]*P[1][0]+H[1]*P[1][1]+H[2]*P[1][2])/K_denom;
    K[2] = (H[0]*P[2][0]+H[1]*P[2][1]+H[2]*P[2][2])/K_denom;

    // update
    kalman_states[0] = kalman_states[0]-K[0]*(-hapt_encoderPaddleAngle+H[0]*kalman_states[0]+H[1]*kalman_states[1]+H[2]*kalman_states[2]);
    kalman_states[1] = kalman_states[1]-K[1]*(-hapt_encoderPaddleAngle+H[0]*kalman_states[0]+H[1]*kalman_states[1]+H[2]*kalman_states[2]);
    kalman_states[2] = kalman_states[2]-K[2]*(-hapt_encoderPaddleAngle+H[0]*kalman_states[0]+H[1]*kalman_states[1]+H[2]*kalman_states[2]);
    P[0][0] = P[0][0]-H[0]*K[0]*P[0][0]-H[1]*K[0]*P[1][0]-H[2]*K[0]*P[2][0];
    P[0][1] = P[0][1]-H[0]*K[0]*P[0][1]-H[1]*K[0]*P[1][1]-H[2]*K[0]*P[2][1];
    P[0][2] = P[0][2]-H[0]*K[0]*P[0][2]-H[1]*K[0]*P[1][2]-H[2]*K[0]*P[2][2];
    P[1][0] = P[1][0]-H[0]*K[1]*P[0][0]-H[1]*K[1]*P[1][0]-H[2]*K[1]*P[2][0];
    P[1][1] = P[1][1]-H[0]*K[1]*P[0][1]-H[1]*K[1]*P[1][1]-H[2]*K[1]*P[2][1];
    P[1][2] = P[1][2]-H[0]*K[1]*P[0][2]-H[1]*K[1]*P[1][2]-H[2]*K[1]*P[2][2];
    P[2][0] = P[2][0]-H[0]*K[2]*P[0][0]-H[1]*K[2]*P[1][0]-H[2]*K[2]*P[2][0];
    P[2][1] = P[2][1]-H[0]*K[2]*P[0][1]-H[1]*K[2]*P[1][1]-H[2]*K[2]*P[2][1];
    P[2][2] = P[2][2]-H[0]*K[2]*P[0][2]-H[1]*K[2]*P[1][2]-H[2]*K[2]*P[2][2];

/* Specialization Project - BEGIN */

/* LAB 3 - BEGIN */

	// Compensation
	paddleTorque = hapt_motorTorque; // At beginning of cycle
	float32_t position;
	float32_t speed;

	switch (speed_selector){
	    case 1:
	        position = hapt_encoderPaddleAngle;
	        speed = hapt_encoderPaddleSpeedFilt;
	        break;
	    case 2:
	        position = hapt_foaw_position;
	        speed = hapt_foaw_filtered_speed;
	        break;
	    case 3:
		    position = hapt_levant_x;
		    speed = hapt_levant_filtered_speed;
		    break;
	    case 4:
	    	position = kalman_states[0];
	    	speed = kalman_states[1];
		    break;
	}

	if(enable_compensation){
		// paddle dry friction compensation
		if(enable_paddle_dry_friction_compensation){

			if(speed > -epsilon_speed_noise_encoder){
				paddleTorque = hapt_dyn_dry_fric_coeff_pos * REDUCTION_RATIO;                     // dynamic dry friction positive direction
			} else if (speed < epsilon_speed_noise_encoder){
				paddleTorque = -hapt_dyn_dry_fric_coeff_neg * REDUCTION_RATIO;                    // dynamic dry friction negative direction
			} else {
				//
				if (position > 0.1)
					paddleTorque = -hapt_stat_dry_fric_coeff_neg * REDUCTION_RATIO; // static dry friction
				else if (position < -0.1)
					paddleTorque = hapt_stat_dry_fric_coeff_pos * REDUCTION_RATIO; // static dry friction
				else
					paddleTorque = 0.0;
			}

			// add viscous friction to see damped oscillations when only compensating dry friction
			//		    	if((enable_motor_viscous_friction_compensation || enable_gravity_compensation) == 0)
			//		    		paddleTorque -= hapt_added_viscous_coeff * hapt_encoderPaddleSpeedFilt;

		} else {
			paddleTorque = 0.0;
		}

		// motor viscous friction compensation
		if(enable_motor_viscous_friction_compensation){
				paddleTorque += hapt_viscous_fric_coeff * speed;
		}

		// gravity_compensation
		if(enable_gravity_compensation){
				paddleTorque += 0.048 * 9.81 * 0.0251 * sin(position * M_PI/180);
		}
	}

	// Virtual wall part using encoder
	if (fabs(position)>fabs(phi_wall)){
		if (position> 0){ // No sign function wtf
			paddleTorque += -hapt_K*(position-phi_wall) -hapt_B*speed;
		}
		else{
			paddleTorque += -hapt_K*(position+phi_wall) -hapt_B*speed;
		}
	}
/* LAB 3 - END */

	hapt_motorTorque = paddleTorque / REDUCTION_RATIO;
    if (!enable_position_regulator){
		torq_SetTorque(hapt_motorTorque);
    }
}


/*************************************************************************************************/
/*                                           FUNCTIONS                                           */
/*************************************************************************************************/

/**
 * @brief Filters a signal with a first-order low-pass filter.
 * @param previousFilteredValue the previous filtered value.
 * @param input the filter input (the current sample of the signal to filter).
 * @param dt the time elapsed since the last call of this function [s].
 * @param cutoffFreq the cutoff frequency of the filter [Hz].
 * @return the new output of the filter.
 */
float32_t hapt_LowPassFilter(float32_t previousFilteredValue, float32_t input, float32_t dt, float32_t cutoffFreq)
{
	float32_t tau = 1.0f / (2.0f * PI * cutoffFreq); // Rise time (time to reach 63% of the steady-state value).
	float32_t alpha = dt / (tau+dt); // Smoothing factor.
	return alpha * input + (1.0f - alpha) * previousFilteredValue;
}

void add_value_to_buffer(float32_t yk){
	// Round robin buffer
	for(uint8_t i=1; i < FOAW_MAX_N; i++){
		foaw_buffer[i-1] = foaw_buffer[i];
	}
	foaw_buffer[FOAW_MAX_N-1] = yk;
}

float32_t get_FOAWFilter(float32_t dt, uint8_t pos_or_speed)
{
	uint8_t n = 1;
	float32_t an = 0;
	float32_t bn = 0;
	uint8_t flag = 1;

	while (n < FOAW_MAX_N && flag){
		float32_t yk_n = foaw_buffer[FOAW_MAX_N-1-n];
		float32_t sum1 = 0;
		float32_t sum2 = 0;
		for(uint8_t i=0; i <= n; i++){
			float32_t yk_i = foaw_buffer[FOAW_MAX_N-1-i];
			sum1 += yk_i;
			sum2 += yk_i*i;
		}
		//bn = (yk-yk_n)/(n*dt);  end-fit-FOAW
		bn = 6*(n*sum1-2*sum2)/(dt*n*(n+1)*(n+2)); // Best-fit-FOAW
		an = yk_n;
		for(uint8_t i=1 ;i <= n; i++){
			float32_t yk_i = foaw_buffer[FOAW_MAX_N-1-i];
			float32_t L_yk_i = an-i*bn/dt;
			if((fabs(yk_i-L_yk_i)) > fabs(hapt_noise_foaw)){
				flag = 0;
			}
		}
		n++;
	}

	for(int j=0;j<4;j++){
		led_Set(j,0.0);
	}

	if ((n/2)<4){
		led_Set(n/2,1.0);
	}

	float32_t prediction_position = an+bn*dt*n;
	float32_t prediction_speed = bn;

	if(pos_or_speed){
		return prediction_position;
	}
	else{
		return prediction_speed;
	}
}

void hapt_Set_enable_position_regulator(uint8_t on)
{
	enable_position_regulator = on;
}

void hapt_SetKp(float32_t Kp)
{
	hapt_anglePid.kp = Kp;
}

void hapt_SetKi(float32_t Ki)
{
	hapt_anglePid.ki = Ki;
}

void hapt_SetKd(float32_t Kd)
{
	hapt_anglePid.kd = Kd;
}

void hapt_SetARW(float32_t ARW)
{
	hapt_anglePid.arw = ARW;
}

float32_t hapt_Get_enable_position_regulator()
{
	return enable_position_regulator;
}

float32_t hapt_GetKp()
{
	return hapt_anglePid.kp;
}

float32_t hapt_GetKi()
{
	return hapt_anglePid.ki;
}

float32_t hapt_GetKd()
{
	return hapt_anglePid.kd;
}

float32_t hapt_GetARW()
{
	return hapt_anglePid.arw;
}

