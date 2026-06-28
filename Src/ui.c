#include "main.h"
#include "edukit_system.h"
#include "hardware.h"
#include "ui.h"
#include "remote_controller.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* UI-private mode selection state (moved from main.c) -------------------- */
static char config_message[16];
static int config_command;
static int mode_index = 1;
static int char_mode_select;
static int mode_interactive;
static int step_size;
static int mode_1, mode_2;
static int mode_8, mode_11, mode_13, mode_15;
static int mode_quit;

static char mode_string_stop[UART_RX_BUFFER_SIZE];
static char mode_string_mode_1[UART_RX_BUFFER_SIZE];
static char mode_string_mode_2[UART_RX_BUFFER_SIZE];
static char mode_string_mode_8[UART_RX_BUFFER_SIZE];
static char mode_string_inc_accel[UART_RX_BUFFER_SIZE];
static char mode_string_dec_accel[UART_RX_BUFFER_SIZE];
static char mode_string_inc_amp[UART_RX_BUFFER_SIZE];
static char mode_string_dec_amp[UART_RX_BUFFER_SIZE];
static char mode_string_mode_test[UART_RX_BUFFER_SIZE];
static char mode_string_mode_control[UART_RX_BUFFER_SIZE];
static char mode_string_mode_motor_characterization_mode[UART_RX_BUFFER_SIZE];
static char mode_string_dec_pend_p[UART_RX_BUFFER_SIZE];
static char mode_string_inc_pend_p[UART_RX_BUFFER_SIZE];
static char mode_string_dec_pend_i[UART_RX_BUFFER_SIZE];
static char mode_string_inc_pend_i[UART_RX_BUFFER_SIZE];
static char mode_string_dec_pend_d[UART_RX_BUFFER_SIZE];
static char mode_string_inc_pend_d[UART_RX_BUFFER_SIZE];
static char mode_string_dec_rotor_p[UART_RX_BUFFER_SIZE];
static char mode_string_inc_rotor_p[UART_RX_BUFFER_SIZE];
static char mode_string_dec_rotor_i[UART_RX_BUFFER_SIZE];
static char mode_string_inc_rotor_i[UART_RX_BUFFER_SIZE];
static char mode_string_dec_rotor_d[UART_RX_BUFFER_SIZE];
static char mode_string_inc_rotor_d[UART_RX_BUFFER_SIZE];
static char mode_string_dec_torq_c[UART_RX_BUFFER_SIZE];
static char mode_string_inc_torq_c[UART_RX_BUFFER_SIZE];
static char mode_string_dec_max_s[UART_RX_BUFFER_SIZE];
static char mode_string_inc_max_s[UART_RX_BUFFER_SIZE];
static char mode_string_dec_min_s[UART_RX_BUFFER_SIZE];
static char mode_string_inc_min_s[UART_RX_BUFFER_SIZE];
static char mode_string_dec_max_a[UART_RX_BUFFER_SIZE];
static char mode_string_inc_max_a[UART_RX_BUFFER_SIZE];
static char mode_string_dec_max_d[UART_RX_BUFFER_SIZE];
static char mode_string_inc_max_d[UART_RX_BUFFER_SIZE];
static char mode_string_enable_step[UART_RX_BUFFER_SIZE];
static char mode_string_disable_step[UART_RX_BUFFER_SIZE];
static char mode_string_enable_pendulum_impulse[UART_RX_BUFFER_SIZE];
static char mode_string_disable_pendulum_impulse[UART_RX_BUFFER_SIZE];
static char mode_string_enable_load_dist[UART_RX_BUFFER_SIZE];
static char mode_string_disable_load_dist[UART_RX_BUFFER_SIZE];
static char mode_string_enable_noise_rej_step[UART_RX_BUFFER_SIZE];
static char mode_string_disable_noise_rej_step[UART_RX_BUFFER_SIZE];
static char mode_string_disable_sensitivity_fnc_step[UART_RX_BUFFER_SIZE];
static char mode_string_enable_sensitivity_fnc_step[UART_RX_BUFFER_SIZE];
static char mode_string_inc_step_size[UART_RX_BUFFER_SIZE];
static char mode_string_dec_step_size[UART_RX_BUFFER_SIZE];
static char mode_string_select_mode_5[UART_RX_BUFFER_SIZE];
static char mode_string_enable_high_speed_sampling[UART_RX_BUFFER_SIZE];
static char mode_string_disable_high_speed_sampling[UART_RX_BUFFER_SIZE];
static char mode_string_enable_speed_prescale[UART_RX_BUFFER_SIZE];
static char mode_string_disable_speed_prescale[UART_RX_BUFFER_SIZE];
static char mode_string_disable_speed_governor[UART_RX_BUFFER_SIZE];
static char mode_string_enable_speed_governor[UART_RX_BUFFER_SIZE];
static char mode_string_reset_system[UART_RX_BUFFER_SIZE];

/* Mode command state + session response flags (moved from main.c) */
static int mode_index_command;
static int enable_angle_cal_resp;
static int enable_swing_up_resp;
static float rotor_position_command_deg;

/* Rotor high-speed test / sysid / motor characterization variables (moved from main.c) */
static int rotor_test_speed_min, rotor_test_speed_max;
static int rotor_test_acceleration_max, swing_deceleration_max;
static uint16_t current_speed;
static int enable_encoder_test;
static int motor_state;
static float rotor_chirp_amplitude;
static int rotor_chirp_step_period;
static float rotor_chirp_start_freq;
static float rotor_chirp_end_freq;
static float rotor_chirp_period;
static float chirp_time;
static float rotor_chirp_frequency;
static uint32_t RxBuffer_ReadIdx;
static uint32_t RxBuffer_WriteIdx;
static uint32_t readBytes;
/* ----------------------------------------------------------------------- */

static void read_float(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, float *float_return) {

	int k;

	while (1) {
		*RxBuffer_WriteIdx = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
		*readBytes = Extract_Msg(RxBuffer, *RxBuffer_ReadIdx, *RxBuffer_WriteIdx, UART_RX_BUFFER_SIZE, &Msg);

		if (*readBytes)
		{
			*RxBuffer_ReadIdx = (*RxBuffer_ReadIdx + *readBytes)
											% UART_RX_BUFFER_SIZE;
			*float_return = atof((char*) Msg.Data);
			for (k = 0; k < SERIAL_MSG_MAXLEN; k++) {
				Msg.Data[k] = 0;
			}
			*readBytes = 0;
			break;
		}
		HAL_Delay(100);
	}
}

static void read_int(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, int * int_return) {

	int k;

	while (1) {
		*RxBuffer_WriteIdx = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
		*readBytes = Extract_Msg(RxBuffer, *RxBuffer_ReadIdx, *RxBuffer_WriteIdx, UART_RX_BUFFER_SIZE, &Msg);

		if (*readBytes)
		{
			*RxBuffer_ReadIdx = (*RxBuffer_ReadIdx + *readBytes)
											% UART_RX_BUFFER_SIZE;

			*int_return = atoi((char*)(Msg.Data));
			for (k = 0; k < SERIAL_MSG_MAXLEN; k++) {
				Msg.Data[k] = 0;
			}
			*readBytes = 0;
			break;
		}
		HAL_Delay(100);
	}
}


int mode_index_identification(AppControlContext *ctx, char * user_config_input, int config_command_control,
		float *adjust_increment, arm_pid_instance_a_f32 *PID_Pend,
		arm_pid_instance_a_f32 *PID_Rotor){

	if (strcmp(user_config_input, mode_string_inc_pend_p) == 0){
		PID_Pend->Kp = PID_Pend->Kp + *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_pend_p) == 0) {
		PID_Pend->Kp = PID_Pend->Kp - *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_pend_d) == 0) {
		PID_Pend->Kd = PID_Pend->Kd + *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_pend_d) == 0) {
		PID_Pend->Kd = PID_Pend->Kd - *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_pend_i) == 0) {
		PID_Pend->Ki = PID_Pend->Ki + *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_pend_i) == 0) {
		PID_Pend->Ki = PID_Pend->Ki - *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_rotor_p) == 0){
		PID_Rotor->Kp = PID_Rotor->Kp + *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_rotor_p) == 0) {
		PID_Rotor->Kp = PID_Rotor->Kp - *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_rotor_d) == 0) {
		PID_Rotor->Kd = PID_Rotor->Kd + *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_rotor_d) == 0) {
		PID_Rotor->Kd = PID_Rotor->Kd - *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_rotor_i) == 0) {
		PID_Rotor->Ki = PID_Rotor->Ki + *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_rotor_i) == 0) {
		PID_Rotor->Ki = PID_Rotor->Ki - *adjust_increment;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_torq_c) == 0) {
		ctx->torq_current_val = L6474_GetAnalogValue(0, L6474_TVAL);
		ctx->torq_current_val = ctx->torq_current_val - *adjust_increment;
		if (ctx->torq_current_val < 200){ ctx->torq_current_val = 200; }
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetAnalogValue(0, L6474_TVAL, ctx->torq_current_val);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_torq_c) == 0) {
		ctx->torq_current_val = L6474_GetAnalogValue(0, L6474_TVAL);
		ctx->torq_current_val = ctx->torq_current_val + *adjust_increment;
		if (ctx->torq_current_val > MAX_TORQUE_CONFIG){ ctx->torq_current_val = MAX_TORQUE_CONFIG; }
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetAnalogValue(0, L6474_TVAL, ctx->torq_current_val);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_max_s) == 0) {
		ctx->max_speed = L6474_GetMaxSpeed(0);
		ctx->max_speed = ctx->max_speed - *adjust_increment;
		if (ctx->max_speed < 100){ ctx->max_speed = 100; }
		if (ctx->max_speed < ctx->min_speed){ ctx->max_speed = ctx->min_speed;}
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetMaxSpeed(0, ctx->max_speed);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_max_s) == 0) {
		ctx->max_speed = L6474_GetMaxSpeed(0);
		ctx->max_speed = ctx->max_speed + *adjust_increment;
		if (ctx->max_speed > 1000){ ctx->max_speed = 1000; }
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetMaxSpeed(0, ctx->max_speed);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_min_s) == 0) {
		ctx->min_speed = L6474_GetMinSpeed(0);
		ctx->min_speed = ctx->min_speed - *adjust_increment;
		if (ctx->min_speed < 100){ ctx->min_speed = 100; }
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetMinSpeed(0, ctx->min_speed);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_min_s) == 0) {
		ctx->min_speed = L6474_GetMinSpeed(0);
		ctx->min_speed = ctx->min_speed + *adjust_increment;
		if (ctx->min_speed > 1000){ ctx->min_speed = 1000; }
		if (ctx->min_speed > ctx->max_speed){ ctx->min_speed = ctx->max_speed;}
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetMinSpeed(0, ctx->min_speed);
		config_command = 1;
		mode_index_command = -1;
	} else if (strcmp(user_config_input, mode_string_dec_max_a) == 0) {
		ctx->max_accel = L6474_GetAcceleration(0);
		ctx->max_accel = ctx->max_accel - *adjust_increment;
		if (ctx->max_accel <  0){ ctx->max_accel = 0;}
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetAcceleration(0, ctx->max_accel);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_max_a) == 0) {
		ctx->max_accel = L6474_GetAcceleration(0);
		ctx->max_accel = ctx->max_accel + *adjust_increment;
		if (ctx->max_accel >  10000){ ctx->max_accel = 10000;}
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetAcceleration(0, ctx->max_accel);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_max_d) == 0) {
		ctx->max_decel = L6474_GetDeceleration(0);
		ctx->max_decel = ctx->max_decel - *adjust_increment;
		if (ctx->max_decel <  0){ ctx->max_decel = 0;}
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetDeceleration(0, ctx->max_decel);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_max_d) == 0) {
		ctx->max_decel = L6474_GetDeceleration(0);
		ctx->max_decel = ctx->max_decel + *adjust_increment;
		if (ctx->max_decel > 10000) { ctx->max_decel = 10000; }
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetDeceleration(0, ctx->max_decel);
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_select_mode_5) == 0) {
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetDeceleration(0, MAX_DECEL);
		L6474_SetAcceleration(0, MAX_ACCEL);
		L6474_SetMinSpeed(0, MIN_SPEED_MODE_5);
		L6474_SetMaxSpeed(0, MAX_SPEED_MODE_5);
		PID_Pend->Kp = PRIMARY_PROPORTIONAL_MODE_5;
		PID_Pend->Ki = PRIMARY_INTEGRAL_MODE_5;
		PID_Pend->Kd = PRIMARY_DERIVATIVE_MODE_5;
		PID_Rotor->Kp = SECONDARY_PROPORTIONAL_MODE_5;
		PID_Rotor->Ki = SECONDARY_INTEGRAL_MODE_5;
		PID_Rotor->Kd = SECONDARY_DERIVATIVE_MODE_5;
		ctx->enable_adaptive_mode = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_enable_noise_rej_step ) == 0 ){
		ctx->gains.enable_noise_rejection_step = 1;
		ctx->gains.enable_disturbance_rejection_step = 0;
		ctx->gains.enable_sensitivity_fnc_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_disable_noise_rej_step ) == 0 ){
		ctx->gains.enable_noise_rejection_step = 0;
		ctx->gains.enable_disturbance_rejection_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_enable_sensitivity_fnc_step ) == 0 ){
		ctx->gains.enable_sensitivity_fnc_step = 1;
		ctx->gains.enable_disturbance_rejection_step = 0;
		ctx->gains.enable_noise_rejection_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_disable_sensitivity_fnc_step ) == 0 ){
		ctx->gains.enable_sensitivity_fnc_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_enable_load_dist ) == 0 ){
		ctx->gains.enable_sensitivity_fnc_step = 0;
		ctx->gains.enable_disturbance_rejection_step = 1;
		ctx->gains.enable_noise_rejection_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_disable_load_dist ) == 0 ){
		ctx->gains.enable_disturbance_rejection_step = 0;
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_inc_step_size ) == 0 ){
		step_size = step_size + 1;
		if (step_size > 4) { step_size = 4; }
		if (step_size == 0) { *adjust_increment = 2;}
		else if (step_size == 1) { *adjust_increment = 5;}
		else if (step_size == 2) { *adjust_increment = 20;}
		else if (step_size == 3) { *adjust_increment = 50;}
		else if (step_size == 4) { *adjust_increment = 100;}
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_dec_step_size ) == 0 ){
		step_size = step_size - 1;
		if (step_size < 0) { step_size = 0; }
		if (step_size == 0) { *adjust_increment = 2;}
		else if (step_size == 1) { *adjust_increment = 5;}
		else if (step_size == 2) { *adjust_increment = 20;}
		else if (step_size == 3) { *adjust_increment = 50;}
		else if (step_size == 4) { *adjust_increment = 100;}
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_enable_high_speed_sampling ) == 0 ){
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_disable_high_speed_sampling ) == 0 ){
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_enable_speed_prescale ) == 0 ){
		config_command = 1;
	} else if (strcmp(user_config_input, mode_string_disable_speed_prescale ) == 0 ){
		config_command = 1;
	} else if  (strcmp(user_config_input, mode_string_disable_speed_governor ) == 0 ){
		config_command = 1;
	} else if  (strcmp(user_config_input, mode_string_enable_speed_governor ) == 0 ){
		config_command = 1;
	} else {
		mode_index_command = atoi((char*) Msg.Data);
	}
	return mode_index_command;
}

void assign_mode_1(AppControlContext *ctx, arm_pid_instance_a_f32 *PID_Pend,
		arm_pid_instance_a_f32 *PID_Rotor){
	ctx->select_suspended_mode = 0;
	ctx->gains.proportional = PRIMARY_PROPORTIONAL_MODE_1;
	ctx->gains.integral = PRIMARY_INTEGRAL_MODE_1;
	ctx->gains.derivative = PRIMARY_DERIVATIVE_MODE_1;
	ctx->gains.rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_1;
	ctx->gains.rotor_i_gain = SECONDARY_INTEGRAL_MODE_1;
	ctx->gains.rotor_d_gain = SECONDARY_DERIVATIVE_MODE_1;
	PID_Pend->Kp = ctx->gains.proportional;
	PID_Pend->Ki = ctx->gains.integral;
	PID_Pend->Kd = ctx->gains.derivative;
	PID_Rotor->Kp = ctx->gains.rotor_p_gain;
	PID_Rotor->Ki = ctx->gains.rotor_i_gain;
	PID_Rotor->Kd = ctx->gains.rotor_d_gain;
	ctx->torq_current_val = MAX_TORQUE_CONFIG;
	L6474_SetAnalogValue(0, L6474_TVAL, ctx->torq_current_val);
}

void assign_mode_2(AppControlContext *ctx, arm_pid_instance_a_f32 *PID_Pend,
		arm_pid_instance_a_f32 *PID_Rotor){
	ctx->select_suspended_mode = 0;
	ctx->gains.proportional = PRIMARY_PROPORTIONAL_MODE_2;
	ctx->gains.integral = PRIMARY_INTEGRAL_MODE_2;
	ctx->gains.derivative = PRIMARY_DERIVATIVE_MODE_2;
	ctx->gains.rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_2;
	ctx->gains.rotor_i_gain = SECONDARY_INTEGRAL_MODE_2;
	ctx->gains.rotor_d_gain = SECONDARY_DERIVATIVE_MODE_2;
	PID_Pend->Kp = ctx->gains.proportional;
	PID_Pend->Ki = ctx->gains.integral;
	PID_Pend->Kd = ctx->gains.derivative;
	PID_Rotor->Kp = ctx->gains.rotor_p_gain;
	PID_Rotor->Ki = ctx->gains.rotor_i_gain;
	PID_Rotor->Kd = ctx->gains.rotor_d_gain;
	ctx->torq_current_val = MAX_TORQUE_CONFIG;
	L6474_SetAnalogValue(0, L6474_TVAL, ctx->torq_current_val);
}

void assign_mode_3(AppControlContext *ctx, arm_pid_instance_a_f32 *PID_Pend,
		arm_pid_instance_a_f32 *PID_Rotor){
	ctx->select_suspended_mode = 0;
	ctx->gains.proportional = PRIMARY_PROPORTIONAL_MODE_3;
	ctx->gains.integral = PRIMARY_INTEGRAL_MODE_3;
	ctx->gains.derivative = PRIMARY_DERIVATIVE_MODE_3;
	ctx->gains.rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_3;
	ctx->gains.rotor_i_gain = SECONDARY_INTEGRAL_MODE_3;
	ctx->gains.rotor_d_gain = SECONDARY_DERIVATIVE_MODE_3;
	PID_Pend->Kp = ctx->gains.proportional;
	PID_Pend->Ki = ctx->gains.integral;
	PID_Pend->Kd = ctx->gains.derivative;
	PID_Rotor->Kp = ctx->gains.rotor_p_gain;
	PID_Rotor->Ki = ctx->gains.rotor_i_gain;
	PID_Rotor->Kd = ctx->gains.rotor_d_gain;
	ctx->torq_current_val = MAX_TORQUE_CONFIG;
	L6474_SetAnalogValue(0, L6474_TVAL, ctx->torq_current_val);
}

void set_mode_strings(void){
	sprintf(mode_string_mode_1, "1");
	sprintf(mode_string_mode_2, "2");
	sprintf(mode_string_mode_8, "g");
	sprintf(mode_string_mode_test, "t");
	sprintf(mode_string_mode_control, "r");
	sprintf(mode_string_mode_motor_characterization_mode, "c");
	sprintf(mode_string_dec_accel, "d");
	sprintf(mode_string_inc_accel, "i");
	sprintf(mode_string_inc_amp, "j");
	sprintf(mode_string_dec_amp, "k");
	sprintf(mode_string_stop, "q");

	sprintf(mode_string_dec_pend_p, "a");
	sprintf(mode_string_inc_pend_p, "A");
	sprintf(mode_string_dec_pend_i, "b");
	sprintf(mode_string_inc_pend_i, "B");
	sprintf(mode_string_dec_pend_d, "c");
	sprintf(mode_string_inc_pend_d, "C");
	sprintf(mode_string_dec_rotor_p, "d");
	sprintf(mode_string_inc_rotor_p, "D");
	sprintf(mode_string_dec_rotor_i, "e");
	sprintf(mode_string_inc_rotor_i, "E");
	sprintf(mode_string_dec_rotor_d, "f");
	sprintf(mode_string_inc_rotor_d, "F");
	sprintf(mode_string_dec_torq_c, "t");
	sprintf(mode_string_inc_torq_c, "T");
	sprintf(mode_string_dec_max_s, "s");
	sprintf(mode_string_inc_max_s, "S");
	sprintf(mode_string_dec_min_s, "m");
	sprintf(mode_string_inc_min_s, "M");
	sprintf(mode_string_dec_max_a, "n");
	sprintf(mode_string_inc_max_a, "N");
	sprintf(mode_string_dec_max_d, "o");
	sprintf(mode_string_inc_max_d, "O");
	sprintf(mode_string_enable_step, "P");
	sprintf(mode_string_disable_step, "p");
	sprintf(mode_string_enable_pendulum_impulse, "H");
	sprintf(mode_string_disable_pendulum_impulse, "h");
	sprintf(mode_string_enable_load_dist, "L");
	sprintf(mode_string_disable_load_dist, "l");
	sprintf(mode_string_enable_noise_rej_step, "R");
	sprintf(mode_string_disable_noise_rej_step, "r");
	sprintf(mode_string_enable_sensitivity_fnc_step, "V");
	sprintf(mode_string_disable_sensitivity_fnc_step, "v");
	sprintf(mode_string_inc_step_size, "J");
	sprintf(mode_string_dec_step_size, "j");
	sprintf(mode_string_select_mode_5, "u");
	sprintf(mode_string_enable_high_speed_sampling, "Y");
	sprintf(mode_string_disable_high_speed_sampling, "y");
	sprintf(mode_string_enable_speed_prescale, ":");
	sprintf(mode_string_disable_speed_prescale, ";");
	sprintf(mode_string_enable_speed_governor, "(");
	sprintf(mode_string_disable_speed_governor, ")");
	sprintf(mode_string_reset_system, "<");

	mode_1 = 1;
	mode_2 = 2;
	mode_8 = 8;
	mode_11 = 11;
	mode_13 = 13;
	mode_15 = 15;
	mode_quit = 0;
}

int ui_process_runtime_input(int cycle_index, AppControlContext *ctx,
		arm_pid_instance_a_f32 *PID_Pend,
		arm_pid_instance_a_f32 *PID_Rotor)
{
	RxBuffer_WriteIdx = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
	readBytes = Extract_Msg(RxBuffer, RxBuffer_ReadIdx, RxBuffer_WriteIdx,
			UART_RX_BUFFER_SIZE, &Msg);

	config_command = 0;
	if (readBytes == 1) {
		RxBuffer_ReadIdx = (RxBuffer_ReadIdx + readBytes) % UART_RX_BUFFER_SIZE;
		return 1;
	}

	/* Multi-byte commands: "r <steps>" (Mode A reference) or "u <steps/s^2>" (Mode B control) */
	if (readBytes >= 3 && Msg.Len >= 2) {
		RxBuffer_ReadIdx = (RxBuffer_ReadIdx + readBytes) % UART_RX_BUFFER_SIZE;
		char cmd = ((char *)Msg.Data)[0];
		if (((char *)Msg.Data)[1] == ' ') {
			float val = strtof((char *)Msg.Data + 2, NULL);
			if (cmd == 'r') {
				ctx->rotor_pos.rotor_position_command_steps = val;
			} else if (cmd == 'u') {
				remote_controller_set_u(val);
			}
		}
	}

	if (readBytes == 2 && Msg.Len == 1 && cycle_index % 10 == 0) {
		RxBuffer_ReadIdx = (RxBuffer_ReadIdx + readBytes) % UART_RX_BUFFER_SIZE;
		ctx->mode_transition_state = 1;
		mode_index_command = mode_index_identification(ctx, (char *) Msg.Data, config_command,
				&ctx->adjust_increment, PID_Pend, PID_Rotor);
		strcpy(config_message, (char *) Msg.Data);
		if (strcmp(config_message, "q") == 0) {
			const char *exit_msg = "\n\rExit Control Loop Command Received ";
			HAL_UART_Transmit(&huart2, (uint8_t*) exit_msg, strlen(exit_msg), HAL_MAX_DELAY);
			return -1;
		}
	}

	if (mode_index_command == 1 && ctx->mode_transition_state == 1) {
		mode_index = 1;
		ctx->mode_transition_state = 0;
		mode_index_command = 0;
		assign_mode_1(ctx, PID_Pend, PID_Rotor);
	}
	if (mode_index_command == 2 && ctx->mode_transition_state == 1) {
		mode_index = 2;
		ctx->mode_transition_state = 0;
		mode_index_command = 0;
		assign_mode_2(ctx, PID_Pend, PID_Rotor);
	}
	if (mode_index_command == 3 && ctx->mode_transition_state == 1) {
		mode_index = 3;
		ctx->mode_transition_state = 0;
		mode_index_command = 0;
		assign_mode_3(ctx, PID_Pend, PID_Rotor);
	}

	return 0;
}

void user_prompt(void){
	sprintf(uart_tx_buf, "\n\r********  System Start Mode Selections  ********\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	sprintf(uart_tx_buf, "Enter 'A' at prompt for Mode A: Inverted Pendulum PID, PC sends 'r <steps>'.. \n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	sprintf(uart_tx_buf, "Enter 'B' at prompt for Mode B: PC computes control, sends 'u <steps/s^2>'.. \n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	sprintf(uart_tx_buf, "Enter 1 at prompt for Inverted Pendulum Control............................... \n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	sprintf(uart_tx_buf, "Enter 2 at prompt for Suspended Pendulum Control.............................. \n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	sprintf(uart_tx_buf, "Enter 'g' at prompt for General Mode: Full State Feedback and PID Controller.. \n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	sprintf(uart_tx_buf, "Enter 't' at prompt for Test of Rotor Actuator and Pendulum Angle Encoder..... \n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	sprintf(uart_tx_buf, "Enter 'r' at prompt for Direct Control of Rotor Actuator Operation............ \n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
}

void ui_set_mode_interactive(int enabled)
{
	mode_interactive = enabled;
}

int ui_get_mode_interactive(void)
{
	return mode_interactive;
}

static void get_user_mode_index(char * user_string, int * char_mode_select, int * mode_index){

	*char_mode_select = 0;

	if (strcmp(user_string,mode_string_mode_test)==0){
		*mode_index = 11;
		*char_mode_select = 1;
	}

	if (strcmp(user_string,mode_string_mode_control)==0){
		*mode_index = 15;
		*char_mode_select = 1;
	}

	if (strcmp(user_string,mode_string_mode_motor_characterization_mode)==0){
		*mode_index = 13;
		*char_mode_select = 1;
	}

	if (strcmp(user_string,mode_string_mode_8)==0){
		*mode_index = 8;
		*char_mode_select = 1;
	}

	if(*char_mode_select == 0){
		*mode_index = atoi(user_string);
	}

	switch (*mode_index) {

	case 1:
		*mode_index = mode_1;
		break;

	case 2:
		*mode_index = mode_2;
		break;

	case 8:
		*mode_index = mode_8;
		mode_interactive = 1;
		break;

	case 11:
		*mode_index = mode_11;
		mode_interactive = 1;
		break;

	case 13:
		*mode_index = mode_13;
		mode_interactive = 1;
		break;

	case 15:
		*mode_index = mode_15;
		mode_interactive = 1;
		break;

	default:
		*mode_index = mode_1;
		break;
	}

}

void user_configuration(AppControlContext *ctx){
	int k;

	ctx->enable_rotor_actuator_test = 0;
	ctx->enable_rotor_actuator_control = 0;
	enable_encoder_test = 0;
	/* enable_rotor_actuator_high_speed_test removed (write-only, never read) */
	ctx->enable_motor_actuator_characterization_mode = 0;

	ctx->gains.enable_disturbance_rejection_step = 0;
	ctx->gains.enable_noise_rejection_step = 0;
	ctx->gains.enable_sensitivity_fnc_step = 0;


	while (1){
		RxBuffer_WriteIdx = UART_RX_BUFFER_SIZE
				- __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

		readBytes = Extract_Msg(RxBuffer, RxBuffer_ReadIdx,
				RxBuffer_WriteIdx, UART_RX_BUFFER_SIZE, &Msg);


		/*
		 * Exit read loop after timeout selecting default Mode 1
		 */

		ctx->timing.tick_read_cycle = HAL_GetTick();
		if (((ctx->timing.tick_read_cycle - ctx->timing.tick_read_cycle_start) > START_DEFAULT_MODE_TIME) && (mode_interactive == 0)) {
			sprintf(uart_tx_buf, "\n\rNo Entry Detected - Now Selecting Default Inverted Pendulum Mode 1......: \n\r");
			HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
			ctx->gains.enable_state_feedback = 0;
			ctx->select_suspended_mode = 0;
			ctx->gains.proportional = 		PRIMARY_PROPORTIONAL_MODE_1;
			ctx->gains.integral = 			PRIMARY_INTEGRAL_MODE_1;
			ctx->gains.derivative = 		PRIMARY_DERIVATIVE_MODE_1;
			ctx->gains.rotor_p_gain = 		SECONDARY_PROPORTIONAL_MODE_1;
			ctx->gains.rotor_i_gain = 		SECONDARY_INTEGRAL_MODE_1;
			ctx->gains.rotor_d_gain = 		SECONDARY_DERIVATIVE_MODE_1;
			ctx->max_speed = 		MAX_SPEED_MODE_1;
			ctx->min_speed = 		MIN_SPEED_MODE_1;
			ctx->enc_cal.enable_angle_cal = 1;
			ctx->enable_swing_up = 1;
			L6474_SetAnalogValue(0, L6474_TVAL, TORQ_CURRENT_DEFAULT);
			break;
		}

		if (readBytes) // Message found
		{
			RxBuffer_ReadIdx = (RxBuffer_ReadIdx + readBytes) % UART_RX_BUFFER_SIZE;

			if (Msg.Len != 1) {
				continue;
			}

			sprintf(uart_tx_buf, "%s", (char*)Msg.Data);
			HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

			{
				char sel = ((char *)Msg.Data)[0];
				if (sel == 'A' || sel == 'B') {
					ctx->core_controller_ops = (sel == 'B') ? &CONTROLLER_OPS_REMOTE
					                                        : &CONTROLLER_OPS_DEFAULT;
					ctx->gains.enable_state_feedback = 0;
					ctx->select_suspended_mode = 0;
					ctx->enc_cal.enable_angle_cal = 1;
					ctx->enable_swing_up = 1;
					ctx->gains.proportional =   PRIMARY_PROPORTIONAL_MODE_1;
					ctx->gains.integral =       PRIMARY_INTEGRAL_MODE_1;
					ctx->gains.derivative =     PRIMARY_DERIVATIVE_MODE_1;
					ctx->gains.rotor_p_gain =   SECONDARY_PROPORTIONAL_MODE_1;
					ctx->gains.rotor_i_gain =   SECONDARY_INTEGRAL_MODE_1;
					ctx->gains.rotor_d_gain =   SECONDARY_DERIVATIVE_MODE_1;
					ctx->gains.integral_compensator_gain = 0;
					ctx->gains.feedforward_gain = 1;
					ctx->max_speed = MAX_SPEED_MODE_1;
					ctx->min_speed = MIN_SPEED_MODE_1;
					L6474_SetAnalogValue(0, L6474_TVAL, TORQ_CURRENT_DEFAULT);
					if (sel == 'A') {
						sprintf(uart_tx_buf,
							"\n\rMode A: Inverted Pendulum PID. During run: 'r <steps>' sets rotor reference.\r\n");
					} else {
						sprintf(uart_tx_buf,
							"\n\rMode B: Remote Control. During run: 'u <steps/s^2>' sets control output.\r\n");
					}
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					break;
				}
			}

			get_user_mode_index((char*)Msg.Data, &char_mode_select, &mode_index);

			/*
			 * Configure Motor Speed Profile and PID Controller Gains
			 */

			ctx->gains.enable_disturbance_rejection_step = 0;
			ctx->gains.enable_noise_rejection_step = 0;
			ctx->gains.enable_sensitivity_fnc_step = 0;


			switch (mode_index) {

			/* Mode 1 selection */

			case 1:
				/* Flush read buffer  */
				for (k = 0; k < SERIAL_MSG_MAXLEN; k++) { Msg.Data[k] = 0; }

				ctx->gains.enable_state_feedback = 0;
				ctx->select_suspended_mode = 0;
				ctx->gains.proportional = 		PRIMARY_PROPORTIONAL_MODE_1;
				ctx->gains.integral = 			PRIMARY_INTEGRAL_MODE_1;
				ctx->gains.derivative = 		PRIMARY_DERIVATIVE_MODE_1;
				ctx->gains.rotor_p_gain = 		SECONDARY_PROPORTIONAL_MODE_1;
				ctx->gains.rotor_i_gain = 		SECONDARY_INTEGRAL_MODE_1;
				ctx->gains.rotor_d_gain = 		SECONDARY_DERIVATIVE_MODE_1;
				ctx->max_speed = 		MAX_SPEED_MODE_1;
				ctx->min_speed = 		MIN_SPEED_MODE_1;

				sprintf(uart_tx_buf, "\n\rMode 1 Configured\n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf,
						strlen(uart_tx_buf), HAL_MAX_DELAY);

				ctx->enc_cal.enable_angle_cal = 1;
				ctx->enable_swing_up = 1;
				L6474_SetAnalogValue(0, L6474_TVAL, TORQ_CURRENT_DEFAULT);

				break;

				/* Mode 2 Suspended Mode selection */
			case 2:
				/* Flush read buffer  */
				for (k = 0; k < SERIAL_MSG_MAXLEN; k++) { Msg.Data[k] = 0; }

				ctx->gains.enable_state_feedback = 0;
				ctx->select_suspended_mode = 1;
				ctx->gains.proportional = 		PRIMARY_PROPORTIONAL_MODE_4;
				ctx->gains.integral = 			PRIMARY_INTEGRAL_MODE_4;
				ctx->gains.derivative = 		PRIMARY_DERIVATIVE_MODE_4;
				ctx->gains.rotor_p_gain = 		SECONDARY_PROPORTIONAL_MODE_4;
				ctx->gains.rotor_i_gain = 		SECONDARY_INTEGRAL_MODE_4;
				ctx->gains.rotor_d_gain = 		SECONDARY_DERIVATIVE_MODE_4;
				ctx->max_speed = 		MAX_SPEED_MODE_1;
				ctx->min_speed = 		MIN_SPEED_MODE_1;

				sprintf(uart_tx_buf, "\n\rMode %i Configured", mode_index);
				HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf,
						strlen(uart_tx_buf), HAL_MAX_DELAY);

				ctx->enc_cal.enable_angle_cal = 1;

				break;


				/* General mode selection requiring user specification of all configurations */
			case 8:
				/* Flush read buffer  */
				for (k = 0; k < SERIAL_MSG_MAXLEN; k++) { Msg.Data[k] = 0; }

				sprintf(uart_tx_buf, "\n\r.....Enter negative value at any prompt to correct entry and Restart... \n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

				sprintf(uart_tx_buf, "\n\rEnter 0 for Dual PID - Enter 1 for State Feedback.........................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->gains.enable_state_feedback);
				sprintf(uart_tx_buf, "%i", ctx->gains.enable_state_feedback);
				HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
				if ( ctx->gains.enable_state_feedback < 0 ){
					sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if (ctx->gains.enable_state_feedback == 1){

					/*
					 * State feedback includes only ctx->gains.proportional and ctx->gains.derivative gains
					 * State feedback also includes optional ctx->gains.integral compensator gain
					 */

					ctx->gains.integral = 0;
					ctx->gains.rotor_i_gain = 0;
					ctx->gains.feedforward_gain = 1;

					sprintf(uart_tx_buf, "\n\rEnter Pendulum Angle Gain.................................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->gains.proportional);
					sprintf(uart_tx_buf, "%0.2f", ctx->gains.proportional);
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					if ( ctx->gains.proportional < 0 ){
						sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(uart_tx_buf, "\n\rEnter Pendulum Angle Derivative Gain......................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->gains.derivative);
					sprintf(uart_tx_buf, "%0.2f", ctx->gains.derivative);
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					if ( ctx->gains.derivative < 0 ){
						sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(uart_tx_buf, "\n\rEnter Rotor Angle Gain....................................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->gains.rotor_p_gain);
					sprintf(uart_tx_buf, "%0.2f", ctx->gains.rotor_p_gain);
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					if ( ctx->gains.rotor_p_gain < 0 ){
						sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(uart_tx_buf, "\n\rEnter Rotor Angle Derivative Gain.........................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->gains.rotor_d_gain);
					sprintf(uart_tx_buf, "%0.2f", ctx->gains.rotor_d_gain);
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					if ( ctx->gains.rotor_d_gain < 0 ){
						sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(uart_tx_buf, "\n\rEnter Integral Compensator Gain (zero to disable).........................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->gains.integral_compensator_gain);
					sprintf(uart_tx_buf, "%0.2f", ctx->gains.integral_compensator_gain);
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					if ( ctx->gains.integral_compensator_gain < 0 ){
						sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(uart_tx_buf, "\n\rEnter Feedforward Gain (return or zero to set to unity)...................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->gains.feedforward_gain);
					if (ctx->gains.feedforward_gain == 0){
						ctx->gains.feedforward_gain = 1;
					}
					sprintf(uart_tx_buf, "%0.2f", ctx->gains.feedforward_gain);
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

					ctx->gains.proportional = ctx->gains.proportional * FULL_STATE_FEEDBACK_SCALE;
					ctx->gains.derivative = ctx->gains.derivative * FULL_STATE_FEEDBACK_SCALE;
					ctx->gains.rotor_p_gain = ctx->gains.rotor_p_gain * FULL_STATE_FEEDBACK_SCALE;
					ctx->gains.rotor_d_gain = ctx->gains.rotor_d_gain * FULL_STATE_FEEDBACK_SCALE;
					ctx->gains.integral_compensator_gain = ctx->gains.integral_compensator_gain * FULL_STATE_FEEDBACK_SCALE;
					ctx->gains.feedforward_gain = ctx->gains.feedforward_gain * FULL_STATE_FEEDBACK_SCALE;
				}

				if (ctx->gains.enable_state_feedback == 0){

					sprintf(uart_tx_buf, "\n\rEnter Pendulum PID Proportional Gain......................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf,
							strlen(uart_tx_buf),
							HAL_MAX_DELAY);

					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->gains.proportional);
					sprintf(uart_tx_buf, "%0.2f", ctx->gains.proportional);
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					if ( ctx->gains.proportional < 0 ){
						sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(uart_tx_buf, "\n\rEnter Pendulum PID Integral Gain..........................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf,
							strlen(uart_tx_buf),
							HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->gains.integral);
					sprintf(uart_tx_buf, "%0.2f", ctx->gains.integral);
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					if ( ctx->gains.integral < 0 ){
						sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(uart_tx_buf, "\n\rEnter Pendulum PID Differential Gain......................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf,
							strlen(uart_tx_buf),
							HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->gains.derivative);
					sprintf(uart_tx_buf, "%0.2f", ctx->gains.derivative);
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					if ( ctx->gains.derivative < 0 ){
						sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(uart_tx_buf, "\n\rEnter Rotor PID Proportional Gain.........................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf,
							strlen(uart_tx_buf),
							HAL_MAX_DELAY);

					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->gains.rotor_p_gain);
					sprintf(uart_tx_buf, "%0.2f", ctx->gains.rotor_p_gain);
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					if ( ctx->gains.rotor_p_gain < 0 ){
						sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(uart_tx_buf, "\n\rEnter Rotor PID Integral Gain:............................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf,
							strlen(uart_tx_buf),
							HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->gains.rotor_i_gain);
					sprintf(uart_tx_buf, "%0.2f", ctx->gains.rotor_i_gain);
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					if ( ctx->gains.rotor_i_gain < 0 ){
						sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}

					sprintf(uart_tx_buf, "\n\rEnter Rotor PID Differential Gain.........................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->gains.rotor_d_gain);
					sprintf(uart_tx_buf, "%0.2f", ctx->gains.rotor_d_gain);
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					if ( ctx->gains.rotor_d_gain < 0 ){
						sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}


				}

				ctx->select_suspended_mode = 0;

				sprintf(uart_tx_buf, "\n\rEnter 0 for Inverted Mode - Enter 1 for Suspended Mode....................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->select_suspended_mode);
				sprintf(uart_tx_buf, "%i", ctx->select_suspended_mode);
				HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
				if ( ctx->select_suspended_mode < 0 ){
					sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if ( ctx->select_suspended_mode == 1 ){
					ctx->enc_cal.enable_angle_cal = 0;
				}

				ctx->enc_cal.enable_angle_cal = 0;
				sprintf(uart_tx_buf, "\n\rPlatform Angle Calibration Enabled - Enter 1 to Disable...................: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_angle_cal_resp);
				if (enable_angle_cal_resp == 0){
					ctx->enc_cal.enable_angle_cal = 1;
				}
				sprintf(uart_tx_buf, "%i", enable_angle_cal_resp);
				HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

				if ( enable_angle_cal_resp < 0 ){
					sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}

				if ( ctx->select_suspended_mode == 0 ){

					ctx->enable_swing_up = ENABLE_SWING_UP;
					enable_swing_up_resp = 0;
					sprintf(uart_tx_buf, "\n\rSwing Up Enabled - Enter 1 to Disable:....................................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &enable_swing_up_resp);
					if (enable_swing_up_resp == 1){
						ctx->enable_swing_up = 0;
					}
					sprintf(uart_tx_buf, "%i", enable_swing_up_resp);
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					if ( enable_swing_up_resp < 0 ){
						sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}
				}

				ctx->gains.enable_disturbance_rejection_step = 0;
				ctx->gains.enable_noise_rejection_step = 0;
				ctx->gains.enable_sensitivity_fnc_step = 0;

				sprintf(uart_tx_buf, "\n\rEnter 1 to Enable Disturbance Rejection Sensitivity; 0 to Disable.........: ");
				HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
				read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->gains.enable_disturbance_rejection_step);
				sprintf(uart_tx_buf, "%i", ctx->gains.enable_disturbance_rejection_step);
				if ( ctx->gains.enable_disturbance_rejection_step < 0 ){
					sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					HAL_Delay(3000);
					NVIC_SystemReset();
				}
				HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
				if (ctx->gains.enable_disturbance_rejection_step == 1) {
					ctx->gains.enable_sensitivity_fnc_step = 0;
					ctx->gains.enable_noise_rejection_step = 0;
				}



				if (ctx->gains.enable_disturbance_rejection_step == 0){
					sprintf(uart_tx_buf, "\n\rEnter 1 to Enable Noise Rejection Sensitivity; 0 to Disable...............: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->gains.enable_noise_rejection_step);
					sprintf(uart_tx_buf, "%i", ctx->gains.enable_noise_rejection_step);
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					if ( ctx->gains.enable_noise_rejection_step < 0 ){
						sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}
					if (ctx->gains.enable_noise_rejection_step == 1) {
						ctx->gains.enable_sensitivity_fnc_step = 0;
						ctx->gains.enable_disturbance_rejection_step = 0;
					}
				}

				if (ctx->gains.enable_noise_rejection_step == 0 && ctx->gains.enable_disturbance_rejection_step == 0){
					sprintf(uart_tx_buf, "\n\rEnter 1 to Enable Sensitivity Function; 0 to Disable......................: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->gains.enable_sensitivity_fnc_step);
					sprintf(uart_tx_buf, "%i", ctx->gains.enable_sensitivity_fnc_step);
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					if ( ctx->gains.enable_sensitivity_fnc_step < 0 ){
						sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
						HAL_Delay(3000);
						NVIC_SystemReset();
					}
					if (ctx->gains.enable_sensitivity_fnc_step == 1) {
						ctx->gains.enable_disturbance_rejection_step = 0;
						ctx->gains.enable_noise_rejection_step = 0;
					}
				}

				/*
				 * Reverse polarity of gain values to account for suspended mode angle configuration
				 */

				if(ctx->select_suspended_mode == 1){
					ctx->gains.proportional = 	-ctx->gains.proportional;
					ctx->gains.integral = 		-ctx->gains.integral;
					ctx->gains.derivative = 	-ctx->gains.derivative;
					ctx->gains.rotor_p_gain = 	-ctx->gains.rotor_p_gain;
					ctx->gains.rotor_i_gain = 	-ctx->gains.rotor_i_gain;
					ctx->gains.rotor_d_gain = 	-ctx->gains.rotor_d_gain;
					ctx->gains.integral_compensator_gain = -ctx->gains.integral_compensator_gain;
				}

				/*

				if (ctx->plant.enable_rotor_plant_gain_design == 0 && ctx->gains.enable_state_feedback == 1 && abs(ctx->gains.integral_compensator_gain) > 0 ){
					sprintf(uart_tx_buf, "\n\rEnter 1 for Rotor Plant Design Grotor = Wn^2/(s^2 + 2D*s + Wn^2) ...........: ");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->plant.select_rotor_plant_design);
					if (ctx->plant.select_rotor_plant_design == 1) {
						ctx->plant.enable_rotor_plant_design = 1;
					}
					if (ctx->plant.enable_rotor_plant_design == 1) {
						if (ctx->select_suspended_mode == 1){
							sprintf(uart_tx_buf, "\n\rEnter Natural Frequency (rad/sec) of Minimum 0.5 and Maximum 2 ...........: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
							read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->plant.rotor_natural_frequency);
							if (ctx->plant.rotor_natural_frequency > 2){
								ctx->plant.rotor_natural_frequency = 2;
							}
							if (ctx->plant.rotor_natural_frequency < 0.5){
								ctx->plant.rotor_natural_frequency = 0.5;
							}

							sprintf(uart_tx_buf, "%0.2f", ctx->plant.rotor_natural_frequency);
							HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

							sprintf(uart_tx_buf, "\n\rEnter Rotor Damping Coefficient of Minimum 0.1 and Maximum 5..............: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
							read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->plant.rotor_damping_coefficient);
							if (ctx->plant.rotor_damping_coefficient > 5){
								ctx->plant.rotor_damping_coefficient = 5;
							}

							if (ctx->plant.rotor_damping_coefficient < 0.1){
								ctx->plant.rotor_damping_coefficient = 0.1;
							}

							sprintf(uart_tx_buf, "%0.2f", ctx->plant.rotor_damping_coefficient);
							HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

							ctx->plant.rotor_plant_gain = 1;

						}

						if (ctx->select_suspended_mode == 0 ){
							sprintf(uart_tx_buf, "\n\rEnter Natural Frequency (rad/sec) of Minimum 0.5 and Maximum 2............: ");
							HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
							read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->plant.rotor_natural_frequency);
							if (ctx->plant.rotor_natural_frequency > 5){
								ctx->plant.rotor_natural_frequency = 5;
							}
							if (ctx->plant.rotor_natural_frequency <= 0.5){
								ctx->plant.rotor_natural_frequency = 0.5;
							}

							sprintf(uart_tx_buf, "%0.2f", ctx->plant.rotor_natural_frequency);
							HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

							sprintf(uart_tx_buf, "\n\rEnter Rotor Damping Coefficient of Minimum 0.5 and Maximum 5 ............ : ");
							HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
							read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->plant.rotor_damping_coefficient);
							if (ctx->plant.rotor_damping_coefficient > 5){
								ctx->plant.rotor_damping_coefficient = 5;
							}

							if (ctx->plant.rotor_damping_coefficient < 0.5){
								ctx->plant.rotor_damping_coefficient = 0.5;
							}
							sprintf(uart_tx_buf, "%0.2f", ctx->plant.rotor_damping_coefficient);
							HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

							ctx->plant.rotor_plant_gain = 1;
						}
					}
				}

				*/



				ctx->torq_current_val = MAX_TORQUE_CONFIG;

				if (ENABLE_TORQUE_CURRENT_ENTRY == 1){
					sprintf(uart_tx_buf, "\n\rEnter Torque Current mA (default is %i)................................: ", (int)MAX_TORQUE_CONFIG);
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
					read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx , &readBytes, &ctx->torq_current_val);
					if (ctx->torq_current_val == 0){
						ctx->torq_current_val = MAX_TORQUE_CONFIG;
					}

					sprintf(uart_tx_buf, "%0.2f", ctx->torq_current_val);
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
				}

				sprintf(uart_tx_buf, "\n\rPendulum PID Gains: \tP: %.02f; I: %.02f; D: %.02f", ctx->gains.proportional, ctx->gains.integral, ctx->gains.derivative);
				HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf,strlen(uart_tx_buf),HAL_MAX_DELAY);
				sprintf(uart_tx_buf, "\n\rRotor PID Gains: \tP: %.02f; I: %.02f; D: %.02f", ctx->gains.rotor_p_gain, ctx->gains.rotor_i_gain, ctx->gains.rotor_d_gain);
				HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf,strlen(uart_tx_buf),HAL_MAX_DELAY);
				if (ctx->select_suspended_mode == 1){
					sprintf(uart_tx_buf, "\n\rSuspended Mode gain values must be negative or zero");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf,strlen(uart_tx_buf),HAL_MAX_DELAY);
				}

				ctx->max_speed = MAX_SPEED_MODE_1;
				ctx->min_speed = MIN_SPEED_MODE_1;
				ctx->enc_cal.enable_angle_cal = 1;
				ctx->enable_swing_up = (ctx->select_suspended_mode == 0) ? 1 : 0;

				break;

				/* Rotor actuator and encoder test mode */

			case 11:
				ctx->enable_rotor_actuator_test = 1;
				enable_encoder_test = 1;
				sprintf(uart_tx_buf, "\n\rTest Mode Configured");
				HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf,
						strlen(uart_tx_buf), HAL_MAX_DELAY);
				break;

				/* Rotor actuator characterization test mode */
			case 13:
				ctx->enable_motor_actuator_characterization_mode = 1;
				sprintf(uart_tx_buf, "\n\rMotor Characterization Mode Configured");
				HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf,strlen(uart_tx_buf), HAL_MAX_DELAY);

				rotor_test_speed_min = 200;
				rotor_test_speed_max = 1000;
				rotor_test_acceleration_max = 3000;
				swing_deceleration_max = 3000;
				ctx->torq_current_val = MAX_TORQUE_CONFIG;
				rotor_chirp_amplitude = 5;
				rotor_chirp_start_freq = 0.05f;
				rotor_chirp_end_freq = 5.0f;
				rotor_chirp_period = 40.0f;

				break;

				/* Rotor actuator control mode */
			case 15:
				ctx->enable_rotor_actuator_control = 1;
				sprintf(uart_tx_buf, "\n\rRotor Actuator Control Mode Configured");
				HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf,
						strlen(uart_tx_buf), HAL_MAX_DELAY);
				break;

			/* Default start mode */
			default:

				ctx->gains.enable_state_feedback = 0;
				ctx->select_suspended_mode = 0;
				ctx->gains.proportional = 		PRIMARY_PROPORTIONAL_MODE_1;
				ctx->gains.integral = 			PRIMARY_INTEGRAL_MODE_1;
				ctx->gains.derivative = 		PRIMARY_DERIVATIVE_MODE_1;
				ctx->gains.rotor_p_gain = 		SECONDARY_PROPORTIONAL_MODE_1;
				ctx->gains.rotor_i_gain = 		SECONDARY_INTEGRAL_MODE_1;
				ctx->gains.rotor_d_gain = 		SECONDARY_DERIVATIVE_MODE_1;
				ctx->max_speed = 		MAX_SPEED_MODE_1;
				ctx->min_speed = 		MIN_SPEED_MODE_1;
				ctx->enc_cal.enable_angle_cal = 1;
				L6474_SetAnalogValue(0, L6474_TVAL, TORQ_CURRENT_DEFAULT);
				sprintf(uart_tx_buf, "\n\rDefault Mode 1 Configured");
				HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf,
						strlen(uart_tx_buf), HAL_MAX_DELAY);
				break;
			}
			return;
		}
	}
	return;
}

/*
 * Rotor and encoder test mode
 */

void rotor_encoder_test(AppControlContext *ctx){
	int j;
	/*
	 * Set Motor Speed Profile
	 */

	BSP_MotorControl_SetMaxSpeed(0, MAX_SPEED_MODE_1);
	BSP_MotorControl_SetMinSpeed(0, MIN_SPEED_MODE_1);

	sprintf(uart_tx_buf, "\n\rMotor Profile Speeds Min %u Max %u",
			rotor_test_speed_min, rotor_test_speed_max);
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),HAL_MAX_DELAY);

	BSP_MotorControl_SetAcceleration(0,(uint16_t)(MAX_ACCEL));
	BSP_MotorControl_SetDeceleration(0,(uint16_t)(MAX_DECEL));

	sprintf(uart_tx_buf, "\n\rMotor Profile Acceleration Max %u Deceleration Max %u",
			BSP_MotorControl_GetAcceleration(0), BSP_MotorControl_GetDeceleration(0));
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),HAL_MAX_DELAY);

	j = 0;

	while (j < ROTOR_ACTUATOR_TEST_CYCLES) {

		j++;

		sprintf(uart_tx_buf, "\r\n\r\n********  Starting Rotor Motor Control Test  ********\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),HAL_MAX_DELAY);

		(void)hardware_rotor_position_read(&ctx->rotor_pos.rotor_position_steps);
		sprintf(uart_tx_buf, "Motor Position at Zero Angle: %.2f\r\n",
				(float) ((ctx->rotor_pos.rotor_position_steps) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),HAL_MAX_DELAY);

		sprintf(uart_tx_buf, "Next Test in 3s\r\n\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),HAL_MAX_DELAY);
		HAL_Delay(3000);

		rotor_position_command_deg = -45;
		BSP_MotorControl_GoTo(0, (int)(rotor_position_command_deg*STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE));
		BSP_MotorControl_WaitWhileActive(0);

		(void)hardware_rotor_position_read(&ctx->rotor_pos.rotor_position_steps);
		sprintf(uart_tx_buf, "Motor Position Test to -45 Degree Angle: %.2f\r\n",
				(float) ((ctx->rotor_pos.rotor_position_steps) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),HAL_MAX_DELAY);

		sprintf(uart_tx_buf, "Correct motion shows rotor rotating to left\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

		sprintf(uart_tx_buf, "Next Test in 3s\r\n\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
		HAL_Delay(3000);

		rotor_position_command_deg = 0;
		BSP_MotorControl_GoTo(0, (int)(rotor_position_command_deg*STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE));
		BSP_MotorControl_WaitWhileActive(0);

		(void)hardware_rotor_position_read(&ctx->rotor_pos.rotor_position_steps);
		sprintf(uart_tx_buf, "Motor Position Test to Zero Angle: %.2f\r\n",
				(float) ((ctx->rotor_pos.rotor_position_steps) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),HAL_MAX_DELAY);

		sprintf(uart_tx_buf, "Correct motion shows rotor returning to zero angle\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),HAL_MAX_DELAY);

		sprintf(uart_tx_buf, "Next Test in 3s\r\n\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),HAL_MAX_DELAY);
		HAL_Delay(3000);

		rotor_position_command_deg = 90;
		BSP_MotorControl_GoTo(0, (int)(rotor_position_command_deg*STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE));
		BSP_MotorControl_WaitWhileActive(0);

		(void)hardware_rotor_position_read(&ctx->rotor_pos.rotor_position_steps);
		sprintf(uart_tx_buf, "Motor Position at 90 Degree Angle: %.2f\r\n",
				(float) ((ctx->rotor_pos.rotor_position_steps) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

		sprintf(uart_tx_buf, "Correct motion shows rotor rotating to right\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),HAL_MAX_DELAY);

		sprintf(uart_tx_buf, "Next Test in 3s\r\n\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),HAL_MAX_DELAY);
		HAL_Delay(3000);

		rotor_position_command_deg = 0;
		BSP_MotorControl_GoTo(0, (int)(rotor_position_command_deg*STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE));
		BSP_MotorControl_WaitWhileActive(0);

		(void)hardware_rotor_position_read(&ctx->rotor_pos.rotor_position_steps);
		sprintf(uart_tx_buf, "Motor Position at Zero Angle: %.2f\r\n",
				(float) ((ctx->rotor_pos.rotor_position_steps) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),HAL_MAX_DELAY);

		sprintf(uart_tx_buf, "Correct motion shows rotor rotating to zero angle\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),HAL_MAX_DELAY);

		sprintf(uart_tx_buf, "Rotor Actuator Test Cycle Complete, Next Test in 3s\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),HAL_MAX_DELAY);
		HAL_Delay(3000);
	}
	/*
	 * 	Encoder Test Sequence will execute at each cycle of operation if enable_encoder_test is set to 1
	 */

	if (enable_encoder_test == 1) {
		sprintf(uart_tx_buf, "\r\n*************  Starting Encoder Test  ***************\r\n\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);

		sprintf(uart_tx_buf, "Permit Pendulum to Stabilize in Vertical Down\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);
		HAL_Delay(1000);
		sprintf(uart_tx_buf, "Angle will be measured in 3 seconds\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);
		HAL_Delay(3000);

		(void)hardware_encoder_position_read(&ctx->enc_cal.encoder_position_steps, ctx->enc_cal.encoder_position_init, &htim3);
		ctx->enc_cal.encoder_position_down = ctx->enc_cal.encoder_position;
		sprintf(uart_tx_buf, "Encoder Angle is: %.2f \r\n(Correct value should lie between -0.5 and 0.5 degrees))\r\n\r\n",
				(float) (ctx->enc_cal.encoder_position_down / ctx->angle_scale));
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);

		sprintf(uart_tx_buf,
				"Manually Rotate Pendulum in Clock Wise Direction One Full 360 Degree Turn and Stabilize Down\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);
		sprintf(uart_tx_buf, "Angle will be measured in 10 seconds\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);
		HAL_Delay(10000);

		(void)hardware_encoder_position_read(&ctx->enc_cal.encoder_position_steps, ctx->enc_cal.encoder_position_init, &htim3);
		sprintf(uart_tx_buf, "Encoder Angle is: %.2f\r\n(Correct value should lie between -359.5 and -360.5 degrees)\r\n\r\n",
				(float) ((ctx->enc_cal.encoder_position_steps - ctx->enc_cal.encoder_position_down)
						/ ctx->angle_scale));
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);

		sprintf(uart_tx_buf,
				"Manually Rotate Pendulum in Counter Clock Wise Direction One Full 360 Degree Turn and Stabilize Down\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);
		sprintf(uart_tx_buf, "Angle will be measured in 10 seconds\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);
		HAL_Delay(10000);

		(void)hardware_encoder_position_read(&ctx->enc_cal.encoder_position_steps, ctx->enc_cal.encoder_position_init, &htim3);
		sprintf(uart_tx_buf, "Encoder Angle is: %.2f \r\n(Correct value should lie between -0.5 and 0.5 degrees) \r\n\r\n",
				(float) ((ctx->enc_cal.encoder_position_steps - ctx->enc_cal.encoder_position_down)
						/ ctx->angle_scale));
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);

		L6474_CmdDisable(0);
		sprintf(uart_tx_buf, "\r\nPendulum Encoder Characterization Complete");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);
		sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
		HAL_Delay(3000);
		NVIC_SystemReset();
	}
}

/*
 * Rotor actuator characterization mode
 */

void motor_actuator_characterization_mode(AppControlContext *ctx){
	int i, j, k;
	/*
	 * Set Motor Speed Profile
	 */

	BSP_MotorControl_SetMaxSpeed(0, rotor_test_speed_max);
	BSP_MotorControl_SetMinSpeed(0, rotor_test_speed_min);

	sprintf(uart_tx_buf, "\n\rMotor Profile Speeds Min %u Max %u",
			rotor_test_speed_min, rotor_test_speed_max);
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
			HAL_MAX_DELAY);

	BSP_MotorControl_SetAcceleration(0,
			(uint16_t) (rotor_test_acceleration_max));
	BSP_MotorControl_SetDeceleration(0,
			(uint16_t) (swing_deceleration_max));

	sprintf(uart_tx_buf,
			"\n\rMotor Profile Acceleration Max %u Deceleration Max %u",
			BSP_MotorControl_GetAcceleration(0),
			BSP_MotorControl_GetDeceleration(0));
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
			HAL_MAX_DELAY);

	/*
	 * Set Rotor Position Zero
	 */

	hardware_rotor_home();
	/* test_time was write-only, removed */

	rotor_chirp_step_period = (int) (rotor_chirp_period * 240.0);
	ctx->timing.tick_cycle_start = HAL_GetTick();
	mode_index_command = 1;
	mode_index = 1;

	while (1) {
		i = 0;
		while (i < rotor_chirp_step_period) {
			RxBuffer_WriteIdx = UART_RX_BUFFER_SIZE
					- __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
			readBytes = Extract_Msg(RxBuffer, RxBuffer_ReadIdx,
					RxBuffer_WriteIdx, UART_RX_BUFFER_SIZE, &Msg);

			if (readBytes == 2 && Msg.Len == 1 && i % 10 == 0) {
				RxBuffer_ReadIdx = (RxBuffer_ReadIdx + readBytes)
								% UART_RX_BUFFER_SIZE;
				ctx->mode_transition_state = 1;
				if (strcmp((char *) Msg.Data, mode_string_stop) == 0) {
					mode_index_command = mode_quit;
				} else if (strcmp((char *) Msg.Data, mode_string_inc_accel)
						== 0) {
					mode_index_command = 17;
				} else if (strcmp((char *) Msg.Data, mode_string_dec_accel)
						== 0) {
					mode_index_command = 16;
				} else if (strcmp((char *) Msg.Data,mode_string_inc_amp)
						== 0) {
					mode_index_command = 18;
				} else if (strcmp((char *) Msg.Data,mode_string_dec_amp)
						== 0) {
					mode_index_command = 19;
				} else if (strcmp((char *) Msg.Data,
						mode_string_mode_motor_characterization_mode)
						== 0) {
					mode_index_command = 1;
				} else {
					mode_index_command = atoi((char*) Msg.Data);
				}
			}



			if (mode_index_command == mode_quit) {
				break;
			}

			if (mode_index_command == 1 && ctx->mode_transition_state == 1) {
				mode_index = 1;
				ctx->mode_transition_state = 0;
			}

			if (mode_index_command == 2 && ctx->mode_transition_state == 1) {
				mode_index = 2;
				ctx->mode_transition_state = 0;
			}

			if (mode_index_command == 3 && ctx->mode_transition_state == 1) {
				L6474_SetAnalogValue(0, L6474_TVAL, MAX_TORQUE_CONFIG);
				ctx->mode_transition_state = 0;
			}

			if (mode_index_command == 4 && ctx->mode_transition_state == 1) {
				L6474_SetAnalogValue(0, L6474_TVAL, MAX_TORQUE_CONFIG);
				ctx->mode_transition_state = 0;
			}
			if (mode_index_command == 5 && ctx->mode_transition_state == 1) {
				L6474_SetAnalogValue(0, L6474_TVAL, MAX_TORQUE_CONFIG);
				ctx->mode_transition_state = 0;
			}

			if (mode_index_command == 6 && ctx->mode_transition_state == 1) {
				rotor_test_speed_max = rotor_test_speed_max + 100;
				if (rotor_test_speed_max > 1000) {
					rotor_test_speed_max = 1000;
				}
				BSP_MotorControl_SoftStop(0);
				BSP_MotorControl_WaitWhileActive(0);
				BSP_MotorControl_SetMaxSpeed(0, rotor_test_speed_max);
				ctx->mode_transition_state = 0;
			}

			if (mode_index_command == 7 && ctx->mode_transition_state == 1) {
				rotor_test_speed_max = rotor_test_speed_max - 100;
				if (rotor_test_speed_max < 200) {
					rotor_test_speed_max = 200;
				}
				if (rotor_test_speed_min > rotor_test_speed_max) {
					rotor_test_speed_max = rotor_test_speed_min;
				}
				BSP_MotorControl_SoftStop(0);
				BSP_MotorControl_WaitWhileActive(0);
				BSP_MotorControl_SetMaxSpeed(0, rotor_test_speed_max);
				ctx->mode_transition_state = 0;
			}

			if (mode_index_command == 8 && ctx->mode_transition_state == 1) {
				rotor_test_speed_min = rotor_test_speed_min + 100;
				if (rotor_test_speed_min > rotor_test_speed_max) {
					rotor_test_speed_min = rotor_test_speed_max;
				}
				if (rotor_test_speed_min > 1000) {
					rotor_test_speed_min = 1000;
				}
				BSP_MotorControl_SoftStop(0);
				BSP_MotorControl_WaitWhileActive(0);
				BSP_MotorControl_SetMinSpeed(0, rotor_test_speed_min);
				ctx->mode_transition_state = 0;
			}

			if (mode_index_command == 9 && ctx->mode_transition_state == 1) {
				rotor_test_speed_min = rotor_test_speed_min - 100;
				if (rotor_test_speed_min < 200) {
					rotor_test_speed_min = 200;
				}
				BSP_MotorControl_SoftStop(0);
				BSP_MotorControl_WaitWhileActive(0);
				BSP_MotorControl_SetMinSpeed(0, rotor_test_speed_min);
				ctx->mode_transition_state = 0;
			}

			if (mode_index_command == 16 && ctx->mode_transition_state == 1) {
				rotor_test_acceleration_max = rotor_test_acceleration_max - 500;
				if (rotor_test_acceleration_max < 0) {
					rotor_test_acceleration_max = 0;
				}
				swing_deceleration_max = rotor_test_acceleration_max;
				BSP_MotorControl_SoftStop(0);
				BSP_MotorControl_WaitWhileActive(0);
				BSP_MotorControl_SetAcceleration(0,
						(uint16_t) (rotor_test_acceleration_max));
				BSP_MotorControl_SetDeceleration(0,
						(uint16_t) (swing_deceleration_max));
				ctx->mode_transition_state = 0;
			}

			if (mode_index_command == 17 && ctx->mode_transition_state == 1) {
				rotor_test_acceleration_max = rotor_test_acceleration_max + 500;
				if (rotor_test_acceleration_max > 10000) {
					rotor_test_acceleration_max = 10000;
				}
				swing_deceleration_max = rotor_test_acceleration_max;
				BSP_MotorControl_SoftStop(0);
				BSP_MotorControl_WaitWhileActive(0);
				BSP_MotorControl_SetAcceleration(0,
						(uint16_t) (rotor_test_acceleration_max));
				BSP_MotorControl_SetDeceleration(0,
						(uint16_t) (swing_deceleration_max));
				ctx->mode_transition_state = 0;
			}

			if (mode_index_command == 18 && ctx->mode_transition_state == 1) {
				rotor_chirp_amplitude = rotor_chirp_amplitude + 1;
				if (rotor_chirp_amplitude > 10) {
					rotor_chirp_amplitude = 10;
				}
				ctx->mode_transition_state = 0;
			}

			if (mode_index_command == 19 && ctx->mode_transition_state == 1) {
				rotor_chirp_amplitude = rotor_chirp_amplitude - 1;
				if (rotor_chirp_amplitude < 1) {
					rotor_chirp_amplitude = 1;
				}
				ctx->mode_transition_state = 0;
			}

			if (i == 0) {
				ctx->timing.cycle_period_start = HAL_GetTick();
				ctx->timing.cycle_period_sum = 100 * ctx->timing.t_sample_s * 1000 - 1;
			}
			if (i % 100 == 0) {
				ctx->timing.cycle_period_sum = HAL_GetTick() - ctx->timing.cycle_period_start;
				ctx->timing.cycle_period_start = HAL_GetTick();
			}

			ctx->timing.tick = HAL_GetTick();
			ctx->timing.tick_cycle_previous = ctx->timing.tick_cycle_current;
			ctx->timing.tick_cycle_current = ctx->timing.tick;
			chirp_time = (float) (i) / 400;
			rotor_chirp_frequency = rotor_chirp_start_freq
					+ (rotor_chirp_end_freq - rotor_chirp_start_freq)
					* (float) (i) / rotor_chirp_step_period;

			if (mode_index == 1) {
				ctx->rotor_pos.rotor_position_command_steps =
						rotor_chirp_amplitude
						* (float) (STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE)
						* sin(
								2.0 * 3.14159
								* rotor_chirp_frequency
								* chirp_time);
			}

			if (mode_index == 2) {
				if (sin(
						2.0 * 3.14159 * rotor_chirp_frequency
						* chirp_time) < 0) {
					k = -1;
				} else {
					k = 1;
				}
				ctx->rotor_pos.rotor_position_command_steps = k * rotor_chirp_amplitude
						* STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE;
			}

			current_speed = BSP_MotorControl_GetCurrentSpeed(0);
			BSP_MotorControl_GoTo(0, (int) (ctx->rotor_pos.rotor_position_command_steps));

			if (BSP_MotorControl_GetDeviceState(0) == ACCELERATING) {
				motor_state = 1;
			}
			if (BSP_MotorControl_GetDeviceState(0) == DECELERATING) {
				motor_state = -1;
			}
			if (BSP_MotorControl_GetDeviceState(0) == STEADY) {
				motor_state = -2;
			}
			if (BSP_MotorControl_GetDeviceState(0) == INACTIVE) {
				motor_state = 0;
			}
			(void)hardware_rotor_position_read(&ctx->rotor_pos.rotor_position_steps);
			current_speed = BSP_MotorControl_GetCurrentSpeed(0);
			sprintf(uart_tx_buf,
					"%i\t%i\t%i\t%i\t%i\t%f\t%i\t%i\t%i\t%i\t%i\r\n", i,
					ctx->timing.cycle_period_sum,
					(int) (ctx->timing.tick_cycle_current - ctx->timing.tick_cycle_previous),
					current_speed, ctx->rotor_pos.rotor_position_steps,
					ctx->rotor_pos.rotor_position_command_steps, motor_state,
					rotor_test_speed_max, rotor_test_speed_min,
					rotor_test_acceleration_max, swing_deceleration_max);
			HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
					HAL_MAX_DELAY);
			i = i + 1;
		}
		if (mode_index_command == mode_quit) {
			break;
		}
		j = j + 1;
	}
	L6474_CmdDisable(0);
	sprintf(uart_tx_buf, "\r\nMotor Characterization Complete");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
			HAL_MAX_DELAY);
	sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	HAL_Delay(3000);
	NVIC_SystemReset();
}

/*
 * Interactive rotor control
 */

void interactive_rotor_actuator_control(void){
	int j;
	int rotor_position_steps;
	while (1) {

		/*
		 * Set Motor Speed Profile
		 */

		sprintf(uart_tx_buf, "\r\nEnter Motor Maximum Speed..............................................: ");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);

		read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes,
				&rotor_test_speed_max);
		sprintf(uart_tx_buf, "%i", rotor_test_speed_max);
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);

		sprintf(uart_tx_buf, "\r\nEnter Motor Minimum Speed..............................................: ");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);

		read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes,
				&rotor_test_speed_min);
		sprintf(uart_tx_buf, "%i", rotor_test_speed_min);
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);

		sprintf(uart_tx_buf, "\r\nEnter Motor Maximum Acceleration.......................................: ");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);

		read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes,
				&rotor_test_acceleration_max);
		sprintf(uart_tx_buf, "%i", rotor_test_acceleration_max);
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);

		sprintf(uart_tx_buf, "\r\nEnter Motor Maximum Deceleration.......................................: ");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);

		read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes,
				&swing_deceleration_max);
		sprintf(uart_tx_buf, "%i", swing_deceleration_max);
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);

		BSP_MotorControl_SetMaxSpeed(0, rotor_test_speed_max);
		BSP_MotorControl_SetMinSpeed(0, rotor_test_speed_min);

		sprintf(uart_tx_buf, "\n\rMotor Profile Speeds Minimum %u Maximum %u",
				rotor_test_speed_min, rotor_test_speed_max);
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);

		BSP_MotorControl_SetAcceleration(0, rotor_test_acceleration_max);
		BSP_MotorControl_SetDeceleration(0, swing_deceleration_max);

		sprintf(uart_tx_buf,"\n\rMotor Profile Acceleration Maximum %u Deceleration Maximum %u",
				BSP_MotorControl_GetAcceleration(0), BSP_MotorControl_GetDeceleration(0));
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),HAL_MAX_DELAY);

		j = 1;

		/*
		 * Set initial rotor position
		 */

		while (1) {

			sprintf(uart_tx_buf, "\r\nEnter Motor Position Target in Degrees ................................: ");
			HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

			read_float(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes, &rotor_position_command_deg);
			sprintf(uart_tx_buf, "%0.2f", rotor_position_command_deg);
			HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

			BSP_MotorControl_GoTo(0, (int)(rotor_position_command_deg*STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE));
			BSP_MotorControl_WaitWhileActive(0);

			(void)hardware_rotor_position_read(&rotor_position_steps);
			sprintf(uart_tx_buf, "\n\rMotor Position in Steps %i and Degrees %.2f\r\n",
					rotor_position_steps, (float) ((rotor_position_steps) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
			HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

			sprintf(uart_tx_buf, "\r\nEnter 1 to Enter New Motor Configuration, 0 to Continue, -1 to Exit ...: ");
			HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

			read_int(&RxBuffer_ReadIdx, &RxBuffer_WriteIdx, &readBytes,
					&j);
			if (j == 1) {
				break;
			}
			if (j == -1) {
				break;
			}

		}
		if (j == -1) {
				break;
		}

	}
	L6474_CmdDisable(0);
	sprintf(uart_tx_buf, "\r\nMotor Characterization Terminated");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
			HAL_MAX_DELAY);
	sprintf(uart_tx_buf, "\n\r\n\r*************************System Reset and Restart***************************\n\r\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	HAL_Delay(3000);
	NVIC_SystemReset();
}
