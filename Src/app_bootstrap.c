#include "main.h"
#include "edukit_system.h"
#include "app_control.h"
#include "app_bootstrap.h"
#include "ui.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

void app_bootstrap_system(AppControlContext *ctx, L6474_Init_t *motor_init)
{
	RxBuffer_ReadIdx = 0;
	RxBuffer_WriteIdx = 0;
	readBytes = 0;

	encoder_position = 0;
	encoder_position_down = 0;
	angle_scale = ENCODER_READ_ANGLE_SCALE;

	rotor_control_target_steps = 0;
	rotor_control_target_steps_curr = 0;
	app_reset_command_shaper_state(ctx);
	current_error_rotor_integral = 0;

	enable_rotor_chirp = 0;
	rotor_chirp_start_freq = ROTOR_CHIRP_START_FREQ;
	rotor_chirp_end_freq = ROTOR_CHIRP_END_FREQ;
	rotor_chirp_period = ROTOR_CHIRP_PERIOD;
	enable_mod_sin_rotor_tracking = ENABLE_MOD_SIN_ROTOR_TRACKING;
	enable_rotor_position_step_response_cycle =
			ENABLE_ROTOR_POSITION_STEP_RESPONSE_CYCLE;
	disable_mod_sin_rotor_tracking = 0;
	sine_drive_transition = 0;
	mod_sin_amplitude = MOD_SIN_AMPLITUDE;
	rotor_control_sin_amplitude = MOD_SIN_AMPLITUDE;

	enable_disturbance_rejection_step = 0;
	enable_noise_rejection_step = 0;
	enable_sensitivity_fnc_step = 0;
	enable_pendulum_position_impulse_response_cycle = 0;

	step_size = 0;
	adjust_increment = 0.5;
	mode_transition_state = 0;
	transition_to_adaptive_mode = 0;
	char_mode_select = 0;

	HAL_Init();
	SystemClock_Config();
	select_suspended_mode = ENABLE_SUSPENDED_PENDULUM_CONTROL;

	BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_L6474, 1);
	BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, motor_init);

	MX_TIM3_Init();
	encoder_position_init = 0;
	HAL_Delay(10);
	MX_USART2_UART_Init();

	HAL_Delay(1);
	BSP_MotorControl_SetMaxSpeed(0, MAX_SPEED_UPPER_INIT);
	HAL_Delay(1);
	BSP_MotorControl_SetMinSpeed(0, MIN_SPEED_UPPER_INIT);
	HAL_Delay(1);
	BSP_MotorControl_SetMaxSpeed(0, MAX_SPEED_LOWER_INIT);
	HAL_Delay(1);
	BSP_MotorControl_SetMinSpeed(0, MIN_SPEED_LOWER_INIT);
	HAL_Delay(1);
	BSP_MotorControl_SetAcceleration(0, MAX_ACCEL_UPPER_INIT);
	HAL_Delay(1);
	BSP_MotorControl_SetDeceleration(0, MAX_DECEL_UPPER_INIT);
	HAL_Delay(1);

	max_accel = MAX_ACCEL;
	max_decel = MAX_DECEL;
	max_speed = MAX_SPEED_MODE_1;
	min_speed = MIN_SPEED_MODE_1;
	HAL_Delay(1);
	BSP_MotorControl_SetMaxSpeed(0, max_speed);
	HAL_Delay(1);
	BSP_MotorControl_SetMinSpeed(0, min_speed);
	HAL_Delay(1);
	BSP_MotorControl_SetAcceleration(0, max_accel);
	HAL_Delay(1);
	BSP_MotorControl_SetDeceleration(0, max_decel);
	HAL_Delay(1);

	torq_current_val = MAX_TORQUE_CONFIG;
	L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);

	proportional = PRIMARY_PROPORTIONAL_MODE_1;
	integral = PRIMARY_INTEGRAL_MODE_1;
	derivative = PRIMARY_DERIVATIVE_MODE_1;
	rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_1;
	rotor_i_gain = SECONDARY_INTEGRAL_MODE_1;
	rotor_d_gain = SECONDARY_DERIVATIVE_MODE_1;
	enable_state_feedback = 1;
	integral_compensator_gain = 0;
	feedforward_gain = 1;
	enable_adaptive_mode = 0;

	HAL_UART_Receive_DMA(&huart2, RxBuffer, UART_RX_BUFFER_SIZE);
	BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);
	BSP_MotorControl_AttachErrorHandler(Error_Handler);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	set_mode_strings();

	if (RCC_SYS_CLOCK_FREQ != HAL_RCC_GetSysClockFreq()) {
		sprintf(test_msg,
				"RCC_SYS_CLOCK_FREQ not equal to HAL_RCC_GetSysClockFreq() (%lu). Exiting.\r\n",
				HAL_RCC_GetSysClockFreq());
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}
	if (RCC_HCLK_FREQ != HAL_RCC_GetHCLKFreq()) {
		sprintf(test_msg,
				"RCC_HCLK_FREQ not equal to HAL_RCC_GetHCLKFreq() (%lu). Exiting.\r\n",
				HAL_RCC_GetHCLKFreq());
		HAL_UART_Transmit(&huart2, (uint8_t*) test_msg, strlen(test_msg),
				HAL_MAX_DELAY);
	}

	*deriv_lp_corner_f = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY;
	*deriv_lp_corner_f_rotor = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR;
	t_sample_cpu_cycles = (uint32_t) round(T_SAMPLE_DEFAULT * RCC_HCLK_FREQ);
	Tsample = (float) t_sample_cpu_cycles / RCC_HCLK_FREQ;
	*sample_period = Tsample;
	Tsample_rotor = Tsample;
	*sample_period_rotor = Tsample_rotor;
	assert(RCC_SYS_CLOCK_FREQ == HAL_RCC_GetSysClockFreq());
	assert(RCC_HCLK_FREQ == HAL_RCC_GetHCLKFreq());

	windup = PRIMARY_WINDUP_LIMIT;
	rotor_windup = SECONDARY_WINDUP_LIMIT;

	fo = LP_CORNER_FREQ_ROTOR;
	Wo = 2 * 3.141592654 * fo;
	IWon = 2 / (Wo * Tsample);
	iir_0 = 1 / (1 + IWon);
	iir_1 = iir_0;
	iir_2 = iir_0 * (1 - IWon);
	fo_s = LP_CORNER_FREQ_STEP;
	Wo_s = 2 * 3.141592654 * fo_s;
	IWon_s = 2 / (Wo_s * Tsample);
	iir_0_s = 1 / (1 + IWon_s);
	iir_1_s = iir_0_s;
	iir_2_s = iir_0_s * (1 - IWon_s);
	fo_LT = LP_CORNER_FREQ_LONG_TERM;
	Wo_LT = 2 * 3.141592654 * fo_LT;
	IWon_LT = 2 / (Wo_LT * Tsample);
	iir_LT_0 = 1 / (1 + IWon_LT);
	iir_LT_1 = iir_LT_0;
	iir_LT_2 = iir_LT_0 * (1 - IWon_LT);

	tick_read_cycle_start = HAL_GetTick();
	sprintf(msg, "\n\rSystem Starting Prepare to Enter Mode Selection... ");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	enable_adaptive_mode = ENABLE_ADAPTIVE_MODE;
	adaptive_threshold_low = ADAPTIVE_THRESHOLD_LOW;
	adaptive_threshold_high = ADAPTIVE_THRESHOLD_HIGH;
	adaptive_state = ADAPTIVE_STATE;
	adaptive_state_change = 0;
	adaptive_dwell_period = ADAPTIVE_DWELL_PERIOD;
}
