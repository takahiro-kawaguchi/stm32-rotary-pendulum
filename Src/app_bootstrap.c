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
	ctx->enc_cal.encoder_position = 0;
	ctx->enc_cal.encoder_position_down = 0;
	ctx->angle_scale = ENCODER_READ_ANGLE_SCALE;

	ctx->rotor_pos.rotor_control_target_steps = 0;
	app_reset_command_shaper_state(ctx);
	ctx->core_dual_pid_runtime.current_error_rotor_integral = 0;
	ctx->core_dual_pid_runtime.current_error_rotor_steps = 0;

	ctx->tracking.enable_rotor_chirp = 0;
	ctx->tracking.rotor_chirp_start_freq = ROTOR_CHIRP_START_FREQ;
	ctx->tracking.rotor_chirp_end_freq = ROTOR_CHIRP_END_FREQ;
	ctx->tracking.rotor_chirp_period = ROTOR_CHIRP_PERIOD;
	ctx->tracking.enable_mod_sin_rotor_tracking = ENABLE_MOD_SIN_ROTOR_TRACKING;
	ctx->tracking.enable_rotor_position_step_response_cycle =
			ENABLE_ROTOR_POSITION_STEP_RESPONSE_CYCLE;
	ctx->tracking.disable_mod_sin_rotor_tracking = 0;
	ctx->tracking.sine_drive_transition = 0;
	ctx->tracking.mod_sin_amplitude = MOD_SIN_AMPLITUDE;
	ctx->tracking.rotor_control_sin_amplitude = MOD_SIN_AMPLITUDE;

	ctx->gains.enable_disturbance_rejection_step = 0;
	ctx->gains.enable_noise_rejection_step = 0;
	ctx->gains.enable_sensitivity_fnc_step = 0;
	ctx->tracking.enable_pendulum_position_impulse_response_cycle = 0;

	ctx->adjust_increment = 0.5;
	ctx->mode_transition_state = 0;

	HAL_Init();
	SystemClock_Config();
	ctx->select_suspended_mode = ENABLE_SUSPENDED_PENDULUM_CONTROL;

	BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_L6474, 1);
	BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, motor_init);

	MX_TIM3_Init();
	ctx->enc_cal.encoder_position_init = 0;
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

	ctx->max_accel = MAX_ACCEL;
	ctx->max_decel = MAX_DECEL;
	ctx->max_speed = MAX_SPEED_MODE_1;
	ctx->min_speed = MIN_SPEED_MODE_1;
	HAL_Delay(1);
	BSP_MotorControl_SetMaxSpeed(0, ctx->max_speed);
	HAL_Delay(1);
	BSP_MotorControl_SetMinSpeed(0, ctx->min_speed);
	HAL_Delay(1);
	BSP_MotorControl_SetAcceleration(0, ctx->max_accel);
	HAL_Delay(1);
	BSP_MotorControl_SetDeceleration(0, ctx->max_decel);
	HAL_Delay(1);

	ctx->torq_current_val = MAX_TORQUE_CONFIG;
	L6474_SetAnalogValue(0, L6474_TVAL, ctx->torq_current_val);

	ctx->gains.proportional = PRIMARY_PROPORTIONAL_MODE_1;
	ctx->gains.integral = PRIMARY_INTEGRAL_MODE_1;
	ctx->gains.derivative = PRIMARY_DERIVATIVE_MODE_1;
	ctx->gains.rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_1;
	ctx->gains.rotor_i_gain = SECONDARY_INTEGRAL_MODE_1;
	ctx->gains.rotor_d_gain = SECONDARY_DERIVATIVE_MODE_1;
	ctx->gains.enable_state_feedback = 1;
	ctx->gains.integral_compensator_gain = 0;
	ctx->gains.feedforward_gain = 1;
	ctx->enable_adaptive_mode = 0;

	HAL_UART_Receive_DMA(&huart2, RxBuffer, UART_RX_BUFFER_SIZE);
	BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);
	BSP_MotorControl_AttachErrorHandler(Error_Handler);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	set_mode_strings();

	if (RCC_SYS_CLOCK_FREQ != HAL_RCC_GetSysClockFreq()) {
		sprintf(uart_tx_buf,
				"RCC_SYS_CLOCK_FREQ not equal to HAL_RCC_GetSysClockFreq() (%lu). Exiting.\r\n",
				HAL_RCC_GetSysClockFreq());
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);
	}
	if (RCC_HCLK_FREQ != HAL_RCC_GetHCLKFreq()) {
		sprintf(uart_tx_buf,
				"RCC_HCLK_FREQ not equal to HAL_RCC_GetHCLKFreq() (%lu). Exiting.\r\n",
				HAL_RCC_GetHCLKFreq());
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
				HAL_MAX_DELAY);
	}

	ctx->timing.t_sample_cpu_cycles = (uint32_t) round(T_SAMPLE_DEFAULT * RCC_HCLK_FREQ);
	ctx->timing.t_sample_s = (float) ctx->timing.t_sample_cpu_cycles / RCC_HCLK_FREQ;
	ctx->timing.t_sample_rotor_s = ctx->timing.t_sample_s;
	assert(RCC_SYS_CLOCK_FREQ == HAL_RCC_GetSysClockFreq());
	assert(RCC_HCLK_FREQ == HAL_RCC_GetHCLKFreq());


	{
		float fo, Wo, IWon;
		fo = LP_CORNER_FREQ_ROTOR;
		Wo = 2 * 3.141592654 * fo;
		IWon = 2 / (Wo * ctx->timing.t_sample_s);
		ctx->lpf.iir_0 = 1 / (1 + IWon);
		ctx->lpf.iir_1 = ctx->lpf.iir_0;
		ctx->lpf.iir_2 = ctx->lpf.iir_0 * (1 - IWon);
		fo = LP_CORNER_FREQ_STEP;
		Wo = 2 * 3.141592654 * fo;
		IWon = 2 / (Wo * ctx->timing.t_sample_s);
		ctx->lpf.iir_0_s = 1 / (1 + IWon);
		ctx->lpf.iir_1_s = ctx->lpf.iir_0_s;
		ctx->lpf.iir_2_s = ctx->lpf.iir_0_s * (1 - IWon);
		fo = LP_CORNER_FREQ_LONG_TERM;
		Wo = 2 * 3.141592654 * fo;
		IWon = 2 / (Wo * ctx->timing.t_sample_s);
		ctx->lpf.iir_LT_0 = 1 / (1 + IWon);
		ctx->lpf.iir_LT_1 = ctx->lpf.iir_LT_0;
		ctx->lpf.iir_LT_2 = ctx->lpf.iir_LT_0 * (1 - IWon);
	}

	ctx->timing.tick_read_cycle_start = HAL_GetTick();
	sprintf(uart_tx_buf, "\n\rSystem Starting Prepare to Enter Mode Selection... ");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

	ctx->enable_adaptive_mode = ENABLE_ADAPTIVE_MODE;
	ctx->adaptive_state = ADAPTIVE_STATE;
	ctx->adaptive_state_change = 0;
}
