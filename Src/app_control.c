#include "main.h"
#include "edukit_system.h"
#include "hardware.h"
#include "ui.h"
#include "app_control.h"
#include <stdio.h>
#include <string.h>

__STATIC_INLINE void DWT_Delay_until_cycle(volatile uint32_t cycle)
{
	while (DWT->CYCCNT < cycle);
}

void app_reset_command_shaper_state(AppControlContext *ctx)
{
	ctx->core_cmd_shaper_state.rotor_control_target_steps_prev = 0.0f;
	ctx->core_cmd_shaper_state.rotor_control_target_steps_prev_prev = 0.0f;
	ctx->core_cmd_shaper_state.rotor_control_target_steps_filter_2 = 0.0f;
	ctx->core_cmd_shaper_state.rotor_control_target_steps_filter_prev_2 = 0.0f;
	ctx->core_cmd_shaper_state.rotor_control_target_steps_filter_prev_prev_2 = 0.0f;
	ctx->core_cmd_shaper_state.rotor_control_target_steps_gain = 0.0f;
	ctx->core_cmd_shaper_state.rotor_position_command_steps_prev = 0.0f;
}

void app_assign_pid_gains_from_user(AppControlContext *ctx)
{
	ctx->core_ctl_state.PID_Pend.Kp = ctx->gains.proportional * CONTROLLER_GAIN_SCALE;
	ctx->core_ctl_state.PID_Pend.Ki = ctx->gains.integral * CONTROLLER_GAIN_SCALE;
	ctx->core_ctl_state.PID_Pend.Kd = ctx->gains.derivative * CONTROLLER_GAIN_SCALE;

	ctx->core_ctl_state.PID_Rotor.Kp = ctx->gains.rotor_p_gain * CONTROLLER_GAIN_SCALE;
	ctx->core_ctl_state.PID_Rotor.Ki = ctx->gains.rotor_i_gain * CONTROLLER_GAIN_SCALE;
	ctx->core_ctl_state.PID_Rotor.Kd = ctx->gains.rotor_d_gain * CONTROLLER_GAIN_SCALE;

	ctx->core_ctl_state.PID_Pend.state_a[0] = 0.0f;
	ctx->core_ctl_state.PID_Pend.state_a[1] = 0.0f;
	ctx->core_ctl_state.PID_Pend.state_a[2] = 0.0f;
	ctx->core_ctl_state.PID_Pend.state_a[3] = 0.0f;

	ctx->core_ctl_state.PID_Rotor.state_a[0] = 0.0f;
	ctx->core_ctl_state.PID_Rotor.state_a[1] = 0.0f;
	ctx->core_ctl_state.PID_Rotor.state_a[2] = 0.0f;
	ctx->core_ctl_state.PID_Rotor.state_a[3] = 0.0f;
}

void app_init_control_pipeline(AppControlContext *ctx, int encoder_init_counts,
		float sample_period_s)
{
	PidGains active_gains;

	hardware_init(&htim3, encoder_init_counts);
	ctx->core_observer_ops->init(&ctx->core_obs_state, sample_period_s);

	active_gains.Kp_pend = ctx->core_ctl_state.PID_Pend.Kp;
	active_gains.Ki_pend = ctx->core_ctl_state.PID_Pend.Ki;
	active_gains.Kd_pend = ctx->core_ctl_state.PID_Pend.Kd;
	active_gains.Kp_rotor = ctx->core_ctl_state.PID_Rotor.Kp;
	active_gains.Ki_rotor = ctx->core_ctl_state.PID_Rotor.Ki;
	active_gains.Kd_rotor = ctx->core_ctl_state.PID_Rotor.Kd;

	ctx->core_controller_ops->init(&ctx->core_ctl_state, &active_gains,
			sample_period_s);
}

void control_shutdown_sequence(AppControlContext *ctx)
{
	if (ACCEL_CONTROL == 1) {
		desired_pwm_period = 0;
		current_pwm_period = 0;
	}

	hardware_sensor_read(&ctx->core_hw_raw, NULL);
	ctx->rotor_pos.rotor_position_steps = ctx->core_hw_raw.rotor_steps;
	BSP_MotorControl_GoTo(0, 0);
	BSP_MotorControl_SoftStop(0);

	hardware_sensor_read(&ctx->core_hw_raw, NULL);
	ctx->rotor_pos.rotor_position_steps = ctx->core_hw_raw.rotor_steps;
	sprintf(uart_tx_buf, "Exit Control at Rotor Angle, %.2f\r\n",
			(float) ((ctx->rotor_pos.rotor_position_steps) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

	NVIC_SystemReset();
}

int control_handle_runtime_configuration(AppControlContext *ctx, int i)
{
	if (ctx->enable_swing_up == 1 && i == SWING_UP_CONTROL_CONFIG_DELAY && ctx->enc_cal.enable_angle_cal == 0) {
		ctx->core_ctl_state.PID_Rotor.Kp = ctx->init_params.Kp_rotor;
		ctx->core_ctl_state.PID_Rotor.Ki = ctx->init_params.Ki_rotor;
		ctx->core_ctl_state.PID_Rotor.Kd = ctx->init_params.Kd_rotor;
		ctx->core_ctl_state.PID_Pend.Kp  = ctx->init_params.Kp_pend;
		ctx->core_ctl_state.PID_Pend.Ki  = ctx->init_params.Ki_pend;
		ctx->core_ctl_state.PID_Pend.Kd  = ctx->init_params.Kd_pend;
		ctx->gains.enable_state_feedback             = ctx->init_params.enable_state_feedback;
		ctx->gains.integral_compensator_gain         = ctx->init_params.integral_compensator_gain;
		ctx->gains.feedforward_gain                  = ctx->init_params.feedforward_gain;
		ctx->gains.enable_disturbance_rejection_step = ctx->init_params.enable_disturbance_rejection_step;
		ctx->gains.enable_sensitivity_fnc_step       = ctx->init_params.enable_sensitivity_fnc_step;
		ctx->gains.enable_noise_rejection_step       = ctx->init_params.enable_noise_rejection_step;
	}

	int ui_status = ui_process_runtime_input(i, ctx, &ctx->core_ctl_state.PID_Pend,
			&ctx->core_ctl_state.PID_Rotor);
	if (ui_status != 0) {
		return ui_status;
	}

	if (i > ctx->timing.cycle_count && ENABLE_CYCLE_INFINITE == 0) {
		return -1;
	}

	return 0;
}

int control_update_state_and_safety(AppControlContext *ctx)
{
	ctx->core_hw_cal.encoder_down_counts = ctx->enc_cal.encoder_position_down;
	ctx->core_hw_cal.encoder_offset_counts = (float) ctx->enc_cal.encoder_position_offset;
	ctx->core_hw_cal.select_suspended_mode = ctx->select_suspended_mode;

	hardware_sensor_read(&ctx->core_hw_raw, NULL);
	ctx->core_observer_ops->update(&ctx->core_hw_raw, &ctx->core_hw_cal,
			&ctx->core_obs_state, &ctx->core_sys_state);

	ctx->enc_cal.encoder_position_steps = ctx->core_hw_raw.encoder_counts;
	ctx->rotor_pos.rotor_position_steps = ctx->core_hw_raw.rotor_steps;
	ctx->enc_cal.encoder_position = (int) (ctx->core_sys_state.pendulum_angle_rad / ENCODER_RAD_PER_COUNT);

	if (ctx->select_suspended_mode == 0) {
		if ((ctx->enc_cal.encoder_position / ENCODER_READ_ANGLE_SCALE) > ENCODER_POSITION_POSITIVE_LIMIT
				|| (ctx->enc_cal.encoder_position / ENCODER_READ_ANGLE_SCALE) < ENCODER_POSITION_NEGATIVE_LIMIT) {
			sprintf(uart_tx_buf, "Error Exit Encoder Position Exceeded: %i\r\n",
					ctx->enc_cal.encoder_position_steps);
			HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
			return 1;
		}
	}

	if (ctx->rotor_pos.rotor_position_steps > (ROTOR_POSITION_POSITIVE_LIMIT
			* STEPPER_READ_POSITION_STEPS_PER_DEGREE)
			|| ctx->rotor_pos.rotor_position_steps < (ROTOR_POSITION_NEGATIVE_LIMIT
					* STEPPER_READ_POSITION_STEPS_PER_DEGREE)) {
		sprintf(uart_tx_buf, "Error Exit Motor Position Exceeded: %i\r\n", ctx->rotor_pos.rotor_position_steps);
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
		return 1;
	}

	return 0;
}

void control_update_slope_correction(AppControlContext *ctx, int i)
{
	ctx->rotor_pos.rotor_position_diff_prev = ctx->rotor_pos.rotor_position_diff;

	if (ctx->gains.enable_disturbance_rejection_step == 0) {
		ctx->rotor_pos.rotor_position_diff = ctx->rotor_pos.rotor_position_filter_steps - ctx->rotor_pos.rotor_position_command_steps;
	}
	if (ctx->gains.enable_disturbance_rejection_step == 1) {
		ctx->rotor_pos.rotor_position_diff = ctx->rotor_pos.rotor_position_filter_steps;
	}

	if (ENABLE_ENCODER_ANGLE_SLOPE_CORRECTION == 1 && i > ctx->enc_cal.angle_cal_complete) {
		ctx->rotor_pos.rotor_position_diff_filter =
				(float) (ctx->rotor_pos.rotor_position_diff * ctx->lpf.iir_LT_0) + ctx->rotor_pos.rotor_position_diff_prev * ctx->lpf.iir_LT_1
						- ctx->rotor_pos.rotor_position_diff_filter_prev * ctx->lpf.iir_LT_2;
		if ((i < ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT)
				|| (ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT == 0)) {
			ctx->enc_cal.encoder_angle_slope_corr_steps =
					ctx->rotor_pos.rotor_position_diff_filter / ENCODER_ANGLE_SLOPE_CORRECTION_SCALE;
		}
		ctx->rotor_pos.rotor_position_diff_filter_prev = ctx->rotor_pos.rotor_position_diff_filter;
	}
}

void control_update_dual_pid(AppControlContext *ctx)
{
	ControllerDualPidInput input;

	input.rotor_position_filter_steps = ctx->rotor_pos.rotor_position_filter_steps;
	input.rotor_position_command_steps = ctx->rotor_pos.rotor_position_command_steps;
	input.feedforward_gain = ctx->gains.feedforward_gain;
	input.integral_compensator_gain = ctx->gains.integral_compensator_gain;
	input.load_disturbance_sensitivity_scale = ctx->gains.load_disturbance_sensitivity_scale;
	input.sample_period_rotor_s = ctx->timing.t_sample_rotor_s;
	input.enable_state_feedback = ctx->gains.enable_state_feedback;
	input.enable_disturbance_rejection_step = ctx->gains.enable_disturbance_rejection_step;
	input.enable_sensitivity_fnc_step = ctx->gains.enable_sensitivity_fnc_step;
	input.enable_noise_rejection_step = ctx->gains.enable_noise_rejection_step;

	if (ctx->core_controller_ops->compute_dual != NULL) {
		ctx->core_controller_ops->compute_dual(&ctx->core_ctl_state,
				&ctx->core_sys_state, &ctx->core_ctl_target, &input,
				&ctx->core_dual_pid_runtime, &ctx->core_ctl_out);
	} else {
		ctx->core_ctl_target.rotor_angle_ref_rad = ctx->core_sys_state.rotor_angle_rad
				- ctx->core_dual_pid_runtime.current_error_rotor_steps * STEPPER_RAD_PER_STEP;
		ctx->core_controller_ops->compute(&ctx->core_ctl_state,
				&ctx->core_sys_state, &ctx->core_ctl_target, &ctx->core_ctl_out);
	}

	ctx->rotor_pos.rotor_control_target_steps = ctx->core_ctl_out.rotor_accel_steps_s2;
}

void control_finalize_command_and_actuate(AppControlContext *ctx, int i)
{
	CommandShaperConfig shaper_cfg;
	shaper_cfg.sample_period_s = ctx->timing.t_sample_s;
	shaper_cfg.accel_control = ACCEL_CONTROL;
	shaper_cfg.angle_cal_complete = ctx->enc_cal.angle_cal_complete;
	shaper_cfg.full_sysid_start_index = -1;
	shaper_cfg.full_sysid_max_vel_amplitude_deg_per_s = 0.0f;
	shaper_cfg.full_sysid_min_freq_hz = 0.0f;
	shaper_cfg.full_sysid_num_freqs = 0;
	shaper_cfg.full_sysid_freq_log_step = 0.0f;
	shaper_cfg.enable_rotor_plant_design = 0;
	shaper_cfg.enable_rotor_plant_gain_design = 0;
	shaper_cfg.rotor_plant_gain = 0.0f;
	shaper_cfg.rotor_damping_coefficient = 0.0f;
	shaper_cfg.rotor_natural_frequency = 0.0f;
	shaper_cfg.c0 = 0.0f;
	shaper_cfg.c1 = 0.0f;
	shaper_cfg.c2 = 0.0f;
	shaper_cfg.c3 = 0.0f;
	shaper_cfg.c4 = 0.0f;
	shaper_cfg.iir_0_r = 0.0f;
	shaper_cfg.iir_1_r = 0.0f;
	shaper_cfg.iir_2_r = 0.0f;

	ctx->core_command_shaper_ops->process_and_actuate(&shaper_cfg, i,
			ctx->rotor_pos.rotor_position_command_steps, &ctx->rotor_pos.rotor_control_target_steps,
			&ctx->core_cmd_shaper_state, &ctx->core_ctl_out);
}

int control_wait_next_cycle(AppControlContext *ctx)
{
	ctx->timing.prev_target_cpu_cycle = ctx->timing.target_cpu_cycle;
	ctx->timing.target_cpu_cycle += ctx->timing.t_sample_cpu_cycles;

	ctx->timing.current_cpu_cycle = DWT->CYCCNT;

	if (((int) (ctx->timing.target_cpu_cycle - ctx->timing.current_cpu_cycle)) > 0) {
		if (ctx->timing.current_cpu_cycle > ctx->timing.target_cpu_cycle) {
			do {
				ctx->timing.last_cpu_cycle = ctx->timing.current_cpu_cycle;
				ctx->timing.current_cpu_cycle = DWT->CYCCNT;
			} while (ctx->timing.current_cpu_cycle >= ctx->timing.last_cpu_cycle);
		}
		DWT_Delay_until_cycle(ctx->timing.target_cpu_cycle);
	} else if (ctx->timing.current_cpu_cycle - ctx->timing.target_cpu_cycle > ctx->timing.t_sample_cpu_cycles * 5
			&& ctx->timing.enable_cycle_delay_warning == 1) {
		sprintf(uart_tx_buf, "Error: control loop lag\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
		return 1;
	}

	ctx->timing.current_cpu_cycle = DWT->CYCCNT;
	ctx->timing.current_cpu_cycle_delay_relative_report =
			(int) (ctx->timing.t_sample_cpu_cycles - (ctx->timing.current_cpu_cycle - ctx->timing.prev_cpu_cycle));
	ctx->timing.current_cpu_cycle_delay_relative_report =
			(ctx->timing.current_cpu_cycle_delay_relative_report * 1000000) / RCC_HCLK_FREQ;
	ctx->timing.prev_cpu_cycle = ctx->timing.current_cpu_cycle;

	return 0;
}
