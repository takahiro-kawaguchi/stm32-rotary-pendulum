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
	ctx->core_ctl_state.PID_Pend.Kp = proportional * CONTROLLER_GAIN_SCALE;
	ctx->core_ctl_state.PID_Pend.Ki = integral * CONTROLLER_GAIN_SCALE;
	ctx->core_ctl_state.PID_Pend.Kd = derivative * CONTROLLER_GAIN_SCALE;

	ctx->core_ctl_state.PID_Rotor.Kp = rotor_p_gain * CONTROLLER_GAIN_SCALE;
	ctx->core_ctl_state.PID_Rotor.Ki = rotor_i_gain * CONTROLLER_GAIN_SCALE;
	ctx->core_ctl_state.PID_Rotor.Kd = rotor_d_gain * CONTROLLER_GAIN_SCALE;

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
	rotor_position_steps = ctx->core_hw_raw.rotor_steps;
	BSP_MotorControl_GoTo(0, 0);
	BSP_MotorControl_SoftStop(0);

	hardware_sensor_read(&ctx->core_hw_raw, NULL);
	rotor_position_steps = ctx->core_hw_raw.rotor_steps;
	sprintf(msg, "Exit Control at Rotor Angle, %.2f\r\n",
			(float) ((rotor_position_steps) / STEPPER_READ_POSITION_STEPS_PER_DEGREE));
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	NVIC_SystemReset();
}

int control_handle_runtime_configuration(AppControlContext *ctx, int i)
{
	if (enable_swing_up == 1 && i == SWING_UP_CONTROL_CONFIG_DELAY && enable_angle_cal == 0) {
		ctx->core_ctl_state.PID_Rotor.Kp = init_r_p_gain;
		ctx->core_ctl_state.PID_Rotor.Ki = init_r_i_gain;
		ctx->core_ctl_state.PID_Rotor.Kd = init_r_d_gain;
		ctx->core_ctl_state.PID_Pend.Kp = init_p_p_gain;
		ctx->core_ctl_state.PID_Pend.Ki = init_p_i_gain;
		ctx->core_ctl_state.PID_Pend.Kd = init_p_d_gain;
		enable_state_feedback = init_enable_state_feedback;
		integral_compensator_gain = init_integral_compensator_gain;
		feedforward_gain = init_feedforward_gain;
		enable_state_feedback = init_enable_state_feedback;
		enable_disturbance_rejection_step = init_enable_disturbance_rejection_step;
		enable_sensitivity_fnc_step = init_enable_sensitivity_fnc_step;
		enable_noise_rejection_step = init_enable_noise_rejection_step;
		enable_rotor_plant_design = init_enable_rotor_plant_design;
		enable_rotor_plant_gain_design = init_enable_rotor_plant_gain_design;
	}

	int ui_status = ui_process_runtime_input(i, &ctx->core_ctl_state.PID_Pend,
			&ctx->core_ctl_state.PID_Rotor);
	if (ui_status != 0) {
		return ui_status;
	}

	if (i > cycle_count && ENABLE_CYCLE_INFINITE == 0) {
		return -1;
	}

	return 0;
}

int control_update_state_and_safety(AppControlContext *ctx)
{
	ctx->core_hw_cal.encoder_down_counts = encoder_position_down;
	ctx->core_hw_cal.encoder_offset_counts = (float) encoder_position_offset;
	ctx->core_hw_cal.select_suspended_mode = select_suspended_mode;

	hardware_sensor_read(&ctx->core_hw_raw, NULL);
	ctx->core_observer_ops->update(&ctx->core_hw_raw, &ctx->core_hw_cal,
			&ctx->core_obs_state, &ctx->core_sys_state);

	encoder_position_steps = ctx->core_hw_raw.encoder_counts;
	rotor_position_steps = ctx->core_hw_raw.rotor_steps;
	encoder_position = (int) (ctx->core_sys_state.pendulum_angle_rad / ENCODER_RAD_PER_COUNT);

	if (select_suspended_mode == 0) {
		if ((encoder_position / ENCODER_READ_ANGLE_SCALE) > ENCODER_POSITION_POSITIVE_LIMIT
				|| (encoder_position / ENCODER_READ_ANGLE_SCALE) < ENCODER_POSITION_NEGATIVE_LIMIT) {
			sprintf(msg, "Error Exit Encoder Position Exceeded: %i\r\n",
					encoder_position_steps);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
			return 1;
		}
	}

	if (rotor_position_steps > (ROTOR_POSITION_POSITIVE_LIMIT
			* STEPPER_READ_POSITION_STEPS_PER_DEGREE)
			|| rotor_position_steps < (ROTOR_POSITION_NEGATIVE_LIMIT
					* STEPPER_READ_POSITION_STEPS_PER_DEGREE)) {
		sprintf(msg, "Error Exit Motor Position Exceeded: %i\r\n", rotor_position_steps);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		return 1;
	}

	return 0;
}

void control_update_slope_correction(int i)
{
	rotor_position_diff_prev = rotor_position_diff;

	if (enable_disturbance_rejection_step == 0) {
		rotor_position_diff = rotor_position_filter_steps - rotor_position_command_steps;
	}
	if (enable_disturbance_rejection_step == 1) {
		rotor_position_diff = rotor_position_filter_steps;
	}

	if (ENABLE_ENCODER_ANGLE_SLOPE_CORRECTION == 1 && i > angle_cal_complete) {
		rotor_position_diff_filter =
				(float) (rotor_position_diff * iir_LT_0) + rotor_position_diff_prev * iir_LT_1
						- rotor_position_diff_filter_prev * iir_LT_2;
		if ((i < ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT)
				|| (ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT == 0)) {
			encoder_angle_slope_corr_steps =
					rotor_position_diff_filter / ENCODER_ANGLE_SLOPE_CORRECTION_SCALE;
		}
		rotor_position_diff_filter_prev = rotor_position_diff_filter;
	}
}

void control_update_dual_pid(AppControlContext *ctx)
{
	ControllerDualPidInput input;
	ControllerDualPidRuntime runtime;

	input.rotor_position_filter_steps = rotor_position_filter_steps;
	input.rotor_position_command_steps = rotor_position_command_steps;
	input.feedforward_gain = feedforward_gain;
	input.integral_compensator_gain = integral_compensator_gain;
	input.load_disturbance_sensitivity_scale = load_disturbance_sensitivity_scale;
	input.sample_period_rotor_s = *sample_period_rotor;
	input.enable_state_feedback = enable_state_feedback;
	input.enable_disturbance_rejection_step = enable_disturbance_rejection_step;
	input.enable_sensitivity_fnc_step = enable_sensitivity_fnc_step;
	input.enable_noise_rejection_step = enable_noise_rejection_step;

	runtime.current_error_rotor_steps = *current_error_rotor_steps;
	runtime.current_error_rotor_integral = current_error_rotor_integral;

	if (ctx->core_controller_ops->compute_dual != NULL) {
		ctx->core_controller_ops->compute_dual(&ctx->core_ctl_state,
				&ctx->core_sys_state, &ctx->core_ctl_target, &input, &runtime,
				&ctx->core_ctl_out);
	} else {
		ctx->core_ctl_target.rotor_angle_ref_rad = ctx->core_sys_state.rotor_angle_rad
				- (*current_error_rotor_steps) * STEPPER_RAD_PER_STEP;
		ctx->core_controller_ops->compute(&ctx->core_ctl_state,
				&ctx->core_sys_state, &ctx->core_ctl_target, &ctx->core_ctl_out);
	}

	*current_error_rotor_steps = runtime.current_error_rotor_steps;
	current_error_rotor_integral = runtime.current_error_rotor_integral;
	rotor_control_target_steps = ctx->core_ctl_out.rotor_accel_steps_s2;
}

void control_finalize_command_and_actuate(AppControlContext *ctx, int i)
{
	CommandShaperConfig shaper_cfg;
	shaper_cfg.sample_period_s = Tsample;
	shaper_cfg.accel_control = ACCEL_CONTROL;
	shaper_cfg.angle_cal_complete = angle_cal_complete;
	shaper_cfg.full_sysid_start_index = full_sysid_start_index;
	shaper_cfg.full_sysid_max_vel_amplitude_deg_per_s = full_sysid_max_vel_amplitude_deg_per_s;
	shaper_cfg.full_sysid_min_freq_hz = full_sysid_min_freq_hz;
	shaper_cfg.full_sysid_num_freqs = full_sysid_num_freqs;
	shaper_cfg.full_sysid_freq_log_step = full_sysid_freq_log_step;
	shaper_cfg.enable_rotor_plant_design = enable_rotor_plant_design;
	shaper_cfg.enable_rotor_plant_gain_design = enable_rotor_plant_gain_design;
	shaper_cfg.rotor_plant_gain = rotor_plant_gain;
	shaper_cfg.rotor_damping_coefficient = rotor_damping_coefficient;
	shaper_cfg.rotor_natural_frequency = rotor_natural_frequency;
	shaper_cfg.c0 = c0;
	shaper_cfg.c1 = c1;
	shaper_cfg.c2 = c2;
	shaper_cfg.c3 = c3;
	shaper_cfg.c4 = c4;
	shaper_cfg.iir_0_r = iir_0_r;
	shaper_cfg.iir_1_r = iir_1_r;
	shaper_cfg.iir_2_r = iir_2_r;

	ctx->core_command_shaper_ops->process_and_actuate(&shaper_cfg, i,
			rotor_position_command_steps, &rotor_control_target_steps,
			&ctx->core_cmd_shaper_state, &ctx->core_ctl_out);
}

int control_wait_next_cycle(void)
{
	prev_target_cpu_cycle = target_cpu_cycle;
	target_cpu_cycle += t_sample_cpu_cycles;

	current_cpu_cycle = DWT->CYCCNT;

	if (((int) (target_cpu_cycle - current_cpu_cycle)) > 0) {
		if (current_cpu_cycle > target_cpu_cycle) {
			do {
				last_cpu_cycle = current_cpu_cycle;
				current_cpu_cycle = DWT->CYCCNT;
			} while (current_cpu_cycle >= last_cpu_cycle);
		}
		DWT_Delay_until_cycle(target_cpu_cycle);
	} else if (current_cpu_cycle - target_cpu_cycle > t_sample_cpu_cycles * 5
			&& enable_cycle_delay_warning == 1) {
		sprintf(msg, "Error: control loop lag\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		return 1;
	}

	current_cpu_cycle = DWT->CYCCNT;
	current_cpu_cycle_delay_relative_report =
			(int) (t_sample_cpu_cycles - (current_cpu_cycle - prev_cpu_cycle));
	current_cpu_cycle_delay_relative_report =
			(current_cpu_cycle_delay_relative_report * 1000000) / RCC_HCLK_FREQ;
	prev_cpu_cycle = current_cpu_cycle;

	return 0;
}
