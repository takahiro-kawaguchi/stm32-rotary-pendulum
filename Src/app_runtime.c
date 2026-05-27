#include "main.h"
#include "edukit_system.h"
#include "app_control.h"
#include "app_runtime.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int display_parameter;

static void reference_update(AppControlContext *ctx, int i);
static void angle_cal_update(AppControlContext *ctx, int i);
static void report_data(AppControlContext *ctx, int i);
static void control_prepare_targets_and_filters(AppControlContext *ctx, int i);

int control_execute_cycle(AppControlContext *ctx, int i)
{
	if (control_update_state_and_safety(ctx) != 0) {
		return 1;
	}

	control_update_slope_correction(i);
	control_prepare_targets_and_filters(ctx, i);

	if (ENABLE_DUAL_PID == 1) {
		control_update_dual_pid(ctx);
	}
	control_finalize_command_and_actuate(ctx, i);
	report_data(ctx, i);

	return control_wait_next_cycle();
}

static void control_prepare_targets_and_filters(AppControlContext *ctx, int i)
{
	if ((i > ENCODER_START_OFFSET_DELAY) || (ENCODER_START_OFFSET_DELAY == 0)) {
		encoder_position = encoder_position - ENCODER_START_OFFSET;
	}

	*current_error_steps = encoder_angle_slope_corr_steps
			+ ENCODER_ANGLE_POLARITY
					* (encoder_position / ((float) (ENCODER_READ_ANGLE_SCALE
							/ STEPPER_READ_POSITION_STEPS_PER_DEGREE)));
	*current_error_steps = *current_error_steps + pendulum_position_command_steps;

	ctx->core_ctl_target.slope_correction_steps = encoder_angle_slope_corr_steps;
	ctx->core_ctl_target.pendulum_cmd_steps = pendulum_position_command_steps;
	ctx->core_ctl_target.pendulum_angle_ref_rad = 0.0f;

	rotor_position_filter_steps = (float) (rotor_position_steps) * iir_0
			+ rotor_position_steps_prev * iir_1
			- rotor_position_filter_steps_prev * iir_2;
	rotor_position_steps_prev = (float) (rotor_position_steps);
	rotor_position_filter_steps_prev = rotor_position_filter_steps;

	rotor_position_filter_steps = rotor_position_steps;

	reference_update(ctx, i);
	angle_cal_update(ctx, i);
}

static void reference_update(AppControlContext *ctx, int i)
{
	if (enable_rotor_chirp == 1 && enable_mod_sin_rotor_tracking == 0
			&& enable_rotor_tracking_comb_signal == 0 && i > angle_cal_complete) {

		if (i < ROTOR_CHIRP_PERIOD - 1) {
			chirp_cycle = 0;
		}
		if (chirp_cycle > ROTOR_CHIRP_PERIOD - 1) {
			chirp_cycle = 0;
			chirp_dwell_cycle = ROTOR_CHIRP_SWEEP_DELAY;
		}
		if (chirp_dwell_cycle > 0) {
			chirp_dwell_cycle--;
			chirp_cycle = 0;
		}
		if (chirp_dwell_cycle == 0 && i >= ROTOR_CHIRP_PERIOD - 1) {
			chirp_cycle = chirp_cycle + 1;
			chirp_time = (float) ((chirp_cycle - 1) / ROTOR_CHIRP_SAMPLE_RATE);
			rotor_chirp_frequency = rotor_chirp_start_freq
					+ (rotor_chirp_end_freq - rotor_chirp_start_freq)
							* ((float) (chirp_cycle / rotor_chirp_period));
			rotor_position_command_steps =
					((float) (ROTOR_CHIRP_STEP_AMPLITUDE
							* STEPPER_READ_POSITION_STEPS_PER_DEGREE))
							* sin(2.0 * 3.14159 * rotor_chirp_frequency * chirp_time);
		}
	}

	if (enable_rotor_tracking_comb_signal > 0 && i > 1000 && i > angle_cal_complete) {
		chirp_time = ((float) (i - 1)) / 500.0;
		rotor_track_comb_signal_frequency = 0.01;
		rotor_track_comb_command = ((float) (rotor_track_comb_amplitude))
				* sin(rotor_track_comb_signal_frequency * chirp_time);
		rotor_track_comb_signal_frequency = 0.017783;
		rotor_track_comb_command = rotor_track_comb_command
				+ ((float) (rotor_track_comb_amplitude))
						* sin(rotor_track_comb_signal_frequency * chirp_time);
		rotor_track_comb_signal_frequency = 0.031623;
		rotor_track_comb_command = rotor_track_comb_command
				+ ((float) (rotor_track_comb_amplitude))
						* sin(rotor_track_comb_signal_frequency * chirp_time);
		rotor_track_comb_signal_frequency = 0.056234;
		rotor_track_comb_command = rotor_track_comb_command
				+ ((float) (rotor_track_comb_amplitude))
						* sin(rotor_track_comb_signal_frequency * chirp_time);
		rotor_track_comb_signal_frequency = 0.1;
		rotor_track_comb_command = rotor_track_comb_command
				+ ((float) (rotor_track_comb_amplitude))
						* sin(rotor_track_comb_signal_frequency * chirp_time);
		rotor_track_comb_signal_frequency = 0.17783;
		rotor_track_comb_command = rotor_track_comb_command
				+ ((float) (rotor_track_comb_amplitude))
						* sin(rotor_track_comb_signal_frequency * chirp_time);
		rotor_track_comb_signal_frequency = 0.31623;
		rotor_track_comb_command = rotor_track_comb_command
				+ ((float) (rotor_track_comb_amplitude))
						* sin(rotor_track_comb_signal_frequency * chirp_time);
		rotor_track_comb_signal_frequency = 0.56234;
		rotor_track_comb_command = rotor_track_comb_command
				+ ((float) (rotor_track_comb_amplitude))
						* sin(rotor_track_comb_signal_frequency * chirp_time);
		rotor_track_comb_signal_frequency = 1.0;
		rotor_track_comb_command = rotor_track_comb_command
				+ ((float) (rotor_track_comb_amplitude))
						* sin(rotor_track_comb_signal_frequency * chirp_time);
		rotor_track_comb_signal_frequency = 1.7783;
		rotor_track_comb_command = rotor_track_comb_command
				+ ((float) (rotor_track_comb_amplitude))
						* sin(rotor_track_comb_signal_frequency * chirp_time);
		rotor_track_comb_signal_frequency = 3.1623;
		rotor_track_comb_command = rotor_track_comb_command
				+ ((float) (rotor_track_comb_amplitude))
						* sin(rotor_track_comb_signal_frequency * chirp_time);
		rotor_track_comb_signal_frequency = 5.6234;
		rotor_track_comb_command = rotor_track_comb_command
				+ ((float) (rotor_track_comb_amplitude))
						* sin(rotor_track_comb_signal_frequency * chirp_time);
		rotor_track_comb_signal_frequency = 10;
		rotor_track_comb_command = rotor_track_comb_command
				+ ((float) (rotor_track_comb_amplitude))
						* sin(rotor_track_comb_signal_frequency * chirp_time);
	}
	if (enable_rotor_chirp == 0 && enable_mod_sin_rotor_tracking == 0
			&& enable_rotor_tracking_comb_signal == 1) {
		rotor_position_command_steps = rotor_track_comb_command;
	}

	rotor_sine_drive = 0;
	if (enable_mod_sin_rotor_tracking == 1 && ENABLE_ROTOR_CHIRP == 0
			&& i > angle_cal_complete) {
		if (ENABLE_ROTOR_CHIRP == 0) {
			mod_sin_carrier_frequency = MOD_SIN_CARRIER_FREQ;
		}
		if (i > MOD_SIN_START_CYCLES && enable_mod_sin_rotor_tracking == 1) {
			rotor_sine_drive = (float) (mod_sin_amplitude * (1 + sin(
					-1.5707
							+ ((i - MOD_SIN_START_CYCLES) / MOD_SIN_SAMPLE_RATE)
									* (MOD_SIN_MODULATION_FREQ * 6.2832))));
			rotor_sine_drive_mod = sin(
					0
							+ ((i - MOD_SIN_START_CYCLES) / MOD_SIN_SAMPLE_RATE)
									* (mod_sin_carrier_frequency * 6.2832));
			rotor_sine_drive = rotor_sine_drive + MOD_SIN_MODULATION_MIN;
			rotor_sine_drive = rotor_sine_drive * rotor_sine_drive_mod
					* rotor_mod_control;
		}
		if (i > MOD_SIN_START_CYCLES && ENABLE_SIN_MOD == 0) {
			rotor_sine_drive_mod = sin(
					0
							+ ((i - MOD_SIN_START_CYCLES) / MOD_SIN_SAMPLE_RATE)
									* (mod_sin_carrier_frequency * 6.2832));
			rotor_sine_drive = rotor_control_sin_amplitude * rotor_sine_drive_mod
					* rotor_mod_control;
		}
		if (fabs(rotor_sine_drive_mod * MOD_SIN_AMPLITUDE) < 2
				&& disable_mod_sin_rotor_tracking == 1
				&& sine_drive_transition == 1) {
			rotor_mod_control = 0.0;
			sine_drive_transition = 0;
		}
		if (fabs(rotor_sine_drive_mod * MOD_SIN_AMPLITUDE) < 2
				&& disable_mod_sin_rotor_tracking == 0
				&& sine_drive_transition == 1) {
			rotor_mod_control = 1.0;
			sine_drive_transition = 0;
		}
		if (enable_rotor_position_step_response_cycle == 0) {
			rotor_position_command_steps = rotor_sine_drive;
		}
	}

	if (ENABLE_ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE == 1 && i != 0
			&& i > angle_cal_complete) {
		if ((i % ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL) == 0) {
			rotor_position_command_steps =
					(float) (ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE
							* STEPPER_READ_POSITION_STEPS_PER_DEGREE);
			impulse_start_index = 0;
		}
		if (impulse_start_index > ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD) {
			rotor_position_command_steps = 0;
		}
		impulse_start_index++;
	}

	if (enable_pendulum_position_impulse_response_cycle == 1 && i != 0
			&& i > angle_cal_complete) {
		if ((i % PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL) == 0) {
			if (select_suspended_mode == 1) {
				pendulum_position_command_steps =
						(float) PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE;
			}
			if (select_suspended_mode == 0) {
				pendulum_position_command_steps =
						(float) (PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE
								/ PENDULUM_POSITION_IMPULSE_AMPLITUDE_SCALE);
			}
			chirp_cycle = 0;
			impulse_start_index = 0;
		}
		if (impulse_start_index > PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD) {
			pendulum_position_command_steps = 0;
		}
		impulse_start_index++;
		chirp_cycle++;
	}

	if ((i % ROTOR_POSITION_STEP_RESPONSE_CYCLE_INTERVAL) == 0
			&& enable_rotor_position_step_response_cycle == 1
			&& i > angle_cal_complete) {
		rotor_position_step_polarity = -rotor_position_step_polarity;
		if (rotor_position_step_polarity == 1) {
			chirp_cycle = 0;
		}
	}
	if (enable_rotor_position_step_response_cycle == 1
			&& enable_rotor_tracking_comb_signal == 0 && i > angle_cal_complete) {
		if (STEP_RESPONSE_AMP_LIMIT_ENABLE == 1
				&& fabsf(rotor_sine_drive) > STEP_RESPONSE_AMP_LIMIT) {
			chirp_cycle = chirp_cycle + 1;
		} else {
			if (enable_mod_sin_rotor_tracking == 1) {
				rotor_position_command_steps = rotor_sine_drive + (float) ((rotor_position_step_polarity)
						* ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE
						* STEPPER_READ_POSITION_STEPS_PER_DEGREE);
			}
			if (enable_mod_sin_rotor_tracking == 0) {
				rotor_position_command_steps_pf = (float) ((rotor_position_step_polarity)
						* ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE
						* STEPPER_READ_POSITION_STEPS_PER_DEGREE);
			}
			chirp_cycle = chirp_cycle + 1;
		}
	}

	if (enable_rotor_position_step_response_cycle == 1
			&& enable_mod_sin_rotor_tracking == 0
			&& enable_rotor_tracking_comb_signal == 0 && i > angle_cal_complete) {
		rotor_position_command_steps = rotor_position_command_steps_pf * iir_0_s
				+ rotor_position_command_steps_pf_prev * iir_1_s
				- ctx->core_cmd_shaper_state.rotor_position_command_steps_prev * iir_2_s;
		rotor_position_command_steps_pf_prev = rotor_position_command_steps_pf;
	}
}

static void angle_cal_update(AppControlContext *ctx, int i)
{
	if (enable_angle_cal == 1) {
		if (i == 1 && select_suspended_mode == 0) {
			ctx->core_ctl_state.PID_Rotor.Kp = 21.1;
			ctx->core_ctl_state.PID_Rotor.Ki = 0;
			ctx->core_ctl_state.PID_Rotor.Kd = 17.2;
			ctx->core_ctl_state.PID_Pend.Kp = 419;
			ctx->core_ctl_state.PID_Pend.Ki = 0.0;
			ctx->core_ctl_state.PID_Pend.Kd = 56;
			enable_state_feedback = 1;
			integral_compensator_gain = 10;
			feedforward_gain = 1;
			rotor_position_command_steps = 0;
			current_error_rotor_integral = 0;
		}
		if (i == 1 && select_suspended_mode == 1) {
			ctx->core_ctl_state.PID_Rotor.Kp = -23.86;
			ctx->core_ctl_state.PID_Rotor.Ki = 0;
			ctx->core_ctl_state.PID_Rotor.Kd = -19.2;
			ctx->core_ctl_state.PID_Pend.Kp = -293.2;
			ctx->core_ctl_state.PID_Pend.Ki = 0.0;
			ctx->core_ctl_state.PID_Pend.Kd = -41.4;
			enable_state_feedback = 1;
			integral_compensator_gain = -11.45;
			feedforward_gain = 1;
			rotor_position_command_steps = 0;
			current_error_rotor_integral = 0;
		}
		if (i == 1) {
			offset_end_state = 0;
			offset_start_index = 4000;
			angle_index = ANGLE_CAL_OFFSET_STEP_COUNT;
			angle_cal_end = INT32_MAX;
			angle_cal_complete = INT32_MAX;
			encoder_position_offset_zero = 0;
		}
		if (offset_end_state == 0) {
			enable_cycle_delay_warning = 0;
			if (i > 1 && i < 4000) {
				rotor_position_command_steps =
						(i / 4000.0) * ANGLE_CAL_OFFSET_STEP_COUNT / 2;
				offset_start_index = i + 4000;
			}
			if (i == offset_start_index + 10 && angle_index > 0) {
				offset_angle[angle_index] = encoder_position;
				angle_index = angle_index - 1;
				offset_start_index = offset_start_index + 10;
				rotor_position_command_steps = rotor_position_command_steps - 1;
			}
			if (angle_index >= 2 * ANGLE_AVG_SPAN
					&& angle_index < ANGLE_CAL_OFFSET_STEP_COUNT + 1) {
				for (angle_avg_index = angle_index - 2 * ANGLE_AVG_SPAN;
						angle_avg_index < (angle_index + 1); angle_avg_index++) {
					encoder_position_offset_avg[angle_index] = 0;
					for (angle_avg_index = angle_index - ANGLE_AVG_SPAN;
							angle_avg_index < (1 + angle_index + ANGLE_AVG_SPAN);
							angle_avg_index++) {
						encoder_position_offset_avg[angle_index] =
								encoder_position_offset_avg[angle_index]
										+ offset_angle[angle_avg_index];
					}
					encoder_position_offset_avg[angle_index] =
							encoder_position_offset_avg[angle_index]
									/ (float) (2 * ANGLE_AVG_SPAN + 1);
				}
			}
			if (angle_index == 0) {
				rotor_position_command_steps = rotor_position_command_steps
						+ 0.02 * STEPPER_READ_POSITION_STEPS_PER_DEGREE;
			}
			if (rotor_position_command_steps >= 0 && angle_index == 0) {
				offset_end_state = 1;
				angle_cal_end = i;
				enable_cycle_delay_warning = 1;
			}
		}
	}

	if (offset_end_state == 1 && i > angle_cal_end) {
		angle_index =
				(int) ((ANGLE_CAL_OFFSET_STEP_COUNT - 1) / 2)
						+ rotor_position_filter_steps;
		if (angle_index < ANGLE_AVG_SPAN) {
			angle_index = ANGLE_AVG_SPAN;
		}
		if (angle_index > ANGLE_CAL_OFFSET_STEP_COUNT - ANGLE_AVG_SPAN) {
			angle_index = ANGLE_CAL_OFFSET_STEP_COUNT - ANGLE_AVG_SPAN;
		}
		encoder_position_offset = 2.0 * encoder_position_offset_avg[angle_index];
	}

	if (offset_end_state == 1
			&& i > angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING
			&& i < angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING
					+ ANGLE_CAL_ZERO_OFFSET_DWELL) {
		encoder_position_offset_zero = encoder_position_offset_zero + encoder_position;
	}

	if (i == (angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING
			+ ANGLE_CAL_ZERO_OFFSET_DWELL + 1)) {
		encoder_position_offset_zero = encoder_position_offset_zero
				/ ANGLE_CAL_ZERO_OFFSET_DWELL;
		for (angle_index = 0; angle_index < ANGLE_CAL_OFFSET_STEP_COUNT + 1;
				angle_index++) {
			encoder_position_offset_avg[angle_index] =
					encoder_position_offset_avg[angle_index]
							+ encoder_position_offset_zero;
		}
		angle_cal_complete = angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING
				+ ANGLE_CAL_ZERO_OFFSET_DWELL + 1 + ANGLE_CAL_COMPLETION;
	}

	if (offset_end_state == 1 && (enable_angle_cal == 1)
			&& i == angle_cal_complete + 1) {
		ctx->core_ctl_state.PID_Rotor.Kp = ctx->init_params.Kp_rotor;
		ctx->core_ctl_state.PID_Rotor.Ki = ctx->init_params.Ki_rotor;
		ctx->core_ctl_state.PID_Rotor.Kd = ctx->init_params.Kd_rotor;
		ctx->core_ctl_state.PID_Pend.Kp  = ctx->init_params.Kp_pend;
		ctx->core_ctl_state.PID_Pend.Ki  = ctx->init_params.Ki_pend;
		ctx->core_ctl_state.PID_Pend.Kd  = ctx->init_params.Kd_pend;
		current_error_rotor_integral = 0;
		enable_state_feedback           = ctx->init_params.enable_state_feedback;
		integral_compensator_gain       = ctx->init_params.integral_compensator_gain;
		feedforward_gain                = ctx->init_params.feedforward_gain;
		enable_disturbance_rejection_step = ctx->init_params.enable_disturbance_rejection_step;
		enable_sensitivity_fnc_step     = ctx->init_params.enable_sensitivity_fnc_step;
		enable_noise_rejection_step     = ctx->init_params.enable_noise_rejection_step;
		enable_rotor_plant_design       = ctx->init_params.enable_rotor_plant_design;
	}
}

static void report_data(AppControlContext *ctx, int i)
{
	if (enable_pendulum_position_impulse_response_cycle == 1) {
		reference_tracking_command = pendulum_position_command_steps;
	} else {
		reference_tracking_command = rotor_position_command_steps;
	}

	if (i == 1) {
		cycle_period_start = HAL_GetTick();
		cycle_period_sum = 100 * Tsample * 1000 - 1;
	}
	if (i % 100 == 0) {
		cycle_period_sum = HAL_GetTick() - cycle_period_start;
		cycle_period_start = HAL_GetTick();
	}
	tick = HAL_GetTick();
	tick_cycle_previous = tick_cycle_current;
	tick_cycle_current = tick;

	if (enable_high_speed_sampling == 1 && enable_rotor_chirp == 1
			&& enable_rotor_tracking_comb_signal == 0 && ACCEL_CONTROL_DATA == 0) {
		sprintf(msg, "%i\t%i\t%i\t%i\t%i\r\n", cycle_period_sum - 200,
				(int) (roundf(encoder_position)), display_parameter,
				(int) (roundf(rotor_control_target_steps)),
				(int) (reference_tracking_command));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}
	if (enable_high_speed_sampling == 1 && enable_rotor_chirp == 0
			&& enable_rotor_tracking_comb_signal == 1 && ACCEL_CONTROL_DATA == 0) {
		sprintf(msg, "%i\t%i\t%i\t%i\t%i\r\n", current_cpu_cycle_delay_relative_report,
				(int) (roundf(encoder_position)), display_parameter,
				(int) (roundf(rotor_control_target_steps)),
				(int) (roundf(100 * rotor_position_command_steps)));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}
	if (enable_high_speed_sampling == 1 && enable_rotor_chirp == 0
			&& enable_rotor_tracking_comb_signal == 0 && ACCEL_CONTROL_DATA == 0) {
		sprintf(msg, "%i\t%i\t%i\t%i\t%i\r\n", cycle_period_sum - 200,
				(int) (roundf(encoder_position)), display_parameter,
				(int) (roundf(rotor_control_target_steps)),
				(int) (reference_tracking_command));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}
	if (enable_high_speed_sampling == 1 && enable_rotor_chirp == 0
			&& ACCEL_CONTROL_DATA == 1) {
		if (enable_pendulum_position_impulse_response_cycle == 1) {
			reference_tracking_command = pendulum_position_command_steps;
		} else {
			reference_tracking_command = rotor_position_command_steps;
		}
		if (Tsample <= 0.00125) {
			sprintf(msg, "%i\t%lu\r\n", (int) reference_tracking_command,
					current_pwm_period);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		} else {
			sprintf(msg, "%i\t%i\t%i\t%lu\t%lu\t%lu\r\n",
					(int) reference_tracking_command,
					(int) (roundf(rotor_control_target_steps / 10)),
					(int) (rotor_position_command_steps), current_pwm_period,
					desired_pwm_period / 10000, (clock_int_time / 100000));
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}
	}

	if (enable_disturbance_rejection_step == 1) {
		display_parameter = rotor_position_steps / load_disturbance_sensitivity_scale;
	} else if (enable_noise_rejection_step == 1) {
		noise_rej_signal = rotor_control_target_steps;
	} else if (enable_sensitivity_fnc_step == 1) {
		display_parameter = rotor_position_command_steps - rotor_position_steps;
	} else {
		display_parameter = rotor_position_steps;
	}
	if (enable_noise_rejection_step == 1) {
		display_parameter = noise_rej_signal;
	}

	if (enable_high_speed_sampling == 0) {
		if (report_mode != 1000 && report_mode != 2000 && speed_governor == 0) {
			sprintf(msg, "%i\t%i\t%i\t%i\t%i\t%i\t%.1f\t%i\t%i\r\n", (int) 2,
					cycle_period_sum - 200, current_cpu_cycle_delay_relative_report,
					(int) (roundf(encoder_position)), display_parameter,
					(int) (ctx->core_ctl_state.PID_Pend.int_term) / 100,
					reference_tracking_command,
					(int) (roundf(rotor_control_target_steps)),
					(int) (ctx->core_ctl_state.PID_Rotor.control_output) / 100);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}
		if (report_mode != 1000 && report_mode != 2000 && (i % speed_scale) == 0
				&& speed_governor == 1) {
			sprintf(msg, "%i\t%i\t%i\t%i\t%i\t%i\t%.1f\t%i\t%i\r\n", (int) 2,
					cycle_period_sum - 200, current_cpu_cycle_delay_relative_report,
					(int) (roundf(encoder_position)), display_parameter,
					(int) (ctx->core_ctl_state.PID_Pend.int_term) / 100,
					reference_tracking_command,
					(int) (roundf(rotor_control_target_steps)),
					(int) (ctx->core_ctl_state.PID_Rotor.control_output) / 100);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}
		if (report_mode == 1000) {
			sprintf(msg, "%i\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%i\t%i\r\n",
					(int) 0, ctx->core_ctl_state.PID_Pend.Kp,
					ctx->core_ctl_state.PID_Pend.Ki, ctx->core_ctl_state.PID_Pend.Kd,
					ctx->core_ctl_state.PID_Rotor.Kp, ctx->core_ctl_state.PID_Rotor.Ki,
					ctx->core_ctl_state.PID_Rotor.Kd, max_speed / 10, min_speed / 10);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}
		if (report_mode == 2000) {
			sprintf(msg, "%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\r\n", (int) 1,
					(int) torq_current_val, max_accel, max_decel,
					enable_disturbance_rejection_step, enable_noise_rejection_step,
					enable_rotor_position_step_response_cycle, (int) (adjust_increment * 10),
					enable_sensitivity_fnc_step);
			report_mode = 0;
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}
		report_mode = report_mode + 1;
	}
}
