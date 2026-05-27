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

	control_update_slope_correction(ctx, i);
	control_prepare_targets_and_filters(ctx, i);

	if (ENABLE_DUAL_PID == 1) {
		control_update_dual_pid(ctx);
	}
	control_finalize_command_and_actuate(ctx, i);
	report_data(ctx, i);

	return control_wait_next_cycle(ctx);
}

static void control_prepare_targets_and_filters(AppControlContext *ctx, int i)
{
	if ((i > ENCODER_START_OFFSET_DELAY) || (ENCODER_START_OFFSET_DELAY == 0)) {
		ctx->enc_cal.encoder_position = ctx->enc_cal.encoder_position - ENCODER_START_OFFSET;
	}

	float current_error_steps = ctx->enc_cal.encoder_angle_slope_corr_steps
			+ ENCODER_ANGLE_POLARITY
					* (ctx->enc_cal.encoder_position / ((float) (ENCODER_READ_ANGLE_SCALE
							/ STEPPER_READ_POSITION_STEPS_PER_DEGREE)));
	current_error_steps = current_error_steps + ctx->tracking.pendulum_position_command_steps;

	ctx->core_ctl_target.slope_correction_steps = ctx->enc_cal.encoder_angle_slope_corr_steps;
	ctx->core_ctl_target.pendulum_cmd_steps = ctx->tracking.pendulum_position_command_steps;
	ctx->core_ctl_target.pendulum_angle_ref_rad = 0.0f;

	ctx->rotor_pos.rotor_position_filter_steps = (float) (ctx->rotor_pos.rotor_position_steps) * ctx->lpf.iir_0
			+ ctx->rotor_pos.rotor_position_steps_prev * ctx->lpf.iir_1
			- ctx->rotor_pos.rotor_position_filter_steps_prev * ctx->lpf.iir_2;
	ctx->rotor_pos.rotor_position_steps_prev = (float) (ctx->rotor_pos.rotor_position_steps);
	ctx->rotor_pos.rotor_position_filter_steps_prev = ctx->rotor_pos.rotor_position_filter_steps;

	ctx->rotor_pos.rotor_position_filter_steps = ctx->rotor_pos.rotor_position_steps;

	reference_update(ctx, i);
	angle_cal_update(ctx, i);
}

static void reference_update(AppControlContext *ctx, int i)
{
	if (ctx->tracking.enable_rotor_chirp == 1 && ctx->tracking.enable_mod_sin_rotor_tracking == 0
			&& ctx->tracking.enable_rotor_tracking_comb_signal == 0 && i > ctx->enc_cal.angle_cal_complete) {

		if (i < ROTOR_CHIRP_PERIOD - 1) {
			ctx->tracking.chirp_cycle = 0;
		}
		if (ctx->tracking.chirp_cycle > ROTOR_CHIRP_PERIOD - 1) {
			ctx->tracking.chirp_cycle = 0;
			ctx->tracking.chirp_dwell_cycle = ROTOR_CHIRP_SWEEP_DELAY;
		}
		if (ctx->tracking.chirp_dwell_cycle > 0) {
			ctx->tracking.chirp_dwell_cycle--;
			ctx->tracking.chirp_cycle = 0;
		}
		if (ctx->tracking.chirp_dwell_cycle == 0 && i >= ROTOR_CHIRP_PERIOD - 1) {
			ctx->tracking.chirp_cycle = ctx->tracking.chirp_cycle + 1;
			ctx->tracking.chirp_time = (float) ((ctx->tracking.chirp_cycle - 1) / ROTOR_CHIRP_SAMPLE_RATE);
			ctx->tracking.rotor_chirp_frequency = ctx->tracking.rotor_chirp_start_freq
					+ (ctx->tracking.rotor_chirp_end_freq - ctx->tracking.rotor_chirp_start_freq)
							* ((float) (ctx->tracking.chirp_cycle / ctx->tracking.rotor_chirp_period));
			ctx->rotor_pos.rotor_position_command_steps =
					((float) (ROTOR_CHIRP_STEP_AMPLITUDE
							* STEPPER_READ_POSITION_STEPS_PER_DEGREE))
							* sin(2.0 * 3.14159 * ctx->tracking.rotor_chirp_frequency * ctx->tracking.chirp_time);
		}
	}

	if (ctx->tracking.enable_rotor_tracking_comb_signal > 0 && i > 1000 && i > ctx->enc_cal.angle_cal_complete) {
		ctx->tracking.chirp_time = ((float) (i - 1)) / 500.0;
		ctx->tracking.rotor_track_comb_signal_frequency = 0.01;
		ctx->tracking.rotor_track_comb_command = ((float) (ctx->tracking.rotor_track_comb_amplitude))
				* sin(ctx->tracking.rotor_track_comb_signal_frequency * ctx->tracking.chirp_time);
		ctx->tracking.rotor_track_comb_signal_frequency = 0.017783;
		ctx->tracking.rotor_track_comb_command = ctx->tracking.rotor_track_comb_command
				+ ((float) (ctx->tracking.rotor_track_comb_amplitude))
						* sin(ctx->tracking.rotor_track_comb_signal_frequency * ctx->tracking.chirp_time);
		ctx->tracking.rotor_track_comb_signal_frequency = 0.031623;
		ctx->tracking.rotor_track_comb_command = ctx->tracking.rotor_track_comb_command
				+ ((float) (ctx->tracking.rotor_track_comb_amplitude))
						* sin(ctx->tracking.rotor_track_comb_signal_frequency * ctx->tracking.chirp_time);
		ctx->tracking.rotor_track_comb_signal_frequency = 0.056234;
		ctx->tracking.rotor_track_comb_command = ctx->tracking.rotor_track_comb_command
				+ ((float) (ctx->tracking.rotor_track_comb_amplitude))
						* sin(ctx->tracking.rotor_track_comb_signal_frequency * ctx->tracking.chirp_time);
		ctx->tracking.rotor_track_comb_signal_frequency = 0.1;
		ctx->tracking.rotor_track_comb_command = ctx->tracking.rotor_track_comb_command
				+ ((float) (ctx->tracking.rotor_track_comb_amplitude))
						* sin(ctx->tracking.rotor_track_comb_signal_frequency * ctx->tracking.chirp_time);
		ctx->tracking.rotor_track_comb_signal_frequency = 0.17783;
		ctx->tracking.rotor_track_comb_command = ctx->tracking.rotor_track_comb_command
				+ ((float) (ctx->tracking.rotor_track_comb_amplitude))
						* sin(ctx->tracking.rotor_track_comb_signal_frequency * ctx->tracking.chirp_time);
		ctx->tracking.rotor_track_comb_signal_frequency = 0.31623;
		ctx->tracking.rotor_track_comb_command = ctx->tracking.rotor_track_comb_command
				+ ((float) (ctx->tracking.rotor_track_comb_amplitude))
						* sin(ctx->tracking.rotor_track_comb_signal_frequency * ctx->tracking.chirp_time);
		ctx->tracking.rotor_track_comb_signal_frequency = 0.56234;
		ctx->tracking.rotor_track_comb_command = ctx->tracking.rotor_track_comb_command
				+ ((float) (ctx->tracking.rotor_track_comb_amplitude))
						* sin(ctx->tracking.rotor_track_comb_signal_frequency * ctx->tracking.chirp_time);
		ctx->tracking.rotor_track_comb_signal_frequency = 1.0;
		ctx->tracking.rotor_track_comb_command = ctx->tracking.rotor_track_comb_command
				+ ((float) (ctx->tracking.rotor_track_comb_amplitude))
						* sin(ctx->tracking.rotor_track_comb_signal_frequency * ctx->tracking.chirp_time);
		ctx->tracking.rotor_track_comb_signal_frequency = 1.7783;
		ctx->tracking.rotor_track_comb_command = ctx->tracking.rotor_track_comb_command
				+ ((float) (ctx->tracking.rotor_track_comb_amplitude))
						* sin(ctx->tracking.rotor_track_comb_signal_frequency * ctx->tracking.chirp_time);
		ctx->tracking.rotor_track_comb_signal_frequency = 3.1623;
		ctx->tracking.rotor_track_comb_command = ctx->tracking.rotor_track_comb_command
				+ ((float) (ctx->tracking.rotor_track_comb_amplitude))
						* sin(ctx->tracking.rotor_track_comb_signal_frequency * ctx->tracking.chirp_time);
		ctx->tracking.rotor_track_comb_signal_frequency = 5.6234;
		ctx->tracking.rotor_track_comb_command = ctx->tracking.rotor_track_comb_command
				+ ((float) (ctx->tracking.rotor_track_comb_amplitude))
						* sin(ctx->tracking.rotor_track_comb_signal_frequency * ctx->tracking.chirp_time);
		ctx->tracking.rotor_track_comb_signal_frequency = 10;
		ctx->tracking.rotor_track_comb_command = ctx->tracking.rotor_track_comb_command
				+ ((float) (ctx->tracking.rotor_track_comb_amplitude))
						* sin(ctx->tracking.rotor_track_comb_signal_frequency * ctx->tracking.chirp_time);
	}
	if (ctx->tracking.enable_rotor_chirp == 0 && ctx->tracking.enable_mod_sin_rotor_tracking == 0
			&& ctx->tracking.enable_rotor_tracking_comb_signal == 1) {
		ctx->rotor_pos.rotor_position_command_steps = ctx->tracking.rotor_track_comb_command;
	}

	ctx->tracking.rotor_sine_drive = 0;
	if (ctx->tracking.enable_mod_sin_rotor_tracking == 1 && ENABLE_ROTOR_CHIRP == 0
			&& i > ctx->enc_cal.angle_cal_complete) {
		if (ENABLE_ROTOR_CHIRP == 0) {
			ctx->tracking.mod_sin_carrier_frequency = MOD_SIN_CARRIER_FREQ;
		}
		if (i > MOD_SIN_START_CYCLES && ctx->tracking.enable_mod_sin_rotor_tracking == 1) {
			ctx->tracking.rotor_sine_drive = (float) (ctx->tracking.mod_sin_amplitude * (1 + sin(
					-1.5707
							+ ((i - MOD_SIN_START_CYCLES) / MOD_SIN_SAMPLE_RATE)
									* (MOD_SIN_MODULATION_FREQ * 6.2832))));
			ctx->tracking.rotor_sine_drive_mod = sin(
					0
							+ ((i - MOD_SIN_START_CYCLES) / MOD_SIN_SAMPLE_RATE)
									* (ctx->tracking.mod_sin_carrier_frequency * 6.2832));
			ctx->tracking.rotor_sine_drive = ctx->tracking.rotor_sine_drive + MOD_SIN_MODULATION_MIN;
			ctx->tracking.rotor_sine_drive = ctx->tracking.rotor_sine_drive * ctx->tracking.rotor_sine_drive_mod
					* ctx->tracking.rotor_mod_control;
		}
		if (i > MOD_SIN_START_CYCLES && ENABLE_SIN_MOD == 0) {
			ctx->tracking.rotor_sine_drive_mod = sin(
					0
							+ ((i - MOD_SIN_START_CYCLES) / MOD_SIN_SAMPLE_RATE)
									* (ctx->tracking.mod_sin_carrier_frequency * 6.2832));
			ctx->tracking.rotor_sine_drive = ctx->tracking.rotor_control_sin_amplitude * ctx->tracking.rotor_sine_drive_mod
					* ctx->tracking.rotor_mod_control;
		}
		if (fabs(ctx->tracking.rotor_sine_drive_mod * MOD_SIN_AMPLITUDE) < 2
				&& ctx->tracking.disable_mod_sin_rotor_tracking == 1
				&& ctx->tracking.sine_drive_transition == 1) {
			ctx->tracking.rotor_mod_control = 0.0;
			ctx->tracking.sine_drive_transition = 0;
		}
		if (fabs(ctx->tracking.rotor_sine_drive_mod * MOD_SIN_AMPLITUDE) < 2
				&& ctx->tracking.disable_mod_sin_rotor_tracking == 0
				&& ctx->tracking.sine_drive_transition == 1) {
			ctx->tracking.rotor_mod_control = 1.0;
			ctx->tracking.sine_drive_transition = 0;
		}
		if (ctx->tracking.enable_rotor_position_step_response_cycle == 0) {
			ctx->rotor_pos.rotor_position_command_steps = ctx->tracking.rotor_sine_drive;
		}
	}

	if (ENABLE_ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE == 1 && i != 0
			&& i > ctx->enc_cal.angle_cal_complete) {
		if ((i % ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL) == 0) {
			ctx->rotor_pos.rotor_position_command_steps =
					(float) (ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE
							* STEPPER_READ_POSITION_STEPS_PER_DEGREE);
			ctx->rotor_pos.impulse_start_index = 0;
		}
		if (ctx->rotor_pos.impulse_start_index > ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD) {
			ctx->rotor_pos.rotor_position_command_steps = 0;
		}
		ctx->rotor_pos.impulse_start_index++;
	}

	if (ctx->tracking.enable_pendulum_position_impulse_response_cycle == 1 && i != 0
			&& i > ctx->enc_cal.angle_cal_complete) {
		if ((i % PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL) == 0) {
			if (ctx->select_suspended_mode == 1) {
				ctx->tracking.pendulum_position_command_steps =
						(float) PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE;
			}
			if (ctx->select_suspended_mode == 0) {
				ctx->tracking.pendulum_position_command_steps =
						(float) (PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE
								/ PENDULUM_POSITION_IMPULSE_AMPLITUDE_SCALE);
			}
			ctx->tracking.chirp_cycle = 0;
			ctx->rotor_pos.impulse_start_index = 0;
		}
		if (ctx->rotor_pos.impulse_start_index > PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD) {
			ctx->tracking.pendulum_position_command_steps = 0;
		}
		ctx->rotor_pos.impulse_start_index++;
		ctx->tracking.chirp_cycle++;
	}

	if ((i % ROTOR_POSITION_STEP_RESPONSE_CYCLE_INTERVAL) == 0
			&& ctx->tracking.enable_rotor_position_step_response_cycle == 1
			&& i > ctx->enc_cal.angle_cal_complete) {
		ctx->rotor_pos.rotor_position_step_polarity = -ctx->rotor_pos.rotor_position_step_polarity;
		if (ctx->rotor_pos.rotor_position_step_polarity == 1) {
			ctx->tracking.chirp_cycle = 0;
		}
	}
	if (ctx->tracking.enable_rotor_position_step_response_cycle == 1
			&& ctx->tracking.enable_rotor_tracking_comb_signal == 0 && i > ctx->enc_cal.angle_cal_complete) {
		if (STEP_RESPONSE_AMP_LIMIT_ENABLE == 1
				&& fabsf(ctx->tracking.rotor_sine_drive) > STEP_RESPONSE_AMP_LIMIT) {
			ctx->tracking.chirp_cycle = ctx->tracking.chirp_cycle + 1;
		} else {
			if (ctx->tracking.enable_mod_sin_rotor_tracking == 1) {
				ctx->rotor_pos.rotor_position_command_steps = ctx->tracking.rotor_sine_drive + (float) ((ctx->rotor_pos.rotor_position_step_polarity)
						* ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE
						* STEPPER_READ_POSITION_STEPS_PER_DEGREE);
			}
			if (ctx->tracking.enable_mod_sin_rotor_tracking == 0) {
				ctx->rotor_pos.rotor_position_command_steps_pf = (float) ((ctx->rotor_pos.rotor_position_step_polarity)
						* ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE
						* STEPPER_READ_POSITION_STEPS_PER_DEGREE);
			}
			ctx->tracking.chirp_cycle = ctx->tracking.chirp_cycle + 1;
		}
	}

	if (ctx->tracking.enable_rotor_position_step_response_cycle == 1
			&& ctx->tracking.enable_mod_sin_rotor_tracking == 0
			&& ctx->tracking.enable_rotor_tracking_comb_signal == 0 && i > ctx->enc_cal.angle_cal_complete) {
		ctx->rotor_pos.rotor_position_command_steps = ctx->rotor_pos.rotor_position_command_steps_pf * ctx->lpf.iir_0_s
				+ ctx->rotor_pos.rotor_position_command_steps_pf_prev * ctx->lpf.iir_1_s
				- ctx->core_cmd_shaper_state.rotor_position_command_steps_prev * ctx->lpf.iir_2_s;
		ctx->rotor_pos.rotor_position_command_steps_pf_prev = ctx->rotor_pos.rotor_position_command_steps_pf;
	}
}

static void angle_cal_update(AppControlContext *ctx, int i)
{
	if (ctx->enc_cal.enable_angle_cal == 1) {
		if (i == 1 && ctx->select_suspended_mode == 0) {
			ctx->core_ctl_state.PID_Rotor.Kp = 21.1;
			ctx->core_ctl_state.PID_Rotor.Ki = 0;
			ctx->core_ctl_state.PID_Rotor.Kd = 17.2;
			ctx->core_ctl_state.PID_Pend.Kp = 419;
			ctx->core_ctl_state.PID_Pend.Ki = 0.0;
			ctx->core_ctl_state.PID_Pend.Kd = 56;
			ctx->gains.enable_state_feedback = 1;
			ctx->gains.integral_compensator_gain = 10;
			ctx->gains.feedforward_gain = 1;
			ctx->rotor_pos.rotor_position_command_steps = 0;
			ctx->core_dual_pid_runtime.current_error_rotor_integral = 0;
		}
		if (i == 1 && ctx->select_suspended_mode == 1) {
			ctx->core_ctl_state.PID_Rotor.Kp = -23.86;
			ctx->core_ctl_state.PID_Rotor.Ki = 0;
			ctx->core_ctl_state.PID_Rotor.Kd = -19.2;
			ctx->core_ctl_state.PID_Pend.Kp = -293.2;
			ctx->core_ctl_state.PID_Pend.Ki = 0.0;
			ctx->core_ctl_state.PID_Pend.Kd = -41.4;
			ctx->gains.enable_state_feedback = 1;
			ctx->gains.integral_compensator_gain = -11.45;
			ctx->gains.feedforward_gain = 1;
			ctx->rotor_pos.rotor_position_command_steps = 0;
			ctx->core_dual_pid_runtime.current_error_rotor_integral = 0;
		}
		if (i == 1) {
			ctx->enc_cal.offset_end_state = 0;
			ctx->enc_cal.offset_start_index = 4000;
			ctx->enc_cal.angle_index = ANGLE_CAL_OFFSET_STEP_COUNT;
			ctx->enc_cal.angle_cal_end = INT32_MAX;
			ctx->enc_cal.angle_cal_complete = INT32_MAX;
			ctx->enc_cal.encoder_position_offset_zero = 0;
		}
		if (ctx->enc_cal.offset_end_state == 0) {
			ctx->timing.enable_cycle_delay_warning = 0;
			if (i > 1 && i < 4000) {
				ctx->rotor_pos.rotor_position_command_steps =
						(i / 4000.0) * ANGLE_CAL_OFFSET_STEP_COUNT / 2;
				ctx->enc_cal.offset_start_index = i + 4000;
			}
			if (i == ctx->enc_cal.offset_start_index + 10 && ctx->enc_cal.angle_index > 0) {
				ctx->enc_cal.offset_angle[ctx->enc_cal.angle_index] = ctx->enc_cal.encoder_position;
				ctx->enc_cal.angle_index = ctx->enc_cal.angle_index - 1;
				ctx->enc_cal.offset_start_index = ctx->enc_cal.offset_start_index + 10;
				ctx->rotor_pos.rotor_position_command_steps = ctx->rotor_pos.rotor_position_command_steps - 1;
			}
			if (ctx->enc_cal.angle_index >= 2 * ANGLE_AVG_SPAN
					&& ctx->enc_cal.angle_index < ANGLE_CAL_OFFSET_STEP_COUNT + 1) {
				for (ctx->enc_cal.angle_avg_index = ctx->enc_cal.angle_index - 2 * ANGLE_AVG_SPAN;
						ctx->enc_cal.angle_avg_index < (ctx->enc_cal.angle_index + 1); ctx->enc_cal.angle_avg_index++) {
					ctx->enc_cal.encoder_position_offset_avg[ctx->enc_cal.angle_index] = 0;
					for (ctx->enc_cal.angle_avg_index = ctx->enc_cal.angle_index - ANGLE_AVG_SPAN;
							ctx->enc_cal.angle_avg_index < (1 + ctx->enc_cal.angle_index + ANGLE_AVG_SPAN);
							ctx->enc_cal.angle_avg_index++) {
						ctx->enc_cal.encoder_position_offset_avg[ctx->enc_cal.angle_index] =
								ctx->enc_cal.encoder_position_offset_avg[ctx->enc_cal.angle_index]
										+ ctx->enc_cal.offset_angle[ctx->enc_cal.angle_avg_index];
					}
					ctx->enc_cal.encoder_position_offset_avg[ctx->enc_cal.angle_index] =
							ctx->enc_cal.encoder_position_offset_avg[ctx->enc_cal.angle_index]
									/ (float) (2 * ANGLE_AVG_SPAN + 1);
				}
			}
			if (ctx->enc_cal.angle_index == 0) {
				ctx->rotor_pos.rotor_position_command_steps = ctx->rotor_pos.rotor_position_command_steps
						+ 0.02 * STEPPER_READ_POSITION_STEPS_PER_DEGREE;
			}
			if (ctx->rotor_pos.rotor_position_command_steps >= 0 && ctx->enc_cal.angle_index == 0) {
				ctx->enc_cal.offset_end_state = 1;
				ctx->enc_cal.angle_cal_end = i;
				ctx->timing.enable_cycle_delay_warning = 1;
			}
		}
	}

	if (ctx->enc_cal.offset_end_state == 1 && i > ctx->enc_cal.angle_cal_end) {
		ctx->enc_cal.angle_index =
				(int) ((ANGLE_CAL_OFFSET_STEP_COUNT - 1) / 2)
						+ ctx->rotor_pos.rotor_position_filter_steps;
		if (ctx->enc_cal.angle_index < ANGLE_AVG_SPAN) {
			ctx->enc_cal.angle_index = ANGLE_AVG_SPAN;
		}
		if (ctx->enc_cal.angle_index > ANGLE_CAL_OFFSET_STEP_COUNT - ANGLE_AVG_SPAN) {
			ctx->enc_cal.angle_index = ANGLE_CAL_OFFSET_STEP_COUNT - ANGLE_AVG_SPAN;
		}
		ctx->enc_cal.encoder_position_offset = 2.0 * ctx->enc_cal.encoder_position_offset_avg[ctx->enc_cal.angle_index];
	}

	if (ctx->enc_cal.offset_end_state == 1
			&& i > ctx->enc_cal.angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING
			&& i < ctx->enc_cal.angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING
					+ ANGLE_CAL_ZERO_OFFSET_DWELL) {
		ctx->enc_cal.encoder_position_offset_zero = ctx->enc_cal.encoder_position_offset_zero + ctx->enc_cal.encoder_position;
	}

	if (i == (ctx->enc_cal.angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING
			+ ANGLE_CAL_ZERO_OFFSET_DWELL + 1)) {
		ctx->enc_cal.encoder_position_offset_zero = ctx->enc_cal.encoder_position_offset_zero
				/ ANGLE_CAL_ZERO_OFFSET_DWELL;
		for (ctx->enc_cal.angle_index = 0; ctx->enc_cal.angle_index < ANGLE_CAL_OFFSET_STEP_COUNT + 1;
				ctx->enc_cal.angle_index++) {
			ctx->enc_cal.encoder_position_offset_avg[ctx->enc_cal.angle_index] =
					ctx->enc_cal.encoder_position_offset_avg[ctx->enc_cal.angle_index]
							+ ctx->enc_cal.encoder_position_offset_zero;
		}
		ctx->enc_cal.angle_cal_complete = ctx->enc_cal.angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING
				+ ANGLE_CAL_ZERO_OFFSET_DWELL + 1 + ANGLE_CAL_COMPLETION;
	}

	if (ctx->enc_cal.offset_end_state == 1 && (ctx->enc_cal.enable_angle_cal == 1)
			&& i == ctx->enc_cal.angle_cal_complete + 1) {
		ctx->core_ctl_state.PID_Rotor.Kp = ctx->init_params.Kp_rotor;
		ctx->core_ctl_state.PID_Rotor.Ki = ctx->init_params.Ki_rotor;
		ctx->core_ctl_state.PID_Rotor.Kd = ctx->init_params.Kd_rotor;
		ctx->core_ctl_state.PID_Pend.Kp  = ctx->init_params.Kp_pend;
		ctx->core_ctl_state.PID_Pend.Ki  = ctx->init_params.Ki_pend;
		ctx->core_ctl_state.PID_Pend.Kd  = ctx->init_params.Kd_pend;
		ctx->core_dual_pid_runtime.current_error_rotor_integral = 0;
		ctx->gains.enable_state_feedback           = ctx->init_params.enable_state_feedback;
		ctx->gains.integral_compensator_gain       = ctx->init_params.integral_compensator_gain;
		ctx->gains.feedforward_gain                = ctx->init_params.feedforward_gain;
		ctx->gains.enable_disturbance_rejection_step = ctx->init_params.enable_disturbance_rejection_step;
		ctx->gains.enable_sensitivity_fnc_step     = ctx->init_params.enable_sensitivity_fnc_step;
		ctx->gains.enable_noise_rejection_step     = ctx->init_params.enable_noise_rejection_step;
		ctx->plant.enable_rotor_plant_design = ctx->init_params.enable_rotor_plant_design;
	}
}

static void report_data(AppControlContext *ctx, int i)
{
	float noise_rej_signal;
	if (ctx->tracking.enable_pendulum_position_impulse_response_cycle == 1) {
		ctx->rotor_pos.reference_tracking_command = ctx->tracking.pendulum_position_command_steps;
	} else {
		ctx->rotor_pos.reference_tracking_command = ctx->rotor_pos.rotor_position_command_steps;
	}

	if (i == 1) {
		ctx->timing.cycle_period_start = HAL_GetTick();
		ctx->timing.cycle_period_sum = 100 * ctx->timing.Tsample * 1000 - 1;
	}
	if (i % 100 == 0) {
		ctx->timing.cycle_period_sum = HAL_GetTick() - ctx->timing.cycle_period_start;
		ctx->timing.cycle_period_start = HAL_GetTick();
	}
	ctx->timing.tick = HAL_GetTick();
	ctx->timing.tick_cycle_previous = ctx->timing.tick_cycle_current;
	ctx->timing.tick_cycle_current = ctx->timing.tick;

	if (ctx->enable_high_speed_sampling == 1 && ctx->tracking.enable_rotor_chirp == 1
			&& ctx->tracking.enable_rotor_tracking_comb_signal == 0 && ACCEL_CONTROL_DATA == 0) {
		sprintf(msg, "%i\t%i\t%i\t%i\t%i\r\n", ctx->timing.cycle_period_sum - 200,
				(int) (roundf(ctx->enc_cal.encoder_position)), display_parameter,
				(int) (roundf(ctx->rotor_pos.rotor_control_target_steps)),
				(int) (ctx->rotor_pos.reference_tracking_command));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}
	if (ctx->enable_high_speed_sampling == 1 && ctx->tracking.enable_rotor_chirp == 0
			&& ctx->tracking.enable_rotor_tracking_comb_signal == 1 && ACCEL_CONTROL_DATA == 0) {
		sprintf(msg, "%i\t%i\t%i\t%i\t%i\r\n", ctx->timing.current_cpu_cycle_delay_relative_report,
				(int) (roundf(ctx->enc_cal.encoder_position)), display_parameter,
				(int) (roundf(ctx->rotor_pos.rotor_control_target_steps)),
				(int) (roundf(100 * ctx->rotor_pos.rotor_position_command_steps)));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}
	if (ctx->enable_high_speed_sampling == 1 && ctx->tracking.enable_rotor_chirp == 0
			&& ctx->tracking.enable_rotor_tracking_comb_signal == 0 && ACCEL_CONTROL_DATA == 0) {
		sprintf(msg, "%i\t%i\t%i\t%i\t%i\r\n", ctx->timing.cycle_period_sum - 200,
				(int) (roundf(ctx->enc_cal.encoder_position)), display_parameter,
				(int) (roundf(ctx->rotor_pos.rotor_control_target_steps)),
				(int) (ctx->rotor_pos.reference_tracking_command));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}
	if (ctx->enable_high_speed_sampling == 1 && ctx->tracking.enable_rotor_chirp == 0
			&& ACCEL_CONTROL_DATA == 1) {
		if (ctx->tracking.enable_pendulum_position_impulse_response_cycle == 1) {
			ctx->rotor_pos.reference_tracking_command = ctx->tracking.pendulum_position_command_steps;
		} else {
			ctx->rotor_pos.reference_tracking_command = ctx->rotor_pos.rotor_position_command_steps;
		}
		if (ctx->timing.Tsample <= 0.00125) {
			sprintf(msg, "%i\t%lu\r\n", (int) ctx->rotor_pos.reference_tracking_command,
					current_pwm_period);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		} else {
			sprintf(msg, "%i\t%i\t%i\t%lu\t%lu\t%lu\r\n",
					(int) ctx->rotor_pos.reference_tracking_command,
					(int) (roundf(ctx->rotor_pos.rotor_control_target_steps / 10)),
					(int) (ctx->rotor_pos.rotor_position_command_steps), current_pwm_period,
					desired_pwm_period / 10000, (clock_int_time / 100000));
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}
	}

	if (ctx->gains.enable_disturbance_rejection_step == 1) {
		display_parameter = ctx->rotor_pos.rotor_position_steps / ctx->gains.load_disturbance_sensitivity_scale;
	} else if (ctx->gains.enable_noise_rejection_step == 1) {
		noise_rej_signal = ctx->rotor_pos.rotor_control_target_steps;
	} else if (ctx->gains.enable_sensitivity_fnc_step == 1) {
		display_parameter = ctx->rotor_pos.rotor_position_command_steps - ctx->rotor_pos.rotor_position_steps;
	} else {
		display_parameter = ctx->rotor_pos.rotor_position_steps;
	}
	if (ctx->gains.enable_noise_rejection_step == 1) {
		display_parameter = noise_rej_signal;
	}

	if (ctx->enable_high_speed_sampling == 0) {
		if (ctx->report_mode != 1000 && ctx->report_mode != 2000 && ctx->speed_governor == 0) {
			sprintf(msg, "%i\t%i\t%i\t%i\t%i\t%i\t%.1f\t%i\t%i\r\n", (int) 2,
					ctx->timing.cycle_period_sum - 200, ctx->timing.current_cpu_cycle_delay_relative_report,
					(int) (roundf(ctx->enc_cal.encoder_position)), display_parameter,
					(int) (ctx->core_ctl_state.PID_Pend.int_term) / 100,
					ctx->rotor_pos.reference_tracking_command,
					(int) (roundf(ctx->rotor_pos.rotor_control_target_steps)),
					(int) (ctx->core_ctl_state.PID_Rotor.control_output) / 100);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}
		if (ctx->report_mode != 1000 && ctx->report_mode != 2000 && (i % ctx->speed_scale) == 0
				&& ctx->speed_governor == 1) {
			sprintf(msg, "%i\t%i\t%i\t%i\t%i\t%i\t%.1f\t%i\t%i\r\n", (int) 2,
					ctx->timing.cycle_period_sum - 200, ctx->timing.current_cpu_cycle_delay_relative_report,
					(int) (roundf(ctx->enc_cal.encoder_position)), display_parameter,
					(int) (ctx->core_ctl_state.PID_Pend.int_term) / 100,
					ctx->rotor_pos.reference_tracking_command,
					(int) (roundf(ctx->rotor_pos.rotor_control_target_steps)),
					(int) (ctx->core_ctl_state.PID_Rotor.control_output) / 100);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}
		if (ctx->report_mode == 1000) {
			sprintf(msg, "%i\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%i\t%i\r\n",
					(int) 0, ctx->core_ctl_state.PID_Pend.Kp,
					ctx->core_ctl_state.PID_Pend.Ki, ctx->core_ctl_state.PID_Pend.Kd,
					ctx->core_ctl_state.PID_Rotor.Kp, ctx->core_ctl_state.PID_Rotor.Ki,
					ctx->core_ctl_state.PID_Rotor.Kd, ctx->max_speed / 10, ctx->min_speed / 10);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}
		if (ctx->report_mode == 2000) {
			sprintf(msg, "%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\r\n", (int) 1,
					(int) ctx->torq_current_val, ctx->max_accel, ctx->max_decel,
					ctx->gains.enable_disturbance_rejection_step, ctx->gains.enable_noise_rejection_step,
					ctx->tracking.enable_rotor_position_step_response_cycle, (int) (ctx->adjust_increment * 10),
					ctx->gains.enable_sensitivity_fnc_step);
			ctx->report_mode = 0;
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}
		ctx->report_mode = ctx->report_mode + 1;
	}
}
