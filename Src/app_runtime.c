#include "main.h"
#include "edukit_system.h"
#include "app_control.h"
#include "app_runtime.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static void control_prepare_targets_and_filters(AppControlContext *ctx, int i);
static void angle_cal_update(AppControlContext *ctx, int i);
static void report_telemetry(AppControlContext *ctx, int i);

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
	report_telemetry(ctx, i);

	return control_wait_next_cycle(ctx);
}

static void control_prepare_targets_and_filters(AppControlContext *ctx, int i)
{
	if ((i > ENCODER_START_OFFSET_DELAY) || (ENCODER_START_OFFSET_DELAY == 0)) {
		ctx->enc_cal.encoder_position = ctx->enc_cal.encoder_position - ENCODER_START_OFFSET;
	}

	ctx->core_ctl_target.slope_correction_steps = ctx->enc_cal.encoder_angle_slope_corr_steps;
	ctx->core_ctl_target.pendulum_cmd_steps = 0.0f;
	ctx->core_ctl_target.pendulum_angle_ref_rad = 0.0f;

	ctx->rotor_pos.rotor_position_filter_steps = (float) (ctx->rotor_pos.rotor_position_steps) * ctx->lpf.iir_0
			+ ctx->rotor_pos.rotor_position_steps_prev * ctx->lpf.iir_1
			- ctx->rotor_pos.rotor_position_filter_steps_prev * ctx->lpf.iir_2;
	ctx->rotor_pos.rotor_position_steps_prev = (float) (ctx->rotor_pos.rotor_position_steps);
	ctx->rotor_pos.rotor_position_filter_steps_prev = ctx->rotor_pos.rotor_position_filter_steps;

	ctx->rotor_pos.rotor_position_filter_steps = ctx->rotor_pos.rotor_position_steps;

	angle_cal_update(ctx, i);
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
	}
}

static void report_telemetry(AppControlContext *ctx, int i)
{
	if (i % 5 != 0) return;   /* 500 Hz loop / 5 = 100 Hz output */
	sprintf(uart_tx_buf, "%i,%.3f,%.3f,%.3f,%.3f,%.1f\r\n",
			i,
			ctx->core_sys_state.pendulum_angle_rad    * (180.0f / 3.14159265f),
			ctx->core_sys_state.rotor_angle_rad       * (180.0f / 3.14159265f),
			ctx->core_sys_state.pendulum_velocity_rad_s * (180.0f / 3.14159265f),
			ctx->core_sys_state.rotor_velocity_rad_s    * (180.0f / 3.14159265f),
			ctx->rotor_pos.rotor_control_target_steps);
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
}
