#include "main.h"
#include "edukit_system.h"
#include "hardware.h"
#include "app_control.h"
#include "app_runtime.h"
#include "app_session.h"
#include "ui.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void app_session_home_rotor(AppControlContext *ctx);
static void app_session_wait_pendulum_rest(AppControlContext *ctx);
static void app_session_wait_pendulum_upright(AppControlContext *ctx);
static void app_session_init_state(AppControlContext *ctx);
static void app_run_swing_up(AppControlContext *ctx);
static void app_run_balance_loop(AppControlContext *ctx);

void app_run_mode_loop(AppControlContext *ctx)
{
	int k;

	while (1) {
		ui_set_mode_interactive(0);
		user_prompt();

		sprintf(uart_tx_buf, "\n\rEnter Mode Selection Now: \n\r");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

		for (k = 0; k < SERIAL_MSG_MAXLEN; k++) {
			Msg.Data[k] = 0;
		}
		user_configuration(ctx);

		app_prepare_control_session(ctx);
		app_run_control_session(ctx);
	}
}

void app_prepare_control_session(AppControlContext *ctx)
{
	BSP_MotorControl_SoftStop(0);
	BSP_MotorControl_WaitWhileActive(0);
	L6474_SetAnalogValue(0, L6474_TVAL, ctx->torq_current_val);
	BSP_MotorControl_SetMaxSpeed(0, ctx->max_speed);
	BSP_MotorControl_SetMinSpeed(0, ctx->min_speed);
	BSP_MotorControl_SetAcceleration(0, MAX_ACCEL);
	BSP_MotorControl_SetDeceleration(0, MAX_DECEL);

	if (ACCEL_CONTROL == 0) {
		sprintf(uart_tx_buf, "\n\rMotor Profile Speeds Set at Min %u Max %u Steps per Second",
				ctx->min_speed, ctx->max_speed);
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	}
	if (ctx->select_suspended_mode == 0) {
		sprintf(uart_tx_buf, "\n\rInverted Pendulum Mode Selected");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	}
	if (ctx->select_suspended_mode == 1) {
		sprintf(uart_tx_buf, "\n\rSuspended Pendulum Mode Selected");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	}

	sprintf(uart_tx_buf, "\n\rMotor Torque Current Set at %0.1f mA", ctx->torq_current_val);
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

	if (ctx->enable_motor_actuator_characterization_mode == 1) {
		motor_actuator_characterization_mode(ctx);
	}
	if (ctx->enable_rotor_actuator_control == 1) {
		interactive_rotor_actuator_control();
	}
	if (ctx->enable_rotor_actuator_test == 1) {
		rotor_encoder_test(ctx);
	}

	app_assign_pid_gains_from_user(ctx);
	ctx->gains.integral_compensator_gain *= CONTROLLER_GAIN_SCALE;
}

void app_run_control_session(AppControlContext *ctx)
{
	ctx->enable_control_action = ENABLE_CONTROL_ACTION;

	app_session_home_rotor(ctx);
	app_session_wait_pendulum_rest(ctx);
	app_session_wait_pendulum_upright(ctx);
	app_session_init_state(ctx);
	if (ctx->enable_swing_up == 1 && ctx->select_suspended_mode == 0
			&& !ctx->enable_remote_swing_up) {
		app_run_swing_up(ctx);
	}
	app_run_balance_loop(ctx);
}

static void app_session_home_rotor(AppControlContext *ctx)
{
	int ret;

	if (ctx->reset_state == 1) {
		hardware_rotor_home();
	}
	ret = hardware_rotor_position_read(&ctx->rotor_pos.rotor_position_steps);
	sprintf(uart_tx_buf, "\r\nPrepare for Control Start - Initial Rotor Position: %i\r\n",
			ctx->rotor_pos.rotor_position_steps);
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

	BSP_MotorControl_GoTo(0, 3);
	BSP_MotorControl_WaitWhileActive(0);
	HAL_Delay(150);
	BSP_MotorControl_GoTo(0, -3);
	BSP_MotorControl_WaitWhileActive(0);
	HAL_Delay(150);
	BSP_MotorControl_GoTo(0, 3);
	BSP_MotorControl_WaitWhileActive(0);
	HAL_Delay(150);
	BSP_MotorControl_GoTo(0, 0);
	BSP_MotorControl_WaitWhileActive(0);
	(void)ret;
}

static void app_session_wait_pendulum_rest(AppControlContext *ctx)
{
	int ret;
	int encoder_position_curr, encoder_position_prev;

	sprintf(uart_tx_buf, "Test for Pendulum at Rest - Waiting for Pendulum to Stabilize\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

	ctx->enc_cal.encoder_position_init = 0;
	ret = hardware_encoder_position_read(&ctx->enc_cal.encoder_position_steps, ctx->enc_cal.encoder_position_init,
			&htim3);
	encoder_position_prev = ctx->enc_cal.encoder_position_steps;
	HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
	ret = hardware_encoder_position_read(&ctx->enc_cal.encoder_position_steps, ctx->enc_cal.encoder_position_init,
			&htim3);
	encoder_position_curr = ctx->enc_cal.encoder_position_steps;
	while (encoder_position_curr != encoder_position_prev) {
		ret = hardware_encoder_position_read(&ctx->enc_cal.encoder_position_steps,
				ctx->enc_cal.encoder_position_init, &htim3);
		encoder_position_prev = ctx->enc_cal.encoder_position_steps;
		HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
		ret = hardware_encoder_position_read(&ctx->enc_cal.encoder_position_steps,
				ctx->enc_cal.encoder_position_init, &htim3);
		encoder_position_curr = ctx->enc_cal.encoder_position_steps;

		if (encoder_position_prev == encoder_position_curr) {
			HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
			ret = hardware_encoder_position_read(&ctx->enc_cal.encoder_position_steps,
					ctx->enc_cal.encoder_position_init, &htim3);
			encoder_position_prev = ctx->enc_cal.encoder_position_steps;
			HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
			ret = hardware_encoder_position_read(&ctx->enc_cal.encoder_position_steps,
					ctx->enc_cal.encoder_position_init, &htim3);
			encoder_position_curr = ctx->enc_cal.encoder_position_steps;
			if (encoder_position_prev == encoder_position_curr) {
				break;
			}
		}
		sprintf(uart_tx_buf,
				"Pendulum Motion Detected with angle %0.2f - Waiting for Pendulum to Stabilize\r\n",
				(float) ((encoder_position_curr - encoder_position_prev)
						/ ENCODER_READ_ANGLE_SCALE));
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	}

	sprintf(uart_tx_buf, "Pendulum Now at Rest and Measuring Pendulum Down Angle\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

	HAL_Delay(100);
	ret = hardware_encoder_position_read(&ctx->enc_cal.encoder_position_steps, ctx->enc_cal.encoder_position_init,
			&htim3);
	ctx->enc_cal.encoder_position_init = ctx->enc_cal.encoder_position_steps;

	if (ret == -1) {
		sprintf(uart_tx_buf, "Encoder Position Under Range Error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	}
	if (ret == 1) {
		sprintf(uart_tx_buf, "Encoder Position Over Range Error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	}

	ret = hardware_encoder_position_read(&ctx->enc_cal.encoder_position_steps, ctx->enc_cal.encoder_position_init,
			&htim3);
	ctx->enc_cal.encoder_position_down = ctx->enc_cal.encoder_position_steps;
	sprintf(uart_tx_buf, "Pendulum Initial Angle %i\r\n", ctx->enc_cal.encoder_position_steps);
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	(void)ret;
}

static void app_session_wait_pendulum_upright(AppControlContext *ctx)
{
	int ret = 0;

	if (ctx->enable_swing_up == 0) {
		BSP_MotorControl_GoTo(0, 30);
		BSP_MotorControl_WaitWhileActive(0);
		HAL_Delay(150);
		BSP_MotorControl_GoTo(0, -30);
		BSP_MotorControl_WaitWhileActive(0);
		HAL_Delay(150);
		BSP_MotorControl_GoTo(0, 30);
		BSP_MotorControl_WaitWhileActive(0);
		HAL_Delay(150);
		BSP_MotorControl_GoTo(0, 0);
		BSP_MotorControl_WaitWhileActive(0);

		if (ctx->select_suspended_mode == 0) {
			sprintf(uart_tx_buf,
					"Adjust Pendulum Upright By Turning CCW Control Will Start When Vertical\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
		}
	}

	if (ctx->enable_swing_up == 0) {
		uint32_t tick_wait_start = HAL_GetTick();
		if (ctx->select_suspended_mode == 0) {
			while (1) {
				ret = hardware_encoder_position_read(&ctx->enc_cal.encoder_position_steps,
						ctx->enc_cal.encoder_position_init, &htim3);
				if (fabs(
						ctx->enc_cal.encoder_position_steps - ctx->enc_cal.encoder_position_down
								- (int) (180 * ctx->angle_scale)) < START_ANGLE * ctx->angle_scale) {
					HAL_Delay(START_ANGLE_DELAY);
					break;
				}
				if (fabs(
						ctx->enc_cal.encoder_position_steps - ctx->enc_cal.encoder_position_down
								+ (int) (180 * ctx->angle_scale)) < START_ANGLE * ctx->angle_scale) {
					ctx->enc_cal.encoder_position_down = ctx->enc_cal.encoder_position_down
							- 2 * (int) (180 * ctx->angle_scale);
					HAL_Delay(START_ANGLE_DELAY);
					break;
				}
				uint32_t tick_wait = HAL_GetTick();

				if ((tick_wait - tick_wait_start)
						> PENDULUM_ORIENTATION_START_DELAY) {
					sprintf(uart_tx_buf,
							"Pendulum Upright Action Not Detected - Restarting ...\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf),
							HAL_MAX_DELAY);
					ctx->enable_control_action = 0;
					break;
				}
			}
		}
	}

	if (ctx->select_suspended_mode == 1) {
		sprintf(uart_tx_buf, "Suspended Mode Control Will Start in %i Seconds\r\n",
				(int) (CONTROL_START_DELAY / 1000));
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
	}
	(void)ret;
}

static void app_session_init_state(AppControlContext *ctx)
{
	int k, m;

	ctx->core_dual_pid_runtime.current_error_rotor_steps = 0;
	ctx->core_dual_pid_runtime.current_error_rotor_integral = 0;

	ctx->timing.cycle_count = CYCLE_LIMIT;
	ctx->rotor_pos.rotor_position_steps = 0;
	ctx->rotor_pos.rotor_position_steps_prev = 0;
	ctx->rotor_pos.rotor_position_filter_steps = 0;
	ctx->rotor_pos.rotor_position_filter_steps_prev = 0;
	ctx->rotor_pos.rotor_position_command_steps = 0;
	ctx->rotor_pos.rotor_position_diff = 0;
	ctx->rotor_pos.rotor_position_diff_prev = 0;
	ctx->rotor_pos.rotor_position_diff_filter = 0;
	ctx->rotor_pos.rotor_position_diff_filter_prev = 0;
	ctx->enc_cal.encoder_angle_slope_corr_steps = 0;
	ctx->enable_adaptive_mode = 0;
	ctx->timing.tick_cycle_start = HAL_GetTick();
	ctx->timing.tick_cycle_previous = ctx->timing.tick_cycle_start;
	ctx->timing.tick_cycle_current = ctx->timing.tick_cycle_start;
	ctx->timing.enable_cycle_delay_warning = ENABLE_CYCLE_DELAY_WARNING;
	ctx->mode_transition_state = 0;
	ctx->adaptive_state = 4;
	app_reset_command_shaper_state(ctx);
	ctx->timing.current_cpu_cycle = 0;
	ctx->enc_cal.encoder_position_offset = 0;
	ctx->enc_cal.encoder_position_offset_zero = 0;

	for (m = 0; m < ANGLE_CAL_OFFSET_STEP_COUNT + 1; m++) {
		ctx->enc_cal.offset_angle[m] = 0;
	}

	for (k = 0; k < SERIAL_MSG_MAXLEN; k++) {
		Msg.Data[k] = 0;
	}
	__HAL_DMA_RESET_HANDLE_STATE(&hdma_usart2_rx);

	ctx->init_params.Kp_rotor = ctx->core_ctl_state.PID_Rotor.Kp;
	ctx->init_params.Ki_rotor = ctx->core_ctl_state.PID_Rotor.Ki;
	ctx->init_params.Kd_rotor = ctx->core_ctl_state.PID_Rotor.Kd;
	ctx->init_params.Kp_pend  = ctx->core_ctl_state.PID_Pend.Kp;
	ctx->init_params.Ki_pend  = ctx->core_ctl_state.PID_Pend.Ki;
	ctx->init_params.Kd_pend  = ctx->core_ctl_state.PID_Pend.Kd;
	ctx->init_params.enable_state_feedback           = ctx->gains.enable_state_feedback;
	ctx->init_params.integral_compensator_gain       = ctx->gains.integral_compensator_gain;
	ctx->init_params.feedforward_gain                = ctx->gains.feedforward_gain;
	ctx->init_params.enable_disturbance_rejection_step = ctx->gains.enable_disturbance_rejection_step;
	ctx->init_params.enable_sensitivity_fnc_step     = ctx->gains.enable_sensitivity_fnc_step;
	ctx->init_params.enable_noise_rejection_step     = ctx->gains.enable_noise_rejection_step;

	if (ctx->select_suspended_mode == 1) {
		ctx->gains.load_disturbance_sensitivity_scale = 1.0;
	}
	if (ctx->select_suspended_mode == 0) {
		ctx->gains.load_disturbance_sensitivity_scale = LOAD_DISTURBANCE_SENSITIVITY_SCALE;
	}
}

static void app_run_swing_up(AppControlContext *ctx)
{
	int ret;
	motorDir_t swing_up_direction;
	int swing_up_state;
	int stage_count, stage_amp;
	int swing_up_cycle = 0;
	int rotor_position_steps = 0;
	float theta_p_deg_prev = 0.0f;
	float theta_r_deg_prev = 0.0f;

	ctx->core_ctl_state.PID_Rotor.Kp = 20;
	ctx->core_ctl_state.PID_Rotor.Ki = 10;
	ctx->core_ctl_state.PID_Rotor.Kd = 10;
	ctx->core_ctl_state.PID_Pend.Kp = 300;
	ctx->core_ctl_state.PID_Pend.Ki = 0.0;
	ctx->core_ctl_state.PID_Pend.Kd = 30.0;
	ctx->gains.enable_state_feedback = 0;
	ctx->gains.integral_compensator_gain = 0;
	ctx->gains.feedforward_gain = 1;
	ctx->rotor_pos.rotor_position_command_steps = 0;
	ctx->gains.enable_state_feedback = 0;
	ctx->gains.enable_disturbance_rejection_step = 0;
	ctx->gains.enable_sensitivity_fnc_step = 0;
	ctx->gains.enable_noise_rejection_step = 0;

	ctx->torq_current_val = MAX_TORQUE_SWING_UP;
	L6474_SetAnalogValue(0, L6474_TVAL, ctx->torq_current_val);

	sprintf(uart_tx_buf, "Pendulum Swing Up Starting\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);

	hardware_swing_up_reset();
	swing_up_state = 0;
	stage_count = 0;
	stage_amp = STAGE_0_AMP;

	swing_up_direction = FORWARD;
	BSP_MotorControl_Move(0, swing_up_direction, 150);
	BSP_MotorControl_WaitWhileActive(0);

	/* Prime the previous-sample values so the first telemetry line's
	 * omega_p/omega_r aren't a bogus spike against the 0.0f initializers
	 * above. */
	ret = hardware_encoder_position_read(&ctx->enc_cal.encoder_position_steps,
			ctx->enc_cal.encoder_position_init, &htim3);
	theta_p_deg_prev = (float) (ctx->enc_cal.encoder_position_steps
			- ctx->enc_cal.encoder_position_down) / ctx->angle_scale;
	hardware_rotor_position_read(&rotor_position_steps);
	theta_r_deg_prev = (float) rotor_position_steps / STEPPER_READ_POSITION_STEPS_PER_DEGREE;

	while (1) {
		SwingUpSensorState sus;
		HAL_Delay(2);
		ret = hardware_encoder_position_read(&ctx->enc_cal.encoder_position_steps,
				ctx->enc_cal.encoder_position_init, &htim3);
		hardware_swing_up_get(&sus);

		/* Swing-up doesn't run through the observer (control_update_state_
		 * and_safety()/report_telemetry(), Src/app_control.c, Src/app_runtime.c),
		 * so build the same 6-field CSV line here instead, at roughly the same
		 * ~100Hz rate. theta_p_deg is relative to encoder_position_down (0 =
		 * hang-down), NOT yet corrected to 0 = upright the way the balance
		 * loop's telemetry is — that correction (below) only happens once the
		 * upright crossing is actually detected, so expect a ~180/360 deg
		 * jump in the log right at the swing-up -> balance handoff if the
		 * pendulum approaches from the "negative" side. */
		swing_up_cycle++;
		if (swing_up_cycle % 5 == 0) {
			float theta_p_deg = (float) (ctx->enc_cal.encoder_position_steps
					- ctx->enc_cal.encoder_position_down) / ctx->angle_scale;
			float theta_r_deg;
			float omega_p_deg_s, omega_r_deg_s;
			const float dt_s = 0.002f * 5.0f;

			hardware_rotor_position_read(&rotor_position_steps);
			theta_r_deg = (float) rotor_position_steps / STEPPER_READ_POSITION_STEPS_PER_DEGREE;

			omega_p_deg_s = (theta_p_deg - theta_p_deg_prev) / dt_s;
			omega_r_deg_s = (theta_r_deg - theta_r_deg_prev) / dt_s;
			theta_p_deg_prev = theta_p_deg;
			theta_r_deg_prev = theta_r_deg;

			sprintf(uart_tx_buf, "%i,%.3f,%.3f,%.3f,%.3f,%.1f\r\n",
					swing_up_cycle, theta_p_deg, theta_r_deg, omega_p_deg_s, omega_r_deg_s,
					(float) (swing_up_direction == FORWARD ? stage_amp : -stage_amp));
			HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buf, strlen(uart_tx_buf), HAL_MAX_DELAY);
		}

		if (fabs(
				ctx->enc_cal.encoder_position_steps - ctx->enc_cal.encoder_position_down
						- (int) (180 * ctx->angle_scale)) < START_ANGLE * ctx->angle_scale) {
			break;
		}
		if (fabs(
				ctx->enc_cal.encoder_position_steps - ctx->enc_cal.encoder_position_down
						+ (int) (180 * ctx->angle_scale)) < START_ANGLE * ctx->angle_scale) {
			ctx->enc_cal.encoder_position_down = ctx->enc_cal.encoder_position_down
					- 2 * (int) (180 * ctx->angle_scale);
			break;
		}

		if (sus.zero_crossed) {
			hardware_swing_up_clear_zero_crossed();
			if (swing_up_state == 0) {
				BSP_MotorControl_Move(0, swing_up_direction, stage_amp);
				BSP_MotorControl_WaitWhileActive(0);
				stage_count++;

				if (sus.prev_global_max_encoder_position != sus.global_max_encoder_position
						&& stage_count > 4) {
					if (abs(sus.global_max_encoder_position) < 600) {
						stage_amp = STAGE_0_AMP;
					}
					if (abs(sus.global_max_encoder_position) >= 600
							&& abs(sus.global_max_encoder_position) < 1000) {
						stage_amp = STAGE_1_AMP;
					}
					if (abs(sus.global_max_encoder_position) >= 1000) {
						stage_amp = STAGE_2_AMP;
					}
				}
				hardware_swing_up_set_prev_global_max(sus.global_max_encoder_position);
				hardware_swing_up_reset_global_max();
				ret = hardware_encoder_position_read(&ctx->enc_cal.encoder_position_steps,
						ctx->enc_cal.encoder_position_init, &htim3);
				hardware_swing_up_get(&sus);
			}
		}

		if (sus.peaked && !sus.handled_peak) {
			hardware_swing_up_handle_peak();
			swing_up_direction =
					swing_up_direction == FORWARD ? BACKWARD : FORWARD;
		}
	}
	(void)ret;
	(void)swing_up_state;
}

static void app_run_balance_loop(AppControlContext *ctx)
{
	int i = 0, ret;

	ctx->enable_control_action = 1;

	if (ACCEL_CONTROL == 1) {
		BSP_MotorControl_HardStop(0);
		L6474_CmdEnable(0);
		target_velocity_prescaled = 0;
		L6474_Board_SetDirectionGpio(0, BACKWARD);
	}

	ctx->torq_current_val = MAX_TORQUE_CONFIG;
	L6474_SetAnalogValue(0, L6474_TVAL, ctx->torq_current_val);

	ctx->timing.target_cpu_cycle = DWT->CYCCNT;
	ctx->timing.prev_cpu_cycle = DWT->CYCCNT;

	ret = hardware_encoder_position_read(&ctx->enc_cal.encoder_position_steps, ctx->enc_cal.encoder_position_init,
			&htim3);
	if (ctx->select_suspended_mode == 0) {
		ctx->enc_cal.encoder_position = ctx->enc_cal.encoder_position_steps - ctx->enc_cal.encoder_position_down
				- (int) (180 * ctx->angle_scale);
		ctx->enc_cal.encoder_position = ctx->enc_cal.encoder_position - ctx->enc_cal.encoder_position_offset;
	}

	app_init_control_pipeline(ctx, ctx->enc_cal.encoder_position_init, ctx->timing.t_sample_s);

	/* Mode D only: controller_init() (just above, including its derivative
	 * low-pass filter coefficients) and t_sample_rotor_s (set once at boot,
	 * Src/app_bootstrap.c) both assume control_update_dual_pid() runs every
	 * cycle. It only runs every CONTROL_DECIMATION_FACTOR-th cycle here
	 * (Src/app_runtime.c), so re-init the controller with the true elapsed
	 * period between calls instead of the base cycle time — otherwise the
	 * derivative term is CONTROL_DECIMATION_FACTOR times too large (dividing
	 * a 10ms angle change by an assumed 2ms) and the response runs away. */
	if (ctx->enable_decimated_control) {
		PidGains active_gains;
		active_gains.Kp_pend  = ctx->core_ctl_state.PID_Pend.Kp;
		active_gains.Ki_pend  = ctx->core_ctl_state.PID_Pend.Ki;
		active_gains.Kd_pend  = ctx->core_ctl_state.PID_Pend.Kd;
		active_gains.Kp_rotor = ctx->core_ctl_state.PID_Rotor.Kp;
		active_gains.Ki_rotor = ctx->core_ctl_state.PID_Rotor.Ki;
		active_gains.Kd_rotor = ctx->core_ctl_state.PID_Rotor.Kd;
		ctx->core_controller_ops->init(&ctx->core_ctl_state, &active_gains,
				ctx->timing.t_sample_s * CONTROL_DECIMATION_FACTOR);
		ctx->timing.t_sample_rotor_s = ctx->timing.t_sample_s * CONTROL_DECIMATION_FACTOR;
	} else {
		ctx->timing.t_sample_rotor_s = ctx->timing.t_sample_s;
	}

	while (ctx->enable_control_action == 1) {
		ret = control_handle_runtime_configuration(ctx, i);
		if (ret < 0) {
			break;
		}
		if (ret > 0) {
			continue;
		}

		if (i < 1) {
			ctx->core_ctl_state.PID_Pend.int_term = 0;
			ctx->core_ctl_state.PID_Rotor.int_term = 0;
		}

		ret = control_execute_cycle(ctx, i);
		if (ret != 0) {
			break;
		}

		i++;
	}

	control_shutdown_sequence(ctx);
	(void)ret;
}
