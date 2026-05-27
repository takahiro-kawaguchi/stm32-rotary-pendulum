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

void app_run_mode_loop(AppControlContext *ctx)
{
	int k;

	while (1) {
		ui_set_mode_interactive(0);
		user_prompt();

		if (ui_get_mode_interactive() == 0) {
			sprintf(msg,
					"\n\rEnter Mode Selection Now or System Will Start in Default Mode in 5 Seconds..: ");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}

		if (ui_get_mode_interactive() == 1) {
			sprintf(msg, "\n\rEnter Mode Selection Now: \n\r");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}

		for (k = 0; k < SERIAL_MSG_MAXLEN; k++) {
			Msg.Data[k] = 0;
		}
		tick_read_cycle_start = HAL_GetTick();
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
	BSP_MotorControl_SetMaxSpeed(0, max_speed);
	BSP_MotorControl_SetMinSpeed(0, min_speed);
	BSP_MotorControl_SetAcceleration(0, MAX_ACCEL);
	BSP_MotorControl_SetDeceleration(0, MAX_DECEL);

	if (ACCEL_CONTROL == 0) {
		sprintf(msg, "\n\rMotor Profile Speeds Set at Min %u Max %u Steps per Second",
				min_speed, max_speed);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}
	if (ctx->select_suspended_mode == 0) {
		sprintf(msg, "\n\rInverted Pendulum Mode Selected");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}
	if (ctx->select_suspended_mode == 1) {
		sprintf(msg, "\n\rSuspended Pendulum Mode Selected");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}

	sprintf(msg, "\n\rMotor Torque Current Set at %0.1f mA", ctx->torq_current_val);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

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

	if (ctx->plant.rotor_damping_coefficient != 0 || ctx->plant.rotor_natural_frequency != 0) {
		ctx->plant.Wn2 = ctx->plant.rotor_natural_frequency * ctx->plant.rotor_natural_frequency;
		ctx->plant.rotor_plant_gain = ctx->plant.rotor_plant_gain * ctx->plant.Wn2;
		ctx->plant.ao = ((2.0F / Tsample) * (2.0F / Tsample)
				+ (2.0F / Tsample) * 2.0F * ctx->plant.rotor_damping_coefficient
						* ctx->plant.rotor_natural_frequency
				+ ctx->plant.rotor_natural_frequency * ctx->plant.rotor_natural_frequency);
		ctx->plant.c0 = ((2.0F / Tsample) * (2.0F / Tsample) / ctx->plant.ao);
		ctx->plant.c1 = -2.0F * ctx->plant.c0;
		ctx->plant.c2 = ctx->plant.c0;
		ctx->plant.c3 = -(2.0F * ctx->plant.rotor_natural_frequency * ctx->plant.rotor_natural_frequency
				- 2.0F * (2.0F / Tsample) * (2.0F / Tsample)) / ctx->plant.ao;
		ctx->plant.c4 = -((2.0F / Tsample) * (2.0F / Tsample)
				- (2.0F / Tsample) * 2.0F * ctx->plant.rotor_damping_coefficient
						* ctx->plant.rotor_natural_frequency
				+ ctx->plant.rotor_natural_frequency * ctx->plant.rotor_natural_frequency) / ctx->plant.ao;
	}

	if (ctx->plant.enable_rotor_plant_design == 2) {
		ctx->plant.IWon_r = 2 / (ctx->plant.Wo_r * Tsample);
		ctx->plant.iir_0_r = 1 - (1 / (1 + ctx->plant.IWon_r));
		ctx->plant.iir_1_r = -ctx->plant.iir_0_r;
		ctx->plant.iir_2_r = (1 / (1 + ctx->plant.IWon_r)) * (1 - ctx->plant.IWon_r);
	}
}

void app_run_control_session(AppControlContext *ctx)
{
	int i = 0, k, m, ret;
	int encoder_position_curr, encoder_position_prev;
	motorDir_t swing_up_direction;
	int swing_up_state;
	int stage_count, stage_amp;

	ctx->enable_control_action = ENABLE_CONTROL_ACTION;

	if (ctx->reset_state == 1) {
		hardware_rotor_home();
	}
	ret = hardware_rotor_position_read(&rotor_position_steps);
	sprintf(msg, "\r\nPrepare for Control Start - Initial Rotor Position: %i\r\n",
			rotor_position_steps);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

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

	sprintf(msg, "Test for Pendulum at Rest - Waiting for Pendulum to Stabilize\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	encoder_position_init = 0;
	ret = hardware_encoder_position_read(&encoder_position_steps, encoder_position_init,
			&htim3);
	encoder_position_prev = encoder_position_steps;
	HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
	ret = hardware_encoder_position_read(&encoder_position_steps, encoder_position_init,
			&htim3);
	encoder_position_curr = encoder_position_steps;
	while (encoder_position_curr != encoder_position_prev) {
		ret = hardware_encoder_position_read(&encoder_position_steps,
				encoder_position_init, &htim3);
		encoder_position_prev = encoder_position_steps;
		HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
		ret = hardware_encoder_position_read(&encoder_position_steps,
				encoder_position_init, &htim3);
		encoder_position_curr = encoder_position_steps;

		if (encoder_position_prev == encoder_position_curr) {
			HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
			ret = hardware_encoder_position_read(&encoder_position_steps,
					encoder_position_init, &htim3);
			encoder_position_prev = encoder_position_steps;
			HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
			ret = hardware_encoder_position_read(&encoder_position_steps,
					encoder_position_init, &htim3);
			encoder_position_curr = encoder_position_steps;
			if (encoder_position_prev == encoder_position_curr) {
				break;
			}
		}
		sprintf(msg,
				"Pendulum Motion Detected with angle %0.2f - Waiting for Pendulum to Stabilize\r\n",
				(float) ((encoder_position_curr - encoder_position_prev)
						/ ENCODER_READ_ANGLE_SCALE));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}

	sprintf(msg, "Pendulum Now at Rest and Measuring Pendulum Down Angle\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	HAL_Delay(100);
	ret = hardware_encoder_position_read(&encoder_position_steps, encoder_position_init,
			&htim3);
	encoder_position_init = encoder_position_steps;

	if (ret == -1) {
		sprintf(msg, "Encoder Position Under Range Error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}
	if (ret == 1) {
		sprintf(msg, "Encoder Position Over Range Error\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}

	ret = hardware_encoder_position_read(&encoder_position_steps, encoder_position_init,
			&htim3);
	encoder_position_down = encoder_position_steps;
	sprintf(msg, "Pendulum Initial Angle %i\r\n", encoder_position_steps);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

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
			sprintf(msg,
					"Adjust Pendulum Upright By Turning CCW Control Will Start When Vertical\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}
	}

	if (ctx->enable_swing_up == 0) {
		uint32_t tick_wait_start = HAL_GetTick();
		if (ctx->select_suspended_mode == 0) {
			while (1) {
				ret = hardware_encoder_position_read(&encoder_position_steps,
						encoder_position_init, &htim3);
				if (fabs(
						encoder_position_steps - encoder_position_down
								- (int) (180 * ctx->angle_scale)) < START_ANGLE * ctx->angle_scale) {
					HAL_Delay(START_ANGLE_DELAY);
					break;
				}
				if (fabs(
						encoder_position_steps - encoder_position_down
								+ (int) (180 * ctx->angle_scale)) < START_ANGLE * ctx->angle_scale) {
					encoder_position_down = encoder_position_down
							- 2 * (int) (180 * ctx->angle_scale);
					HAL_Delay(START_ANGLE_DELAY);
					break;
				}
				uint32_t tick_wait = HAL_GetTick();

				if ((tick_wait - tick_wait_start)
						> PENDULUM_ORIENTATION_START_DELAY) {
					sprintf(msg,
							"Pendulum Upright Action Not Detected - Restarting ...\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
							HAL_MAX_DELAY);
					ctx->enable_control_action = 0;
					break;
				}
			}
		}
	}

	if (ctx->select_suspended_mode == 1) {
		sprintf(msg, "Suspended Mode Control Will Start in %i Seconds\r\n",
				(int) (CONTROL_START_DELAY / 1000));
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	}

	ctx->core_dual_pid_runtime.current_error_rotor_steps = 0;
	ctx->core_dual_pid_runtime.current_error_rotor_integral = 0;

	ctx->timing.cycle_count = CYCLE_LIMIT;
	rotor_position_steps = 0;
	rotor_position_steps_prev = 0;
	rotor_position_filter_steps = 0;
	rotor_position_filter_steps_prev = 0;
	rotor_position_command_steps = 0;
	rotor_position_diff = 0;
	rotor_position_diff_prev = 0;
	rotor_position_diff_filter = 0;
	rotor_position_diff_filter_prev = 0;
	rotor_position_step_polarity = 1;
	encoder_angle_slope_corr_steps = 0;
	rotor_sine_drive = 0;
	sine_drive_transition = 0;
	rotor_mod_control = 1.0;
	ctx->enable_adaptive_mode = 0;
	tick_cycle_start = HAL_GetTick();
	tick_cycle_previous = tick_cycle_start;
	tick_cycle_current = tick_cycle_start;
	ctx->timing.enable_cycle_delay_warning = ENABLE_CYCLE_DELAY_WARNING;
	chirp_cycle = 0;
	chirp_dwell_cycle = 0;
	pendulum_position_command_steps = 0;
	impulse_start_index = 0;
	ctx->mode_transition_state = 0;
	ctx->adaptive_state = 4;
	app_reset_command_shaper_state(ctx);
	rotor_position_command_steps_pf_prev = 0;
	ctx->enable_high_speed_sampling = ENABLE_HIGH_SPEED_SAMPLING_MODE;
	rotor_track_comb_command = 0;
	full_sysid_start_index = -1;
	ctx->timing.current_cpu_cycle = 0;
	ctx->speed_scale = DATA_REPORT_SPEED_SCALE;
	ctx->speed_governor = 0;
	encoder_position_offset = 0;
	encoder_position_offset_zero = 0;

	for (m = 0; m < ANGLE_CAL_OFFSET_STEP_COUNT + 1; m++) {
		offset_angle[m] = 0;
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
	ctx->init_params.enable_rotor_plant_design       = ctx->plant.enable_rotor_plant_design;
	ctx->init_params.enable_rotor_plant_gain_design  = ctx->plant.enable_rotor_plant_gain_design;

	if (ctx->select_suspended_mode == 1) {
		ctx->gains.load_disturbance_sensitivity_scale = 1.0;
	}
	if (ctx->select_suspended_mode == 0) {
		ctx->gains.load_disturbance_sensitivity_scale = LOAD_DISTURBANCE_SENSITIVITY_SCALE;
	}

	if (ctx->enable_swing_up == 1 && ctx->select_suspended_mode == 0) {
		ctx->core_ctl_state.PID_Rotor.Kp = 20;
		ctx->core_ctl_state.PID_Rotor.Ki = 10;
		ctx->core_ctl_state.PID_Rotor.Kd = 10;
		ctx->core_ctl_state.PID_Pend.Kp = 300;
		ctx->core_ctl_state.PID_Pend.Ki = 0.0;
		ctx->core_ctl_state.PID_Pend.Kd = 30.0;
		ctx->gains.enable_state_feedback = 0;
		ctx->gains.integral_compensator_gain = 0;
		ctx->gains.feedforward_gain = 1;
		rotor_position_command_steps = 0;
		ctx->gains.enable_state_feedback = 0;
		ctx->gains.enable_disturbance_rejection_step = 0;
		ctx->gains.enable_sensitivity_fnc_step = 0;
		ctx->gains.enable_noise_rejection_step = 0;
		ctx->plant.enable_rotor_plant_design = 0;
		ctx->plant.enable_rotor_plant_gain_design = 0;

		ctx->torq_current_val = MAX_TORQUE_SWING_UP;
		L6474_SetAnalogValue(0, L6474_TVAL, ctx->torq_current_val);

		sprintf(msg, "Pendulum Swing Up Starting\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		hardware_swing_up_reset();
		swing_up_state = 0;
		stage_count = 0;
		stage_amp = STAGE_0_AMP;

		swing_up_direction = FORWARD;
		BSP_MotorControl_Move(0, swing_up_direction, 150);
		BSP_MotorControl_WaitWhileActive(0);

		while (1) {
			SwingUpSensorState sus;
			HAL_Delay(2);
			ret = hardware_encoder_position_read(&encoder_position_steps,
					encoder_position_init, &htim3);
			hardware_swing_up_get(&sus);

			if (fabs(
					encoder_position_steps - encoder_position_down
							- (int) (180 * ctx->angle_scale)) < START_ANGLE * ctx->angle_scale) {
				break;
			}
			if (fabs(
					encoder_position_steps - encoder_position_down
							+ (int) (180 * ctx->angle_scale)) < START_ANGLE * ctx->angle_scale) {
				encoder_position_down = encoder_position_down
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
					ret = hardware_encoder_position_read(&encoder_position_steps,
							encoder_position_init, &htim3);
					hardware_swing_up_get(&sus);
				}
			}

			if (sus.peaked && !sus.handled_peak) {
				hardware_swing_up_handle_peak();
				swing_up_direction =
						swing_up_direction == FORWARD ? BACKWARD : FORWARD;
			}
		}
	}

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

	ret = hardware_encoder_position_read(&encoder_position_steps, encoder_position_init,
			&htim3);
	if (ctx->select_suspended_mode == 0) {
		encoder_position = encoder_position_steps - encoder_position_down
				- (int) (180 * ctx->angle_scale);
		encoder_position = encoder_position - encoder_position_offset;
	}

	app_init_control_pipeline(ctx, encoder_position_init, Tsample);

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
}
