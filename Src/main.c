
/*
 ******************************************************************************
 * @file    Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/main.c
 *
 *    Acknowledgments to the invaluable development, support and guidance by
 *    Marco De Fazio, Giorgio Mariano, Enrico Poli, and Davide Ghezzi
 *    of STMicroelectronics
 *
 *    Acknowledgements to the innovative development, support and guidance by
 *    Markus Dauberschmidt for development of the Pendulum Swing Up algorithm
 *    Please see https://github.com/OevreFlataeker/steval_edukit_swingup
 *
 *              Motor Control Curriculum Feedback Control System
 *
 * Includes:
 * 		Stepper Motor Control interface based on the IHM01A1 and Nucleo F401RE
 * 		Optical Encoder interface supported by the Nucleo F401RE
 * 		Primary PID controller for Pendulum Angle Control
 * 		Secondary PID controller for Rotor Angle Control
 * 		User interfaces for remote access to system configuration including
 * 				Stepper Motor speed profile, current limits, and others
 * 				Control system parameters
 *
 *
 * @author  William J. Kaiser (UCLA Electrical and Computer Engineering).
 *
 * Application based on development by STMicroelectronics as described below
 *
 * @version V2.0
 * @date    January 13, 2021
 *
 ******************************************************************************
 * @file    Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/main.c
 * @author  IPC Rennes
 * @version V1.10.0
 * @date    March 16th, 2018
 * @brief   This example shows how to use 1 IHM01A1 expansion board
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/*
 * Integrated Rotary Inverted Pendulum System Configuration
 *
 * ********** Primary Components *************************
 * Processor:			Nucleo F401RE
 * Motor Interface: 	IHM01A1 Stepper Motor Controller
 * Motor Power Supply:	12V 2A Supply
 * Encoder:				LPD3806-600BM-G5-24C 600 Pulse Per Revolution Incremental Rotary Encoder
 * Stepper Motor:		NEMA-17 17HS19-2004S  Stepper Motor
 *
 *********** Stepper Lead Assignment *********************
 *
 * Lead Color	IHM01A1 Terminal
 * 	 Blue			-A
 * 	 Red			+A
 * 	 Green			-B
 * 	 Black			+B
 *
 * Stepper Lead Extension Cable (if present) replaces Blue with White
 *
 * Caution: Please note that motors have been receieved from the vendor showing
 * reversal of Motor White and Motor Red.  Check rotor operation after assembly
 * and in initial testing.
 *
 * ********** Encoder Lead Assignment *********************
 *
 * Lead Color	Nucleo F401RE Terminal
 * 	Red 			5V
 * 	Black 			GND
 * 	White 			GPIO Dir3
 * 	Green 			GPIO Dir2
 *
 * Note: Optical Encoder is LPD3806-600BM-G5-24C This encoder requires an open collector pull up resistor.
 * Note: GPIO_PULLUP is set in HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder) of
 * stm32f4xx_hal_msp.c Line 217
 *
 * Initial Motor Speed Profiles, Torque Current, and Overcurrent Thresholds set in l6474_target_config.h
 *
 *
 *
 */




/*
 * System Configuration Parameters
 *
 * ENABLE_SUSPENDED_PENDULUM_CONTROL is set to 0 for Inverted Pendulum and
 * 									 set to 1 for Suspended Pendulum
 * ENABLE_DUAL_PID is set to 1 to enable control action
 *
 * High Speed 2 millisecond Control Loop delay, 500 Hz Cycle System Configuration
 *
 * Motor Speed Profile Configurations are:
 *
 *						MAX_ACCEL: 3000; 	MAX_DECEL 3000
 * High Speed Mode:		MAX_SPEED: 1000; 	MIN_SPEED 500
 * Medium Speed Mode:	MAX_SPEED: 1000; 	MIN_SPEED 300
 * Low Speed Mode:		MAX_SPEED: 1000; 	MIN_SPEED 200
 * Suspended Mode: 		MAX_SPEED: 1000; 	MIN_SPEED 200
 *
 */



/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "edukit_system.h"
#include "hardware.h"
#include "observer.h"
#include "controller.h"
#include "command_shaper.h"
#include "app_control.h"
#include "app_bootstrap.h"
#include "app_session.h"
#include "ui.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <stdlib.h>


/*
 * Motor Interface Data Structure
 *
 * Note that this is not required by default since Motor Profile is included
 * from l6474_target_config.h
 *
 * Note: This application is based on usage of l6474_target_config.h header
 * file for initialization of configuration of L6474
 */

L6474_Init_t gL6474InitParams = {
		MAX_ACCEL,           	/// Acceleration rate in step/s2. Range: (0..+inf).
		MAX_DECEL,           	/// Deceleration rate in step/s2. Range: (0..+inf).
		MAX_SPEED,              /// Maximum speed in step/s. Range: (30..10000].
		MIN_SPEED,              /// Minimum speed in step/s. Range: [30..10000).
		MAX_TORQUE_CONFIG, 		/// Torque regulation current in mA. (TVAL register) Range: 31.25mA to 4000mA.
		OVERCURRENT_THRESHOLD, 	/// Overcurrent threshold (OCD_TH register). Range: 375mA to 6000mA.
		L6474_CONFIG_OC_SD_ENABLE, /// Overcurrent shutwdown (OC_SD field of CONFIG register).
		L6474_CONFIG_EN_TQREG_TVAL_USED, /// Torque regulation method (EN_TQREG field of CONFIG register).
		L6474_STEP_SEL_1_16, 	/// Step selection (STEP_SEL field of STEP_MODE register).
		L6474_SYNC_SEL_1_2, 	/// Sync selection (SYNC_SEL field of STEP_MODE register).
		L6474_FAST_STEP_12us, 	/// Fall time value (T_FAST field of T_FAST register). Range: 2us to 32us.
		L6474_TOFF_FAST_8us, 	/// Maximum fast decay time (T_OFF field of T_FAST register). Range: 2us to 32us.
		3,   					/// Minimum ON time in us (TON_MIN register). Range: 0.5us to 64us.
		21, 					/// Minimum OFF time in us (TOFF_MIN register). Range: 0.5us to 64us.
		L6474_CONFIG_TOFF_044us, /// Target Swicthing Period (field TOFF of CONFIG register).
		L6474_CONFIG_SR_320V_us, /// Slew rate (POW_SR field of CONFIG register).
		L6474_CONFIG_INT_16MHZ, /// Clock setting (OSC_CLK_SEL field of CONFIG register).
		(L6474_ALARM_EN_OVERCURRENT | L6474_ALARM_EN_THERMAL_SHUTDOWN
				| L6474_ALARM_EN_THERMAL_WARNING | L6474_ALARM_EN_UNDERVOLTAGE
				| L6474_ALARM_EN_SW_TURN_ON | L6474_ALARM_EN_WRONG_NPERF_CMD) /// Alarm (ALARM_EN register).
};

 /* CMSIS */
#define ARM_MATH_CM4

/*
 * Apply Swing Up algorithm developed by Markus Dauberschmidt
 */

#define swing_up 1

/* Private function prototypes -----------------------------------------------*/
/* read_float, read_int, read_char, user_configuration → declared in ui.h */

/* Delay_Pulse, Main_StepClockHandler, apply_acceleration → moved to hardware.c */

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
T_Serial_Msg Msg;

uint8_t RxBuffer[UART_RX_BUFFER_SIZE];

/* apply_acc_start_time, clock_int_time, clock_int_tick,
   desired_pwm_period, current_pwm_period, target_velocity_prescaled
   → moved to hardware.c */
/* enable_speed_prescale → write-only (never read), removed */

/* System data reporting */
char tmp_string[256];
char msg[192];
/* msg_pad → never referenced, removed */
char test_msg[128];

/*
 * Timer 3, UART Transmit, and UART DMA Receive declarations
 */

TIM_HandleTypeDef htim3;

static AppControlContext g_app = {
	.core_observer_ops = &OBSERVER_OPS_DEFAULT,
	.core_controller_ops = &CONTROLLER_OPS_DEFAULT,
	.core_command_shaper_ops = &COMMAND_SHAPER_OPS_DEFAULT,
};

/* Control system output signal */
float rotor_control_target_steps;
float rotor_control_target_steps_curr;

/* Control system variables */
/* rotor_position_delta, initial_rotor_position → never referenced, removed */
int cycle_count;
/* i, j, k, m, ret → local in app_run_control_session() */

/* PID control system variables */
static float _sample_period, _sample_period_rotor;
float *sample_period       = &_sample_period;
float *sample_period_rotor = &_sample_period_rotor;

/* PID control variables */
static float _deriv_lp_corner_f, _deriv_lp_corner_f_rotor;
float *deriv_lp_corner_f       = &_deriv_lp_corner_f;
float *deriv_lp_corner_f_rotor = &_deriv_lp_corner_f_rotor;
float proportional, rotor_p_gain;
float integral, rotor_i_gain;
float derivative, rotor_d_gain;

/* State Feedback variables */
int enable_state_feedback;
float integral_compensator_gain;
float feedforward_gain;
/* Reference tracking command */
float reference_tracking_command;

/* Pendulum position and tracking command */

/* Rotor position and tracking command */
int rotor_position_steps;
float rotor_position_command_steps;
float rotor_position_command_steps_pf, rotor_position_command_steps_pf_prev;
/* rotor_position_command_deg → ui.c static */
float rotor_position_steps_prev, rotor_position_filter_steps, rotor_position_filter_steps_prev;
float rotor_position_diff, rotor_position_diff_prev;
float rotor_position_diff_filter, rotor_position_diff_filter_prev;
/* rotor_target_in_steps → never referenced, removed */

/* Rotor Plant Design variables */
int select_rotor_plant_design, enable_rotor_plant_design, enable_rotor_plant_gain_design;
/* rotor_control_target_steps_int → never referenced, removed */
float rotor_damping_coefficient, rotor_natural_frequency;
float rotor_plant_gain;
float c0, c1, c2, c3, c4, ao, Wn2;
float fo_r, Wo_r, IWon_r, iir_0_r, iir_1_r, iir_2_r;

/* Encoder position variables */
/* cnt3, range_error → moved to hardware.c */
/* previous_encoder_position, max/global/prev_global_max_encoder_position → hardware.c static */
float encoder_position;
int encoder_position_steps;
int encoder_position_init;
int encoder_position_down;
/* encoder_position_curr, encoder_position_prev → local in app_run_control_session() */

/* Angle calibration variables */
float encoder_position_offset;
float encoder_position_offset_zero;
int enable_angle_cal;
/* enable_angle_cal_resp → ui.c static */
int offset_end_state;
int offset_start_index;
int angle_index;
int angle_avg_index;
int angle_avg_span;
int offset_angle[ANGLE_CAL_OFFSET_STEP_COUNT + 2];
float encoder_position_offset_avg[ANGLE_CAL_OFFSET_STEP_COUNT + 2];
int angle_cal_end;
int angle_cal_complete;

/* Swing Up system variables */
int enable_swing_up;
/* enable_swing_up_resp → ui.c static */
/* peaked, handled_peak, zero_crossed,
   max/global/prev_global_max_encoder_position, previous_encoder_position → hardware.c static */
/* swing_up_direction, swing_up_state/prev, stage_count, stage_amp → local in app_run_control_session() */

/* init_r_*, init_p_*, init_enable_* → ctx->init_params in AppControlContext */

/* Low pass filter variables */
float fo, Wo, IWon, iir_0, iir_1, iir_2;
float fo_LT, Wo_LT, IWon_LT;
float iir_LT_0, iir_LT_1, iir_LT_2;
float fo_s, Wo_s, IWon_s, iir_0_s, iir_1_s, iir_2_s;

/* Slope correction system variables */
/* slope, slope_prev → removed (write-only, never read) */
float encoder_angle_slope_corr_steps;

/* Adaptive control variables */
/* adaptive_error, adaptive_threshold_low/high, error_sum_prev, error_sum,
   error_sum_filter_prev, error_sum_filter, adaptive_entry_tick,
   adaptive_dwell_period → removed (write-only, never read) */
int enable_adaptive_mode, adaptive_state, adaptive_state_change;

/* Rotor impulse variables */
int rotor_position_step_polarity;
int impulse_start_index;

/* User configuration variables */
/* clear_input, max_speed_read, min_speed_read, motor_response_model,
   enable_rotor_actuator_high_speed_test → removed (never referenced) */
/* enable_encoder_test, motor_state → ui.c static */
uint32_t enable_control_action;
int select_suspended_mode;
int enable_rotor_actuator_test, enable_rotor_actuator_control;
int enable_motor_actuator_characterization_mode;
float torq_current_val;


/* Rotor chirp system variables */
int enable_rotor_chirp;
int chirp_cycle;
int chirp_dwell_cycle;
float chirp_time;
float rotor_chirp_start_freq;
float rotor_chirp_end_freq;
float rotor_chirp_period ;
float rotor_chirp_frequency;
/* rotor_chirp_amplitude, rotor_chirp_step_period → ui.c static */

float pendulum_position_command_steps;

/* Modulates sine tracking signal system variables */
int enable_mod_sin_rotor_tracking;
int enable_rotor_position_step_response_cycle;
int disable_mod_sin_rotor_tracking;
int sine_drive_transition;
float mod_sin_amplitude;
float rotor_control_sin_amplitude;
float rotor_sine_drive, rotor_sine_drive_mod;
float rotor_mod_control;
float mod_sin_carrier_frequency;

/* Pendulum impulse system variables */
int enable_pendulum_position_impulse_response_cycle;

/* Rotor high speed test system variables */
/* swing_cycles, start/end_angle_a, motion_dwell_a,
   abs_encoder_position_prior/after/max → removed (never referenced) */
/* rotor_test_speed_min/max, rotor_test_acceleration_max, swing_deceleration_max,
   current_speed → ui.c static */

/*Pendulum system ID variable */
/* enable_pendulum_sysid_test → ui.c static */

/* Full system identification variables */
/* enable_full_sysid, full_sysid_max_freq_hz → ui.c static */
float full_sysid_max_vel_amplitude_deg_per_s;
float full_sysid_min_freq_hz;
int full_sysid_num_freqs;
float full_sysid_freq_log_step;
int full_sysid_start_index;

/* Rotor comb drive system variables */
int enable_rotor_tracking_comb_signal;
float rotor_track_comb_signal_frequency;
float rotor_track_comb_command;
float rotor_track_comb_amplitude;

/* Sensitivity function system variables */
int enable_disturbance_rejection_step;
int enable_noise_rejection_step;
/* enable_plant_rejection_step → removed (never referenced) */
int enable_sensitivity_fnc_step;
float load_disturbance_sensitivity_scale;

/* Noise rejection sensitivity function low pass filter */
/* noise_rej_signal_filter, noise_rej_signal_prev, noise_rej_signal_filter_prev
   → removed (write-only, never read) */
float noise_rej_signal;

/*
 * Real time user input system variables
 */

float adjust_increment;

/* Real time data reporting index */
int report_mode;
int speed_scale;
int speed_governor;

/* mode_1..mode_19, mode_quit, mode_adaptive*, mode_string_* → moved to ui.c (static) */
/* mode_index_prev → write-only, removed; mode_index_command → ui.c static */
/* mode_transition_tick → never used, removed */
int mode_transition_state;


/* message_received → never referenced, removed */
/* mode_string_mode_high_speed_test, mode_string_mode_pendulum_sysid_test → removed (unused) */

/* System timing variables */

uint32_t tick, tick_cycle_current, tick_cycle_previous, tick_cycle_start,
tick_read_cycle, tick_read_cycle_start;

float Tsample, Tsample_rotor;
/* test_time → write-only (never read), removed */
float angle_scale;
int enable_high_speed_sampling;

/* Reset state tracking */
int reset_state;

/* Motor configuration */
uint16_t min_speed, max_speed, max_accel, max_decel;

int main(void) {
	/* Initialize reset state indicating that reset has occurred */

	reset_state = 1;

	/* Initialize and enable cycle counter */
	ITM->LAR = 0xC5ACCE55; 	// at address 0xE0001FB0
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // at address 0xE000EDFC, CoreDebug_DEMCR_TRCENA_Msk = 0x01000000
	DWT->CTRL |= 1; 		// at address 0xE0001000
	DWT->CYCCNT = 0; 		// at address 0xE0001004

	/* initialize Integrator Mode time variables */
	apply_acc_start_time = 0;
	clock_int_time = 0;
	clock_int_tick = 0;

	/* Initialize PWM period variables used by step interrupt */
	desired_pwm_period = 0;
	current_pwm_period = 0;
	target_velocity_prescaled = 0;

	/* Initialize default start mode and reporting mode */
	report_mode = 1;

	app_bootstrap_system(&g_app, &gL6474InitParams);
	app_run_mode_loop(&g_app);
	return 0;
}

/*
 ******************************************************************************
 *
 * Edukit System Functions Definitions
 *
 ******************************************************************************
 */

/* pid_filter_control_execute → moved to controller.c as static pid_execute */

/* encoder_position_read, oppositeSigns, rotor_position_set, rotor_position_read
   → moved to hardware.c */

/* read_float, read_int, read_char → moved to ui.c */

/* assign_mode_1, assign_mode_2, assign_mode_3 → moved to ui.c */

/* mode_index_identification → moved to ui.c */


/* set_mode_strings, user_prompt, get_user_mode_index → moved to ui.c */

/* user_configuration → moved to ui.c */
/* rotor_encoder_test, motor_actuator_characterization_mode, interactive_rotor_actuator_control → moved to ui.c */

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/






