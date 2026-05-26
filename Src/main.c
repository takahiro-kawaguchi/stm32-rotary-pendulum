
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
#include "app_runtime.h"
#include "app_bootstrap.h"
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
int32_t enable_speed_prescale;

/* System data reporting */
char tmp_string[256];
char msg[192];
char msg_pad[64];
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

/* Transitional aliases: keep call sites stable while globals are consolidated. */

/*
  * Timer 3, UART Transmit, and UART DMA Receive declarations
  */

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* Control system output signal */
float rotor_control_target_steps;
float rotor_control_target_steps_curr;

/* Control system variables */
int rotor_position_delta;
int initial_rotor_position;
int cycle_count;
int i, j, k, m;
int ret;

/* PID control system variables */
float windup, rotor_windup;
static float _current_error_steps, _current_error_rotor_steps;
float *current_error_steps     = &_current_error_steps;
float *current_error_rotor_steps = &_current_error_rotor_steps;
static float _sample_period, _sample_period_rotor;
float *sample_period       = &_sample_period;
float *sample_period_rotor = &_sample_period_rotor;

/* Loop timing measurement variables */
int cycle_period_start;
int cycle_period_sum;
int enable_cycle_delay_warning;

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
float current_error_rotor_integral;

/* Reference tracking command */
float reference_tracking_command;

/* Pendulum position and tracking command */

/* Rotor position and tracking command */
int rotor_position_steps;
float rotor_position_command_steps;
float rotor_position_command_steps_pf, rotor_position_command_steps_pf_prev;
float rotor_position_command_deg;
float rotor_position_steps_prev, rotor_position_filter_steps, rotor_position_filter_steps_prev;
float rotor_position_diff, rotor_position_diff_prev;
float rotor_position_diff_filter, rotor_position_diff_filter_prev;
int rotor_target_in_steps;
int initial_rotor_position;

/* Rotor Plant Design variables */
int select_rotor_plant_design, enable_rotor_plant_design, enable_rotor_plant_gain_design;
int rotor_control_target_steps_int;
float rotor_damping_coefficient, rotor_natural_frequency;
float rotor_plant_gain;
float c0, c1, c2, c3, c4, ao, Wn2;
float fo_r, Wo_r, IWon_r, iir_0_r, iir_1_r, iir_2_r;

/* Encoder position variables */
/* cnt3, range_error → moved to hardware.c */
float encoder_position;
int encoder_position_steps;
int encoder_position_init;
int previous_encoder_position;
int max_encoder_position;
int global_max_encoder_position;
int prev_global_max_encoder_position;
int encoder_position_down;
int encoder_position_curr;
int encoder_position_prev;

/* Angle calibration variables */
float encoder_position_offset;
float encoder_position_offset_zero;
int enable_angle_cal;
int enable_angle_cal_resp;
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
int enable_swing_up_resp;
bool peaked;
bool handled_peak;
int zero_crossed;
motorDir_t swing_up_direction;
int swing_up_state, swing_up_state_prev;
int stage_count;
int stage_amp;

/* Initial control state parameter storage */
float init_r_p_gain, init_r_i_gain, init_r_d_gain;
float init_p_p_gain, init_p_i_gain, init_p_d_gain;
int init_enable_state_feedback;
float init_integral_compensator_gain;
float init_feedforward_gain;
int init_enable_state_feedback;
int init_enable_disturbance_rejection_step;
int init_enable_sensitivity_fnc_step;
int init_enable_noise_rejection_step;
int init_enable_rotor_plant_design;
int init_enable_rotor_plant_gain_design;

/* Low pass filter variables */
float fo, Wo, IWon, iir_0, iir_1, iir_2;
float fo_LT, Wo_LT, IWon_LT;
float iir_LT_0, iir_LT_1, iir_LT_2;
float fo_s, Wo_s, IWon_s, iir_0_s, iir_1_s, iir_2_s;

/* Slope correction system variables */
int slope;
int slope_prev;
float encoder_angle_slope_corr_steps;

/* Adaptive control variables */
float adaptive_error, adaptive_threshold_low, adaptive_threshold_high;
float error_sum_prev, error_sum, error_sum_filter_prev, error_sum_filter;
int adaptive_entry_tick, adaptive_dwell_period;
int enable_adaptive_mode, adaptive_state, adaptive_state_change;

/* Rotor impulse variables */
int rotor_position_step_polarity;
int impulse_start_index;

/* User configuration variables */
int clear_input;
uint32_t enable_control_action;
int max_speed_read, min_speed_read;
int select_suspended_mode;
int motor_response_model;
int enable_rotor_actuator_test, enable_rotor_actuator_control;
int enable_encoder_test;
int enable_rotor_actuator_high_speed_test;
int enable_motor_actuator_characterization_mode;
int motor_state;
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
float rotor_chirp_amplitude;
int rotor_chirp_step_period;

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
int swing_cycles, rotor_test_speed_min, rotor_test_speed_max;
int rotor_test_acceleration_max, swing_deceleration_max;
int start_angle_a[20], end_angle_a[20], motion_dwell_a[20];
int abs_encoder_position_prior, abs_encoder_position_after, abs_encoder_position_max;
uint16_t current_speed;

/*Pendulum system ID variable */
int enable_pendulum_sysid_test;

/* Full system identification variables */
int enable_full_sysid;
float full_sysid_max_vel_amplitude_deg_per_s;
float full_sysid_min_freq_hz;
float full_sysid_max_freq_hz;
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
int enable_plant_rejection_step;
int enable_sensitivity_fnc_step;
float load_disturbance_sensitivity_scale;

/* Noise rejection sensitivity function low pass filter */

float noise_rej_signal_filter, noise_rej_signal;
float noise_rej_signal_prev, noise_rej_signal_filter_prev;

/*
 * Real time user input system variables
 */

char config_message[16];
int config_command;
int display_parameter;
int step_size;
float adjust_increment;
int mode_index;

/* Real time data reporting index */
int report_mode;
int speed_scale;
int speed_governor;

/*
 * User selection mode values
 */

int mode_1;				// Enable LQR Motor Model M
int mode_2;				// Enable LRR Motor Model H
int mode_3;				// Enable LQR Motor Model L
int mode_4;				// Enable Suspended Mode Motor Model M
int mode_5;				// Enable sin drive track signal
int mode_adaptive_off;	// Disable adaptive control
int mode_adaptive;		// Enable adaptive control
int mode_8;				// Enable custom configuration entry
int mode_9;				// Disable sin drive track signal
int mode_10;			// Enable Single PID Mode with Motor Model M
int mode_11;			// Enable rotor actuator and encoder test mode
int mode_13;			// Enable rotor control system evaluation
int mode_15;			// Enable interactive control of rotor actuator
int mode_16;			// Enable load disturbance function step mode
int mode_17;			// Enable noise disturbance function step mode
int mode_18;			// Enable sensitivity function step mode
int mode_19;            // Enable full system identification mode
int mode_quit;			// Initiate exit from control loop
int mode_interactive;	// Enable continued terminal interactive user session
int mode_index_prev, mode_index_command;
int mode_transition_tick;
int mode_transition_state;
int transition_to_adaptive_mode;


/*
 * Real time user input characters
 */

char mode_string_stop[UART_RX_BUFFER_SIZE];
char mode_string_mode_1[UART_RX_BUFFER_SIZE];
char mode_string_mode_2[UART_RX_BUFFER_SIZE];
char mode_string_mode_3[UART_RX_BUFFER_SIZE];
char mode_string_mode_4[UART_RX_BUFFER_SIZE];
char mode_string_mode_8[UART_RX_BUFFER_SIZE];
char mode_string_mode_5[UART_RX_BUFFER_SIZE];
char mode_string_inc_accel[UART_RX_BUFFER_SIZE];
char mode_string_dec_accel[UART_RX_BUFFER_SIZE];
char mode_string_inc_amp[UART_RX_BUFFER_SIZE];
char mode_string_dec_amp[UART_RX_BUFFER_SIZE];
char mode_string_mode_single_pid[UART_RX_BUFFER_SIZE];
char mode_string_mode_test[UART_RX_BUFFER_SIZE];
char mode_string_mode_control[UART_RX_BUFFER_SIZE];
char mode_string_mode_motor_characterization_mode[UART_RX_BUFFER_SIZE];
char mode_string_mode_load_dist[UART_RX_BUFFER_SIZE];
char mode_string_mode_load_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_mode_noise_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_mode_plant_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_mode_full_sysid[UART_RX_BUFFER_SIZE];
char mode_string_dec_pend_p[UART_RX_BUFFER_SIZE];
char mode_string_inc_pend_p[UART_RX_BUFFER_SIZE];
char mode_string_dec_pend_i[UART_RX_BUFFER_SIZE];
char mode_string_inc_pend_i[UART_RX_BUFFER_SIZE];
char mode_string_dec_pend_d[UART_RX_BUFFER_SIZE];
char mode_string_inc_pend_d[UART_RX_BUFFER_SIZE];
char mode_string_dec_rotor_p[UART_RX_BUFFER_SIZE];
char mode_string_inc_rotor_p[UART_RX_BUFFER_SIZE];
char mode_string_dec_rotor_i[UART_RX_BUFFER_SIZE];
char mode_string_inc_rotor_i[UART_RX_BUFFER_SIZE];
char mode_string_dec_rotor_d[UART_RX_BUFFER_SIZE];
char mode_string_inc_rotor_d[UART_RX_BUFFER_SIZE];
char mode_string_dec_torq_c[UART_RX_BUFFER_SIZE];
char mode_string_inc_torq_c[UART_RX_BUFFER_SIZE];
char mode_string_dec_max_s[UART_RX_BUFFER_SIZE];
char mode_string_inc_max_s[UART_RX_BUFFER_SIZE];
char mode_string_dec_min_s[UART_RX_BUFFER_SIZE];
char mode_string_inc_min_s[UART_RX_BUFFER_SIZE];
char mode_string_dec_max_a[UART_RX_BUFFER_SIZE];
char mode_string_inc_max_a[UART_RX_BUFFER_SIZE];
char mode_string_dec_max_d[UART_RX_BUFFER_SIZE];
char mode_string_inc_max_d[UART_RX_BUFFER_SIZE];
char mode_string_enable_step[UART_RX_BUFFER_SIZE];
char mode_string_disable_step[UART_RX_BUFFER_SIZE];
char mode_string_enable_pendulum_impulse[UART_RX_BUFFER_SIZE];
char mode_string_disable_pendulum_impulse[UART_RX_BUFFER_SIZE];
char mode_string_enable_load_dist[UART_RX_BUFFER_SIZE];
char mode_string_disable_load_dist[UART_RX_BUFFER_SIZE];
char mode_string_enable_noise_rej_step[UART_RX_BUFFER_SIZE];
char mode_string_disable_noise_rej_step[UART_RX_BUFFER_SIZE];
char mode_string_disable_sensitivity_fnc_step[UART_RX_BUFFER_SIZE];
char mode_string_enable_sensitivity_fnc_step[UART_RX_BUFFER_SIZE];
char mode_string_inc_step_size[UART_RX_BUFFER_SIZE];
char mode_string_dec_step_size[UART_RX_BUFFER_SIZE];
char mode_string_select_mode_5[UART_RX_BUFFER_SIZE];
char mode_string_enable_high_speed_sampling[UART_RX_BUFFER_SIZE];
char mode_string_disable_high_speed_sampling[UART_RX_BUFFER_SIZE];
char mode_string_enable_speed_prescale[UART_RX_BUFFER_SIZE];
char mode_string_disable_speed_prescale[UART_RX_BUFFER_SIZE];
char mode_string_disable_speed_governor[UART_RX_BUFFER_SIZE];
char mode_string_enable_speed_governor[UART_RX_BUFFER_SIZE];
char mode_string_reset_system[UART_RX_BUFFER_SIZE];


int char_mode_select;	// Flag detecting whether character mode select entered


char message_received[UART_RX_BUFFER_SIZE];
char mode_string_mode_1[UART_RX_BUFFER_SIZE];
char mode_string_mode_2[UART_RX_BUFFER_SIZE];
char mode_string_mode_3[UART_RX_BUFFER_SIZE];
char mode_string_mode_4[UART_RX_BUFFER_SIZE];
char mode_string_mode_5[UART_RX_BUFFER_SIZE];
char mode_string_mode_8[UART_RX_BUFFER_SIZE];
char mode_string_mode_single_pid[UART_RX_BUFFER_SIZE];
char mode_string_mode_test[UART_RX_BUFFER_SIZE];
char mode_string_mode_control[UART_RX_BUFFER_SIZE];
char mode_string_mode_high_speed_test[UART_RX_BUFFER_SIZE];
char mode_string_mode_motor_characterization_mode[UART_RX_BUFFER_SIZE];
char mode_string_mode_pendulum_sysid_test[UART_RX_BUFFER_SIZE];
char mode_string_dec_accel[UART_RX_BUFFER_SIZE];
char mode_string_inc_accel[UART_RX_BUFFER_SIZE];
char mode_string_inc_amp[UART_RX_BUFFER_SIZE];
char mode_string_dec_amp[UART_RX_BUFFER_SIZE];
char mode_string_mode_load_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_mode_noise_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_mode_plant_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_stop[UART_RX_BUFFER_SIZE];

/* System timing variables */

uint32_t tick, tick_cycle_current, tick_cycle_previous, tick_cycle_start,
tick_read_cycle, tick_read_cycle_start,tick_wait_start,tick_wait;

volatile uint32_t current_cpu_cycle, prev_cpu_cycle, last_cpu_cycle, target_cpu_cycle, prev_target_cpu_cycle;
volatile int current_cpu_cycle_delay_relative_report;

uint32_t t_sample_cpu_cycles;
float Tsample, Tsample_rotor, test_time;
float angle_scale;
int enable_high_speed_sampling;

/* Reset state tracking */
int reset_state;

/* Motor configuration */
uint16_t min_speed, max_speed, max_accel, max_decel;

/* Serial interface variables */
uint32_t RxBuffer_ReadIdx;
uint32_t RxBuffer_WriteIdx;
uint32_t readBytes;


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
	mode_index = 1;
	report_mode = 1;

	app_bootstrap_system(&g_app, &gL6474InitParams);

	while (1) {

		mode_interactive = 0;
		user_prompt();

		/*
		 * If user has responded to previous query for configuration, then system remains in interactive mode
		 * and default state is not automatically enabled
		 */


		if (mode_interactive == 0) {
			sprintf(msg, "\n\rEnter Mode Selection Now or System Will Start in Default Mode in 5 Seconds..: ");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}

		/*
		 * If user has responded to query for configuration, then system remains in interactive mode
		 * and default state is not automatically enabled
		 */

		if (mode_interactive == 1) {
			sprintf(msg, "\n\rEnter Mode Selection Now: \n\r");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}

		/* Flush read buffer  */
		for (k = 0; k < SERIAL_MSG_MAXLEN; k++) { Msg.Data[k] = 0; }
		/* Start timer for configuration command read loop */
		tick_read_cycle_start = HAL_GetTick();
		/* Configuration command read loop */
		user_configuration();

		/* Set Motor Speed Profile and torque current */
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
		BSP_MotorControl_SetMaxSpeed(0, max_speed);
		BSP_MotorControl_SetMinSpeed(0, min_speed);
		BSP_MotorControl_SetAcceleration(0, MAX_ACCEL);
		BSP_MotorControl_SetDeceleration(0, MAX_DECEL);

		/* Report configuration values */
		if (ACCEL_CONTROL == 0){
		sprintf(msg, "\n\rMotor Profile Speeds Set at Min %u Max %u Steps per Second",
				min_speed, max_speed);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg,
				strlen(msg), HAL_MAX_DELAY);
		}
			if (select_suspended_mode == 0){
		sprintf(msg, "\n\rInverted Pendulum Mode Selected");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg,
				strlen(msg), HAL_MAX_DELAY);
			}
			if (select_suspended_mode == 1){
		sprintf(msg, "\n\rSuspended Pendulum Mode Selected");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg,
				strlen(msg), HAL_MAX_DELAY);
			}

		sprintf(msg, "\n\rMotor Torque Current Set at %0.1f mA",
				torq_current_val);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg,
				strlen(msg), HAL_MAX_DELAY);

		/* Motor Control Characterization Test*/
		if (enable_motor_actuator_characterization_mode == 1) {
			motor_actuator_characterization_mode();
		}
		/* Interactive digital motor control system */
		if (enable_rotor_actuator_control == 1) {
			interactive_rotor_actuator_control();
		}

		/*
		 * 	Rotor and Encoder Test Sequence will execute by moving rotor and reportin angle values
		 * 	as well as requesting pendulum motion followed by reporting of pendulum angles
		 *
		 * 	Agreement between actions and reported values confirms proper installation of actuator
		 * 	and pendulum encoder.
		 *
		 */

		if (enable_rotor_actuator_test == 1) {
			rotor_encoder_test();
		}

		/*
		 * Configure Primary and Secondary PID controller data structures
		 * Scale by CONTROLLER_GAIN_SCALE set to default value of unity
		 */


		app_assign_pid_gains_from_user(&g_app);

		integral_compensator_gain = integral_compensator_gain * CONTROLLER_GAIN_SCALE;

		/* Assign Rotor Plant Design variable values */


		/* Transfer function model of form 1/(s^2 + 2*Damping_Coefficient*Wn*s + Wn^2) */
		if (rotor_damping_coefficient != 0 || rotor_natural_frequency != 0){
			Wn2 = rotor_natural_frequency * rotor_natural_frequency;
			rotor_plant_gain = rotor_plant_gain * Wn2;
			ao = ((2.0F/Tsample)*(2.0F/Tsample) + (2.0F/Tsample)*2.0F*rotor_damping_coefficient*rotor_natural_frequency
					+ rotor_natural_frequency*rotor_natural_frequency);
			c0 = ((2.0F/Tsample)*(2.0F/Tsample)/ao);
			c1 = -2.0F * c0;
			c2 = c0;
			c3 = -(2.0F*rotor_natural_frequency*rotor_natural_frequency - 2.0F*(2.0F/Tsample)*(2.0F/Tsample))/ao;
			c4 = -((2.0F/Tsample)*(2.0F/Tsample) - (2.0F/Tsample)*2.0F*rotor_damping_coefficient*rotor_natural_frequency
					+ rotor_natural_frequency*rotor_natural_frequency)/ao;
		}

		/* Transfer function model of form 1/(s^2 + Wn*s) */
		if (enable_rotor_plant_design == 2){
			IWon_r = 2 / (Wo_r * Tsample);
			iir_0_r = 1 - (1 / (1 + IWon_r));
			iir_1_r = -iir_0_r;
			iir_2_r = (1 / (1 + IWon_r)) * (1 - IWon_r);
		}

		/* Optional Transfer function model of form Wn/(s^3 + Wn*s^2)
		if (enable_rotor_plant_design == 3 && enable_state_feedback == 0){
		      IWon_r = 2 / (Wo_r * Tsample);
		      iir_0_r = 1 / (1 + IWon_r);
		      iir_1_r = iir_0_r;
		      iir_2_r = iir_0_r * (1 - IWon_r);
		}
		*/

		/*

		//Optional display coefficients for rotor plant design transfer function
		sprintf(tmp_string, "\n\rEnable Design: %i iir_0 %0.4f iir_1 %0.4f iir_2 %0.4f\n\r", enable_rotor_plant_design, iir_0_r, iir_1_r, iir_2_r);
		HAL_UART_Transmit(&huart2, (uint8_t*) tmp_string, strlen(tmp_string), HAL_MAX_DELAY);


		//Optional display coefficients for rotor plant design transfer function
		sprintf(tmp_string,
				"\n\ra0 %0.4f c0 %0.4f c1 %0.4f c2 %0.4f c3 %0.4f c4 %0.4f\n\r", ao, c0, c1, c2, c3, c4);
		HAL_UART_Transmit(&huart2, (uint8_t*) tmp_string, strlen(tmp_string), HAL_MAX_DELAY);

		 */


		/*
		 * *************************************************************************************************
		 *
		 * Control System Initialization Sequence
		 *
		 * *************************************************************************************************
		 */

		/* Setting enable_control_action enables control loop */
		enable_control_action = ENABLE_CONTROL_ACTION;

		/*
		 * Set Motor Position Zero occuring only once after reset and suppressed thereafter
		 * to maintain angle calibration
		 */

		if (reset_state == 1){
			hardware_rotor_home();
		}
		ret = hardware_rotor_position_read(&rotor_position_steps);
		sprintf(msg,
				"\r\nPrepare for Control Start - Initial Rotor Position: %i\r\n",
				rotor_position_steps);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);


		/*
		 * Determination of vertical down orientation of the pendulum is required
		 * to establish the reference angle for the vertical upward control setpoint.
		 *
		 * This is measured when the pendulum is determined to be motionless.
		 *
		 * The user is informed to allow the pendulum to remain at rest.
		 *
		 * Motion is detected in the loop below.  Exit from the loop and
		 * initiation of control occurs next.
		 *
		 * Prior to measurement, and due to previous action, the Pendulum may be poised
		 * at upright orientation.
		 *
		 * A small stimulus is applied to ensure Pendulum will fall to Suspended orientation
		 * in the event that it may be finely balanced in the vertical position
		 *
		 */

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
		ret = hardware_encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
		encoder_position_prev = encoder_position_steps;
		HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
		ret = hardware_encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
		encoder_position_curr = encoder_position_steps;
		while (encoder_position_curr != encoder_position_prev) {
			ret = hardware_encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
			encoder_position_prev = encoder_position_steps;
			HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
			ret = hardware_encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
			encoder_position_curr = encoder_position_steps;

			/*
			 * Ensure stability reached with final motion test
			 */

			if (encoder_position_prev == encoder_position_curr) {
				HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
				ret = hardware_encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
				encoder_position_prev = encoder_position_steps;
				HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
				ret = hardware_encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
				encoder_position_curr = encoder_position_steps;
				if (encoder_position_prev == encoder_position_curr) {
					break;
				}
			}
			/* Alert user of undesired motion */
			sprintf(msg, "Pendulum Motion Detected with angle %0.2f - Waiting for Pendulum to Stabilize\r\n",
					(float) ((encoder_position_curr - encoder_position_prev)
							/ ENCODER_READ_ANGLE_SCALE));
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
		}

		sprintf(msg, "Pendulum Now at Rest and Measuring Pendulum Down Angle\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

		/* Calibrate down angle */

		/*
		 * Initialize Pendulum Angle Read offset by setting encoder_position_init
		 */

		HAL_Delay(100);
		ret = hardware_encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
		encoder_position_init = encoder_position_steps;

		if (ret == -1) {
			sprintf(msg, "Encoder Position Under Range Error\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
		}
		if (ret == 1) {
			sprintf(msg, "Encoder Position Over Range Error\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
		}

		ret = hardware_encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
		encoder_position_down = encoder_position_steps;
		sprintf(msg, "Pendulum Initial Angle %i\r\n", encoder_position_steps);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);


		if (enable_swing_up == 0){
			/*
			 * Alert user with rotor motion prompt to adjust pendulum upright by
			 */
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

			/* Request user action to bring pendulum upright */
			if(select_suspended_mode == 0){
				sprintf(msg,
						"Adjust Pendulum Upright By Turning CCW Control Will Start When Vertical\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
			}

		}

		/*
		 * Detect Start Condition for Pendulum Angle for Inverted Model
		 *
		 * Detect Pendulum Angle equal to vertical within tolerance of START_ANGLE
		 *
		 * Exit if no vertical orientation action detected and alert user to restart,
		 * then disable control and enable system restart.
		 *
		 * Permitted delay for user action is PENDULUM_ORIENTATION_START_DELAY.
		 *
		 */

		/*
		 * System start option with manual lifting of Pendulum to vertical by user
		 */

		if (enable_swing_up == 0){

			tick_wait_start = HAL_GetTick();
			if (select_suspended_mode == 0) {
				while (1){
					ret = hardware_encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
					if (fabs(encoder_position_steps - encoder_position_down - (int) (180 * angle_scale)) < START_ANGLE * angle_scale){
						HAL_Delay(START_ANGLE_DELAY);
						break;
					}
					if (fabs(encoder_position_steps - encoder_position_down + (int)(180 * angle_scale)) < START_ANGLE * angle_scale){
						encoder_position_down = encoder_position_down - 2*(int)(180 * angle_scale);
						HAL_Delay(START_ANGLE_DELAY);
						break;
					}
					tick_wait = HAL_GetTick();

					if ( (tick_wait - tick_wait_start) > PENDULUM_ORIENTATION_START_DELAY){
						sprintf(msg, "Pendulum Upright Action Not Detected - Restarting ...\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
						enable_control_action = 0;
						break;
					}
				}
			}
		}


		/*
		 * For case of Suspended Mode Operation, no initial condition check is required
		 * for pendulum down angle.
		 */

		if(select_suspended_mode == 1){
			sprintf(msg, "Suspended Mode Control Will Start in %i Seconds\r\n",
					(int) (CONTROL_START_DELAY / 1000));
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		}


		/*
		 * Initialize Primary and Secondary PID controllers
		 */

		*current_error_steps = 0;
		*current_error_rotor_steps = 0;
		/* PID state is zeroed by controller_init() at control-run start */

		/* Initialize control system variables */

		cycle_count = CYCLE_LIMIT;
		i = 0;
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
		enable_adaptive_mode = 0;
		tick_cycle_start = HAL_GetTick();
		tick_cycle_previous = tick_cycle_start;
		tick_cycle_current =  tick_cycle_start;
		enable_cycle_delay_warning = ENABLE_CYCLE_DELAY_WARNING;
		chirp_cycle = 0;
		chirp_dwell_cycle = 0;
		pendulum_position_command_steps = 0;
		impulse_start_index = 0;
		mode_transition_state = 0;
		transition_to_adaptive_mode = 0;
		error_sum_prev = 0;
		error_sum_filter_prev = 0;
		adaptive_state = 4;
		app_reset_command_shaper_state(&g_app);
		rotor_position_command_steps_pf_prev = 0;
		enable_high_speed_sampling = ENABLE_HIGH_SPEED_SAMPLING_MODE;
		slope_prev = 0;
		rotor_track_comb_command = 0;
		noise_rej_signal_prev = 0;
		noise_rej_signal_filter_prev = 0;
		full_sysid_start_index = -1;
		current_cpu_cycle = 0;
		speed_scale = DATA_REPORT_SPEED_SCALE;
		speed_governor = 0;
		encoder_position_offset = 0;
		encoder_position_offset_zero = 0;

		for (m = 0; m < ANGLE_CAL_OFFSET_STEP_COUNT + 1; m++){
			offset_angle[m] = 0;
		}

		/* Clear read buffer */
		for (k = 0; k < SERIAL_MSG_MAXLEN; k++) {
			Msg.Data[k] = 0;
		}
		/* Initialize UART receive system */
		__HAL_DMA_RESET_HANDLE_STATE(&hdma_usart2_rx);

		/*
		 * Record user selected operation variable values.  Values will be
		 * restored after Swing Up completion or after Angle Calibration
		 * completion
		 */

		init_r_p_gain = g_app.core_ctl_state.PID_Rotor.Kp;
		init_r_i_gain = g_app.core_ctl_state.PID_Rotor.Ki;
		init_r_d_gain = g_app.core_ctl_state.PID_Rotor.Kd;
		init_p_p_gain = g_app.core_ctl_state.PID_Pend.Kp;
		init_p_i_gain = g_app.core_ctl_state.PID_Pend.Ki;
		init_p_d_gain = g_app.core_ctl_state.PID_Pend.Kd;
		init_enable_state_feedback = enable_state_feedback;
		init_integral_compensator_gain = integral_compensator_gain;
		init_feedforward_gain = feedforward_gain;
		init_enable_state_feedback = enable_state_feedback;
		init_enable_disturbance_rejection_step = enable_disturbance_rejection_step;
		init_enable_sensitivity_fnc_step = enable_sensitivity_fnc_step;
		init_enable_noise_rejection_step = enable_noise_rejection_step;
		init_enable_rotor_plant_design = enable_rotor_plant_design;
		init_enable_rotor_plant_gain_design = enable_rotor_plant_gain_design;

		if(select_suspended_mode == 1){
			load_disturbance_sensitivity_scale = 1.0;
		}
		if(select_suspended_mode == 0){
			load_disturbance_sensitivity_scale = LOAD_DISTURBANCE_SENSITIVITY_SCALE;
		}


		/*
		 * Initiate Pendulum Swing Up with automatic system requiring no user action
		 *
		 * This system was developed by Markus Dauberschmidt see
		 * https://github.com/OevreFlataeker/steval_edukit_swingup
		 *
		 */


		if (enable_swing_up == 1 && select_suspended_mode == 0){

			/*
			 * Apply controller parameters for initial operation at completion of
			 * Swing Up
			 */

			g_app.core_ctl_state.PID_Rotor.Kp = 20;
			g_app.core_ctl_state.PID_Rotor.Ki = 10;
			g_app.core_ctl_state.PID_Rotor.Kd = 10;
			g_app.core_ctl_state.PID_Pend.Kp = 300;
			g_app.core_ctl_state.PID_Pend.Ki = 0.0;
			g_app.core_ctl_state.PID_Pend.Kd = 30.0;
			enable_state_feedback = 0;
			integral_compensator_gain = 0;
			feedforward_gain = 1;
			rotor_position_command_steps = 0;
			enable_state_feedback = 0;
			enable_disturbance_rejection_step = 0;
			enable_sensitivity_fnc_step = 0;
			enable_noise_rejection_step = 0;
			enable_rotor_plant_design = 0;
			enable_rotor_plant_gain_design = 0;

			/* Set Torque Current value to 800 mA (normal operation will revert to 400 mA */
			torq_current_val = MAX_TORQUE_SWING_UP;
			L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);

			sprintf(msg, "Pendulum Swing Up Starting\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

			/* Initialize position and motion variables */
			max_encoder_position = 0;
			global_max_encoder_position = 0;
			peaked = 0;
			handled_peak = 0;
			swing_up_state = 0;
			swing_up_state_prev = 0;
			zero_crossed = 0;
			stage_count = 0;
			/* Select initial amplitude for rotor impulse */
			stage_amp = STAGE_0_AMP;

			/* Optional encoder state reporting */
			//sprintf(tmp_string,"Current Position %0.2f\r\n", (encoder_position - encoder_position_down)/angle_scale);
			//HAL_UART_Transmit(&huart2, (uint8_t*) tmp_string, strlen(tmp_string), HAL_MAX_DELAY);

			//sprintf(tmp_string,"Current Position Down %0.2f\r\n", encoder_position_down/angle_scale);
			//HAL_UART_Transmit(&huart2, (uint8_t*) tmp_string, strlen(tmp_string), HAL_MAX_DELAY);

			/* Initiate first swing */
			swing_up_direction = FORWARD;
			BSP_MotorControl_Move(0, swing_up_direction, 150);
			BSP_MotorControl_WaitWhileActive(0);


			/* Enter Swing Up Loop */
			while (1)
			{
				HAL_Delay(2);
				ret = hardware_encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
				/* Optional Swing Up progress reporting */
				//sprintf(tmp_string,"Rotor Impulse Amplitude %i Max Angle (degrees) %0.3f\r\n", stage_amp, fabs((float)(global_max_encoder_position)/(ENCODER_READ_ANGLE_SCALE)));
				//HAL_UART_Transmit(&huart2, (uint8_t*) tmp_string, strlen(tmp_string), HAL_MAX_DELAY);

				/* Break if pendulum angle relative to vertical meets tolerance (for clockwise or counter clockwise approach */
				if (fabs(encoder_position_steps - encoder_position_down - (int) (180 * angle_scale)) < START_ANGLE * angle_scale){
					break;
				}
				if (fabs(encoder_position_steps - encoder_position_down + (int)(180 * angle_scale)) < START_ANGLE * angle_scale){
					encoder_position_down = encoder_position_down - 2*(int)(180 * angle_scale);
					break;
				}

				if (zero_crossed)
				{
					zero_crossed = 0;
					// Push it aka put some more kinetic energy into the pendulum
					if (swing_up_state == 0){
						BSP_MotorControl_Move(0, swing_up_direction, stage_amp);
						BSP_MotorControl_WaitWhileActive(0);
						stage_count++;

						if (prev_global_max_encoder_position != global_max_encoder_position && stage_count > 4){
						if (abs(global_max_encoder_position) < 600){
							stage_amp = STAGE_0_AMP;
						}
						if (abs(global_max_encoder_position) >= 600 && abs(global_max_encoder_position) < 1000){
							stage_amp = STAGE_1_AMP;
						}
						if (abs(global_max_encoder_position) >= 1000){
							stage_amp = STAGE_2_AMP;
						}
						}
						prev_global_max_encoder_position = global_max_encoder_position;
						global_max_encoder_position = 0;
						ret = hardware_encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
					}
				}


				// We have a peak but did not handle it yet
				if (peaked && !handled_peak)
				{
					// Ensure we only enter this branch one per peak
					handled_peak = 1;
					// Reset maximum encoder value to reassess after crossing the bottom
					max_encoder_position = 0;
					// Switch motor direction
					swing_up_direction = swing_up_direction == FORWARD ? BACKWARD : FORWARD;
				}
			}
		}

		/*
		 * *************************************************************************************************
		 *
		 * Control Loop Start
		 *
		 * *************************************************************************************************
		 */

		enable_control_action = 1;

		if (ACCEL_CONTROL == 1) {
			BSP_MotorControl_HardStop(0);
			L6474_CmdEnable(0);
			target_velocity_prescaled = 0;
			L6474_Board_SetDirectionGpio(0, BACKWARD);
		}

		/*
		 * Set Torque Current to value for normal operation
		 */
		torq_current_val = MAX_TORQUE_CONFIG;
		L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);

		target_cpu_cycle = DWT->CYCCNT;
		prev_cpu_cycle = DWT->CYCCNT;

		ret = hardware_encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
		if (select_suspended_mode == 0) {
			encoder_position = encoder_position_steps - encoder_position_down - (int)(180 * angle_scale);
			encoder_position = encoder_position - encoder_position_offset;
		}

		/* Step 5: initialize hardware layer and observer for this control run */
		app_init_control_pipeline(&g_app, encoder_position_init, Tsample);

		while (enable_control_action == 1) {


			ret = control_handle_runtime_configuration(&g_app, i);
			if (ret < 0) {
				break;
			}
			if (ret > 0) {
				continue;
			}

			/*
			 * *************************************************************************************************
			 *
			 * Initiate Measurement and Control
			 *
			 * *************************************************************************************************
			 */

			/*
			 * Optional Reset and clear integrator error during initial start of controllers
			 */
			if (i < 1){
				g_app.core_ctl_state.PID_Pend.int_term = 0;
				g_app.core_ctl_state.PID_Rotor.int_term = 0;
			}

			/*
			 * Acquire encoder position and correct for initial angle value of encoder measured at
			 * vertical down position at system start including 180 degree offset corresponding to
			 * vertical upwards orientation.
			 *
			 * For case of Suspended Mode Operation the 180 degree offset is not applied
			 *
			 * The encoder_position_offset variable value is determined by the Automatic Inclination
			 * Angle Calibration system
			 */

			ret = control_execute_cycle(&g_app, i);
			if (ret != 0) {
				break;
			}

			/* Increment cycle counter */

			i++;


		}

		control_shutdown_sequence(&g_app);

	}
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






