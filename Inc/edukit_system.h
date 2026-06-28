




#ifndef EDUKIT_SYSTEM_H
#define EDUKIT_SYSTEM_H

/*
 ******************************************************************************
 * @file    Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/main.c
 *
 *    Acknowledgments to the invaluable development, support and guidance by
 *    Marco De Fazio, Giorgio Mariano, Enrico Poli, and Davide Ghezzi
 *    of STMicroelectronics
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
 * @author  William J. Kaiser (UCLA Electrical and Computer Engineering).
 *
 * Application based on development by STMicroelectronics as described below
 *
 * @version V1.0
 * @date    May 15th, 2019
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

#include <stdbool.h>

/*
 * Control System and Motor Configuration Parameter Definitions
 */

/*
 * Sample rates defined for controller operation
 *
 * Note that these values scale derivative and integral computations
 *
 */

#define RCC_SYS_CLOCK_FREQ 84000000 // should equal HAL_RCC_GetSysClockFreq()
#define RCC_HCLK_FREQ 84000000 // should equal HAL_RCC_GetHCLKFreq()


#define T_SAMPLE_DEFAULT 0.002

#define CONTROLLER_GAIN_SCALE 						1
#define STEPPER_READ_POSITION_STEPS_PER_DEGREE 		8.888889	//	Stepper position read value in steps per degree
#define STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE 	STEPPER_READ_POSITION_STEPS_PER_DEGREE
#define ENCODER_READ_ANGLE_SCALE 					6.666667 // Angle Scale 6.66667 for 600 Pulse Per Rev Resolution Optical Encoder
#define FULL_STATE_FEEDBACK_SCALE 					1.00 // Scale factor for Full State Feedback Architecture

#define ENABLE_CYCLE_DELAY_WARNING 1			// Enable warning and control loop exit if loop delay exceeds threshold

#define ENCODER_ANGLE_POLARITY -1.0				// Note that physical system applies negative polarity to pendulum angle
												// by definition of coordinate system.

#define CYCLE_LIMIT 100000 						// Cycle limit determines run time as product of cycle limit and cycle time (typical 5 msec).
#define ENABLE_CYCLE_INFINITE 1 				// ENABLE_CYCLE_INFINITE set to 1 if continuous operation is to be enabled


#define MAX_TORQUE_CONFIG 800 					// 400 Selected Value for normal control operation
#define MAX_TORQUE_SWING_UP 800					// 800 Selected Value for Swing Up operation

#define OVERCURRENT_THRESHOLD 2000				// 2000 Selected Value for Integrated Rotary Inverted Pendulum System
#define SHUTDOWN_TORQUE_CURRENT 0				// 0 Selected Value for Integrated Rotary Inverted Pendulum System
#define TORQ_CURRENT_DEFAULT MAX_TORQUE_CONFIG				// Default torque current	// Default torque current

/*
 * Note that Speed Profiles are set at run time during execution.
 *
 * Default speed Profiles are set in l6474_target_config.h
 */

#define MAX_SPEED_UPPER_INIT 10000						// Initialization value
#define MIN_SPEED_UPPER_INIT 10000						// Initialization value
#define MAX_SPEED_LOWER_INIT 30							// Initialization value
#define MIN_SPEED_LOWER_INIT 30							// Initialization value
#define MAX_ACCEL_UPPER_INIT 10000			 			// Initialization value
#define MAX_DECEL_UPPER_INIT 10000	 					// Initialization value

/*
 * Configuration of Motor Speed Profile at initialization
 */

#define MAX_SPEED 2000
#define MIN_SPEED 800
#define MAX_ACCEL 6000
#define MAX_DECEL 6000

#define MAX_SPEED_MODE_2 2000
#define MIN_SPEED_MODE_2 1000
#define MAX_SPEED_MODE_1 2000
#define MIN_SPEED_MODE_1 800
#define MAX_SPEED_MODE_3 2000
#define MIN_SPEED_MODE_3 600
#define MAX_SPEED_MODE_4 2000
#define MIN_SPEED_MODE_4 400
#define MAX_SPEED_MODE_5 2000
#define MIN_SPEED_MODE_5 800

#define ENABLE_SUSPENDED_PENDULUM_CONTROL 0     // Set to 0 for Inverted Pendulum Mode - Set to 1 for Suspended Pendulum Mode

#define ENCODER_START_OFFSET 0 				// Encoder configurations may display up to 1 degree initial offset
#define ENCODER_START_OFFSET_DELAY 0			// Encoder offset delay limits application of initial offset
#define START_ANGLE 1							// Pendulum angle tolerance for system pendulum orientation at start
#define START_ANGLE_DELAY 0						// Delay at start for orientation of pendulum upright

/*
 * Pendulum Swing Up Configuration
 */

/*
 * Iinital Measurement of Edukit Platform angle relative to vertical.
 *
 * The Edukit system may be resting on a sloped surface.  Therefore, accurate measurement of Pendulum upright angle
 * includes a slope error.  This is corrected for by the Angle Calibration System that operates at start time
 *
 */

#define ENABLE_ANGLE_CAL 1
#define ANGLE_CAL_OFFSET_STEP_COUNT 1801	// Full span angle range from negative to positive rotor angle
#define ANGLE_AVG_SPAN 50 					// Angle element smoothing span in rotor steps reduces full span by twice set value
#define ANGLE_CAL_ZERO_OFFSET_DWELL 5000	// 10 second zero offset measurement period
#define ANGLE_CAL_ZERO_OFFSET_SETTLING 2000 // 4 second settling period after offset correction
#define ANGLE_CAL_COMPLETION 2000			// 4 seconds settling period after final offset update prior to assigning new controller


/*
 * Single PID, Dual PID and LQR Controllers are implemented as summation of Primary
 * and Secondary PID controller structures.  These two controller outputs are summed
 * and supplied to Rotor Control
 *
 * PID Controller parameters.  LQR Controllers are implemented with integral gain of 0.
 *
 */

#define ENABLE_PID_INTEGRATOR_LIMIT 0			// Enable PID filter integrator limit
#define PRIMARY_WINDUP_LIMIT 1000				// Integrator wind up limits for PID Pendulum controller
#define SECONDARY_WINDUP_LIMIT 1000				// Integrator wind up limits for PID Rotor controller

/*
 * High Speed Mode Values: 5, 25, 30
 */
#define DERIVATIVE_LOW_PASS_CORNER_FREQUENCY 10  		// 10 - Corner frequency of low pass filter of Primary PID derivative
#define LP_CORNER_FREQ_ROTOR 100 						// 100 - Corner frequency of low pass filter of Rotor Angle
#define DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR 50 	// 50 - Corner frequency of low pass filter of Secondary PID derivative
#define LP_CORNER_FREQ_STEP 50							// Low pass filter operating on rotor reference step command signal


/* Mode 1 is Dual PID Demonstration Mode */

#define PRIMARY_PROPORTIONAL_MODE_1 	300
#define PRIMARY_INTEGRAL_MODE_1     	0.0
#define PRIMARY_DERIVATIVE_MODE_1   	30

#define SECONDARY_PROPORTIONAL_MODE_1 	15.0
#define SECONDARY_INTEGRAL_MODE_1     	0.0
#define SECONDARY_DERIVATIVE_MODE_1   	7.5

#define PRIMARY_PROPORTIONAL_MODE_2 	518.0
#define PRIMARY_INTEGRAL_MODE_2     	0
#define PRIMARY_DERIVATIVE_MODE_2   	57.0

#define SECONDARY_PROPORTIONAL_MODE_2 	2.20
#define SECONDARY_INTEGRAL_MODE_2     	0.0
#define SECONDARY_DERIVATIVE_MODE_2   	4.82

#define PRIMARY_PROPORTIONAL_MODE_3 	300
#define PRIMARY_INTEGRAL_MODE_3     	0.0
#define PRIMARY_DERIVATIVE_MODE_3   	30.0

#define SECONDARY_PROPORTIONAL_MODE_3 	15.0
#define SECONDARY_INTEGRAL_MODE_3     	0.0
#define SECONDARY_DERIVATIVE_MODE_3   	15.0

#define PRIMARY_PROPORTIONAL_MODE_5 	300
#define PRIMARY_INTEGRAL_MODE_5     	0.0
#define PRIMARY_DERIVATIVE_MODE_5   	30.0

#define SECONDARY_PROPORTIONAL_MODE_5 	15.0
#define SECONDARY_INTEGRAL_MODE_5     	0.0
#define SECONDARY_DERIVATIVE_MODE_5   	15.0

/*
 * Single PID Mode Gains
 */

#define ROTOR_PID_PROPORTIONAL_GAIN_SINGLE_PID_MODE  15.0
#define ROTOR_PID_INTEGRAL_GAIN_SINGLE_PID_MODE		 0.0
#define ROTOR_PID_DIFFERENTIAL_GAIN_SINGLE_PID_MODE	 7.5

/*
 * Mode 4 is Dual PID Suspended Demonstration Mode
 */

#define PRIMARY_PROPORTIONAL_MODE_4 	-10.0
#define PRIMARY_INTEGRAL_MODE_4     	 0.0
#define PRIMARY_DERIVATIVE_MODE_4   	-5.0

#define SECONDARY_PROPORTIONAL_MODE_4 	-2.0
#define SECONDARY_INTEGRAL_MODE_4     	 0.0
#define SECONDARY_DERIVATIVE_MODE_4   	-2.0


#define DEFAULT_START_MODE	mode_1

#define ENABLE_ADAPTIVE_MODE 0
#define ADAPTIVE_THRESHOLD_LOW 30				// Default value 30
#define ADAPTIVE_THRESHOLD_HIGH 2				// Default value 2
#define ADAPTIVE_STATE 0
#define ADAPTIVE_DWELL_PERIOD 2000				// Determines dwell period during state transition

#define USER_TRANSITION_DWELL 500

/*
 * START_DEFAULT_MODE_TIME determines time delay for waiting for user input after start or reset.
 * For a time (in ticks) greater than this period, control will initiate with default mode 1 if
 * no user input appears.
 *
 * This permits system operation in default mode independent of external command from separate
 * host.
 */

#define START_DEFAULT_MODE_TIME 60000			// delay in ms to permit user input after system start
												// If no user response then set default values
#define PENDULUM_ORIENTATION_START_DELAY 10000	// Time permitted to user to orient Pendulum vertical at start

#define INITIAL_START_DELAY 1000				// Determines time available for ensuring pendulum down and prior to user prompt
#define CONTROL_START_DELAY 1000 				// Determines time available to user after prompt for adjusting pendulum upright
#define INITIAL_PENDULUM_MOTION_TEST_DELAY 2000 // Determines delay time between successive evaluations of pendulum motion

#define ENABLE_CONTROL_ACTION 1							// Enable control operation - default value of 1 (may be disabled for test operations)
#define ENABLE_DUAL_PID 1						// Note ENABLE_DUAL_PID is set to 1 by default for summation of PID controllers
												// for either Dual PID or LQR systems

#define STATE_FEEDBACK_CONFIG_ENABLE 1			// Default selection of Dual PID Architecture - Set to 1 for State Feedback			1

/*
 * Swing Up System Parameters : Swing Up Algorithm developed and provided by Markus Dauberschmidt
 * Please see
 */
#define ENABLE_SWING_UP 1						// Enable Pendulum Swing Up system
#define SWING_UP_CONTROL_CONFIG_DELAY 3000 		// Delay in cycles prior to switching from Swing Up controller to user selected controller
#define STAGE_0_AMP 200							// Swing Up Impulse amplitude for initial state
#define STAGE_1_AMP 130							// Swing Up Impulse amplitude for intermediate state
#define STAGE_2_AMP 120							// Swing Up Impulse amplitude for final state



/*
 * ENCODER ANGLE SLOPE CORRECTION compensates for any error introduced by a platform tilt relative to vertical.
 * The correction is computed over a time constant of greater than 100 seconds to avoid any distortion in measurements
 * that are conducted over shorter intervals of up to 10 seconds.
 */
#define ENABLE_ENCODER_ANGLE_SLOPE_CORRECTION 	0		// Default 1 for system operation enable
#define OFFSET_FILTER_GAIN 						1		// Offset compensation gain value
#define LP_CORNER_FREQ_LONG_TERM 				0.01	// Corner frequency of low pass filter - default to 0.001
#define ENCODER_ANGLE_SLOPE_CORRECTION_SCALE 	200		// Set to 200
#define ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT	0	// Sets limit on operation time for slope angle correction
														// If set to zero, slope correction operates at all times
														// Default set to zero
/*
 * Rotor position limits are defined to limit rotor rotation to one full rotation in clockwise or
 * counterclockwise motion.
 */

#define ROTOR_POSITION_POSITIVE_LIMIT 240		// Maximum allowed rotation in positive angle in degrees
#define ROTOR_POSITION_NEGATIVE_LIMIT -240		// Minimum allowed rotation in negative angle in degrees

/*
 * Encoder position limits are defined to detect excursions in pendulum angle corresponding to
 * departure from control and to initiate control loop exit
 */

#define ENCODER_POSITION_POSITIVE_LIMIT  120		// Maximum allowed rotation in positive angle in steps
#define ENCODER_POSITION_NEGATIVE_LIMIT -120		// Minimum allowed rotation in negative angle in steps

#define ENABLE_TORQUE_CURRENT_ENTRY		0		    // Enables user input of torque current configuration in general mode
#define ENABLE_DISTURBANCE_REJECTION_STEP 	1
#define LOAD_DISTURBANCE_SENSITIVITY_SCALE 	20			// Scale factor applied to increase measurement resolution for Load Disturbance Sensitivity Function

/*
 * Set ENABLE_ENCODER_TEST to 1 to enable a testing of encoder response for verification
 * occurring prior to control system start.
 *
 * This will be set to 0 and disabled for normal operation
 *
 */

#define ENABLE_ENCODER_TEST 0

/*
 * Set ENABLE_ROTOR_ACTUATOR_TEST to 1 to enable a testing of encoder response for verification
 * occurring prior to control system start.
 *
 * This will be set to 0 and disabled for normal operation
 *
 */

#define ENABLE_ROTOR_ACTUATOR_TEST 0
#define ROTOR_ACTUATOR_TEST_CYCLES 1

/*
 * UART DMA definitions
 */

#define UART_RX_BUFFER_SIZE 	(200)
#define SERIAL_MSG_MAXLEN 		(100)
#define SERIAL_MSG_EOF          '\r'

/*
 * Structure for the augmented floating-point PID Control.
 * This includes additional state associated with derivative filter
 *
 * This follows the ARM CMSIS architecture
 */

typedef struct
{
  float state_a[4];  /** The filter state array of length 4. */
  float Kp;          /** The proportional gain. */
  float Ki;          /** The integral gain. */
  float Kd;          /** The derivative gain. */
  float int_term;    /** The controller integral output */
  float control_output; /** The controller output */
} arm_pid_instance_a_f32;

#define delayUS_ASM(us) do {\
		asm volatile (	"MOV R0,%[loops]\n\t"\
				"1: \n\t"\
				"SUB R0, #1\n\t"\
				"CMP R0, #0\n\t"\
				"BNE 1b \n\t" : : [loops] "r" (16*us) : "memory"\
		);\
} while(0)

/* DWT_Delay_until_cycle defined in main.c — not declared here to avoid -Wunused warnings */

/* pid_filter_control_execute → private pid_execute in controller.c */


/* UI functions → declared in ui.h */
/* Hardware functions → declared in hardware.h */

/* oppositeSigns → static in hardware.c, not accessible externally */

extern volatile uint16_t gLastError;
/* Private function prototypes -----------------------------------------------*/
extern void MyFlagInterruptHandler(void);
extern void MX_TIM3_Init(void);
extern void MX_USART2_UART_Init(void);
extern void Error_Handler(uint16_t error);
extern void select_mode_1(void);
extern int Delay_Pulse();
extern void Main_StepClockHandler();
extern void apply_acceleration(float * acc, float* target_velocity_prescaled, float t_sample);


/*
 * Timer 3, UART Transmit, and UART DMA Receive declarations
 */

extern TIM_HandleTypeDef htim3;

/*
  * Timer 3, UART Transmit, and UART DMA Receive declarations
  */

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

/*
 * UART Receive data structure
 */

typedef struct {
	uint32_t Len; /*!< Message length           */
	uint8_t Data[SERIAL_MSG_MAXLEN]; /*!< Message data             */
} T_Serial_Msg;

extern T_Serial_Msg Msg;

extern uint8_t RxBuffer[UART_RX_BUFFER_SIZE];
extern uint16_t Extract_Msg(uint8_t *CircularBuff, uint16_t StartPos, uint16_t LastPos,
		uint16_t BufMaxLen, T_Serial_Msg *Msg);

/* Acceleration control system variables */
extern volatile uint32_t apply_acc_start_time;
extern volatile uint32_t clock_int_time;
extern volatile uint32_t clock_int_tick;

/// PWM period variables used by step interrupt
extern volatile uint32_t desired_pwm_period;
extern volatile uint32_t current_pwm_period;

extern float target_velocity_prescaled;
/* enable_speed_prescale → write-only (never read), removed */

/* System data reporting */
extern char uart_tx_buf[192];

/* System timing variables */

/* tick, tick_cycle_current, tick_cycle_previous, tick_cycle_start,
   tick_read_cycle, tick_read_cycle_start → ctx->timing fields */
/* t_sample_s, t_sample_rotor_s → ctx->timing fields */
/* test_time → write-only, removed */
/* angle_scale, enable_high_speed_sampling, reset_state → ctx fields */

/* Motor configuration */
/* min_speed, max_speed, max_accel, max_decel → ctx fields */

/* Control system output, rotor position/filter/diff/command/impulse → ctx->rotor_pos (RotorPositionState) */
/* Encoder position, angle calibration, slope correction → ctx->enc_cal (EncoderCalibState) */

/* User configuration variables */
/* clear_input → unreferenced, removed */
/* enable_control_action, select_suspended_mode → ctx fields */
/* max_speed_read, min_speed_read → unreferenced, removed */
/* motor_response_model → unreferenced, removed */
/* enable_rotor_actuator_test, enable_rotor_actuator_control → ctx fields */
/* enable_encoder_test → ui.c static */
/* enable_rotor_actuator_high_speed_test → write-only, removed */
/* enable_motor_actuator_characterization_mode, torq_current_val → ctx fields */
/* motor_state → ui.c static */


/* Rotor chirp, sine tracking, sysid, comb drive → ctx->tracking (RotorTrackingState) */
/* rotor_chirp_amplitude, rotor_chirp_step_period → ui.c static */

/* Rotor high speed test system variables */
/* swing_cycles, start/end_angle_a, motion_dwell_a,
   abs_encoder_position_prior/after/max → removed (never referenced) */
/* rotor_test_speed_min/max, rotor_test_acceleration_max, swing_deceleration_max,
   current_speed → ui.c static */

/*Pendulum system ID variable */
/* enable_pendulum_sysid_test → ui.c static */

/* Full system identification variables */
/* enable_full_sysid, full_sysid_max_freq_hz → ui.c static */

/* Sensitivity function system variables → ctx->gains (PidGainSet) */
/* enable_disturbance_rejection_step, enable_noise_rejection_step → ctx->gains */
/* enable_plant_rejection_step → unreferenced, removed */
/* enable_sensitivity_fnc_step, load_disturbance_sensitivity_scale → ctx->gains */



/* Noise rejection sensitivity function low pass filter */

/* noise_rej_signal → local variable in control_execute_cycle */
/* adjust_increment, report_mode, speed_scale, speed_governor → ctx fields */
/* mode_transition_state → ctx field */
/* mode_1..mode_19, mode_quit, mode_adaptive*, mode_string_* → ui.c (static) */
/* mode_index_prev → write-only, removed; mode_index_command → ui.c static */
/* mode_transition_tick → never used, removed */


/* message_received → never referenced, removed */
/* mode_string_mode_high_speed_test, mode_string_mode_pendulum_sysid_test → removed (unused) */

/* CMSIS Variables: PID_Pend, PID_Rotor, Deriv_Filt_Pend, Deriv_Filt_Rotor, Wo_t, fo_t, IWon_t
   are ControllerState/ObserverState struct fields; no global definitions existed.
   Dangling extern declarations removed. Duplicate system timing section removed. */

#endif /* EDUKIT_SYSTEM_H */
