#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include "hardware.h"
#include "observer.h"
#include "controller.h"
#include "command_shaper.h"
#include "edukit_system.h"

/* Diagnostic (Mode D, see Src/ui.c): 500Hz / 5 = 100Hz, matching the rate
 * report_telemetry() (Src/app_runtime.c) already decimates to and thus what
 * a PC-side remote controller actually sees/reacts to each update. */
#define CONTROL_DECIMATION_FACTOR 5

typedef struct {
	float proportional;
	float integral;
	float derivative;
	float rotor_p_gain;
	float rotor_i_gain;
	float rotor_d_gain;
	int   enable_state_feedback;
	float integral_compensator_gain;
	float feedforward_gain;
	int   enable_disturbance_rejection_step;
	int   enable_noise_rejection_step;
	int   enable_sensitivity_fnc_step;
	float load_disturbance_sensitivity_scale;
} PidGainSet;

/*
 * SessionInitialParams: snapshot of gain/mode state at the start of each
 * control session. Saved by app_session.c, restored by app_control.c and
 * app_runtime.c when the user resets gains during a live session.
 */
typedef struct {
	float Kp_rotor, Ki_rotor, Kd_rotor;
	float Kp_pend,  Ki_pend,  Kd_pend;
	float integral_compensator_gain;
	float feedforward_gain;
	int enable_state_feedback;
	int enable_disturbance_rejection_step;
	int enable_sensitivity_fnc_step;
	int enable_noise_rejection_step;
} SessionInitialParams;

typedef struct {
	volatile uint32_t current_cpu_cycle;
	volatile uint32_t prev_cpu_cycle;
	volatile uint32_t last_cpu_cycle;
	volatile uint32_t target_cpu_cycle;
	volatile uint32_t prev_target_cpu_cycle;
	volatile int current_cpu_cycle_delay_relative_report;
	uint32_t t_sample_cpu_cycles;
	int cycle_count;
	int cycle_period_start;
	int cycle_period_sum;
	int enable_cycle_delay_warning;
	uint32_t tick;
	uint32_t tick_cycle_current;
	uint32_t tick_cycle_previous;
	uint32_t tick_cycle_start;
	float t_sample_s;
	float t_sample_rotor_s;
} LoopTimingState;

typedef struct {
	float iir_0, iir_1, iir_2;           /* rotor position LP filter */
	float iir_LT_0, iir_LT_1, iir_LT_2; /* long-term LP filter */
	float iir_0_s, iir_1_s, iir_2_s;    /* step-response LP filter */
} RotorFilterState;

typedef struct {
	float rotor_control_target_steps;
	int   rotor_position_steps;
	float rotor_position_command_steps;
	float rotor_position_steps_prev;
	float rotor_position_filter_steps;
	float rotor_position_filter_steps_prev;
	float rotor_position_diff;
	float rotor_position_diff_prev;
	float rotor_position_diff_filter;
	float rotor_position_diff_filter_prev;
} RotorPositionState;

typedef struct {
	float encoder_position;
	int   encoder_position_steps;
	int   encoder_position_init;
	int   encoder_position_down;
	float encoder_position_offset;
	float encoder_position_offset_zero;
	int   enable_angle_cal;
	int   offset_end_state;
	int   offset_start_index;
	int   angle_index;
	int   angle_avg_index;
	int   angle_avg_span;
	int   offset_angle[ANGLE_CAL_OFFSET_STEP_COUNT + 2];
	float encoder_position_offset_avg[ANGLE_CAL_OFFSET_STEP_COUNT + 2];
	int   angle_cal_end;
	int   angle_cal_complete;
	float encoder_angle_slope_corr_steps;
} EncoderCalibState;

typedef struct AppControlContext {
	SensorRaw core_hw_raw;
	SensorCalib core_hw_cal;
	ObserverState core_obs_state;
	SystemState core_sys_state;
	ControllerState core_ctl_state;
	ControlTarget core_ctl_target;
	ControlOutput core_ctl_out;
	CommandShaperState core_cmd_shaper_state;
	SessionInitialParams init_params;
	LoopTimingState timing;
	RotorFilterState lpf;
	RotorPositionState rotor_pos;
	EncoderCalibState enc_cal;
	PidGainSet gains;
	ControllerDualPidRuntime core_dual_pid_runtime;
	float angle_scale;
	float adjust_increment;
	int   reset_state;
	int   mode_transition_state;
	int   select_suspended_mode;
	int   enable_swing_up;
	int   enable_remote_swing_up;
	int   enable_decimated_control;
	uint32_t enable_control_action;
	int   enable_adaptive_mode;
	int   adaptive_state;
	int   adaptive_state_change;
	int   enable_rotor_actuator_test;
	int   enable_rotor_actuator_control;
	int   enable_motor_actuator_characterization_mode;
	float torq_current_val;
	uint16_t min_speed;
	uint16_t max_speed;
	uint16_t max_accel;
	uint16_t max_decel;
	const ObserverOps *core_observer_ops;
	const ControllerOps *core_controller_ops;
	const CommandShaperOps *core_command_shaper_ops;
} AppControlContext;

void app_reset_command_shaper_state(AppControlContext *ctx);
void app_assign_pid_gains_from_user(AppControlContext *ctx);
void app_init_control_pipeline(AppControlContext *ctx, int encoder_init_counts,
		float sample_period_s);

void control_shutdown_sequence(AppControlContext *ctx);
int control_handle_runtime_configuration(AppControlContext *ctx, int i);
int control_update_state_and_safety(AppControlContext *ctx);
void control_update_slope_correction(AppControlContext *ctx, int i);
void control_update_dual_pid(AppControlContext *ctx);
void control_finalize_command_and_actuate(AppControlContext *ctx, int i);
int control_wait_next_cycle(AppControlContext *ctx);

#endif
