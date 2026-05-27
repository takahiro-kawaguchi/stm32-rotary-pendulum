#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include "hardware.h"
#include "observer.h"
#include "controller.h"
#include "command_shaper.h"

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
	int enable_rotor_plant_design;
	int enable_rotor_plant_gain_design;
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
	uint32_t tick_read_cycle;
	uint32_t tick_read_cycle_start;
	float Tsample;
	float Tsample_rotor;
} LoopTimingState;

typedef struct {
	int select_rotor_plant_design;
	int enable_rotor_plant_design;
	int enable_rotor_plant_gain_design;
	float rotor_plant_gain;
	float rotor_damping_coefficient;
	float rotor_natural_frequency;
	float ao, Wn2;
	float c0, c1, c2, c3, c4;
	float fo_r, Wo_r, IWon_r;
	float iir_0_r, iir_1_r, iir_2_r;
} RotorPlantState;

typedef struct {
	float iir_0, iir_1, iir_2;           /* rotor position LP filter */
	float iir_LT_0, iir_LT_1, iir_LT_2; /* long-term LP filter */
	float iir_0_s, iir_1_s, iir_2_s;    /* step-response LP filter */
} RotorFilterState;

typedef struct {
	int   enable_rotor_chirp;
	int   chirp_cycle;
	int   chirp_dwell_cycle;
	float chirp_time;
	float rotor_chirp_start_freq;
	float rotor_chirp_end_freq;
	float rotor_chirp_period;
	float rotor_chirp_frequency;
	float pendulum_position_command_steps;
	int   enable_mod_sin_rotor_tracking;
	int   enable_rotor_position_step_response_cycle;
	int   disable_mod_sin_rotor_tracking;
	int   sine_drive_transition;
	float mod_sin_amplitude;
	float rotor_control_sin_amplitude;
	float rotor_sine_drive;
	float rotor_sine_drive_mod;
	float rotor_mod_control;
	float mod_sin_carrier_frequency;
	int   enable_pendulum_position_impulse_response_cycle;
	float full_sysid_max_vel_amplitude_deg_per_s;
	float full_sysid_min_freq_hz;
	int   full_sysid_num_freqs;
	float full_sysid_freq_log_step;
	int   full_sysid_start_index;
	int   enable_rotor_tracking_comb_signal;
	float rotor_track_comb_signal_frequency;
	float rotor_track_comb_command;
	float rotor_track_comb_amplitude;
} RotorTrackingState;

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
	RotorPlantState plant;
	RotorTrackingState tracking;
	PidGainSet gains;
	ControllerDualPidRuntime core_dual_pid_runtime;
	float angle_scale;
	float adjust_increment;
	int   enable_high_speed_sampling;
	int   reset_state;
	int   report_mode;
	int   speed_scale;
	int   speed_governor;
	int   mode_transition_state;
	int   select_suspended_mode;
	int   enable_swing_up;
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
