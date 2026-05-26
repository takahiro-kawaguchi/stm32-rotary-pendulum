#ifndef COMMAND_SHAPER_H
#define COMMAND_SHAPER_H

#include "hardware.h"

typedef struct {
    float sample_period_s;
    int accel_control;

    int angle_cal_complete;
    int full_sysid_start_index;
    float full_sysid_max_vel_amplitude_deg_per_s;
    float full_sysid_min_freq_hz;
    int full_sysid_num_freqs;
    float full_sysid_freq_log_step;

    int enable_rotor_plant_design;
    int enable_rotor_plant_gain_design;
    float rotor_plant_gain;
    float rotor_damping_coefficient;
    float rotor_natural_frequency;

    float c0, c1, c2, c3, c4;
    float iir_0_r, iir_1_r, iir_2_r;
} CommandShaperConfig;

typedef struct {
    float rotor_control_target_steps_prev;
    float rotor_control_target_steps_prev_prev;
    float rotor_control_target_steps_filter_2;
    float rotor_control_target_steps_filter_prev_2;
    float rotor_control_target_steps_filter_prev_prev_2;
    float rotor_control_target_steps_gain;
    float rotor_position_command_steps_prev;
} CommandShaperState;

void command_shaper_process_and_actuate(const CommandShaperConfig *config,
                                        int cycle_index,
                                        float rotor_position_command_steps,
                                        float *rotor_control_target_steps,
                                        CommandShaperState *state,
                                        ControlOutput *out);

typedef struct {
    void (*process_and_actuate)(const CommandShaperConfig *config,
                                int cycle_index,
                                float rotor_position_command_steps,
                                float *rotor_control_target_steps,
                                CommandShaperState *state,
                                ControlOutput *out);
} CommandShaperOps;

extern const CommandShaperOps COMMAND_SHAPER_OPS_DEFAULT;

#endif /* COMMAND_SHAPER_H */
