#include "main.h"
#include "edukit_system.h"
#include "hardware.h"
#include "command_shaper.h"
#include <math.h>

void command_shaper_process_and_actuate(const CommandShaperConfig *config,
                                        int cycle_index,
                                        float rotor_position_command_steps,
                                        float *rotor_control_target_steps,
                                        CommandShaperState *state,
                                        ControlOutput *out)
{
    if (config->full_sysid_start_index != -1
            && cycle_index >= config->full_sysid_start_index
            && cycle_index > config->angle_cal_complete) {
        float total_acc = 0.0f;
        float t = (cycle_index - config->full_sysid_start_index) * config->sample_period_s;
        float w = config->full_sysid_min_freq_hz * M_TWOPI;

        for (int k_step = 0; k_step < config->full_sysid_num_freqs; k_step++) {
            float wave_value = w * cosf(w * t);
            total_acc += wave_value;
            w *= config->full_sysid_freq_log_step;
        }

        *rotor_control_target_steps =
                ((config->full_sysid_max_vel_amplitude_deg_per_s / config->full_sysid_num_freqs)
                        * total_acc * STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE);
    }

    if (config->rotor_damping_coefficient != 0 || config->rotor_natural_frequency != 0) {
        state->rotor_control_target_steps_filter_2 = config->c0 * (*rotor_control_target_steps)
                + config->c1 * state->rotor_control_target_steps_prev
                + config->c2 * state->rotor_control_target_steps_prev_prev
                + config->c3 * state->rotor_control_target_steps_filter_prev_2
                + config->c4 * state->rotor_control_target_steps_filter_prev_prev_2;

        state->rotor_control_target_steps_prev_prev = state->rotor_control_target_steps_prev;
        state->rotor_control_target_steps_filter_prev_prev_2 =
                state->rotor_control_target_steps_filter_prev_2;
        state->rotor_control_target_steps_filter_prev_2 =
                state->rotor_control_target_steps_filter_2;
    }

    if (config->enable_rotor_plant_design == 2) {
        state->rotor_control_target_steps_filter_2 =
                config->iir_0_r * (*rotor_control_target_steps)
                        + config->iir_1_r * state->rotor_control_target_steps_prev
                        - config->iir_2_r * state->rotor_control_target_steps_filter_prev_2;
        state->rotor_control_target_steps_filter_prev_2 =
                state->rotor_control_target_steps_filter_2;
    }

    state->rotor_control_target_steps_prev = *rotor_control_target_steps;
    state->rotor_position_command_steps_prev = rotor_position_command_steps;

    if (config->accel_control == 1) {
        if (config->enable_rotor_plant_design != 0) {
            state->rotor_control_target_steps_filter_2 =
                    config->rotor_plant_gain * state->rotor_control_target_steps_filter_2;
            out->rotor_accel_steps_s2 = state->rotor_control_target_steps_filter_2;
        } else if (config->enable_rotor_plant_gain_design == 1) {
            state->rotor_control_target_steps_gain =
                    config->rotor_plant_gain * (*rotor_control_target_steps);
            out->rotor_accel_steps_s2 = state->rotor_control_target_steps_gain;
        } else {
            out->rotor_accel_steps_s2 = *rotor_control_target_steps;
        }
        hardware_motor_write(out, config->sample_period_s);
    } else {
        BSP_MotorControl_GoTo(0, (*rotor_control_target_steps) / 2);
    }
}

const CommandShaperOps COMMAND_SHAPER_OPS_DEFAULT = {
    .process_and_actuate = command_shaper_process_and_actuate,
};
