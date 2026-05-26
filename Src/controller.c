#include "main.h"
#include "edukit_system.h"
#include "hardware.h"
#include "observer.h"
#include "controller.h"
#include <string.h>

/*
 * Private PID with first-order low-pass filter on the derivative term.
 * Renamed pid_execute to avoid linkage conflict with the non-static copy
 * in main.c (which is declared extern in edukit_system.h until Step 5).
 */
static void pid_execute(arm_pid_instance_a_f32 *PID, float *current_error,
                        float *sample_period, float *Deriv_Filt)
{
    float int_term, diff, diff_filt;

    int_term = PID->Ki * (*sample_period) * ((*current_error) + PID->state_a[0]) / 2;

    diff = PID->Kd * ((*current_error) - PID->state_a[0]) / (*sample_period);

    diff_filt = Deriv_Filt[0] * diff
              + Deriv_Filt[0] * PID->state_a[2]
              - Deriv_Filt[1] * PID->state_a[3];

    PID->control_output = diff_filt + int_term + PID->Kp * (*current_error);

    PID->state_a[1] = PID->state_a[0];
    PID->state_a[0] = *current_error;
    PID->state_a[2] = diff;
    PID->state_a[3] = diff_filt;
    PID->int_term   = int_term;
}

void controller_init(ControllerState *state, const PidGains *gains, float sample_period_s)
{
    memset(state, 0, sizeof(ControllerState));
    state->sample_period_s = sample_period_s;

    state->PID_Pend.Kp  = gains->Kp_pend;
    state->PID_Pend.Ki  = gains->Ki_pend;
    state->PID_Pend.Kd  = gains->Kd_pend;

    state->PID_Rotor.Kp = gains->Kp_rotor;
    state->PID_Rotor.Ki = gains->Ki_rotor;
    state->PID_Rotor.Kd = gains->Kd_rotor;

    float fo, Wo, IWon;

    fo   = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY;
    Wo   = 2.0f * 3.141592654f * fo;
    IWon = 2.0f / (Wo * sample_period_s);
    state->Deriv_Filt_Pend[0] = 1.0f / (1.0f + IWon);
    state->Deriv_Filt_Pend[1] = state->Deriv_Filt_Pend[0] * (1.0f - IWon);

    fo   = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR;
    Wo   = 2.0f * 3.141592654f * fo;
    IWon = 2.0f / (Wo * sample_period_s);
    state->Deriv_Filt_Rotor[0] = 1.0f / (1.0f + IWon);
    state->Deriv_Filt_Rotor[1] = state->Deriv_Filt_Rotor[0] * (1.0f - IWon);
}

void controller_compute(ControllerState     *state,
                        const SystemState   *sys,
                        const ControlTarget *target,
                        ControlOutput       *out)
{
    /* Pendulum angle error in stepper steps (unit conversion + polarity) */
    float pend_error = target->slope_correction_steps
                     + target->pendulum_cmd_steps
                     + ENCODER_ANGLE_POLARITY
                       * (sys->pendulum_angle_rad - target->pendulum_angle_ref_rad)
                       / STEPPER_RAD_PER_STEP;

    pid_execute(&state->PID_Pend, &pend_error,
                &state->sample_period_s, state->Deriv_Filt_Pend);

    out->rotor_accel_steps_s2 = state->PID_Pend.control_output;

    if (ENABLE_DUAL_PID == 1) {
        /* Rotor position error in stepper steps */
        float rotor_error = (sys->rotor_angle_rad - target->rotor_angle_ref_rad)
                            / STEPPER_RAD_PER_STEP;

        pid_execute(&state->PID_Rotor, &rotor_error,
                    &state->sample_period_s, state->Deriv_Filt_Rotor);

        out->rotor_accel_steps_s2 += state->PID_Rotor.control_output;
    }
}
