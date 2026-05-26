#include "main.h"
#include "edukit_system.h"
#include "hardware.h"
#include "observer.h"
#include <math.h>
#include <string.h>

void observer_init(ObserverState *state, float sample_period_s)
{
    memset(state, 0, sizeof(ObserverState));
    state->sample_period_s = sample_period_s;

    float fo, Wo, IWon;

    /* Pendulum velocity low-pass filter (bilinear-transform first-order) */
    fo   = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY;
    Wo   = 2.0f * 3.141592654f * fo;
    IWon = 2.0f / (Wo * sample_period_s);
    state->Deriv_Filt_Pend[0] = 1.0f / (1.0f + IWon);
    state->Deriv_Filt_Pend[1] = state->Deriv_Filt_Pend[0] * (1.0f - IWon);

    /* Rotor velocity low-pass filter */
    fo   = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR;
    Wo   = 2.0f * 3.141592654f * fo;
    IWon = 2.0f / (Wo * sample_period_s);
    state->Deriv_Filt_Rotor[0] = 1.0f / (1.0f + IWon);
    state->Deriv_Filt_Rotor[1] = state->Deriv_Filt_Rotor[0] * (1.0f - IWon);
}

void observer_update(const SensorRaw   *raw,
                     const SensorCalib *cal,
                     ObserverState     *state,
                     SystemState       *out)
{
    /* --- Pendulum angle (raw counts → rad) -------------------------------- */
    float pend_counts = (float)(raw->encoder_counts - cal->encoder_down_counts);
    if (cal->select_suspended_mode == 0) {
        /* Inverted mode: subtract half-revolution so 0 = upright */
        pend_counts -= ENCODER_COUNTS_PER_REV / 2.0f;
    }
    pend_counts -= cal->encoder_offset_counts;
    out->pendulum_angle_rad = pend_counts * ENCODER_RAD_PER_COUNT;

    /* --- Rotor angle (steps → rad) ---------------------------------------- */
    out->rotor_angle_rad = (float)raw->rotor_steps * STEPPER_RAD_PER_STEP;

    /* --- Pendulum velocity (finite difference + IIR) ---------------------- */
    float dpend = (out->pendulum_angle_rad - state->prev_pend_angle_rad)
                  / state->sample_period_s;
    out->pendulum_velocity_rad_s =
          state->Deriv_Filt_Pend[0] * dpend
        + state->Deriv_Filt_Pend[0] * state->pend_vel_diff_prev
        - state->Deriv_Filt_Pend[1] * state->pend_vel_filt_prev;
    state->pend_vel_diff_prev  = dpend;
    state->pend_vel_filt_prev  = out->pendulum_velocity_rad_s;
    state->prev_pend_angle_rad = out->pendulum_angle_rad;

    /* --- Rotor velocity (finite difference + IIR) ------------------------- */
    float drotor = (out->rotor_angle_rad - state->prev_rotor_angle_rad)
                   / state->sample_period_s;
    out->rotor_velocity_rad_s =
          state->Deriv_Filt_Rotor[0] * drotor
        + state->Deriv_Filt_Rotor[0] * state->rotor_vel_diff_prev
        - state->Deriv_Filt_Rotor[1] * state->rotor_vel_filt_prev;
    state->rotor_vel_diff_prev  = drotor;
    state->rotor_vel_filt_prev  = out->rotor_velocity_rad_s;
    state->prev_rotor_angle_rad = out->rotor_angle_rad;
}

const ObserverOps OBSERVER_OPS_DEFAULT = {
    .init = observer_init,
    .update = observer_update,
};
