#include "main.h"
#include "edukit_system.h"
#include "remote_controller.h"

static float s_u = 0.0f;

void remote_controller_set_u(float u)
{
    s_u = u;
}

static void remote_init(ControllerState *state, const PidGains *gains, float sample_period_s)
{
    (void)state; (void)gains; (void)sample_period_s;
}

static void remote_compute_dual(ControllerState *state,
                                const SystemState *sys,
                                ControlTarget *target,
                                const ControllerDualPidInput *input,
                                ControllerDualPidRuntime *runtime,
                                ControlOutput *out)
{
    (void)state; (void)sys; (void)target; (void)input; (void)runtime;
    out->rotor_accel_steps_s2 = s_u;
}

const ControllerOps CONTROLLER_OPS_REMOTE = {
    .init         = remote_init,
    .compute      = NULL,
    .compute_dual = remote_compute_dual,
};
