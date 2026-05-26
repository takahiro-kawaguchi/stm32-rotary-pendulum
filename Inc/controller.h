#ifndef CONTROLLER_H
#define CONTROLLER_H

/* stm32f4xx_hal.h, edukit_system.h, hardware.h, observer.h must be included before this header */
#include "observer.h"   /* SystemState */
#include "hardware.h"   /* ControlOutput */

/*
 * PidGains: initial gain and filter parameters for controller_init().
 */
typedef struct {
    float Kp_pend,  Ki_pend,  Kd_pend;
    float Kp_rotor, Ki_rotor, Kd_rotor;
} PidGains;

/*
 * ControlTarget: reference signals and feedforward terms for one control cycle.
 * Set by the main loop; passed to controller_compute() each cycle.
 */
typedef struct {
    float pendulum_angle_ref_rad;  /* pendulum target angle [rad]  (0 = upright) */
    float rotor_angle_ref_rad;     /* rotor target angle [rad]     (0 = home) */
    float slope_correction_steps;  /* platform inclination correction [steps] */
    float pendulum_cmd_steps;      /* additional pendulum tracking command [steps] */
} ControlTarget;

/*
 * ControllerState: all mutable controller state.
 * Allocate one instance in main(); pass a pointer to every controller call.
 */
typedef struct {
    arm_pid_instance_a_f32 PID_Pend;
    arm_pid_instance_a_f32 PID_Rotor;
    float Deriv_Filt_Pend[2];   /* [b0, a1] for pendulum derivative filter */
    float Deriv_Filt_Rotor[2];  /* [b0, a1] for rotor derivative filter */
    float sample_period_s;
} ControllerState;

/*
 * Initialize controller state and precompute filter coefficients.
 * Call once after the sample period is known.
 */
void controller_init(ControllerState *state, const PidGains *gains, float sample_period_s);

/*
 * Compute one control cycle: SystemState + ControlTarget → ControlOutput.
 * Call once per cycle, after observer_update().
 */
void controller_compute(ControllerState     *state,
                        const SystemState   *sys,
                        const ControlTarget *target,
                        ControlOutput       *out);

#endif /* CONTROLLER_H */
