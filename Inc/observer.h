#ifndef OBSERVER_H
#define OBSERVER_H

/* stm32f4xx_hal.h and hardware.h must be included before this header */
#include "hardware.h"

/*
 * SystemState: full system state in SI units (rad, rad/s).
 * Output of observer_update(). Input to controller_compute().
 * Swapping the observer implementation must fill all four fields.
 */
typedef struct {
    float pendulum_angle_rad;       /* pendulum angle (0 = upright) */
    float pendulum_velocity_rad_s;  /* pendulum angular velocity [rad/s] */
    float rotor_angle_rad;          /* rotor angle (0 = home) */
    float rotor_velocity_rad_s;     /* rotor angular velocity [rad/s] */
} SystemState;

/*
 * ObserverState: internal state for the default (finite-difference + IIR) observer.
 * Allocate one instance in main(); pass a pointer to every observer call.
 */
typedef struct {
    float sample_period_s;

    float Deriv_Filt_Pend[2];   /* pendulum velocity IIR coefficients [b0, a1] */
    float Deriv_Filt_Rotor[2];  /* rotor velocity IIR coefficients [b0, a1] */

    float prev_pend_angle_rad;
    float pend_vel_diff_prev;
    float pend_vel_filt_prev;

    float prev_rotor_angle_rad;
    float rotor_vel_diff_prev;
    float rotor_vel_filt_prev;
} ObserverState;

/*
 * Initialize filter coefficients and zero all state.
 * Call once after the sample period is known.
 */
void observer_init(ObserverState *state, float sample_period_s);

/*
 * Convert raw sensor counts to SI-unit system state.
 * Call once per control cycle, after hardware_sensor_read().
 */
void observer_update(const SensorRaw   *raw,
                     const SensorCalib *cal,
                     ObserverState     *state,
                     SystemState       *out);

#endif /* OBSERVER_H */
