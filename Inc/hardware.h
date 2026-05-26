#ifndef HARDWARE_H
#define HARDWARE_H

#include <stdint.h>
#include <stdbool.h>
/* stm32f4xx_hal.h (for TIM_HandleTypeDef) must be included before this header.
   In practice, main.h or x_nucleo_ihmxx.h already provides it. */

/* System clock frequency (84 MHz for Nucleo F401RE) */
#ifndef RCC_SYS_CLOCK_FREQ
#define RCC_SYS_CLOCK_FREQ  84000000
#endif

/* Physical unit conversion constants */
#define ENCODER_COUNTS_PER_REV  2400.0f   /* 600 PPR * 4 edges (quadrature mode) */
#define ENCODER_RAD_PER_COUNT   (2.0f * 3.14159265f / ENCODER_COUNTS_PER_REV)

#define STEPPER_STEPS_PER_REV   3200.0f   /* 200 full-step * 16 microstepping */
#define STEPPER_RAD_PER_STEP    (2.0f * 3.14159265f / STEPPER_STEPS_PER_REV)

/* Acceleration control mode: 1 = acceleration control, 0 = position command */
#define ACCEL_CONTROL       1
/* Set to 1 to print apply_acceleration timing diagnostics via UART */
#define ACCEL_CONTROL_DATA  0

/*
 * SensorRaw: raw integer counts directly from hardware registers.
 * No calibration, no filtering, no unit conversion applied.
 *
 * This is the ONLY input to the observer layer. The observer converts these
 * raw counts to physical units and estimates velocities.
 */
typedef struct {
    int encoder_counts;
    /* TIM3 quadrature counter, signed integer.
       Scale: ~6.667 counts/degree (600 PPR encoder, quadrature = 4x).
       Reference: TIM3 initial count captured at hardware_init().          */

    int rotor_steps;
    /* L6474 microstep accumulator, signed integer.
       Scale: ~8.889 steps/degree (200 full-step * 1/16 microstepping).
       Reference: zero after hardware_rotor_home().                        */
} SensorRaw;

/*
 * SensorCalib: calibration offsets applied by the observer, not the hardware layer.
 * The hardware layer returns raw counts only.
 */
typedef struct {
    int   encoder_down_counts;   /* TIM3 count at physical pendulum-down position */
    float encoder_offset_counts; /* Additional offset from angle-cal procedure */
    int   select_suspended_mode; /* 0 = inverted (upright target), 1 = suspended */
} SensorCalib;

/*
 * SwingUpSensorState: pendulum excursion tracking for the swing-up algorithm.
 * Updated by hardware_sensor_read() on each call. Stored by the caller (main.c).
 * Pass NULL to hardware_sensor_read() if swing-up tracking is not needed.
 */
typedef struct {
    bool  peaked;
    bool  handled_peak;
    int   zero_crossed;
    int   max_encoder_position;
    int   global_max_encoder_position;
    int   prev_global_max_encoder_position;
    int   previous_encoder_position;
} SwingUpSensorState;

/*
 * ControlOutput: motor acceleration command produced by the controller.
 * Consumed by hardware_motor_write().
 *
 * This is the ONLY output from the controller layer to the hardware layer.
 */
typedef struct {
    float rotor_accel_steps_s2;
    /* Rotor acceleration command [steps/s^2].
       Positive = FORWARD direction.
       Passed directly to apply_acceleration().                            */
} ControlOutput;

/*
 * Initialize the hardware layer.
 * Must be called once before hardware_sensor_read() or hardware_motor_write().
 *
 * @param htim3_handle       Pointer to the TIM3 handle (encoder timer).
 * @param encoder_init_counts  TIM3 counter value at startup (relative reference).
 */
void hardware_init(TIM_HandleTypeDef *htim3_handle, int encoder_init_counts);

/*
 * Read raw sensor values from encoder and motor driver.
 *
 * @param out       Output: populated with current encoder and rotor counts.
 * @param swing_up  Optional swing-up tracking state, updated in place.
 *                  Pass NULL if swing-up tracking is not needed.
 */
void hardware_sensor_read(SensorRaw *out, SwingUpSensorState *swing_up);

/*
 * Read rotor position in microstep units.
 *
 * @param rotor_position Output: current rotor position [steps].
 * @return Range status (-1/0/1), consistent with legacy behavior.
 */
int hardware_rotor_position_read(int *rotor_position);

/*
 * Read pendulum encoder position relative to encoder_position_init.
 *
 * @param encoder_position       Output: current encoder position [counts].
 * @param encoder_position_init  Reference TIM3 count captured at startup.
 * @param htim3                  TIM3 encoder handle.
 * @return Range status (-1/0/1), consistent with legacy behavior.
 */
int hardware_encoder_position_read(int *encoder_position,
                                   int encoder_position_init,
                                   TIM_HandleTypeDef *htim3);

/*
 * Apply a control output to the motor.
 *
 * @param cmd  Acceleration command from controller_compute().
 * @param dt   Sample period in seconds (e.g. 0.002 for 500 Hz).
 */
void hardware_motor_write(const ControlOutput *cmd, float dt);

/*
 * Set the current rotor position as the zero reference.
 * Call once after the rotor has been oriented to the home position.
 */
void hardware_rotor_home(void);

/*
 * PWM step interrupt handler.
 * Registered with the L6474 BSP; do not call directly.
 */
void Main_StepClockHandler(void);

/*
 * Returns nonzero when the motor step PWM period is at maximum (motor stopped).
 * Used by the swing-up algorithm to detect a stopped state.
 */
int Delay_Pulse(void);

#endif /* HARDWARE_H */
