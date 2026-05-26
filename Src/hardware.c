#include "main.h"
#include "hardware.h"
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/* Swing-up state globals defined in main.c */
extern bool     peaked;
extern bool     handled_peak;
extern int      zero_crossed;
extern int      max_encoder_position;
extern int      global_max_encoder_position;
extern int      prev_global_max_encoder_position;
extern int      previous_encoder_position;

/* Internal acceleration/PWM control parameters (private to hardware layer) */
#define HW_PWM_COUNT_SAFETY_MARGIN  2
#define HW_MAXIMUM_ACCELERATION     131071
#define HW_MAXIMUM_DECELERATION     131071
#define HW_MAXIMUM_SPEED            131071

/* ---------------------------------------------------------------------------
 * State variables (previously global in main.c).
 * Declared extern in edukit_system.h so existing code can still access them.
 * --------------------------------------------------------------------------- */

/* ISR-shared volatile state */
volatile uint32_t apply_acc_start_time;
volatile uint32_t clock_int_time;
volatile uint32_t clock_int_tick;
volatile uint32_t desired_pwm_period;
volatile uint32_t current_pwm_period;

/* Motor velocity integrator */
float target_velocity_prescaled;

/* Encoder range error (set by encoder_position_read, returned as int) */
int range_error;

/* TIM3 counter staging register */
uint32_t cnt3;

/* Private to this file: initialized by hardware_init() */
static TIM_HandleTypeDef *s_htim3;
static int                s_encoder_init;


/* ===========================================================================
 * hardware_init
 * =========================================================================== */
void hardware_init(TIM_HandleTypeDef *htim3_handle, int encoder_init_counts)
{
    s_htim3        = htim3_handle;
    s_encoder_init = encoder_init_counts;

    target_velocity_prescaled = 0.0f;
    desired_pwm_period        = 0;
    current_pwm_period        = 0;
}


/* ===========================================================================
 * hardware_rotor_home
 * =========================================================================== */
void hardware_rotor_home(void)
{
    uint32_t pos = BSP_MotorControl_GetPosition(0);
    BSP_MotorControl_SetHome(0, pos);
}


/* ===========================================================================
 * Main_StepClockHandler  (PWM step ISR, called by L6474 BSP)
 *
 * Stepper motor acceleration/speed/direction/position control
 * developed by Ryan Nemiroff.
 * =========================================================================== */
void Main_StepClockHandler(void)
{
    uint32_t period_local = desired_pwm_period;
    clock_int_time = DWT->CYCCNT;
    if (period_local != 0) {
        L6474_Board_Pwm1SetPeriod(period_local);
        current_pwm_period = period_local;
    }
}


/* ===========================================================================
 * Delay_Pulse
 * =========================================================================== */
int Delay_Pulse(void)
{
    return desired_pwm_period == UINT32_MAX;
}


/* ===========================================================================
 * apply_acceleration  (private helper)
 *
 * Integrates acceleration into velocity, enforces speed limits, and updates
 * the PWM period for the stepper driver.
 *
 * Stepper motor acceleration/speed/direction/position control
 * developed by Ryan Nemiroff.
 * =========================================================================== */
void apply_acceleration(float *acc, float *vel_prescaled, float dt)
{
    uint32_t cur_period = current_pwm_period;
    uint32_t des_period = desired_pwm_period;

    apply_acc_start_time = DWT->CYCCNT;

    motorDir_t old_dir = (*vel_prescaled > 0) ? FORWARD : BACKWARD;

    if (old_dir == FORWARD) {
        if (*acc >  HW_MAXIMUM_ACCELERATION) *acc =  HW_MAXIMUM_ACCELERATION;
        if (*acc < -HW_MAXIMUM_DECELERATION) *acc = -HW_MAXIMUM_DECELERATION;
    } else {
        if (*acc < -HW_MAXIMUM_ACCELERATION) *acc = -HW_MAXIMUM_ACCELERATION;
        if (*acc >  HW_MAXIMUM_DECELERATION) *acc =  HW_MAXIMUM_DECELERATION;
    }

    *vel_prescaled += L6474_Board_Pwm1PrescaleFreq(*acc) * dt;
    motorDir_t new_dir = (*vel_prescaled > 0) ? FORWARD : BACKWARD;

    float max_vel = L6474_Board_Pwm1PrescaleFreq(HW_MAXIMUM_SPEED);
    if      (*vel_prescaled >  max_vel)  *vel_prescaled =  max_vel;
    else if (*vel_prescaled < -max_vel)  *vel_prescaled = -max_vel;

    float speed_prescaled = (new_dir == FORWARD) ? *vel_prescaled : -(*vel_prescaled);

    uint32_t effective_period = des_period;
    float period_float = roundf((float)RCC_SYS_CLOCK_FREQ / speed_prescaled);
    if (!(period_float < 4294967296.0f)) {
        des_period = UINT32_MAX;
    } else {
        des_period = (uint32_t)period_float;
    }

    if (old_dir != new_dir) {
        L6474_Board_SetDirectionGpio(0, new_dir);
    }

    if (cur_period != 0) {
        uint32_t count     = L6474_Board_Pwm1GetCounter();
        uint32_t time_left = cur_period - count;
        if (time_left > HW_PWM_COUNT_SAFETY_MARGIN) {
            if (old_dir != new_dir) {
                time_left = effective_period;
            }
            uint32_t new_time_left = ((uint64_t)time_left * des_period) / effective_period;
            if (new_time_left != time_left) {
                if (new_time_left < HW_PWM_COUNT_SAFETY_MARGIN)
                    new_time_left = HW_PWM_COUNT_SAFETY_MARGIN;
                cur_period = count + new_time_left;
                if (cur_period < count)
                    cur_period = UINT32_MAX;
                L6474_Board_Pwm1SetPeriod(cur_period);
                current_pwm_period = cur_period;
            }
        }
    } else {
        L6474_Board_Pwm1SetPeriod(des_period);
        current_pwm_period = des_period;
    }

    desired_pwm_period = des_period;
}


/* ===========================================================================
 * hardware_motor_write
 * =========================================================================== */
void hardware_motor_write(const ControlOutput *cmd, float dt)
{
    float acc = cmd->rotor_accel_steps_s2;
    apply_acceleration(&acc, &target_velocity_prescaled, dt);
}


/* ===========================================================================
 * oppositeSigns  (helper used by encoder_position_read)
 *
 * Returns true when x and y have opposite signs (i.e. pendulum crossed bottom).
 * Developed by Markus Dauberschmidt.
 * =========================================================================== */
static bool oppositeSigns(int x, int y)
{
    return ((x ^ y) < 0);
}


/* ===========================================================================
 * encoder_position_read
 *
 * Reads TIM3 quadrature counter and converts to a signed integer relative
 * to the initial count captured at startup.
 * Also updates swing-up peak/zero-crossing tracking (global variables).
 *
 * Swing-up tracking by Markus Dauberschmidt.
 * See https://github.com/OevreFlataeker/steval_edukit_swingup
 * =========================================================================== */
int encoder_position_read(int *encoder_position,
                                   int  encoder_position_init,
                                   TIM_HandleTypeDef *htim3)
{
    cnt3 = __HAL_TIM_GET_COUNTER(htim3);

    if (cnt3 >= 32768) {
        *encoder_position = (int)(cnt3) - 65536;
    } else {
        *encoder_position = (int)(cnt3);
    }

    range_error = 0;
    if (*encoder_position <= -32768) { range_error = -1; *encoder_position = -32768; }
    if (*encoder_position >=  32767) { range_error =  1; *encoder_position =  32767; }

    *encoder_position -= encoder_position_init;

    /* Detect bottom crossing → re-arm peak detection */
    if (oppositeSigns(*encoder_position, previous_encoder_position)) {
        peaked       = 0;
        zero_crossed = 1;
    }

    if (!peaked) {
        if (abs(*encoder_position) >= abs(global_max_encoder_position))
            global_max_encoder_position = *encoder_position;

        if (abs(*encoder_position) >= abs(max_encoder_position)) {
            max_encoder_position = *encoder_position;
        } else {
            peaked       = 1;
            handled_peak = 0;
        }
    }

    previous_encoder_position = *encoder_position;

    return range_error;
}


/* ===========================================================================
 * rotor_position_set  (legacy name; use hardware_rotor_home() in new code)
 * =========================================================================== */
void rotor_position_set(void)
{
    hardware_rotor_home();
}


/* ===========================================================================
 * rotor_position_read
 *
 * Reads the L6474 microstep accumulator and converts to a signed integer.
 * =========================================================================== */
int rotor_position_read(int *rotor_position)
{
    uint32_t rotor_u;
    int ret;

    rotor_u = BSP_MotorControl_GetPosition(0);
    *rotor_position = (int32_t)rotor_u;  /* two's-complement reinterpretation */

    ret = 0;
    if (*rotor_position <= -2147483647 - 1) { ret = -1; }
    if (*rotor_position >=  2147483647)      { ret =  1; }
    return ret;
}


/* ===========================================================================
 * hardware_sensor_read
 *
 * Reads both sensors and populates SensorRaw.
 * Also updates swing-up tracking state if swing_up != NULL.
 *
 * Note: during migration, the main control loop still calls
 * encoder_position_read() and rotor_position_read() directly.
 * This function will replace those direct calls in Step 5.
 * =========================================================================== */
void hardware_sensor_read(SensorRaw *out, SwingUpSensorState *swing_up)
{
    int enc   = 0;
    int rotor = 0;

    encoder_position_read(&enc, s_encoder_init, s_htim3);
    rotor_position_read  (&rotor);

    out->encoder_counts = enc;
    out->rotor_steps    = rotor;

    /* Copy swing-up global state to struct if the caller wants it */
    if (swing_up != NULL) {
        swing_up->peaked                      = peaked;
        swing_up->handled_peak                = handled_peak;
        swing_up->zero_crossed                = zero_crossed;
        swing_up->max_encoder_position        = max_encoder_position;
        swing_up->global_max_encoder_position = global_max_encoder_position;
        swing_up->prev_global_max_encoder_position = prev_global_max_encoder_position;
        swing_up->previous_encoder_position   = previous_encoder_position;
    }
}
