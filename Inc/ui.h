#ifndef UI_H
#define UI_H

#include <stdint.h>
#include "app_control.h"
/* stm32f4xx_hal.h and edukit_system.h must be included before this header */

void assign_mode_1(AppControlContext *ctx, arm_pid_instance_a_f32 *PID_Pend, arm_pid_instance_a_f32 *PID_Rotor);
void assign_mode_2(AppControlContext *ctx, arm_pid_instance_a_f32 *PID_Pend, arm_pid_instance_a_f32 *PID_Rotor);
void assign_mode_3(AppControlContext *ctx, arm_pid_instance_a_f32 *PID_Pend, arm_pid_instance_a_f32 *PID_Rotor);

int  mode_index_identification(AppControlContext *ctx, char *user_config_input, int config_command_control,
        float *adjust_increment, arm_pid_instance_a_f32 *PID_Pend,
        arm_pid_instance_a_f32 *PID_Rotor);

void set_mode_strings(void);
void user_prompt(void);
void ui_set_mode_interactive(int enabled);
int ui_get_mode_interactive(void);
void user_configuration(AppControlContext *ctx);
int ui_process_runtime_input(int cycle_index, AppControlContext *ctx,
        arm_pid_instance_a_f32 *PID_Pend,
        arm_pid_instance_a_f32 *PID_Rotor);

void rotor_encoder_test(AppControlContext *ctx);
void motor_actuator_characterization_mode(AppControlContext *ctx);
void interactive_rotor_actuator_control(void);

#endif /* UI_H */
