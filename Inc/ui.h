#ifndef UI_H
#define UI_H

#include <stdint.h>
/* stm32f4xx_hal.h and edukit_system.h must be included before this header */

void read_float(uint32_t *RxBuffer_ReadIdx, uint32_t *RxBuffer_WriteIdx, uint32_t *readBytes, float *float_return);
void read_int(uint32_t *RxBuffer_ReadIdx, uint32_t *RxBuffer_WriteIdx, uint32_t *readBytes, int *int_return);
void read_char(uint32_t *RxBuffer_ReadIdx, uint32_t *RxBuffer_WriteIdx, uint32_t *readBytes, char *char_return);

void assign_mode_1(arm_pid_instance_a_f32 *PID_Pend, arm_pid_instance_a_f32 *PID_Rotor);
void assign_mode_2(arm_pid_instance_a_f32 *PID_Pend, arm_pid_instance_a_f32 *PID_Rotor);
void assign_mode_3(arm_pid_instance_a_f32 *PID_Pend, arm_pid_instance_a_f32 *PID_Rotor);

int  mode_index_identification(char *user_config_input, int config_command_control,
        float *adjust_increment, arm_pid_instance_a_f32 *PID_Pend,
        arm_pid_instance_a_f32 *PID_Rotor);

void set_mode_strings(void);
void user_prompt(void);
void get_user_mode_index(char *user_string, int *char_mode_select, int *mode_index, int *mode_interactive);
void user_configuration(void);

void rotor_encoder_test(void);
void motor_actuator_characterization_mode(void);
void interactive_rotor_actuator_control(void);

#endif /* UI_H */
