#ifndef REMOTE_CONTROLLER_H
#define REMOTE_CONTROLLER_H

/* controller.h requires stm32f4xx_hal.h and edukit_system.h to be included first.
 * Callers must include main.h and edukit_system.h before this header,
 * or include app_control.h which already pulls in the full chain. */
#include "controller.h"

/* ControllerOps implementation that accepts control output from PC via UART.
 * The balance loop passes through s_u (set by remote_controller_set_u) instead
 * of computing a PID output.  Swing-up is unaffected (it drives the motor
 * directly and never calls ControllerOps). */
extern const ControllerOps CONTROLLER_OPS_REMOTE;

/* Called by the UART command parser when "u <value>" is received. */
void remote_controller_set_u(float u);

#endif /* REMOTE_CONTROLLER_H */
