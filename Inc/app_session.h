#ifndef APP_SESSION_H
#define APP_SESSION_H

#include "app_control.h"

void app_run_mode_loop(AppControlContext *ctx);
void app_prepare_control_session(AppControlContext *ctx);
void app_run_control_session(AppControlContext *ctx);

#endif
