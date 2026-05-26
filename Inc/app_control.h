#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include "hardware.h"
#include "observer.h"
#include "controller.h"
#include "command_shaper.h"

typedef struct AppControlContext {
	SensorRaw core_hw_raw;
	SensorCalib core_hw_cal;
	ObserverState core_obs_state;
	SystemState core_sys_state;
	ControllerState core_ctl_state;
	ControlTarget core_ctl_target;
	ControlOutput core_ctl_out;
	CommandShaperState core_cmd_shaper_state;
	const ObserverOps *core_observer_ops;
	const ControllerOps *core_controller_ops;
	const CommandShaperOps *core_command_shaper_ops;
} AppControlContext;

void app_reset_command_shaper_state(AppControlContext *ctx);
void app_assign_pid_gains_from_user(AppControlContext *ctx);
void app_init_control_pipeline(AppControlContext *ctx, int encoder_init_counts,
		float sample_period_s);

void control_shutdown_sequence(AppControlContext *ctx);
int control_handle_runtime_configuration(AppControlContext *ctx, int i);
int control_update_state_and_safety(AppControlContext *ctx);
void control_update_slope_correction(int i);
void control_update_dual_pid(AppControlContext *ctx);
void control_finalize_command_and_actuate(AppControlContext *ctx, int i);
int control_wait_next_cycle(void);

#endif
