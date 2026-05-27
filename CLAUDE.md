# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

This is a STM32CubeIDE project. The primary build method is via the IDE (Project → Build All, or Ctrl+B). The generated makefile is in `Debug/` and is auto-maintained by the IDE — do not edit it manually.

Target: **STM32 Nucleo F401RE**, compiler `arm-none-eabi-gcc`, toolchain GNU Tools for STM32 13.3.rel1.

Expected build result: **0 errors, 0 warnings**.

To verify changes compile cleanly without IDE access, the makefile can be invoked from the `Debug/` directory:
```
cd Debug && make -j32 all
```

There are no unit tests — hardware-in-the-loop is the only verification path.

## Hardware

- **MCU**: STM32 Nucleo F401RE (Cortex-M4, 84 MHz)
- **Motor driver**: IHM01A1 / L6474 stepper driver (SPI)
- **Motor**: NEMA-17, 200 full-step, 1/16 microstepping → 3200 steps/rev
- **Encoder**: LPD3806-600BM-G5-24C, 600 PPR, quadrature mode (TIM3) → 2400 counts/rev
- **UART2**: Serial terminal at 115200 baud — used for all user interaction and real-time data reporting
- **Motor speed profile**: defaults in `Drivers/BSP/Components/l6474/l6474_target_config.h`

## Architecture

The codebase was refactored from a monolithic `main.c` into a layered architecture. The data flow is:

```
Hardware (TIM3/L6474)
    ↓  SensorRaw  (raw integer counts)
Observer  (observer.c)
    ↓  SystemState  (rad, rad/s)
Controller  (controller.c)
    ↓  ControlOutput  (rotor_accel_steps_s2)
CommandShaper  (command_shaper.c)
    ↓
Hardware motor write  (hardware.c)
```

All layer states and operation function pointers are bundled in **`AppControlContext`** (defined in `Inc/app_control.h`). Pass `&g_app` (declared in `main.c`) to every function that needs cross-layer access.

### Key files

| File | Role |
|---|---|
| `Inc/app_control.h` | `AppControlContext` — the central struct owning all layer states and ops pointers |
| `Inc/hardware.h` | `SensorRaw`, `SensorCalib`, `ControlOutput`, `SwingUpSensorState` — layer boundary types |
| `Inc/observer.h` | `ObserverOps`, `ObserverState`, `SystemState` |
| `Inc/controller.h` | `ControllerOps`, `ControllerState`, `ControlTarget`, `PidGains` |
| `Inc/command_shaper.h` | `CommandShaperOps`, `CommandShaperState`, `CommandShaperConfig` |
| `Src/app_bootstrap.c` | One-time system init: HAL, motor, observer, controller |
| `Src/app_session.c` | Full control session: `app_run_control_session()` dispatches to `app_session_home_rotor()`, `app_session_wait_pendulum_rest()`, `app_run_swing_up()`, `app_run_balance_loop()` |
| `Src/app_runtime.c` | `control_execute_cycle()` — single control cycle: read sensors → observer → controller → command shaper → actuate. `reference_update()` dispatches to named static functions per feature (chirp, comb, sine, impulse, step) |
| `Src/app_control.c` | Helper functions used by both `app_session.c` and `app_runtime.c` |
| `Src/ui.c` | Serial terminal UI — mode selection, PID gain entry, real-time reporting |
| `Src/main.c` | HAL init + outer `while(1)` loop: `user_prompt() → user_configuration() → app_run_control_session()` |
| `Inc/edukit_system.h` | Shared `extern` declarations for globals that cross file boundaries; also `#define` constants and `UART_RX_BUFFER_SIZE` |

### Pluggable interfaces (function pointer pattern)

`ObserverOps`, `ControllerOps`, and `CommandShaperOps` are vtable-like structs of function pointers stored in `AppControlContext`. The default implementations are `OBSERVER_OPS_DEFAULT`, `CONTROLLER_OPS_DEFAULT`, `COMMAND_SHAPER_OPS_DEFAULT`. To substitute a custom controller, allocate its state, populate a `ControllerOps` struct, and assign both to the context before calling `app_run_control_session()`.

Current limitation: `ControllerState` and `ObserverState` are concrete structs (contain `arm_pid_instance_a_f32` and IIR coefficients respectively), so a custom implementation that needs different state must work around this.

### Global variables

Refactoring (Batches 1–18 + Phase 1–2) is complete. Remaining globals in `edukit_system.h` are infrastructure-only:
- HAL handles: `huart2`, `htim3`, `hdma_usart2_rx`
- DMA receive: `RxBuffer`, `Msg`
- UART transmit buffer: `uart_tx_buf[192]`
- ISR volatiles: `desired_pwm_period`, `current_pwm_period`, etc.
- Motor acceleration helpers: `apply_acc_start_time`, `target_velocity_prescaled`, etc.

All control/configuration state lives in `AppControlContext` (sub-structs: `gains`, `timing`, `lpf`, `plant`, `tracking`, `rotor_pos`, `enc_cal`). UI-private state is `static` in `ui.c`.

### Control modes

The outer loop in `main.c` calls `user_configuration()` which selects among 19 named modes (LQR model variants, suspended pendulum, sine tracking, chirp sweep, system identification, interactive control, etc.). Mode flags like `enable_swing_up`, `enable_state_feedback`, `select_suspended_mode` are read inside `app_session.c` and `app_runtime.c`.
