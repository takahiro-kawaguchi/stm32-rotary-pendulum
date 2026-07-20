# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

This repo has two components that talk to each other over a serial link: **STM32 firmware** (this directory's C sources) and a **Python remote controller** (`remote_controller.py`) that can take over control from a PC. On this branch (`my-control`), almost all active development is on the Python side — the firmware is a comparatively stable base the Python side drives via the wire protocol described below.

## Firmware (STM32)

### Build

STM32CubeIDE project. Primary build method is the IDE (Project → Build All, or Ctrl+B). The generated makefile is in `Debug/` and is auto-maintained by the IDE — do not edit it manually.

Target: **STM32 Nucleo F401RE**, compiler `arm-none-eabi-gcc`, toolchain GNU Tools for STM32 13.3.rel1.

Expected build result: **0 errors, 0 warnings**.

To verify changes compile cleanly without IDE access, the makefile can be invoked from the `Debug/` directory:
```
cd Debug && make -j32 all
```

There are no unit tests — hardware-in-the-loop is the only verification path.

### Hardware

- **MCU**: STM32 Nucleo F401RE (Cortex-M4, 84 MHz)
- **Motor driver**: IHM01A1 / L6474 stepper driver (SPI)
- **Motor**: NEMA-17, 200 full-step, 1/16 microstepping → 3200 steps/rev
- **Encoder**: LPD3806-600BM-G5-24C, 600 PPR, quadrature mode (TIM3) → 2400 counts/rev
- **UART2**: Serial terminal at 230400 baud (see `BAUD` in `remote_controller.py`) — used for all user interaction, real-time telemetry, and remote-control commands
- **Motor speed profile**: defaults in `Drivers/BSP/Components/l6474/l6474_target_config.h`

### Architecture

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

#### Key files

| File | Role |
|---|---|
| `Inc/app_control.h` | `AppControlContext` — the central struct owning all layer states and ops pointers |
| `Inc/hardware.h` | `SensorRaw`, `SensorCalib`, `ControlOutput`, `SwingUpSensorState` — layer boundary types |
| `Inc/observer.h` | `ObserverOps`, `ObserverState`, `SystemState` |
| `Inc/controller.h` | `ControllerOps`, `ControllerState`, `ControlTarget`, `PidGains` |
| `Inc/command_shaper.h` | `CommandShaperOps`, `CommandShaperState`, `CommandShaperConfig` |
| `Src/app_bootstrap.c` | One-time system init: HAL, motor, observer, controller |
| `Src/app_session.c` | Full control session: `app_run_control_session()` dispatches to `app_session_home_rotor()`, `app_session_wait_pendulum_rest()`, `app_run_swing_up()`, `app_run_balance_loop()` |
| `Src/app_runtime.c` | `control_execute_cycle()` — single control cycle: read sensors → observer → controller → command shaper → actuate. `report_telemetry()` streams CSV telemetry over UART2. `reference_update()` dispatches to named static functions per feature (chirp, comb, sine, impulse, step) |
| `Src/app_control.c` | Helper functions used by both `app_session.c` and `app_runtime.c` |
| `Src/ui.c` | Serial terminal UI — mode selection (including the `A`/`B`/`C`/`D` remote-control letters below), PID gain entry, real-time reporting |
| `Src/main.c` | HAL init + outer `while(1)` loop: `user_prompt() → user_configuration() → app_run_control_session()` |
| `Inc/edukit_system.h` | Shared `extern` declarations for globals that cross file boundaries; also `#define` constants and `UART_RX_BUFFER_SIZE` |

#### Pluggable interfaces (function pointer pattern)

`ObserverOps`, `ControllerOps`, and `CommandShaperOps` are vtable-like structs of function pointers stored in `AppControlContext`. The default implementations are `OBSERVER_OPS_DEFAULT`, `CONTROLLER_OPS_DEFAULT`, `COMMAND_SHAPER_OPS_DEFAULT`. To substitute a custom controller, allocate its state, populate a `ControllerOps` struct, and assign both to the context before calling `app_run_control_session()`. `CONTROLLER_OPS_REMOTE` is the implementation used by wire modes B/C below — it's a pass-through that applies whatever acceleration the PC last sent.

Current limitation: `ControllerState` and `ObserverState` are concrete structs (contain `arm_pid_instance_a_f32` and IIR coefficients respectively), so a custom implementation that needs different state must work around this.

#### Global variables

Remaining globals in `edukit_system.h` are infrastructure-only:
- HAL handles: `huart2`, `htim3`, `hdma_usart2_rx`
- DMA receive: `RxBuffer`, `Msg`
- UART transmit buffer: `uart_tx_buf[192]`
- ISR volatiles: `desired_pwm_period`, `current_pwm_period`, etc.
- Motor acceleration helpers: `apply_acc_start_time`, `target_velocity_prescaled`, etc.

All control/configuration state lives in `AppControlContext` (sub-structs: `gains`, `timing`, `lpf`, `plant`, `tracking`, `rotor_pos`, `enc_cal`). UI-private state is `static` in `ui.c`.

#### Session modes

At the `user_configuration()` prompt (`Src/ui.c`), a single character selects the session:

- **Numeric digits** dispatch through `get_user_mode_index()` to onboard-only sessions (LQR model variants, suspended pendulum, sine tracking, chirp sweep, system identification, etc.) — only a handful of digits are actually wired to a named config (`mode_1, mode_2, mode_8, mode_11, mode_13, mode_15`); other digits fall through to a default with no session-specific setup.
- **`A`** — onboard PID (`CONTROLLER_OPS_DEFAULT`), starts with the firmware's own swing-up. The PC can send `'r <steps>'` to update the rotor reference mid-session but doesn't compute control.
- **`B`** — remote PID: firmware still runs its own onboard swing-up, but once caught, control switches to `CONTROLLER_OPS_REMOTE` and the PC sends `'u <steps/s^2>\r'` every cycle.
- **`C`** — fully remote: sets `AppControlContext.enable_remote_swing_up = 1`, which (a) skips the firmware's own swing-up routine entirely (the PC drives from hang-down), (b) disables the rotor soft-limit safety check (`Src/app_control.c`) for the session, and (c) switches `report_telemetry()`'s pendulum-angle convention to a direction-agnostic "0 = hang-down" value (`(encoder_position_steps - encoder_position_down) / angle_scale`) instead of the upright-relative `core_sys_state.pendulum_angle_rad` — deliberately not resolving which side is "upright" itself, since `remote_controller.py` owns that interpretation.
- **`D`** — same onboard PID as `A`, but `control_update_dual_pid()` only recomputes every `CONTROL_DECIMATION_FACTOR` (5) cycles instead of every cycle, as a diagnostic to mimic Mode B's 100 Hz-telemetry/500 Hz-actuation split for direct comparison.

Telemetry (all letter modes, and while a session is running generally) streams at 100 Hz as one CSV line per sample: `i,theta_p_deg,theta_r_deg,omega_p_deg_s,omega_r_deg_s,u_last\r\n` (`report_telemetry()`, `Src/app_runtime.c`). Commands accepted mid-session: `u <steps_per_s2>\r` (modes B/C), `r <steps>\r` (mode A), `q\r` (quit — always triggers a full reset back to the mode-selection prompt).

## Python Remote Controller (`remote_controller.py`)

Single-file [PEP 723](https://peps.python.org/pep-0723/) script — dependencies (`pyserial`, `matplotlib`, `numpy`, `scipy`) are declared inline and resolved automatically by `uv`. **Always invoke it with `uv run`, never bare `python`/`python3`** — this installs/caches the declared deps in an isolated environment on first run.

```
uv run remote_controller.py [PORT]          # CLI: auto-starts Mode B, prints telemetry to stdout
uv run remote_controller.py [PORT] --gui     # Tk/matplotlib GUI: live plot, mode selector, Start/Stop
uv run remote_controller.py [PORT] --school  # simplified 8-step slider GUI (implies --gui); see STEP_DEFS
```
`PORT` defaults to `COM3` if omitted. There's no separate build/lint/test step; `uv run python -m py_compile remote_controller.py` is the quick correctness check used throughout development, and real verification is always against live hardware.

⚠️ Before running anything that opens the serial port, confirm no real pendulum hardware is connected/expected to respond unexpectedly — commands sent to a live device move the motor.

### `LinkManager` — connection lifecycle

`LinkManager` (owns the `serial.Serial` port) runs on a background thread as a small state machine: `WAITING_BOOT → AT_PROMPT → RUNNING → STOPPING`, resyncing after the MCU's mandatory full reset on any session end (`'q'` always reboots it). The Tk main thread only ever reads `state`/`status_text`/`data` from it — never touches `ser` directly. `selected_mode` ('B'/'C'/'1'/'D') picks which firmware wire mode to start; `_step_running()` is the per-telemetry-line dispatch and is the central place to read for how a given mode is actually driven.

### Balance/swing-up controllers (Mode C, all interchangeable via `LinkManager.balance_controller`/`lqr_state_source`)

- **`SwingUpController`** — a faithful Python port of the firmware's own bang-bang swing-up (zero-crossing/peak detection ported from `hardware.c`'s tracking, stage amplitudes from `SWINGUP_PARAMS`), plus a continuously-running rotor-position PID (the firmware's own `BSP_MotorControl_Move()` has no Python equivalent, since this script's only actuator primitive is a commanded acceleration `u`).
- **`DualPidController`** — mirrors the firmware's `controller_compute_dual_pid()` gain-scheduling (catch-phase vs. steady-state gains, `Src/app_runtime.c`'s `angle_cal_update()`), computing its own filtered derivative from position rather than trusting telemetry's `omega_p`/`omega_r`.
- **`LQRController`** — full 4-state `[phi, theta_r, phi_dot, omega_r]` LQR, gain from `LQR_PARAMS` via Bryson's rule (`recompute_lqr_gain()`), reads `omega_p`/`omega_r` from telemetry directly (clamped — see `OMEGA_GLITCH_CLAMP_DEG_S`).
- **`KalmanObserver`** / **`DisturbanceObserver`** — steady-state LQE state estimators (dual of the LQR problem, `recompute_observer_gain()`/`recompute_dob_gain()`) that estimate the LQR's state (the latter also a slowly-relaxing matched input disturbance) from `theta_p`/`theta_r` position measurements only, as an alternative to trusting telemetry's raw differentiated `omega_p`/`omega_r`.

The identified plant model (`MODEL_W0_SQ`/`MODEL_GAMMA`/`MODEL_K`, from real swing-up telemetry) backs all of the above; `rebuild_model_matrices()` must be called after changing those three (e.g. from `identify_model()`'s live system-ID) to propagate into every dependent gain.

### School mode (`--school`)

A simplified GUI layered on top of the same engine above, not a separate implementation — Steps 1-4 use a small pendulum-blind `RotorPDController`; Steps 5-8 reuse `SwingUpController`/`DualPidController`/`LQRController` unmodified, scaled on/off per step via `STEP_GAINS` 0/1 multipliers. `STEP_DEFS` is the single source of truth the GUI builds each step's tab from (label, challenge text, slider specs, which controller/gains that step drives); the curriculum it implements is `計画.md`. Most sliders are plain per-step values, but some are tagged `shared` (Step2/Step3 intentionally drive the same underlying gain) — check a slider's `shared` key before assuming changing it elsewhere is safe.

### Logs

Each GUI session writes a timestamped CSV to `logs/` (gitignored) with both raw telemetry and every controller's internal state (model predictions, Kalman estimates, disturbance estimates) for offline analysis — check the header row in `LinkManager._open_log()` for the current exact column set, as it grows when new estimators are added.
