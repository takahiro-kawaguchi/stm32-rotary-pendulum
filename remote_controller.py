#!/usr/bin/env python3
# /// script
# requires-python = ">=3.9"
# dependencies = [
#   "pyserial",
#   "matplotlib",
# ]
# ///
"""
Mode B remote controller for STM32 inverted pendulum.

Usage:
    python remote_controller.py [PORT]          # CLI mode (auto-starts Mode B)
    python remote_controller.py [PORT] --gui     # GUI mode: live plot + Start/Stop

Telemetry from STM32:  i,theta_p_deg,theta_r_deg,omega_p_deg_s,omega_r_deg_s,u_last
Command to STM32:      'u <steps_per_s2>\\r'    (during balance loop)
Quit command:          'q\\r'

GUI mode (--gui):
    Opens the serial port once, then waits for the MCU's mode-selection
    prompt before enabling a "Start" button that sends 'B' to begin Mode B.
    "Stop" sends 'q', which the firmware always answers with a full reset;
    the GUI then waits for the reboot banner and re-arms Start. An unplanned
    reboot (physical reset button, or a firmware safety trip) is detected
    the same way and also re-arms Start automatically. If the firmware ever
    starts a session on its own (e.g. its 60s no-input default-mode
    timeout) while the GUI isn't RUNNING, the GUI sends 'q' to force it back
    to the prompt.
"""

import argparse
import csv
import math
import sys
import threading
import time
from collections import deque
from pathlib import Path

import serial

LOG_DIR = Path(__file__).resolve().parent / "logs"

# ---------------------------------------------------------------------------
# Serial port
# ---------------------------------------------------------------------------
BAUD = 230400

# ---------------------------------------------------------------------------
# Plant constants (must match STM32 firmware)
# ---------------------------------------------------------------------------
STEPPER_RAD_PER_STEP = 2.0 * math.pi / 3200.0   # 200 steps * 16 µstep
ENCODER_ANGLE_POLARITY = -1.0                     # physical sign convention

# ---------------------------------------------------------------------------
# Controller gains
#
# These are NOT the edukit_system.h PRIMARY/SECONDARY_*_MODE_1 macros
# (300/0/30 pend, 15.0/0.0/7.5 rotor) — those only apply before swing-up and
# again ~26s after balance starts, once angle calibration finishes. The gains
# actually catching the pendulum right after swing-up come from
# angle_cal_update()'s i==1 branch (Src/app_runtime.c): Kp_pend=419/Kd_pend=56,
# Kp_rotor=21.1/Kd_rotor=17.2, PLUS enable_state_feedback=1 and a genuine
# accumulating integral compensator (gain=10) that this simple PID doesn't
# implement. Using just the higher P/D values here as a first approximation,
# without the state-feedback/integral-compensator machinery.
# ---------------------------------------------------------------------------
Kp_pend  = 419.0
Kd_pend  =  56.0
Kp_rotor =  21.1
Ki_rotor =   0.0
Kd_rotor =  17.2

# ---------------------------------------------------------------------------
# Derivative IIR low-pass filter coefficients
# Matches STM32 pid_execute filter with same corner frequencies:
#   Primary  (pendulum): fc = 10 Hz,  Ts = 0.01 s (100 Hz telemetry rate)
#   Secondary (rotor):   fc = 50 Hz,  Ts = 0.01 s
# Formula: IWon = 2/(Wo*Ts),  a0 = 1/(1+IWon),  a1 = a0*(1-IWon)
# ---------------------------------------------------------------------------
def _lpf_coeffs(fc_hz: float, ts_s: float):
    Wo = 2.0 * math.pi * fc_hz
    IWon = 2.0 / (Wo * ts_s)
    a0 = 1.0 / (1.0 + IWon)
    a1 = a0 * (1.0 - IWon)
    return a0, a1

_TS = 0.01   # 100 Hz = 5 STM32 cycles × 2 ms
_LP_PEND  = _lpf_coeffs(10.0,  _TS)   # (a0, a1) for pendulum derivative
_LP_ROTOR = _lpf_coeffs(50.0,  _TS)   # (a0, a1) for rotor derivative

# ---------------------------------------------------------------------------
# Safety limits
# ---------------------------------------------------------------------------
ROTOR_LIMIT_DEG  = 200.0    # send u=0 when rotor exceeds this

# Cycles where |theta_p| exceeds this abandon control (treated as "still
# swinging up"). Logged captures showed the pendulum overshoot past the old
# 30.0 threshold on its second (post-zero-crossing) swing while still
# actively decelerating (omega_p magnitude falling: -389 -> -356 -> -309
# right before the cutoff) — i.e. the controller was recovering it, but got
# cut off early. Widened to give oscillations more room to damp out.
PEND_CAPTURE_DEG =  60.0

# steps/s^2, hard clip on total output. Matches HW_MAXIMUM_ACCELERATION /
# HW_MAXIMUM_DECELERATION (Src/hardware.c) — the real physical ceiling
# apply_acceleration() clamps to on the MCU regardless of requested value.
# The onboard PID (Mode 1) never clips its own output at all and relies
# entirely on that hardware-layer clamp; logged telemetry showed it
# requesting up to ~330000 during a successful catch. The previous
# U_MAX=20000 here was an arbitrary extra restriction ~6.5x below even the
# real hardware limit and was the dominant cause of Mode B's divergence
# (logged u saturating at -20000 for ~170ms while theta_p ran away from
# -0.9° past the 30° capture threshold).
U_MAX = 131071.0

# ---------------------------------------------------------------------------
# Rotor reference (steps from centre) and state-feedback / integral
# compensator params, matching angle_cal_update()'s i==1 catch-phase setup
# (Src/app_runtime.c): rotor_position_command_steps=0, feedforward_gain=1,
# integral_compensator_gain=10.
# ---------------------------------------------------------------------------
rotor_ref_steps = 0.0
FEEDFORWARD_GAIN = 1.0
INTEGRAL_COMPENSATOR_GAIN = 10.0


class DualPidController:
    """Stateful dual-PID that mirrors STM32 controller_compute_dual_pid()."""

    def __init__(self):
        # IIR filter state: [prev_diff, prev_diff_filt] for each axis
        self._pend_state  = [0.0, 0.0]   # [diff_k-1, diff_filt_k-1]
        self._rotor_state = [0.0, 0.0]
        self._prev_ep = 0.0
        self._prev_er = 0.0
        self._rotor_integral = 0.0   # accumulating state-feedback integral (rotor position, steps)
        self._ts = _TS

    def reset(self):
        self._pend_state  = [0.0, 0.0]
        self._rotor_state = [0.0, 0.0]
        self._prev_ep = 0.0
        self._prev_er = 0.0
        self._rotor_integral = 0.0

    def _pid_execute(self, Kp, Ki, Kd, error, prev_error, state, lpf):
        """One-step PID with first-order IIR LPF on derivative.
        Matches STM32 pid_execute() exactly."""
        a0, a1 = lpf
        diff      = Kd * (error - prev_error) / self._ts
        diff_filt = a0 * diff + a0 * state[0] - a1 * state[1]
        output    = Kp * error + Ki * self._ts * (error + prev_error) / 2.0 + diff_filt
        state[0]  = diff
        state[1]  = diff_filt
        return output

    def compute(self, theta_p_deg: float, theta_r_deg: float,
                rotor_ref: float = 0.0) -> float:
        theta_p = math.radians(theta_p_deg)
        theta_r = math.radians(theta_r_deg)

        # Primary PID: pendulum
        e_p = ENCODER_ANGLE_POLARITY * theta_p / STEPPER_RAD_PER_STEP
        u_pend = self._pid_execute(
            Kp_pend, 0.0, Kd_pend,
            e_p, self._prev_ep, self._pend_state, _LP_PEND)
        self._prev_ep = e_p

        # Secondary PID: rotor (state-feedback mode: error = current position)
        theta_r_steps = theta_r / STEPPER_RAD_PER_STEP
        e_r = theta_r_steps - rotor_ref
        u_rotor = self._pid_execute(
            Kp_rotor, Ki_rotor, Kd_rotor,
            e_r, self._prev_er, self._rotor_state, _LP_ROTOR)
        self._prev_er = e_r

        # Accumulating integral compensator (controller.c:126-133), active
        # whenever integral_compensator_gain != 0 — replaces, not adds to,
        # the plain feedforward term used when it's 0.
        self._rotor_integral += (rotor_ref * FEEDFORWARD_GAIN - theta_r_steps) * self._ts
        u = u_pend + u_rotor - INTEGRAL_COMPENSATOR_GAIN * self._rotor_integral
        return max(-U_MAX, min(U_MAX, u))


# ---------------------------------------------------------------------------
# CLI mode (unchanged behaviour)
# ---------------------------------------------------------------------------

def connect_and_select_mode(port: str) -> serial.Serial:
    try:
        ser = serial.Serial(port, BAUD, timeout=0.5)
    except serial.SerialException as e:
        print(f"Cannot open {port}: {e}")
        sys.exit(1)

    print(f"Connected: {port}  {BAUD} baud")
    print("Waiting for mode selection prompt …")
    while True:
        raw = ser.readline()
        line = raw.decode('ascii', errors='ignore').strip()
        if line:
            print(line)
        if 'Enter Mode Selection Now' in line:
            break
    ser.write(b'B\r')
    print("Sent: B")
    return ser


def run_cli(ser: serial.Serial) -> None:
    print(f"{'i':>6}  {'θp':>8}  {'θr':>8}  {'u':>9}")

    ctrl = DualPidController()
    in_balance = False

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode('ascii', errors='ignore').strip()
            parts = line.split(',')

            # Non-CSV lines are status messages — print them verbatim
            if len(parts) != 6:
                print(line)
                # Reset controller state when a new session starts
                if 'Pendulum Swing Up Starting' in line:
                    ctrl.reset()
                    in_balance = False
                continue

            try:
                vals = list(map(float, parts))
            except ValueError:
                print(line)
                continue

            i_idx, theta_p, theta_r, omega_p, omega_r, u_prev = vals

            # Safety: rotor near mechanical limit
            if abs(theta_r) > ROTOR_LIMIT_DEG:
                ser.write(b'u 0.0\r')
                print(f"  *** ROTOR LIMIT {theta_r:.1f}° — u=0 ***")
                continue

            # Pendulum too far from upright — don't send u yet
            if abs(theta_p) > PEND_CAPTURE_DEG:
                if in_balance:
                    # Already in balance but pendulum escaped — emergency stop
                    ser.write(b'u 0.0\r')
                print(f"  [swing-up]  θp={theta_p:.2f}°")
                in_balance = False
                ctrl.reset()
                continue

            # First cycle in balance region: initialize PID state from current
            # position so derivative starts at 0 (avoids startup spike)
            if not in_balance:
                ctrl.reset()
                # Prime prev_error to current error so first diff = 0
                theta_p_rad = math.radians(theta_p)
                theta_r_rad = math.radians(theta_r)
                ctrl._prev_ep = ENCODER_ANGLE_POLARITY * theta_p_rad / STEPPER_RAD_PER_STEP
                ctrl._prev_er = theta_r_rad / STEPPER_RAD_PER_STEP
                in_balance = True

            u = ctrl.compute(theta_p, theta_r, rotor_ref_steps)
            ser.write(f'u {u:.1f}\r'.encode())

            print(f"{i_idx:6.0f}  {theta_p:8.3f}°  {theta_r:8.3f}°  {u:9.1f}")

    except KeyboardInterrupt:
        print("\nCtrl+C — sending quit")
        ser.write(b'q\r')
        time.sleep(0.1)
        ser.close()


# ---------------------------------------------------------------------------
# GUI mode: connection lifecycle + Start/Stop + live plot
# ---------------------------------------------------------------------------

BOOT_BANNER = 'System Starting Prepare to Enter Mode Selection'
PROMPT_TEXT = 'Enter Mode Selection Now'
SWING_UP_STARTING = 'Pendulum Swing Up Starting'

STATE_WAITING_BOOT = 'waiting_boot'
STATE_AT_PROMPT    = 'at_prompt'
STATE_RUNNING       = 'running'
STATE_STOPPING      = 'stopping'
STATE_ERROR         = 'error'

STATUS_TEXT = {
    STATE_WAITING_BOOT: '起動待ち…',
    STATE_AT_PROMPT:    '接続済み・モード選択待ち(Startを押してください)',
    STATE_RUNNING:       '制御中 (Mode B)',
    STATE_STOPPING:      '停止処理中…',
    STATE_ERROR:         '通信エラー',
}

MAX_POINTS = 1000


class LinkManager:
    """Owns the serial port and drives the connect/start/stop lifecycle for GUI mode.

    Runs entirely on a background thread. The Tk main thread only ever reads
    `state`, `status_text` and `data` — it never touches `ser` directly.
    """

    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self.state = STATE_WAITING_BOOT
        self.status_text = STATUS_TEXT[STATE_WAITING_BOOT]
        self.start_requested = threading.Event()
        self.stop_requested = threading.Event()
        self.quit = threading.Event()
        self.data = [deque(maxlen=MAX_POINTS) for _ in range(3)]  # theta_p, theta_r, u
        self.ctrl = DualPidController()
        self.in_balance = False
        self._reset_pending = False
        # Mode to start when `start_requested` fires. 'B' = PC computes u
        # (DualPidController, active_control=True). '1' = MCU's own onboard
        # PID runs the whole session (active_control=False, GUI only observes
        # the telemetry the firmware already reports for every mode).
        self.selected_mode = 'B'
        self.active_control = True
        self._log_file = None
        self._log_writer = None
        self._log_start_time = 0.0

    def _set_state(self, state, extra=''):
        self.state = state
        self.status_text = STATUS_TEXT[state] + extra

    def _open_log(self):
        LOG_DIR.mkdir(exist_ok=True)
        path = LOG_DIR / f"log_{time.strftime('%Y%m%d_%H%M%S')}_mode{self.selected_mode}.csv"
        self._log_file = open(path, 'w', newline='')
        self._log_writer = csv.writer(self._log_file)
        self._log_writer.writerow(['time_s', 'i', 'theta_p_deg', 'theta_r_deg',
                                    'omega_p_deg_s', 'omega_r_deg_s', 'u_mcu', 'u_python'])
        self._log_start_time = time.time()
        print(f"Logging to {path}")

    def _close_log(self):
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None
            self._log_writer = None

    def _log_sample(self, i_idx, theta_p, theta_r, omega_p, omega_r, u_mcu, u_python):
        if self._log_writer is None:
            return
        t = time.time() - self._log_start_time
        self._log_writer.writerow([f"{t:.3f}", i_idx, theta_p, theta_r, omega_p, omega_r,
                                    u_mcu, '' if u_python is None else f"{u_python:.1f}"])

    def _read_line(self):
        raw = self.ser.readline()
        if not raw:
            return None
        return raw.decode('ascii', errors='ignore').strip()

    @staticmethod
    def _looks_like_telemetry(line):
        parts = line.split(',')
        if len(parts) != 6:
            return False
        try:
            list(map(float, parts))
        except ValueError:
            return False
        return True

    @classmethod
    def _signals_unwanted_session(cls, line):
        """True if this line means the firmware started a session we didn't
        ask for. Swing-up itself emits no telemetry (report_telemetry() only
        runs once the balance loop starts), so without this check the
        self-heal below would only fire *after* swing-up already completed —
        letting the arm swing up on its own before being reset."""
        if SWING_UP_STARTING in line:
            return True
        return cls._looks_like_telemetry(line)

    def run(self):
        try:
            while not self.quit.is_set():
                if self.state in (STATE_WAITING_BOOT, STATE_STOPPING):
                    self._step_waiting()
                elif self.state == STATE_AT_PROMPT:
                    self._step_at_prompt()
                elif self.state == STATE_RUNNING:
                    self._step_running()
        except serial.SerialException as e:
            self._set_state(STATE_ERROR, f': {e}')
        finally:
            self._close_log()

    def _step_waiting(self):
        """Shared body for WAITING_BOOT and STOPPING: poll until the
        mode-selection prompt reappears (only happens after a full reboot)."""
        line = self._read_line()
        if not line:
            return
        if PROMPT_TEXT in line:
            self._reset_pending = False
            self._set_state(STATE_AT_PROMPT)
            return
        if self._signals_unwanted_session(line) and not self._reset_pending:
            self.ser.write(b'q\r')
            self._reset_pending = True

    def _step_at_prompt(self):
        if self.start_requested.is_set():
            self.start_requested.clear()
            self.active_control = (self.selected_mode == 'B')
            self.ser.write((self.selected_mode + '\r').encode())
            self.ctrl = DualPidController()
            self.in_balance = False
            for d in self.data:
                d.clear()
            self._open_log()
            self._set_state(STATE_RUNNING)
            return

        line = self._read_line()
        if not line:
            return
        if PROMPT_TEXT in line:
            self._reset_pending = False
            return
        if self._signals_unwanted_session(line) and not self._reset_pending:
            # Firmware silently started a session we didn't ask for
            # (e.g. its 60s no-input default-mode timeout) — force it back.
            self.ser.write(b'q\r')
            self._reset_pending = True

    def _step_running(self):
        if self.stop_requested.is_set():
            self.stop_requested.clear()
            self.ser.write(b'q\r')
            self._close_log()
            self._set_state(STATE_STOPPING)
            return

        line = self._read_line()
        if not line:
            return

        if BOOT_BANNER in line:
            self._close_log()
            self._set_state(STATE_WAITING_BOOT, extra='(予期しないリセットを検知しました)')
            return

        parts = line.split(',')
        if len(parts) != 6:
            print(line)
            if 'Pendulum Swing Up Starting' in line:
                self.ctrl.reset()
                self.in_balance = False
            return

        try:
            vals = list(map(float, parts))
        except ValueError:
            print(line)
            return

        i_idx, theta_p, theta_r, omega_p, omega_r, u_prev = vals

        if not self.active_control:
            # Mode 1: the MCU's own onboard PID runs the session — just
            # observe the telemetry it already reports, send nothing back.
            self._log_sample(i_idx, theta_p, theta_r, omega_p, omega_r, u_prev, None)
            self.data[0].append(theta_p)
            self.data[1].append(theta_r)
            self.data[2].append(u_prev)
            return

        if abs(theta_r) > ROTOR_LIMIT_DEG:
            self._log_sample(i_idx, theta_p, theta_r, omega_p, omega_r, u_prev, None)
            self.ser.write(b'u 0.0\r')
            return

        if abs(theta_p) > PEND_CAPTURE_DEG:
            if self.in_balance:
                self.ser.write(b'u 0.0\r')
            self.in_balance = False
            self.ctrl.reset()
            self._log_sample(i_idx, theta_p, theta_r, omega_p, omega_r, u_prev, None)
            return

        if not self.in_balance:
            self.ctrl.reset()
            theta_p_rad = math.radians(theta_p)
            theta_r_rad = math.radians(theta_r)
            self.ctrl._prev_ep = ENCODER_ANGLE_POLARITY * theta_p_rad / STEPPER_RAD_PER_STEP
            self.ctrl._prev_er = theta_r_rad / STEPPER_RAD_PER_STEP
            self.in_balance = True

        u = self.ctrl.compute(theta_p, theta_r, rotor_ref_steps)
        self.ser.write(f'u {u:.1f}\r'.encode())
        self._log_sample(i_idx, theta_p, theta_r, omega_p, omega_r, u_prev, u)

        self.data[0].append(theta_p)
        self.data[1].append(theta_r)
        self.data[2].append(u)


def run_gui(port: str) -> None:
    # Imported lazily so plain CLI usage never requires matplotlib/tkinter.
    import tkinter as tk
    from tkinter import ttk
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

    try:
        ser = serial.Serial(port, BAUD, timeout=0.5)
    except serial.SerialException as e:
        print(f"Cannot open {port}: {e}")
        sys.exit(1)

    print(f"Connected: {port}  {BAUD} baud")

    link = LinkManager(ser)
    threading.Thread(target=link.run, daemon=True).start()

    root = tk.Tk()
    root.title("STM32 Pendulum — Remote Controller")
    root.geometry("900x650")

    top = ttk.Frame(root, padding=8)
    top.pack(fill="x")

    status_var = tk.StringVar(value=link.status_text)
    ttk.Label(top, textvariable=status_var).pack(side="left")

    stop_btn = ttk.Button(top, text="Stop", command=link.stop_requested.set)
    start_btn = ttk.Button(top, text="Start", command=link.start_requested.set)
    stop_btn.pack(side="right", padx=4)
    start_btn.pack(side="right", padx=4)

    mode_var = tk.StringVar(value=link.selected_mode)

    def on_mode_change():
        link.selected_mode = mode_var.get()

    mode_frame = ttk.Frame(top)
    mode_frame.pack(side="right", padx=12)
    mode_b_radio = ttk.Radiobutton(mode_frame, text="Mode B (Python PID)",
                                    variable=mode_var, value='B', command=on_mode_change)
    mode_1_radio = ttk.Radiobutton(mode_frame, text="Mode 1 (MCU内蔵PID)",
                                    variable=mode_var, value='1', command=on_mode_change)
    mode_b_radio.pack(side="left")
    mode_1_radio.pack(side="left")

    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(7, 6))
    labels = ["Pendulum Angle [deg]", "Rotor Angle [deg]", "Input u [steps/s²]"]
    lines = []
    for i in range(3):
        line, = ax[i].plot([], [])
        lines.append(line)
        ax[i].set_ylabel(labels[i])
    ax[-1].set_xlabel("Sample")
    fig.tight_layout()

    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.get_tk_widget().pack(fill="both", expand=True)

    def tick():
        status_var.set(link.status_text)
        at_prompt = link.state == STATE_AT_PROMPT
        start_btn.state(['!disabled'] if at_prompt else ['disabled'])
        stop_btn.state(['!disabled'] if link.state == STATE_RUNNING else ['disabled'])
        mode_b_radio.state(['!disabled'] if at_prompt else ['disabled'])
        mode_1_radio.state(['!disabled'] if at_prompt else ['disabled'])

        for i in range(3):
            lines[i].set_data(range(len(link.data[i])), list(link.data[i]))
            ax[i].relim()
            ax[i].autoscale_view()
        canvas.draw_idle()

        if not link.quit.is_set():
            root.after(100, tick)

    def on_close():
        link.quit.set()
        if link.state == STATE_RUNNING:
            link.stop_requested.set()
        time.sleep(0.2)
        try:
            ser.close()
        except Exception:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    tick()
    root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Mode B remote controller for STM32 inverted pendulum.")
    parser.add_argument('port', nargs='?', default='COM3',
                         help="serial port, e.g. COM3 or /dev/ttyACM0")
    parser.add_argument('--gui', action='store_true',
                         help="show a live-plot GUI with Start/Stop buttons "
                              "instead of the CLI loop")
    args = parser.parse_args()

    if args.gui:
        run_gui(args.port)
    else:
        ser = connect_and_select_mode(args.port)
        run_cli(ser)


if __name__ == '__main__':
    main()
