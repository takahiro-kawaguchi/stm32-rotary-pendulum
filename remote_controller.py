#!/usr/bin/env python3
# /// script
# requires-python = ">=3.9"
# dependencies = [
#   "pyserial",
#   "matplotlib",
#   "numpy",
#   "scipy",
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
import os
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import serial
from scipy.linalg import solve_continuous_are
from scipy.signal import cont2discrete

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
# Controller gains — two-phase schedule, mirroring the MCU's own behaviour:
# angle_cal_update()'s i==1 branch (Src/app_runtime.c) injects an aggressive
# catch-phase controller right as the balance loop starts, then restores the
# softer PRIMARY/SECONDARY_*_MODE_1 macro gains (300/0/30 pend, 15.0/0.0/7.5
# rotor, no integral compensator) once its ~60s onboard angle-calibration
# process completes. Python doesn't run that calibration, so instead of
# matching its exact duration, CATCH_PHASE_DURATION_S below is chosen from
# our own logged catches, which settled to near-zero within ~5-7s.
#
# Settled on CATCH_GAINS (with the integral compensator on) after
# ablation-testing all three other combinations on hardware (2026-07-19):
#   plain macro gains, integral OFF  -> 2/4 catches
#   these high gains,  integral OFF  -> 1/3 catches, failing FASTER/harder
#                                       (theta_r ran to +233 deg in ~0.4s
#                                       both times, u pinned at the U_MAX
#                                       rail for a long stretch)
#   these high gains,  integral ON   -> 2/2 catches, theta_r stayed within
#                                       roughly -66..+20 deg (one hold ran 46s)
# So it's specifically the integral compensator — not gain magnitude — that
# keeps the rotor from drifting over repeated catch oscillations; without it,
# higher P/D gains alone made the rotor excursions worse, not better.
#
# STEADY_GAINS (macro defaults, no integral) is untested on its own so far —
# only ever run as part of the "plain macro gains, integral OFF" ablation
# above, which was evaluated as a *catch* controller, not as a steady-state
# holder after CATCH_GAINS has already stabilized the pendulum. Worth
# watching the first few sessions after this schedule ships in case the
# switch-down itself introduces a bump.
# ---------------------------------------------------------------------------
CATCH_PHASE_DURATION_S = 10.0

CATCH_GAINS = dict(Kp_pend=419.0, Kd_pend=56.0, Kp_rotor=21.1, Kd_rotor=17.2, Ki_comp=10.0)
STEADY_GAINS = dict(Kp_pend=300.0, Kd_pend=30.0, Kp_rotor=15.0, Kd_rotor=7.5, Ki_comp=0.0)

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
# Linearized pendulum model (Mode C, up-mode only), system-identified
# 2026-07-19 from real swing-up telemetry (scratch/identify_pendulum_model.py
# -- least-squares fit of the standard Furuta-pendulum equation of motion
# against ~93s / 9330 samples of logged theta_p, omega_p, omega_r, u_mcu;
# R^2 = 0.99). theta_r is omitted -- it doesn't feed back into the pendulum's
# own dynamics (only its rate, omega_r, does, via the centrifugal term below,
# which is dropped here since it vanishes under linearization around upright
# with the small omega_r typical of a balanced hold).
#
# state = [phi, phi_dot, omega_r]; phi = pendulum angle from upright [rad]
# input = u, commanded rotor angular acceleration [rad/s^2]
#   phi_ddot   = MODEL_W0_SQ*phi - MODEL_GAMMA*phi_dot + MODEL_K*u
#   omega_r_dot = u   (rotor kinematically follows u; see SwingUpController)
# ---------------------------------------------------------------------------
MODEL_W0_SQ = 48.0553   # rad^2/s^2  (T_down = 0.906 s at theta_p=0)
MODEL_GAMMA = 0.1757    # 1/s        (zeta = 0.013, lightly damped)
MODEL_K     = 0.4456    # rad/s^2 of phi_ddot per rad/s^2 of u, at theta_p=0
_MODEL_A = np.array([
    [0.0,         1.0,          0.0],
    [MODEL_W0_SQ, -MODEL_GAMMA, 0.0],
    [0.0,         0.0,          0.0],
])
_MODEL_B = np.array([[0.0], [MODEL_K], [1.0]])
_MODEL_Ad, _MODEL_Bd, *_ = cont2discrete(
    (_MODEL_A, _MODEL_B, np.eye(3), np.zeros((3, 1))), _TS, method='zoh')

# ---------------------------------------------------------------------------
# LQR balance controller (Mode C up-mode only, see LQRController below) --
# an alternative to DualPidController built on the same identified model,
# but the full 4-state [phi, theta_r, phi_dot, omega_r] this time (unlike
# the 3-state one-step predictor above, this one needs theta_r as a state
# to also regulate rotor drift, DualPidController's secondary objective).
#
# Q/R are derived from LQR_PARAMS via Bryson's rule (Q_ii = 1/x_i_max^2,
# R = 1/u_max^2) rather than tuned directly -- "largest excursion I'll
# tolerate" per state is a much easier dial to turn live from the GUI than
# an abstract weight. See scratch/lqr_design.py for the design derivation;
# recompute_lqr_gain() re-solves the continuous-time algebraic Riccati
# equation on demand (module load below, and the GUI's Apply button) since
# Q/R can change live, unlike MODEL_W0_SQ/GAMMA/K above (those come from
# system identification, not meant to be retuned at runtime).
# ---------------------------------------------------------------------------
LQR_PARAMS = dict(
    phi_max_deg=5.0, theta_r_max_deg=30.0,
    phi_dot_max_deg_s=120.0, omega_r_max_deg_s=300.0,
    # rad/s^2 -- a "typical" ceiling for Bryson's rule, distinct from U_MAX
    # below (the hardware's absolute output clip). Live-tuned down from an
    # initial 100 after real hardware testing (2026-07-19): higher values
    # gave a K aggressive enough to saturate and oscillate (see
    # OMEGA_GLITCH_CLAMP_DEG_S's docstring for one such divergence); u_max=3
    # is the first value confirmed to hold a stable, sustained inversion.
    u_max=3.0,
)
_LQR_A = np.array([
    [0.0,         0.0, 1.0,          0.0],
    [0.0,         0.0, 0.0,          1.0],
    [MODEL_W0_SQ, 0.0, -MODEL_GAMMA, 0.0],
    [0.0,         0.0, 0.0,          0.0],
])
_LQR_B = np.array([[0.0], [0.0], [MODEL_K], [1.0]])
_lqr_state = {'K': None}


def recompute_lqr_gain():
    """(Re)computes the LQR gain from the current LQR_PARAMS. Called once
    at import time below, and again by the GUI's Apply button whenever
    LQR_PARAMS changes."""
    p = LQR_PARAMS
    Q = np.diag([
        1.0 / math.radians(p['phi_max_deg']) ** 2,
        1.0 / math.radians(p['theta_r_max_deg']) ** 2,
        1.0 / math.radians(p['phi_dot_max_deg_s']) ** 2,
        1.0 / math.radians(p['omega_r_max_deg_s']) ** 2,
    ])
    R = np.array([[1.0 / p['u_max'] ** 2]])
    P = solve_continuous_are(_LQR_A, _LQR_B, Q, R)
    _lqr_state['K'] = np.linalg.solve(R, _LQR_B.T @ P)


recompute_lqr_gain()

# ---------------------------------------------------------------------------
# Kalman filter (steady-state, continuous-time LQE -- the dual of the LQR
# problem above) estimating LQRController's full state from theta_p/theta_r
# POSITION measurements only.
#
# Motivation: omega_p/omega_r are not independent sensor readings -- the
# firmware differentiates theta_p/theta_r internally to produce them. A
# single glitched sample in that firmware-side differentiation (see
# OMEGA_GLITCH_CLAMP_DEG_S above) once fed straight into LQRController and
# caused a saturation cascade; the clamp is a band-aid on the symptom. This
# observer instead treats only theta_p/theta_r as measured, and estimates
# phi_dot/omega_r from the identified model -- sensor noise/glitches get
# averaged out by the filter instead of amplified by raw differentiation.
#
# Same "physically-meaningful GUI-tunable knobs, not raw matrix entries"
# pattern as LQR_PARAMS/recompute_lqr_gain(): measurement-noise std (degrees)
# and process-noise std (deg/s) per channel, re-solved on demand via
#   solve_continuous_are(A^T, C^T, Q_e, R_e) -> P_e;  L = P_e @ C^T @ R_e^-1
# ---------------------------------------------------------------------------
_OBS_C = np.array([
    [1.0, 0.0, 0.0, 0.0],   # measures phi
    [0.0, 1.0, 0.0, 0.0],   # measures theta_r
])
OBSERVER_PARAMS = dict(
    phi_meas_noise_deg=0.1,              # pendulum encoder measurement noise std
    theta_r_meas_noise_deg=0.1,          # rotor position measurement noise std
    phi_dot_process_noise_deg_s=50.0,    # unmodeled disturbance std, phi_dot channel
    omega_r_process_noise_deg_s=200.0,   # unmodeled disturbance std, omega_r channel
)
_observer_state = {'L': None}


def recompute_observer_gain():
    """(Re)computes the Kalman observer gain from OBSERVER_PARAMS. Called
    once at import time below, and again by the GUI's Apply button."""
    p = OBSERVER_PARAMS
    Q_e = np.diag([
        1e-10, 1e-10,   # position states: no direct process noise -- driven
                        # purely by integrating the (uncertain) velocity states
        math.radians(p['phi_dot_process_noise_deg_s']) ** 2,
        math.radians(p['omega_r_process_noise_deg_s']) ** 2,
    ])
    R_e = np.diag([
        math.radians(p['phi_meas_noise_deg']) ** 2,
        math.radians(p['theta_r_meas_noise_deg']) ** 2,
    ])
    P_e = solve_continuous_are(_LQR_A.T, _OBS_C.T, Q_e, R_e)
    _observer_state['L'] = P_e @ _OBS_C.T @ np.linalg.inv(R_e)


recompute_observer_gain()

# ---------------------------------------------------------------------------
# Disturbance observer: KalmanObserver above, augmented with a 5th state, d --
# a "matched" input disturbance (enters the plant through the exact same B
# column as u, i.e. behaves exactly like an unmeasured extra acceleration
# command) assumed to vary slowly ("low frequency", per discussion 2026-07-19
# motivated by a real hardware symptom -- the rotor oscillating side to side
# during a hold, suspected model-mismatch such as stepper cogging torque or
# an unmodeled friction term the pure "omega_r_dot = u" kinematic assumption
# doesn't capture). Modeled as a simple first-order relaxation,
#   d_dot = -d/tau + w
# rather than a pure integrator (tau -> infinity): a finite tau keeps the
# augmented system's observability well away from colliding with omega_r's
# own free-integrator pole at eigenvalue 0, and gives an explicit "how slow
# is 'low frequency'" knob (OBSERVER_PARAMS['disturbance_tau_s']) instead of
# an implicit assumption baked into the math.
#
# LQRController cancels d_hat by subtracting it from its own commanded u
# before sending (see compute_from_state_with_disturbance()) -- since d
# enters identically to u, this is a direct feedforward cancellation, not a
# retuned gain.
#
# IMPORTANT identifiability caveat: because d is matched (same B column as
# u), the observer can only tell d apart from "the real state responding to
# u" by their different time constants -- this only works if tau is
# noticeably slower than the closed-loop LQR dynamics it's layered under
# (with the live-tuned LQR_PARAMS['u_max']=3.0 default, closed-loop poles
# are around -1.7..-7.0 rad/s, i.e. time constants of ~150ms-600ms).
# disturbance_tau_s should stay meaningfully larger than that (default 2s,
# checked numerically -- see scratch/check_dob*.py -- to keep the combined
# plant+observer+controller system's eigenvalues stable even when the real
# disturbance's own dynamics don't exactly match this relaxation model) or
# d_hat and the LQR will end up fighting over the same fast dynamics instead
# of splitting cleanly. If LQR_PARAMS is retuned to a much faster gain later,
# revisit this default too.
# ---------------------------------------------------------------------------
_DOB_C = np.array([
    [1.0, 0.0, 0.0, 0.0, 0.0],   # measures phi
    [0.0, 1.0, 0.0, 0.0, 0.0],   # measures theta_r
])
OBSERVER_PARAMS['disturbance_tau_s'] = 2.0                    # d's relaxation time constant [s]
OBSERVER_PARAMS['disturbance_process_noise_deg_s2'] = 50.0     # std of w driving d [deg/s^2]
_dob_state = {'L': None}


def recompute_dob_gain():
    """(Re)computes the 5-state disturbance-observer gain from
    OBSERVER_PARAMS. Called once at import time below, and again by the
    GUI's Apply button (same button as recompute_observer_gain(), since both
    read the same OBSERVER_PARAMS dict)."""
    p = OBSERVER_PARAMS
    alpha = 1.0 / p['disturbance_tau_s']
    A_dob = np.zeros((5, 5))
    A_dob[:4, :4] = _LQR_A
    A_dob[2, 4] = MODEL_K   # d enters phi_ddot exactly like u does (see _LQR_B)
    A_dob[3, 4] = 1.0       # d enters omega_r_dot exactly like u does
    A_dob[4, 4] = -alpha
    Q_e = np.diag([
        1e-10, 1e-10,
        math.radians(p['phi_dot_process_noise_deg_s']) ** 2,
        math.radians(p['omega_r_process_noise_deg_s']) ** 2,
        math.radians(p['disturbance_process_noise_deg_s2']) ** 2,
    ])
    R_e = np.diag([
        math.radians(p['phi_meas_noise_deg']) ** 2,
        math.radians(p['theta_r_meas_noise_deg']) ** 2,
    ])
    P_e = solve_continuous_are(A_dob.T, _DOB_C.T, Q_e, R_e)
    _dob_state['A'] = A_dob
    _dob_state['L'] = P_e @ _DOB_C.T @ np.linalg.inv(R_e)


recompute_dob_gain()

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

# Mode C only: GUI-tunable margin (degrees) for LinkManager.origin_mode's
# down/up switch -- how close to upright before Python starts treating "up"
# as its origin instead of "down". A dict (like SWINGUP_PARAMS below) so the
# GUI's Apply button can mutate it in place with no `global` needed; read
# directly by the background link thread, matching that same pattern.
# Independent of PEND_CAPTURE_DEG above (Mode B's actual capture-zone
# threshold) -- start narrow since this only drives a display switch for now
# and a wide margin would flip it well before the pendulum is genuinely near
# upright.
ORIGIN_MODE_THRESHOLD = dict(deg=10.0)

# Mode C only: hardware sanity-check pulse fired once at the very start of
# every session (LinkManager._startup_kick_done) -- a small, brief 'u' held
# for STARTUP_KICK_DURATION_S, before any origin_mode/capture logic runs,
# purely to see quickly (via theta_r in the log/plot) whether the rotor
# physically responds at all this session, independent of any control law.
STARTUP_KICK_U = 1500.0
STARTUP_KICK_DURATION_S = 0.15

# u is an acceleration, not a stop command -- dropping straight to u=0 after
# the kick above leaves the rotor coasting at whatever velocity it picked up
# instead of stopping, let alone returning to where it started. This phase
# runs right after the kick and drives the rotor back toward its start
# position with a small P(D) on theta_r/omega_r (both already in the
# telemetry, no extra derivative needed) instead of leaving it to wander.
STARTUP_RETURN_DURATION_S = 2.0
STARTUP_RETURN_KP = 300.0    # u per degree of rotor position error
STARTUP_RETURN_KD = 100.0    # u per (deg/s) of rotor velocity
STARTUP_RETURN_U_MAX = 8000.0

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
# Rotor reference (steps from centre) and feedforward gain, matching
# angle_cal_update()'s i==1 catch-phase setup (Src/app_runtime.c):
# rotor_position_command_steps=0, feedforward_gain=1. The integral
# compensator's own gain is scheduled per-phase — see CATCH_GAINS/
# STEADY_GAINS['Ki_comp'] above.
# ---------------------------------------------------------------------------
rotor_ref_steps = 0.0
FEEDFORWARD_GAIN = 1.0


def pid_execute(Kp, Ki, Kd, error, prev_error, state, lpf, ts):
    """One-step PID with first-order IIR LPF on derivative.
    Matches STM32 pid_execute() exactly. `state` is a mutable [prev_diff,
    prev_diff_filt] pair, updated in place."""
    a0, a1 = lpf
    diff      = Kd * (error - prev_error) / ts
    diff_filt = a0 * diff + a0 * state[0] - a1 * state[1]
    output    = Kp * error + Ki * ts * (error + prev_error) / 2.0 + diff_filt
    state[0]  = diff
    state[1]  = diff_filt
    return output


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
        self._catch_start_time = time.time()

    def reset(self):
        """Clears PID/filter state. Does NOT touch the catch-phase
        gain-schedule timer — this is also called at swing-up-start (well
        before any real telemetry/control begins), so restarting the timer
        here would let it expire during swing-up itself. Call start_catch()
        separately, exactly when actually entering/re-entering the capture
        zone."""
        self._pend_state  = [0.0, 0.0]
        self._rotor_state = [0.0, 0.0]
        self._prev_ep = 0.0
        self._prev_er = 0.0
        self._rotor_integral = 0.0

    def start_catch(self):
        """Marks 'now' as the start of a catch attempt for the gain-schedule
        timer. Call whenever actually entering/re-entering the capture zone
        (i.e. right before computing/sending real corrective u values)."""
        self._catch_start_time = time.time()

    def enter_capture(self, theta_p_deg: float, theta_r_deg: float):
        """Convenience for the common case of first entering/re-entering the
        capture zone: resets state, starts the gain-schedule timer, and
        primes the previous-error terms to the current position so the
        first derivative sample is zero instead of a startup spike.

        (2026-07-19: briefly removed this priming to match controller_init()
        on the MCU, which memset()s state to 0 rather than priming — that
        produces a large first-cycle derivative kick there. Shadow-logging
        confirmed the open-loop match, but real Mode B catches got *worse*
        without the priming, not better. Root cause turned out to be
        unrelated: a controlled Mode D experiment [decimating the MCU's own
        control-update rate to 100Hz with everything else identical] showed
        the real driver of Mode B's lower catch reliability is the 100Hz vs
        500Hz control-update-rate gap itself, not this startup transient.
        Restored the priming, which is the better choice for this
        architecture regardless of what the MCU's own cold-start does.)"""
        self.reset()
        self.start_catch()
        theta_p_rad = math.radians(theta_p_deg)
        theta_r_rad = math.radians(theta_r_deg)
        self._prev_ep = ENCODER_ANGLE_POLARITY * theta_p_rad / STEPPER_RAD_PER_STEP
        self._prev_er = theta_r_rad / STEPPER_RAD_PER_STEP

    def _pid_execute(self, Kp, Ki, Kd, error, prev_error, state, lpf):
        return pid_execute(Kp, Ki, Kd, error, prev_error, state, lpf, self._ts)

    def compute(self, theta_p_deg: float, theta_r_deg: float,
                rotor_ref: float = 0.0, force_steady: bool = False) -> float:
        """force_steady: skip the CATCH_GAINS phase and use STEADY_GAINS from
        the first sample — Mode C's up-mode control uses this; Mode B leaves
        it False to keep its existing (well-tuned) catch/steady schedule."""
        in_catch_phase = (not force_steady
                and (time.time() - self._catch_start_time) < CATCH_PHASE_DURATION_S)
        gains = CATCH_GAINS if in_catch_phase else STEADY_GAINS

        theta_p = math.radians(theta_p_deg)
        theta_r = math.radians(theta_r_deg)

        # Primary PID: pendulum
        e_p = ENCODER_ANGLE_POLARITY * theta_p / STEPPER_RAD_PER_STEP
        u_pend = self._pid_execute(
            gains['Kp_pend'], 0.0, gains['Kd_pend'],
            e_p, self._prev_ep, self._pend_state, _LP_PEND)
        self._prev_ep = e_p

        # Secondary PID: rotor (state-feedback mode: error = current position)
        theta_r_steps = theta_r / STEPPER_RAD_PER_STEP
        e_r = theta_r_steps - rotor_ref
        u_rotor = self._pid_execute(
            gains['Kp_rotor'], 0.0, gains['Kd_rotor'],
            e_r, self._prev_er, self._rotor_state, _LP_ROTOR)
        self._prev_er = e_r

        # Accumulating integral compensator (controller.c:126-133), active
        # whenever integral_compensator_gain != 0 — replaces, not adds to,
        # the plain feedforward term used when it's 0.
        #
        # Anti-windup (conditional integration, added 2026-07-19): only
        # commit this step's accumulation if the output isn't already
        # saturated in the same direction that accumulating would push it
        # further. controller.c's onboard equivalent has no anti-windup
        # either, but the MCU runs this at 500Hz vs Python's 100Hz, so any
        # windup it accumulates unwinds 5x faster in wall-clock time — a big
        # kick that pins u at U_MAX for many cycles let the integral wind up
        # for the whole saturated stretch here, overshooting once it finally
        # started to unwind.
        u_before_step = u_pend + u_rotor - gains['Ki_comp'] * self._rotor_integral
        integral_step = (rotor_ref * FEEDFORWARD_GAIN - theta_r_steps) * self._ts
        delta_u = -gains['Ki_comp'] * integral_step
        saturated_same_direction = (
            (u_before_step >= U_MAX and delta_u > 0) or
            (u_before_step <= -U_MAX and delta_u < 0)
        )
        if not saturated_same_direction:
            self._rotor_integral += integral_step

        u = u_pend + u_rotor - gains['Ki_comp'] * self._rotor_integral
        return max(-U_MAX, min(U_MAX, u))


# Sanity clamp for LQRController's omega_p/omega_r inputs (2026-07-19): a
# live catch diverged when a single glitched telemetry sample reported
# omega_p=15445 deg/s while theta_p had barely moved (max ever seen in real
# swing data was ~700 deg/s) -- likely a one-sample corruption in the
# firmware's own velocity filter. DualPidController never sees this because
# it derives its own (filtered) derivative from theta_p/theta_r instead of
# trusting the telemetry's omega fields; LQRController trusts them directly
# (matching how the model was identified/validated), so it needs its own
# guard. Well above any physically-plausible value, so this only rejects
# clear corruption, not real fast dynamics.
OMEGA_GLITCH_CLAMP_DEG_S = 1500.0


class LQRController:
    """Full-state-feedback LQR balance controller for Mode C's up-mode, an
    alternative to DualPidController built on the same identified model --
    see LQR_PARAMS/recompute_lqr_gain() above for the design.

    Stateless: unlike DualPidController's finite-differenced/IIR-filtered
    derivatives, this reads omega_p/omega_r straight from telemetry every
    call (matching how the model was identified and validated) instead of
    computing its own, aside from clamping clearly-impossible values (see
    OMEGA_GLITCH_CLAMP_DEG_S) -- so there's no internal filter state to
    reset or prime between catches. reset() and enter_capture() are no-ops
    kept only for interface parity with DualPidController, so LinkManager
    can treat either uniformly.
    """

    def reset(self):
        pass

    def enter_capture(self, theta_p_deg: float, theta_r_deg: float):
        pass

    def compute(self, theta_p_deg: float, theta_r_deg: float,
                omega_p_deg: float, omega_r_deg: float,
                rotor_ref: float = 0.0) -> float:
        """theta_p_deg: 0 = upright (phi). rotor_ref: steps, matching
        DualPidController's own convention (always 0 in practice -- see
        rotor_ref_steps below)."""
        omega_p_deg = max(-OMEGA_GLITCH_CLAMP_DEG_S, min(OMEGA_GLITCH_CLAMP_DEG_S, omega_p_deg))
        omega_r_deg = max(-OMEGA_GLITCH_CLAMP_DEG_S, min(OMEGA_GLITCH_CLAMP_DEG_S, omega_r_deg))
        x = np.array([
            math.radians(theta_p_deg),
            math.radians(theta_r_deg) - rotor_ref * STEPPER_RAD_PER_STEP,
            math.radians(omega_p_deg),
            math.radians(omega_r_deg),
        ])
        return self.compute_from_state(x)

    def compute_from_state(self, x: np.ndarray) -> float:
        """Same control law as compute() above, but takes an already-
        assembled state vector [phi, theta_r_rel, phi_dot, omega_r] (rad,
        rad, rad/s, rad/s) -- e.g. a KalmanObserver's estimate -- instead of
        raw telemetry. No glitch clamp here: a state-estimator's own
        measurement-noise model (OBSERVER_PARAMS) already bounds how much a
        single bad position sample can move the estimate, so there's nothing
        analogous to OMEGA_GLITCH_CLAMP_DEG_S to guard against."""
        u_rad_s2 = float(-(_lqr_state['K'] @ x)[0])
        u = u_rad_s2 / STEPPER_RAD_PER_STEP
        return max(-U_MAX, min(U_MAX, u))

    def compute_from_state_with_disturbance(self, x: np.ndarray, d_hat_rad_s2: float) -> float:
        """Same as compute_from_state(), plus feedforward cancellation of a
        DisturbanceObserver's matched-disturbance estimate: since d enters
        the plant through the exact same channel as u (see
        recompute_dob_gain()), subtracting it from the nominal LQR command
        makes the *actual* applied acceleration (u + d) track what the LQR
        law intended, independent of the disturbance."""
        u_rad_s2 = float(-(_lqr_state['K'] @ x)[0]) - d_hat_rad_s2
        u = u_rad_s2 / STEPPER_RAD_PER_STEP
        return max(-U_MAX, min(U_MAX, u))


class KalmanObserver:
    """Steady-state Kalman filter estimating LQRController's full state
    [phi, theta_r, phi_dot, omega_r] from theta_p/theta_r POSITION
    measurements only -- see OBSERVER_PARAMS/recompute_observer_gain() above.

    Stateful (unlike LQRController): x_hat persists across calls between
    enter_capture()/reset(). update() is a combined predict+correct step
    (continuous-time observer integrated with a simple Euler step at _TS),
    mirroring how the rest of this file assumes a fixed sample period rather
    than measuring actual wall-clock dt (see DualPidController._ts).
    """

    def __init__(self):
        self.x_hat = np.zeros(4)

    def reset(self):
        self.x_hat[:] = 0.0

    def enter_capture(self, theta_p_deg: float, theta_r_deg: float,
                       rotor_ref: float = 0.0):
        """Primes position states to the measured value and zeroes the
        velocity states -- start from a physically sane guess instead of
        carrying over a stale estimate from a previous catch."""
        self.x_hat[0] = math.radians(theta_p_deg)
        self.x_hat[1] = math.radians(theta_r_deg) - rotor_ref * STEPPER_RAD_PER_STEP
        self.x_hat[2] = 0.0
        self.x_hat[3] = 0.0

    def update(self, theta_p_deg: float, theta_r_deg: float,
               u_prev_rad_s2: float, rotor_ref: float = 0.0,
               dt: float = _TS) -> np.ndarray:
        """One filter step: predict with the identified model + the u that
        was actually commanded last cycle, correct against this cycle's
        theta_p/theta_r measurement. Returns the updated x_hat -- feed
        straight into LQRController.compute_from_state()."""
        y = np.array([
            math.radians(theta_p_deg),
            math.radians(theta_r_deg) - rotor_ref * STEPPER_RAD_PER_STEP,
        ])
        innovation = y - _OBS_C @ self.x_hat
        x_hat_dot = (_LQR_A @ self.x_hat + _LQR_B.flatten() * u_prev_rad_s2
                     + _observer_state['L'] @ innovation)
        self.x_hat = self.x_hat + x_hat_dot * dt
        return self.x_hat


class DisturbanceObserver:
    """Steady-state Kalman filter estimating an augmented state
    [phi, theta_r, phi_dot, omega_r, d] -- KalmanObserver's 4 states plus a
    slowly-relaxing matched input disturbance d -- from theta_p/theta_r
    POSITION measurements only. See the module comment above
    recompute_dob_gain() for the disturbance model and the identifiability
    caveat on OBSERVER_PARAMS['disturbance_tau_s'].

    Same update()-returns-x_hat / enter_capture() / reset() shape as
    KalmanObserver so LinkManager can treat either uniformly; x_hat here is
    length 5, with x_hat[4] the disturbance estimate (rad/s^2, same units as
    u) to feed into LQRController.compute_from_state_with_disturbance().
    """

    def __init__(self):
        self.x_hat = np.zeros(5)

    def reset(self):
        self.x_hat[:] = 0.0

    def enter_capture(self, theta_p_deg: float, theta_r_deg: float,
                       rotor_ref: float = 0.0):
        """Primes position states to the measured value; velocity AND
        disturbance start at zero -- an unproven disturbance estimate from a
        previous catch shouldn't carry into a fresh one."""
        self.x_hat[0] = math.radians(theta_p_deg)
        self.x_hat[1] = math.radians(theta_r_deg) - rotor_ref * STEPPER_RAD_PER_STEP
        self.x_hat[2] = 0.0
        self.x_hat[3] = 0.0
        self.x_hat[4] = 0.0

    def update(self, theta_p_deg: float, theta_r_deg: float,
               u_prev_rad_s2: float, rotor_ref: float = 0.0,
               dt: float = _TS) -> np.ndarray:
        """Same predict+correct structure as KalmanObserver.update(), over
        the 5-state augmented model. u_prev_rad_s2 only drives the plant
        rows (B is zero on the d row -- the disturbance evolves on its own,
        u doesn't feed it) -- see recompute_dob_gain()'s A_dob/_LQR_B."""
        y = np.array([
            math.radians(theta_p_deg),
            math.radians(theta_r_deg) - rotor_ref * STEPPER_RAD_PER_STEP,
        ])
        B5 = np.zeros(5)
        B5[:4] = _LQR_B.flatten()
        innovation = y - _DOB_C @ self.x_hat
        x_hat_dot = (_dob_state['A'] @ self.x_hat + B5 * u_prev_rad_s2
                     + _dob_state['L'] @ innovation)
        self.x_hat = self.x_hat + x_hat_dot * dt
        return self.x_hat


# ---------------------------------------------------------------------------
# Swing-up parameters (Mode C). Ported from the onboard bang-bang algorithm
# that Mode 1/A/B already run successfully (app_run_swing_up(),
# Src/app_session.c, plus the hardware_swing_up_*() zero-crossing/peak
# tracking it depends on, Src/hardware.c:207-246,295-336) instead of the
# earlier from-scratch PD energy-pump port (STM32pendulum's gui_tool
# branch), which had no known-good gains and proved very hard to tune from
# zero after hours of real testing.
#
# The firmware version drives the rotor with BSP_MotorControl_Move(), which
# precomputes a full accel/cruise/decel speed profile sized so velocity is
# already back to exactly 0 the instant it reaches the target step count
# (Drivers/BSP/Components/l6474/l6474.c:858, L6474_ComputeSpeedProfile()) --
# an open-loop trajectory, not a feedback loop. This script only has an
# acceleration command (u), so the closest equivalent is a real closed-loop
# rotor-position PID (rotor_kp/ki/kd below) that runs continuously and holds
# at whatever target the zero-crossing logic last set -- the first version
# here instead handed off to u=0 once "close enough" to the target, which
# left the rotor coasting well past it under its own momentum (same root
# cause as needing STARTUP_RETURN_* after STARTUP_KICK). A continuously
# running PID brakes and holds instead of ever going open-loop.
#
# stage*_deg / *_threshold_deg are the firmware's STAGE_0/1/2_AMP (200/130/
# 120 steps, plus the initial 150-step priming push) and 600/1000-count
# amplitude thresholds, converted to degrees via STEPPER_RAD_PER_STEP and
# ENCODER_READ_ANGLE_SCALE=6.666667 counts/deg (Inc/edukit_system.h) -- real
# values already proven on this hardware. rotor_kp/ki/kd/rotor_u_max had
# nothing to port from; rotor_kp=1500/rotor_u_max=20000 (2026-07-19) is the
# first combination that reliably swung all the way up on real hardware --
# a faster/firmer rotor response was needed so the software move keeps up
# with L6474_ComputeSpeedProfile()'s near-instant one as swing amplitude
# (and therefore bottom-crossing speed) grows; weaker gains plateaued around
# +-120 deg instead of reaching upright.
# ---------------------------------------------------------------------------
SWINGUP_PARAMS = dict(
    stage0_deg=22.5, stage1_deg=14.625, stage2_deg=13.5,
    stage1_threshold_deg=90.0, stage2_threshold_deg=150.0,
    rotor_kp=1500.0, rotor_ki=50.0, rotor_kd=150.0, rotor_u_max=20000.0,
)


class SwingUpController:
    """Bang-bang swing-up policy for Mode C, ported from the onboard
    algorithm (see SWINGUP_PARAMS comment above). Active while origin_mode
    is 'down'; LinkManager hands off to DualPidController once origin_mode
    flips to 'up'.

    Unlike everywhere else in this file, compute()'s theta_p_deg is 0 = hang
    down (matching the onboard swing-up telemetry's own convention, see
    Src/app_session.c's dedicated print block) rather than the usual 0 =
    upright. LinkManager passes it straight through unconverted.

    Every sample does two independent things:
      - A rotor-position PID continuously drives/holds the rotor at
        self._rotor_target_deg (updated below, not a one-shot burst).
      - theta_p is watched for a zero-crossing (pendulum passing back
        through hang-down -- bumps the target by the current stage amplitude
        in the current direction) or a peak (this half-swing's amplitude
        stopped growing -- flips the direction for the next zero-crossing).
    """

    def __init__(self):
        self._ts = _TS
        self.reset()

    def reset(self):
        self._direction = 1.0     # +1/-1, flips on each detected peak
        self._stage_amp_deg = SWINGUP_PARAMS['stage0_deg']
        self._stage_count = 0
        self._prev_theta_p = None
        self._peaked = False
        self._handled_peak = False
        self._max_deg = 0.0            # "still growing since last peak handled" tracker
        self._global_max_deg = 0.0     # this half-swing's largest |theta_p|
        self._prev_global_max_deg = 0.0
        self._rotor_target_deg = None  # set on first compute() call below
        self._rotor_integral = 0.0

    def compute(self, theta_p_deg: float, theta_r_deg: float,
                omega_r_deg: float = 0.0) -> float:
        """theta_p_deg: 0 = hang down (see class docstring)."""
        p = SWINGUP_PARAMS

        if self._rotor_target_deg is None:
            # Mirrors app_run_swing_up()'s initial BSP_MotorControl_Move(0,
            # FORWARD, 150) priming push before its own main loop starts --
            # without this the pendulum just hangs at rest and no
            # zero-crossing ever happens.
            self._rotor_target_deg = theta_r_deg + self._direction * 16.875
        if self._prev_theta_p is None:
            self._prev_theta_p = theta_p_deg

        # hardware_encoder_position_read()'s zero-crossing/peak bookkeeping
        # (Src/hardware.c:226-241), sampled at Python's 100Hz instead of the
        # MCU's 500Hz -- fine here since a swing half-period is hundreds of
        # ms. Runs every sample now (not gated behind a "move in progress"
        # phase) since the rotor PID below never goes open-loop.
        zero_crossed = (theta_p_deg > 0) != (self._prev_theta_p > 0)
        if zero_crossed:
            self._peaked = False

        if not self._peaked:
            if abs(theta_p_deg) >= abs(self._global_max_deg):
                self._global_max_deg = theta_p_deg
            if abs(theta_p_deg) >= abs(self._max_deg):
                self._max_deg = theta_p_deg
            else:
                self._peaked = True
                self._handled_peak = False

        self._prev_theta_p = theta_p_deg

        if zero_crossed:
            self._stage_count += 1
            if (self._prev_global_max_deg != self._global_max_deg
                    and self._stage_count > 4):
                amp = abs(self._global_max_deg)
                if amp < p['stage1_threshold_deg']:
                    self._stage_amp_deg = p['stage0_deg']
                elif amp < p['stage2_threshold_deg']:
                    self._stage_amp_deg = p['stage1_deg']
                else:
                    self._stage_amp_deg = p['stage2_deg']
            self._prev_global_max_deg = self._global_max_deg
            self._global_max_deg = 0.0
            self._rotor_target_deg = theta_r_deg + self._direction * self._stage_amp_deg

        if self._peaked and not self._handled_peak:
            self._handled_peak = True
            self._max_deg = 0.0
            self._direction = -self._direction

        # Rotor-position PID, continuously driving/holding at
        # self._rotor_target_deg. Anti-windup (conditional integration,
        # same pattern as DualPidController.compute()): only accumulate
        # while not already saturated in the direction that would push
        # further.
        error = theta_r_deg - self._rotor_target_deg
        u_before_step = (-p['rotor_kp'] * error - p['rotor_kd'] * omega_r_deg
                          - p['rotor_ki'] * self._rotor_integral)
        integral_step = error * self._ts
        delta_u = -p['rotor_ki'] * integral_step
        saturated_same_direction = (
            (u_before_step >= p['rotor_u_max'] and delta_u > 0) or
            (u_before_step <= -p['rotor_u_max'] and delta_u < 0)
        )
        if not saturated_same_direction:
            self._rotor_integral += integral_step

        u = (-p['rotor_kp'] * error - p['rotor_kd'] * omega_r_deg
             - p['rotor_ki'] * self._rotor_integral)
        return max(-p['rotor_u_max'], min(p['rotor_u_max'], u))


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
                ctrl.enter_capture(theta_p, theta_r)
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
        # GUI plot series: pendulum angle (Mode C: theta_p_perceived --
        # whichever origin Python currently treats as 0, see _step_running()
        # -- other modes: theta_p as-is), theta_r, u.
        self.data = [deque(maxlen=MAX_POINTS) for _ in range(3)]
        self.ctrl = DualPidController()
        self.lqr_ctrl = LQRController()
        # Mode C up-mode only: which balance controller is currently active,
        # 'pid' (DualPidController) or 'lqr' (LQRController) -- see the GUI's
        # controller radio buttons. Switchable live; DualPidController may
        # take a cycle to settle after switching back to it since it wasn't
        # being called (and its derivative-filter state wasn't updating)
        # while LQRController was active.
        self.balance_controller = 'pid'
        # LQR-only: where LQRController's state vector comes from --
        # 'telemetry' (theta_p/theta_r/omega_p/omega_r straight off the wire,
        # clamped -- see OMEGA_GLITCH_CLAMP_DEG_S), 'observer' (KalmanObserver's
        # filtered estimate, theta_p/theta_r position-only), or 'observer_dob'
        # (DisturbanceObserver -- same, plus a matched-disturbance estimate
        # fed forward to cancel a suspected low-frequency model-mismatch
        # term, e.g. stepper cogging, behind the rotor oscillating side to
        # side during a hold -- see DisturbanceObserver's docstring).
        self.lqr_state_source = 'telemetry'
        self.observer = KalmanObserver()
        self.dob = DisturbanceObserver()
        # u actually commanded last cycle (steps/s^2, same units this file
        # sends over serial) -- KalmanObserver/DisturbanceObserver.update()
        # need this as their predict-step input, since it isn't in the
        # telemetry line itself.
        self._last_u_python = 0.0
        self.swing_ctrl = SwingUpController()
        self.in_balance = False
        # Mode 1 only: DualPidController run in parallel, never sent, purely
        # to log what Mode B's control law would have done at each sample
        # for direct comparison against Mode 1's actual (MCU-computed) u —
        # see u_python column in the CSV log during a Mode 1 session.
        self.shadow_ctrl = DualPidController()
        self.shadow_in_balance = False
        self._reset_pending = False
        # Mode to start when `start_requested` fires. 'B' = PC computes u
        # once caught (DualPidController, active_control=True), onboard
        # bang-bang swing-up first. 'C' = PC also drives swing-up from
        # hang-down (SwingUpController, active_control=True). '1' = MCU's own
        # onboard PID runs the whole session (active_control=False, GUI only
        # observes the telemetry the firmware already reports for every mode).
        self.selected_mode = 'B'
        self.active_control = True
        # Mode C only: which origin frame the GUI should currently show,
        # 'down' (0 = hang down) or 'up' (0 = upright) -- see _step_running().
        self.origin_mode = 'down'
        # Mode C up-mode only: MODEL_A/B's one-step-ahead prediction of this
        # cycle's phi (made last cycle from last cycle's state+u), logged
        # alongside the actual value to validate the identified model live
        # instead of only offline. None whenever there's no prediction to
        # compare yet (mode just entered, or not in up-mode).
        self._model_pred_phi_deg = None
        # Mode C only: fires a short, fixed test pulse at the very start of
        # each session (see _step_running()) to quickly confirm the rotor
        # physically responds at all in *this* session, independent of
        # origin_mode/capture logic -- a hardware sanity check to rule in/out
        # the driver before trusting a longer swing-up attempt.
        self._startup_kick_done = True
        self._startup_kick_start = None
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
                                    'omega_p_deg_s', 'omega_r_deg_s', 'u_mcu', 'u_python',
                                    'theta_p_perceived_deg', 'origin_mode', 'model_pred_phi_deg',
                                    'phi_hat_deg', 'theta_r_hat_deg', 'phi_dot_hat_deg_s', 'omega_r_hat_deg_s',
                                    'd_hat_deg_s2'])
        self._log_start_time = time.time()
        print(f"Logging to {path}")

    def _close_log(self):
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None
            self._log_writer = None

    def _log_sample(self, i_idx, theta_p, theta_r, omega_p, omega_r, u_mcu, u_python,
                     theta_p_perceived=None, origin_mode=None, model_pred_phi_deg=None,
                     phi_hat_deg=None, theta_r_hat_deg=None, phi_dot_hat_deg=None, omega_r_hat_deg=None,
                     d_hat_deg_s2=None):
        if self._log_writer is None:
            return
        t = time.time() - self._log_start_time
        fmt = lambda v: '' if v is None else f"{v:.4f}"
        self._log_writer.writerow([f"{t:.3f}", i_idx, theta_p, theta_r, omega_p, omega_r,
                                    u_mcu, '' if u_python is None else f"{u_python:.1f}",
                                    '' if theta_p_perceived is None else theta_p_perceived,
                                    '' if origin_mode is None else origin_mode,
                                    '' if model_pred_phi_deg is None else f"{model_pred_phi_deg:.3f}",
                                    fmt(phi_hat_deg), fmt(theta_r_hat_deg),
                                    fmt(phi_dot_hat_deg), fmt(omega_r_hat_deg), fmt(d_hat_deg_s2)])

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
        print(line)  # otherwise nothing is visible while waiting for reboot
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
            self.active_control = self.selected_mode in ('B', 'C')
            self.ser.write((self.selected_mode + '\r').encode())
            self.ctrl = DualPidController()
            self.lqr_ctrl.reset()
            self.observer.reset()
            self.dob.reset()
            self._last_u_python = 0.0
            self.swing_ctrl.reset()
            self.in_balance = False
            self.origin_mode = 'down'
            self._model_pred_phi_deg = None
            self._startup_kick_done = self.selected_mode != 'C'
            self._startup_kick_start = None
            self.shadow_ctrl = DualPidController()
            self.shadow_in_balance = False
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

        if self.selected_mode == 'C':
            if not self._startup_kick_done:
                if self._startup_kick_start is None:
                    self._startup_kick_start = time.time()
                elapsed = time.time() - self._startup_kick_start

                if elapsed < STARTUP_KICK_DURATION_S:
                    u = STARTUP_KICK_U
                elif elapsed < STARTUP_KICK_DURATION_S + STARTUP_RETURN_DURATION_S:
                    u = max(-STARTUP_RETURN_U_MAX, min(STARTUP_RETURN_U_MAX,
                            -STARTUP_RETURN_KP * theta_r - STARTUP_RETURN_KD * omega_r))
                else:
                    u = 0.0
                    self._startup_kick_done = True

                self.ser.write(f'u {u:.1f}\r'.encode())
                self._log_sample(i_idx, theta_p, theta_r, omega_p, omega_r, u_prev,
                                  u, theta_p, self.origin_mode)
                self.data[0].append(theta_p)
                self.data[1].append(theta_r)
                self.data[2].append(u)
                return

            # Mode C: firmware always reports theta_p in the same "0 = hang
            # down" convention app_run_swing_up()'s own telemetry uses (Src/
            # app_session.c) -- it never tries to guess which side is
            # upright (see report_telemetry(), Src/app_runtime.c). Derive
            # the "0 = upright" value purely to classify which origin frame
            # we're currently in, valid regardless of which side the
            # pendulum approached from.
            theta_p_upright = theta_p - 180.0 if theta_p >= 0 else theta_p + 180.0
            self.origin_mode = 'up' if abs(theta_p_upright) <= ORIGIN_MODE_THRESHOLD['deg'] else 'down'
            # What Python is currently treating as "0" is the *current*
            # mode's origin, not always upright: theta_p itself (0 = down)
            # while in down-mode, theta_p_upright (0 = up) once switched to
            # up-mode. This intentionally jumps by ~180 deg at the exact
            # moment origin_mode flips -- that jump is the expected signature
            # of the origin switch, not a bug.
            theta_p_perceived = theta_p if self.origin_mode == 'down' else theta_p_upright

            # Down-mode: SwingUpController pumps energy in, fed theta_p
            # directly (0 = down, exactly what it expects -- see its
            # docstring). Up-mode: whichever of DualPidController/
            # LQRController self.balance_controller currently selects, fed
            # theta_p_upright so both controllers' "0 = upright" assumptions
            # hold regardless of which side the pendulum approached from.
            active_ctrl = self.lqr_ctrl if self.balance_controller == 'lqr' else self.ctrl
            phi_dot_hat_deg = theta_r_hat_deg = phi_hat_deg = omega_r_hat_deg = d_hat_deg_s2 = None
            if abs(theta_r) > ROTOR_LIMIT_DEG:
                u = 0.0
                self.in_balance = False
                self.ctrl.reset()
                self.lqr_ctrl.reset()
                self.observer.reset()
                self.dob.reset()
            elif self.origin_mode == 'up':
                if not self.in_balance:
                    active_ctrl.enter_capture(theta_p_upright, theta_r)
                    if self.balance_controller == 'lqr' and self.lqr_state_source == 'observer':
                        self.observer.enter_capture(theta_p_upright, theta_r, rotor_ref_steps)
                    elif self.balance_controller == 'lqr' and self.lqr_state_source == 'observer_dob':
                        self.dob.enter_capture(theta_p_upright, theta_r, rotor_ref_steps)
                    self.in_balance = True
                if self.balance_controller == 'lqr':
                    if self.lqr_state_source == 'observer':
                        u_prev_rad_s2 = self._last_u_python * STEPPER_RAD_PER_STEP
                        x_hat = self.observer.update(theta_p_upright, theta_r, u_prev_rad_s2, rotor_ref_steps)
                        u = self.lqr_ctrl.compute_from_state(x_hat)
                        phi_hat_deg, theta_r_hat_deg = math.degrees(x_hat[0]), math.degrees(x_hat[1])
                        phi_dot_hat_deg, omega_r_hat_deg = math.degrees(x_hat[2]), math.degrees(x_hat[3])
                    elif self.lqr_state_source == 'observer_dob':
                        u_prev_rad_s2 = self._last_u_python * STEPPER_RAD_PER_STEP
                        x_hat = self.dob.update(theta_p_upright, theta_r, u_prev_rad_s2, rotor_ref_steps)
                        u = self.lqr_ctrl.compute_from_state_with_disturbance(x_hat[:4], x_hat[4])
                        phi_hat_deg, theta_r_hat_deg = math.degrees(x_hat[0]), math.degrees(x_hat[1])
                        phi_dot_hat_deg, omega_r_hat_deg = math.degrees(x_hat[2]), math.degrees(x_hat[3])
                        d_hat_deg_s2 = math.degrees(x_hat[4])
                    else:
                        u = self.lqr_ctrl.compute(theta_p_upright, theta_r, omega_p, omega_r, rotor_ref_steps)
                else:
                    u = self.ctrl.compute(theta_p_upright, theta_r, rotor_ref_steps, force_steady=True)
            else:
                if self.in_balance:
                    # Was catching, escaped back down -- clear catch state
                    # and start the swing controller fresh instead of
                    # resuming whatever stale derivative state it had from
                    # before capture.
                    self.in_balance = False
                    self.ctrl.reset()
                    self.lqr_ctrl.reset()
                    self.observer.reset()
                    self.dob.reset()
                    self.swing_ctrl.reset()
                u = self.swing_ctrl.compute(theta_p, theta_r, omega_r)
            self._last_u_python = u

            # Up-mode only: one-step-ahead prediction from MODEL_A/B/_MODEL_Ad
            # (Bd), logged so it can be compared against the actual next
            # sample offline -- live validation of the identified model.
            # model_pred_deg holds the prediction *made last cycle* for THIS
            # cycle (None on the first up-mode sample, nothing to compare
            # yet); a fresh prediction for the next cycle is computed right
            # after using this cycle's own state and u.
            if self.origin_mode == 'up':
                model_pred_deg = self._model_pred_phi_deg
                x_k = np.array([[math.radians(theta_p_upright)],
                                 [math.radians(omega_p)],
                                 [math.radians(omega_r)]])
                x_next = _MODEL_Ad @ x_k + _MODEL_Bd * (u * STEPPER_RAD_PER_STEP)
                self._model_pred_phi_deg = math.degrees(x_next[0, 0])
            else:
                model_pred_deg = None
                self._model_pred_phi_deg = None

            self.ser.write(f'u {u:.1f}\r'.encode())
            self._log_sample(i_idx, theta_p, theta_r, omega_p, omega_r, u_prev, u,
                              theta_p_perceived, self.origin_mode, model_pred_deg,
                              phi_hat_deg, theta_r_hat_deg, phi_dot_hat_deg, omega_r_hat_deg,
                              d_hat_deg_s2)
            self.data[0].append(theta_p_perceived)
            self.data[1].append(theta_r)
            self.data[2].append(u)
            return

        if not self.active_control:
            # Mode 1: the MCU's own onboard PID runs the session — send
            # nothing back, but also run Mode B's control law in shadow
            # (same capture/rotor-limit logic, never sent to the MCU) purely
            # to log what it would have done here, for direct comparison
            # against Mode 1's actual behaviour.
            if abs(theta_r) > ROTOR_LIMIT_DEG:
                u_shadow = 0.0
            elif abs(theta_p) > PEND_CAPTURE_DEG:
                self.shadow_in_balance = False
                self.shadow_ctrl.reset()
                u_shadow = None
            else:
                if not self.shadow_in_balance:
                    self.shadow_ctrl.enter_capture(theta_p, theta_r)
                    self.shadow_in_balance = True
                u_shadow = self.shadow_ctrl.compute(theta_p, theta_r, rotor_ref_steps)

            self._log_sample(i_idx, theta_p, theta_r, omega_p, omega_r, u_prev, u_shadow)
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
            self.ctrl.enter_capture(theta_p, theta_r)
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

    # matplotlib's default font (DejaVu Sans) has no CJK glyphs, so the plot
    # legend's Japanese labels would render as tofu boxes (with a "Glyph ...
    # missing" warning) without this. These are common preinstalled Windows
    # fonts; matplotlib silently skips whichever aren't present.
    plt.rcParams['font.family'] = ['Yu Gothic', 'Meiryo', 'MS Gothic', 'sans-serif']

    try:
        ser = serial.Serial(port, BAUD, timeout=0.5)
    except serial.SerialException as e:
        print(f"Cannot open {port}: {e}")
        sys.exit(1)

    print(f"Connected: {port}  {BAUD} baud")

    link = LinkManager(ser)
    link_thread = threading.Thread(target=link.run, daemon=True)
    link_thread.start()

    root = tk.Tk()
    root.title("STM32 Pendulum — Remote Controller")
    root.geometry("900x650")

    top = ttk.Frame(root, padding=8)
    top.pack(fill="x")

    status_var = tk.StringVar(value=link.status_text)
    ttk.Label(top, textvariable=status_var).pack(side="left")

    # Mode C only: shows which origin frame _step_running() currently thinks
    # the pendulum is in (see LinkManager.origin_mode) -- lets the "0 = down
    # -> 0 = upright" frame switch be verified visually before any control
    # law is reconnected.
    origin_mode_var = tk.StringVar(value="原点: —")
    origin_mode_label = ttk.Label(top, textvariable=origin_mode_var, font=("", 10, "bold"))
    origin_mode_label.pack(side="left", padx=(16, 0))

    ttk.Label(top, text="真上しきい値[deg]").pack(side="left", padx=(16, 2))
    origin_threshold_var = tk.DoubleVar(value=ORIGIN_MODE_THRESHOLD['deg'])
    ttk.Entry(top, textvariable=origin_threshold_var, width=6).pack(side="left")

    def apply_origin_threshold():
        try:
            ORIGIN_MODE_THRESHOLD['deg'] = origin_threshold_var.get()
        except tk.TclError:
            pass  # invalid entry text — leave the threshold unchanged

    ttk.Button(top, text="適用", command=apply_origin_threshold).pack(side="left", padx=(2, 0))

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
    mode_c_radio = ttk.Radiobutton(mode_frame, text="Mode C (Python swing-up)",
                                    variable=mode_var, value='C', command=on_mode_change)
    mode_1_radio = ttk.Radiobutton(mode_frame, text="Mode 1 (MCU内蔵PID)",
                                    variable=mode_var, value='1', command=on_mode_change)
    mode_d_radio = ttk.Radiobutton(mode_frame, text="Mode D (MCU@100Hz診断)",
                                    variable=mode_var, value='D', command=on_mode_change)
    mode_b_radio.pack(side="left")
    mode_c_radio.pack(side="left")
    mode_1_radio.pack(side="left")
    mode_d_radio.pack(side="left")

    # --- Swing-up parameter panel (Mode C) — live-tunable, no restart needed ---
    swingup_frame = ttk.LabelFrame(root, text="Swing-up params (Mode C)", padding=6)
    swingup_frame.pack(fill="x", padx=8, pady=(0, 8))

    swingup_vars = {key: tk.DoubleVar(value=val) for key, val in SWINGUP_PARAMS.items()}
    swingup_labels = {
        'stage0_deg': 'Stage0[deg]', 'stage1_deg': 'Stage1[deg]', 'stage2_deg': 'Stage2[deg]',
        'stage1_threshold_deg': 'しきい値1[deg]', 'stage2_threshold_deg': 'しきい値2[deg]',
        'rotor_kp': 'Rotor Kp', 'rotor_ki': 'Rotor Ki', 'rotor_kd': 'Rotor Kd',
        'rotor_u_max': 'Rotor U上限',
    }
    row0_keys = ('stage0_deg', 'stage1_deg', 'stage2_deg', 'stage1_threshold_deg', 'stage2_threshold_deg')
    row1_keys = ('rotor_kp', 'rotor_ki', 'rotor_kd', 'rotor_u_max')
    for row, keys in enumerate((row0_keys, row1_keys)):
        for i, key in enumerate(keys):
            ttk.Label(swingup_frame, text=swingup_labels[key]).grid(row=row, column=2 * i, padx=(4, 2), sticky="e")
            ttk.Entry(swingup_frame, textvariable=swingup_vars[key], width=8).grid(row=row, column=2 * i + 1, padx=(0, 8))

    def apply_swingup_params():
        for key, var in swingup_vars.items():
            try:
                SWINGUP_PARAMS[key] = var.get()
            except tk.TclError:
                pass  # invalid entry text — leave that param unchanged

    ttk.Button(swingup_frame, text="適用", command=apply_swingup_params).grid(row=0, column=10, rowspan=2, padx=8)

    # --- Balance controller selector + LQR parameter panel (Mode C up-mode) ---
    # Switchable live (see LinkManager.balance_controller); Q/R re-derived
    # via Bryson's rule from these bounds (recompute_lqr_gain()) rather than
    # tuned directly, matching SWINGUP_PARAMS' live-tunable-with-no-restart
    # pattern above.
    balance_frame = ttk.LabelFrame(root, text="Balance controller (Mode C up-mode)", padding=6)
    balance_frame.pack(fill="x", padx=8, pady=(0, 8))

    balance_ctrl_var = tk.StringVar(value=link.balance_controller)

    def on_balance_ctrl_change():
        link.balance_controller = balance_ctrl_var.get()

    ttk.Radiobutton(balance_frame, text="PID (DualPidController)", variable=balance_ctrl_var,
                     value='pid', command=on_balance_ctrl_change).grid(row=0, column=0, columnspan=4, sticky="w")
    ttk.Radiobutton(balance_frame, text="LQR", variable=balance_ctrl_var,
                     value='lqr', command=on_balance_ctrl_change).grid(row=0, column=4, columnspan=2, sticky="w")

    lqr_vars = {key: tk.DoubleVar(value=val) for key, val in LQR_PARAMS.items()}
    lqr_labels = {
        'phi_max_deg': 'phi上限[deg]', 'theta_r_max_deg': 'theta_r上限[deg]',
        'phi_dot_max_deg_s': 'phi_dot上限[deg/s]', 'omega_r_max_deg_s': 'omega_r上限[deg/s]',
        'u_max': 'u上限[rad/s²]',
    }
    lqr_keys = ('phi_max_deg', 'theta_r_max_deg', 'phi_dot_max_deg_s', 'omega_r_max_deg_s', 'u_max')
    for i, key in enumerate(lqr_keys):
        ttk.Label(balance_frame, text=lqr_labels[key]).grid(row=1, column=2 * i, padx=(4, 2), sticky="e")
        ttk.Entry(balance_frame, textvariable=lqr_vars[key], width=8).grid(row=1, column=2 * i + 1, padx=(0, 8))

    def apply_lqr_params():
        for key, var in lqr_vars.items():
            try:
                LQR_PARAMS[key] = var.get()
            except tk.TclError:
                pass  # invalid entry text — leave that param unchanged
        recompute_lqr_gain()

    ttk.Button(balance_frame, text="適用", command=apply_lqr_params).grid(row=1, column=10, padx=8)

    # LQR only: where the state vector fed into LQRController comes from.
    lqr_source_var = tk.StringVar(value=link.lqr_state_source)

    def on_lqr_source_change():
        link.lqr_state_source = lqr_source_var.get()

    ttk.Radiobutton(balance_frame, text="状態: テレメトリ直接(+グリッチクランプ)", variable=lqr_source_var,
                     value='telemetry', command=on_lqr_source_change).grid(row=2, column=0, columnspan=5, sticky="w")
    ttk.Radiobutton(balance_frame, text="状態: Kalman Observer", variable=lqr_source_var,
                     value='observer', command=on_lqr_source_change).grid(row=2, column=5, columnspan=3, sticky="w")
    ttk.Radiobutton(balance_frame, text="状態: Kalman Observer + 外乱推定", variable=lqr_source_var,
                     value='observer_dob', command=on_lqr_source_change).grid(row=3, column=0, columnspan=5, sticky="w")

    # --- Kalman observer parameter panel (LQR state source = observer /
    # observer_dob). Same OBSERVER_PARAMS dict backs both -- row 0 is
    # KalmanObserver's own noise knobs, row 1 is DisturbanceObserver's extra
    # disturbance-channel knobs (only used when observer_dob is selected).
    observer_frame = ttk.LabelFrame(root, text="Kalman Observer / 外乱オブザーバ (LQR用状態推定)", padding=6)
    observer_frame.pack(fill="x", padx=8, pady=(0, 8))

    observer_vars = {key: tk.DoubleVar(value=val) for key, val in OBSERVER_PARAMS.items()}
    observer_labels = {
        'phi_meas_noise_deg': '測定雑音 phi[deg]',
        'theta_r_meas_noise_deg': '測定雑音 theta_r[deg]',
        'phi_dot_process_noise_deg_s': 'プロセス雑音 phi_dot[deg/s]',
        'omega_r_process_noise_deg_s': 'プロセス雑音 omega_r[deg/s]',
        'disturbance_tau_s': '外乱時定数 τ[s]',
        'disturbance_process_noise_deg_s2': '外乱プロセス雑音[deg/s²]',
    }
    observer_keys = ('phi_meas_noise_deg', 'theta_r_meas_noise_deg',
                      'phi_dot_process_noise_deg_s', 'omega_r_process_noise_deg_s')
    dob_keys = ('disturbance_tau_s', 'disturbance_process_noise_deg_s2')
    for i, key in enumerate(observer_keys):
        ttk.Label(observer_frame, text=observer_labels[key]).grid(row=0, column=2 * i, padx=(4, 2), sticky="e")
        ttk.Entry(observer_frame, textvariable=observer_vars[key], width=8).grid(row=0, column=2 * i + 1, padx=(0, 8))
    for i, key in enumerate(dob_keys):
        ttk.Label(observer_frame, text=observer_labels[key]).grid(row=1, column=2 * i, padx=(4, 2), sticky="e")
        ttk.Entry(observer_frame, textvariable=observer_vars[key], width=8).grid(row=1, column=2 * i + 1, padx=(0, 8))

    def apply_observer_params():
        for key, var in observer_vars.items():
            try:
                OBSERVER_PARAMS[key] = var.get()
            except tk.TclError:
                pass  # invalid entry text — leave that param unchanged
        recompute_observer_gain()
        recompute_dob_gain()

    ttk.Button(observer_frame, text="適用", command=apply_observer_params).grid(row=0, column=10, rowspan=2, padx=8)

    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(7, 6))
    # Pendulum Angle shows theta_p_perceived (Mode C: whichever origin Python
    # currently treats as 0, see _step_running() -- jumps by ~180 deg at the
    # exact moment origin_mode flips, which is expected, not a bug).
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

    tick_after_id = None

    def tick():
        nonlocal tick_after_id
        status_var.set(link.status_text)
        at_prompt = link.state == STATE_AT_PROMPT
        start_btn.state(['!disabled'] if at_prompt else ['disabled'])
        stop_btn.state(['!disabled'] if link.state == STATE_RUNNING else ['disabled'])
        mode_b_radio.state(['!disabled'] if at_prompt else ['disabled'])
        mode_c_radio.state(['!disabled'] if at_prompt else ['disabled'])
        mode_1_radio.state(['!disabled'] if at_prompt else ['disabled'])
        mode_d_radio.state(['!disabled'] if at_prompt else ['disabled'])

        if link.selected_mode == 'C' and link.state == STATE_RUNNING:
            if link.origin_mode == 'up':
                origin_mode_var.set("原点: 真上")
                origin_mode_label.configure(foreground="blue")
            else:
                origin_mode_var.set("原点: 真下")
                origin_mode_label.configure(foreground="black")
        else:
            origin_mode_var.set("原点: —")
            origin_mode_label.configure(foreground="black")

        for i in range(3):
            lines[i].set_data(range(len(link.data[i])), list(link.data[i]))
        for i in range(3):
            ax[i].relim()
            ax[i].autoscale_view()
        canvas.draw_idle()

        if not link.quit.is_set():
            tick_after_id = root.after(100, tick)

    def on_close():
        link.quit.set()
        if tick_after_id is not None:
            # Cancel the pending tick() call — otherwise Tk still tries to
            # fire it after root.destroy() below, which raises "invalid
            # command name ...tick" since the interpreter is already gone.
            root.after_cancel(tick_after_id)
        if link.state == STATE_RUNNING:
            link.stop_requested.set()
        time.sleep(0.2)

        # Wait for the background thread to actually stop touching `ser`
        # before closing it — closing a port while another thread is
        # blocked inside ser.readline() on it is not safe on Windows and
        # can leave that thread (and the whole process) stuck forever.
        link_thread.join(timeout=2.0)
        try:
            ser.close()
        except Exception:
            pass
        root.destroy()

        # Safety net: if anything (a lingering Tcl/matplotlib timer, a
        # daemon thread wedged in a blocking OS call, etc.) is still
        # keeping the interpreter alive at this point, force the process
        # to exit rather than leave the terminal hung with no window to
        # show for it.
        os._exit(0)

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
