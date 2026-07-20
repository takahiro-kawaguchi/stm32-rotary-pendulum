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
    python remote_controller.py [PORT] --school  # simplified slider GUI for
                                                  # high-school outreach sessions
                                                  # (see STEP_DEFS)

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

# School-mode Station 2 slider baseline: an untouched snapshot of the
# hand-tuned STEADY_GAINS above, taken before any GUI slider can mutate it,
# so "responsiveness"/"damping" sliders are multipliers on the known-good
# values (and can be reset to them exactly) instead of drifting from
# whatever a previous rotation group left behind.
_BASE_STEADY_GAINS = dict(STEADY_GAINS)

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


def rebuild_model_matrices() -> None:
    """(Re)builds every numpy matrix derived from MODEL_W0_SQ/MODEL_GAMMA/
    MODEL_K. Called once at import time below. Split out as its own
    function (rather than bare module-level literals) so any future code
    that retunes those three constants at runtime has a single place to
    call to propagate the change into every dependent matrix/gain."""
    global _MODEL_A, _MODEL_B, _MODEL_Ad, _MODEL_Bd, _LQR_A, _LQR_B
    _MODEL_A = np.array([
        [0.0,         1.0,          0.0],
        [MODEL_W0_SQ, -MODEL_GAMMA, 0.0],
        [0.0,         0.0,          0.0],
    ])
    _MODEL_B = np.array([[0.0], [MODEL_K], [1.0]])
    _MODEL_Ad, _MODEL_Bd, *_ = cont2discrete(
        (_MODEL_A, _MODEL_B, np.eye(3), np.zeros((3, 1))), _TS, method='zoh')
    _LQR_A = np.array([
        [0.0,         0.0, 1.0,          0.0],
        [0.0,         0.0, 0.0,          1.0],
        [MODEL_W0_SQ, 0.0, -MODEL_GAMMA, 0.0],
        [0.0,         0.0, 0.0,          0.0],
    ])
    _LQR_B = np.array([[0.0], [0.0], [MODEL_K], [1.0]])


rebuild_model_matrices()

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
_lqr_state = {'K': None}

# School mode Step7 only: DualPidController-equivalent gains derived from
# the LQR gain K, so Step7 can teach LQR design (tune Q/R-style bounds,
# watch K change) while what actually gets *sent* to hardware is the
# battle-tested DualPidController -- 2026-07-20, after live testing showed
# PID holding noticeably more reliably than the raw state-feedback
# LQRController (whose own docstring already notes it trusts telemetry's
# omega_p/omega_r directly, unlike DualPidController's filtered
# self-computed derivative -- see OMEGA_GLITCH_CLAMP_DEG_S's comment for
# the concrete incident that traced back to exactly that difference).
#
# Derivation: LQRController computes u_rad_s2 = -(K @ [phi, theta_r,
# phi_dot, omega_r]), u = u_rad_s2 / STEPPER_RAD_PER_STEP. DualPidController
# (force_steady=True, Ki_comp=0, ignoring the derivative IIR filter's phase
# lag) computes u = Kp_pend*e_p + Kd_pend*e_p_dot + Kp_rotor*e_r +
# Kd_rotor*e_r_dot, where e_p = ENCODER_ANGLE_POLARITY*phi/STEPPER_RAD_PER_STEP
# and e_r = theta_r/STEPPER_RAD_PER_STEP (no polarity flip on the rotor
# terms). Matching coefficients term-by-term and using
# ENCODER_ANGLE_POLARITY = -1.0:
#   Kp_pend = K[0]      Kd_pend = K[2]
#   Kp_rotor = -K[1]    Kd_rotor = -K[3]
# Sanity-checked against the live-tuned LQR_PARAMS default (u_max=3.0):
# K=[354.2, -5.73, 50.23, -5.15] -> Kp_pend=354, Kd_pend=50, Kp_rotor=5.7,
# Kd_rotor=5.2 -- all four positive (the sign a PD gain "should" have) and
# in the same ballpark as STEADY_GAINS' own hand-tuned 300/30/15/7.5, a
# reassuring cross-check that the mapping is directionally correct.
LQR_EQUIVALENT_PID = dict(Kp_pend=0.0, Kd_pend=0.0, Kp_rotor=0.0, Kd_rotor=0.0, Ki_comp=0.0)


def recompute_lqr_gain():
    """(Re)computes the LQR gain from the current LQR_PARAMS, and its
    DualPidController-equivalent (LQR_EQUIVALENT_PID, see above). Called
    once at import time below, and again by the GUI's Apply button whenever
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

    K = _lqr_state['K'].flatten()
    LQR_EQUIVALENT_PID['Kp_pend'] = float(K[0])
    LQR_EQUIVALENT_PID['Kd_pend'] = float(K[2])
    LQR_EQUIVALENT_PID['Kp_rotor'] = float(-K[1])
    LQR_EQUIVALENT_PID['Kd_rotor'] = float(-K[3])


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
                rotor_ref: float = 0.0, force_steady: bool = False,
                gains: dict = None) -> float:
        """force_steady: skip the CATCH_GAINS phase and use STEADY_GAINS from
        the first sample — Mode C's up-mode control uses this; Mode B leaves
        it False to keep its existing (well-tuned) catch/steady schedule.
        gains: bypass the CATCH_GAINS/STEADY_GAINS schedule entirely and use
        this dict instead (school mode Step7 passes LQR_EQUIVALENT_PID) --
        force_steady is ignored when gains is given."""
        if gains is None:
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
# School mode: a simplified, slider-driven layer for high-school outreach
# sessions, built on top of everything above rather than replacing it (see
# STEP_DEFS and run_gui(school=True) below). Eight sequential "Steps"
# (following 計画.md), all riding firmware wire mode 'C' (the one protocol
# that streams telemetry and accepts Python 'u' commands from t=0 with no
# onboard swing-up attempt of its own):
#
#   Step1-4 run a pendulum-blind "rotor engine" (raw u / RotorPDController)
#     that never looks at origin_mode/up-down phase at all -- see the
#     STEPS_WITH_ROTOR_ENGINE dispatch in LinkManager._step_running().
#   Step5-8 do NOT get their own controllers. They reuse the swing-up +
#     balance engine that already exists in _step_running()'s Mode C block
#     (SwingUpController for the down phase, DualPidController/LQRController
#     for the up phase, switching automatically on origin_mode) completely
#     unmodified, and just scale each phase's output by STEP_GAINS:
#       Step5 (振り上げ):        swing_gain=1, balance_gain=0 -- pumps
#         energy in but never catches, so students can watch the swing grow
#         without a capture attempt cutting it short.
#       Step6 (倒立制御, PID):    swing_gain=0, balance_gain=1 -- rotor is
#         passive (u=0) until manually lifted into the capture zone, then
#         DualPidController holds it.
#       Step7 (LQR体験):         swing_gain=0, balance_gain=1, same idea
#         but LQRController instead of PID.
#       Step8 (フィナーレ):       swing_gain=1, balance_gain=1 -- the
#         engine's original, undiminished behaviour: autonomous swing-up
#         straight into a catch, PID or LQR chosen live via radio button.
#     This is deliberately NOT "swap in a different controller per step" --
#     it's the exact same engine every time, just with 0/1 gains on its two
#     halves, which is both far less code than per-step controller objects
#     and a nice callback to the whole course's "what happens when you turn
#     a gain to 0?" theme.
#
# Sliders are always multipliers/interpolations/direct-but-bounded values
# over a pre-vetted baseline, never free-text gain entry.
# ---------------------------------------------------------------------------
STEP1_PARAMS = dict(u=0.0)

ROTOR_PD_PARAMS = dict(Kp_rotor=20.0, Kd_rotor=0.0, u_max=20000.0, target_deg=0.0)
_BASE_ROTOR_PD_PARAMS = dict(ROTOR_PD_PARAMS)

# Step4: k1 (position) and k2 (velocity) are gains applied DIRECTLY to the
# pendulum's own angle/angular velocity and added straight into u, alongside
# (not routed through) Step2/3's rotor-position PD -- k1*theta_p + k2*omega_p
# is a second, independent feedback term, not a target for the rotor loop.
# (2026-07-20: originally wired as a *target* for the shared rotor-position
# PD, i.e. u included a Kp*k1 product -- that made k1/k2's actual effect on
# the pendulum silently depend on whatever Kp Step2/3 happened to be left
# at, defeating the point of sharing Kp/Kd with them. Additive instead of
# nested keeps Kp/Kd sharing meaningful: Step2/3 tune "how hard the rotor
# holds its position," k1/k2 tune "how hard the pendulum's own state pushes
# back," and the two no longer multiply together.) Exposed as independent
# sliders so students can discover empirically which term (and which sign/
# magnitude) actually damps the free-hanging pendulum fastest, per
# 計画.md's open question ("どういうルールで目標値を決めたら振り子が素早く
# 止まるだろうか？").
STEP4_PARAMS = dict(k1=0.0, k2=0.0)


class RotorPDController:
    """Rotor-position PD used by Steps 2-4 and Step6's "反転版" mode. Unlike
    a fixed-setpoint controller, target_deg is supplied fresh by the caller
    every cycle -- Step2/3/4 all pass ROTOR_PD_PARAMS['target_deg'] (Step4
    adds its own k1/k2 pendulum feedback separately, on top of this
    controller's output -- see STEP4_PARAMS and its dispatch in
    _step_running()); Step6-inverted passes k1*phi_deg + k2*phi_dot_deg_s
    instead (see STEP6_INVERTED_PARAMS), the one case that still nests it
    as a target. No internal state to reset between sessions since there's
    nothing to prime. `params` defaults to ROTOR_PD_PARAMS (Steps 2-4's own
    gains); pass STEP6_INVERTED_PARAMS explicitly to use Step6's independent
    set instead, so tuning one doesn't disturb the other."""

    def compute(self, theta_r_deg: float, omega_r_deg: float, target_deg: float,
                params: dict = None) -> float:
        p = params if params is not None else ROTOR_PD_PARAMS
        error = theta_r_deg - target_deg
        u = -p['Kp_rotor'] * error - p['Kd_rotor'] * omega_r_deg
        return max(-p['u_max'], min(p['u_max'], u))


def apply_step2_sliders(target_deg: float, Kp: float) -> None:
    ROTOR_PD_PARAMS.update(target_deg=target_deg, Kp_rotor=Kp, Kd_rotor=0.0)


def apply_step3_sliders(target_deg: float, Kp: float, Kd: float) -> None:
    ROTOR_PD_PARAMS.update(target_deg=target_deg, Kp_rotor=Kp, Kd_rotor=Kd)


def apply_step4_sliders(Kp: float, Kd: float, k1: float, k2: float) -> None:
    # Kp/Kd write into the same ROTOR_PD_PARAMS dict Step2/3 use (so the
    # rotor-position PD itself is the identical mechanism), but Step4's own
    # slider values (see STEP_DEFS's step4, no `shared` tag) are what's
    # actually in effect while Step4's tab is active -- switching to this
    # tab re-applies them, overwriting whatever Step2/3 last left behind.
    # See STEP_DEFS['step4']'s comment for why Kp/Kd are independent here.
    # k1/k2 (STEP4_PARAMS) are ADDED to u directly, alongside the rotor PD's
    # own output -- see the dispatch in _step_running() and STEP4_PARAMS'
    # comment for why additive, not nested.
    ROTOR_PD_PARAMS['Kp_rotor'] = Kp
    ROTOR_PD_PARAMS['Kd_rotor'] = Kd
    STEP4_PARAMS.update(k1=k1, k2=k2)


def apply_step5_sliders(**kwargs) -> None:
    # 8 of SWINGUP_PARAMS' 9 knobs -- same full set the instructor's
    # advanced panel exposes (see swingup_frame in run_gui()), just
    # relabeled/grouped for students, minus rotor_ki (see below). kwargs'
    # keys match SWINGUP_PARAMS' own keys 1:1 (see STEP_DEFS['step5']'s
    # slider specs).
    SWINGUP_PARAMS.update(kwargs)
    # 2026-07-20: I control removed from Step5's own tuning -- the swing-up
    # rotor PID works fine without it (P+D alone tracks the bang-bang
    # target well enough; see SwingUpController's docstring), and dropping
    # it is one fewer knob for students to reason about. Forced to 0 here
    # (rather than just omitting its slider) so Step5 always runs without
    # it regardless of whatever the advanced panel's own rotor_ki slider
    # (still exposed there, unaffected) currently shows.
    SWINGUP_PARAMS['rotor_ki'] = 0.0


def apply_step6_sliders(responsiveness: float, damping: float) -> None:
    # Scale pend/rotor gains together so the pend:rotor authority ratio the
    # file's own hardware tuning already found (~20:1, see CATCH_GAINS'
    # comment) stays invariant under the slider.
    STEADY_GAINS['Kp_pend']  = _BASE_STEADY_GAINS['Kp_pend']  * responsiveness
    STEADY_GAINS['Kp_rotor'] = _BASE_STEADY_GAINS['Kp_rotor'] * responsiveness
    STEADY_GAINS['Kd_pend']  = _BASE_STEADY_GAINS['Kd_pend']  * damping
    STEADY_GAINS['Kd_rotor'] = _BASE_STEADY_GAINS['Kd_rotor'] * damping


def apply_step6_independent_sliders(Kp_pend: float, Kd_pend: float,
                                     Kp_rotor: float, Kd_rotor: float) -> None:
    # Same STEADY_GAINS dict as the coupled "responsiveness/damping" mode
    # above, but each of the 4 gains is set directly -- no ratio-preserving
    # multiplier, so the pend:rotor authority balance is no longer protected.
    # Lets students discover *why* that ~20:1 ratio matters by being free to
    # break it.
    STEADY_GAINS.update(Kp_pend=Kp_pend, Kd_pend=Kd_pend, Kp_rotor=Kp_rotor, Kd_rotor=Kd_rotor)


# Step6's "反転版(Step4方式)" mode: the exact target=k1*phi+k2*phi_dot ->
# rotor-position-PD structure from Step4 (STEPS_WITH_ROTOR_ENGINE), reused
# near upright instead of hang-down. Numerically confirmed NOT to admit any
# stabilizing (Kp>0, Kd, k1, k2) combination (2026-07-20: a 500k-sample
# random search over the full 4-state closed loop A-B*K_eff found stable
# solutions only for Kp<0, which breaks the "same intuitive P gain as
# Step4" framing entirely -- LQR's own optimal gain has a theta_r-coefficient
# of the opposite sign a plain position-PD would need, a genuinely coupled
# MIMO effect this single-target-value structure can't reach). Kept anyway,
# deliberately, as a "watch it fail and see why" exploration -- see the
# conversation this was designed in for the full derivation -- rather than
# silently omitted, per explicit request to experience the reversed version
# before deciding the final course structure.
STEP6_INVERTED_PARAMS = dict(Kp_rotor=20.0, Kd_rotor=0.0, u_max=20000.0, k1=-1.0, k2=-0.5)


def apply_step6_inverted_sliders(Kp: float, Kd: float, k1: float, k2: float) -> None:
    STEP6_INVERTED_PARAMS.update(Kp_rotor=Kp, Kd_rotor=Kd, k1=k1, k2=k2)


def apply_step7_sliders(**kwargs) -> None:
    # All 5 LQR_PARAMS -- same set the instructor's advanced panel exposes
    # (see balance_frame's lqr_keys in run_gui()), just relabeled for
    # students. kwargs' keys match LQR_PARAMS' own keys 1:1 (see
    # STEP_DEFS['step7']'s slider specs).
    LQR_PARAMS.update(kwargs)
    recompute_lqr_gain()


# Multiplies the swing-up/balance engine's two halves on/off per step (see
# the module comment above) -- read directly by LinkManager._step_running()
# only when school_mode is True; the plain/advanced --gui mode is unaffected.
STEP_GAINS = dict(swing_gain=1.0, balance_gain=1.0)

STEPS_WITH_ROTOR_ENGINE = ('step1', 'step2', 'step3', 'step4')

# step key -> label, challenge text, slider specs, and (for step5-8) the
# STEP_GAINS/balance_controller this step drives. The GUI reads this dict
# alone to build each tab's panel -- wording/ranges can be re-tuned here
# without touching layout code.
STEP_DEFS = {
    'step1': dict(
        label="Step1: 観察",
        challenge="uを変えてロータの動きを観察しよう。uを0に戻しても慣性で動き続ける?",
        sliders=(
            dict(key='u', label='入力 u [steps/s²]', frm=-3000.0, to=3000.0, default=0.0),
        ),
        apply=lambda v: STEP1_PARAMS.__setitem__('u', v['u']),
    ),
    'step2': dict(
        label="Step2: 比例制御",
        challenge="比例ゲインKpを変えてみよう。目標角度に近づく? 振動しない?",
        # target_deg/Kp use `shared` tags (see run_gui()'s shared_slider_vars)
        # so Step2<->Step3 share the exact same value -- Step3 = Step2 +
        # a Kd term, not a separate independent setup, so switching between
        # them shouldn't reset what was already tuned.
        sliders=(
            dict(key='target_deg', label='目標角度[deg]', frm=-90.0, to=90.0, default=0.0,
                 shared='rotor_target_deg'),
            dict(key='Kp', label='比例ゲインKp', frm=0.0, to=2000.0, default=20.0, shared='rotor_Kp'),
        ),
        apply=lambda v: apply_step2_sliders(v['target_deg'], v['Kp']),
    ),
    'step3': dict(
        label="Step3: 微分制御",
        challenge="Kdを追加してみよう。振動がどう変わる? オーバーシュートの境界は?",
        sliders=(
            dict(key='target_deg', label='目標角度[deg]', frm=-90.0, to=90.0, default=0.0,
                 shared='rotor_target_deg'),
            dict(key='Kp', label='比例ゲインKp', frm=0.0, to=2000.0, default=20.0, shared='rotor_Kp'),
            dict(key='Kd', label='微分ゲインKd', frm=0.0, to=200.0, default=0.0, shared='rotor_Kd'),
        ),
        apply=lambda v: apply_step3_sliders(v['target_deg'], v['Kp'], v['Kd']),
    ),
    'step4': dict(
        label="Step4: 振り子を下側で止める",
        challenge="振り子の角度・角速度にかかるゲインk1・k2を足してみよう。どちらが・どの符号でよく効く?",
        # Kp/Kd are Step4's OWN independent gains (2026-07-20: unshared from
        # Step2/3's 'rotor_Kp'/'rotor_Kd' tags again -- even with k1/k2 now
        # additive rather than nested in the target [see STEP4_PARAMS'
        # comment], Step2/3's choice of Kp/Kd still changes the ROTOR
        # SUBSYSTEM's own bandwidth, which changes which *sign* of k2 damps
        # the pendulum well: with Kp=20 the rotor responds slower than the
        # pendulum's free swing and k2>0 helps, but with Kp=1500 [swing-up
        # scale] the rotor becomes faster than the pendulum and the
        # relationship flips -- k2>0 destabilizes, k2<0 (roughly -100..-250)
        # is what works instead. Independent sliders mean Step4 always
        # starts from ONE verified, self-consistent (Kp, Kd, k1, k2) point,
        # not something whose meaning shifts depending on what Step2/3
        # happen to currently show. target_deg is NOT one of Step4's own
        # sliders (see _step_running()'s dispatch) -- it still reads
        # ROTOR_PD_PARAMS['target_deg'], i.e. whatever Step2/3 last set (0.0
        # if untouched); revisit if that turns out to matter in practice.
        #
        # Defaults/ranges verified 2026-07-20 against the closed-loop
        # eigenvalues of a down-side linearized model (theta_p near 0 =
        # hang-down; same construction as _LQR_A/_LQR_B but with the sign
        # flips that equilibrium requires -- see the conversation this was
        # derived in), at THIS step's own Kp=20/Kd=10. Findings, in case
        # these ever need retuning:
        #   - Kd=0 leaves the rotor loop completely undamped -- a marginal
        #     +-1.5j pole with Kp=20, independent of k1/k2 -- and any
        #     nonzero k1/k2 then tips that marginal pole slightly unstable.
        #     Kd needs to be meaningfully nonzero, hence the default (10.0).
        #   - k2 (velocity term) has a strong, clean, monotonic effect at
        #     this Kp/Kd: k2=-10 is already clearly unstable (worst
        #     eigenvalue +0.17), k2=+10..+30 cuts the pendulum's settling
        #     time by roughly 3-6x versus k2=0. This is the dominant knob
        #     -- but see the note above: the sign that helps is NOT fixed,
        #     it depends on Kp/Kd's own value relative to the pendulum's
        #     ~6.9 rad/s natural frequency.
        #   - k1 (position term) has a much weaker direct effect in the
        #     full 4-state system than a naive 2-state approximation
        #     suggests (the rotor's own ~1.5 rad/s response here is slower
        #     than the pendulum's free swing, so it can't track fast enough
        #     for k1 alone to matter much) -- kept at a small positive
        #     default per 計画.md's own hypothesis ("重心位置を目標値に
        #     する"), but k2 is where the real effect is at this Kp/Kd.
        sliders=(
            dict(key='Kp', label='比例ゲインKp', frm=0.0, to=2000.0, default=20.0),
            dict(key='Kd', label='微分ゲインKd', frm=0.0, to=200.0, default=10.0),
            # k1/k2 ranges widened 2026-07-20: a Bryson's-rule LQR design
            # swept across tighter theta_p_max_deg tolerances (30 deg down
            # to 1 deg) landed k1 anywhere from ~27 up to ~1550 and k2 from
            # ~10 up to ~177 -- the old +-30/+-30 only covered the loosest
            # end of that range. k1's own range given extra headroom above
            # that ~1550 ceiling (matching Kp's own 2000 span) since it's
            # the one that kept climbing fastest as tolerance tightened.
            dict(key='k1', label='位置比例 k1', frm=-2000.0, to=2000.0, default=10.0),
            dict(key='k2', label='速度比例 k2', frm=-200.0, to=200.0, default=20.0),
        ),
        apply=lambda v: apply_step4_sliders(v['Kp'], v['Kd'], v['k1'], v['k2']),
    ),
    'step5': dict(
        label="Step5: 振り上げ",
        challenge="振り上げのパラメータを変えて、エネルギーの入れ方を観察しよう。",
        # 8 of SWINGUP_PARAMS' 9 knobs the instructor's advanced panel
        # exposes -- rotor_ki omitted (see apply_step5_sliders(), forced to
        # 0 for this step).
        sliders=(
            dict(key='stage0_deg', label='初期振幅Stage0[deg]', frm=10.0, to=35.0, default=22.5),
            dict(key='stage1_deg', label='中間振幅Stage1[deg]', frm=5.0, to=25.0, default=14.625),
            dict(key='stage2_deg', label='後半振幅Stage2[deg]', frm=5.0, to=20.0, default=13.5),
            dict(key='stage1_threshold_deg', label='しきい値1[deg]', frm=50.0, to=150.0, default=90.0),
            dict(key='stage2_threshold_deg', label='しきい値2[deg]', frm=100.0, to=200.0, default=150.0),
            dict(key='rotor_kp', label='ロータ比例Kp', frm=500.0, to=3000.0, default=1500.0),
            dict(key='rotor_kd', label='ロータ微分Kd', frm=0.0, to=400.0, default=150.0),
            dict(key='rotor_u_max', label='ロータu上限', frm=5000.0, to=30000.0, default=20000.0),
        ),
        apply=lambda v: apply_step5_sliders(**v),
        swing_gain=1.0, balance_gain=0.0, balance_controller=None,
    ),
    'step6': dict(
        label="Step6: 倒立制御",
        challenge="手で上まで持っていくと倒立制御が始まるよ。調整方法を切り替えて比べてみよう。",
        # Step6 has 3 distinct slider sets (see run_gui()'s build_step6_tab) --
        # a mode radio button switches between them live. No top-level
        # 'sliders'/'apply' here (unlike every other step) -- see modes[...]
        # instead, keyed the same way link.step6_mode is.
        modes={
            'coupled': dict(
                mode_label="まとめて調整",
                sliders=(
                    dict(key='responsiveness', label='反応の速さ', frm=0.5, to=1.8, default=1.0),
                    dict(key='damping', label='揺れの抑え方', frm=0.5, to=2.0, default=1.0),
                ),
                apply=lambda v: apply_step6_sliders(v['responsiveness'], v['damping']),
            ),
            'independent': dict(
                mode_label="個別調整",
                sliders=(
                    dict(key='Kp_pend', label='比例ゲイン(振り子)', frm=0.0, to=600.0, default=300.0),
                    dict(key='Kd_pend', label='微分ゲイン(振り子)', frm=0.0, to=100.0, default=30.0),
                    dict(key='Kp_rotor', label='比例ゲイン(ロータ)', frm=0.0, to=60.0, default=15.0),
                    dict(key='Kd_rotor', label='微分ゲイン(ロータ)', frm=0.0, to=30.0, default=7.5),
                ),
                apply=lambda v: apply_step6_independent_sliders(
                    v['Kp_pend'], v['Kd_pend'], v['Kp_rotor'], v['Kd_rotor']),
            ),
            'inverted': dict(
                mode_label="反転版(Step4方式)",
                # Ranges match Step4's own (2026-07-20) so a value copied in
                # via "Step4の値を反映" always fits without the slider
                # clamping it down to a narrower span on the next touch.
                sliders=(
                    dict(key='Kp', label='比例ゲインKp', frm=0.0, to=2000.0, default=20.0),
                    dict(key='Kd', label='微分ゲインKd', frm=0.0, to=200.0, default=0.0),
                    dict(key='k1', label='位置比例 k1', frm=-2000.0, to=2000.0, default=-1.0),
                    dict(key='k2', label='速度比例 k2', frm=-200.0, to=200.0, default=-0.5),
                ),
                apply=lambda v: apply_step6_inverted_sliders(v['Kp'], v['Kd'], v['k1'], v['k2']),
            ),
        },
        swing_gain=0.0, balance_gain=1.0, balance_controller='pid',
    ),
    'step7': dict(
        label="Step7: LQR体験",
        challenge="許容ふらつきを変えてLQRゲインの変化を観察しよう。下の換算PIDゲインも一緒に見てみよう。",
        # Full LQR_PARAMS, same 5 knobs the instructor's advanced panel
        # exposes (see balance_frame's lqr_keys in run_gui()) -- not just
        # phi_max_deg, so students can see the whole Bryson's-rule picture.
        sliders=(
            dict(key='phi_max_deg', label='phi上限[deg]', frm=1.0, to=20.0, default=5.0),
            dict(key='theta_r_max_deg', label='theta_r上限[deg]', frm=5.0, to=90.0, default=30.0),
            dict(key='phi_dot_max_deg_s', label='phi_dot上限[deg/s]', frm=20.0, to=400.0, default=120.0),
            dict(key='omega_r_max_deg_s', label='omega_r上限[deg/s]', frm=50.0, to=800.0, default=300.0),
            dict(key='u_max', label='u上限[rad/s²]', frm=0.5, to=50.0, default=3.0),
        ),
        apply=lambda v: apply_step7_sliders(**v),
        # balance_controller is 'pid', not 'lqr' -- these sliders design an
        # LQR gain (recompute_lqr_gain() still solves the Riccati equation
        # live), but what's actually SENT is DualPidController running the
        # LQR-equivalent gains (LQR_EQUIVALENT_PID, see its derivation
        # comment) rather than raw state feedback -- see the school-mode
        # dispatch in _step_running(). More reliable in practice (PID's own
        # filtered derivative vs. LQRController trusting telemetry's
        # omega_p/omega_r directly), while still teaching the LQR design
        # process end to end.
        swing_gain=0.0, balance_gain=1.0, balance_controller='pid',
    ),
    'step8': dict(
        label="Step8: 振り上げ→倒立(フィナーレ)",
        challenge="Step5の振り上げとStep6/7の倒立制御をつなげて、通しで見てみよう。",
        sliders=(),
        apply=lambda v: None,
        # balance_controller chosen via the "上側(倒立後)の制御則" radio
        # (has_controller_choice below) -- 'pid' reuses whatever Step6 last
        # tuned (STEADY_GAINS); 'lqr' reuses whatever Step7 last tuned too
        # (LQR_EQUIVALENT_PID, same DualPidController-via-LQR-gains path as
        # Step7 -- see the school_lqr_as_pid dispatch in _step_running()),
        # not a fresh/independent LQR run.
        swing_gain=1.0, balance_gain=1.0, balance_controller=None,
        has_controller_choice=True,
    ),
}
STEP_ORDER = ('step1', 'step2', 'step3', 'step4', 'step5', 'step6', 'step7', 'step8')


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
        # School mode Steps 2/3 only: the rotor's commanded target_deg each
        # cycle, plotted overlaid on the rotor-angle series (see run_gui())
        # so students can see actual-vs-target directly instead of only
        # inferring tracking quality from the rotor line alone.
        self.target_data = deque(maxlen=MAX_POINTS)
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
        # School mode only (see run_gui(school=True)/STEP_DEFS): the Step1-4
        # rotor engine's controller and a group-ID tag threaded into log
        # filenames. All inert (school_mode stays False) for plain/
        # advanced-GUI sessions.
        self.rotor_pd_ctrl = RotorPDController()
        self.school_mode = False
        self.current_step = 'step1'
        # Step6 only: which of STEP_DEFS['step6']['modes'] is active
        # ('coupled'/'independent'/'inverted') -- see build_step6_tab().
        self.step6_mode = 'coupled'
        self.group_tag = ''
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
        tag = f"_group{self.group_tag}" if self.group_tag else ""
        path = LOG_DIR / f"log_{time.strftime('%Y%m%d_%H%M%S')}_mode{self.selected_mode}{tag}.csv"
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

    def set_current_step(self, step_key: str) -> None:
        """Applies a Step's swing/balance gains and controller choice.
        Called both when Start is pressed and live whenever the GUI's Step
        tab changes (run_gui()'s on_tab_change()) -- including mid-session,
        so switching Steps takes effect on the very next telemetry sample
        instead of requiring a Stop/Start (and therefore a firmware reboot,
        since 'q' always triggers one) between every Step. Steps 1-4 (the
        rotor engine, dispatched separately in _step_running()) have no
        gains/controller to set here."""
        self.current_step = step_key
        if step_key not in STEPS_WITH_ROTOR_ENGINE:
            step_def = STEP_DEFS[step_key]
            STEP_GAINS['swing_gain'] = step_def.get('swing_gain', 1.0)
            STEP_GAINS['balance_gain'] = step_def.get('balance_gain', 1.0)
            if step_def.get('balance_controller') is not None:
                self.balance_controller = step_def['balance_controller']
                if step_def['balance_controller'] == 'lqr':
                    # Whenever LQR becomes the active controller, default to
                    # the Kalman-filtered state estimate rather than raw
                    # telemetry -- see on_balance_ctrl_change() in run_gui()
                    # for the same rule applied when the advanced panel's own
                    # PID/LQR radio is used directly.
                    self.lqr_state_source = 'observer'
            # else: Step8 -- the GUI's own PID/LQR radio already set
            # self.balance_controller directly; Step5 doesn't care
            # (balance_gain=0 zeroes its output regardless of which
            # controller would have run).

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
            if self.school_mode:
                # Every Step rides wire mode 'C' unconditionally (see the
                # module comment above STEP_DEFS). set_current_step() (also
                # called live by the GUI whenever the Step tab changes, even
                # mid-session -- see run_gui()'s on_tab_change()) applies
                # this step's STEP_GAINS/balance_controller; re-applying it
                # here too is just a harmless double-check in case Start is
                # pressed before any tab-change callback has fired yet.
                self.selected_mode = 'C'
                self.set_current_step(self.current_step)
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
            self.target_data.clear()
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

        if self.selected_mode == 'C' and not self._startup_kick_done:
            # Hardware sanity-check pulse, fired once at the very start of
            # every Mode C session regardless of which Step is selected
            # (school mode always rides wire 'C' -- see the module comment
            # above STEP_DEFS) -- must run *before* the Steps 1-4 rotor
            # engine dispatch below, which would otherwise return early and
            # skip this entirely every time a session starts on Step1-4.
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

        if self.school_mode and self.current_step in STEPS_WITH_ROTOR_ENGINE:
            # Steps 1-4: a pendulum-blind rotor engine, entirely separate
            # from Mode C's swing-up/balance dispatch below (see the module
            # comment above STEP_DEFS).
            if self.current_step == 'step1':
                u = STEP1_PARAMS['u']
            elif self.current_step in ('step2', 'step3'):
                target_deg = ROTOR_PD_PARAMS['target_deg']
                u = self.rotor_pd_ctrl.compute(theta_r, omega_r, target_deg)
                self.target_data.append(target_deg)
            else:  # step4
                # Same rotor-position PD as Step2/3 (shared Kp/Kd AND
                # target_deg -- see apply_step4_sliders), PLUS a direct,
                # independent pendulum feedback term added straight into u
                # (not folded into target_deg -- see STEP4_PARAMS' comment
                # for why additive beats nested here).
                target_deg = ROTOR_PD_PARAMS['target_deg']
                u_rotor = self.rotor_pd_ctrl.compute(theta_r, omega_r, target_deg)
                u = u_rotor + STEP4_PARAMS['k1'] * theta_p + STEP4_PARAMS['k2'] * omega_p
                self.target_data.append(target_deg)
            if abs(theta_r) > ROTOR_LIMIT_DEG:
                u = 0.0
            self.ser.write(f'u {u:.1f}\r'.encode())
            self._log_sample(i_idx, theta_p, theta_r, omega_p, omega_r, u_prev, u)
            self.data[0].append(theta_p)
            self.data[1].append(theta_r)
            self.data[2].append(u)
            return

        if self.selected_mode == 'C':
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
            school_lqr_as_pid = (self.school_mode and self.current_step in ('step7', 'step8')
                                  and self.balance_controller == 'lqr')
            active_ctrl = (self.ctrl if school_lqr_as_pid
                            else self.lqr_ctrl if self.balance_controller == 'lqr'
                            else self.ctrl)
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
                    if self.school_mode and self.current_step == 'step5':
                        # Step5: stop right as the swing-up first reaches the
                        # upright zone, instead of quietly going passive
                        # (balance_gain=0 already keeps it from catching)
                        # and cycling back down to swing up again. The point
                        # of this step is watching the energy-pumping swing
                        # grow, not what happens after it first arrives.
                        self.stop_requested.set()
                    active_ctrl.enter_capture(theta_p_upright, theta_r)
                    if not school_lqr_as_pid and self.balance_controller == 'lqr' and self.lqr_state_source == 'observer':
                        self.observer.enter_capture(theta_p_upright, theta_r, rotor_ref_steps)
                    elif not school_lqr_as_pid and self.balance_controller == 'lqr' and self.lqr_state_source == 'observer_dob':
                        self.dob.enter_capture(theta_p_upright, theta_r, rotor_ref_steps)
                    self.in_balance = True
                if school_lqr_as_pid:
                    # School mode's "LQR" always means DualPidController
                    # running the gains derived from the LQR design
                    # (LQR_EQUIVALENT_PID) instead of LQRController's raw
                    # state feedback -- see STEP_DEFS' step7 comment for why.
                    # Step8 sharing this branch (not just step7) is what
                    # makes "whatever was tuned in Step7 carries straight
                    # into Step8's finale" true: LQR_EQUIVALENT_PID is one
                    # live global, not something re-derived per step.
                    u = self.ctrl.compute(theta_p_upright, theta_r, rotor_ref_steps,
                                           gains=LQR_EQUIVALENT_PID)
                elif self.balance_controller == 'lqr':
                    # Advanced/instructor GUI only (school mode never reaches
                    # here) -- genuine full-state-feedback LQRController.
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
                elif (self.school_mode and self.current_step == 'step6'
                        and self.step6_mode == 'inverted'):
                    # "反転版": Step4's own target=k1*phi+k2*phi_dot ->
                    # rotor-position-PD mechanism, reused near upright with
                    # its own independent gains (STEP6_INVERTED_PARAMS) --
                    # NOT expected to stabilize with Kp>0 (verified
                    # numerically, see STEP6_INVERTED_PARAMS' comment); kept
                    # as a deliberate "watch it fail" exploration.
                    p = STEP6_INVERTED_PARAMS
                    target_deg = p['k1'] * theta_p_upright + p['k2'] * omega_p
                    u = self.rotor_pd_ctrl.compute(theta_r, omega_r, target_deg, params=p)
                else:
                    u = self.ctrl.compute(theta_p_upright, theta_r, rotor_ref_steps, force_steady=True)
                if self.school_mode:
                    u *= STEP_GAINS['balance_gain']
            else:
                if self.in_balance:
                    # Was catching, escaped back down -- clear catch state
                    # and start the swing controller fresh instead of
                    # resuming whatever stale derivative state it had from
                    # before capture.
                    #
                    # Step6 only: once a hand-caught inversion falls back
                    # out of the capture zone, stop the session outright
                    # (same path the GUI's own Stop button uses) instead of
                    # going passive and waiting to be lifted again -- Step6
                    # is meant as one deliberate catch-and-hold attempt per
                    # run, not a repeat-until-it-sticks loop.
                    if self.school_mode and self.current_step == 'step6':
                        self.stop_requested.set()
                    self.in_balance = False
                    self.ctrl.reset()
                    self.lqr_ctrl.reset()
                    self.observer.reset()
                    self.dob.reset()
                    self.swing_ctrl.reset()
                u = self.swing_ctrl.compute(theta_p, theta_r, omega_r)
                if self.school_mode:
                    u *= STEP_GAINS['swing_gain']
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


def run_gui(port: str, school: bool = False) -> None:
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
    link.school_mode = school
    link_thread = threading.Thread(target=link.run, daemon=True)
    link_thread.start()

    root = tk.Tk()
    root.title("STM32 Pendulum — Remote Controller")
    root.geometry("1700x950")

    # 2026-07-20: bumped from the original 900x650 -- the plot and every
    # widget were too small to read at classroom/demo distance. UI_FONT
    # backs a broad ttk style override (base widgets, LabelFrame titles,
    # Notebook tabs); BIG_FONT is reserved for Start/Stop specifically (see
    # "Big.TButton" below) so they stay unmissable at a glance. Individual
    # widgets that hardcode their own `font=(...)` (e.g. the Step challenge
    # text) still need bumping by hand -- ttk.Style only covers widgets that
    # don't override their own font.
    UI_FONT = ("Yu Gothic UI", 13)
    BIG_FONT = ("Yu Gothic UI", 22, "bold")
    style = ttk.Style()
    style.configure(".", font=UI_FONT)
    style.configure("TLabelframe.Label", font=(UI_FONT[0], UI_FONT[1], "bold"))
    style.configure("TNotebook.Tab", font=UI_FONT, padding=(12, 6))
    style.configure("TButton", padding=6)
    style.configure("Big.TButton", font=BIG_FONT, padding=(28, 18))
    # Windows' default ttk theme doesn't reliably cascade the "." font
    # override down to Entry/Radiobutton/Checkbutton -- set them explicitly
    # rather than relying on inheritance (this was the concrete cause of
    # "the number next to the slider is small": TEntry was silently staying
    # at the Tk default size despite the "." override above).
    style.configure("TEntry", font=UI_FONT)
    style.configure("TRadiobutton", font=UI_FONT)
    style.configure("TCheckbutton", font=UI_FONT)
    SLIDER_ENTRY_FONT = (UI_FONT[0], 15, "bold")
    # School mode's Step tab bar (see below): ttk.Notebook can't wrap tabs
    # onto multiple rows, so it's built from Toolbutton-styled Radiobuttons
    # in a manual grid instead -- Toolbutton is ttk's built-in "flat,
    # looks-pressed-when-selected" style, the standard way to fake a tab bar
    # when you need more layout control than Notebook allows.
    style.configure("Toolbutton", font=(UI_FONT[0], UI_FONT[1], "bold"), padding=(10, 10))

    # School mode only: the instructor-facing "詳細設定" toggle (see below)
    # shows/hides the advanced widgets collected here instead of them always
    # being packed -- students only ever see the simplified school_frame
    # (Step1-8 Notebook tabs) built further down. group_var is declared
    # unconditionally (harmless when school=False) so do_start()/tick() can
    # reference it without branching everywhere.
    advanced_items = []

    def pack_maybe(widget, **kwargs):
        if not school:
            widget.pack(**kwargs)
        else:
            advanced_items.append((widget, kwargs))

    def toggle_advanced():
        if toggle_advanced.visible:
            for widget, _ in advanced_items:
                widget.pack_forget()
        else:
            for widget, kwargs in advanced_items:
                widget.pack(**kwargs)
        toggle_advanced.visible = not toggle_advanced.visible

    toggle_advanced.visible = False

    top = ttk.Frame(root, padding=8)
    top.pack(fill="x")

    status_var = tk.StringVar(value=link.status_text)
    ttk.Label(top, textvariable=status_var).pack(side="left")

    # Mode C only: shows which origin frame _step_running() currently thinks
    # the pendulum is in (see LinkManager.origin_mode) -- lets the "0 = down
    # -> 0 = upright" frame switch be verified visually before any control
    # law is reconnected.
    origin_mode_var = tk.StringVar(value="原点: —")
    origin_mode_label = ttk.Label(top, textvariable=origin_mode_var, font=(UI_FONT[0], UI_FONT[1], "bold"))
    pack_maybe(origin_mode_label, side="left", padx=(16, 0))

    origin_threshold_label = ttk.Label(top, text="真上しきい値[deg]")
    pack_maybe(origin_threshold_label, side="left", padx=(16, 2))
    origin_threshold_var = tk.DoubleVar(value=ORIGIN_MODE_THRESHOLD['deg'])
    origin_threshold_entry = ttk.Entry(top, textvariable=origin_threshold_var, width=6)
    pack_maybe(origin_threshold_entry, side="left")

    def apply_origin_threshold():
        try:
            ORIGIN_MODE_THRESHOLD['deg'] = origin_threshold_var.get()
        except tk.TclError:
            pass  # invalid entry text — leave the threshold unchanged

    origin_threshold_btn = ttk.Button(top, text="適用", command=apply_origin_threshold)
    pack_maybe(origin_threshold_btn, side="left", padx=(2, 0))

    # School mode: Start also stamps the current group ID into the log
    # filename (see LinkManager._open_log); group_var is unused otherwise.
    group_var = tk.StringVar(value='1')

    def do_start():
        if school:
            link.group_tag = group_var.get().strip()
        link.start_requested.set()

    stop_btn = ttk.Button(top, text="Stop", command=link.stop_requested.set, style="Big.TButton")
    start_btn = ttk.Button(top, text="Start", command=do_start, style="Big.TButton")
    stop_btn.pack(side="right", padx=6)
    start_btn.pack(side="right", padx=6)

    if school:
        ttk.Button(top, text="詳細設定(指導者用)", command=toggle_advanced).pack(side="right", padx=(0, 12))

    mode_var = tk.StringVar(value=link.selected_mode)

    def on_mode_change():
        link.selected_mode = mode_var.get()

    mode_frame = ttk.Frame(top)
    pack_maybe(mode_frame, side="right", padx=12)
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

    # --- Two-column body: waveform on the left, every settings panel on the
    # right (2026-07-20, replacing the old single top-to-bottom stack where
    # the plot only got whatever vertical space was left over below a tall
    # stack of panels). Equal-weighted 50/50 split via grid, both sides
    # filling the window's full height.
    main_pane = ttk.Frame(root)
    main_pane.pack(fill="both", expand=True)
    main_pane.columnconfigure(0, weight=1)
    main_pane.columnconfigure(1, weight=1)
    main_pane.rowconfigure(0, weight=1)

    left_frame = ttk.Frame(main_pane)
    left_frame.grid(row=0, column=0, sticky="nsew")

    # Right column's panels can add up to more vertical space than the
    # window is tall (all of swingup/balance/observer plus, in school mode,
    # 8 Step tabs) -- wrap in a scrollable canvas rather than letting
    # content get clipped or forcing the window ever-taller. settings_frame
    # (not right_frame directly) is what every panel below is built inside.
    right_frame = ttk.Frame(main_pane)
    right_frame.grid(row=0, column=1, sticky="nsew")
    right_canvas = tk.Canvas(right_frame, highlightthickness=0)
    right_scrollbar = ttk.Scrollbar(right_frame, orient="vertical", command=right_canvas.yview)
    settings_frame = ttk.Frame(right_canvas)
    settings_frame.bind("<Configure>",
                         lambda e: right_canvas.configure(scrollregion=right_canvas.bbox("all")))
    right_canvas_window = right_canvas.create_window((0, 0), window=settings_frame, anchor="nw")
    right_canvas.bind("<Configure>",
                       lambda e: right_canvas.itemconfigure(right_canvas_window, width=e.width))
    right_canvas.configure(yscrollcommand=right_scrollbar.set)
    right_canvas.pack(side="left", fill="both", expand=True)
    right_scrollbar.pack(side="right", fill="y")

    def _on_mousewheel(event):
        right_canvas.yview_scroll(int(-event.delta / 120), "units")

    # Scoped to only fire while the cursor is actually over the right
    # column (bind_all while hovering, unbind on leave) -- an unscoped
    # bind_all would hijack scrolling anywhere in the window, including
    # over the plot on the left.
    right_canvas.bind("<Enter>", lambda e: right_canvas.bind_all("<MouseWheel>", _on_mousewheel))
    right_canvas.bind("<Leave>", lambda e: right_canvas.unbind_all("<MouseWheel>"))

    # --- Swing-up parameter panel (Mode C) — live-tunable, no restart needed ---
    swingup_frame = ttk.LabelFrame(settings_frame, text="Swing-up params (Mode C)", padding=6)
    pack_maybe(swingup_frame, fill="x", padx=8, pady=(0, 8))

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
    balance_frame = ttk.LabelFrame(settings_frame, text="Balance controller (Mode C up-mode)", padding=6)
    pack_maybe(balance_frame, fill="x", padx=8, pady=(0, 8))

    balance_ctrl_var = tk.StringVar(value=link.balance_controller)

    def on_balance_ctrl_change():
        link.balance_controller = balance_ctrl_var.get()
        if link.balance_controller == 'lqr':
            # Default to the Kalman-filtered state estimate rather than raw
            # telemetry whenever LQR becomes active (same rule
            # set_current_step() applies for Step7) -- lqr_source_var is
            # defined further down in this function, but by the time this
            # callback can actually fire (a button click) the whole GUI is
            # already built, so the forward reference is safe.
            link.lqr_state_source = 'observer'
            lqr_source_var.set('observer')

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
    observer_frame = ttk.LabelFrame(settings_frame, text="Kalman Observer / 外乱オブザーバ (LQR用状態推定)", padding=6)
    pack_maybe(observer_frame, fill="x", padx=8, pady=(0, 8))

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

    # --- School mode: Step1-8 tab bar (see STEP_DEFS) ---
    capture_stats = {'A': None, 'B': None}
    capture_label_vars = {'A': tk.StringVar(value='記録A: -'), 'B': tk.StringVar(value='記録B: -')}
    step_slider_vars = {}   # step_key -> {slider_key: DoubleVar}
    step7_pid_var = None    # live LQR_EQUIVALENT_PID readout, updated in tick()
    step_frames = {}        # step_key -> content Frame (manual tabs, see below)
    active_step_var = None

    if school:
        school_frame = ttk.LabelFrame(settings_frame, text="スクールモード", padding=8)
        school_frame.pack(fill="x", padx=8, pady=(0, 8))

        top_row = ttk.Frame(school_frame)
        top_row.pack(fill="x")
        ttk.Label(top_row, text="グループID:").pack(side="left")
        ttk.Entry(top_row, textvariable=group_var, width=4).pack(side="left", padx=(4, 16))

        # 2 rows x 4 columns of tab-like toggle buttons (see show_step()
        # below) instead of a ttk.Notebook -- Notebook only ever lays its
        # tabs out on one row, which either overflows into a tiny scroll
        # arrow or forces the tab font back down at 8 Steps' worth of
        # labels, defeating the point of the larger font elsewhere.
        tab_bar = ttk.Frame(school_frame)
        tab_bar.pack(fill="x", pady=(6, 4))
        for col in range(4):
            tab_bar.columnconfigure(col, weight=1)

        content_container = ttk.Frame(school_frame)
        content_container.pack(fill="both", expand=True)

        active_step_var = tk.StringVar(value=STEP_ORDER[0])

        def show_step(step_key):
            active_step_var.set(step_key)
            for frame in step_frames.values():
                frame.pack_forget()
            step_frames[step_key].pack(fill="both", expand=True)
            # Applies live, even mid-session -- see set_current_step()'s
            # docstring for why switching Steps doesn't need a Stop/Start.
            link.set_current_step(step_key)
            apply_step_sliders(step_key)

        for i, step_key in enumerate(STEP_ORDER):
            row, col = divmod(i, 4)
            ttk.Radiobutton(tab_bar, text=STEP_DEFS[step_key]['label'], variable=active_step_var,
                             value=step_key, style="Toolbutton",
                             command=lambda k=step_key: show_step(k)).grid(
                row=row, column=col, sticky="ew", padx=2, pady=2)

        def apply_step_sliders(step_key):
            if step_key == 'step6':
                # Step6 has no top-level 'sliders'/'apply' (see
                # STEP_DEFS['step6']['modes']) -- always re-applies whichever
                # mode is currently selected, mirroring
                # apply_current_step_sliders()'s own "always re-derive, never
                # bake in a stale value" reasoning below.
                mode_def = STEP_DEFS['step6']['modes'][link.step6_mode]
                values = {s['key']: step_slider_vars['step6'][s['key']].get() for s in mode_def['sliders']}
                mode_def['apply'](values)
                return
            step_def = STEP_DEFS[step_key]
            values = {s['key']: step_slider_vars[step_key][s['key']].get() for s in step_def['sliders']}
            step_def['apply'](values)

        def apply_current_step_sliders():
            # Always re-applies whichever Step is *currently selected*,
            # never a step_key baked in at widget-construction time. This
            # matters because shared sliders (see shared_slider_vars below)
            # are driven by one tk.DoubleVar watched by several Steps'
            # Scale widgets at once -- Tk invokes every one of those
            # widgets' own -command callback when the shared variable
            # changes, not just the one the user is actually looking at.
            # Re-deriving from link.current_step every time keeps the
            # result correct (e.g. Step2's apply forcing Kd=0 must never
            # fire just because its hidden Scale also noticed the shared
            # Kp variable move while Step4 is the active tab).
            apply_step_sliders(link.current_step)

        def bind_scale_click_to_jump(scale) -> None:
            """ttk.Scale's default click-on-trough behaviour nudges toward
            the click by one page-increment instead of jumping straight
            there, which reads as unresponsive for a click-to-set slider.
            Repositioning directly to the clicked point *before* the
            default binding runs (an instance binding fires ahead of the
            widget's class binding) makes clicking behave as expected,
            while leaving the class binding's own press/drag handling
            intact for normal dragging afterwards."""
            def _jump(event):
                frm, to = float(scale.cget('from')), float(scale.cget('to'))
                width = scale.winfo_width()
                if width > 1:
                    frac = min(1.0, max(0.0, event.x / width))
                    scale.set(frm + frac * (to - frm))
            scale.bind('<Button-1>', _jump)

        def make_slider_row(parent, row, var, spec, on_apply):
            """One Step slider row: label, draggable/click-to-jump Scale,
            and a live numeric readout that's also directly editable
            (Enter or focus-out commits a typed value)."""
            ttk.Label(parent, text=spec['label']).grid(row=row, column=0, sticky="e", padx=(0, 6), pady=2)
            scale = ttk.Scale(parent, from_=spec['frm'], to=spec['to'], variable=var, orient="horizontal",
                               length=280, command=lambda _=None: on_apply())
            scale.grid(row=row, column=1, sticky="w", pady=2)
            bind_scale_click_to_jump(scale)

            entry_var = tk.StringVar(value=f"{var.get():.3g}")
            entry = ttk.Entry(parent, textvariable=entry_var, width=8, font=SLIDER_ENTRY_FONT)
            entry.grid(row=row, column=2, padx=(8, 0), pady=2)

            def sync_entry_from_var(*_):
                entry_var.set(f"{var.get():.3g}")

            def commit_entry(event=None):
                try:
                    val = float(entry_var.get())
                except ValueError:
                    sync_entry_from_var()  # invalid text -- revert to current value
                    return
                var.set(min(spec['to'], max(spec['frm'], val)))
                sync_entry_from_var()
                on_apply()

            var.trace_add('write', sync_entry_from_var)
            entry.bind('<Return>', commit_entry)
            entry.bind('<FocusOut>', commit_entry)

        # Slider specs may tag a `shared` group name (see STEP_DEFS's step2/
        # step3/step4 target_deg/Kp/Kd) so multiple Steps' widgets drive the
        # exact same tk.DoubleVar -- moving/typing into any one of them
        # updates all the others immediately, with no separate save/restore
        # logic needed.
        shared_slider_vars = {}

        def build_step6_tab(frame):
            """Step6's 3-mode panel (see STEP_DEFS['step6']['modes']): a
            radio button switches link.step6_mode, which swaps the entire
            slider set below it -- unlike every other step's fixed slider
            list, so it can't be built by the generic loop this is called
            from."""
            mode_var = tk.StringVar(value=link.step6_mode)

            mode_row = ttk.Frame(frame)
            mode_row.pack(fill="x", pady=(0, 8))
            ttk.Label(mode_row, text="調整方法:").pack(side="left")

            slider_frame = ttk.Frame(frame)

            def reflect_step4_values():
                # "反転版" only: copies Step4's CURRENT Kp/Kd/k1/k2 sliders
                # into Step6-inverted's own (same key names, pure coincidence
                # that's convenient here) -- a one-time snapshot copy, not a
                # live link, so it's a starting point to then adjust further,
                # not something that keeps tracking Step4 afterwards.
                #
                # k1/k2 (the gains ON the pendulum's own state) are negated
                # on copy -- the sign that damps the pendulum at hang-down
                # generally needs to flip to have any hope of counteracting
                # it near upright instead (see the earlier discussion on why
                # this mode is a "watch it fail and see why" exploration,
                # not a working stabilizer). Kp/Kd (the rotor's OWN
                # position-hold PD, not a pendulum-facing gain) are copied
                # as-is -- no comparable reason to flip those.
                step4_vars = step_slider_vars.get('step4', {})
                sign = {'Kp': 1.0, 'Kd': 1.0, 'k1': -1.0, 'k2': -1.0}
                for key in ('Kp', 'Kd', 'k1', 'k2'):
                    if key in step4_vars and key in step_slider_vars.get('step6', {}):
                        step_slider_vars['step6'][key].set(sign[key] * step4_vars[key].get())
                apply_step_sliders('step6')

            def render_step6_sliders():
                for w in slider_frame.winfo_children():
                    w.destroy()
                mode_def = STEP_DEFS['step6']['modes'][mode_var.get()]
                step_slider_vars['step6'] = {}
                for i, s in enumerate(mode_def['sliders']):
                    var = tk.DoubleVar(value=s['default'])
                    step_slider_vars['step6'][s['key']] = var
                    make_slider_row(slider_frame, i, var, s, apply_current_step_sliders)
                if mode_var.get() == 'inverted':
                    ttk.Button(slider_frame, text="Step4の値を反映",
                               command=reflect_step4_values).grid(
                        row=len(mode_def['sliders']), column=0, columnspan=3, pady=(6, 0))

            def on_step6_mode_change():
                link.step6_mode = mode_var.get()
                render_step6_sliders()
                apply_step_sliders('step6')

            for mode_key, mode_def in STEP_DEFS['step6']['modes'].items():
                ttk.Radiobutton(mode_row, text=mode_def['mode_label'], variable=mode_var,
                                 value=mode_key, command=on_step6_mode_change).pack(side="left", padx=(8, 0))

            slider_frame.pack(fill="x")
            render_step6_sliders()
            apply_step_sliders('step6')

        for step_key in STEP_ORDER:
            step_def = STEP_DEFS[step_key]
            frame = ttk.Frame(content_container, padding=8)
            step_frames[step_key] = frame

            ttk.Label(frame, text=step_def['challenge'], wraplength=680,
                      justify="left", font=(UI_FONT[0], 14, "bold")).pack(fill="x", pady=(0, 8))

            if step_key == 'step6':
                # 3 distinct slider sets behind a mode radio button (see
                # STEP_DEFS['step6']['modes']) instead of one fixed set --
                # handled entirely separately from the generic per-step loop
                # below.
                build_step6_tab(frame)
                continue

            slider_frame = ttk.Frame(frame)
            slider_frame.pack(fill="x")
            step_slider_vars[step_key] = {}
            for i, s in enumerate(step_def['sliders']):
                tag = s.get('shared')
                if tag is not None:
                    if tag not in shared_slider_vars:
                        shared_slider_vars[tag] = tk.DoubleVar(value=s['default'])
                    var = shared_slider_vars[tag]
                else:
                    var = tk.DoubleVar(value=s['default'])
                step_slider_vars[step_key][s['key']] = var
                make_slider_row(slider_frame, i, var, s, apply_current_step_sliders)

            if step_key == 'step7':
                # Live readout of LQR_EQUIVALENT_PID (see its derivation
                # comment near recompute_lqr_gain()) -- updated in tick()
                # whenever the sliders above (re-)solve the LQR gain, so
                # students can watch the design process land on a concrete
                # PID before it's ever sent to hardware.
                step7_pid_var = tk.StringVar(value="換算PIDゲイン: (計算中)")
                ttk.Label(frame, textvariable=step7_pid_var, wraplength=680,
                          justify="left").pack(fill="x", pady=(10, 0))

            if step_def.get('has_controller_choice'):
                # Step8: reuse the existing advanced-panel PID/LQR selector
                # (balance_ctrl_var/on_balance_ctrl_change, defined above for
                # the "詳細設定" panel) so both stay in sync automatically.
                choice_frame = ttk.Frame(frame)
                choice_frame.pack(fill="x", pady=(8, 0))
                ttk.Label(choice_frame, text="上側(倒立後)の制御則:").pack(side="left")
                ttk.Radiobutton(choice_frame, text="PID (Step6)", variable=balance_ctrl_var,
                                value='pid', command=on_balance_ctrl_change).pack(side="left", padx=(8, 0))
                ttk.Radiobutton(choice_frame, text="LQR (Step7)", variable=balance_ctrl_var,
                                value='lqr', command=on_balance_ctrl_change).pack(side="left", padx=(8, 0))

        show_step(STEP_ORDER[0])

        def reset_current_step():
            step_key = active_step_var.get()
            if step_key == 'step6':
                mode_def = STEP_DEFS['step6']['modes'][link.step6_mode]
                for s in mode_def['sliders']:
                    step_slider_vars['step6'][s['key']].set(s['default'])
            else:
                for s in STEP_DEFS[step_key]['sliders']:
                    step_slider_vars[step_key][s['key']].set(s['default'])
            apply_step_sliders(step_key)

        button_row = ttk.Frame(school_frame)
        button_row.pack(fill="x", pady=(6, 0))
        ttk.Button(button_row, text="このStepの既定値にリセット",
                   command=reset_current_step).pack(side="left")

        capture_row = ttk.Frame(school_frame)
        capture_row.pack(fill="x", pady=(6, 0))

        def capture(slot):
            n = min(len(link.data[0]), 500)
            if n < 10:
                return
            stat = float(np.std(list(link.data[0])[-n:]))
            capture_stats[slot] = stat
            capture_label_vars[slot].set(f"記録{slot}: 揺れ幅(標準偏差) {stat:.2f}°")
            out_dir = LOG_DIR / "school_captures"
            out_dir.mkdir(parents=True, exist_ok=True)
            group = group_var.get().strip() or '0'
            fname = out_dir / f"{time.strftime('%Y%m%d_%H%M%S')}_group{group}_{link.current_step}_{slot}.png"
            fig.savefig(fname)

        ttk.Button(capture_row, text="記録A(変更前)", command=lambda: capture('A')).pack(side="left")
        ttk.Label(capture_row, textvariable=capture_label_vars['A']).pack(side="left", padx=(6, 20))
        ttk.Button(capture_row, text="記録B(変更後)", command=lambda: capture('B')).pack(side="left")
        ttk.Label(capture_row, textvariable=capture_label_vars['B']).pack(side="left", padx=(6, 20))

        def emergency_stop(event=None):
            link.stop_requested.set()

        ttk.Button(capture_row, text="緊急停止(Space/Esc)", command=emergency_stop).pack(side="right")
        root.bind('<space>', emergency_stop)
        root.bind('<Escape>', emergency_stop)

    # Bigger figure + larger fonts throughout (2026-07-20) -- fills the
    # left column, which now gets a full-height half of a much larger
    # window instead of whatever vertical space was left below a tall
    # stack of settings panels.
    plt.rcParams['font.size'] = 13
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(9, 9))
    # Pendulum Angle shows theta_p_perceived (Mode C: whichever origin Python
    # currently treats as 0, see _step_running() -- jumps by ~180 deg at the
    # exact moment origin_mode flips, which is expected, not a bug).
    labels = ["Pendulum Angle [deg]", "Rotor Angle [deg]", "Input u [steps/s²]"]
    lines = []
    for i in range(3):
        line, = ax[i].plot([], [], linewidth=1.8)
        lines.append(line)
        ax[i].set_ylabel(labels[i], fontsize=14)
        ax[i].tick_params(labelsize=12)
    ax[-1].set_xlabel("Sample", fontsize=14)

    target_line = None
    if school:
        # Step2/3 only (see LinkManager.target_data) -- actual-vs-target
        # overlay on the Rotor Angle axes.
        target_line, = ax[1].plot([], [], linestyle='--', color='tab:orange', linewidth=1.8, label='目標角度')
        ax[1].legend(loc='upper right', fontsize=11)

    fig.tight_layout()

    canvas = FigureCanvasTkAgg(fig, master=left_frame)
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

        if step7_pid_var is not None:
            p = LQR_EQUIVALENT_PID
            step7_pid_var.set(
                f"→ 換算PIDゲイン: Kp_pend={p['Kp_pend']:.1f}  Kd_pend={p['Kd_pend']:.1f}  "
                f"Kp_rotor={p['Kp_rotor']:.1f}  Kd_rotor={p['Kd_rotor']:.1f}")

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
        if target_line is not None:
            target_line.set_data(range(len(link.target_data)), list(link.target_data))
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
    parser.add_argument('--school', action='store_true',
                         help="simplified slider-driven GUI for high-school "
                              "outreach sessions (implies --gui); see "
                              "STEP_DEFS")
    args = parser.parse_args()

    if args.gui or args.school:
        run_gui(args.port, school=args.school)
    else:
        ser = connect_and_select_mode(args.port)
        run_cli(ser)


if __name__ == '__main__':
    main()
