"""
analysing_cap_v5.py — Autonomous Drone Navigation Stack
════════════════════════════════════════════════════════
Safety & Simulation Pass (v5) over the v4 production system.

Hardware platform (unchanged):
  • Arduino Nano         — flight controller, ALL flight logic
  • ESP32-CAM            — video streaming ONLY
  • MPU6050              — gyro + accelerometer (stub interface)
  • 4× brushed coreless motors
  • SI2300 MOSFET drivers
  • 3.7V LiPo battery
  • 5V regulator

What changed (v4 → v5):
  1.  Safe-test mode            — reduced PWM, sensitive ES, hover-preferred
  2.  Motor abstraction layer   — set_motor_speed(fl, fr, rl, rr) + DroneMixer
  3.  PID hooks (roll/pitch/yaw)— PlaceholderPID dataclass; MPU6050 stub
  4.  Virtual sensor framework  — VirtualSensorSuite (dist, alt, IMU, battery)
  5.  Extended simulation HUD   — virtual alt, velocity, PWM, IMU, battery
  6.  Motor safety limiter      — MAX_PWM_STEP, ramp_motor_pwm()
  7.  Serial safety (v4→v5)     — already solid; added stale-cmd TTL + log
  8.  AI stability filter       — command hold-time, confidence smoothing (v4 had stubs)
  9.  Hardware-ready comments   — ToF / ultrasonic / IMU-fusion / coord-nav markers
  10. Architecture preservation — YOLO, master arbiter, emergency FSM, obstacle
                                  memory, pseudo-depth, search FSM, motion
                                  primitives all kept verbatim.

Architecture layers (unchanged):

  PERCEPTION          camera frames  →  _frame_reader_loop / _yolo_loop
       ↓
  TARGET ESTIMATION   ai_decision()  →  AIIntent enum
       ↓
  AI INTENT           Layer 1        →  where is the target?
       ↓
  NAVIGATION REASONING nav_decision()→  NavState enum
       ↓
  MASTER ARBITER      master_decision() → single authority for motion
       ↓
  MOTION PRIMITIVES   MotionPrimitive enum → physical action
       ↓
  MOTOR ABSTRACTION   set_motor_speed()  → DroneMixer → PWM values   ← NEW v5
       ↓
  MOTOR SAFETY        ramp_motor_pwm()   → MAX_PWM_STEP limiter       ← NEW v5
       ↓
  HARDWARE ABSTRACTION AbstractFlightController → serial tokens
       ↓
  FLIGHT CONTROLLER   ArduinoController / DryRunController
"""

from __future__ import annotations

# ─────────────────────────────────────────────────────────────────────────────
#  Standard library
# ─────────────────────────────────────────────────────────────────────────────
import abc
import dataclasses
import json
import logging
import math
import os
import platform
import random
import re
import subprocess
import threading
import time
from collections import defaultdict, deque
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

# ─────────────────────────────────────────────────────────────────────────────
#  Third-party
# ─────────────────────────────────────────────────────────────────────────────
import cv2
import numpy as np
import torch
from ultralytics import YOLO


# ══════════════════════════════════════════════════════════════════════════════
#  RUNTIME MODES  (v4 preserved + SAFE_TEST_MODE is now first-class)
# ══════════════════════════════════════════════════════════════════════════════

class RuntimeMode(Enum):
    """
    Execution mode selector.

    SAFE_TEST_MODE   — reduced PWM, hover-preferred, ultra-sensitive ES
    SIMULATION_MODE  — no serial, full virtual sensors, full HUD
    REAL_FLIGHT_MODE — serial enabled, full safety, production behaviour
    DEBUG_MODE       — verbose logs, extra HUD fields
    LOW_POWER_MODE   — reduced inference resolution, frame-skipping
    """
    SAFE_TEST_MODE   = auto()
    SIMULATION_MODE  = auto()
    REAL_FLIGHT_MODE = auto()
    DEBUG_MODE       = auto()
    LOW_POWER_MODE   = auto()


# Active runtime mode — change this line before deploying.
ACTIVE_MODE: RuntimeMode = RuntimeMode.SAFE_TEST_MODE


def _mode_allows_serial() -> bool:
    return ACTIVE_MODE == RuntimeMode.REAL_FLIGHT_MODE

def _mode_is_debug() -> bool:
    return ACTIVE_MODE == RuntimeMode.DEBUG_MODE

def _mode_is_low_power() -> bool:
    return ACTIVE_MODE == RuntimeMode.LOW_POWER_MODE

def _mode_is_safe_test() -> bool:
    """True in SAFE_TEST_MODE — activates all conservative overrides."""
    return ACTIVE_MODE == RuntimeMode.SAFE_TEST_MODE

def _mode_is_simulation() -> bool:
    return ACTIVE_MODE == RuntimeMode.SIMULATION_MODE


# ══════════════════════════════════════════════════════════════════════════════
#  IMPROVEMENT 7 — EVENT LOGGER  (unchanged from v4)
# ══════════════════════════════════════════════════════════════════════════════

class DroneEventLogger:
    """Structured, lightweight event logger. Console always; file optional."""

    CATEGORIES = frozenset({
        "EMERGENCY", "RECONNECT", "OWNERSHIP",
        "OBSTACLE", "SERIAL", "WATCHDOG", "PERF", "MOTOR", "SENSOR",
    })

    def __init__(self, file_path: Optional[str] = None):
        self._lock      = threading.Lock()
        self._file_path = file_path
        self._fh        = None
        self._logger    = logging.getLogger("DroneNav")
        if not self._logger.handlers:
            handler = logging.StreamHandler()
            handler.setFormatter(logging.Formatter(
                "[%(asctime)s] %(name)s %(levelname)s %(message)s",
                datefmt="%H:%M:%S",
            ))
            self._logger.addHandler(handler)
            self._logger.setLevel(logging.DEBUG if _mode_is_debug() else logging.INFO)
        if file_path:
            try:
                self._fh = open(file_path, "a", buffering=1)
            except OSError as exc:
                self._logger.warning(f"Cannot open log file {file_path}: {exc}")

    def log(self, category: str, message: str, **fields) -> None:
        if category not in self.CATEGORIES:
            category = "GENERAL"
        record = {"ts": time.time(), "cat": category, "msg": message, **fields}
        self._logger.info(f"[{category}] {message}" + (f" | {fields}" if fields else ""))
        if self._fh:
            with self._lock:
                try:
                    self._fh.write(json.dumps(record) + "\n")
                except OSError:
                    pass

    def close(self) -> None:
        if self._fh:
            try:
                self._fh.close()
            except OSError:
                pass


event_log = DroneEventLogger(file_path=None)





# ══════════════════════════════════════════════════════════════════════════════
#  IMPROVEMENT 3 — CENTRALIZED CONFIG SYSTEM  (v4 preserved + v5 additions)
# ══════════════════════════════════════════════════════════════════════════════

@dataclasses.dataclass
class DroneConfig:
    """
    Single tuning location for every threshold, timing, and YOLO parameter.

    v5 additions
    ─────────────
    safe_test_pwm_max       — PWM ceiling in SAFE_TEST_MODE (0-255)
    safe_test_es_frac       — reduced emergency area fraction for safe-test
    safe_test_search_delay  — slow search interval multiplier in safe-test
    max_pwm_step            — maximum per-tick PWM change (motor ramp limiter)
    virtual_sensor_noise    — std-dev noise on virtual sensor readings
    stale_cmd_ttl           — seconds before a queued serial command is discarded
    """

    # ── Proximity thresholds ──────────────────────────────────────────────
    front_danger_frac:          float = 0.10
    side_danger_frac:           float = 0.05
    human_hover_frac:           float = 0.08
    human_center_overlap_min:   float = 0.40
    emergency_area_frac:        float = 0.68   # v6: was 0.60 — less aggressive ES trigger
    safe_release_frac:          float = 0.60

    # ── Speed tier thresholds ─────────────────────────────────────────────
    speed_clear_frac:           float = 0.04
    speed_caution_frac:         float = 0.07

    # ── Zone boundaries ───────────────────────────────────────────────────
    left_zone_end:              float = 0.30
    right_zone_start:           float = 0.70
    top_zone_end:               float = 0.30
    bottom_zone_start:          float = 0.70

    # ── Obstacle density ──────────────────────────────────────────────────
    center_density_limit:       int   = 3

    # ── Temporal memory ───────────────────────────────────────────────────
    last_obstacle_timeout:      float = 0.5

    # ── Search FSM timing ─────────────────────────────────────────────────
    search_phase_duration:      float = 2.5
    forward_scan_duration:      float = 0.5
    search_recovery_cycles:     int   = 4

    # ── Emergency timing ──────────────────────────────────────────────────
    emergency_hold_duration:    float = 2.5
    emergency_resend_interval:  float = 0.8
    oscillation_guard_window:   float = 1.5   # v6: was 1.0 — wider window to count real osc.
    oscillation_guard_limit:    int   = 12    # v6: was 8  — harder to trigger osc. emergency

    # ── Serial / heartbeat ────────────────────────────────────────────────
    command_send_interval:      float = 0.1
    heartbeat_interval:         float = 0.5
    serial_port:                str   = "COM5"
    serial_baud:                int   = 115200

    # ── Stale-frame / reader fail-safe ────────────────────────────────────
    stale_frame_timeout:        float = 1.0
    reader_fail_hold:           float = 1.5

    # ── Master arbiter cooldown ───────────────────────────────────────────
    command_hold_time:          float = 0.55   # v6: was 0.35 — longer dwell prevents flicker

    # ── Stability filters ─────────────────────────────────────────────────
    nav_stability_min:          int   = 5      # v6: was 3 — nav needs 5 consecutive agrees
    ai_stability_min:           int   = 5      # v6: was 3 — AI intent needs 5 consecutive
    ai_center_thresh:           float = 0.22   # v6: was 0.18 — wider dead-band stops L↔R jitter

    # ── Obstacle EMA ──────────────────────────────────────────────────────
    depth_ema_alpha:            float = 0.15   # v6: was 0.45 — heavier smoothing on proximity

    # ── YOLO / camera ─────────────────────────────────────────────────────
    model:                      str   = "yolov8n.pt"
    imgsz:                      int   = 416
    conf:                       float = 0.30
    iou:                        float = 0.45
    half:                       bool  = False
    device:                     str   = "cpu"
    cam_w:                      int   = 640
    cam_h:                      int   = 480
    display_scale:              float = 1.0
    tier:                       str   = "CPU"

    # ── Human count throttle ──────────────────────────────────────────────
    human_count_write_interval: float = 0.5

    # ── Adaptive inference ────────────────────────────────────────────────
    adaptive_min_inference_interval: float = 0.0

    # ── Track smoothing ───────────────────────────────────────────────────
    smooth_frames:              int   = 1

    # ── Hardware classifiers ──────────────────────────────────────────────
    human_class:                int   = 0
    navigation_classes: frozenset = dataclasses.field(
        default_factory=lambda: frozenset({0, 56, 57, 58, 59, 60, 62, 63})
    )

    # ─────────────────────────────────────────────────────────────────────
    #  v5 NEW FIELDS
    # ─────────────────────────────────────────────────────────────────────

    # ── IMPROVEMENT 1: Safe-test mode overrides ───────────────────────────
    safe_test_pwm_max:          int   = 120   # max PWM 0-255 in safe-test
    safe_test_es_frac:          float = 0.78  # v7: raised from 0.35 — was far too sensitive
    safe_test_search_delay:     float = 2.0   # extra seconds added to search phase

    # ── IMPROVEMENT 6: Motor safety limiter ──────────────────────────────
    max_pwm_step:               int   = 20    # max change per ramp tick
    pwm_ramp_interval:          float = 0.05  # seconds between ramp ticks

    # ── IMPROVEMENT 4: Virtual sensor noise level ─────────────────────────
    virtual_sensor_noise:       float = 0.02  # std-dev of Gaussian noise

    # ── IMPROVEMENT 7: Stale-command TTL ─────────────────────────────────
    stale_cmd_ttl:              float = 0.5   # discard queued cmds older than this

    # ── v7 EMERGENCY STABILITY FIELDS ─────────────────────────────────────
    # These six fields replace the single EMERGENCY_AREA_FRAC trigger with a
    # confirmation buffer + hysteresis + centre-corridor validation system.
    emergency_confirm_frames:   int   = 6     # consecutive CENTER-danger frames before ES
    emergency_enter_frac:       float = 0.78  # proximity threshold to START emergency
    emergency_exit_frac:        float = 0.42  # proximity to END emergency (hysteresis gap)
    min_obstacle_area_frac:     float = 0.008 # ignore YOLO boxes < 0.8% of frame area
    max_obstacle_aspect_ratio:  float = 4.0   # ignore very wide/tall boxes (walls/ceilings)
    proximity_ema_live:         float = 0.30  # EMA alpha for tid=-1 live proximity path

    # ─────────────────────────────────────────────────────────────────────
    #  v6 STABILITY FIELDS  (no new subsystems — tuning only)
    # ─────────────────────────────────────────────────────────────────────

    # Fix 5: frames-without-target before entering SEARCH (prevents flicker)
    search_enter_delay:         int   = 6     # missed-target frames before SEARCH

    # Fix 4: minimum frames holding a LEFT/RIGHT intent before it can flip
    lr_switch_cooldown:         int   = 6     # frames same L/R intent needed to switch

    # Fix 7: minimum frames a FORWARD intent must persist before committing
    forward_stability_min:      int   = 4     # frames for stable FORWARD commit

    # Fix 6: secondary proximity EMA applied in analyse_obstacles (global)
    obs_smooth_alpha:           float = 0.30  # extra per-tick smoothing on stored proximity

    # Fix 8: frames of nav CAUTION/CLEAR before AI can override with forward motion
    nav_override_guard:         int   = 3     # nav must be stable before AI forward allowed

    # ─────────────────────────────────────────────────────────────────────
    #  v8 REAL-WORLD ROBUSTNESS FIELDS
    # ─────────────────────────────────────────────────────────────────────
    # IMPROVEMENT 1: bbox stability — max IoU-frame-delta before box is
    # considered "flickering".  0.0 = perfectly still; 1.0 = fully replaced.
    bbox_stability_jitter_thresh:  float = 0.55   # displacement fraction of box size
    bbox_stability_history:        int   = 5      # frames kept per tracked box
    # IMPROVEMENT 2: approach velocity — minimum fractional area growth per
    # second to count as "genuinely approaching".
    approach_min_growth_rate:      float = 0.04   # 4% area growth/s = real approach
    # IMPROVEMENT 3: confidence weighting — low-confidence detections get
    # their proximity score scaled down.
    conf_weight_floor:             float = 0.40   # conf below this → full penalty
    conf_weight_ceil:              float = 0.70   # conf above this → no penalty
    # IMPROVEMENT 4: motion-blur guard — if the single-frame bbox area jumps
    # by more than this fraction, treat as a blur spike and suppress ES.
    blur_area_explosion_thresh:    float = 2.5    # 2.5× area expansion in one frame
    blur_suppression_decay:        float = 0.25   # suppression decays by this per frame
    # IMPROVEMENT 6: danger confidence accumulator — slow build / slow decay
    danger_conf_build_rate:        float = 0.18   # added per frame when dangerous
    danger_conf_decay_rate:        float = 0.08   # subtracted per frame when not
    danger_conf_threshold:         float = 0.72   # must reach this to trigger ES
    # IMPROVEMENT 7: camera-shake — optical-flow magnitude above this (px/frame)
    # enables shake-suppression mode; magnitude below exits it.
    shake_flow_enter:              float = 14.0   # px/frame → suppression on
    shake_flow_exit:               float = 6.0    # px/frame → suppression off
    shake_suppression_factor:      float = 0.55   # multiply ES proximity weight

    # ─────────────────────────────────────────────────────────────────────
    #  Hardware profiles (unchanged)
    # ─────────────────────────────────────────────────────────────────────

    @classmethod
    def high_end_gpu(cls) -> "DroneConfig":
        return cls(model="yolov8x.pt", imgsz=960, conf=0.30, iou=0.45,
                   half=True, device="cuda", cam_w=1920, cam_h=1080,
                   tier="High-end GPU (CUDA)")

    @classmethod
    def mid_range_gpu(cls) -> "DroneConfig":
        return cls(model="yolov8m.pt", imgsz=640, conf=0.25, iou=0.45,
                   half=True, device="cuda", cam_w=1280, cam_h=720,
                   tier="Mid-range GPU (CUDA)")

    @classmethod
    def low_vram_gpu(cls) -> "DroneConfig":
        return cls(model="yolov8s.pt", imgsz=640, conf=0.25, iou=0.45,
                   half=True, device="cuda", cam_w=1280, cam_h=720,
                   tier="Low-VRAM GPU (CUDA)")

    @classmethod
    def cpu_profile(cls, device_name: str = "CPU") -> "DroneConfig":
        return cls(model="yolov8n.pt", imgsz=416, conf=0.30, iou=0.45,
                   half=False, device="cpu", cam_w=640, cam_h=480,
                   tier=f"CPU ({device_name})")


# ══════════════════════════════════════════════════════════════════════════════
#  IMPROVEMENT 1 — STRICT TYPE SAFETY  (unchanged from v4)
# ══════════════════════════════════════════════════════════════════════════════

class AIIntent(Enum):
    TRACK_LEFT    = "AI_TRACK_LEFT"
    TRACK_RIGHT   = "AI_TRACK_RIGHT"
    TRACK_CENTER  = "AI_TRACK_CENTER"
    HOVER         = "AI_HOVER"
    SEARCH_TARGET = "AI_SEARCH_TARGET"


class NavState(Enum):
    CLEAR          = "NAV_CLEAR_PATH"
    CAUTION        = "NAV_CAUTION"
    BLOCKED_FRONT  = "NAV_BLOCKED_FRONT"
    BLOCKED_LEFT   = "NAV_BLOCKED_LEFT"
    BLOCKED_RIGHT  = "NAV_BLOCKED_RIGHT"
    DENSE          = "NAV_DENSE_OBSTACLES"
    CEILING        = "NAV_CEILING_THREAT"
    SEARCH         = "NAV_SEARCH_MODE"
    EMERGENCY      = "NAV_EMERGENCY"


class MotionPrimitive(Enum):
    FORWARD        = "MOVE_FORWARD"
    FORWARD_FAST   = "MOVE_FORWARD_FAST"
    FORWARD_SLOW   = "MOVE_FORWARD_SLOW"
    BACKWARD       = "MOVE_BACKWARD"
    YAW_LEFT       = "MOVE_YAW_LEFT"
    YAW_RIGHT      = "MOVE_YAW_RIGHT"
    HOVER          = "MOVE_HOVER"
    STOP           = "MOVE_STOP"
    SEARCH_LEFT    = "MOVE_SEARCH_LEFT"
    SEARCH_RIGHT   = "MOVE_SEARCH_RIGHT"
    SCAN_FORWARD   = "MOVE_SCAN_FORWARD"
    SAFE_SEARCH    = "MOVE_SAFE_SEARCH"
    EMERGENCY_STOP = "MOVE_EMERGENCY_STOP"
    BACKOFF        = "MOVE_BACKOFF"          # collision-emergency reverse thrust


MOTION_TOKENS: Dict[MotionPrimitive, str] = {
    MotionPrimitive.FORWARD        : "F",
    MotionPrimitive.FORWARD_FAST   : "FF",
    MotionPrimitive.FORWARD_SLOW   : "SF",
    MotionPrimitive.BACKWARD       : "B",
    MotionPrimitive.YAW_LEFT       : "YL",
    MotionPrimitive.YAW_RIGHT      : "YR",
    MotionPrimitive.HOVER          : "H",
    MotionPrimitive.STOP           : "ST",
    MotionPrimitive.SEARCH_LEFT    : "SL",
    MotionPrimitive.SEARCH_RIGHT   : "SR",
    MotionPrimitive.SCAN_FORWARD   : "FS",
    MotionPrimitive.SAFE_SEARCH    : "SS",
    MotionPrimitive.EMERGENCY_STOP : "ES",
    MotionPrimitive.BACKOFF        : "E",    # collision-emergency token
}


# ══════════════════════════════════════════════════════════════════════════════
#  IMPROVEMENT 2 — STRUCTURED FSM TRANSITIONS  (unchanged from v4)
# ══════════════════════════════════════════════════════════════════════════════

MOTION_PRIORITY: Dict[MotionPrimitive, int] = {
    MotionPrimitive.EMERGENCY_STOP : 0,
    MotionPrimitive.HOVER          : 1,
    MotionPrimitive.BACKWARD       : 2,
    MotionPrimitive.YAW_LEFT       : 3,
    MotionPrimitive.YAW_RIGHT      : 3,
    MotionPrimitive.STOP           : 4,
    MotionPrimitive.FORWARD_SLOW   : 5,
    MotionPrimitive.SAFE_SEARCH    : 5,
    MotionPrimitive.FORWARD_FAST   : 6,
    MotionPrimitive.FORWARD        : 6,
    MotionPrimitive.SCAN_FORWARD   : 6,
    MotionPrimitive.SEARCH_LEFT    : 7,
    MotionPrimitive.SEARCH_RIGHT   : 7,
}

_ALWAYS_ALLOWED_TARGETS: frozenset = frozenset({
    MotionPrimitive.EMERGENCY_STOP,
    MotionPrimitive.BACKWARD,
})

_FORBIDDEN_TRANSITIONS: frozenset = frozenset({
    (MotionPrimitive.BACKWARD, MotionPrimitive.FORWARD_FAST),
})


def _validate_transition(current: MotionPrimitive,
                          candidate: MotionPrimitive) -> bool:
    if candidate == current:
        return True
    if candidate in _ALWAYS_ALLOWED_TARGETS:
        return True
    if (current, candidate) in _FORBIDDEN_TRANSITIONS:
        return False
    curr_rank = MOTION_PRIORITY.get(current, 99)
    cand_rank = MOTION_PRIORITY.get(candidate, 99)
    return cand_rank <= curr_rank or curr_rank >= 6


# ── Legacy string constants ───────────────────────────────────────────────────
AI_TRACK_LEFT    = AIIntent.TRACK_LEFT.value
AI_TRACK_RIGHT   = AIIntent.TRACK_RIGHT.value
AI_TRACK_CENTER  = AIIntent.TRACK_CENTER.value
AI_HOVER         = AIIntent.HOVER.value
AI_SEARCH_TARGET = AIIntent.SEARCH_TARGET.value

NAV_STATE_CLEAR         = NavState.CLEAR.value
NAV_STATE_CAUTION       = NavState.CAUTION.value
NAV_STATE_BLOCKED_FRONT = NavState.BLOCKED_FRONT.value
NAV_STATE_BLOCKED_LEFT  = NavState.BLOCKED_LEFT.value
NAV_STATE_BLOCKED_RIGHT = NavState.BLOCKED_RIGHT.value
NAV_STATE_DENSE         = NavState.DENSE.value
NAV_STATE_CEILING       = NavState.CEILING.value
NAV_STATE_SEARCH        = NavState.SEARCH.value
NAV_STATE_EMERGENCY     = NavState.EMERGENCY.value

MOVE_FORWARD        = MotionPrimitive.FORWARD.value
MOVE_FORWARD_FAST   = MotionPrimitive.FORWARD_FAST.value
MOVE_FORWARD_SLOW   = MotionPrimitive.FORWARD_SLOW.value
MOVE_BACKWARD       = MotionPrimitive.BACKWARD.value
MOVE_YAW_LEFT       = MotionPrimitive.YAW_LEFT.value
MOVE_YAW_RIGHT      = MotionPrimitive.YAW_RIGHT.value
MOVE_HOVER          = MotionPrimitive.HOVER.value
MOVE_STOP           = MotionPrimitive.STOP.value
MOVE_SEARCH_LEFT    = MotionPrimitive.SEARCH_LEFT.value
MOVE_SEARCH_RIGHT   = MotionPrimitive.SEARCH_RIGHT.value
MOVE_SCAN_FORWARD   = MotionPrimitive.SCAN_FORWARD.value
MOVE_SAFE_SEARCH    = MotionPrimitive.SAFE_SEARCH.value
MOVE_EMERGENCY_STOP = MotionPrimitive.EMERGENCY_STOP.value
MOVE_BACKOFF        = MotionPrimitive.BACKOFF.value   # collision-emergency backoff

NAV_FAST_FORWARD   = "FAST_FORWARD"
NAV_SLOW_FORWARD   = "SLOW_FORWARD"
NAV_STOP           = "STOP"
NAV_SEARCH_LEFT    = "SEARCH_LEFT"
NAV_SEARCH_RIGHT   = "SEARCH_RIGHT"
NAV_FORWARD_SCAN   = "FORWARD_SCAN"
NAV_SAFE_SEARCH    = "SAFE_SEARCH"
NAV_FORWARD        = "FORWARD"
NAV_BACKWARD       = "BACKWARD"
NAV_AVOID_LEFT     = "AVOID_LEFT"
NAV_AVOID_RIGHT    = "AVOID_RIGHT"
NAV_HOVER          = "HOVER"
NAV_SEARCH         = "SEARCH"
NAV_EMERGENCY_STOP = "EMERGENCY_STOP"

STATE_IDLE      = "IDLE"
STATE_TRACKING  = "TRACKING"
STATE_SEARCHING = "SEARCHING"

OWNER_EMERGENCY  = "EMERGENCY"
OWNER_NAVIGATION = "NAVIGATION"
OWNER_AI         = "AI_TRACKING"
OWNER_SEARCH     = "SEARCH"

ARDUINO_TOKENS: Dict[str, str] = {
    NAV_FORWARD: "F", NAV_BACKWARD: "B", NAV_AVOID_LEFT: "YL",
    NAV_AVOID_RIGHT: "YR", NAV_HOVER: "H", NAV_SEARCH: "SS",
    NAV_FAST_FORWARD: "FF", NAV_SLOW_FORWARD: "SF", NAV_STOP: "ST",
    NAV_SEARCH_LEFT: "SL", NAV_SEARCH_RIGHT: "SR", NAV_FORWARD_SCAN: "FS",
    NAV_SAFE_SEARCH: "SS", NAV_EMERGENCY_STOP: "ES",
    MOVE_FORWARD: "F", MOVE_FORWARD_FAST: "FF", MOVE_FORWARD_SLOW: "SF",
    MOVE_BACKWARD: "B", MOVE_YAW_LEFT: "YL", MOVE_YAW_RIGHT: "YR",
    MOVE_HOVER: "H", MOVE_STOP: "ST", MOVE_SEARCH_LEFT: "SL",
    MOVE_SEARCH_RIGHT: "SR", MOVE_SCAN_FORWARD: "FS",
    MOVE_SAFE_SEARCH: "SS", MOVE_EMERGENCY_STOP: "ES",
    MOVE_BACKOFF: "E",   # collision-emergency token
}

ARDUINO_TOKENS_V3: Dict[str, str] = {
    NAV_FAST_FORWARD: "FF", NAV_SLOW_FORWARD: "SF", NAV_STOP: "ST",
}

_SEARCH_CYCLE = [NAV_SEARCH_LEFT, NAV_SEARCH_RIGHT, NAV_FORWARD_SCAN]

_OSCILLATION_PAIRS: frozenset = frozenset({
    frozenset({NAV_AVOID_LEFT,   NAV_AVOID_RIGHT}),
    frozenset({NAV_FAST_FORWARD, NAV_BACKWARD}),
    frozenset({NAV_SLOW_FORWARD, NAV_BACKWARD}),
    frozenset({NAV_FORWARD,      NAV_BACKWARD}),
    frozenset({NAV_STOP,         NAV_BACKWARD}),
})


# ══════════════════════════════════════════════════════════════════════════════
#  IMPROVEMENT 4 — SENSOR FUSION INTERFACES  (unchanged from v4)
# ══════════════════════════════════════════════════════════════════════════════

@dataclasses.dataclass
class SensorReading:
    sensor_id: str
    timestamp: float
    value:     float
    unit:      str  = "m"
    valid:     bool = True


class AbstractSensorFusion(abc.ABC):
    @abc.abstractmethod
    def update(self, reading: SensorReading) -> None: ...
    @abc.abstractmethod
    def get_proximity(self, zone: str) -> Optional[float]: ...
    @abc.abstractmethod
    def get_altitude(self) -> Optional[float]: ...
    @abc.abstractmethod
    def get_velocity(self) -> Optional[Tuple[float, float, float]]: ...


class NullSensorFusion(AbstractSensorFusion):
    def update(self, reading: SensorReading) -> None: pass
    def get_proximity(self, zone: str) -> Optional[float]: return None
    def get_altitude(self) -> Optional[float]: return None
    def get_velocity(self) -> Optional[Tuple[float, float, float]]: return None


# ──────────────────────────────────────────────────────────────────────────────
# HARDWARE-READY COMMENT: ToF / Ultrasonic hook point
# When VL53L0X or HC-SR04 is added:
#   1. Create a concrete AbstractSensorFusion subclass (e.g. UltrasonicFusion).
#   2. Replace sensor_fusion singleton below with your implementation.
#   3. Uncomment the sensor_prox blend block in estimate_pseudo_depth_v3().
#   4. Zones: LEFT / CENTER / RIGHT match the horizontal classifier output.
# ──────────────────────────────────────────────────────────────────────────────
sensor_fusion: AbstractSensorFusion = NullSensorFusion()


# ══════════════════════════════════════════════════════════════════════════════
#  v5 IMPROVEMENT 4 — VIRTUAL SENSOR FRAMEWORK
#  Simulates obstacle distance, altitude, IMU drift, and battery state so the
#  full safety stack can be exercised WITHOUT any physical sensors connected.
#  All outputs are flagged as synthetic; they do NOT reach the serial port.
# ══════════════════════════════════════════════════════════════════════════════

@dataclasses.dataclass
class VirtualSensorState:
    """Live state of the virtual sensor suite (read-only for display)."""
    obstacle_dist_m:  float = 2.0    # simulated distance to nearest obstacle (m)
    altitude_m:       float = 0.0    # simulated altitude (m)
    velocity_x:       float = 0.0    # simulated forward velocity (m/s)
    velocity_y:       float = 0.0    # simulated lateral velocity (m/s)
    battery_voltage:  float = 3.7    # simulated cell voltage (V)
    battery_pct:      int   = 100    # derived state-of-charge %
    imu_roll_deg:     float = 0.0    # MPU6050 simulated roll (°)
    imu_pitch_deg:    float = 0.0    # MPU6050 simulated pitch (°)
    imu_yaw_deg:      float = 0.0    # MPU6050 simulated yaw (°)
    battery_warning:  bool  = False
    obstacle_warning: bool  = False


class VirtualSensorSuite:
    """
    Simulation-mode virtual sensor framework.

    Produces plausible noisy sensor readings so that the full navigation
    and safety stack runs as if sensors were physically present.

    Usage
    -----
    virtual_sensors.tick(dt, motion_primitive_str)  — call every display frame
    virtual_sensors.state                            — read current values

    HARDWARE-READY COMMENT: IMU (MPU6050)
    When MPU6050 is wired to the Arduino Nano (I²C SDA=A4, SCL=A5):
      1. Add the MPU6050 Arduino library (I2Cdev / Jeff Rowberg).
      2. Stream roll/pitch/yaw angles over serial alongside motor tokens.
      3. Parse them in _parse_imu_line() in ArduinoController.
      4. Replace virtual_sensors.state.imu_* with real parsed values.
      5. Feed real roll/pitch into pid_roll.compute() / pid_pitch.compute().

    HARDWARE-READY COMMENT: Coordinate Navigation
    When GPS or external positioning (UWB, AprilTag, etc.) is available:
      1. Add a CoordinateNavigator class with waypoint queue.
      2. Replace the search FSM with coordinate-driven nav.
      3. Fuse position into the master arbiter as a new priority layer.
    """

    def __init__(self, noise_std: float = 0.02):
        self._noise_std    = noise_std
        self._lock         = threading.Lock()
        self.state         = VirtualSensorState()
        self._t_last        = time.time()
        self._alt_vel       = 0.0       # vertical velocity m/s
        self._discharge_rate= 0.001     # V/s at full throttle

    def _noisy(self, value: float) -> float:
        return value + random.gauss(0.0, self._noise_std)

    def tick(self, motion: str) -> None:
        """Advance virtual sensor state by one simulation tick."""
        now = time.time()
        dt  = min(now - self._t_last, 0.2)
        self._t_last = now

        with self._lock:
            s = self.state

            # ── Altitude simulation ───────────────────────────────────────
            if motion in (MOVE_FORWARD, MOVE_FORWARD_FAST, MOVE_FORWARD_SLOW,
                          MOVE_HOVER, MOVE_YAW_LEFT, MOVE_YAW_RIGHT):
                self._alt_vel = min(self._alt_vel + 0.3 * dt, 0.5)
            elif motion in (MOVE_EMERGENCY_STOP, MOVE_STOP, MOVE_BACKWARD):
                self._alt_vel = max(self._alt_vel - 0.6 * dt, -0.5)
            else:
                self._alt_vel *= 0.9  # decay towards hover

            s.altitude_m = max(0.0, self._noisy(s.altitude_m + self._alt_vel * dt))

            # ── Velocity estimate ─────────────────────────────────────────
            if motion == MOVE_FORWARD_FAST:
                s.velocity_x = self._noisy(min(s.velocity_x + 0.4 * dt, 1.0))
            elif motion == MOVE_FORWARD_SLOW:
                s.velocity_x = self._noisy(min(s.velocity_x + 0.2 * dt, 0.6))
            elif motion == MOVE_BACKWARD:
                s.velocity_x = self._noisy(max(s.velocity_x - 0.5 * dt, -0.5))
            else:
                s.velocity_x = self._noisy(s.velocity_x * 0.85)

            if motion == MOVE_YAW_LEFT:
                s.velocity_y = self._noisy(max(s.velocity_y - 0.3 * dt, -0.5))
            elif motion == MOVE_YAW_RIGHT:
                s.velocity_y = self._noisy(min(s.velocity_y + 0.3 * dt, 0.5))
            else:
                s.velocity_y = self._noisy(s.velocity_y * 0.85)

            # ── IMU drift simulation (MPU6050 placeholder) ────────────────
            # HARDWARE-READY COMMENT: replace these random-walk values with
            # real MPU6050 readings when the sensor is available.
            s.imu_roll_deg  = self._noisy(s.imu_roll_deg  * 0.98)
            s.imu_pitch_deg = self._noisy(s.imu_pitch_deg * 0.98)
            s.imu_yaw_deg   = (s.imu_yaw_deg + random.gauss(0, 0.1)) % 360

            # ── Obstacle distance simulation ──────────────────────────────
            # Shrinks when moving forward, grows when backing up.
            if motion in (MOVE_FORWARD, MOVE_FORWARD_FAST):
                s.obstacle_dist_m = max(0.3, self._noisy(s.obstacle_dist_m - 0.05))
            elif motion == MOVE_BACKWARD:
                s.obstacle_dist_m = min(5.0, self._noisy(s.obstacle_dist_m + 0.08))
            else:
                s.obstacle_dist_m = self._noisy(s.obstacle_dist_m)
            s.obstacle_warning = s.obstacle_dist_m < 0.5

            # ── Battery simulation ────────────────────────────────────────
            throttle = 1.0 if motion == MOVE_FORWARD_FAST else 0.5
            s.battery_voltage  = max(3.0,
                s.battery_voltage - self._discharge_rate * throttle * dt)
            s.battery_pct      = max(0, int(
                (s.battery_voltage - 3.0) / (4.2 - 3.0) * 100))
            s.battery_warning  = s.battery_voltage < 3.4

    def snapshot(self) -> VirtualSensorState:
        with self._lock:
            return dataclasses.replace(self.state)


# Module-level virtual sensor singleton (active in SIMULATION and SAFE_TEST modes)
virtual_sensors = VirtualSensorSuite()


# ══════════════════════════════════════════════════════════════════════════════
#  v5 IMPROVEMENT 2 — MOTOR ABSTRACTION LAYER
#  set_motor_speed(fl, fr, rl, rr) is the single entry point for all PWM
#  writes.  DroneMixer translates MotionPrimitive → per-motor values.
#  The ramp limiter lives here so the serial layer never sees sudden spikes.
# ══════════════════════════════════════════════════════════════════════════════

@dataclasses.dataclass
class MotorState:
    """Current and target PWM for all four motors (0-255)."""
    fl: int = 0   # front-left
    fr: int = 0   # front-right
    rl: int = 0   # rear-left
    rr: int = 0   # rear-right


# Live motor state — updated by the ramp loop; readable by the HUD.
_motor_state  = MotorState()
_motor_target = MotorState()
_motor_lock   = threading.Lock()


def _clamp_pwm(value: int, safe_test: bool = False) -> int:
    """Clamp PWM to [0, max] respecting safe-test ceiling."""
    # IMPROVEMENT 6: safe-test ceiling applied here
    ceiling = cfg.safe_test_pwm_max if safe_test else 255
    return max(0, min(ceiling, int(value)))


def set_motor_speed(fl: int, fr: int, rl: int, rr: int) -> None:
    """
    Motor abstraction entry point.

    Parameters
    ----------
    fl, fr, rl, rr : desired PWM values (0-255)

    In SAFE_TEST_MODE values are clamped to safe_test_pwm_max.
    Actual PWM ramps toward these targets at MAX_PWM_STEP per tick
    via the background _motor_ramp_loop.

    FUTURE PID HOOK: feed pid_roll / pid_pitch corrections into
    per-motor trim offsets before calling set_motor_speed().
    """
    safe = _mode_is_safe_test()
    with _motor_lock:
        _motor_target.fl = _clamp_pwm(fl, safe)
        _motor_target.fr = _clamp_pwm(fr, safe)
        _motor_target.rl = _clamp_pwm(rl, safe)
        _motor_target.rr = _clamp_pwm(rr, safe)


# ── PWM values per MotionPrimitive ────────────────────────────────────────────
# Tune these for your specific motors / propeller combination.
# Format: (fl, fr, rl, rr)
_MOTION_PWM: Dict[str, Tuple[int, int, int, int]] = {
    MOVE_FORWARD        : (160, 160, 160, 160),
    MOVE_FORWARD_FAST   : (220, 220, 220, 220),
    MOVE_FORWARD_SLOW   : (130, 130, 130, 130),
    MOVE_BACKWARD       : (100, 100, 100, 100),
    MOVE_YAW_LEFT       : (110, 160, 110, 160),
    MOVE_YAW_RIGHT      : (160, 110, 160, 110),
    MOVE_HOVER          : (150, 150, 150, 150),
    MOVE_STOP           : (0,   0,   0,   0),
    MOVE_SEARCH_LEFT    : (105, 150, 105, 150),
    MOVE_SEARCH_RIGHT   : (150, 105, 150, 105),
    MOVE_SCAN_FORWARD   : (140, 140, 140, 140),
    MOVE_SAFE_SEARCH    : (100, 100, 100, 100),
    MOVE_EMERGENCY_STOP : (0,   0,   0,   0),
}


class DroneMixer:
    """
    Translates a MotionPrimitive string → per-motor PWM tuple and
    calls set_motor_speed().

    HARDWARE-READY COMMENT: Roll / Pitch / Yaw PID integration
    When pid_roll / pid_pitch / pid_yaw are implemented, add correction
    offsets here before the set_motor_speed() call:

        roll_corr  = pid_roll.compute(0.0, imu.roll_deg)
        pitch_corr = pid_pitch.compute(0.0, imu.pitch_deg)
        fl += pitch_corr - roll_corr
        fr += pitch_corr + roll_corr
        rl -= pitch_corr - roll_corr
        rr -= pitch_corr + roll_corr
        set_motor_speed(fl, fr, rl, rr)
    """

    @staticmethod
    def apply(motion: str) -> None:
        """Map a motion primitive string to motor PWM and submit."""
        pwm = _MOTION_PWM.get(motion, (0, 0, 0, 0))
        set_motor_speed(*pwm)


drone_mixer = DroneMixer()


# ──────────────────────────────────────────────────────────────────────────────
# IMPROVEMENT 6 — MOTOR SAFETY LIMITER
# Background thread ramps _motor_state toward _motor_target at MAX_PWM_STEP
# per tick to prevent sudden PWM spikes that could damage motors or throw
# the frame off balance.
# ──────────────────────────────────────────────────────────────────────────────

def ramp_motor_pwm(current: int, target: int, max_step: int) -> int:
    """Advance `current` toward `target` by at most `max_step`."""
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + (max_step if delta > 0 else -max_step)


def _motor_ramp_loop() -> None:
    """
    Background thread: smoothly ramps actual motor PWM toward targets.
    Runs while _reader_alive is set.
    """
    while _reader_alive.is_set():
        with _motor_lock:
            step = cfg.max_pwm_step
            _motor_state.fl = ramp_motor_pwm(_motor_state.fl, _motor_target.fl, step)
            _motor_state.fr = ramp_motor_pwm(_motor_state.fr, _motor_target.fr, step)
            _motor_state.rl = ramp_motor_pwm(_motor_state.rl, _motor_target.rl, step)
            _motor_state.rr = ramp_motor_pwm(_motor_state.rr, _motor_target.rr, step)
        time.sleep(cfg.pwm_ramp_interval)


def get_motor_pwm_snapshot() -> Tuple[int, int, int, int]:
    """Return current (ramped) PWM values for HUD display."""
    with _motor_lock:
        return (_motor_state.fl, _motor_state.fr,
                _motor_state.rl, _motor_state.rr)


# ══════════════════════════════════════════════════════════════════════════════
#  IMPROVEMENT 8 — HARDWARE ABSTRACTION LAYER  (unchanged from v4)
# ══════════════════════════════════════════════════════════════════════════════

class AbstractFlightController(abc.ABC):
    @abc.abstractmethod
    def send_token(self, token: str, force: bool = False) -> None: ...
    @abc.abstractmethod
    def close(self) -> None: ...
    @abc.abstractmethod
    def is_connected(self) -> bool: ...


class DryRunController(AbstractFlightController):
    def send_token(self, token: str, force: bool = False) -> None:
        if _mode_is_debug():
            print(f"[DryRun] token='{token}' force={force}")

    def close(self) -> None:
        pass

    def is_connected(self) -> bool:
        return True


# ══════════════════════════════════════════════════════════════════════════════
#  IMPROVEMENT 5 — REAL-TIME PERFORMANCE METRICS  (unchanged from v4)
# ══════════════════════════════════════════════════════════════════════════════

class PerformanceMonitor:
    def __init__(self):
        self._lock              = threading.Lock()
        self.yolo_inference_ms  = 0.0
        self.frame_latency_ms   = 0.0
        self.command_latency_ms = 0.0
        self.serial_latency_ms  = 0.0
        self.dropped_frames     = 0
        self.reconnect_count    = 0
        self.queue_depth        = 0
        self._inference_times: deque = deque(maxlen=60)

    def record_inference(self, ms: float) -> None:
        with self._lock:
            self._inference_times.append(ms)
            self.yolo_inference_ms = sum(self._inference_times) / len(self._inference_times)

    def record_frame_latency(self, ms: float) -> None:
        with self._lock:
            self.frame_latency_ms = ms

    def record_command_latency(self, ms: float) -> None:
        with self._lock:
            self.command_latency_ms = ms

    def record_serial_latency(self, ms: float) -> None:
        with self._lock:
            self.serial_latency_ms = ms

    def increment_dropped(self) -> None:
        with self._lock:
            self.dropped_frames += 1

    def increment_reconnect(self) -> None:
        with self._lock:
            self.reconnect_count += 1

    def snapshot(self) -> Dict[str, float]:
        with self._lock:
            return {
                "yolo_ms"    : round(self.yolo_inference_ms, 1),
                "latency_ms" : round(self.frame_latency_ms, 1),
                "cmd_ms"     : round(self.command_latency_ms, 1),
                "serial_ms"  : round(self.serial_latency_ms, 1),
                "dropped"    : self.dropped_frames,
                "reconnects" : self.reconnect_count,
            }


perf = PerformanceMonitor()


# ══════════════════════════════════════════════════════════════════════════════
#  IMPROVEMENT 9 — PID MOTION CONTROLLER HOOKS
#  v5 extends v4 stubs with roll and pitch axes.
# ══════════════════════════════════════════════════════════════════════════════

@dataclasses.dataclass
class PIDHook:
    """
    Single-axis PID stub.

    Integration guide
    -----------------
    1. Set kp / ki / kd from tuning experiments.
    2. Replace compute() body with standard PID formula.
    3. Call reset() when the controlled axis changes command.
    4. Feed output into DroneMixer offsets before set_motor_speed().

    HARDWARE-READY COMMENT: MPU6050 integration
    Roll measurement  → imu.roll_deg   (replace virtual_sensors.state.imu_roll_deg)
    Pitch measurement → imu.pitch_deg
    Yaw   measurement → imu.yaw_deg
    Sample rate should match _motor_ramp_loop interval (~50 Hz).
    """
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    _integral:   float = dataclasses.field(default=0.0, init=False)
    _prev_error: float = dataclasses.field(default=0.0, init=False)
    _last_t:     float = dataclasses.field(default_factory=time.time, init=False)

    def compute(self, setpoint: float, measurement: float) -> float:
        """Return PID output — currently a no-op (returns 0.0)."""
        # TODO: implement when real MPU6050 data is available
        # dt = time.time() - self._last_t
        # error = setpoint - measurement
        # self._integral += error * dt
        # derivative = (error - self._prev_error) / max(dt, 1e-6)
        # self._prev_error = error
        # self._last_t = time.time()
        # return self.kp * error + self.ki * self._integral + self.kd * derivative
        return 0.0

    def reset(self) -> None:
        self._integral   = 0.0
        self._prev_error = 0.0
        self._last_t     = time.time()


# Per-axis PID stubs (v5 adds roll + pitch)
pid_yaw     = PIDHook(kp=0.0, ki=0.0, kd=0.0)
pid_forward = PIDHook(kp=0.0, ki=0.0, kd=0.0)
pid_roll    = PIDHook(kp=0.0, ki=0.0, kd=0.0)   # NEW v5
pid_pitch   = PIDHook(kp=0.0, ki=0.0, kd=0.0)   # NEW v5


def _apply_pid_smoothing(primitive: MotionPrimitive,
                          target_offset: float = 0.0) -> MotionPrimitive:
    """
    PID smoothing hook — currently pass-through.
    FUTURE: query pid_yaw / pid_forward / pid_roll / pid_pitch here,
    adjust primitive or DroneMixer offsets accordingly.
    """
    return primitive


# ══════════════════════════════════════════════════════════════════════════════
#  BUILD ACTIVE CONFIG
# ══════════════════════════════════════════════════════════════════════════════

def _query_windows_gpu():
    try:
        out = subprocess.check_output(
            ["wmic", "path", "win32_VideoController", "get", "name"],
            stderr=subprocess.DEVNULL, timeout=4
        ).decode(errors="ignore")
        return [l.strip() for l in out.splitlines()
                if l.strip() and l.strip().lower() != "name"]
    except Exception:
        return []


def get_device():
    if torch.cuda.is_available():
        name    = torch.cuda.get_device_name(0)
        vram_gb = torch.cuda.get_device_properties(0).total_memory / 1e9
        print(f"[Device] ✅ NVIDIA CUDA GPU: {name} ({vram_gb:.1f} GB VRAM)")
        return "cuda", name, vram_gb

    if platform.system() == "Windows":
        for name in _query_windows_gpu():
            nl = name.lower()
            if "nvidia" in nl:
                print(f"[Device] ⚠️  NVIDIA GPU ({name}) found but CUDA not installed → CPU")
                return "cpu_nvidia_hint", name, 0
            if "amd" in nl or "radeon" in nl:
                print(f"[Device] ℹ️  AMD GPU: {name} → CPU")
                return "cpu_amd_hint", name, 0
            if "intel" in nl and ("arc" in nl or "xe" in nl):
                print(f"[Device] ℹ️  Intel Arc/Xe: {name} → CPU")
                return "cpu_intel_hint", name, 0

    print(f"[Device] 🖥️  CPU mode ({torch.get_num_threads()} threads)")
    return "cpu", "CPU", 0


device_type, device_name, vram_gb = get_device()


def _build_config_from_device(dev_type: str, dev_name: str, vram: float) -> DroneConfig:
    if dev_type == "cuda":
        if vram >= 8:
            return DroneConfig.high_end_gpu()
        elif vram >= 4:
            return DroneConfig.mid_range_gpu()
        else:
            return DroneConfig.low_vram_gpu()
    return DroneConfig.cpu_profile(dev_name)


cfg = _build_config_from_device(device_type, device_name, vram_gb)

print(f"\n[Config] Tier    : {cfg.tier}")
print(f"[Config] Model   : {cfg.model}")
print(f"[Config] Img size: {cfg.imgsz}")
print(f"[Config] FP16    : {cfg.half}")
print(f"[Config] Res     : {cfg.cam_w}×{cfg.cam_h}")
print(f"[Config] Mode    : {ACTIVE_MODE.name}")
if _mode_is_safe_test():
    print(f"[Config] Safe-test PWM ceiling: {cfg.safe_test_pwm_max}")
    print(f"[Config] Safe-test ES frac    : {cfg.safe_test_es_frac}")
print()


# ── Expose config values as module-level names ────────────────────────────────
NAVIGATION_CLASSES        = cfg.navigation_classes
FRONT_DANGER_FRAC         = cfg.front_danger_frac
SIDE_DANGER_FRAC          = cfg.side_danger_frac
HUMAN_HOVER_FRAC          = cfg.human_hover_frac
HUMAN_CENTER_OVERLAP_MIN  = cfg.human_center_overlap_min
# v5: use tighter emergency threshold in safe-test mode
EMERGENCY_AREA_FRAC       = (cfg.safe_test_es_frac
                              if _mode_is_safe_test()
                              else cfg.emergency_area_frac)
SAFE_RELEASE_FRAC         = cfg.safe_release_frac
SPEED_CLEAR_FRAC          = cfg.speed_clear_frac
SPEED_CAUTION_FRAC        = cfg.speed_caution_frac
LEFT_ZONE_END             = cfg.left_zone_end
RIGHT_ZONE_START          = cfg.right_zone_start
TOP_ZONE_END              = cfg.top_zone_end
BOTTOM_ZONE_START         = cfg.bottom_zone_start
CENTER_DENSITY_LIMIT      = cfg.center_density_limit
LAST_OBSTACLE_TIMEOUT     = cfg.last_obstacle_timeout
# v5: slow search in safe-test mode
SEARCH_PHASE_DURATION     = (cfg.search_phase_duration + cfg.safe_test_search_delay
                              if _mode_is_safe_test()
                              else cfg.search_phase_duration)
FORWARD_SCAN_DURATION     = cfg.forward_scan_duration
SEARCH_RECOVERY_CYCLES    = cfg.search_recovery_cycles
EMERGENCY_HOLD_DURATION   = cfg.emergency_hold_duration
EMERGENCY_RESEND_INTERVAL = cfg.emergency_resend_interval
OSCILLATION_GUARD_WINDOW  = cfg.oscillation_guard_window
OSCILLATION_GUARD_LIMIT   = cfg.oscillation_guard_limit
COMMAND_SEND_INTERVAL     = cfg.command_send_interval
HEARTBEAT_INTERVAL        = cfg.heartbeat_interval
STALE_FRAME_TIMEOUT       = cfg.stale_frame_timeout
READER_FAIL_HOLD          = cfg.reader_fail_hold
COMMAND_HOLD_TIME         = cfg.command_hold_time
NAV_STABILITY_MIN         = cfg.nav_stability_min
_AI_STABILITY_MIN         = cfg.ai_stability_min
_AI_CENTER_THRESH         = cfg.ai_center_thresh
_DEPTH_EMA_ALPHA          = cfg.depth_ema_alpha
HUMAN_COUNT_WRITE_INTERVAL= cfg.human_count_write_interval
ADAPTIVE_MIN_INFERENCE_INTERVAL = cfg.adaptive_min_inference_interval
HUMAN_CLASS               = cfg.human_class
SERIAL_PORT_DEFAULT       = cfg.serial_port
SERIAL_BAUD_DEFAULT       = cfg.serial_baud
SMOOTH_FRAMES             = cfg.smooth_frames
MAX_PWM_STEP              = cfg.max_pwm_step
STALE_CMD_TTL             = cfg.stale_cmd_ttl

# ── v6 stability constants ────────────────────────────────────────────────────
_SEARCH_ENTER_DELAY       = cfg.search_enter_delay
_LR_SWITCH_COOLDOWN       = cfg.lr_switch_cooldown
_FORWARD_STABILITY_MIN    = cfg.forward_stability_min
_OBS_SMOOTH_ALPHA         = cfg.obs_smooth_alpha
_NAV_OVERRIDE_GUARD       = cfg.nav_override_guard

# ── v7 emergency stability constants ─────────────────────────────────────────
# These replace the single EMERGENCY_AREA_FRAC trigger with a confirmation
# buffer + hysteresis system. See _check_emergency() for full details.
_EMERGENCY_CONFIRM_FRAMES = cfg.emergency_confirm_frames   # 6 consecutive frames
_EMERGENCY_ENTER_FRAC     = cfg.emergency_enter_frac       # 0.78 — enter ES
_EMERGENCY_EXIT_FRAC      = cfg.emergency_exit_frac        # 0.42 — exit ES (hysteresis)
_MIN_OBS_AREA_FRAC        = cfg.min_obstacle_area_frac     # 0.008 — blob size gate
_MAX_OBS_ASPECT           = cfg.max_obstacle_aspect_ratio  # 4.0  — flat surface gate
_LIVE_PROX_EMA_ALPHA      = cfg.proximity_ema_live         # 0.30 — tid=-1 smoothing
# ── v8 robustness constants ───────────────────────────────────────────────────
_BBOX_STAB_JITTER     = cfg.bbox_stability_jitter_thresh
_BBOX_STAB_HISTORY    = cfg.bbox_stability_history
_APPROACH_MIN_GROWTH  = cfg.approach_min_growth_rate
_CONF_WEIGHT_FLOOR    = cfg.conf_weight_floor
_CONF_WEIGHT_CEIL     = cfg.conf_weight_ceil
_BLUR_EXPLOSION_THRESH= cfg.blur_area_explosion_thresh
_BLUR_SUPPRESSION_DEC = cfg.blur_suppression_decay
_DANGER_CONF_BUILD    = cfg.danger_conf_build_rate
_DANGER_CONF_DECAY    = cfg.danger_conf_decay_rate
_DANGER_CONF_THRESH   = cfg.danger_conf_threshold
_SHAKE_FLOW_ENTER     = cfg.shake_flow_enter
_SHAKE_FLOW_EXIT      = cfg.shake_flow_exit
_SHAKE_SUPPRESSION    = cfg.shake_suppression_factor


# ══════════════════════════════════════════════════════════════════════════════
#  TORCH CPU THREAD TUNING (unchanged)
# ══════════════════════════════════════════════════════════════════════════════

try:
    import psutil
    _PHYSICAL = psutil.cpu_count(logical=False) or torch.get_num_threads()
except ImportError:
    _PHYSICAL = max(1, torch.get_num_threads() // 2)

torch.set_num_threads(_PHYSICAL)
torch.set_num_interop_threads(max(1, _PHYSICAL // 2))
print(f"[Torch] CPU threads: {_PHYSICAL} (physical cores)")


# ══════════════════════════════════════════════════════════════════════════════
#  THROTTLED HUMAN COUNT WRITES (unchanged)
# ══════════════════════════════════════════════════════════════════════════════

_last_human_write_time: float = 0.0
_last_human_count:      int   = -1


def _write_human_count(count: int) -> None:
    global _last_human_write_time, _last_human_count
    now = time.time()
    if (count == _last_human_count
            and (now - _last_human_write_time) < HUMAN_COUNT_WRITE_INTERVAL):
        return
    try:
        with open("human_count.txt", "w") as fh:
            fh.write(str(count))
        _last_human_write_time = now
        _last_human_count      = count
    except OSError as exc:
        event_log.log("SERIAL", f"human_count.txt write error: {exc}")


# ══════════════════════════════════════════════════════════════════════════════
#  PSEUDO-DEPTH ESTIMATION (unchanged)
# ══════════════════════════════════════════════════════════════════════════════

_depth_ema: Dict[int, float] = {}
# v7: EMA for the untracked (tid=-1) live proximity path used by _check_emergency
_live_prox_ema: Dict[int, float] = {}

# ══════════════════════════════════════════════════════════════════════════════
#  v8 REAL-WORLD ROBUSTNESS LAYER
#  All classes are read-only to the rest of the stack — they produce scalar
#  multipliers (0.0–1.0) that weight down proximity scores without removing
#  any existing validation system.
# ══════════════════════════════════════════════════════════════════════════════

# ── IMPROVEMENT 1: BBox Stability Tracker ────────────────────────────────────

class BBoxStabilityTracker:
    """
    Tracks per-object bounding-box position history and returns a
    stability score in [0, 1].

    score = 1.0  → box has been geometrically consistent across history
    score → 0.0  → box is jumping around (motion-blur / flicker)

    The score is computed as:
        1 − clamp(mean_displacement / box_size, 0, 1)
    where displacement is the centre-point movement between successive
    frames normalised by the diagonal of the bounding box.

    Usage:  bbox_stability.update(tid, box)  → float stability score
    """

    def __init__(self, history_len: int = 5, jitter_thresh: float = 0.55):
        self._history:  Dict[int, deque]  = {}
        self._scores:   Dict[int, float]  = {}
        self._history_len  = history_len
        self._jitter_thresh = jitter_thresh

    def update(self, tid: int, box: Tuple[int, int, int, int]) -> float:
        """
        Record box for `tid` and return current stability score [0, 1].
        tid=-1 (untracked) always returns 1.0 — we can't assess stability
        without identity continuity.
        """
        if tid < 0:
            return 1.0

        x1, y1, x2, y2 = box
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        diag = math.hypot(max(1, x2 - x1), max(1, y2 - y1))

        hist = self._history.setdefault(tid, deque(maxlen=self._history_len))
        hist.append((cx, cy, diag))

        if len(hist) < 2:
            self._scores[tid] = 1.0
            return 1.0

        displacements = []
        prev_cx, prev_cy, prev_diag = hist[0]
        for (hcx, hcy, hd) in list(hist)[1:]:
            d = math.hypot(hcx - prev_cx, hcy - prev_cy)
            displacements.append(d / max(1.0, hd))
            prev_cx, prev_cy, prev_diag = hcx, hcy, hd

        mean_disp = sum(displacements) / len(displacements)
        # Smooth score with a gentle EMA so brief jitter doesn't immediately
        # tank the score (gives the tracker time to settle after a real move).
        raw_score = float(np.clip(1.0 - mean_disp / max(self._jitter_thresh, 0.01),
                                  0.0, 1.0))
        prev_score = self._scores.get(tid, raw_score)
        score = 0.35 * raw_score + 0.65 * prev_score
        self._scores[tid] = score
        return score

    def score(self, tid: int) -> float:
        """Return last computed stability score without updating."""
        if tid < 0:
            return 1.0
        return self._scores.get(tid, 1.0)

    def evict_stale(self, active_tids: set) -> None:
        """Remove history for objects no longer tracked."""
        for tid in list(self._history.keys()):
            if tid not in active_tids:
                self._history.pop(tid, None)
                self._scores.pop(tid, None)


bbox_stability = BBoxStabilityTracker(
    history_len=_BBOX_STAB_HISTORY,
    jitter_thresh=_BBOX_STAB_JITTER,
)


# ── IMPROVEMENT 2: Approach Velocity Tracker ─────────────────────────────────

class ApproachVelocityTracker:
    """
    Estimates whether a tracked object is genuinely approaching the camera
    by measuring fractional bounding-box area growth over time.

    approach_velocity > _APPROACH_MIN_GROWTH  → object is closing in
    approach_velocity ≤ 0                     → object is stationary or receding

    Returns a weight in [0, 1]:
        1.0 when the object is clearly approaching at or above threshold.
        0.0 when area is shrinking.
        Linearly interpolated between the two.
    """

    def __init__(self, min_growth_rate: float = 0.04):
        self._areas:      Dict[int, Tuple[float, float]] = {}  # tid → (area, timestamp)
        self._velocities: Dict[int, float]               = {}
        self._min_growth  = min_growth_rate

    def update(self, tid: int, box: Tuple[int, int, int, int],
               frame_area: int) -> float:
        """
        Record normalised box area for `tid` and return an approach weight [0, 1].
        tid=-1 returns 0.5 (neutral — no history to assess direction).
        """
        if tid < 0:
            return 0.5   # untracked — assume neutral; do not reward or penalise

        x1, y1, x2, y2 = box
        now = time.time()
        norm_area = ((x2 - x1) * (y2 - y1)) / max(1, frame_area)

        if tid in self._areas:
            prev_area, prev_t = self._areas[tid]
            dt = max(now - prev_t, 1e-3)
            # Fractional growth rate per second
            vel = (norm_area - prev_area) / (prev_area * dt + 1e-6)
            # EMA on velocity to smooth noisy growth estimates
            prev_vel = self._velocities.get(tid, vel)
            vel = 0.30 * vel + 0.70 * prev_vel
            self._velocities[tid] = vel
        else:
            vel = 0.0
            self._velocities[tid] = 0.0

        self._areas[tid] = (norm_area, now)

        if vel >= self._min_growth:
            # Approaching — scale from 0.5 at floor to 1.0 at 3× floor
            weight = float(np.clip(
                0.5 + 0.5 * (vel - self._min_growth) / (2 * self._min_growth + 1e-6),
                0.5, 1.0,
            ))
        else:
            # Stationary or receding — scale from 0.5 down to 0.0
            weight = float(np.clip(0.5 + vel / (self._min_growth + 1e-6) * 0.5,
                                   0.0, 0.5))

        return weight

    def velocity(self, tid: int) -> float:
        """Return last estimated approach velocity (fractional area / s)."""
        return self._velocities.get(tid, 0.0)

    def evict_stale(self, active_tids: set, timeout: float = 1.0) -> None:
        now = time.time()
        for tid in list(self._areas.keys()):
            if tid not in active_tids or now - self._areas[tid][1] > timeout:
                self._areas.pop(tid, None)
                self._velocities.pop(tid, None)


approach_tracker = ApproachVelocityTracker(min_growth_rate=_APPROACH_MIN_GROWTH)


# ── IMPROVEMENT 4: Motion-Blur Guard ─────────────────────────────────────────

class MotionBlurGuard:
    """
    Detects sudden single-frame bounding-box area explosions that are
    characteristic of motion blur, rapid camera shake, or YOLO hallucination
    on low-texture / low-light frames.

    When an explosion is detected the guard raises a suppression flag.
    The suppression factor decays back to 1.0 over subsequent frames.

    suppression_factor() returns a value in (0, 1]:
        1.0  → no suppression
        →0.0 → maximum suppression (proximity scores multiplied down)
    """

    def __init__(self, explosion_thresh: float = 2.5, decay: float = 0.25):
        self._prev_areas:   Dict[int, float] = {}
        self._suppression:  float = 1.0        # current global suppression factor
        self._explosion_thresh = explosion_thresh
        self._decay        = decay

    def update(self, boxes: list, frame_area: int) -> None:
        """
        Call once per YOLO result set.  Checks each box for an area explosion
        relative to the previous frame for the same track id.
        """
        current_areas: Dict[int, float] = {}
        explosion_detected = False

        for (x1, y1, x2, y2, conf, tid, cls_id) in boxes:
            area = max(0, (x2 - x1) * (y2 - y1)) / max(1, frame_area)
            if tid >= 0:
                current_areas[tid] = area
                prev = self._prev_areas.get(tid, area)
                if prev > 0 and area / max(prev, 1e-6) > self._explosion_thresh:
                    explosion_detected = True
                    event_log.log(
                        "WATCHDOG",
                        f"Blur guard: tid={tid} area explosion "
                        f"{prev * 100:.1f}% → {area * 100:.1f}%",
                    )

        self._prev_areas = current_areas

        if explosion_detected:
            # Drop suppression immediately
            self._suppression = max(0.15, self._suppression - 0.60)
        else:
            # Recover gradually
            self._suppression = min(1.0, self._suppression + self._decay)

    def suppression_factor(self) -> float:
        """Multiply emergency proximity scores by this value [0, 1]."""
        return self._suppression

    @property
    def is_suppressed(self) -> bool:
        return self._suppression < 0.90


blur_guard = MotionBlurGuard(
    explosion_thresh=_BLUR_EXPLOSION_THRESH,
    decay=_BLUR_SUPPRESSION_DEC,
)


# ── IMPROVEMENT 6: Danger Confidence Accumulator ─────────────────────────────

class DangerConfidenceAccumulator:
    """
    Slow-build / slow-decay confidence score for emergency transitions.

    Instead of counting raw confirmation frames (which can reset on a
    single missed frame via the v7 soft-decay), this accumulator integrates
    evidence over time like a leaky bucket:

        on danger frame  : confidence += build_rate   (capped at 1.0)
        on safe frame    : confidence -= decay_rate   (floored at 0.0)

    `ready` returns True when confidence ≥ threshold.

    This works **in addition to** the existing _emergency_confirm_count
    logic — both gates must pass before ES fires (AND condition in
    _check_emergency).
    """

    def __init__(self, build_rate: float = 0.18, decay_rate: float = 0.08,
                 threshold: float = 0.72):
        self.confidence  = 0.0
        self._build      = build_rate
        self._decay      = decay_rate
        self._threshold  = threshold

    def update_danger(self, is_dangerous: bool) -> None:
        if is_dangerous:
            self.confidence = min(1.0, self.confidence + self._build)
        else:
            self.confidence = max(0.0, self.confidence - self._decay)

    @property
    def ready(self) -> bool:
        return self.confidence >= self._threshold

    def reset(self) -> None:
        self.confidence = 0.0


danger_confidence = DangerConfidenceAccumulator(
    build_rate=_DANGER_CONF_BUILD,
    decay_rate=_DANGER_CONF_DECAY,
    threshold=_DANGER_CONF_THRESH,
)


# ── IMPROVEMENT 7: Camera-Shake Detector ─────────────────────────────────────

class CameraShakeDetector:
    """
    Estimates global camera motion using sparse optical flow (Lucas-Kanade)
    on a downscaled greyscale frame pair.

    When the mean flow magnitude exceeds `shake_flow_enter` the detector
    reports shake; it exits when magnitude drops below `shake_flow_exit`.

    ── COLLISION EMERGENCY ESCALATION (new) ────────────────────────────────
    If optical flow stays above `shake_flow_enter` for `collision_confirm_frames`
    consecutive frames (default 4), the detector declares a COLLISION EMERGENCY:

        check_collision_emergency() → True

    The collision emergency stays active for at least `collision_hold_sec` seconds
    (default 1.5) even if flow drops immediately (hysteresis / cooldown).
    After the hold expires the emergency clears only when flow stays below
    `shake_flow_exit` — preventing rapid ENTER/EXIT spam.

    The collision emergency is independent of YOLO / human detection.  It fires
    on raw optical-flow evidence alone so that wall / hand impacts are caught
    even when no objects are tracked.

    On hardware without OpenCV's optical flow (e.g. headless CPU) the
    detector gracefully degrades — suppression_factor=1.0, no collision alarm.
    """

    # ── Collision escalation defaults ─────────────────────────────────────
    _COLLISION_CONFIRM_FRAMES: int   = 4    # consecutive high-flow frames needed
    _COLLISION_HOLD_SEC:       float = 1.5  # minimum active duration (hysteresis)

    def __init__(self, enter: float = 14.0, exit_: float = 6.0,
                 suppression: float = 0.55):
        self._enter       = enter
        self._exit        = exit_
        self._supp_val    = suppression
        self._shaking     = False
        self._prev_grey: Optional[np.ndarray] = None
        self._mean_flow   = 0.0
        self._lk_params   = dict(winSize=(15,15), maxLevel=2,
                                  criteria=(cv2.TERM_CRITERIA_EPS |
                                            cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self._feature_params = dict(maxCorners=40, qualityLevel=0.3,
                                     minDistance=10, blockSize=7)

        # ── Collision escalation state ─────────────────────────────────────
        self._collision_consec:   int   = 0      # consecutive high-flow frames
        self._collision_active:   bool  = False  # True while hold is in effect
        self._collision_start_t:  float = 0.0   # wall-clock when collision confirmed
        self._collision_hold_sec: float = self._COLLISION_HOLD_SEC

    def update(self, frame: np.ndarray) -> None:
        """
        Call once per display frame.  Internally downscales to 160×120 for
        speed.  Updates self._shaking, self._mean_flow, and collision state.
        """
        try:
            small = cv2.resize(frame, (160, 120), interpolation=cv2.INTER_NEAREST)
            grey  = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

            if self._prev_grey is None:
                self._prev_grey = grey
                return

            pts = cv2.goodFeaturesToTrack(self._prev_grey, **self._feature_params)
            if pts is None or len(pts) < 4:
                self._prev_grey = grey
                self._advance_collision_state(high_flow=False)
                return

            next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                self._prev_grey, grey, pts, None, **self._lk_params)

            good_old = pts[status == 1]
            good_new = next_pts[status == 1]

            if len(good_old) < 4:
                self._prev_grey = grey
                self._advance_collision_state(high_flow=False)
                return

            flow = good_new - good_old
            magnitudes = np.linalg.norm(flow, axis=1)
            # Scale back to original resolution
            scale = frame.shape[1] / 160.0
            self._mean_flow = float(np.median(magnitudes) * scale)

            # ── Shake enter/exit (unchanged) ──────────────────────────────
            if self._shaking:
                if self._mean_flow < self._exit:
                    self._shaking = False
                    event_log.log("WATCHDOG",
                                  f"Shake detector: EXIT (flow={self._mean_flow:.1f}px)")
            else:
                if self._mean_flow > self._enter:
                    self._shaking = True
                    event_log.log("WATCHDOG",
                                  f"Shake detector: ENTER (flow={self._mean_flow:.1f}px)")

            # ── Collision escalation counter ──────────────────────────────
            self._advance_collision_state(high_flow=self._mean_flow > self._enter)

            self._prev_grey = grey

        except Exception:
            # Optical flow failure (e.g. greyscale conversion error) — silently skip
            pass

    # ── Internal collision state machine ──────────────────────────────────

    def _advance_collision_state(self, high_flow: bool) -> None:
        """
        Called every frame with whether flow is currently high.
        Manages the consecutive-frame counter and the timed hold.
        """
        now = time.time()

        if self._collision_active:
            # Already in collision emergency — check if hold has expired
            held = now - self._collision_start_t
            if held >= self._collision_hold_sec and not high_flow:
                # Hold expired and flow is calm — clear the emergency
                self._collision_active   = False
                self._collision_consec   = 0
                event_log.log("WATCHDOG",
                              f"Collision emergency CLEARED "
                              f"(held {held:.1f}s, flow={self._mean_flow:.1f}px)")
            # else: keep active (hold still running or flow still high)
        else:
            if high_flow:
                self._collision_consec += 1
                if self._collision_consec >= self._COLLISION_CONFIRM_FRAMES:
                    # Confirmed collision — enter emergency
                    self._collision_active  = True
                    self._collision_start_t = now
                    self._collision_consec  = 0
                    event_log.log("WATCHDOG",
                                  f"Emergency collision triggered — "
                                  f"flow={self._mean_flow:.1f}px sustained "
                                  f"{self._COLLISION_CONFIRM_FRAMES} frames")
            else:
                # Flow fell below threshold — reset counter (no false alarm)
                self._collision_consec = max(0, self._collision_consec - 1)

    # ── Public API ────────────────────────────────────────────────────────

    def check_collision_emergency(self) -> bool:
        """
        Returns True when a sustained high-flow collision has been confirmed
        and the hold period is still active.  Independent of YOLO tracking.
        """
        return self._collision_active

    @property
    def collision_consec_frames(self) -> int:
        """Current consecutive high-flow frame count (for HUD debug)."""
        return self._collision_consec

    @property
    def collision_active_elapsed(self) -> float:
        """Seconds since the collision emergency started (0 if inactive)."""
        if not self._collision_active:
            return 0.0
        return time.time() - self._collision_start_t

    @property
    def is_shaking(self) -> bool:
        return self._shaking

    @property
    def mean_flow(self) -> float:
        return self._mean_flow

    def suppression_factor(self) -> float:
        """Returns _supp_val when shaking, 1.0 otherwise."""
        return self._supp_val if self._shaking else 1.0


camera_shake = CameraShakeDetector(
    enter=_SHAKE_FLOW_ENTER,
    exit_=_SHAKE_FLOW_EXIT,
    suppression=_SHAKE_SUPPRESSION,
)


def estimate_pseudo_depth_v3(x1, y1, x2, y2, frame_w, frame_h, tid=-1) -> float:
    """
    Pseudo-depth estimator — v7 rewrite.

    v7 changes (architecture and ToF hook preserved verbatim):
    ─────────────────────────────────────────────────────────
    FIX-8a  Minimum size gate: boxes < _MIN_OBS_AREA_FRAC of frame area
            return 0.0 immediately — noisy texture blobs eliminated.
    FIX-4c  Aspect-ratio penalty: boxes with w:h > _MAX_OBS_ASPECT or
            < 1/_MAX_OBS_ASPECT (flat floor/ceiling, thin wall column) lose
            up to 55% of their proximity weight via a smooth curve.
    FIX-4a  Area weight reduced 0.55 → 0.35 so large distant surfaces no
            longer dominate the raw score.
    FIX-4b  Centre-distance factor: an obstacle at the horizontal frame edge
            (cx ≈ 0 or frame_w) gets 60% weight reduction — side walls can
            no longer masquerade as frontal threats.
    FIX-6   EMA applied on the tid=-1 live path too (via _live_prox_ema keyed
            on bbox hash) so single-frame spikes reaching _check_emergency are
            pre-smoothed before the confirmation buffer sees them.
    """
    if frame_w <= 0 or frame_h <= 0:
        return 0.0

    bw, bh = max(0, x2 - x1), max(0, y2 - y1)

    # ── FIX-8a: minimum size gate ─────────────────────────────────────────
    raw_area_frac = (bw * bh) / max(1, frame_w * frame_h)
    if raw_area_frac < _MIN_OBS_AREA_FRAC:
        return 0.0   # blob too small — not a real collision threat

    # ── FIX-4c: aspect-ratio penalty ──────────────────────────────────────
    aspect = bw / max(1, bh)                              # width : height ratio
    if aspect > _MAX_OBS_ASPECT or aspect < (1.0 / _MAX_OBS_ASPECT):
        # Extreme flat (floor/ceiling) or thin (wall column) detection
        aspect_penalty = 0.45
    else:
        # Smooth penalty curve: 1.0 at square, decreasing toward limits
        excess = max(aspect / _MAX_OBS_ASPECT,
                     (_MAX_OBS_ASPECT / max(aspect, 1e-6)) ** -1)
        aspect_penalty = float(np.clip(1.0 - 0.3 * (excess - 1.0), 0.45, 1.0))

    # ── Spatial cues ──────────────────────────────────────────────────────
    area_cue   = min(1.0, raw_area_frac)
    width_cue  = min(1.0, bw / frame_w)
    height_cue = min(1.0, bh / frame_h)
    cy_norm    = ((y1 + y2) / 2.0) / frame_h
    cx_norm    = ((x1 + x2) / 2.0) / frame_w
    vpos_cue   = float(np.clip(cy_norm, 0.0, 1.0))

    # ── FIX-4b: centre-distance weighting ─────────────────────────────────
    # Obstacle dead-centre (cx_norm=0.5) = full weight.
    # At frame edge (cx_norm=0 or 1) = 40% weight — side walls ignored.
    centre_dist   = abs(cx_norm - 0.5)
    centre_factor = float(np.clip(1.0 - centre_dist * 1.2, 0.40, 1.0))

    # HARDWARE-READY COMMENT: ToF / ultrasonic proximity blend
    # Uncomment and implement once hardware is connected:
    # zone = classify_horizontal_zone((x1+x2)//2, frame_w)
    # sensor_prox = sensor_fusion.get_proximity(zone)
    # if sensor_prox is not None: raw = 0.6*raw + 0.4*sensor_prox

    # ── FIX-4a: lower area dominance, apply spatial modifiers ────────────
    # Old: 0.55*area + 0.20*width + 0.15*height + 0.10*vpos
    # New: area weight reduced, centre + aspect corrections applied
    raw = float(np.clip(
        (0.35 * area_cue + 0.28 * width_cue + 0.22 * height_cue + 0.15 * vpos_cue)
        * centre_factor
        * aspect_penalty,
        0.0, 1.0,
    ))

    # ── FIX-6: EMA on BOTH tracked and live paths ─────────────────────────
    if tid >= 0:
        # Original tracked-object EMA (unchanged)
        prev     = _depth_ema.get(tid, raw)
        smoothed = _DEPTH_EMA_ALPHA * raw + (1.0 - _DEPTH_EMA_ALPHA) * prev
        _depth_ema[tid] = smoothed
        return smoothed
    else:
        # v7: smooth the live/untracked path used by _check_emergency
        bbox_key = hash((x1, y1, x2, y2)) & 0xFFFFFFFF
        prev     = _live_prox_ema.get(bbox_key, raw)
        smoothed = _LIVE_PROX_EMA_ALPHA * raw + (1.0 - _LIVE_PROX_EMA_ALPHA) * prev
        if len(_live_prox_ema) > 150:   # evict oldest entries to bound memory
            for _k in list(_live_prox_ema.keys())[:50]:
                _live_prox_ema.pop(_k, None)
        _live_prox_ema[bbox_key] = smoothed
        return smoothed


def classify_horizontal_zone(cx: int, frame_w: int) -> str:
    frac = cx / frame_w
    if frac < LEFT_ZONE_END:   return "LEFT"
    if frac > RIGHT_ZONE_START: return "RIGHT"
    return "CENTER"


def classify_vertical_zone(cy: int, frame_h: int) -> str:
    frac = cy / frame_h
    if frac < TOP_ZONE_END:        return "TOP"
    if frac > BOTTOM_ZONE_START:   return "BOTTOM"
    return "MIDDLE"


# ══════════════════════════════════════════════════════════════════════════════
#  OBSTACLE DATA CONTAINER (unchanged)
# ══════════════════════════════════════════════════════════════════════════════

class ObstacleInfo:
    __slots__ = ("box", "proximity", "zone", "v_zone",
                 "is_human", "tid", "cls_id", "timestamp")

    def __init__(self, box, proximity, zone, v_zone,
                 is_human, tid, cls_id, timestamp):
        self.box       = box
        self.proximity = proximity
        self.zone      = zone
        self.v_zone    = v_zone
        self.is_human  = is_human
        self.tid       = tid
        self.cls_id    = cls_id
        self.timestamp = timestamp


# ══════════════════════════════════════════════════════════════════════════════
#  TEMPORAL OBSTACLE MEMORY (unchanged)
# ══════════════════════════════════════════════════════════════════════════════

_obstacle_memory: Dict[int, ObstacleInfo] = {}


def _update_obstacle_memory(live: List[ObstacleInfo]) -> List[ObstacleInfo]:
    now = time.time()
    for obs in live:
        if obs.tid >= 0:
            prev = _obstacle_memory.get(obs.tid)
            if prev is not None:
                # v6 Fix 6: secondary EMA on stored proximity to damp rapid changes
                obs.proximity = (_OBS_SMOOTH_ALPHA * obs.proximity
                                 + (1.0 - _OBS_SMOOTH_ALPHA) * prev.proximity)
            _obstacle_memory[obs.tid] = obs
    expired = [tid for tid, obs in _obstacle_memory.items()
               if now - obs.timestamp > LAST_OBSTACLE_TIMEOUT]
    for tid in expired:
        del _obstacle_memory[tid]
        _depth_ema.pop(tid, None)
    live_tids  = {o.tid for o in live}
    remembered = [obs for tid, obs in _obstacle_memory.items() if tid not in live_tids]
    merged = live + remembered
    merged.sort(key=lambda o: o.proximity, reverse=True)
    return merged


def _compute_live_obstacles(boxes: list, frame_w: int, frame_h: int) -> List[ObstacleInfo]:
    """
    v8 additions (architecture unchanged):

    IMPROVEMENT 3 — confidence weighting
        Low-YOLO-confidence detections have their proximity scaled down
        via a linear ramp between conf_weight_floor and conf_weight_ceil.

    IMPROVEMENT 5 — forward-corridor priority
        Objects near the lateral frame edges (cx close to 0 or frame_w)
        get an additional proximity weight reduction.  Only objects
        overlapping the flight corridor receive full weight.

    IMPROVEMENT 1 — stability score applied here
        bbox_stability.update() is called; the returned score multiplies
        the final proximity.  Flickering boxes lose weight automatically.

    IMPROVEMENT 2 — approach velocity called here
        approach_tracker.update() is called but its weight is NOT applied
        to _compute_live_obstacles — this path feeds _check_emergency
        directly, so we want unmodified proximity values.  The tracker
        merely accumulates history so _check_emergency can query it.
    """
    now        = time.time()
    frame_area = max(1, frame_w * frame_h)
    live       = []

    for (x1, y1, x2, y2, conf, tid, cls_id) in boxes:
        if cls_id not in NAVIGATION_CLASSES:
            continue

        # ── IMPROVEMENT 3: confidence weight ─────────────────────────────
        # Scale proximity down for low-confidence detections.
        # Detections above conf_weight_ceil are unaffected (weight=1.0).
        conf_weight = float(np.clip(
            (conf - _CONF_WEIGHT_FLOOR) / max(_CONF_WEIGHT_CEIL - _CONF_WEIGHT_FLOOR, 1e-4),
            0.20,   # floor: even very low-conf detections keep 20% weight
            1.0,
        ))

        cx, cy    = (x1+x2)//2, (y1+y2)//2
        proximity = estimate_pseudo_depth_v3(x1, y1, x2, y2, frame_w, frame_h, tid=-1)

        # ── IMPROVEMENT 5: flight-corridor priority ────────────────────────
        # Objects at the lateral edges of the frame (side walls, door frames)
        # receive a reduced weight because they are NOT in the flight path.
        # The existing centre-distance factor inside estimate_pseudo_depth_v3
        # already accounts for this geometrically; here we add a second soft
        # gate to further suppress clearly off-axis detections.
        cx_norm        = cx / max(1, frame_w)
        lateral_offset = abs(cx_norm - 0.5)          # 0 = centre, 0.5 = edge
        # Objects more than 40% off-centre get up to 30% additional penalty
        corridor_weight = float(np.clip(
            1.0 - max(0.0, lateral_offset - 0.25) * 1.20,
            0.70, 1.0,
        ))

        # ── IMPROVEMENT 1: bbox stability weight ──────────────────────────
        stability_score = bbox_stability.update(tid, (x1, y1, x2, y2))

        # ── IMPROVEMENT 2: approach velocity history ──────────────────────
        # We call update here purely to keep the tracker's history current.
        approach_tracker.update(tid, (x1, y1, x2, y2), frame_area)

        # Apply all v8 weights to this path's proximity
        proximity = proximity * conf_weight * corridor_weight * stability_score

        live.append(ObstacleInfo(
            box=(x1, y1, x2, y2), proximity=proximity,
            zone=classify_horizontal_zone(cx, frame_w),
            v_zone=classify_vertical_zone(cy, frame_h),
            is_human=(cls_id == HUMAN_CLASS), tid=tid, cls_id=cls_id, timestamp=now,
        ))
    return live


def analyse_obstacles(boxes: list, frame_w: int, frame_h: int) -> List[ObstacleInfo]:
    """
    v7 addition: two pre-filters applied before proximity estimation.

    FIX-8b  Boxes smaller than _MIN_OBS_AREA_FRAC are skipped entirely —
            they never enter obstacle memory and cannot trigger navigation
            or emergency decisions.
    FIX-8c  Boxes with an extreme aspect ratio (> _MAX_OBS_ASPECT × 1.5 or
            < its reciprocal) are also skipped.  Detections this flat or thin
            are overwhelmingly walls, floors, or ceiling reflections — not
            navigable objects in the forward flight corridor.

    Everything else (memory merge, EMA proximity, sort) is identical to v6.
    """
    now  = time.time()
    live = []
    for (x1, y1, x2, y2, conf, tid, cls_id) in boxes:
        if cls_id not in NAVIGATION_CLASSES:
            continue

        # ── FIX-8b: minimum size gate ─────────────────────────────────────
        bw = max(0, x2 - x1)
        bh = max(0, y2 - y1)
        if (bw * bh) / max(1, frame_w * frame_h) < _MIN_OBS_AREA_FRAC:
            continue   # blob too small — skip entirely

        # ── FIX-8c: extreme aspect ratio gate ────────────────────────────
        # Very wide (floor/ceiling) and razor-thin (wall column) detections
        # are pre-rejected before they reach estimate_pseudo_depth_v3.
        aspect = bw / max(1, bh)
        if aspect > (_MAX_OBS_ASPECT * 1.5) or aspect < (1.0 / (_MAX_OBS_ASPECT * 1.5)):
            continue   # extreme flat/thin surface — not a navigable obstacle

        cx, cy    = (x1 + x2) // 2, (y1 + y2) // 2
        proximity = estimate_pseudo_depth_v3(x1, y1, x2, y2, frame_w, frame_h, tid)

        live.append(ObstacleInfo(
            box=(x1, y1, x2, y2), proximity=proximity,
            zone=classify_horizontal_zone(cx, frame_w),
            v_zone=classify_vertical_zone(cy, frame_h),
            is_human=(cls_id == HUMAN_CLASS), tid=tid, cls_id=cls_id, timestamp=now,
        ))

    live.sort(key=lambda o: o.proximity, reverse=True)
    return _update_obstacle_memory(live)


# ══════════════════════════════════════════════════════════════════════════════
#  HUMAN CENTER OVERLAP (unchanged)
# ══════════════════════════════════════════════════════════════════════════════

def _human_center_overlap(obs: ObstacleInfo, frame_w: int) -> float:
    x1, _, x2, _ = obs.box
    box_w        = max(1, x2 - x1)
    center_left  = int(frame_w * LEFT_ZONE_END)
    center_right = int(frame_w * RIGHT_ZONE_START)
    overlap_px   = max(0, min(x2, center_right) - max(x1, center_left))
    return overlap_px / box_w


# ══════════════════════════════════════════════════════════════════════════════
#  AI DECISION LAYER  (v6: stronger smoothing, L↔R cooldown, search delay)
# ══════════════════════════════════════════════════════════════════════════════

_ai_state        = STATE_IDLE
_ai_intent       = AI_SEARCH_TARGET
_ai_decision_str = AI_SEARCH_TARGET
_ai_prev_intent  = AI_SEARCH_TARGET
_ai_same_count   = 0

# v6 Fix 4: prevent rapid L↔R flipping
_ai_last_lr_intent: str   = AI_HOVER   # last committed L or R intent
_ai_lr_same_count:  int   = 0          # consecutive frames this L/R raw intent held

# v6 Fix 5: delay before entering SEARCH after losing target
_ai_no_target_frames: int = 0          # consecutive frames with no human box


def ai_decision(boxes: list, frame_w: int, frame_h: int):
    """
    Layer-1 AI Intent — WHERE is the target?

    v6 stability changes (architecture unchanged):
    • wider dead-band (_AI_CENTER_THRESH raised) stops L↔R jitter
    • L↔R flip requires _LR_SWITCH_COOLDOWN consecutive frames of the new intent
    • SEARCH_TARGET only committed after _SEARCH_ENTER_DELAY missed frames
    • HOVER preferred when proximity is borderline or intent is ambiguous
    • all existing safe-test and stability-counter logic preserved
    """
    global _ai_state, _ai_intent, _ai_decision_str
    global _ai_prev_intent, _ai_same_count
    global _ai_last_lr_intent, _ai_lr_same_count
    global _ai_no_target_frames

    frame_cx    = frame_w // 2
    human_boxes = [b for b in boxes if b[6] == HUMAN_CLASS]

    if not human_boxes:
        _ai_no_target_frames += 1
        _ai_state  = STATE_SEARCHING
        # v6 Fix 5: hold current intent until we've missed enough frames
        if _ai_no_target_frames < _SEARCH_ENTER_DELAY:
            # keep whatever intent was last committed — do not flip to SEARCH yet
            return _ai_state, _ai_intent, None
        raw_intent    = AI_SEARCH_TARGET
        target_center = None
    else:
        _ai_no_target_frames = 0   # reset miss counter when target re-appears

        def _area(b):
            return (b[2]-b[0]) * (b[3]-b[1])
        best = max(human_boxes, key=_area)
        x1, y1, x2, y2, conf, tid, cls_id = best
        cx, cy = (x1+x2)//2, (y1+y2)//2
        target_center = (cx, cy)

        frame_area  = max(1, frame_w * frame_h)
        bbox_area   = max(0, (x2-x1) * (y2-y1))
        prox_frac   = bbox_area / frame_area
        offset_frac = (cx - frame_cx) / frame_w

        if _mode_is_safe_test():
            raw_intent = AI_HOVER
        elif prox_frac >= HUMAN_HOVER_FRAC and abs(offset_frac) <= _AI_CENTER_THRESH:
            raw_intent = AI_HOVER
        elif offset_frac < -_AI_CENTER_THRESH:
            raw_intent = AI_TRACK_LEFT
        elif offset_frac > _AI_CENTER_THRESH:
            raw_intent = AI_TRACK_RIGHT
        else:
            raw_intent = AI_TRACK_CENTER

        # v6 Fix 4: L↔R switch cooldown — must hold new L/R for N frames
        if raw_intent in (AI_TRACK_LEFT, AI_TRACK_RIGHT):
            if raw_intent == _ai_last_lr_intent:
                _ai_lr_same_count += 1
            else:
                # New L/R direction — require cooldown before committing
                if _ai_lr_same_count < _LR_SWITCH_COOLDOWN:
                    # Not stable yet: prefer HOVER over switching immediately
                    raw_intent = AI_HOVER
                else:
                    _ai_last_lr_intent = raw_intent
                    _ai_lr_same_count  = 1
        else:
            # Not a L/R intent — decay the LR counter gently so a return is fast
            _ai_lr_same_count = max(0, _ai_lr_same_count - 1)

        _ai_state = STATE_TRACKING

    # Existing stability counter (unchanged logic)
    if raw_intent == _ai_prev_intent:
        _ai_same_count += 1
    else:
        _ai_same_count  = 1
        _ai_prev_intent = raw_intent

    if _ai_same_count >= _AI_STABILITY_MIN:
        if _ai_intent != raw_intent:
            _ai_intent       = raw_intent
            _ai_decision_str = raw_intent

    return _ai_state, _ai_intent, target_center


def draw_ai_overlay(frame: np.ndarray, target_center,
                    frame_w: int, frame_h: int) -> None:
    fc = (frame_w // 2, frame_h // 2)
    cv2.circle(frame, fc, 6, (0, 0, 255), -1)
    cv2.circle(frame, fc, 8, (255, 255, 255), 1)
    if target_center is not None:
        cv2.line(frame, fc, target_center, (255, 80, 0), 2)
        cv2.circle(frame, target_center, 5, (255, 80, 0), -1)


# ══════════════════════════════════════════════════════════════════════════════
#  SEARCH FSM  (v5: safe-test slows search cycle via SEARCH_PHASE_DURATION)
# ══════════════════════════════════════════════════════════════════════════════

_search_phase_index: int   = 0
_search_phase_start: float = time.time()
_forward_scan_start: float = 0.0
_search_cycle_count: int   = 0


def _next_search_state(obstacles: List[ObstacleInfo]) -> str:
    global _search_phase_index, _search_phase_start
    global _forward_scan_start, _search_cycle_count

    now = time.time()
    if now - _search_phase_start >= SEARCH_PHASE_DURATION:
        _search_phase_index = (_search_phase_index + 1) % len(_SEARCH_CYCLE)
        _search_phase_start = now
        if _search_phase_index == 0:
            _search_cycle_count += 1

    state = _SEARCH_CYCLE[_search_phase_index]

    if state == NAV_FORWARD_SCAN:
        center_obs = [o for o in obstacles
                      if o.zone == "CENTER" and o.proximity >= SIDE_DANGER_FRAC]
        if center_obs:
            return NAV_SAFE_SEARCH
        if _forward_scan_start == 0.0 or (now - _forward_scan_start) > SEARCH_PHASE_DURATION:
            _forward_scan_start = now
        if (now - _forward_scan_start) >= FORWARD_SCAN_DURATION:
            _search_phase_index = (_search_phase_index + 1) % len(_SEARCH_CYCLE)
            _search_phase_start = now
            _forward_scan_start = 0.0
            return _SEARCH_CYCLE[_search_phase_index]
        return NAV_FORWARD_SCAN

    if _search_cycle_count >= SEARCH_RECOVERY_CYCLES:
        _search_cycle_count = 0
        center_obs = [o for o in obstacles
                      if o.zone == "CENTER" and o.proximity >= SIDE_DANGER_FRAC]
        if not center_obs:
            _forward_scan_start = now
            _search_phase_index = _SEARCH_CYCLE.index(NAV_FORWARD_SCAN)
            _search_phase_start = now
            return NAV_FORWARD_SCAN
        return NAV_SAFE_SEARCH

    return state


# ══════════════════════════════════════════════════════════════════════════════
#  EMERGENCY SAFETY LAYER  (unchanged from v4)
# ══════════════════════════════════════════════════════════════════════════════

_oscillation_timestamps: deque = deque()
_EMERG_IDLE     = "IDLE"
_EMERG_ACTIVE   = "ACTIVE"
_EMERG_RECOVERY = "RECOVERY"
_emergency_phase:         str   = _EMERG_IDLE
_emergency_start_time:   float = 0.0
_emergency_reason:       str   = ""
_emergency_last_sent:    float = 0.0
_emergency_confirm_count: int  = 0   # v7: consecutive dangerous CENTER frames


def _is_dangerous_oscillation(cmd_a: str, cmd_b: str) -> bool:
    return frozenset({cmd_a, cmd_b}) in _OSCILLATION_PAIRS


def _check_emergency(raw_cmd: str, prev_raw: str,
                     live_obstacles: List[ObstacleInfo]) -> bool:
    """
    Emergency Safety Layer — v8 rewrite of IDLE phase only.
    ACTIVE and RECOVERY phases are preserved verbatim from v7.

    v8 IDLE-phase additions (all existing v7 fixes retained):
    ──────────────────────────────────────────────────────────
    IMPROVEMENT 4 — blur guard multiplier
        blur_guard.suppression_factor() scales every obstacle's effective
        proximity before the threshold comparison.  If the camera just saw
        a bbox explosion (blur / shake artefact), suppression drops toward
        0.15 so single-frame YOLO hallucinations cannot fill the
        confirmation buffer.

    IMPROVEMENT 6 — danger confidence gate (second AND condition)
        danger_confidence must also be ≥ threshold before ES fires.
        It builds slowly (build_rate per frame) and decays slowly so
        brief dangerous frames are absorbed without triggering ES.

    IMPROVEMENT 7 — camera-shake suppression
        camera_shake.suppression_factor() further attenuates proximity
        when global frame motion is detected.  This handles hand-held
        webcam testing and physical vibration on the real drone.

    IMPROVEMENT 2 — approach velocity gate
        For each candidate CENTER obstacle we check approach_tracker;
        objects that are NOT growing in area (stationary wall, ceiling)
        have their contribution reduced by 50%.
    """
    global _emergency_phase, _emergency_start_time, _emergency_reason
    global _emergency_last_sent, _emergency_confirm_count

    now = time.time()

    # ── Oscillation tracking (unchanged) ──────────────────────────────────
    if _is_dangerous_oscillation(raw_cmd, prev_raw):
        _oscillation_timestamps.append(now)
    while (_oscillation_timestamps
           and (now - _oscillation_timestamps[0]) > OSCILLATION_GUARD_WINDOW):
        _oscillation_timestamps.popleft()

    # ── IDLE phase ────────────────────────────────────────────────────────
    if _emergency_phase == _EMERG_IDLE:

        # v8: combined suppression factor from blur + shake guards
        suppress = blur_guard.suppression_factor() * camera_shake.suppression_factor()

        # v8 IMPROVEMENT 2 + 7 + 4: centre-corridor check with suppression
        center_danger = []
        for obs in live_obstacles:
            if obs.zone != "CENTER" or obs.v_zone != "MIDDLE":
                continue

            # Apply combined suppression to effective proximity
            eff_prox = obs.proximity * suppress

            # IMPROVEMENT 2: penalise non-approaching obstacles
            approach_weight = approach_tracker.velocity(obs.tid)
            # approach_weight is a raw velocity (fractional area/s).
            # If velocity ≤ 0 the object is not closing — reduce contribution.
            if approach_weight <= 0:
                eff_prox *= 0.60   # 40% reduction for stationary/receding objects
            elif approach_weight < _APPROACH_MIN_GROWTH:
                eff_prox *= float(np.clip(
                    0.60 + 0.40 * (approach_weight / _APPROACH_MIN_GROWTH), 0.60, 1.0))

            if eff_prox >= _EMERGENCY_ENTER_FRAC:
                center_danger.append(obs)

        is_dangerous = len(center_danger) > 0

        # v8 IMPROVEMENT 6: update danger confidence accumulator
        danger_confidence.update_danger(is_dangerous)

        if is_dangerous:
            _emergency_confirm_count += 1
            event_log.log(
                "WATCHDOG",
                f"ES confirm {_emergency_confirm_count}/{_EMERGENCY_CONFIRM_FRAMES} "
                f"| DangerConf {danger_confidence.confidence:.2f}/{_DANGER_CONF_THRESH:.2f} "
                f"| BlurSupp {blur_guard.suppression_factor():.2f} "
                f"| ShakeSupp {camera_shake.suppression_factor():.2f} "
                f"| prox {max(o.proximity for o in center_danger) * 100:.0f}%",
            )
        else:
            _emergency_confirm_count = max(0, _emergency_confirm_count - 2)

        # v8: BOTH confirmation frames AND danger confidence must be satisfied
        if (_emergency_confirm_count >= _EMERGENCY_CONFIRM_FRAMES
                and danger_confidence.ready):

            # Final blur/shake check — if still heavily suppressed, defer
            if suppress < 0.40:
                event_log.log(
                    "WATCHDOG",
                    f"ES deferred — combined suppression {suppress:.2f} too low "
                    f"(blur={blur_guard.suppression_factor():.2f} "
                    f"shake={camera_shake.suppression_factor():.2f})",
                )
                _emergency_confirm_count = max(0, _emergency_confirm_count - 3)
                return False

            worst = max(center_danger, key=lambda o: o.proximity)
            _emergency_phase         = _EMERG_ACTIVE
            _emergency_start_time    = now
            _emergency_confirm_count = 0
            danger_confidence.reset()
            _emergency_reason = (
                f"CENTER obstacle {worst.proximity * 100:.0f}% "
                f">= {_EMERGENCY_ENTER_FRAC * 100:.0f}% "
                f"({_EMERGENCY_CONFIRM_FRAMES} frames + conf "
                f"{danger_confidence._threshold:.2f} confirmed)"
            )
            _emergency_last_sent  = 0.0
            _oscillation_timestamps.clear()
            event_log.log("EMERGENCY", f"Entered — {_emergency_reason}",
                          proximity=worst.proximity)
            return True

        # Oscillation guard (unchanged)
        if len(_oscillation_timestamps) > OSCILLATION_GUARD_LIMIT:
            _emergency_phase         = _EMERG_ACTIVE
            _emergency_start_time    = now
            _emergency_reason        = "oscillation guard"
            _emergency_last_sent     = 0.0
            _emergency_confirm_count = 0
            danger_confidence.reset()
            _oscillation_timestamps.clear()
            event_log.log("EMERGENCY", "Entered — oscillation guard")
            return True

        return False

    # ── ACTIVE phase (unchanged from v7) ──────────────────────────────────
    if _emergency_phase == _EMERG_ACTIVE:
        elapsed = now - _emergency_start_time
        if elapsed < EMERGENCY_HOLD_DURATION:
            if now - _emergency_last_sent >= EMERGENCY_RESEND_INTERVAL:
                _emergency_last_sent = now
                event_log.log("WATCHDOG",
                              f"ES hold {elapsed:.1f}s / {EMERGENCY_HOLD_DURATION:.1f}s")
            return True
        _emergency_phase      = _EMERG_RECOVERY
        _emergency_start_time = now
        event_log.log("EMERGENCY", "Hold elapsed → RECOVERY phase")

    # ── RECOVERY phase (unchanged from v7) ────────────────────────────────
    if _emergency_phase == _EMERG_RECOVERY:
        max_live_prox = max((obs.proximity for obs in live_obstacles), default=0.0)
        if max_live_prox < _EMERGENCY_EXIT_FRAC:
            _emergency_phase         = _EMERG_IDLE
            _emergency_reason        = ""
            _emergency_confirm_count = 0
            danger_confidence.reset()
            event_log.log(
                "EMERGENCY",
                f"Released — live prox {max_live_prox * 100:.0f}% "
                f"< {_EMERGENCY_EXIT_FRAC * 100:.0f}% (exit threshold)",
            )
            return False
        if now - _emergency_last_sent >= EMERGENCY_RESEND_INTERVAL:
            _emergency_last_sent = now
            event_log.log("WATCHDOG",
                          f"Recovery blocked — prox {max_live_prox * 100:.0f}%")
        return True

    return False

# ══════════════════════════════════════════════════════════════════════════════
#  SPEED-TIERED FORWARD (unchanged)
# ══════════════════════════════════════════════════════════════════════════════

def _speed_tiered_forward(obstacles: List[ObstacleInfo]) -> str:
    center_obs = [o for o in obstacles if o.zone == "CENTER"]
    if not center_obs:
        return NAV_FAST_FORWARD
    nearest = max(center_obs, key=lambda o: o.proximity)
    if nearest.proximity < SPEED_CLEAR_FRAC:
        return NAV_FAST_FORWARD
    if nearest.proximity < SPEED_CAUTION_FRAC:
        return NAV_SLOW_FORWARD
    return NAV_STOP


# ══════════════════════════════════════════════════════════════════════════════
#  CORE NAVIGATION FSM  (v5: safe-test disables FAST_FORWARD)
# ══════════════════════════════════════════════════════════════════════════════

def _nav_raw_decision_v3(obstacles, frame_w, frame_h):
    if not obstacles:
        return _next_search_state([]), NAV_SEARCH, None

    humans = [o for o in obstacles if o.is_human]
    for human in humans:
        overlap = _human_center_overlap(human, frame_w)
        if human.proximity >= HUMAN_HOVER_FRAC and overlap >= HUMAN_CENTER_OVERLAP_MIN:
            return NAV_HOVER, NAV_HOVER, human

    for obs in obstacles:
        if obs.is_human:
            continue
        if obs.zone == "CENTER" and obs.proximity >= FRONT_DANGER_FRAC:
            return NAV_BACKWARD, NAV_BACKWARD, obs

    for obs in obstacles:
        if obs.is_human:
            continue
        if obs.zone == "LEFT"  and obs.proximity >= SIDE_DANGER_FRAC:
            return NAV_AVOID_RIGHT, NAV_AVOID_RIGHT, obs
        if obs.zone == "RIGHT" and obs.proximity >= SIDE_DANGER_FRAC:
            return NAV_AVOID_LEFT, NAV_AVOID_LEFT, obs

    center_objects = [o for o in obstacles if o.zone == "CENTER"]
    if len(center_objects) >= CENTER_DENSITY_LIMIT:
        return NAV_SAFE_SEARCH, NAV_SEARCH, None

    top_threats = [o for o in obstacles
                   if o.v_zone == "TOP" and o.proximity >= SIDE_DANGER_FRAC]
    if top_threats:
        worst_top = max(top_threats, key=lambda o: o.proximity)
        if worst_top.proximity >= FRONT_DANGER_FRAC:
            return NAV_STOP, NAV_STOP, worst_top
        return NAV_SLOW_FORWARD, NAV_FORWARD, worst_top

    fwd_cmd = _speed_tiered_forward(obstacles)
    # IMPROVEMENT 1: safe-test disables FAST_FORWARD
    if _mode_is_safe_test() and fwd_cmd == NAV_FAST_FORWARD:
        fwd_cmd = NAV_SLOW_FORWARD
    return fwd_cmd, NAV_FORWARD, None


# ── Navigation FSM public state ───────────────────────────────────────────────
_nav_state        = NAV_SEARCH
_nav_prev_raw     = NAV_SEARCH
_nav_stable_count = 0
_nav_final_cmd    = NAV_SEARCH

# v6 Fix 7: forward-stability — count consecutive FORWARD raw decisions
_nav_fwd_count: int = 0    # consecutive frames raw decision was a forward variant
# v6 Fix 9: nav-override guard — track consecutive stable nav frames
_nav_override_count: int = 0   # consecutive frames nav was CAUTION/CLEAR


def _force_emergency_nav_state() -> None:
    global _nav_state, _nav_prev_raw, _nav_stable_count, _nav_final_cmd
    global _emergency_phase, _emergency_start_time, _emergency_reason, _emergency_last_sent
    _nav_final_cmd        = NAV_EMERGENCY_STOP
    _nav_state            = NAV_EMERGENCY_STOP
    _nav_prev_raw         = NAV_EMERGENCY_STOP
    _nav_stable_count     = 0
    if _emergency_phase == _EMERG_IDLE:
        _emergency_phase      = _EMERG_ACTIVE
        _emergency_start_time = time.time()
        _emergency_reason     = "forced (stale frame / reader failure)"
        _emergency_last_sent  = 0.0
        event_log.log("EMERGENCY", f"Entered — {_emergency_reason}")


def nav_decision(boxes: list, frame_w: int, frame_h: int):
    global _nav_state, _nav_prev_raw, _nav_stable_count, _nav_final_cmd
    global _nav_fwd_count, _nav_override_count

    t_start = time.perf_counter()
    _live_obs_for_emergency = _compute_live_obstacles(boxes, frame_w, frame_h)
    obstacles = analyse_obstacles(boxes, frame_w, frame_h)
    raw_cmd, new_state, danger_obs = _nav_raw_decision_v3(obstacles, frame_w, frame_h)
    _nav_state = new_state

    if _check_emergency(raw_cmd, _nav_prev_raw, _live_obs_for_emergency):
        _nav_fwd_count      = 0
        _nav_override_count = 0
        _nav_final_cmd = NAV_EMERGENCY_STOP
        send_nav_token(ARDUINO_TOKENS[NAV_EMERGENCY_STOP], force=True)
        perf.record_command_latency((time.perf_counter() - t_start) * 1000)
        return _nav_final_cmd, obstacles, danger_obs

    # v6 Fix 7: track consecutive forward decisions for stability
    _fwd_variants = (NAV_FAST_FORWARD, NAV_SLOW_FORWARD, NAV_FORWARD, NAV_FORWARD_SCAN)
    if raw_cmd in _fwd_variants:
        _nav_fwd_count += 1
    else:
        _nav_fwd_count = 0

    # v6 Fix 9: track nav stability for override guard
    if new_state in (NAV_STATE_CAUTION, NAV_STATE_CLEAR):
        _nav_override_count += 1
    else:
        _nav_override_count = 0

    # v6 Fix 7: don't commit forward motion until it has been stable long enough
    effective_raw = raw_cmd
    if raw_cmd in _fwd_variants and _nav_fwd_count < _FORWARD_STABILITY_MIN:
        # Hold previous non-forward command (e.g. HOVER/SEARCH) while building confidence
        effective_raw = _nav_final_cmd if _nav_final_cmd not in _fwd_variants else NAV_SAFE_SEARCH

    if effective_raw == _nav_prev_raw:
        _nav_stable_count += 1
    else:
        _nav_stable_count = 1
        _nav_prev_raw     = effective_raw

    if _nav_stable_count >= NAV_STABILITY_MIN:
        if _nav_final_cmd != effective_raw:
            _LEGACY_PRIORITY = {
                NAV_EMERGENCY_STOP: 0, NAV_HOVER: 1,
                NAV_BACKWARD: 2, NAV_AVOID_LEFT: 3, NAV_AVOID_RIGHT: 3,
                NAV_STOP: 4, NAV_SLOW_FORWARD: 5, NAV_SAFE_SEARCH: 5,
                NAV_FAST_FORWARD: 6, NAV_FORWARD: 6, NAV_FORWARD_SCAN: 6,
                NAV_SEARCH_LEFT: 7, NAV_SEARCH_RIGHT: 7, NAV_SEARCH: 7,
            }
            current_rank = _LEGACY_PRIORITY.get(_nav_final_cmd, 99)
            new_rank     = _LEGACY_PRIORITY.get(effective_raw, 99)
            if new_rank <= current_rank or current_rank >= 6:
                _nav_final_cmd = effective_raw
                send_nav_token(ARDUINO_TOKENS.get(effective_raw, "?"))

    perf.record_command_latency((time.perf_counter() - t_start) * 1000)
    return _nav_final_cmd, obstacles, danger_obs


# ══════════════════════════════════════════════════════════════════════════════
#  NAVIGATION STATE CLASSIFIER (unchanged)
# ══════════════════════════════════════════════════════════════════════════════

def classify_nav_state(nav_cmd: str, obstacles: list) -> str:
    if nav_cmd == NAV_EMERGENCY_STOP:                   return NAV_STATE_EMERGENCY
    if nav_cmd == NAV_BACKWARD:                         return NAV_STATE_BLOCKED_FRONT
    if nav_cmd == NAV_AVOID_LEFT:                       return NAV_STATE_BLOCKED_RIGHT
    if nav_cmd == NAV_AVOID_RIGHT:                      return NAV_STATE_BLOCKED_LEFT
    if nav_cmd in (NAV_SAFE_SEARCH, NAV_STOP):          return NAV_STATE_DENSE
    if nav_cmd == NAV_SLOW_FORWARD:                     return NAV_STATE_CAUTION
    if nav_cmd in (NAV_SEARCH, NAV_SEARCH_LEFT,
                   NAV_SEARCH_RIGHT, NAV_FORWARD_SCAN): return NAV_STATE_SEARCH
    if nav_cmd in (NAV_FAST_FORWARD, NAV_FORWARD):      return NAV_STATE_CLEAR
    if nav_cmd == NAV_HOVER:                            return NAV_STATE_CAUTION
    return NAV_STATE_CLEAR


# ══════════════════════════════════════════════════════════════════════════════
#  MOTION-PRIMITIVE → ARDUINO COMMAND TRANSLATION
#
#  The Arduino sketch (final_verdict_auto.ino) accepts:
#    ARM | DISARM | STATUS | RECAL | RESETSTATS
#    THROTTLE <0-200>
#    ROLL  <±30°>
#    PITCH <±30°>
#    YAW   <±30°/s>
#
#  Each motion primitive maps to a (throttle, roll°, pitch°, yaw°/s) tuple.
#  Values are relative to IDLE_THR=55; full forward flight uses ~130 throttle.
#  Safe-test mode caps throttle automatically via cfg.safe_test_pwm_max scaling.
#
#  Sign conventions (X-frame, sketch mixMotors):
#    Roll+  → right tilt   Pitch+ → nose-up   Yaw+ → clockwise (from above)
# ══════════════════════════════════════════════════════════════════════════════

# (throttle 0-200,  roll °,  pitch °,  yaw °/s)
_MOTION_TO_FC: Dict[str, Tuple[int, float, float, float]] = {
    MOVE_FORWARD        : (110,   0.0, -10.0,  0.0),   # gentle nose-down → forward
    MOVE_FORWARD_FAST   : (140,   0.0, -18.0,  0.0),   # steeper pitch → fast forward
    MOVE_FORWARD_SLOW   : ( 90,   0.0,  -6.0,  0.0),   # shallow pitch → slow forward
    MOVE_BACKWARD       : ( 90,   0.0,  10.0,  0.0),   # nose-up → backward
    MOVE_YAW_LEFT       : ( 80,   0.0,   0.0, -20.0),  # yaw-left rate setpoint
    MOVE_YAW_RIGHT      : ( 80,   0.0,   0.0,  20.0),  # yaw-right rate setpoint
    MOVE_HOVER          : ( 75,   0.0,   0.0,  0.0),   # level hover
    MOVE_STOP           : ( 55,   0.0,   0.0,  0.0),   # back to idle throttle
    MOVE_SEARCH_LEFT    : ( 70,   0.0,   0.0, -12.0),  # slow yaw scan left
    MOVE_SEARCH_RIGHT   : ( 70,   0.0,   0.0,  12.0),  # slow yaw scan right
    MOVE_SCAN_FORWARD   : ( 85,   0.0,  -5.0,  0.0),   # creep forward while scanning
    MOVE_SAFE_SEARCH    : ( 65,   0.0,   0.0,  8.0),   # very slow yaw, low throttle
    MOVE_EMERGENCY_STOP : (  0,   0.0,   0.0,  0.0),   # zero throttle → DISARM
}

# Throttle ceiling in safe-test mode (maps 0-200 to 0-safe_test_pwm_max)
def _scale_throttle_safe_test(thr: int) -> int:
    """Rescale throttle linearly so 200 → safe_test_pwm_max."""
    if not _mode_is_safe_test():
        return thr
    return int(thr * cfg.safe_test_pwm_max / 200)


import queue as _queue


class ArduinoController(AbstractFlightController):
    """
    Arduino Nano flight controller — final_verdict_auto.ino protocol.

    Translates MotionPrimitive strings into the sketch's native serial
    command set: ARM / DISARM / THROTTLE / ROLL / PITCH / YAW.

    Key behaviours
    ──────────────
    • ARM on first motion command after connect; DISARM on EMERGENCY_STOP.
    • Keepalive thread sends STATUS every 1 s to beat the sketch's 2 s
      cmd-timeout failsafe (CMD_TO_MS 2000).  STATUS also refreshes
      lastCmdMs on the Arduino side.
    • Telemetry reader thread parses the sketch's 5 Hz [ARM]/[DIS] lines
      and JSON STATUS responses, storing them in self.telemetry.
    • Stale-command TTL preserved from v5: outdated commands are dropped.
    • Full reconnect logic preserved from v5.
    """

    # Keepalive must arrive faster than the sketch's CMD_TO_MS = 2000 ms
    _KEEPALIVE_INTERVAL: float = 0.8   # seconds between STATUS pings

    def __init__(self, port: str = SERIAL_PORT_DEFAULT,
                 baud: int = SERIAL_BAUD_DEFAULT,
                 enabled: bool = False):
        self.port          = port
        self.baud          = baud
        self.enabled       = enabled
        self._ser          = None
        self._last_motion  = ""
        self._last_send_t  = 0.0
        self._armed        = False
        self._lock         = threading.Lock()
        self._stop_evt     = threading.Event()
        self._ka_thread:  Optional[threading.Thread] = None
        self._telem_thread: Optional[threading.Thread] = None

        # Latest telemetry parsed from the sketch's serial output
        self.telemetry: Dict[str, object] = {
            "state": "UNKNOWN", "roll": 0.0, "pitch": 0.0,
            "imu": "UNKNOWN", "thr": 0,
            "FL": 0, "FR": 0, "RR": 0, "RL": 0,
            "hz": 0, "ovr": 0,
        }

        if enabled:
            self._connect()

    # ── Connection ────────────────────────────────────────────────────

    def _connect(self) -> bool:
        try:
            import serial as _serial
            self._ser = _serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2.0)        # allow Arduino bootloader to finish
            self._ser.reset_input_buffer()
            event_log.log("SERIAL", f"Connected to {self.port} @ {self.baud} baud")
            self._armed = False
            self._start_background_threads()
            return True
        except Exception as exc:
            event_log.log("SERIAL", f"Could not open {self.port}: {exc}")
            self._ser = None
            return False

    def _reconnect(self) -> bool:
        self._stop_evt.set()
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass
        self._ser  = None
        self._armed = False
        self._stop_evt.clear()
        perf.increment_reconnect()
        event_log.log("RECONNECT", "Attempting serial reconnect …")
        time.sleep(1.0)
        return self._connect()

    # ── Background threads ────────────────────────────────────────────

    def _start_background_threads(self) -> None:
        self._ka_thread = threading.Thread(
            target=self._keepalive_loop, name="FC_Keepalive", daemon=True)
        self._telem_thread = threading.Thread(
            target=self._telemetry_loop, name="FC_Telemetry", daemon=True)
        self._ka_thread.start()
        self._telem_thread.start()
        event_log.log("WATCHDOG",
                      f"FC keepalive + telemetry threads started "
                      f"(keepalive every {self._KEEPALIVE_INTERVAL}s)")

    def _keepalive_loop(self) -> None:
        """
        Sends STATUS every _KEEPALIVE_INTERVAL seconds.
        Beats the Arduino's CMD_TO_MS=2000 ms cmd-timeout failsafe so the
        drone does not auto-disarm during a pause in navigation commands.
        """
        while not self._stop_evt.is_set():
            time.sleep(self._KEEPALIVE_INTERVAL)
            if self._stop_evt.is_set():
                break
            with self._lock:
                if self._armed:
                    self._write_raw("STATUS")

    def _telemetry_loop(self) -> None:
        """
        Reads lines from the Arduino and parses:
          • JSON STATUS response  → self.telemetry dict
          • 5 Hz telemetry line   → self.telemetry dict (subset)
          • ARMED / DISARMED      → self._armed flag
        """
        import json as _json
        buf = ""
        while not self._stop_evt.is_set():
            try:
                if self._ser is None or not self._ser.is_open:
                    time.sleep(0.1)
                    continue
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode(errors="ignore").strip()
                if not line:
                    continue

                event_log.log("SERIAL", f"FC→PC: {line}")

                # JSON STATUS response  {"state":"ARMED","roll":...}
                if line.startswith("{"):
                    try:
                        data = _json.loads(line)
                        with self._lock:
                            self.telemetry.update(data)
                            self._armed = (data.get("state") == "ARMED")
                    except _json.JSONDecodeError:
                        pass
                    continue

                # 5 Hz telemetry  [ARM] R:0.12 P:-0.34 rPID:0.0 ...
                if line.startswith("[ARM]") or line.startswith("[DIS]"):
                    armed_now = line.startswith("[ARM]")
                    with self._lock:
                        self._armed = armed_now
                        self.telemetry["state"] = "ARMED" if armed_now else "DISARMED"
                    try:
                        # Parse key:value pairs from telemetry line
                        parts = line.split()
                        tmap: Dict[str, str] = {}
                        for part in parts[1:]:
                            if ":" in part:
                                k, v = part.split(":", 1)
                                tmap[k] = v
                        with self._lock:
                            if "R" in tmap:
                                self.telemetry["roll"]  = float(tmap["R"])
                            if "P" in tmap:
                                self.telemetry["pitch"] = float(tmap["P"])
                            if "FL" in tmap:
                                self.telemetry["FL"] = int(tmap["FL"])
                            if "FR" in tmap:
                                self.telemetry["FR"] = int(tmap["FR"])
                            if "RR" in tmap:
                                self.telemetry["RR"] = int(tmap["RR"])
                            if "RL" in tmap:
                                self.telemetry["RL"] = int(tmap["RL"])
                            if "thr" in tmap:
                                self.telemetry["thr"] = int(tmap["thr"])
                    except (ValueError, KeyError):
                        pass
                    continue

                # ARM / DISARM confirmations
                if "ARMED" in line and "DISARMED" not in line:
                    with self._lock:
                        self._armed = True
                        self.telemetry["state"] = "ARMED"
                elif "DISARMED" in line:
                    with self._lock:
                        self._armed = False
                        self.telemetry["state"] = "DISARMED"

            except Exception as exc:
                event_log.log("SERIAL", f"Telemetry read error: {exc}")
                time.sleep(0.1)

    # ── Raw serial write ──────────────────────────────────────────────

    def _write_raw(self, cmd: str) -> None:
        """Write a newline-terminated command string to serial."""
        payload = f"{cmd}\n".encode()
        try:
            if self._ser is None or not self._ser.is_open:
                if not self._reconnect():
                    return
            self._ser.write(payload)
        except Exception as exc:
            event_log.log("SERIAL", f"Write error ({cmd!r}): {exc} — reconnecting")
            try:
                if self._reconnect():
                    self._ser.write(payload)
            except Exception as exc2:
                event_log.log("SERIAL", f"Reconnect write failed: {exc2}")

    # ── Motion primitive → FC commands ───────────────────────────────

    def send_motion(self, motion: str, force: bool = False) -> None:
        """
        Translate a MotionPrimitive string into Arduino serial commands
        and transmit them.

        Sequence per motion change:
          1. ARM if not already armed (skip for EMERGENCY_STOP).
          2. Send THROTTLE <n>
          3. Send ROLL <f>
          4. Send PITCH <f>
          5. Send YAW <f>
          6. DISARM if EMERGENCY_STOP.
        """
        if not self.enabled:
            return

        now = time.time()
        with self._lock:
            if not force and now - self._last_send_t < COMMAND_SEND_INTERVAL:
                return
            if not force and motion == self._last_motion:
                return
            # Stale-command TTL
            age = time.time() - now
            if age > STALE_CMD_TTL:
                event_log.log("SERIAL",
                              f"Stale motion '{motion}' discarded (age={age:.3f}s)")
                return
            self._last_motion = motion
            self._last_send_t = now

        t0 = time.perf_counter()

        # ── Emergency: disarm immediately ─────────────────────────────
        if motion == MOVE_EMERGENCY_STOP:
            self._write_raw("DISARM")
            with self._lock:
                self._armed = False
            event_log.log("SERIAL", "DISARM sent — EMERGENCY_STOP")
            perf.record_serial_latency((time.perf_counter() - t0) * 1000)
            return

        # ── Arm if needed ─────────────────────────────────────────────
        with self._lock:
            currently_armed = self._armed
        if not currently_armed:
            self._write_raw("ARM")
            event_log.log("SERIAL", "ARM sent")
            time.sleep(0.05)   # brief pause for the sketch to process arm checks

        # ── Look up setpoints ─────────────────────────────────────────
        thr, roll, pitch, yaw = _MOTION_TO_FC.get(
            motion, _MOTION_TO_FC[MOVE_HOVER])

        thr = _scale_throttle_safe_test(thr)

        # ── Send all four setpoints ───────────────────────────────────
        self._write_raw(f"THROTTLE {thr}")
        self._write_raw(f"ROLL {roll:.1f}")
        self._write_raw(f"PITCH {pitch:.1f}")
        self._write_raw(f"YAW {yaw:.1f}")

        perf.record_serial_latency((time.perf_counter() - t0) * 1000)
        event_log.log("SERIAL",
                      f"motion={motion} → THR={thr} R={roll} P={pitch} Y={yaw}")

    def send_token(self, token: str, force: bool = False) -> None:
        """
        Legacy single-token interface retained for compatibility.
        The token is reverse-mapped to a MotionPrimitive where possible;
        otherwise treated as a raw Arduino command (e.g. 'STATUS').
        """
        # Reverse-map Arduino token → motion primitive string
        _TOKEN_TO_MOTION: Dict[str, str] = {v: k for k, v in ARDUINO_TOKENS.items()
                                             if k in _MOTION_TO_FC}
        motion = _TOKEN_TO_MOTION.get(token)
        if motion:
            self.send_motion(motion, force=force)
        else:
            # Raw command (STATUS, RECAL, RESETSTATS, HB ignored)
            if token not in ("HB",):
                with self._lock:
                    self._write_raw(token)

    def get_telemetry(self) -> Dict[str, object]:
        with self._lock:
            return dict(self.telemetry)

    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def is_armed(self) -> bool:
        with self._lock:
            return self._armed

    def close(self) -> None:
        self._stop_evt.set()
        # Disarm safely before closing
        if self.enabled and self._armed:
            try:
                self._write_raw("DISARM")
            except Exception:
                pass
        for t in (self._ka_thread, self._telem_thread):
            if t and t.is_alive():
                t.join(timeout=2.0)
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
                event_log.log("SERIAL", "Port closed")
        except Exception:
            pass


# Module-level flight controller singleton
if _mode_allows_serial():
    flight_controller: AbstractFlightController = ArduinoController(
        port=SERIAL_PORT_DEFAULT, baud=SERIAL_BAUD_DEFAULT, enabled=True
    )
else:
    flight_controller = DryRunController()

arduino = flight_controller   # legacy alias


def send_nav_token(token: str, force: bool = False) -> None:
    """
    Dispatch a navigation token to the flight controller.
    In REAL_FLIGHT_MODE the ArduinoController translates the motion
    primitive into proper ARM/THROTTLE/ROLL/PITCH/YAW commands.
    In all other modes the DryRunController no-ops.
    """
    if isinstance(flight_controller, ArduinoController):
        # Prefer the full motion translation path
        _TOKEN_TO_MOTION: Dict[str, str] = {v: k for k, v in ARDUINO_TOKENS.items()
                                             if k in _MOTION_TO_FC}
        motion = _TOKEN_TO_MOTION.get(token)
        if motion:
            flight_controller.send_motion(motion, force=force)
            return
    flight_controller.send_token(token, force=force)


# ══════════════════════════════════════════════════════════════════════════════
#  MASTER COMMAND ARBITER  (unchanged logic; motor abstraction integrated)
# ══════════════════════════════════════════════════════════════════════════════

_master_motion:      str   = MOVE_SAFE_SEARCH
_master_owner:       str   = OWNER_SEARCH
_master_nav_state:   str   = NAV_STATE_SEARCH
_master_last_change: float = 0.0


def master_decision(nav_cmd:      str,
                    ai_intent:    str,
                    ai_state:     str,
                    obstacles:    list,
                    stale_frame:  bool,
                    reader_alive: bool) -> Tuple[str, str, str, str]:
    """
    Master Command Arbiter — sole authority for final drone motion.

    Priority: EMERGENCY > NAVIGATION > AI TRACKING > SEARCH
    Returns (motion_primitive_str, motor_token, nav_state_str, owner_str).
    """
    global _master_motion, _master_owner, _master_nav_state, _master_last_change

    now       = time.time()
    nav_state = classify_nav_state(nav_cmd, obstacles)
    _master_nav_state = nav_state

    emergency_active = (_emergency_phase != _EMERG_IDLE)

    # ── Priority 1: EMERGENCY ─────────────────────────────────────────────
    if (emergency_active or stale_frame or not reader_alive
            or nav_state == NAV_STATE_EMERGENCY):
        motion = MOVE_EMERGENCY_STOP
        _commit_motion(motion, OWNER_EMERGENCY, now, force=True)
        return (_master_motion,
                MOTION_TOKENS.get(MotionPrimitive(motion),
                                  ARDUINO_TOKENS.get(motion, "ES")),
                nav_state, _master_owner)

    # ── Priority 2: NAVIGATION ────────────────────────────────────────────
    if nav_state == NAV_STATE_BLOCKED_FRONT:
        _commit_motion(MOVE_BACKWARD, OWNER_NAVIGATION, now, force=True)
    elif nav_state == NAV_STATE_BLOCKED_LEFT:
        _commit_motion(MOVE_YAW_RIGHT, OWNER_NAVIGATION, now)
    elif nav_state == NAV_STATE_BLOCKED_RIGHT:
        _commit_motion(MOVE_YAW_LEFT, OWNER_NAVIGATION, now)
    elif nav_state == NAV_STATE_DENSE:
        _commit_motion(MOVE_SAFE_SEARCH, OWNER_NAVIGATION, now)
    elif nav_state == NAV_STATE_CEILING:
        _commit_motion(MOVE_STOP, OWNER_NAVIGATION, now)
    elif nav_state == NAV_STATE_CAUTION and ai_state != STATE_TRACKING:
        _commit_motion(MOVE_FORWARD_SLOW, OWNER_NAVIGATION, now)

    # ── Priority 3: AI TRACKING ───────────────────────────────────────────
    elif ai_state == STATE_TRACKING:
        if ai_intent == AI_HOVER:
            motion = MOVE_HOVER
        elif ai_intent == AI_TRACK_LEFT:
            left_blocked = any(o.zone == "LEFT" and o.proximity >= SIDE_DANGER_FRAC
                               for o in obstacles)
            motion = MOVE_HOVER if left_blocked else MOVE_YAW_LEFT
        elif ai_intent == AI_TRACK_RIGHT:
            right_blocked = any(o.zone == "RIGHT" and o.proximity >= SIDE_DANGER_FRAC
                                for o in obstacles)
            motion = MOVE_HOVER if right_blocked else MOVE_YAW_RIGHT
        elif ai_intent == AI_TRACK_CENTER:
            # v6 Fix 9: only allow AI to push forward when nav has been stable
            # (prevents AI from fighting nav that just changed to CAUTION)
            nav_stable_enough = _nav_override_count >= _NAV_OVERRIDE_GUARD
            motion = MOVE_FORWARD_SLOW if (nav_state == NAV_STATE_CAUTION
                                           or _mode_is_safe_test()
                                           or not nav_stable_enough) else MOVE_FORWARD_FAST
        else:
            motion = MOVE_HOVER

        # v6 Fix 8: if the last committed motion was the opposite direction, insert
        # a HOVER buffer frame to prevent snapping directly between opposing motions
        _opposing = {
            MOVE_YAW_LEFT:  MOVE_YAW_RIGHT,
            MOVE_YAW_RIGHT: MOVE_YAW_LEFT,
        }
        if _opposing.get(motion) == _master_motion:
            motion = MOVE_HOVER   # one-frame HOVER buffer

        _commit_motion(motion, OWNER_AI, now)

    # ── Priority 4: NAVIGATION forward / search ───────────────────────────
    elif nav_state == NAV_STATE_CLEAR:
        motion = MOVE_FORWARD_SLOW if _mode_is_safe_test() else MOVE_FORWARD_FAST
        _commit_motion(motion, OWNER_NAVIGATION, now)
    else:
        if nav_cmd == NAV_SEARCH_LEFT:
            motion = MOVE_SEARCH_LEFT
        elif nav_cmd == NAV_SEARCH_RIGHT:
            motion = MOVE_SEARCH_RIGHT
        elif nav_cmd == NAV_FORWARD_SCAN:
            motion = MOVE_SCAN_FORWARD
        else:
            motion = MOVE_SAFE_SEARCH
        _commit_motion(motion, OWNER_SEARCH, now)

    token = ARDUINO_TOKENS.get(_master_motion, "?")
    return _master_motion, token, nav_state, _master_owner


def _commit_motion(motion: str, owner: str, now: float, force: bool = False) -> None:
    """
    Apply motion if cooldown/transition allows or force=True.
    Also drives the motor abstraction layer (DroneMixer).
    """
    global _master_motion, _master_owner, _master_last_change

    bypass = (force
              or motion == MOVE_EMERGENCY_STOP
              or motion == MOVE_BACKWARD)

    if not bypass and (now - _master_last_change) < COMMAND_HOLD_TIME:
        return

    try:
        current_mp = MotionPrimitive(motion) if motion != _master_motion else None
        prev_mp    = MotionPrimitive(_master_motion)
        if current_mp and not force:
            if not _validate_transition(prev_mp, current_mp):
                return
    except ValueError:
        pass

    if motion == _master_motion and owner == _master_owner:
        return

    prev               = _master_motion
    _master_motion     = motion
    _master_owner      = owner
    _master_last_change = now

    _apply_pid_smoothing(MotionPrimitive(motion) if motion in
                         [m.value for m in MotionPrimitive] else MotionPrimitive.SAFE_SEARCH)

    token = ARDUINO_TOKENS.get(motion, "?")
    event_log.log("OWNERSHIP",
                  f"{owner} → {motion}  (was: {prev})  token: '{token}'",
                  prev=prev, motion=motion, token=token)
    send_nav_token(token, force=force)

    # IMPROVEMENT 2: drive motor abstraction layer
    drone_mixer.apply(motion)


def _commit(candidate: str, owner: str, now: float, force: bool = False) -> None:
    if candidate == NAV_EMERGENCY_STOP:
        candidate = MOVE_EMERGENCY_STOP
    _commit_motion(candidate, owner, now, force=force)


# ══════════════════════════════════════════════════════════════════════════════
#  IMPROVEMENT 6 — SAFE SHUTDOWN MANAGER  (unchanged)
# ══════════════════════════════════════════════════════════════════════════════

class ShutdownManager:
    def __init__(self):
        self._threads: List[threading.Thread] = []

    def register(self, *threads: threading.Thread) -> None:
        self._threads.extend(threads)

    def run(self) -> None:
        print("[Shutdown] Initiating graceful shutdown …")
        event_log.log("WATCHDOG", "Shutdown initiated")
        _reader_alive.clear()
        try:
            send_nav_token(ARDUINO_TOKENS[NAV_EMERGENCY_STOP], force=True)
        except Exception as exc:
            print(f"[Shutdown] ES send error: {exc}")
        for t in self._threads:
            t.join(timeout=3)
            if t.is_alive():
                print(f"[Shutdown] Thread {t.name} did not exit cleanly")
        try:
            flight_controller.close()
        except Exception as exc:
            print(f"[Shutdown] Controller close error: {exc}")
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        event_log.close()
        print("[Shutdown] Complete")


# ══════════════════════════════════════════════════════════════════════════════
#  DRAW HELPERS (unchanged from v4)
# ══════════════════════════════════════════════════════════════════════════════

_FWD_PTS  = [(0.50,0.0),(1.0,0.6),(0.70,0.6),(0.70,1.0),(0.30,1.0),(0.30,0.6),(0.0,0.6)]
_BWD_PTS  = [(0.50,1.0),(1.0,0.4),(0.70,0.4),(0.70,0.0),(0.30,0.0),(0.30,0.4),(0.0,0.4)]
_LEFT_PTS = [(0.0,0.5),(0.6,0.0),(0.6,0.30),(1.0,0.30),(1.0,0.70),(0.6,0.70),(0.6,1.0)]
_RIGHT_PTS= [(1.0,0.5),(0.4,0.0),(0.4,0.30),(0.0,0.30),(0.0,0.70),(0.4,0.70),(0.4,1.0)]

_NAV_ARROW_DEFS: Dict[str, Optional[list]] = {
    NAV_FORWARD: _FWD_PTS, NAV_FAST_FORWARD: _FWD_PTS,
    NAV_SLOW_FORWARD: _FWD_PTS, NAV_BACKWARD: _BWD_PTS,
    NAV_AVOID_LEFT: _LEFT_PTS, NAV_AVOID_RIGHT: _RIGHT_PTS,
    NAV_SEARCH_LEFT: _LEFT_PTS, NAV_SEARCH_RIGHT: _RIGHT_PTS,
    NAV_HOVER: None, NAV_SEARCH: None, NAV_SAFE_SEARCH: None,
    NAV_FORWARD_SCAN: None, NAV_STOP: None, NAV_EMERGENCY_STOP: None,
    MOVE_FORWARD: _FWD_PTS, MOVE_FORWARD_FAST: _FWD_PTS,
    MOVE_FORWARD_SLOW: _FWD_PTS, MOVE_BACKWARD: _BWD_PTS,
    MOVE_YAW_LEFT: _LEFT_PTS, MOVE_YAW_RIGHT: _RIGHT_PTS,
    MOVE_SEARCH_LEFT: _LEFT_PTS, MOVE_SEARCH_RIGHT: _RIGHT_PTS,
    MOVE_HOVER: None, MOVE_STOP: None, MOVE_SCAN_FORWARD: None,
    MOVE_SAFE_SEARCH: None, MOVE_EMERGENCY_STOP: None,
}

_NAV_CMD_COLOR: Dict[str, Tuple[int, int, int]] = {
    NAV_FORWARD: (0,200,80), NAV_FAST_FORWARD: (0,255,50),
    NAV_SLOW_FORWARD: (0,200,160), NAV_STOP: (0,80,200),
    NAV_BACKWARD: (0,60,200), NAV_AVOID_LEFT: (0,200,200),
    NAV_AVOID_RIGHT: (0,200,200), NAV_HOVER: (0,180,255),
    NAV_SEARCH: (180,180,0), NAV_SEARCH_LEFT: (200,200,0),
    NAV_SEARCH_RIGHT: (200,200,0), NAV_FORWARD_SCAN: (0,200,100),
    NAV_SAFE_SEARCH: (160,160,0), NAV_EMERGENCY_STOP: (0,0,255),
    MOVE_FORWARD: (0,200,80), MOVE_FORWARD_FAST: (0,255,50),
    MOVE_FORWARD_SLOW: (0,200,160), MOVE_BACKWARD: (0,60,200),
    MOVE_YAW_LEFT: (0,200,200), MOVE_YAW_RIGHT: (0,200,200),
    MOVE_HOVER: (0,180,255), MOVE_STOP: (0,80,200),
    MOVE_SEARCH_LEFT: (200,200,0), MOVE_SEARCH_RIGHT: (200,200,0),
    MOVE_SCAN_FORWARD: (0,200,100), MOVE_SAFE_SEARCH: (160,160,0),
    MOVE_EMERGENCY_STOP: (0,0,255),
}

_DANGER_COLORS = {"LOW": (0,200,0), "MEDIUM": (0,165,255), "HIGH": (0,0,220)}


def _proximity_to_danger_label(proximity: float) -> str:
    if proximity >= FRONT_DANGER_FRAC: return "HIGH"
    if proximity >= SIDE_DANGER_FRAC:  return "MEDIUM"
    return "LOW"


def draw_zone_grid(frame: np.ndarray, frame_w: int, frame_h: int) -> None:
    left_x  = int(frame_w * LEFT_ZONE_END)
    right_x = int(frame_w * RIGHT_ZONE_START)
    top_y   = int(frame_h * TOP_ZONE_END)
    bot_y   = int(frame_h * BOTTOM_ZONE_START)
    alpha   = 0.20
    overlay = frame.copy()
    cv2.rectangle(overlay, (0,0), (left_x, frame_h), (255,200,100), -1)
    cv2.rectangle(overlay, (right_x,0), (frame_w, frame_h), (255,200,100), -1)
    cv2.rectangle(overlay, (0,0), (frame_w, top_y), (200,100,200), -1)
    cv2.rectangle(overlay, (0,bot_y), (frame_w, frame_h), (100,180,255), -1)
    cv2.addWeighted(overlay, alpha, frame, 1-alpha, 0, frame)
    cv2.line(frame, (left_x,0), (left_x,frame_h), (200,200,200), 1)
    cv2.line(frame, (right_x,0), (right_x,frame_h), (200,200,200), 1)
    cv2.line(frame, (0,top_y), (frame_w,top_y), (200,150,200), 1)
    cv2.line(frame, (0,bot_y), (frame_w,bot_y), (160,200,200), 1)
    lby = frame_h - 10
    cv2.putText(frame, "L", (left_x//2-6, lby), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200,200,200), 1)
    cv2.putText(frame, "C", ((left_x+right_x)//2-6, lby), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200,200,200), 1)
    cv2.putText(frame, "R", (right_x+(frame_w-right_x)//2-6, lby), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200,200,200), 1)
    cv2.putText(frame, "TOP", (4, top_y-3), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200,150,200), 1)
    cv2.putText(frame, "BOT", (4, bot_y+12), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (160,200,200), 1)


def draw_danger_boxes(frame: np.ndarray, obstacles: List[ObstacleInfo]) -> None:
    for obs in obstacles:
        x1, y1, x2, y2 = obs.box
        danger_label   = _proximity_to_danger_label(obs.proximity)
        colour         = _DANGER_COLORS[danger_label]
        cv2.rectangle(frame, (x1,y1), (x2,y2), colour, 2)
        info = f"{obs.zone}/{obs.v_zone}  {obs.proximity*100:.0f}%  {danger_label}"
        (tw, th), _ = cv2.getTextSize(info, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        lyt = max(0, y2+2)
        cv2.rectangle(frame, (x1,lyt), (x1+tw+6, lyt+th+6), colour, -1)
        cv2.putText(frame, info, (x1+3, lyt+th+2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,0,0), 1)


def _draw_nav_arrow(frame: np.ndarray, cmd: str, cx: int, cy: int, size: int = 52) -> None:
    colour = _NAV_CMD_COLOR.get(cmd, (255,255,255))
    half   = size // 2
    if cmd in (NAV_HOVER, MOVE_HOVER):
        cv2.circle(frame, (cx,cy), half, colour, 3)
        cv2.circle(frame, (cx,cy), half//2, colour, 2)
        cv2.circle(frame, (cx,cy), 4, colour, -1)
        return
    if cmd in (NAV_SEARCH, NAV_SAFE_SEARCH, MOVE_SAFE_SEARCH):
        r = int(half * 0.6)
        cv2.circle(frame, (cx-4,cy-4), r, colour, 2)
        cv2.line(frame, (cx-4+int(r*0.7), cy-4+int(r*0.7)),
                 (cx+half-4, cy+half-4), colour, 3)
        return
    if cmd in (NAV_STOP, MOVE_STOP):
        cv2.rectangle(frame, (cx-half+8,cy-half+8), (cx+half-8,cy+half-8), colour, -1)
        cv2.rectangle(frame, (cx-half+8,cy-half+8), (cx+half-8,cy+half-8), (255,255,255), 1)
        return
    if cmd in (NAV_EMERGENCY_STOP, MOVE_EMERGENCY_STOP):
        cv2.line(frame, (cx-half+6,cy-half+6), (cx+half-6,cy+half-6), (0,0,255), 4)
        cv2.line(frame, (cx+half-6,cy-half+6), (cx-half+6,cy+half-6), (0,0,255), 4)
        return
    if cmd in (NAV_FORWARD_SCAN, MOVE_SCAN_FORWARD):
        for offset in [0, 14]:
            pts = np.array([
                (cx, cy-half+6+offset), (cx+half-6, cy+offset),
                (cx, cy+10+offset), (cx-half+6, cy+offset),
            ], dtype=np.int32)
            cv2.polylines(frame, [pts], isClosed=False, color=colour, thickness=2)
        return
    pts_def = _NAV_ARROW_DEFS.get(cmd)
    if pts_def is None:
        return
    pts = np.array(
        [(int(cx-half+p[0]*size), int(cy-half+p[1]*size)) for p in pts_def],
        dtype=np.int32,
    )
    cv2.fillPoly(frame, [pts], colour)
    cv2.polylines(frame, [pts], isClosed=True, color=(255,255,255), thickness=1)


def draw_nav_overlay(frame: np.ndarray, nav_cmd: str,
                     obstacles: List[ObstacleInfo],
                     danger_obs: Optional[ObstacleInfo],
                     frame_w: int, frame_h: int) -> None:
    draw_zone_grid(frame, frame_w, frame_h)
    visible_obs = [o for o in obstacles if o.proximity >= SIDE_DANGER_FRAC * 0.7]
    draw_danger_boxes(frame, visible_obs)

    # ── Collision emergency banner (highest visual priority) ──────────────
    if camera_shake.check_collision_emergency():
        elapsed_col = camera_shake.collision_active_elapsed
        col_banner  = f"⚡ EMERGENCY COLLISION DETECTED  ({elapsed_col:.1f}s)  ⚡"
        (bw, bh), _ = cv2.getTextSize(col_banner, cv2.FONT_HERSHEY_SIMPLEX, 0.80, 2)
        bx = (frame_w - bw) // 2
        cv2.rectangle(frame, (0, 0), (frame_w, 50), (0, 0, 180), -1)
        cv2.putText(frame, col_banner, (max(4, bx), 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.80, (255, 80, 80), 2)

    is_emergency_cmd = nav_cmd in (NAV_EMERGENCY_STOP, MOVE_EMERGENCY_STOP,
                                   MOVE_BACKOFF)
    if is_emergency_cmd:
        in_recovery  = (_emergency_phase == _EMERG_RECOVERY)
        banner       = ("⚠  RECOVERING FROM EMERGENCY  ⚠"
                        if in_recovery else "⚠⚠  EMERGENCY STOP  ⚠⚠")
        banner_color = (0,140,200) if in_recovery else (0,0,220)
        (bw,bh),_   = cv2.getTextSize(banner, cv2.FONT_HERSHEY_SIMPLEX, 0.80, 2)
        bx = (frame_w - bw) // 2
        cv2.rectangle(frame, (0,0), (frame_w,46), banner_color, -1)
        cv2.putText(frame, banner, (bx,36), cv2.FONT_HERSHEY_SIMPLEX, 0.80, (255,255,255), 2)
    elif danger_obs is not None:
        danger_label = _proximity_to_danger_label(danger_obs.proximity)
        if danger_label == "HIGH":
            banner, banner_colour = (f"⚠ OBSTACLE  {nav_cmd}  ({danger_obs.proximity*100:.0f}%)",
                                     (0,0,220))
        elif danger_label == "MEDIUM":
            banner, banner_colour = (f"! CLOSE  {nav_cmd}  ({danger_obs.proximity*100:.0f}%)",
                                     (0,165,255))
        else:
            banner, banner_colour = None, None
        if banner:
            (bw,bh),_ = cv2.getTextSize(banner, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
            bx = (frame_w - bw) // 2
            cv2.rectangle(frame, (bx-8,8-bh-6), (bx+bw+8,20), banner_colour, -1)
            cv2.putText(frame, banner, (bx,14), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)
        hx1,hy1,hx2,hy2 = danger_obs.box
        cv2.rectangle(frame, (hx1-3,hy1-3), (hx2+3,hy2+3), (0,0,255), 3)

    arrow_cx, arrow_cy = frame_w-48, frame_h-60
    bg_overlay = frame.copy()
    cv2.circle(bg_overlay, (arrow_cx,arrow_cy), 36, (30,30,30), -1)
    cv2.addWeighted(bg_overlay, 0.55, frame, 0.45, 0, frame)
    _draw_nav_arrow(frame, nav_cmd, arrow_cx, arrow_cy, size=44)
    cv2.putText(frame, nav_cmd, (arrow_cx-38, arrow_cy+46),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                _NAV_CMD_COLOR.get(nav_cmd, (255,255,255)), 1)


# ══════════════════════════════════════════════════════════════════════════════
#  ADAPTIVE CPU CONTROL (unchanged, now driven by RuntimeMode)
# ══════════════════════════════════════════════════════════════════════════════

_low_power_mode:  bool = _mode_is_low_power()
_lp_frame_toggle: bool = False


def set_low_power_mode(enabled: bool) -> None:
    global _low_power_mode
    _low_power_mode = enabled
    print(f"[CPU] Low-power mode: {'ON' if enabled else 'OFF'}")


def _yolo_should_skip() -> bool:
    global _lp_frame_toggle
    if not _low_power_mode:
        return False
    _lp_frame_toggle = not _lp_frame_toggle
    if _lp_frame_toggle:
        perf.increment_dropped()
    return _lp_frame_toggle


# ══════════════════════════════════════════════════════════════════════════════
#  IP HELPER (unchanged)
# ══════════════════════════════════════════════════════════════════════════════

_PRIVATE_IP_RE = re.compile(
    r'\b(10\.\d{1,3}\.\d{1,3}\.\d{1,3}'
    r'|172\.(?:1[6-9]|2\d|3[01])\.\d{1,3}\.\d{1,3}'
    r'|192\.168\.\d{1,3}\.\d{1,3})\b'
)


def get_drone_ip(com_port="COM5", baud_rate=115200,
                 timeout_sec=30, max_attempts=None):
    try:
        import serial
        ser = serial.Serial(com_port, baud_rate, timeout=1)
    except Exception as e:
        print(f"[IP Error] {e}")
        return None
    start   = time.time()
    attempt = 0
    try:
        while True:
            if time.time() - start > timeout_sec:
                print(f"[IP Timeout] No IP in {timeout_sec}s")
                return None
            if max_attempts and attempt >= max_attempts:
                return None
            try:
                line    = ser.readline().decode(errors="ignore").strip()
                attempt += 1
                if not line:
                    continue
                m = _PRIVATE_IP_RE.search(line)
                if m:
                    ip = m.group(1)
                    print(f"[IP Found] {ip}")
                    return ip
            except Exception as e:
                print(f"[IP Read Error] {e}")
    finally:
        ser.close()


# ══════════════════════════════════════════════════════════════════════════════
#  LOAD MODEL
# ══════════════════════════════════════════════════════════════════════════════

try:
    model = YOLO(cfg.model)
    model.to(cfg.device)
    _dummy = np.zeros((cfg.imgsz, cfg.imgsz, 3), dtype=np.uint8)
    model.predict(_dummy, verbose=False, imgsz=cfg.imgsz)
    print(f"[Model] ✅ {cfg.model} loaded & warmed up on {cfg.device}")
except Exception as e:
    print(f"[Model Error] {e}")
    raise SystemExit(1)


# ══════════════════════════════════════════════════════════════════════════════
#  FRAME READER THREAD (unchanged)
# ══════════════════════════════════════════════════════════════════════════════

STREAM_PORT = 8080
stream_url  = f"http://192.168.1.5:{STREAM_PORT}/video"

_frame_lock    = threading.Lock()
_latest_frame: Optional[np.ndarray] = None
_frame_counter = 0
_reader_alive  = threading.Event()
_reader_alive.set()
_last_frame_timestamp: float = time.time()


def _open_cap(url: str, retries: int = 5, delay: float = 2.0):
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
        "analyzeduration;0|probesize;32|fflags;nobuffer|flags;low_delay"
    )
    for attempt in range(retries):
        cap = cv2.VideoCapture(url, cv2.CAP_ANY)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if cap.isOpened():
            event_log.log("RECONNECT", f"Stream connected: {url}")
            return cap
        cap.release()
        if attempt < retries - 1:
            event_log.log("RECONNECT", f"Retry {attempt+1}/{retries} in {delay}s")
            perf.increment_reconnect()
            time.sleep(delay)
    event_log.log("RECONNECT", f"All {retries} attempts failed for {url}")
    return None


def _frame_reader_loop(url: str):
    global _latest_frame, _frame_counter, _last_frame_timestamp
    cap = _open_cap(url)
    if cap is None:
        _reader_alive.clear()
        return
    while _reader_alive.is_set():
        ret, frame = cap.read()
        if not ret:
            event_log.log("RECONNECT", "Stream lost — reconnecting")
            cap.release()
            cap = _open_cap(url)
            if cap is None:
                event_log.log("RECONNECT", "Reconnect failed. Stopping reader.")
                _reader_alive.clear()
                break
            continue
        with _frame_lock:
            _latest_frame         = frame
            _frame_counter       += 1
            _last_frame_timestamp = time.time()
    cap.release()
    print("[Reader] Thread exiting.")


def _get_latest_frame():
    with _frame_lock:
        return _latest_frame, _frame_counter


# ══════════════════════════════════════════════════════════════════════════════
#  YOLO INFERENCE THREAD (unchanged from v4)
# ══════════════════════════════════════════════════════════════════════════════

_boxes_lock    = threading.Lock()
_latest_boxes: list = []
_ai_fps_val    = 0.0
_track_hits:   Dict[int, int] = defaultdict(int)
_track_misses: Dict[int, int] = defaultdict(int)
_last_inference_time: float   = 0.0


def _yolo_loop():
    global _latest_boxes, _ai_fps_val, _track_hits, _track_misses
    global _last_inference_time

    last_counter = -1
    t0           = time.time()
    ai_frames    = 0

    while _reader_alive.is_set():
        frame, counter = _get_latest_frame()
        if frame is None or counter == last_counter:
            time.sleep(0.005)
            continue
        if _yolo_should_skip():
            last_counter = counter
            continue
        now = time.time()
        if ADAPTIVE_MIN_INFERENCE_INTERVAL > 0:
            if now - _last_inference_time < ADAPTIVE_MIN_INFERENCE_INTERVAL:
                time.sleep(0.005)
                continue
        _last_inference_time = now
        last_counter         = counter
        infer_imgsz          = 320 if _low_power_mode else cfg.imgsz

        t_infer = time.perf_counter()
        try:
            results = model.track(
                frame, persist=True, conf=cfg.conf, iou=cfg.iou,
                imgsz=infer_imgsz, tracker="bytetrack.yaml", verbose=False, half=cfg.half,
            )
            perf.record_inference((time.perf_counter() - t_infer) * 1000)

            active_ids = set()
            new_boxes  = []
            for result in results:
                if result.boxes is None:
                    continue
                for box in result.boxes:
                    cls_id   = int(box.cls[0])
                    conf_val = float(box.conf[0])
                    tid      = int(box.id[0]) if box.id is not None else -1
                    active_ids.add(tid)
                    _track_hits[tid]   += 1
                    _track_misses[tid]  = 0
                    if _track_hits[tid] < SMOOTH_FRAMES:
                        continue
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    new_boxes.append((x1, y1, x2, y2, conf_val, tid, cls_id))

            for tid in list(_track_hits):
                if tid not in active_ids:
                    _track_misses[tid] += 1
                    if _track_misses[tid] > 8:
                        _track_hits.pop(tid, None)
                        _track_misses.pop(tid, None)

            if len(_depth_ema) > 200:
                evict_count = max(1, len(_depth_ema) // 4)
                for _tid in list(_depth_ema.keys())[:evict_count]:
                    _depth_ema.pop(_tid, None)
            if len(_track_hits) > 300:
                evict_count = max(1, len(_track_hits) // 4)
                for _tid in list(_track_hits.keys())[:evict_count]:
                    _track_hits.pop(_tid, None)
                    _track_misses.pop(_tid, None)

            with _boxes_lock:
                _latest_boxes = new_boxes

            ai_frames += 1
            elapsed    = time.time() - t0
            if elapsed >= 1.0:
                _ai_fps_val = ai_frames / elapsed
                ai_frames   = 0
                t0          = time.time()

        except Exception as e:
            print(f"[AI Error] {e}")

    print("[YOLO] Thread exiting.")


def _get_latest_boxes():
    with _boxes_lock:
        return list(_latest_boxes)


# ══════════════════════════════════════════════════════════════════════════════
#  DRAW BOX HELPERS (unchanged)
# ══════════════════════════════════════════════════════════════════════════════

def draw_boxes(frame: np.ndarray, boxes: list) -> None:
    for (x1, y1, x2, y2, conf_val, tid, cls_id) in boxes:
        is_human = (cls_id == HUMAN_CLASS)
        colour   = (0, 220, 0) if is_human else (60, 180, 255)
        if is_human:
            label = (f"Human #{tid} ({conf_val:.0%})" if tid >= 0
                     else f"Human ({conf_val:.0%})")
        else:
            cls_name = (model.names.get(cls_id, f"cls{cls_id}")
                        if hasattr(model, "names") else f"cls{cls_id}")
            label = (f"{cls_name} #{tid} ({conf_val:.0%})" if tid >= 0
                     else f"{cls_name} ({conf_val:.0%})")
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        label_y_top = max(0, y1 - th - 10)
        cv2.rectangle(frame, (x1, label_y_top), (x1+tw+6, y1), colour, -1)
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
        text_y = max(th+4, y1-4)
        cv2.putText(frame, label, (x1+3, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,0,0), 1)


def _scale_frame_for_display(frame: np.ndarray) -> np.ndarray:
    scale = float(cfg.display_scale)
    if scale == 1.0:
        return frame
    h, w  = frame.shape[:2]
    new_w = max(1, int(w * scale))
    new_h = max(1, int(h * scale))
    return cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)


# ══════════════════════════════════════════════════════════════════════════════
#  v7 FIX-9 — PROXIMITY DEBUG OVERLAY  (DEBUG_MODE only, zero-cost otherwise)
# ══════════════════════════════════════════════════════════════════════════════

def draw_proximity_debug_overlay(
    frame: np.ndarray,
    obstacles: List[ObstacleInfo],
    frame_w: int,
    frame_h: int,
) -> None:
    """
    Debug-only overlay showing per-obstacle proximity diagnostics and the
    emergency confirmation counter.  Drawn only in DEBUG_MODE — strict no-op
    in all other runtime modes.

    Visualises:
      • Centre-corridor boundary rectangle (the only zone checked for ES)
      • Per-obstacle: zone, v_zone, smoothed proximity %, colour-coded bar
      • Highlight when obstacle is inside the ES corridor
      • Emergency confirm counter progress vs required frames
      • Active entry / exit threshold values
    """
    if not _mode_is_debug():
        return

    # ── Centre-corridor boundary (the ES-eligible zone) ───────────────────
    lx = int(frame_w * LEFT_ZONE_END)
    rx = int(frame_w * RIGHT_ZONE_START)
    ty = int(frame_h * TOP_ZONE_END)
    by = int(frame_h * BOTTOM_ZONE_START)
    cv2.rectangle(frame, (lx, ty), (rx, by), (0, 255, 255), 1)
    cv2.putText(frame, "ES CORRIDOR", (lx + 4, ty + 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 255), 1)

    # ── Per-obstacle proximity bars ───────────────────────────────────────
    for obs in obstacles:
        x1, y1, x2, y2 = obs.box
        prox_pct = int(obs.proximity * 100)

        # Colour tier: green < 40%, orange < 70%, red >= 70%
        if obs.proximity < 0.40:
            bar_col = (0, 200, 0)
        elif obs.proximity < 0.70:
            bar_col = (0, 165, 255)
        else:
            bar_col = (0, 0, 220)

        # Vertical bar on left edge of detection box, height ∝ proximity
        bar_h   = max(4, int((y2 - y1) * obs.proximity))
        bar_top = y2 - bar_h
        cv2.rectangle(frame, (x1 - 8, bar_top), (x1 - 2, y2), bar_col, -1)

        # Proximity + zone label; cyan when inside the ES corridor
        in_corridor = (obs.zone == "CENTER" and obs.v_zone == "MIDDLE")
        label_col   = (0, 255, 255) if in_corridor else (180, 180, 180)
        cv2.putText(frame, f"{prox_pct}% {obs.zone}/{obs.v_zone}",
                    (x1, max(12, y1 - 2)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, label_col, 1)

    # ── Emergency confirmation counter ────────────────────────────────────
    conf_color = (0, 80, 220) if _emergency_confirm_count > 0 else (140, 140, 140)
    conf_text  = (f"ES confirm: {_emergency_confirm_count}/"
                  f"{_EMERGENCY_CONFIRM_FRAMES}  "
                  f"DangerConf:{danger_confidence.confidence:.2f}/{_DANGER_CONF_THRESH:.2f}  "
                  f"phase:{_emergency_phase}")
    cv2.rectangle(frame, (0, frame_h - 24), (850, frame_h), (20, 20, 20), -1)
    cv2.putText(frame, conf_text, (4, frame_h - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, conf_color, 1)

    # ── Threshold labels ──────────────────────────────────────────────────
    thresh_text = (f"ENTER>={_EMERGENCY_ENTER_FRAC:.0%}  "
                   f"EXIT<{_EMERGENCY_EXIT_FRAC:.0%}  "
                   f"min_area={_MIN_OBS_AREA_FRAC:.3f}  "
                   f"max_asp={_MAX_OBS_ASPECT:.1f}  "
                   f"confirm={_EMERGENCY_CONFIRM_FRAMES}fr")
    cv2.putText(frame, thresh_text, (4, frame_h - 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (160, 160, 160), 1)

    # ── Suppression state bar ────────────────────────────────────────────
    blur_s  = blur_guard.suppression_factor()
    shake_s = camera_shake.suppression_factor()
    combined = blur_s * shake_s
    s_col   = (0, 200, 0) if combined > 0.85 else (0, 165, 255) if combined > 0.50 else (0, 0, 220)
    supp_text = (f"BlurSupp:{blur_s:.2f}  ShakeFlow:{camera_shake.mean_flow:.1f}px  "
                 f"ShakeSupp:{shake_s:.2f}  Combined:{combined:.2f}  "
                 f"Blur={'ON' if blur_guard.is_suppressed else 'OFF'}  "
                 f"Shake={'ON' if camera_shake.is_shaking else 'OFF'}")
    cv2.putText(frame, supp_text, (4, frame_h - 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, s_col, 1)


# ══════════════════════════════════════════════════════════════════════════════
#  STALE / READER-FAIL OVERLAYS (unchanged)
# ══════════════════════════════════════════════════════════════════════════════

def _render_stale_overlay(frame: np.ndarray) -> None:
    h, w = frame.shape[:2]
    cv2.rectangle(frame, (0,0), (w,50), (0,0,200), -1)
    cv2.putText(frame, "⚠ STALE FRAME — EMERGENCY STOP  (stream frozen)",
                (10,34), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (255,255,255), 2)


def _render_reader_fail_overlay(frame: np.ndarray, elapsed: float) -> None:
    h, w      = frame.shape[:2]
    remaining = max(0.0, READER_FAIL_HOLD - elapsed)
    cv2.rectangle(frame, (0,0), (w,50), (0,0,160), -1)
    cv2.putText(frame,
                f"⚠ READER DEAD — EMERGENCY STOP  (shutdown in {remaining:.1f}s)",
                (10,34), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)


# ══════════════════════════════════════════════════════════════════════════════
#  v5 IMPROVEMENT 5 — SIMULATION HUD OVERLAY
#  Renders virtual sensor panel on the right side of the display frame.
#  Active in SIMULATION_MODE and SAFE_TEST_MODE.
# ══════════════════════════════════════════════════════════════════════════════

def _draw_battery_bar(frame: np.ndarray, x: int, y: int, pct: int) -> None:
    """Draw a small horizontal battery bar at (x, y)."""
    w, h   = 60, 10
    filled = int(w * pct / 100)
    colour = (0,200,0) if pct > 50 else (0,165,255) if pct > 20 else (0,0,220)
    cv2.rectangle(frame, (x, y), (x+w, y+h), (80,80,80), -1)
    cv2.rectangle(frame, (x, y), (x+filled, y+h), colour, -1)
    cv2.rectangle(frame, (x, y), (x+w, y+h), (200,200,200), 1)
    cv2.putText(frame, f"{pct}%", (x+w+4, y+h),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200,200,200), 1)


def draw_simulation_hud(frame: np.ndarray, frame_w: int, frame_h: int,
                         master_motion: str, master_owner: str,
                         nav_state: str) -> None:
    """
    Right-side simulation panel — virtual altitude, velocity, motor PWM,
    IMU state, battery gauge, and emergency indicator.

    Drawn only in SAFE_TEST_MODE or SIMULATION_MODE.
    """
    if not (_mode_is_safe_test() or _mode_is_simulation()):
        return

    vs   = virtual_sensors.snapshot()
    fl, fr, rl, rr = get_motor_pwm_snapshot()

    # Panel background
    panel_x = frame_w - 210
    cv2.rectangle(frame, (panel_x, 60), (frame_w-2, frame_h-2),
                  (20, 20, 20), -1)
    cv2.rectangle(frame, (panel_x, 60), (frame_w-2, frame_h-2),
                  (80, 80, 80), 1)

    lines = [
        ("── SIMULATION ────", (160,160,255)),
        (f"Alt   : {vs.altitude_m:.2f} m",       (200,200,200)),
        (f"Vel X : {vs.velocity_x:+.2f} m/s",    (200,200,200)),
        (f"Vel Y : {vs.velocity_y:+.2f} m/s",    (200,200,200)),
        (f"Dist  : {vs.obstacle_dist_m:.2f} m",
             (0,80,220) if vs.obstacle_warning else (200,200,200)),
        ("── IMU (MPU6050) ─", (160,160,255)),
        (f"Roll  : {vs.imu_roll_deg:+.1f}°",     (200,200,200)),
        (f"Pitch : {vs.imu_pitch_deg:+.1f}°",    (200,200,200)),
        (f"Yaw   : {vs.imu_yaw_deg:.1f}°",       (200,200,200)),
        ("── MOTORS (PWM) ──", (160,160,255)),
        (f"FL:{fl:3d}  FR:{fr:3d}",               (200,200,200)),
        (f"RL:{rl:3d}  RR:{rr:3d}",               (200,200,200)),
        ("── ARBITER ───────", (160,160,255)),
        (f"Owner : {master_owner}",               (200,200,200)),
        (f"Motion: {master_motion[:18]}",         (200,200,200)),
        (f"Nav   : {nav_state[:18]}",             (200,200,200)),
    ]

    y = 76
    for text, colour in lines:
        cv2.putText(frame, text, (panel_x+4, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, colour, 1)
        y += 16

    # Battery bar
    y += 4
    cv2.putText(frame, "Battery:", (panel_x+4, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200,200,200), 1)
    _draw_battery_bar(frame, panel_x+4, y+4, vs.battery_pct)
    y += 22

    # Battery voltage
    batt_col = (0,80,220) if vs.battery_warning else (0,200,100)
    cv2.putText(frame, f"{vs.battery_voltage:.2f} V",
                (panel_x+4, y+12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, batt_col, 1)
    y += 20

    # Emergency indicator
    if _emergency_phase != _EMERG_IDLE:
        cv2.rectangle(frame, (panel_x+2, y), (frame_w-4, y+18), (0,0,180), -1)
        cv2.putText(frame, f"EMERG: {_emergency_phase}",
                    (panel_x+4, y+13),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255,255,255), 1)

    # Safe-test banner
    if _mode_is_safe_test():
        cv2.rectangle(frame, (panel_x+2, frame_h-22), (frame_w-4, frame_h-4),
                      (0, 80, 140), -1)
        cv2.putText(frame, "SAFE TEST MODE",
                    (panel_x+4, frame_h-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255,255,255), 1)


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT — DISPLAY LOOP
# ══════════════════════════════════════════════════════════════════════════════

def main():
    router_ip = "http://192.168.1.1"
    print(f"[Serial] Drone IP: {router_ip}  ({ACTIVE_MODE.name})")

    reader_thread = threading.Thread(
        target=_frame_reader_loop, args=(stream_url,),
        name="FrameReader", daemon=True
    )
    yolo_thread = threading.Thread(
        target=_yolo_loop, name="YOLOInference", daemon=True
    )
    # IMPROVEMENT 6: motor ramp background thread
    motor_ramp_thread = threading.Thread(
        target=_motor_ramp_loop, name="MotorRamp", daemon=True
    )

    shutdown = ShutdownManager()
    shutdown.register(reader_thread, yolo_thread, motor_ramp_thread)

    reader_thread.start()
    motor_ramp_thread.start()
    print("[Main] Frame reader + motor ramp threads started")

    for _ in range(40):
        if not _reader_alive.is_set():
            print("[Main] Reader failed to start. Exiting.")
            return
        frame, _ = _get_latest_frame()
        if frame is not None:
            break
        time.sleep(0.1)
    else:
        print("[Main] Timed out waiting for first frame.")
        _reader_alive.clear()
        return

    yolo_thread.start()
    print("[Main] YOLO thread started")
    print("[Main] Press Q in the display window to quit.\n")

    display_fps_t0     = time.time()
    display_fps_frames = 0
    display_fps_val    = 0.0
    last_display_frame = None
    _reader_fail_start: float = 0.0

    try:
        while True:
            reader_alive = _reader_alive.is_set()

            if not reader_alive:
                if _reader_fail_start == 0.0:
                    _reader_fail_start = time.time()
                    event_log.log("WATCHDOG", "Reader died — holding EMERGENCY_STOP")
                    _force_emergency_nav_state()
                    send_nav_token(ARDUINO_TOKENS[NAV_EMERGENCY_STOP], force=True)

                elapsed_fail = time.time() - _reader_fail_start
                if elapsed_fail < READER_FAIL_HOLD:
                    _force_emergency_nav_state()
                    if last_display_frame is not None:
                        _render_reader_fail_overlay(last_display_frame, elapsed_fail)
                        draw_nav_overlay(last_display_frame, NAV_EMERGENCY_STOP,
                                         [], None, last_display_frame.shape[1],
                                         last_display_frame.shape[0])
                        cv2.imshow("Human Detection",
                                   _scale_frame_for_display(last_display_frame))
                    if cv2.waitKey(50) & 0xFF == ord('q'):
                        break
                    continue
                else:
                    print("[Main] Reader fail-safe hold complete — exiting")
                    break

            t_frame_start = time.perf_counter()
            frame, _      = _get_latest_frame()

            stale_now = (time.time() - _last_frame_timestamp) > STALE_FRAME_TIMEOUT
            if stale_now or frame is None:
                if last_display_frame is not None:
                    if stale_now:
                        _render_stale_overlay(last_display_frame)
                        _force_emergency_nav_state()
                        draw_nav_overlay(last_display_frame, NAV_EMERGENCY_STOP,
                                         [], None, last_display_frame.shape[1],
                                         last_display_frame.shape[0])
                    cv2.imshow("Human Detection",
                               _scale_frame_for_display(last_display_frame))
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break
                continue

            display_frame = frame.copy()
            boxes         = _get_latest_boxes()

            valid_boxes = []
            for b in boxes:
                try:
                    x1, y1, x2, y2, conf_v, tid_v, cls_v = b
                    if x2 > x1 and y2 > y1 and 0.0 <= conf_v <= 1.0:
                        valid_boxes.append(b)
                except (TypeError, ValueError):
                    pass
            boxes = valid_boxes

            human_count = sum(1 for b in boxes if b[6] == HUMAN_CLASS)
            _write_human_count(human_count)
            draw_boxes(display_frame, boxes)

            h_disp, w_disp = display_frame.shape[:2]

            drone_state, ai_intent, target_ctr = ai_decision(boxes, w_disp, h_disp)
            draw_ai_overlay(display_frame, target_ctr, w_disp, h_disp)

            nav_cmd, obstacles, danger_obs = nav_decision(boxes, w_disp, h_disp)

            master_motion, motor_token, nav_state, master_owner = master_decision(
                nav_cmd      = nav_cmd,
                ai_intent    = ai_intent,
                ai_state     = drone_state,
                obstacles    = obstacles,
                stale_frame  = False,
                reader_alive = _reader_alive.is_set(),
            )

            # IMPROVEMENT 4: advance virtual sensor suite
            virtual_sensors.tick(master_motion)

            # v8: update blur guard (uses raw boxes before size filtering)
            blur_guard.update(boxes, w_disp * h_disp)

            # v8: update camera-shake detector (uses raw display frame)
            camera_shake.update(display_frame)

            # v8: evict stale tracker state
            active_tids = {b[5] for b in boxes if b[5] >= 0}
            bbox_stability.evict_stale(active_tids)
            approach_tracker.evict_stale(active_tids)

            draw_nav_overlay(display_frame, master_motion, obstacles, danger_obs,
                             w_disp, h_disp)

            # IMPROVEMENT 5: simulation HUD overlay
            draw_simulation_hud(display_frame, w_disp, h_disp,
                                 master_motion, master_owner, nav_state)

            # v7 FIX-9: proximity debug overlay (DEBUG_MODE only — no-op otherwise)
            draw_proximity_debug_overlay(display_frame, obstacles, w_disp, h_disp)

            perf.record_frame_latency((time.perf_counter() - t_frame_start) * 1000)

            # ── Main HUD ──────────────────────────────────────────────────
            display_fps_frames += 1
            elapsed = time.time() - display_fps_t0
            if elapsed >= 1.0:
                display_fps_val    = display_fps_frames / elapsed
                display_fps_frames = 0
                display_fps_t0     = time.time()

            top_prox     = f"{obstacles[0].proximity*100:.0f}%" if obstacles else "N/A"
            mem_count    = len(_obstacle_memory)
            actual_imgsz = 320 if _low_power_mode else cfg.imgsz
            psnap        = perf.snapshot()
            fl, fr, rl, rr = get_motor_pwm_snapshot()

            hud = [
                f"Humans  : {human_count}",
                f"Device  : {cfg.tier}",
                f"Model   : {cfg.model}  imgsz={actual_imgsz}",
                f"Mode    : {ACTIVE_MODE.name}",
                f"Disp FPS: {display_fps_val:.1f}",
                f"AI  FPS : {_ai_fps_val:.1f}",
                f"── CONTROL STACK ──────────",
                f"AI INTENT   : {ai_intent}",
                f"NAV STATE   : {nav_state}",
                f"MASTER OWNER: {master_owner}",
                f"MOTION CMD  : {master_motion}",
                f"ARDUINO TX  : {motor_token}",
                f"── DIAGNOSTICS ────────────",
                f"Dr State: {drone_state}",
                f"NAV FSM : {nav_cmd}",
                f"Depth   : {top_prox}",
                f"ES Cnfrm: {_emergency_confirm_count}/{_EMERGENCY_CONFIRM_FRAMES}",
                f"Mem Obs : {mem_count}",
                f"Low Pwr : {'ON' if _low_power_mode else 'OFF'}",
                f"Serial  : {'ON' if flight_controller.is_connected() else 'OFF (dry-run)'}",
                f"── MOTORS (ramped) ─────────",       # IMPROVEMENT 2
                f"FL:{fl:3d}  FR:{fr:3d}",
                f"RL:{rl:3d}  RR:{rr:3d}",
                f"── PERFORMANCE ─────────────",
                f"YOLO ms : {psnap['yolo_ms']:.1f}",
                f"Frm  ms : {psnap['latency_ms']:.1f}",
                f"Cmd  ms : {psnap['cmd_ms']:.1f}",
                f"Ser  ms : {psnap['serial_ms']:.1f}",
                f"Dropped : {psnap['dropped']}",
                f"Reconn  : {psnap['reconnects']}",
            ]
            y = 28
            for line in hud:
                cv2.putText(display_frame, line, (10, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
                cv2.putText(display_frame, line, (10, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (30, 30, 30),  1)
                y += 26

            cv2.imshow("Human Detection", _scale_frame_for_display(display_frame))
            last_display_frame = display_frame

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[Main] User quit")
                break

    except KeyboardInterrupt:
        print("[Main] Ctrl+C — shutting down")
    except Exception as exc:
        print(f"[Main] Unexpected error: {exc}")
    finally:
        shutdown.run()


if __name__ == "__main__":
    main()