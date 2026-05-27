from __future__ import annotations

import os
import platform
import re
import subprocess
import threading
import time
from collections import defaultdict, deque
from typing import Optional, Tuple, Dict

import cv2
import numpy as np
import torch
from ultralytics import YOLO


# ══════════════════════════════════════════════════════════════════════════════
#  UPGRADE 1 — NAVIGATION-RELEVANT CLASS FILTER
#  Only objects that a real drone can physically collide with should affect
#  navigation decisions.  Filtering here prevents false avoidance from cups,
#  keyboards, remotes, and other tiny COCO objects.
# ══════════════════════════════════════════════════════════════════════════════

NAVIGATION_CLASSES: frozenset[int] = frozenset({
    0,    # person       — hover + human tracking
    56,   # chair        — common low indoor obstacle
    57,   # couch/sofa   — large floor obstacle
    58,   # potted plant — narrow but tall
    59,   # bed          — large floor area
    60,   # dining table — wide flat obstacle
    62,   # tv / monitor — wall-mounted / on stand
    63,   # laptop       — on table, sometimes head height
    # 64, mouse        — too small; excluded intentionally (cls 64)
    # NOTE: cls 64 (mouse) is deliberately NOT in this set to show intent.
    # Rule of thumb: only include objects whose bounding box will meaningfully
    # represent collision risk at drone cruise height (~0.5-1.5 m).
})

# Remove tiny objects that often misfire (mouse, remote, cell phone, etc.)
# by keeping only the set above.  Downstream: analyse_obstacles filters with:
#   if cls_id not in NAVIGATION_CLASSES: continue


# ══════════════════════════════════════════════════════════════════════════════
#  UPGRADE 5 — SPEED TIERS
#  Three discrete forward speeds allow the drone to decelerate gracefully as
#  obstacles approach rather than switching abruptly from FORWARD to BACKWARD.
# ══════════════════════════════════════════════════════════════════════════════

NAV_FAST_FORWARD  = "FAST_FORWARD"   # clear path, high cruise speed
NAV_SLOW_FORWARD  = "SLOW_FORWARD"   # medium obstacles ahead, cautious
NAV_STOP          = "STOP"           # very close object, hold in place

# Proximity thresholds that separate the three speed tiers (fraction of frame).
#   Objects < SPEED_CLEAR_FRAC  → FAST_FORWARD
#   Objects < SPEED_CAUTION_FRAC → SLOW_FORWARD
#   Objects ≥ SPEED_CAUTION_FRAC → STOP (let avoidance FSM handle direction)
SPEED_CLEAR_FRAC:   float = 0.04   # 4 % — object is small / far
SPEED_CAUTION_FRAC: float = 0.07   # 7 % — object is medium distance

# Arduino token map extended with speed tiers
ARDUINO_TOKENS_V3: dict[str, str] = {
    NAV_FAST_FORWARD  : "FF",
    NAV_SLOW_FORWARD  : "SF",
    NAV_STOP          : "ST",
}


# ══════════════════════════════════════════════════════════════════════════════
#  UPGRADE 8 — SEARCH FSM STATES
#  Replace single SEARCH with a cyclic 4-state mini-FSM so the drone
#  performs structured exploration instead of spinning indefinitely.
# ══════════════════════════════════════════════════════════════════════════════

NAV_SEARCH_LEFT   = "SEARCH_LEFT"    # slow left yaw
NAV_SEARCH_RIGHT  = "SEARCH_RIGHT"   # slow right yaw
NAV_FORWARD_SCAN  = "FORWARD_SCAN"   # brief straight advance during search
NAV_SAFE_SEARCH   = "SAFE_SEARCH"    # ultra-slow yaw when near walls

# Duration (seconds) each search sub-state is held before cycling
SEARCH_PHASE_DURATION: float = 2.5

# Search cycle order.  Drone yaws left, then right, then nudges forward,
# repeating.  If memory shows obstacles ahead the FORWARD_SCAN is skipped.
_SEARCH_CYCLE = [NAV_SEARCH_LEFT, NAV_SEARCH_RIGHT, NAV_FORWARD_SCAN]


# ══════════════════════════════════════════════════════════════════════════════
#  UPGRADE 11 — EMERGENCY SAFETY LAYER
# ══════════════════════════════════════════════════════════════════════════════

NAV_EMERGENCY_STOP = "EMERGENCY_STOP"

# Fraction of frame area that triggers an immediate emergency stop.
# At 640×480 this is roughly 75 % of the frame — object is essentially
# on top of the camera / drone.
EMERGENCY_AREA_FRAC:     float = 0.60

# If the FSM output changes more than this many times in one second the
# oscillation guard fires EMERGENCY_STOP.
OSCILLATION_GUARD_WINDOW: float = 1.0   # seconds
OSCILLATION_GUARD_LIMIT:  int   = 8     # command changes within window


# ══════════════════════════════════════════════════════════════════════════════
#  OBSTACLE AVOIDANCE — CONFIGURATION CONSTANTS  (v2 unchanged + v3 additions)
# ══════════════════════════════════════════════════════════════════════════════

FRONT_DANGER_FRAC: float = 0.10
SIDE_DANGER_FRAC:  float = 0.05
HUMAN_HOVER_FRAC:  float = 0.08

# Upgrade 6 — human hover needs CENTER-zone overlap ≥ this fraction
HUMAN_CENTER_OVERLAP_MIN: float = 0.40   # 40 % of human bbox must overlap CENTER

LEFT_ZONE_END:    float = 0.30
RIGHT_ZONE_START: float = 0.70

# ── Upgrade 7 — vertical zone boundaries ──────────────────────────────────
TOP_ZONE_END:    float = 0.30    # upper 30 % of frame height
BOTTOM_ZONE_START: float = 0.70  # lower 30 % of frame height

# ── Upgrade 2 — obstacle density threshold ────────────────────────────────
CENTER_DENSITY_LIMIT: int = 3   # ≥ this many CENTER objects → SAFE_SEARCH

# ── Upgrade 3 — temporal obstacle memory ──────────────────────────────────
LAST_OBSTACLE_TIMEOUT: float = 0.5   # seconds an obstacle stays in memory

# ── Navigation state labels (v2) ──────────────────────────────────────────
NAV_FORWARD      = "FORWARD"
NAV_BACKWARD     = "BACKWARD"
NAV_AVOID_LEFT   = "AVOID_LEFT"
NAV_AVOID_RIGHT  = "AVOID_RIGHT"
NAV_HOVER        = "HOVER"
NAV_SEARCH       = "SEARCH"

# Full Arduino token map (v2 + v3)
ARDUINO_TOKENS: dict[str, str] = {
    NAV_FORWARD       : "F",
    NAV_BACKWARD      : "B",
    NAV_AVOID_LEFT    : "L",
    NAV_AVOID_RIGHT   : "R",
    NAV_HOVER         : "H",
    NAV_SEARCH        : "S",
    NAV_FAST_FORWARD  : "FF",
    NAV_SLOW_FORWARD  : "SF",
    NAV_STOP          : "ST",
    NAV_SEARCH_LEFT   : "SL",
    NAV_SEARCH_RIGHT  : "SR",
    NAV_FORWARD_SCAN  : "FS",
    NAV_SAFE_SEARCH   : "SS",
    NAV_EMERGENCY_STOP: "ES",
}

NAV_STABILITY_MIN: int = 3

# ══════════════════════════════════════════════════════════════════════════════
#  FIX 1 — THROTTLED human_count.txt WRITES
#  Write only when count changes OR 0.5 s has elapsed to prevent per-frame
#  disk hammering (SSD wear, excessive CPU stalls on slow storage).
# ══════════════════════════════════════════════════════════════════════════════
HUMAN_COUNT_WRITE_INTERVAL: float = 0.5   # seconds between forced writes
_last_human_write_time: float = 0.0
_last_human_count:      int   = -1        # sentinel: force first write


def _write_human_count(count: int) -> None:
    """
    Write human_count.txt only when the count changed or the minimum
    write interval has elapsed.  Thread-safe via GIL on CPython for simple
    integer comparisons; no additional lock needed for this use-case.
    """
    global _last_human_write_time, _last_human_count
    now = time.time()
    if count == _last_human_count and (now - _last_human_write_time) < HUMAN_COUNT_WRITE_INTERVAL:
        return   # nothing to do
    try:
        with open("human_count.txt", "w") as fh:
            fh.write(str(count))
        _last_human_write_time = now
        _last_human_count      = count
    except OSError as exc:
        print(f"[HumanCount] Write error: {exc}")


# ══════════════════════════════════════════════════════════════════════════════
#  ORIGINAL AI DECISION LAYER  (human-tracking — UNCHANGED)
# ══════════════════════════════════════════════════════════════════════════════

STATE_IDLE      = "IDLE"
STATE_TRACKING  = "TRACKING"
STATE_SEARCHING = "SEARCHING"

_ai_state         = STATE_IDLE
_ai_decision_str  = "IDLE"
_ai_prev_decision = "IDLE"
_ai_same_count    = 0
_AI_STABILITY_MIN = 3
_AI_CENTER_THRESH = 0.18

HUMAN_CLASS = 0   # COCO class id for person (needed by ai_decision below)


def ai_decision(boxes: list, frame_w: int, frame_h: int):
    """Original human-tracking AI layer — unchanged from v2."""
    global _ai_state, _ai_decision_str, _ai_prev_decision, _ai_same_count

    frame_cx   = frame_w // 2
    human_boxes = [b for b in boxes if b[6] == HUMAN_CLASS]

    if not human_boxes:
        _ai_state     = STATE_SEARCHING
        raw_cmd       = "SEARCH"
        target_center = None
    else:
        def _area(b):
            return (b[2] - b[0]) * (b[3] - b[1])

        best = max(human_boxes, key=_area)
        x1, y1, x2, y2, conf, tid, cls_id = best
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        target_center = (cx, cy)

        offset_frac = (cx - frame_cx) / frame_w
        if offset_frac < -_AI_CENTER_THRESH:
            raw_cmd = "LEFT"
        elif offset_frac > _AI_CENTER_THRESH:
            raw_cmd = "RIGHT"
        else:
            raw_cmd = "FORWARD"

        _ai_state = STATE_TRACKING

    if raw_cmd == _ai_prev_decision:
        _ai_same_count += 1
    else:
        _ai_same_count    = 1
        _ai_prev_decision = raw_cmd

    if _ai_same_count >= _AI_STABILITY_MIN:
        if _ai_decision_str != raw_cmd:
            _ai_decision_str = raw_cmd
            # NOTE: logging suppressed here — [MASTER] arbiter logs the final cmd

    return _ai_state, _ai_decision_str, target_center


def draw_ai_overlay(frame: np.ndarray,
                    target_center,
                    frame_w: int, frame_h: int) -> None:
    """Draw AI tracking aids — unchanged from v2."""
    fc = (frame_w // 2, frame_h // 2)
    cv2.circle(frame, fc, 6, (0, 0, 255), -1)
    cv2.circle(frame, fc, 8, (255, 255, 255), 1)
    if target_center is not None:
        cv2.line(frame, fc, target_center, (255, 80, 0), 2)
        cv2.circle(frame, target_center, 5, (255, 80, 0), -1)


# ══════════════════════════════════════════════════════════════════════════════
#  UPGRADE 10 — ROBUST PSEUDO-DEPTH ESTIMATION
#  v2 used only bounding-box area, which fails for:
#    • large far objects (wide-angle walls)
#    • small close objects (narrow pillars)
#    • perspective tilt (objects at frame bottom are closer in a forward camera)
#
#  v3 blends four cues with empirically tuned weights:
#    (a) Normalised area           — primary distance proxy
#    (b) Normalised width ratio    — horizontal spread reveals planar walls
#    (c) Normalised height ratio   — vertical span useful for doors/people
#    (d) Vertical position bias    — objects lower in frame tend to be closer
#                                    for a slightly downward-tilted drone camera
#  Temporal smoothing over a short window reduces flicker from detection jitter.
# ══════════════════════════════════════════════════════════════════════════════

# Per-track exponential moving average alpha (higher = more responsive)
_DEPTH_EMA_ALPHA: float = 0.45

# Dict[track_id → smoothed_proximity]
_depth_ema: Dict[int, float] = {}


def estimate_pseudo_depth_v3(x1: int, y1: int, x2: int, y2: int,
                               frame_w: int, frame_h: int,
                               tid: int = -1) -> float:
    """
    Multi-cue pseudo-depth with EMA temporal smoothing.

    Returns normalised proximity in [0.0, 1.0]:
        0.0 → far / tiny
        1.0 → fills frame / extremely close

    Cue weights (sum to 1.0):
        area_cue     0.55  (dominant signal)
        width_cue    0.20  (wall detection)
        height_cue   0.15  (person/door detection)
        vpos_cue     0.10  (vertical position bias)
    """
    if frame_w <= 0 or frame_h <= 0:
        return 0.0

    bw = max(0, x2 - x1)
    bh = max(0, y2 - y1)

    area_cue   = min(1.0, (bw * bh) / (frame_w * frame_h))
    width_cue  = min(1.0, bw / frame_w)
    height_cue = min(1.0, bh / frame_h)

    # Vertical position: cy near bottom → closer (drone camera looks slightly down)
    cy_norm  = ((y1 + y2) / 2.0) / frame_h
    vpos_cue = float(np.clip(cy_norm, 0.0, 1.0))

    raw = (0.55 * area_cue
           + 0.20 * width_cue
           + 0.15 * height_cue
           + 0.10 * vpos_cue)
    raw = float(np.clip(raw, 0.0, 1.0))

    # EMA smoothing — only for properly tracked objects (tid ≥ 0)
    if tid >= 0:
        prev = _depth_ema.get(tid, raw)
        smoothed = _DEPTH_EMA_ALPHA * raw + (1.0 - _DEPTH_EMA_ALPHA) * prev
        _depth_ema[tid] = smoothed
        return smoothed
    return raw


def classify_horizontal_zone(cx: int, frame_w: int) -> str:
    """Map horizontal centre-x → 'LEFT' | 'CENTER' | 'RIGHT'."""
    frac = cx / frame_w
    if frac < LEFT_ZONE_END:
        return "LEFT"
    if frac > RIGHT_ZONE_START:
        return "RIGHT"
    return "CENTER"


# ── UPGRADE 7 — vertical zone classification ──────────────────────────────

def classify_vertical_zone(cy: int, frame_h: int) -> str:
    """
    Map vertical centre-y → 'TOP' | 'MIDDLE' | 'BOTTOM'.

    TOP    — ceiling / high-mounted objects (risk of clipping)
    MIDDLE — cruise altitude band (primary collision zone)
    BOTTOM — floor-level obstacles (legs of chairs, pets)
    """
    frac = cy / frame_h
    if frac < TOP_ZONE_END:
        return "TOP"
    if frac > BOTTOM_ZONE_START:
        return "BOTTOM"
    return "MIDDLE"


# ══════════════════════════════════════════════════════════════════════════════
#  OBSTACLE DATA CONTAINER  (extended from v2)
# ══════════════════════════════════════════════════════════════════════════════

class ObstacleInfo:
    """Lightweight obstacle descriptor — extended with vertical zone (v3)."""

    __slots__ = ("box", "proximity", "zone", "v_zone",
                 "is_human", "tid", "cls_id", "timestamp")

    def __init__(self,
                 box: Tuple[int, int, int, int],
                 proximity: float,
                 zone: str,
                 v_zone: str,
                 is_human: bool,
                 tid: int,
                 cls_id: int,
                 timestamp: float):
        self.box       = box
        self.proximity = proximity
        self.zone      = zone
        self.v_zone    = v_zone       # NEW v3: TOP | MIDDLE | BOTTOM
        self.is_human  = is_human
        self.tid       = tid
        self.cls_id    = cls_id
        self.timestamp = timestamp    # NEW v3: creation time for memory expiry


# ══════════════════════════════════════════════════════════════════════════════
#  UPGRADE 3 — TEMPORAL OBSTACLE MEMORY
#  Keeps recently seen obstacles alive for LAST_OBSTACLE_TIMEOUT seconds so
#  a single missed detection frame does not flush the navigation context.
#  This prevents the flickering FORWARD→AVOID→FORWARD→AVOID pattern.
# ══════════════════════════════════════════════════════════════════════════════

# Dict[tid → ObstacleInfo]  — persists across frames
_obstacle_memory: Dict[int, ObstacleInfo] = {}


def _update_obstacle_memory(live_obstacles: list[ObstacleInfo]) -> list[ObstacleInfo]:
    """
    Merge live detections into obstacle memory and prune expired entries.

    Algorithm:
      1. Update memory with each live detection (overwrites stale entry for
         same tid with fresh data + new timestamp).
      2. Prune any entry older than LAST_OBSTACLE_TIMEOUT.
      3. Return merged list (live + remembered) for this frame.

    Only tracked objects (tid ≥ 0) enter memory; anonymous detections (tid=-1)
    are passed through directly without persistence.
    """
    now = time.time()

    # Insert / refresh live detections
    for obs in live_obstacles:
        if obs.tid >= 0:
            _obstacle_memory[obs.tid] = obs   # timestamp already set to now

    # Prune expired entries
    expired = [tid for tid, obs in _obstacle_memory.items()
               if now - obs.timestamp > LAST_OBSTACLE_TIMEOUT]
    for tid in expired:
        del _obstacle_memory[tid]
        _depth_ema.pop(tid, None)   # also clean up depth EMA state

    # Merge: live takes priority; add remembered obstacles not in live set
    live_tids  = {o.tid for o in live_obstacles}
    remembered = [obs for tid, obs in _obstacle_memory.items()
                  if tid not in live_tids]

    merged = live_obstacles + remembered
    merged.sort(key=lambda o: o.proximity, reverse=True)
    return merged


# ══════════════════════════════════════════════════════════════════════════════
#  OBSTACLE ANALYSIS  (upgrade 1 + 7 + 10 integrated)
# ══════════════════════════════════════════════════════════════════════════════

def _compute_live_obstacles(boxes: list,
                             frame_w: int,
                             frame_h: int) -> list[ObstacleInfo]:
    """
    Compute ObstacleInfo for current-frame boxes WITHOUT merging obstacle memory.

    Used exclusively by the emergency layer so recovery decisions are based
    on what is visible NOW, not on EMA-smoothed or remembered detections.
    Mirrors analyse_obstacles but skips _update_obstacle_memory.
    """
    now  = time.time()
    live: list[ObstacleInfo] = []
    for (x1, y1, x2, y2, conf, tid, cls_id) in boxes:
        if cls_id not in NAVIGATION_CLASSES:
            continue
        cx        = (x1 + x2) // 2
        cy        = (y1 + y2) // 2
        # Use raw (non-EMA) proximity for the emergency gate so smoothed
        # history cannot keep the value artificially high after obstacle clears.
        proximity = estimate_pseudo_depth_v3(x1, y1, x2, y2, frame_w, frame_h, tid=-1)
        zone      = classify_horizontal_zone(cx, frame_w)
        v_zone    = classify_vertical_zone(cy, frame_h)
        is_human  = (cls_id == HUMAN_CLASS)
        live.append(ObstacleInfo(
            box=      (x1, y1, x2, y2),
            proximity=proximity,
            zone=     zone,
            v_zone=   v_zone,
            is_human= is_human,
            tid=      tid,
            cls_id=   cls_id,
            timestamp=now,
        ))
    return live


def analyse_obstacles(boxes: list,
                      frame_w: int,
                      frame_h: int) -> list[ObstacleInfo]:
    """
    Convert raw YOLO boxes → ObstacleInfo list with memory persistence.

    Changes from v2:
      • Class filter: skip objects not in NAVIGATION_CLASSES (upgrade 1)
      • Vertical zone classification (upgrade 7)
      • Multi-cue depth with EMA smoothing (upgrade 10)
      • Timestamp stamping for memory layer (upgrade 3)
    """
    now          = time.time()
    live: list[ObstacleInfo] = []

    for (x1, y1, x2, y2, conf, tid, cls_id) in boxes:

        # ── UPGRADE 1: Skip navigation-irrelevant objects ─────────────────
        if cls_id not in NAVIGATION_CLASSES:
            continue

        cx        = (x1 + x2) // 2
        cy        = (y1 + y2) // 2
        proximity = estimate_pseudo_depth_v3(x1, y1, x2, y2, frame_w, frame_h, tid)
        zone      = classify_horizontal_zone(cx, frame_w)
        v_zone    = classify_vertical_zone(cy, frame_h)   # UPGRADE 7
        is_human  = (cls_id == HUMAN_CLASS)

        live.append(ObstacleInfo(
            box       = (x1, y1, x2, y2),
            proximity = proximity,
            zone      = zone,
            v_zone    = v_zone,
            is_human  = is_human,
            tid       = tid,
            cls_id    = cls_id,
            timestamp = now,
        ))

    live.sort(key=lambda o: o.proximity, reverse=True)

    # ── UPGRADE 3: Merge with temporal memory ──────────────────────────────
    return _update_obstacle_memory(live)


# ══════════════════════════════════════════════════════════════════════════════
#  UPGRADE 6 — IMPROVED HUMAN HOVER LOGIC
#  v2 used only the bounding-box centre point to decide hover zone.  A tall
#  person at the frame edge can have their centre in CENTER even though most
#  of their body is off to the side.  Conversely, a person standing squarely
#  but detected with slight jitter may briefly have centre outside CENTER.
#
#  v3 computes the fraction of the human's bounding box that overlaps the
#  CENTER horizontal band (pixels between LEFT_ZONE_END and RIGHT_ZONE_START).
#  Hover is only triggered when overlap ≥ HUMAN_CENTER_OVERLAP_MIN.
# ══════════════════════════════════════════════════════════════════════════════

def _human_center_overlap(obs: ObstacleInfo, frame_w: int) -> float:
    """
    Compute what fraction of the human's bounding-box width lies inside
    the CENTER zone [LEFT_ZONE_END*frame_w, RIGHT_ZONE_START*frame_w].

    Returns a value in [0.0, 1.0]:
        0.0 → human entirely outside CENTER band
        1.0 → human entirely inside CENTER band
    """
    x1, y1, x2, y2 = obs.box
    box_w = max(1, x2 - x1)

    center_left  = int(frame_w * LEFT_ZONE_END)
    center_right = int(frame_w * RIGHT_ZONE_START)

    # Clamp human box edges to CENTER band
    overlap_left  = max(x1, center_left)
    overlap_right = min(x2, center_right)

    overlap_px = max(0, overlap_right - overlap_left)
    return overlap_px / box_w


# ══════════════════════════════════════════════════════════════════════════════
#  UPGRADE 8 — SEARCH FSM STATE MANAGER
#  Manages the cyclic search sub-state with timing so the drone does not spin
#  forever in one direction.
# ══════════════════════════════════════════════════════════════════════════════

_search_phase_index: int   = 0
_search_phase_start: float = time.time()   # FIX 3: initialise to now so the
# first search phase runs for a full SEARCH_PHASE_DURATION before advancing.
# The previous value of 0.0 caused an immediate phase skip on the very first
# call because (now - 0.0) was always ≥ SEARCH_PHASE_DURATION.

# FIX 4: track when FORWARD_SCAN sub-phase started so it can be duration-capped.
_forward_scan_start: float = 0.0

# FIX 6: Exploration recovery counter.
# After this many full LEFT→RIGHT cycles without a FORWARD_SCAN succeeding,
# inject one forced FORWARD_SCAN attempt (still obstacle-gated) to break the
# oscillation loop.  Keeps exploration CPU-lightweight with no SLAM/mapping.
_search_cycle_count:   int = 0   # increments each time phase wraps to 0
SEARCH_RECOVERY_CYCLES: int = 4  # inject forward scan after N full cycles


def _next_search_state(obstacles: list[ObstacleInfo]) -> str:
    """
    Return the current search sub-state, advancing phase when the timer
    expires.

    FIX 4 — FORWARD_SCAN is capped at FORWARD_SCAN_DURATION seconds AND
    cancelled immediately if a CENTER obstacle appears.

    FIX 6 — After SEARCH_RECOVERY_CYCLES full LEFT/RIGHT cycles without a
    successful FORWARD_SCAN, inject one forced attempt to break oscillation.
    The forced scan is still obstacle-gated — if obstacles block it we fall
    back to SAFE_SEARCH to preserve safety-first behaviour.
    """
    global _search_phase_index, _search_phase_start
    global _forward_scan_start, _search_cycle_count

    now = time.time()

    # ── Advance phase when timer expires ──────────────────────────────────
    if now - _search_phase_start >= SEARCH_PHASE_DURATION:
        prev_index          = _search_phase_index
        _search_phase_index = (_search_phase_index + 1) % len(_SEARCH_CYCLE)
        _search_phase_start = now

        # FIX 6: count completed full cycles (phase wraps from last → 0)
        if _search_phase_index == 0:
            _search_cycle_count += 1

    state = _SEARCH_CYCLE[_search_phase_index]

    # ── FIX 4: FORWARD_SCAN duration cap + real-time obstacle gate ────────
    if state == NAV_FORWARD_SCAN:
        center_obs = [o for o in obstacles if o.zone == "CENTER"
                      and o.proximity >= SIDE_DANGER_FRAC]
        if center_obs:
            # Obstacle ahead — stay in safe sideways search
            return NAV_SAFE_SEARCH

        # Start the scan timer on entry
        if _forward_scan_start == 0.0 or (now - _forward_scan_start) > SEARCH_PHASE_DURATION:
            _forward_scan_start = now

        # Cap the forward creep at FORWARD_SCAN_DURATION
        if (now - _forward_scan_start) >= FORWARD_SCAN_DURATION:
            # PATCH 5 — Micro-creep elapsed.  Advance FSM phase naturally
            # instead of hardcoding NAV_SEARCH_LEFT, which caused a leftward
            # bias and broke cycle symmetry.  The next phase in _SEARCH_CYCLE
            # is whatever follows FORWARD_SCAN (i.e. back to SEARCH_LEFT via
            # modulo wrap), so the cycle remains symmetric.
            _search_phase_index = (_search_phase_index + 1) % len(_SEARCH_CYCLE)
            _search_phase_start = now
            _forward_scan_start = 0.0
            return _SEARCH_CYCLE[_search_phase_index]   # follow cycle, no bias

        return NAV_FORWARD_SCAN

    # ── FIX 6: Exploration recovery injection ─────────────────────────────
    # After enough LEFT/RIGHT cycles without forward progress, attempt a
    # gated forward scan regardless of the normal cycle phase.
    if _search_cycle_count >= SEARCH_RECOVERY_CYCLES:
        _search_cycle_count = 0   # reset counter
        center_obs = [o for o in obstacles if o.zone == "CENTER"
                      and o.proximity >= SIDE_DANGER_FRAC]
        if not center_obs:
            # Path looks clear — inject a short forward scan
            _forward_scan_start = now
            _search_phase_index = _SEARCH_CYCLE.index(NAV_FORWARD_SCAN)
            _search_phase_start = now
            return NAV_FORWARD_SCAN
        # Still blocked — stay safe
        return NAV_SAFE_SEARCH

    return state


# ══════════════════════════════════════════════════════════════════════════════
#  UPGRADE 11 — EMERGENCY SAFETY LAYER  (oscillation guard state)
#  PATCH 7 — Full FSM with hysteresis and proper recovery
# ══════════════════════════════════════════════════════════════════════════════

_oscillation_timestamps: deque = deque()   # timestamps of dangerous command changes

# ── Emergency FSM state ───────────────────────────────────────────────────────
# Three explicit phases:
#   IDLE      → normal navigation, no emergency active
#   ACTIVE    → emergency hold (drone stopped, sending ES)
#   RECOVERY  → hold elapsed, waiting for proximity to drop below SAFE_RELEASE_FRAC
#               before returning to IDLE.  Prevents immediate re-entry flicker.
_EMERG_IDLE     = "IDLE"
_EMERG_ACTIVE   = "ACTIVE"
_EMERG_RECOVERY = "RECOVERY"

_emergency_phase:      str   = _EMERG_IDLE
_emergency_start_time: float = 0.0
_emergency_reason:     str   = ""

# ES token spam guard: track when ES was last transmitted so we only re-send
# periodically (watchdog keepalive) rather than every frame.
_emergency_last_sent:  float = 0.0
EMERGENCY_RESEND_INTERVAL: float = 0.8   # seconds between repeat ES tokens

# Hysteresis thresholds — entry is aggressive, exit is conservative.
# EMERGENCY_AREA_FRAC defined earlier (0.60) is used for entry.
SAFE_RELEASE_FRAC: float = 0.35   # proximity must drop below this to exit recovery

# PATCH 3 — Extended from 1.5 s to 2.5 s.
# 1.5 s was too short for WiFi jitter / temporary stream freezes; the
# emergency could release before the hazard fully cleared.  2.5 s gives
# enough headroom for a real-world network hiccup while not freezing the
# drone indefinitely if the scene recovers quickly.
EMERGENCY_HOLD_DURATION: float = 2.5       # hold EMERGENCY_STOP for N seconds

# ══════════════════════════════════════════════════════════════════════════════
#  FIX 2 — STALE FRAME THRESHOLD
#  If no new frame arrives within this many seconds, force EMERGENCY_STOP.
# ══════════════════════════════════════════════════════════════════════════════
STALE_FRAME_TIMEOUT:  float = 1.0   # seconds before stale-frame emergency fires
READER_FAIL_HOLD:     float = 1.5   # FIX 3: seconds to hold ES on reader death

# ══════════════════════════════════════════════════════════════════════════════
#  FIX 4 — FORWARD_SCAN DURATION CAP
#  FORWARD_SCAN is now a micro-creep that auto-cancels after this duration or
#  immediately if a CENTER obstacle appears during the scan window.
# ══════════════════════════════════════════════════════════════════════════════
FORWARD_SCAN_DURATION: float = 0.5   # seconds; short and immediately cancellable

# ══════════════════════════════════════════════════════════════════════════════
#  FIX 7 — SERIAL COMMAND OUTPUT LAYER
#  Throttled, duplicate-suppressed, queue-backed Arduino serial output.
# ══════════════════════════════════════════════════════════════════════════════
COMMAND_SEND_INTERVAL: float = 0.1   # minimum seconds between serial writes
SERIAL_PORT_DEFAULT:   str   = "COM5"
SERIAL_BAUD_DEFAULT:   int   = 115200
HEARTBEAT_INTERVAL:    float = 0.5   # FIX 8: seconds between HB tokens

# FIX 8 — Only count oscillations between commands that represent genuine
# axis-reversal instability.  Speed-tier transitions (FF→SF→ST), search
# sub-states, and hover transitions are normal behaviour in dynamic scenes
# and must NOT contribute to the oscillation count.
_OSCILLATION_PAIRS: frozenset[frozenset] = frozenset({
    frozenset({NAV_AVOID_LEFT,    NAV_AVOID_RIGHT}),
    frozenset({NAV_FAST_FORWARD,  NAV_BACKWARD}),
    frozenset({NAV_SLOW_FORWARD,  NAV_BACKWARD}),
    frozenset({NAV_FORWARD,       NAV_BACKWARD}),
    frozenset({NAV_STOP,          NAV_BACKWARD}),
})


def _is_dangerous_oscillation(cmd_a: str, cmd_b: str) -> bool:
    """Return True only if the transition between cmd_a and cmd_b represents
    a genuine navigation reversal that could indicate control instability."""
    return frozenset({cmd_a, cmd_b}) in _OSCILLATION_PAIRS


def _check_emergency(raw_cmd: str,
                     prev_raw: str,
                     live_obstacles: list[ObstacleInfo]) -> bool:
    """
    Three-phase emergency FSM: IDLE → ACTIVE → RECOVERY → IDLE.

    PATCH 7 — Full FSM with hysteresis to fix permanent-latch bug.

    Phases
    ------
    IDLE:
      • Evaluate entry triggers (giant obstacle, oscillation).
      • If triggered → transition to ACTIVE, log entry.

    ACTIVE (hold phase):
      • Always return True (ES) for EMERGENCY_HOLD_DURATION seconds.
      • Resend ES token periodically (watchdog keepalive, not every frame).
      • After hold elapses → transition to RECOVERY, log hold complete.

    RECOVERY:
      • Still return True (ES) until current-frame proximity drops below
        SAFE_RELEASE_FRAC on ALL live obstacles (hysteresis exit gate).
      • Key fix: uses ONLY live_obstacles (current frame detections, no
        obstacle memory, no EMA history) so stale values cannot block
        recovery indefinitely.
      • If safe → transition to IDLE, log release.
      • If still unsafe → remain in RECOVERY, log blocked reason.

    Oscillation guard accumulates across all phases; cleared on ACTIVE entry.

    Parameters
    ----------
    raw_cmd        : current raw navigation command (for oscillation check)
    prev_raw       : previous raw navigation command
    live_obstacles : obstacles from CURRENT frame only — NOT obstacle memory.
                     Caller (nav_decision) must pass pre-memory-merge live list.
    """
    global _emergency_phase, _emergency_start_time, _emergency_reason
    global _emergency_last_sent

    now = time.time()

    # ── Oscillation accumulation (runs in all phases) ─────────────────────
    if _is_dangerous_oscillation(raw_cmd, prev_raw):
        _oscillation_timestamps.append(now)
    while (_oscillation_timestamps
           and (now - _oscillation_timestamps[0]) > OSCILLATION_GUARD_WINDOW):
        _oscillation_timestamps.popleft()

    # ══════════════════════════════════════════════════════════════════════
    #  PHASE: IDLE  — evaluate entry triggers
    # ══════════════════════════════════════════════════════════════════════
    if _emergency_phase == _EMERG_IDLE:

        # Trigger A: large obstacle in current frame
        for obs in live_obstacles:
            if obs.proximity >= EMERGENCY_AREA_FRAC:
                _emergency_phase      = _EMERG_ACTIVE
                _emergency_start_time = now
                _emergency_reason     = f"obstacle {obs.proximity*100:.0f}% >= {EMERGENCY_AREA_FRAC*100:.0f}%"
                _emergency_last_sent  = 0.0   # force immediate ES send
                _oscillation_timestamps.clear()
                print(f"[EMERGENCY] Entered — {_emergency_reason}")
                return True

        # Trigger B: command oscillation
        if len(_oscillation_timestamps) > OSCILLATION_GUARD_LIMIT:
            _emergency_phase      = _EMERG_ACTIVE
            _emergency_start_time = now
            _emergency_reason     = "oscillation guard"
            _emergency_last_sent  = 0.0
            _oscillation_timestamps.clear()
            print(f"[EMERGENCY] Entered — {_emergency_reason}")
            return True

        return False   # IDLE, no trigger

    # ══════════════════════════════════════════════════════════════════════
    #  PHASE: ACTIVE  — hold for EMERGENCY_HOLD_DURATION
    # ══════════════════════════════════════════════════════════════════════
    if _emergency_phase == _EMERG_ACTIVE:
        elapsed = now - _emergency_start_time

        if elapsed < EMERGENCY_HOLD_DURATION:
            # Periodic resend so Arduino watchdog stays alive (not every frame)
            if now - _emergency_last_sent >= EMERGENCY_RESEND_INTERVAL:
                _emergency_last_sent = now
                print(f"[EMERGENCY] Holding ({elapsed:.1f}s / {EMERGENCY_HOLD_DURATION:.1f}s)")
            return True

        # Hold elapsed → enter recovery
        _emergency_phase      = _EMERG_RECOVERY
        _emergency_start_time = now   # reuse as recovery start for logging
        print("[EMERGENCY] Hold elapsed → entering RECOVERY phase")
        # Fall through to RECOVERY check immediately this frame

    # ══════════════════════════════════════════════════════════════════════
    #  PHASE: RECOVERY  — wait for proximity to drop (hysteresis exit gate)
    #  CRITICAL: use live_obstacles ONLY — no EMA, no obstacle memory.
    #  This prevents stale/smoothed proximity values from blocking recovery.
    # ══════════════════════════════════════════════════════════════════════
    if _emergency_phase == _EMERG_RECOVERY:

        # Collect max proximity from LIVE (current-frame) detections only.
        # No obstacle — proximity is 0.0 → safe to release.
        max_live_prox = max(
            (obs.proximity for obs in live_obstacles),
            default=0.0
        )

        if max_live_prox < SAFE_RELEASE_FRAC:
            # Safe conditions confirmed → release emergency
            _emergency_phase  = _EMERG_IDLE
            _emergency_reason = ""
            print(f"[EMERGENCY] Released — max live proximity {max_live_prox*100:.0f}% "
                  f"< {SAFE_RELEASE_FRAC*100:.0f}% threshold")
            return False   # IDLE: normal navigation resumes

        # Still unsafe — remain in recovery, log periodically
        if now - _emergency_last_sent >= EMERGENCY_RESEND_INTERVAL:
            _emergency_last_sent = now
            print(f"[EMERGENCY] Recovery blocked — live proximity {max_live_prox*100:.0f}% "
                  f">= release threshold {SAFE_RELEASE_FRAC*100:.0f}%")
        return True   # hold ES until clear

    # Fallback (should never reach here)
    return False


# ══════════════════════════════════════════════════════════════════════════════
#  UPGRADE 5 — SPEED-AWARE FORWARD COMMAND
# ══════════════════════════════════════════════════════════════════════════════

def _speed_tiered_forward(obstacles: list[ObstacleInfo]) -> str:
    """
    Choose FAST_FORWARD / SLOW_FORWARD / STOP based on the proximity of
    the nearest obstacle in the CENTER zone.

    If no CENTER obstacles exist, return FAST_FORWARD (clear path).
    """
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
#  CORE NAVIGATION LOGIC  (pure function — v3 rewrite of _nav_raw_decision)
# ══════════════════════════════════════════════════════════════════════════════

def _nav_raw_decision_v3(obstacles: list[ObstacleInfo],
                          frame_w: int,
                          frame_h: int) -> Tuple[str, str, Optional[ObstacleInfo]]:
    """
    Navigation FSM — pure function, no side-effects.

    Priority order (highest → lowest):
      0. EMERGENCY_STOP   — handled by caller; not evaluated here
      1. Human HOVER      — centred + close human → HOVER, never BACKWARD
                            (must precede collision checks — see note below)
      2. Frontal collision (non-human only) → BACKWARD
      3. Side collision   (non-human only) → AVOID_LEFT / AVOID_RIGHT
      4. Obstacle density in CENTER ≥ CENTER_DENSITY_LIMIT → SAFE_SEARCH
      5. TOP-zone obstacle: HIGH proximity → STOP, MEDIUM → SLOW_FORWARD
      6. Speed-tiered forward motion
      7. No detections → cyclic search

    FIX 1 — Human exclusion from avoidance branches
    ─────────────────────────────────────────────────
    Humans are *targets*, not obstacles.  The drone should approach and
    hover near a person, not retreat.  Placing hover logic at Priority 1
    means a centred close person is captured before the collision check
    at Priority 2.  In both Priority 2 and Priority 3 we additionally
    guard with `if obs.is_human: continue` so that a human who did NOT
    satisfy the hover conditions (e.g. too far off-centre) is silently
    skipped rather than driving BACKWARD or AVOID_*.  A human that
    doesn't qualify for hover is simply ignored by avoidance — the
    drone continues forward / holds position via Priority 6.

    FIX 5 — Stronger TOP-zone response
    ─────────────────────────────────────────────────
    A TOP obstacle at FRONT_DANGER_FRAC proximity is a ceiling collision
    risk; SLOW_FORWARD would still advance the drone into it.  We now
    return NAV_STOP for high-proximity ceiling objects and keep
    SLOW_FORWARD only for medium-risk overhead detections.
    """
    if not obstacles:
        return _next_search_state([]), NAV_SEARCH, None

    # ── Priority 1: Human HOVER (must run before any collision branch) ─────
    # Rationale: a person at close range in the CENTER zone has proximity ≥
    # FRONT_DANGER_FRAC, which would otherwise fire BACKWARD at Priority 2.
    # By evaluating hover first we guarantee the drone pauses in front of
    # people rather than fleeing from them.
    humans = [o for o in obstacles if o.is_human]
    for human in humans:
        overlap = _human_center_overlap(human, frame_w)
        if (human.proximity >= HUMAN_HOVER_FRAC
                and overlap >= HUMAN_CENTER_OVERLAP_MIN):
            return NAV_HOVER, NAV_HOVER, human

    # ── Priority 2: Frontal collision — non-human objects only ────────────
    # FIX 1: humans that failed hover (too far off-centre or too far away)
    # are skipped here.  They are tracked targets, not physical hazards to
    # avoid with a reverse manoeuvre.
    for obs in obstacles:   # sorted closest-first
        if obs.is_human:
            continue        # FIX 1: humans never trigger BACKWARD
        if obs.zone == "CENTER" and obs.proximity >= FRONT_DANGER_FRAC:
            return NAV_BACKWARD, NAV_BACKWARD, obs

    # ── Priority 3: Side collision — non-human objects only ───────────────
    # FIX 1: same guard — humans do not trigger lateral avoidance either.
    # An off-centre person would cause confusing sideways dodging behaviour.
    for obs in obstacles:
        if obs.is_human:
            continue        # FIX 1: humans never trigger AVOID_*
        if obs.zone == "LEFT" and obs.proximity >= SIDE_DANGER_FRAC:
            return NAV_AVOID_RIGHT, NAV_AVOID_RIGHT, obs
        if obs.zone == "RIGHT" and obs.proximity >= SIDE_DANGER_FRAC:
            return NAV_AVOID_LEFT, NAV_AVOID_LEFT, obs

    # ── Priority 4: Obstacle density — UPGRADE 2 ──────────────────────────
    center_objects = [o for o in obstacles if o.zone == "CENTER"]
    if len(center_objects) >= CENTER_DENSITY_LIMIT:
        return NAV_SAFE_SEARCH, NAV_SEARCH, None

    # ── Priority 5: Vertical awareness — FIX 5 + UPGRADE 7 ───────────────
    # FIX 5: split into two response tiers based on proximity severity.
    #   HIGH (≥ FRONT_DANGER_FRAC)  → NAV_STOP: ceiling contact imminent;
    #                                  advancing would cause a collision.
    #   MEDIUM (≥ SIDE_DANGER_FRAC) → NAV_SLOW_FORWARD: overhead object is
    #                                  present but not yet critical; slow down.
    top_threats = [o for o in obstacles
                   if o.v_zone == "TOP" and o.proximity >= SIDE_DANGER_FRAC]
    if top_threats:
        worst_top = max(top_threats, key=lambda o: o.proximity)
        if worst_top.proximity >= FRONT_DANGER_FRAC:
            # Ceiling / high-shelf imminent — stop completely
            return NAV_STOP, NAV_STOP, worst_top
        # Medium overhead risk — decelerate but keep moving
        return NAV_SLOW_FORWARD, NAV_FORWARD, worst_top

    # ── Priority 6: Speed-tiered FORWARD — UPGRADE 5 ──────────────────────
    fwd_cmd = _speed_tiered_forward(obstacles)
    return fwd_cmd, NAV_FORWARD, None


# ══════════════════════════════════════════════════════════════════════════════
#  PUBLIC NAVIGATION DECISION FUNCTION  (replaces nav_decision from v2)
# ══════════════════════════════════════════════════════════════════════════════

_nav_state        = NAV_SEARCH
_nav_prev_raw     = NAV_SEARCH
_nav_stable_count = 0
_nav_final_cmd    = NAV_SEARCH


def _force_emergency_nav_state() -> None:
    """
    PATCH 1 — Force the navigation FSM into EMERGENCY_STOP synchronously.

    Called from the stale-frame path and the reader-failure path in main().
    Without this, send_nav_token() would transmit "ES" to the Arduino while
    nav_cmd / _nav_final_cmd still held a stale value (e.g. "FORWARD").
    That desynchronisation caused:
      • The HUD to display the wrong command
      • draw_nav_overlay() to render the wrong arrow / banner
      • Any code reading _nav_final_cmd to see an unsafe stale state

    This function must be called BEFORE the frame is rendered so that both
    the overlay and the HUD display NAV_EMERGENCY_STOP consistently.
    It is intentionally lightweight (no lock needed — called from the single
    display loop thread) and does not touch the YOLO or reader threads.
    """
    global _nav_state, _nav_prev_raw, _nav_stable_count, _nav_final_cmd
    global _emergency_phase, _emergency_start_time, _emergency_reason, _emergency_last_sent
    _nav_final_cmd        = NAV_EMERGENCY_STOP
    _nav_state            = NAV_EMERGENCY_STOP
    _nav_prev_raw         = NAV_EMERGENCY_STOP
    _nav_stable_count     = 0
    # Force FSM into ACTIVE phase so it holds for the full duration
    if _emergency_phase == _EMERG_IDLE:
        _emergency_phase      = _EMERG_ACTIVE
        _emergency_start_time = time.time()
        _emergency_reason     = "forced (stale frame / reader failure)"
        _emergency_last_sent  = 0.0
        print(f"[EMERGENCY] Entered — {_emergency_reason}")


def nav_decision(boxes: list,
                 frame_w: int,
                 frame_h: int):
    """
    Public entry point for the navigation stack.

    Wraps _nav_raw_decision_v3 with:
      • Stability filter   (3-frame hold before output changes)
      • Emergency override (UPGRADE 11)
      • Serial token print + Arduino hook

    Parameters
    ----------
    boxes    : YOLO box list [(x1,y1,x2,y2,conf,tid,cls_id), ...]
    frame_w  : display frame width
    frame_h  : display frame height

    Returns
    -------
    (nav_cmd, obstacles, danger_obstacle)
    """
    global _nav_state, _nav_prev_raw, _nav_stable_count, _nav_final_cmd

    # Compute LIVE obstacles (before temporal memory merge) for the emergency
    # layer.  analyse_obstacles merges memory internally; we need the raw
    # current-frame list so the recovery gate isn't blocked by stale EMA values.
    _live_obs_for_emergency = _compute_live_obstacles(boxes, frame_w, frame_h)
    obstacles = analyse_obstacles(boxes, frame_w, frame_h)
    raw_cmd, new_state, danger_obs = _nav_raw_decision_v3(obstacles, frame_w, frame_h)

    _nav_state = new_state

    # ── UPGRADE 11: Emergency override (checked before stability filter) ──
    if _check_emergency(raw_cmd, _nav_prev_raw, _live_obs_for_emergency):
        _nav_final_cmd = NAV_EMERGENCY_STOP
        send_nav_token(ARDUINO_TOKENS[NAV_EMERGENCY_STOP], force=True)
        return _nav_final_cmd, obstacles, danger_obs

    # ── Stability filter ────────────────────────────────────────────────────
    if raw_cmd == _nav_prev_raw:
        _nav_stable_count += 1
    else:
        _nav_stable_count = 1
        _nav_prev_raw     = raw_cmd

    if _nav_stable_count >= NAV_STABILITY_MIN:
        if _nav_final_cmd != raw_cmd:
            # ── FIX 9: Strict command priority — lower-priority commands
            # cannot override a currently active higher-priority state until
            # the higher-priority condition fully clears.
            # Priority rank (lower number = higher priority):
            _PRIORITY_RANK: dict = {
                NAV_EMERGENCY_STOP: 0,
                NAV_HOVER:          1,
                NAV_BACKWARD:       2,
                NAV_AVOID_LEFT:     3,
                NAV_AVOID_RIGHT:    3,
                NAV_STOP:           4,
                NAV_SLOW_FORWARD:   5,
                NAV_SAFE_SEARCH:    5,
                NAV_FAST_FORWARD:   6,
                NAV_FORWARD:        6,
                NAV_FORWARD_SCAN:   6,
                NAV_SEARCH_LEFT:    7,
                NAV_SEARCH_RIGHT:   7,
                NAV_SEARCH:         7,
            }
            current_rank = _PRIORITY_RANK.get(_nav_final_cmd, 99)
            new_rank     = _PRIORITY_RANK.get(raw_cmd, 99)
            # Allow transition only if new command is equal or higher priority
            # — OR if current state is already a search/forward (low priority).
            if new_rank <= current_rank or current_rank >= 6:
                _nav_final_cmd = raw_cmd
                send_nav_token(ARDUINO_TOKENS.get(raw_cmd, "?"))   # keep serial hot

    return _nav_final_cmd, obstacles, danger_obs


# ══════════════════════════════════════════════════════════════════════════════
#  MASTER COMMAND ARBITER
#  Single authority that arbitrates among all sub-systems and emits the one
#  final motor command.  Architecture:
#
#    PERCEPTION (YOLO)
#         │
#    ┌────┴──────────────────────────────────────────────────┐
#    │  Emergency FSM  │  Nav avoidance FSM  │  AI tracking  │
#    └────┬──────────────────────────────────────────────────┘
#         │  all candidates → master_decision()
#         ▼
#    MASTER ARBITER  (priority + cooldown)
#         │
#    Arduino serial token
#
#  Priority (highest→lowest):
#    1. EMERGENCY  — any emergency phase active or stale/dead stream
#    2. NAVIGATION — avoidance states (BACKWARD, AVOID_*, STOP, SAFE_SEARCH,
#                    SLOW_FORWARD, speed-tier STOP, FORWARD_SCAN)
#    3. AI_TRACKING — human tracking (FORWARD/LEFT/RIGHT/HOVER) when safe
#    4. SEARCH     — exploration FSM when no target and path is clear
#
#  Command cooldown: commands may not change faster than COMMAND_HOLD_TIME
#  unless the new command is EMERGENCY_STOP or BACKWARD (safety override).
# ══════════════════════════════════════════════════════════════════════════════

# Owner labels shown on HUD
OWNER_EMERGENCY  = "EMERGENCY"
OWNER_NAVIGATION = "NAVIGATION"
OWNER_AI         = "AI_TRACKING"
OWNER_SEARCH     = "SEARCH"

# Minimum wall-clock seconds a command is held before another non-critical
# command may replace it.  Prevents sub-frame jitter from thrashing the drone.
COMMAND_HOLD_TIME: float = 0.35

# Navigation avoidance commands that indicate an active obstacle response.
# When any of these is the current nav output, AI tracking is suppressed.
_NAV_AVOIDANCE_CMDS: frozenset[str] = frozenset({
    NAV_BACKWARD,
    NAV_AVOID_LEFT,
    NAV_AVOID_RIGHT,
    NAV_STOP,
    NAV_SAFE_SEARCH,
    NAV_SLOW_FORWARD,   # caution-speed counts as avoidance-influenced
})

# AI commands allowed to control the drone when no higher priority is active.
_AI_ALLOWED_CMDS: frozenset[str] = frozenset({
    "FORWARD", "LEFT", "RIGHT", "HOVER",
    NAV_FORWARD,        # ai_decision outputs these string variants
    NAV_HOVER,
    NAV_FAST_FORWARD,
})

# Module-level arbiter state
_master_cmd:         str   = NAV_SEARCH
_master_owner:       str   = OWNER_SEARCH
_master_last_change: float = 0.0


def master_decision(nav_cmd:    str,
                    ai_cmd:     str,
                    ai_state:   str,
                    obstacles:  list,
                    stale_frame: bool,
                    reader_alive: bool) -> tuple[str, str]:
    """
    Master Command Arbiter — sole source of final drone commands.

    Parameters
    ----------
    nav_cmd      : output of nav_decision() (already stability-filtered)
    ai_cmd       : output of ai_decision() (_ai_decision_str)
    ai_state     : STATE_TRACKING / STATE_SEARCHING / STATE_IDLE
    obstacles    : merged obstacle list from nav_decision()
    stale_frame  : True if video feed has frozen (FIX 2 stale check)
    reader_alive : True if frame-reader thread is alive

    Returns
    -------
    (final_cmd, owner)  — final_cmd is sent to Arduino; owner labels HUD.
    """
    global _master_cmd, _master_owner, _master_last_change

    now = time.time()

    # ── PRIORITY 1: EMERGENCY ─────────────────────────────────────────────
    # Absolute override — nothing may suppress it.
    # Triggers: emergency FSM active, stale frame, reader dead, or nav
    # itself already escalated to EMERGENCY_STOP.
    emergency_active = (_emergency_phase != _EMERG_IDLE)
    if (emergency_active
            or stale_frame
            or not reader_alive
            or nav_cmd == NAV_EMERGENCY_STOP):
        candidate = NAV_EMERGENCY_STOP
        owner     = OWNER_EMERGENCY
        # Cooldown bypassed for emergency — always apply immediately
        _commit(candidate, owner, now, force=True)
        return _master_cmd, _master_owner

    # ── PRIORITY 2: NAVIGATION avoidance FSM ──────────────────────────────
    # Active when nav has detected an obstacle requiring evasive action.
    # AI tracking is completely suppressed in this state.
    if nav_cmd in _NAV_AVOIDANCE_CMDS:
        candidate = nav_cmd
        owner     = OWNER_NAVIGATION
        _commit(candidate, owner, now)
        return _master_cmd, _master_owner

    # ── PRIORITY 3: AI TRACKING ───────────────────────────────────────────
    # Only allowed when:
    #   • no avoidance active (checked above)
    #   • AI is actively tracking a human (STATE_TRACKING)
    #   • AI command is a meaningful directional/hover command
    if (ai_state == STATE_TRACKING
            and ai_cmd in _AI_ALLOWED_CMDS):
        # Map ai_decision string variants to canonical NAV constants
        _AI_CMD_MAP = {
            "FORWARD": NAV_FAST_FORWARD,
            "LEFT"   : NAV_AVOID_LEFT,    # AI left → drone yaw/strafe left
            "RIGHT"  : NAV_AVOID_RIGHT,
            "HOVER"  : NAV_HOVER,
        }
        canonical = _AI_CMD_MAP.get(ai_cmd, ai_cmd)
        candidate = canonical
        owner     = OWNER_AI
        _commit(candidate, owner, now)
        return _master_cmd, _master_owner

    # ── PRIORITY 4: NAVIGATION forward / speed tiers ──────────────────────
    # Nav has a valid non-avoidance command (FAST_FORWARD, FORWARD_SCAN…).
    # Covers the case where AI has no target but nav sees a clear path.
    if nav_cmd not in (NAV_SEARCH, NAV_SEARCH_LEFT, NAV_SEARCH_RIGHT,
                       NAV_SAFE_SEARCH, NAV_FORWARD_SCAN):
        candidate = nav_cmd
        owner     = OWNER_NAVIGATION
        _commit(candidate, owner, now)
        return _master_cmd, _master_owner

    # ── PRIORITY 5 (lowest): SEARCH FSM ──────────────────────────────────
    # Activated only when no target and no obstacle danger.
    candidate = nav_cmd   # search sub-states come from nav_decision / _next_search_state
    owner     = OWNER_SEARCH
    _commit(candidate, owner, now)
    return _master_cmd, _master_owner


def _commit(candidate: str, owner: str, now: float, force: bool = False) -> None:
    """
    Apply candidate command if cooldown allows or force=True.

    Cooldown is bypassed for:
      • EMERGENCY_STOP  (always immediate)
      • BACKWARD        (safety-critical reversal)
      • Any command when force=True is passed

    A change is logged as [MASTER] only when the final command or owner
    actually changes — no duplicate prints.
    """
    global _master_cmd, _master_owner, _master_last_change

    # Safety overrides bypass cooldown entirely
    bypass_cooldown = (force
                       or candidate == NAV_EMERGENCY_STOP
                       or candidate == NAV_BACKWARD)

    elapsed_since_change = now - _master_last_change
    if not bypass_cooldown and elapsed_since_change < COMMAND_HOLD_TIME:
        # Cooldown active — keep current command
        return

    if candidate == _master_cmd and owner == _master_owner:
        return   # no change; nothing to do

    prev_cmd   = _master_cmd
    _master_cmd         = candidate
    _master_owner       = owner
    _master_last_change = now

    token = ARDUINO_TOKENS.get(candidate, "?")
    print(f"[MASTER] {owner:12s} → {candidate:20s}  (was: {prev_cmd})  token: '{token}'")
    send_nav_token(token, force=force)


# ══════════════════════════════════════════════════════════════════════════════
#  FIX 7 + 8 — SERIAL COMMAND OUTPUT LAYER WITH WATCHDOG HEARTBEAT
#  Provides throttled, duplicate-suppressed Arduino serial transmission with
#  a periodic heartbeat so the Arduino can STOP MOTORS if Python freezes.
#
#  Design:
#    • ArduinoSerial wraps pyserial; safe to construct even without hardware
#      (import guard — pyserial optional).
#    • send_nav_token() is the single call-site: called from nav_decision output
#      and the stale-frame / reader-fail emergency paths.
#    • Heartbeat thread sends "HB\n" every HEARTBEAT_INTERVAL seconds on a
#      background daemon thread — zero impact on nav loop timing.
#    • All serial writes are protected with try/except so a disconnected cable
#      never crashes the navigation stack.
# ══════════════════════════════════════════════════════════════════════════════

import queue as _queue

class ArduinoSerial:
    """
    Lightweight serial wrapper for Arduino Nano communication.

    Features:
      • Automatic reconnect on write failure
      • Command deduplication (same token not re-sent until it changes)
      • Per-command cooldown of COMMAND_SEND_INTERVAL seconds
      • Background heartbeat thread (FIX 8)
      • All paths protected with try/except — never raises into caller
    """

    def __init__(self,
                 port: str  = SERIAL_PORT_DEFAULT,
                 baud: int  = SERIAL_BAUD_DEFAULT,
                 enabled: bool = False):
        """
        Parameters
        ----------
        port    : COM port (Windows) or /dev/ttyUSB0 (Linux)
        baud    : must match Arduino sketch (default 115200)
        enabled : False → dry-run mode (tokens printed but not sent)
                  Set True only when hardware is physically connected.
        """
        self.port            = port
        self.baud            = baud
        self.enabled         = enabled
        self._ser            = None          # pyserial Serial object or None
        self._last_token     = ""            # duplicate suppression
        self._last_send_t    = 0.0           # cooldown timestamp
        self._hb_thread      = None          # heartbeat thread handle
        self._hb_stop        = threading.Event()
        self._lock           = threading.Lock()

        if enabled:
            self._connect()
            self._start_heartbeat()

    # ── Connection management ─────────────────────────────────────────────

    def _connect(self) -> bool:
        """Attempt to open the serial port.  Returns True on success."""
        try:
            import serial as _serial
            self._ser = _serial.Serial(self.port, self.baud, timeout=1)
            print(f"[Serial] ✅ Connected to {self.port} @ {self.baud} baud")
            return True
        except Exception as exc:
            print(f"[Serial] ⚠️  Could not open {self.port}: {exc}")
            self._ser = None
            return False

    def _reconnect(self) -> bool:
        """Close existing connection (if any) then retry."""
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        return self._connect()

    # ── Public send API ───────────────────────────────────────────────────

    def send(self, token: str, force: bool = False) -> None:
        """
        Transmit a token to the Arduino.

        Skipped if:
          • token == last token AND force is False (deduplication)
          • less than COMMAND_SEND_INTERVAL seconds since last send
          • enabled is False (dry-run)

        EMERGENCY tokens bypass deduplication but still respect cooldown
        to avoid flooding.

        Parameters
        ----------
        token : Arduino token string (e.g. "ES", "FF", "HB")
        force : if True, bypass deduplication (used for emergency tokens)
        """
        now = time.time()
        with self._lock:
            # Cooldown guard (always applied)
            if now - self._last_send_t < COMMAND_SEND_INTERVAL:
                return

            # Deduplication (skipped for forced/emergency sends)
            if not force and token == self._last_token:
                return

            self._last_token  = token
            self._last_send_t = now

        if not self.enabled:
            return   # dry-run: token has already been printed by nav_decision

        self._write(token)

    def _write(self, token: str) -> None:
        """Internal write — reconnects once on failure."""
        payload = f"{token}\n".encode()
        try:
            if self._ser is None or not self._ser.is_open:
                if not self._reconnect():
                    return
            self._ser.write(payload)
        except Exception as exc:
            print(f"[Serial] Write error ({exc}) — attempting reconnect")
            try:
                if self._reconnect():
                    self._ser.write(payload)
            except Exception as exc2:
                print(f"[Serial] Reconnect write failed: {exc2}")

    # ── FIX 8: Heartbeat ─────────────────────────────────────────────────

    def _start_heartbeat(self) -> None:
        """Start the background heartbeat daemon thread."""
        self._hb_thread = threading.Thread(
            target=self._heartbeat_loop,
            name="SerialHeartbeat",
            daemon=True
        )
        self._hb_thread.start()
        print(f"[Serial] Heartbeat thread started ({HEARTBEAT_INTERVAL}s interval)")

    def _heartbeat_loop(self) -> None:
        """
        Send 'HB\\n' periodically so the Arduino watchdog knows Python is alive.
        The Arduino sketch should implement: if no HB received within 2×interval,
        stop all motors (fail-safe).

        PATCH 2 — Route through self.send(..., force=True) instead of
        self._write() directly.  This ensures:
          • the per-command cooldown (COMMAND_SEND_INTERVAL) is respected, so
            a burst of nav commands + heartbeat cannot overflow the serial
            buffer on slow devices;
          • duplicate-suppression is bypassed (force=True) so the heartbeat
            is always transmitted even if "HB" was the last sent token;
          • _last_send_t is updated, preventing a nav command from
            immediately following the heartbeat within the cooldown window.
        The lock inside send() serialises access with concurrent nav writes,
        eliminating the race condition present when _write() was called directly.
        """
        while not self._hb_stop.is_set():
            time.sleep(HEARTBEAT_INTERVAL)
            if self._hb_stop.is_set():
                break
            try:
                self.send("HB", force=True)   # PATCH 2: use send(), not _write()
            except Exception:
                pass   # already handled inside send() → _write()

    def close(self) -> None:
        """Graceful shutdown — stop heartbeat and close port."""
        self._hb_stop.set()
        if self._hb_thread and self._hb_thread.is_alive():
            self._hb_thread.join(timeout=2.0)
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
                print("[Serial] Port closed")
        except Exception:
            pass


# Module-level singleton.  Set enabled=True and supply the correct port when
# running on hardware.  Dry-run (enabled=False) is the safe default so the
# nav stack operates without a connected Arduino.
arduino = ArduinoSerial(port=SERIAL_PORT_DEFAULT, baud=SERIAL_BAUD_DEFAULT,
                        enabled=False)


def send_nav_token(token: str, force: bool = False) -> None:
    """
    Convenience wrapper called by nav_decision and emergency paths.
    Translates a NAV_* constant to its Arduino token and forwards to
    the ArduinoSerial singleton.
    """
    arduino.send(token, force=force)
# ══════════════════════════════════════════════════════════════════════════════

_NAV_ARROW_DEFS: dict[str, list[Tuple[float, float]]] = {
    NAV_FORWARD      : [(0.50, 0.0), (1.0, 0.6), (0.70, 0.6), (0.70, 1.0),
                        (0.30, 1.0), (0.30, 0.6), (0.0, 0.6)],
    NAV_FAST_FORWARD : [(0.50, 0.0), (1.0, 0.6), (0.70, 0.6), (0.70, 1.0),
                        (0.30, 1.0), (0.30, 0.6), (0.0, 0.6)],  # same shape, diff colour
    NAV_SLOW_FORWARD : [(0.50, 0.0), (1.0, 0.6), (0.70, 0.6), (0.70, 1.0),
                        (0.30, 1.0), (0.30, 0.6), (0.0, 0.6)],
    NAV_BACKWARD     : [(0.50, 1.0), (1.0, 0.4), (0.70, 0.4), (0.70, 0.0),
                        (0.30, 0.0), (0.30, 0.4), (0.0, 0.4)],
    NAV_AVOID_LEFT   : [(0.0, 0.5), (0.6, 0.0), (0.6, 0.30), (1.0, 0.30),
                        (1.0, 0.70), (0.6, 0.70), (0.6, 1.0)],
    NAV_AVOID_RIGHT  : [(1.0, 0.5), (0.4, 0.0), (0.4, 0.30), (0.0, 0.30),
                        (0.0, 0.70), (0.4, 0.70), (0.4, 1.0)],
    NAV_SEARCH_LEFT  : [(0.0, 0.5), (0.6, 0.0), (0.6, 0.30), (1.0, 0.30),
                        (1.0, 0.70), (0.6, 0.70), (0.6, 1.0)],
    NAV_SEARCH_RIGHT : [(1.0, 0.5), (0.4, 0.0), (0.4, 0.30), (0.0, 0.30),
                        (0.0, 0.70), (0.4, 0.70), (0.4, 1.0)],
    NAV_HOVER        : None,
    NAV_SEARCH       : None,
    NAV_SAFE_SEARCH  : None,
    NAV_FORWARD_SCAN : None,
    NAV_STOP         : None,
    NAV_EMERGENCY_STOP: None,
}

_NAV_CMD_COLOR: dict[str, Tuple[int, int, int]] = {
    NAV_FORWARD      : (0,   200, 80),
    NAV_FAST_FORWARD : (0,   255, 50),    # bright green — full speed
    NAV_SLOW_FORWARD : (0,   200, 160),   # teal — caution speed
    NAV_STOP         : (0,   80,  200),   # blue — stopped
    NAV_BACKWARD     : (0,   60,  200),
    NAV_AVOID_LEFT   : (0,   200, 200),
    NAV_AVOID_RIGHT  : (0,   200, 200),
    NAV_HOVER        : (0,   180, 255),
    NAV_SEARCH       : (180, 180, 0),
    NAV_SEARCH_LEFT  : (200, 200, 0),
    NAV_SEARCH_RIGHT : (200, 200, 0),
    NAV_FORWARD_SCAN : (0,   200, 100),
    NAV_SAFE_SEARCH  : (160, 160, 0),
    NAV_EMERGENCY_STOP: (0,   0,  255),   # red — always visible
}

_DANGER_COLORS = {
    "LOW"   : (0,  200, 0),
    "MEDIUM": (0,  165, 255),
    "HIGH"  : (0,  0,   220),
}


def _proximity_to_danger_label(proximity: float) -> str:
    if proximity >= FRONT_DANGER_FRAC:
        return "HIGH"
    if proximity >= SIDE_DANGER_FRAC:
        return "MEDIUM"
    return "LOW"


def draw_zone_grid(frame: np.ndarray, frame_w: int, frame_h: int) -> None:
    """Draw horizontal + vertical zone boundary lines (UPGRADE 7 adds horizontal bands)."""
    left_x  = int(frame_w * LEFT_ZONE_END)
    right_x = int(frame_w * RIGHT_ZONE_START)
    top_y   = int(frame_h * TOP_ZONE_END)          # UPGRADE 7
    bot_y   = int(frame_h * BOTTOM_ZONE_START)     # UPGRADE 7
    alpha   = 0.20

    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0),       (left_x, frame_h),    (255, 200, 100), -1)
    cv2.rectangle(overlay, (right_x, 0), (frame_w, frame_h),   (255, 200, 100), -1)
    # TOP zone tint (slight purple)
    cv2.rectangle(overlay, (0, 0),       (frame_w, top_y),     (200, 100, 200), -1)
    # BOTTOM zone tint (slight orange)
    cv2.rectangle(overlay, (0, bot_y),   (frame_w, frame_h),   (100, 180, 255), -1)
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

    # Vertical zone lines
    cv2.line(frame, (left_x, 0),  (left_x, frame_h),  (200, 200, 200), 1)
    cv2.line(frame, (right_x, 0), (right_x, frame_h), (200, 200, 200), 1)
    # Horizontal zone lines
    cv2.line(frame, (0, top_y),   (frame_w, top_y),   (200, 150, 200), 1)
    cv2.line(frame, (0, bot_y),   (frame_w, bot_y),   (160, 200, 200), 1)

    # Zone labels
    lby = frame_h - 10
    cv2.putText(frame, "L",   (left_x // 2 - 6, lby),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
    cv2.putText(frame, "C",   ((left_x + right_x) // 2 - 6, lby),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
    cv2.putText(frame, "R",   (right_x + (frame_w - right_x) // 2 - 6, lby),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
    cv2.putText(frame, "TOP", (4, top_y - 3),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 150, 200), 1)
    cv2.putText(frame, "BOT", (4, bot_y + 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (160, 200, 200), 1)


def draw_danger_boxes(frame: np.ndarray, obstacles: list[ObstacleInfo]) -> None:
    """Draw obstacle boxes with danger colouring — extended with v_zone label."""
    for obs in obstacles:
        x1, y1, x2, y2 = obs.box
        danger_label    = _proximity_to_danger_label(obs.proximity)
        colour          = _DANGER_COLORS[danger_label]
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)

        info = f"{obs.zone}/{obs.v_zone}  {obs.proximity*100:.0f}%  {danger_label}"
        (tw, th), _ = cv2.getTextSize(info, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        lyt = max(0, y2 + 2)
        cv2.rectangle(frame, (x1, lyt), (x1 + tw + 6, lyt + th + 6), colour, -1)
        cv2.putText(frame, info, (x1 + 3, lyt + th + 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)


def _draw_nav_arrow(frame: np.ndarray, cmd: str,
                    cx: int, cy: int, size: int = 52) -> None:
    """Draw directional arrow / symbol for the current NAV command."""
    colour = _NAV_CMD_COLOR.get(cmd, (255, 255, 255))
    half   = size // 2

    if cmd == NAV_HOVER:
        cv2.circle(frame, (cx, cy), half,      colour, 3)
        cv2.circle(frame, (cx, cy), half // 2, colour, 2)
        cv2.circle(frame, (cx, cy), 4,         colour, -1)
        return

    if cmd in (NAV_SEARCH, NAV_SAFE_SEARCH):
        r = int(half * 0.6)
        cv2.circle(frame, (cx - 4, cy - 4), r, colour, 2)
        cv2.line(frame,
                 (cx - 4 + int(r * 0.7), cy - 4 + int(r * 0.7)),
                 (cx + half - 4, cy + half - 4), colour, 3)
        return

    if cmd == NAV_STOP:
        # Stop: filled square
        cv2.rectangle(frame,
                      (cx - half + 8, cy - half + 8),
                      (cx + half - 8, cy + half - 8),
                      colour, -1)
        cv2.rectangle(frame,
                      (cx - half + 8, cy - half + 8),
                      (cx + half - 8, cy + half - 8),
                      (255, 255, 255), 1)
        return

    if cmd == NAV_EMERGENCY_STOP:
        # Flashing red X
        cv2.line(frame, (cx - half + 6, cy - half + 6),
                 (cx + half - 6, cy + half - 6), (0, 0, 255), 4)
        cv2.line(frame, (cx + half - 6, cy - half + 6),
                 (cx - half + 6, cy + half - 6), (0, 0, 255), 4)
        return

    if cmd == NAV_FORWARD_SCAN:
        # Double-chevron forward
        for offset in [0, 14]:
            pts = np.array([
                (cx, cy - half + 6 + offset),
                (cx + half - 6, cy + offset),
                (cx, cy + 10 + offset),
                (cx - half + 6, cy + offset),
            ], dtype=np.int32)
            cv2.polylines(frame, [pts], isClosed=False, color=colour, thickness=2)
        return

    pts_def = _NAV_ARROW_DEFS.get(cmd)
    if pts_def is None:
        return
    pts = np.array(
        [(int(cx - half + p[0] * size),
          int(cy - half + p[1] * size))
         for p in pts_def],
        dtype=np.int32
    )
    cv2.fillPoly(frame, [pts], colour)
    cv2.polylines(frame, [pts], isClosed=True, color=(255, 255, 255), thickness=1)


def draw_nav_overlay(frame: np.ndarray,
                     nav_cmd: str,
                     obstacles: list[ObstacleInfo],
                     danger_obs: Optional[ObstacleInfo],
                     frame_w: int,
                     frame_h: int) -> None:
    """
    Composite navigation overlay — unchanged contract from v2, extended content:
      1. Zone grid (horizontal + vertical — v3)
      2. Danger-coloured obstacle boxes
      3. Warning banner
      4. Navigation arrow (new shapes for v3 states)
      5. Highlight on triggering obstacle
      6. EMERGENCY banner (bright red full-width — v3)
    """
    draw_zone_grid(frame, frame_w, frame_h)

    visible_obs = [o for o in obstacles if o.proximity >= SIDE_DANGER_FRAC * 0.7]
    draw_danger_boxes(frame, visible_obs)

    # ── UPGRADE 11: Emergency / Recovery banner ───────────────────────────
    if nav_cmd == NAV_EMERGENCY_STOP:
        # Check if we're in the recovery phase for a different banner colour
        in_recovery = (_emergency_phase == _EMERG_RECOVERY)
        if in_recovery:
            banner       = "⚠  RECOVERING FROM EMERGENCY  ⚠"
            banner_color = (0, 140, 200)   # amber-ish blue — distinct from full ES
        else:
            banner       = "⚠⚠  EMERGENCY STOP  ⚠⚠"
            banner_color = (0, 0, 220)
        (bw, bh), _ = cv2.getTextSize(banner, cv2.FONT_HERSHEY_SIMPLEX, 0.80, 2)
        bx = (frame_w - bw) // 2
        by = 36
        cv2.rectangle(frame, (0, 0), (frame_w, by + 10), banner_color, -1)
        cv2.putText(frame, banner, (bx, by),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.80, (255, 255, 255), 2)

    elif danger_obs is not None:
        danger_label = _proximity_to_danger_label(danger_obs.proximity)
        if danger_label == "HIGH":
            banner = f"⚠ OBSTACLE  {nav_cmd}  ({danger_obs.proximity*100:.0f}%)"
            banner_colour = (0, 0, 220)
        elif danger_label == "MEDIUM":
            banner = f"! CLOSE  {nav_cmd}  ({danger_obs.proximity*100:.0f}%)"
            banner_colour = (0, 165, 255)
        else:
            banner, banner_colour = None, None

        if banner:
            (bw, bh), _ = cv2.getTextSize(banner, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
            bx = (frame_w - bw) // 2
            by = 14
            cv2.rectangle(frame, (bx - 8, by - bh - 6),
                          (bx + bw + 8, by + 6), banner_colour, -1)
            cv2.putText(frame, banner, (bx, by),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        hx1, hy1, hx2, hy2 = danger_obs.box
        cv2.rectangle(frame, (hx1 - 3, hy1 - 3), (hx2 + 3, hy2 + 3), (0, 0, 255), 3)

    arrow_cx = frame_w - 48
    arrow_cy = frame_h - 60
    bg_overlay = frame.copy()
    cv2.circle(bg_overlay, (arrow_cx, arrow_cy), 36, (30, 30, 30), -1)
    cv2.addWeighted(bg_overlay, 0.55, frame, 0.45, 0, frame)
    _draw_nav_arrow(frame, nav_cmd, arrow_cx, arrow_cy, size=44)

    nav_label_x = arrow_cx - 38
    nav_label_y = arrow_cy + 46
    cv2.putText(frame, nav_cmd, (nav_label_x, nav_label_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                _NAV_CMD_COLOR.get(nav_cmd, (255, 255, 255)), 1)


# ══════════════════════════════════════════════════════════════════════════════
#  UPGRADE 9 — CPU ADAPTIVE INFERENCE CONTROL
#  Provides lightweight frame-skip and low-power mode toggles so the pipeline
#  can be tuned at runtime without touching the YOLO thread itself.
#  The YOLO thread already skips frames that haven't changed (counter check).
#  This layer adds a configurable minimum interval between YOLO calls.
# ══════════════════════════════════════════════════════════════════════════════

# Minimum wall-clock seconds between YOLO inference calls.
# Default 0 = run as fast as possible (original behaviour).
# Set to e.g. 0.08 for ~12 fps max on a weak CPU to free cycles for display.
ADAPTIVE_MIN_INFERENCE_INTERVAL: float = 0.0   # seconds; 0 = disabled

# Low-power mode: when True, YOLO runs at a reduced inference resolution and
# skips alternate frames.  Toggle via: set_low_power_mode(True)
# PATCH 6 — Comment corrected: the reduced size is 320 px (a fixed constant
# chosen for a good YOLO speed/accuracy trade-off on weak CPUs), NOT simply
# "half the configured imgsz".  For example, if cfg["imgsz"] == 416 the
# reduction is 416→320, not 416→208.  The HUD (FIX 5) already shows the
# ACTUAL runtime inference size to avoid confusion.
_low_power_mode: bool = False
_lp_frame_toggle: bool = False   # alternating skip flag


def set_low_power_mode(enabled: bool) -> None:
    """
    Enable / disable low-power CPU mode.

    When enabled:
      • Alternate frames are skipped in the YOLO thread (halves frame load)
      • Inference resolution is reduced to 320 px (from cfg["imgsz"]),
        roughly halving memory bandwidth and FLOPs on weak CPUs.
        NOTE: the reduction is always to 320 px, not to half of cfg["imgsz"].

    Note: this sets module-level flags read by _yolo_should_skip().
    """
    global _low_power_mode
    _low_power_mode = enabled
    print(f"[CPU] Low-power mode: {'ON' if enabled else 'OFF'}")


def _yolo_should_skip() -> bool:
    """
    Called from the YOLO thread before each inference to decide whether to
    skip this frame for CPU relief.

    Returns True if inference should be skipped for this frame.
    """
    global _lp_frame_toggle
    if not _low_power_mode:
        return False
    _lp_frame_toggle = not _lp_frame_toggle
    return _lp_frame_toggle   # skip every other frame


# ══════════════════════════════════════════════════════════════════════════════
#  TORCH CPU THREAD TUNING  (unchanged)
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
#  HARDWARE DETECTION  (unchanged)
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


# ══════════════════════════════════════════════════════════════════════════════
#  IP HELPER  (unchanged)
# ══════════════════════════════════════════════════════════════════════════════

_PRIVATE_IP_RE = re.compile(
    r'\b('
    r'10\.\d{1,3}\.\d{1,3}\.\d{1,3}'
    r'|172\.(?:1[6-9]|2\d|3[01])\.\d{1,3}\.\d{1,3}'
    r'|192\.168\.\d{1,3}\.\d{1,3}'
    r')\b'
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
                line = ser.readline().decode(errors="ignore").strip()
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
#  ADAPTIVE CONFIG  (unchanged)
# ══════════════════════════════════════════════════════════════════════════════

def build_config(device_type: str, vram_gb: float) -> dict:
    if device_type == "cuda":
        if vram_gb >= 8:
            return dict(model="yolov8x.pt", imgsz=960, conf=0.30, iou=0.45,
                        half=True,  device="cuda", cam_w=1920, cam_h=1080,
                        tier="High-end GPU (CUDA)")
        elif vram_gb >= 4:
            return dict(model="yolov8m.pt", imgsz=640, conf=0.25, iou=0.45,
                        half=True,  device="cuda", cam_w=1280, cam_h=720,
                        tier="Mid-range GPU (CUDA)")
        else:
            return dict(model="yolov8s.pt", imgsz=640, conf=0.25, iou=0.45,
                        half=True,  device="cuda", cam_w=1280, cam_h=720,
                        tier="Low-VRAM GPU (CUDA)")

    return dict(
        model  = "yolov8n.pt",
        imgsz  = 416,
        conf   = 0.30,
        iou    = 0.45,
        half   = False,
        device = "cpu",
        cam_w  = 640,
        cam_h  = 480,
        tier   = f"CPU ({device_name})",
    )


cfg = build_config(device_type, vram_gb)

print(f"\n[Config] Tier    : {cfg['tier']}")
print(f"[Config] Model   : {cfg['model']}")
print(f"[Config] Img size: {cfg['imgsz']}")
print(f"[Config] FP16    : {cfg['half']}")
print(f"[Config] Res     : {cfg['cam_w']}×{cfg['cam_h']}\n")

cfg['display_scale'] = 1.2
print(f"[Config] Display scale: {cfg['display_scale']:.2f}x\n")


# ══════════════════════════════════════════════════════════════════════════════
#  LOAD MODEL  (unchanged)
# ══════════════════════════════════════════════════════════════════════════════

try:
    model = YOLO(cfg["model"])
    model.to(cfg["device"])
    _dummy = np.zeros((cfg["imgsz"], cfg["imgsz"], 3), dtype=np.uint8)
    model.predict(_dummy, verbose=False, imgsz=cfg["imgsz"])
    print(f"[Model] ✅ {cfg['model']} loaded & warmed up on {cfg['device']}")
except Exception as e:
    print(f"[Model Error] {e}")
    raise SystemExit(1)

HUMAN_CLASS = 0


# ══════════════════════════════════════════════════════════════════════════════
#  FRAME READER THREAD  (unchanged)
# ══════════════════════════════════════════════════════════════════════════════

STREAM_PORT = 8080
stream_url = f"http://192.168.1.4:{STREAM_PORT}/video"
_frame_lock    = threading.Lock()
_latest_frame: Optional[np.ndarray] = None
_frame_counter = 0
_reader_alive  = threading.Event()
_reader_alive.set()

# FIX 6 — Stale frame protection.
# Updated to time.time() whenever a valid frame is received.  The display
# loop checks this against a 1-second threshold and fires EMERGENCY_STOP
# if no fresh frame has arrived, guarding against stream freezes that would
# leave the drone executing a stale nav command indefinitely.
_last_frame_timestamp: float = time.time()


def _open_cap(url: str, retries: int = 5, delay: float = 2.0):
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
        "analyzeduration;0|probesize;32|fflags;nobuffer|flags;low_delay"
    )
    for attempt in range(retries):
        cap = cv2.VideoCapture(url, cv2.CAP_ANY)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if cap.isOpened():
            print(f"[Stream] ✅ Connected ({url})")
            return cap
        cap.release()
        if attempt < retries - 1:
            print(f"[Stream] Retry {attempt+1}/{retries} in {delay}s…")
            time.sleep(delay)
    print(f"[Stream] ❌ Could not connect after {retries} attempts.")
    return None


def _frame_reader_loop(url: str):
    global _latest_frame, _frame_counter, _last_frame_timestamp
    cap = _open_cap(url)
    if cap is None:
        # FIX 10: stream never connected — clear alive flag so the display
        # loop exits cleanly and the emergency layer activates.
        _reader_alive.clear()
        return

    while _reader_alive.is_set():
        ret, frame = cap.read()
        if not ret:
            print("[Reader] Stream lost — reconnecting…")
            cap.release()
            cap = _open_cap(url)
            if cap is None:
                # FIX 10: all reconnect attempts exhausted — signal the rest
                # of the pipeline to stop.  The display loop detects the
                # cleared flag and the stale-frame check will issue
                # EMERGENCY_STOP before the loop exits.
                print("[Reader] Reconnect failed. Stopping reader.")
                _reader_alive.clear()
                break
            continue

        with _frame_lock:
            _latest_frame      = frame
            _frame_counter    += 1
            _last_frame_timestamp = time.time()   # FIX 6: stamp arrival time

    cap.release()
    print("[Reader] Thread exiting.")


def _get_latest_frame():
    with _frame_lock:
        return _latest_frame, _frame_counter


# ══════════════════════════════════════════════════════════════════════════════
#  YOLO INFERENCE THREAD  (upgrade 9 — frame skip hook added)
# ══════════════════════════════════════════════════════════════════════════════

_boxes_lock    = threading.Lock()
_latest_boxes: list = []
_ai_fps_val    = 0.0

_track_hits:   dict = defaultdict(int)
_track_misses: dict = defaultdict(int)
SMOOTH_FRAMES = 1

_last_inference_time: float = 0.0   # for ADAPTIVE_MIN_INFERENCE_INTERVAL


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

        # ── UPGRADE 9: adaptive frame skip ────────────────────────────────
        if _yolo_should_skip():
            last_counter = counter
            continue

        now = time.time()
        if ADAPTIVE_MIN_INFERENCE_INTERVAL > 0:
            if now - _last_inference_time < ADAPTIVE_MIN_INFERENCE_INTERVAL:
                time.sleep(0.005)
                continue
        _last_inference_time = now
        # ── end upgrade 9 ─────────────────────────────────────────────────

        last_counter = counter

        # FIX 4 — Dynamic inference size for low-power mode.
        # When low-power mode is active we use imgsz=320 (half the default
        # 640) which roughly halves both memory bandwidth and FLOPs, giving
        # genuine CPU relief beyond the frame-skip alone.  Normal mode uses
        # cfg["imgsz"] as before so accuracy is unaffected in full-power mode.
        infer_imgsz = 320 if _low_power_mode else cfg["imgsz"]

        try:
            # Run tracking WITHOUT a class filter (all COCO objects detected).
            # NAVIGATION_CLASSES filter is applied in analyse_obstacles().
            results = model.track(
                frame,
                persist = True,
                conf    = cfg["conf"],
                iou     = cfg["iou"],
                imgsz   = infer_imgsz,        # FIX 4: dynamic size
                tracker = "bytetrack.yaml",
                verbose = False,
                half    = cfg["half"],
            )

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

            # PATCH 4 — Replace full .clear() with bounded oldest-first eviction.
            # Clearing ALL entries causes an instant perception collapse:
            # every tracked object loses its EMA history and the nav stack
            # can lurch between depth estimates for 2-3 frames.  Evicting
            # the oldest 25 % of entries instead preserves recently-active
            # track state while still capping memory growth.
            if len(_depth_ema) > 200:
                evict_count = max(1, len(_depth_ema) // 4)   # evict ~25 %
                for _tid in list(_depth_ema.keys())[:evict_count]:
                    _depth_ema.pop(_tid, None)
                print(f"[YOLO] _depth_ema evicted {evict_count} oldest entries")
            if len(_track_hits) > 300:
                evict_count = max(1, len(_track_hits) // 4)
                for _tid in list(_track_hits.keys())[:evict_count]:
                    _track_hits.pop(_tid, None)
                    _track_misses.pop(_tid, None)
                print(f"[YOLO] _track_hits/_misses evicted {evict_count} oldest entries")

            with _boxes_lock:
                _latest_boxes = new_boxes

            ai_frames += 1
            elapsed = time.time() - t0
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
#  DRAW HELPERS  (unchanged)
# ══════════════════════════════════════════════════════════════════════════════

def draw_boxes(frame: np.ndarray, boxes: list) -> None:
    for (x1, y1, x2, y2, conf_val, tid, cls_id) in boxes:
        is_human = (cls_id == HUMAN_CLASS)
        colour   = (0, 220, 0) if is_human else (60, 180, 255)

        if is_human:
            label = (f"Human #{tid} ({conf_val:.0%})" if tid >= 0
                     else f"Human ({conf_val:.0%})")
        else:
            cls_name = model.names.get(cls_id, f"cls{cls_id}") if hasattr(model, "names") else f"cls{cls_id}"
            label    = (f"{cls_name} #{tid} ({conf_val:.0%})" if tid >= 0
                        else f"{cls_name} ({conf_val:.0%})")

        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        label_y_top = max(0, y1 - th - 10)
        cv2.rectangle(frame, (x1, label_y_top), (x1 + tw + 6, y1), colour, -1)
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
        text_y = max(th + 4, y1 - 4)
        cv2.putText(frame, label, (x1 + 3, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)


def _scale_frame_for_display(frame: np.ndarray) -> np.ndarray:
    scale = float(cfg.get('display_scale', 1.0))
    if scale == 1.0:
        return frame
    h, w   = frame.shape[:2]
    new_w  = max(1, int(w * scale))
    new_h  = max(1, int(h * scale))
    return cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT  (display loop — unchanged contract; HUD extended)
# ══════════════════════════════════════════════════════════════════════════════

def _render_stale_overlay(frame: np.ndarray) -> None:
    """
    FIX 2: Render a full-width red banner over the last known frame when
    stale-frame EMERGENCY_STOP is active.  Gives the operator immediate
    visual feedback that the video feed has frozen.
    """
    h, w = frame.shape[:2]
    cv2.rectangle(frame, (0, 0), (w, 50), (0, 0, 200), -1)
    msg = "⚠ STALE FRAME — EMERGENCY STOP  (stream frozen)"
    cv2.putText(frame, msg, (10, 34),
                cv2.FONT_HERSHEY_SIMPLEX, 0.70, (255, 255, 255), 2)


def _render_reader_fail_overlay(frame: np.ndarray, elapsed: float) -> None:
    """
    FIX 3: Render a full-width dark-red banner when the reader thread has
    died.  Shows hold-time remaining so the operator can see the fail-safe
    countdown before shutdown.
    """
    h, w = frame.shape[:2]
    remaining = max(0.0, READER_FAIL_HOLD - elapsed)
    cv2.rectangle(frame, (0, 0), (w, 50), (0, 0, 160), -1)
    msg = f"⚠ READER DEAD — EMERGENCY STOP  (shutdown in {remaining:.1f}s)"
    cv2.putText(frame, msg, (10, 34),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)


def main():
    router_ip = "http://192.168.1.1"
    print(f"[Serial] Drone IP: {router_ip}  (test mode)")

    reader_thread = threading.Thread(
        target=_frame_reader_loop, args=(stream_url,),
        name="FrameReader", daemon=True
    )
    yolo_thread = threading.Thread(
        target=_yolo_loop,
        name="YOLOInference", daemon=True
    )

    reader_thread.start()
    print("[Main] Frame reader thread started")

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

    # ── FIX 3: Reader-failure fail-safe ──────────────────────────────────
    # If the reader thread died (stream never connected or all reconnects
    # exhausted), hold EMERGENCY_STOP for READER_FAIL_HOLD seconds before
    # allowing main() to exit.  This ensures the Arduino receives at least
    # one ES token even when stream setup fails immediately.
    _reader_fail_start: float = 0.0

    try:
        while True:
            # ── FIX 3: Check reader alive FIRST in every iteration ────────
            reader_alive = _reader_alive.is_set()

            if not reader_alive:
                if _reader_fail_start == 0.0:
                    _reader_fail_start = time.time()
                    print("[Main] Reader died — holding EMERGENCY_STOP")
                    # PATCH 1: synchronise internal nav FSM state so HUD and
                    # overlays show EMERGENCY_STOP, not a stale command.
                    _force_emergency_nav_state()
                    send_nav_token(ARDUINO_TOKENS[NAV_EMERGENCY_STOP], force=True)

                elapsed_fail = time.time() - _reader_fail_start
                if elapsed_fail < READER_FAIL_HOLD:
                    # PATCH 1: keep re-asserting emergency state each iteration
                    # in case any background path attempted to overwrite it.
                    _force_emergency_nav_state()
                    # Render ES overlay on last known frame if available
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

            frame, _ = _get_latest_frame()

            # ── Stale/missing frame: render ES overlay on last known frame ──
            # master_decision() handles stale_frame=True with EMERGENCY priority.
            # We still need to pump the display so the window stays responsive.
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

            # ── FIX 10: Guard against corrupted / empty detections ────────
            valid_boxes = []
            for b in boxes:
                try:
                    x1, y1, x2, y2, conf_v, tid_v, cls_v = b
                    if x2 > x1 and y2 > y1 and 0.0 <= conf_v <= 1.0:
                        valid_boxes.append(b)
                except (TypeError, ValueError):
                    pass   # skip malformed detection tuples
            boxes = valid_boxes

            human_count = sum(1 for b in boxes if b[6] == HUMAN_CLASS)
            # FIX 1: throttled write
            _write_human_count(human_count)

            draw_boxes(display_frame, boxes)

            h_disp, w_disp = display_frame.shape[:2]

            # ── Sub-system outputs (producers only — no serial, no final logging) ──
            drone_state, drone_cmd, target_ctr = ai_decision(boxes, w_disp, h_disp)
            draw_ai_overlay(display_frame, target_ctr, w_disp, h_disp)

            nav_cmd, obstacles, danger_obs = nav_decision(boxes, w_disp, h_disp)

            # ── MASTER ARBITER — single authority for final command ────────
            master_cmd, master_owner = master_decision(
                nav_cmd      = nav_cmd,
                ai_cmd       = drone_cmd,
                ai_state     = drone_state,
                obstacles    = obstacles,
                stale_frame  = False,   # stale handled above; here frame is fresh
                reader_alive = _reader_alive.is_set(),
            )

            draw_nav_overlay(display_frame, master_cmd, obstacles, danger_obs, w_disp, h_disp)

            # ── HUD ───────────────────────────────────────────────────────
            display_fps_frames += 1
            elapsed = time.time() - display_fps_t0
            if elapsed >= 1.0:
                display_fps_val    = display_fps_frames / elapsed
                display_fps_frames = 0
                display_fps_t0     = time.time()

            top_prox    = f"{obstacles[0].proximity*100:.0f}%" if obstacles else "N/A"
            mem_count   = len(_obstacle_memory)

            # FIX 5: show ACTUAL runtime inference size (320 in low-power mode)
            actual_imgsz = 320 if _low_power_mode else cfg["imgsz"]

            hud = [
                f"Humans  : {human_count}",
                f"Device  : {cfg['tier']}",
                f"Model   : {cfg['model']}  imgsz={actual_imgsz}",  # FIX 5
                f"Disp FPS: {display_fps_val:.1f}",
                f"AI  FPS : {_ai_fps_val:.1f}",
                f"Dr State: {drone_state}",
                f"AI  Cmd : {drone_cmd}",
                f"NAV Cmd : {nav_cmd}",
                f"MASTER  : {master_owner}",
                f"CMD     : {master_cmd}",
                f"Depth   : {top_prox}",
                f"Mem Obs : {mem_count}",
                f"Low Pwr : {'ON' if _low_power_mode else 'OFF'}",
                f"Serial  : {'ON' if arduino.enabled else 'OFF (dry-run)'}",
            ]
            y = 28
            for line in hud:
                cv2.putText(display_frame, line, (10, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(display_frame, line, (10, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (30,  30,  30),  1)
                y += 26

            cv2.imshow("Human Detection", _scale_frame_for_display(display_frame))
            last_display_frame = display_frame

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[Main] User quit")
                break

    except KeyboardInterrupt:
        print("[Main] Ctrl+C — shutting down")
    except Exception as exc:
        # FIX 10: Catch unexpected exceptions so cleanup always runs
        print(f"[Main] Unexpected error: {exc}")
    finally:
        _reader_alive.clear()
        # FIX 10: Send final EMERGENCY_STOP before closing serial so Arduino
        # receives a stop command even on an unclean exit.
        try:
            send_nav_token(ARDUINO_TOKENS[NAV_EMERGENCY_STOP], force=True)
        except Exception:
            pass
        reader_thread.join(timeout=3)
        yolo_thread.join(timeout=3)
        arduino.close()   # FIX 7+8: graceful serial + heartbeat shutdown
        cv2.destroyAllWindows()
        print("[Main] Cleanup complete")


if __name__ == "__main__":
    main()