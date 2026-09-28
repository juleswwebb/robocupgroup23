"""Target evidence for Group 23's two upper/lower point-ToF pairs.

Adapted from Group 7's height-gap, consecutive-frame and mapped-target gates.
No motor commands are produced here. A single unpaired return is never a
confirmed weight, and a confirmed return must agree with the active map goal.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from route_follower import local_to_mission, mission_to_local


PAIRS = (("xshut6", "xshut5"), ("xshut3", "xshut4"))  # (top, bottom): left, right
MIN_HEIGHT_GAP_MM = 150
MAX_WEIGHT_RANGE_MM = 1200
TARGET_MATCH_MM = 170
CONFIRM_FRAMES = 3


def _range(frame, key):
    value = frame.get(f"tof.{key}")
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    if not math.isfinite(value) or not 30 <= value < 4000:
        return None
    return float(value)


@dataclass(frozen=True)
class WeightEvidence:
    target: tuple[float, float]
    local_target: tuple[float, float]
    hit: tuple[float, float] | None
    left_mm: float | None
    right_mm: float | None
    confirmed: bool
    pending: bool
    centered: bool
    front_is_target: bool


class WeightTracker:
    def __init__(self):
        self.votes = [0, 0]
        self.target = None
        self.last_frame_at = None
        self.pending_since = None

    def observe(self, frame, arena_model, layout, target, front_mm, now):
        """Consume one complete telemetry frame; never count a frame twice."""
        same_target = target == self.target
        if target != self.target or (self.last_frame_at is not None and now - self.last_frame_at > 0.8):
            self.votes = [0, 0]
            self.pending_since = None
        self.target = target
        if same_target and self.last_frame_at == now:
            return self._last
        self.last_frame_at = now
        local_target = mission_to_local(layout.start, layout.heading_deg, target)
        target_distance = math.dist((arena_model.x, arena_model.y), local_target)
        hits = []
        near_ranges = [None, None]
        for index, (top_key, bottom_key) in enumerate(PAIRS):
            top_spec = next((s for s in arena_model.sensor_specs if s.get("key") == top_key
                             and s.get("enabled", True)), None)
            bottom_spec = next((s for s in arena_model.sensor_specs if s.get("key") == bottom_key
                                and s.get("enabled", True)), None)
            top = _range(frame, top_key)
            bottom = _range(frame, bottom_key)
            if (top_spec is None or bottom_spec is None or top is None or bottom is None
                    or bottom > MAX_WEIGHT_RANGE_MM or top - bottom < MIN_HEIGHT_GAP_MM):
                self.votes[index] = 0
                continue
            ox, oy, angle = arena_model._origin_and_angle(bottom_spec)
            local_hit = (ox + bottom * math.cos(angle), oy + bottom * math.sin(angle))
            if math.dist(local_hit, local_target) > TARGET_MATCH_MM:
                self.votes[index] = 0
                continue
            self.votes[index] = min(CONFIRM_FRAMES, self.votes[index] + 1)
            if self.votes[index] >= CONFIRM_FRAMES:
                near_ranges[index] = bottom
                hits.append(local_hit)
        confirmed = bool(hits) and target_distance <= 700
        if any(self.votes) and not confirmed and target_distance <= 700:
            if self.pending_since is None:
                self.pending_since = now
        elif confirmed:
            self.pending_since = None
        pending = (any(self.votes) and self.pending_since is not None and
                   now - self.pending_since <= 1.0)
        local_hit = (sum(p[0] for p in hits) / len(hits),
                     sum(p[1] for p in hits) / len(hits)) if hits else None
        hit = local_to_mission(layout.start, layout.heading_deg, local_hit) if local_hit else None
        centered = (confirmed and all(value is not None and value <= 200 for value in near_ranges)
                    and abs(near_ranges[0] - near_ranges[1]) <= 30)
        matrix = next((s for s in arena_model.sensor_specs if s.get("kind") == "matrix"
                       and s.get("enabled", True)), None)
        expected_front = max(0, target_distance - float(matrix.get("y", 0))) if matrix else None
        heading_error = abs((math.atan2(local_target[1] - arena_model.y,
                                        local_target[0] - arena_model.x) - arena_model.theta
                             + math.pi) % (2 * math.pi) - math.pi)
        front_is_target = (confirmed and matrix is not None and front_mm is not None
                           and 150 <= front_mm < 800 and target_distance <= 700
                           and heading_error <= math.radians(22)
                           and abs(front_mm - expected_front) <= TARGET_MATCH_MM)
        self._last = WeightEvidence(target, local_target, hit, near_ranges[0], near_ranges[1],
                                    confirmed, pending, centered, front_is_target)
        return self._last
