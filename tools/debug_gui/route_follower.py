"""Supervised desktop waypoint follower for the Mission Planner.

Coordinates are millimetres relative to the operator-confirmed start pose:
right is +x, forward is +y. Robot heading is the ArenaModel mathematical
angle, initially pi/2. This module never talks to hardware directly.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


MAX_WAYPOINTS = 64
WAYPOINT_TOLERANCE_MM = 90.0
MAX_CROSSTRACK_MM = 250.0
MAX_ROUTE_MM = 14000.0
MAX_SEGMENT_MM = 2500.0
MIN_MOVING_PERCENT = 80
# Operator-requested close-obstacle threshold. Detections beyond this range
# are not mapped as generic navigation obstacles; this is intentionally very
# close and is not a safe substitute for a physical emergency stop.
EMERGENCY_STOP_MM = 20
MAP_REQUIRED_MM = 800
MAX_POINT_TOF_MAP_MM = 20
MAX_MATRIX_OBSTACLE_MM = 20
TURN_ENTER_DEG = 24.0
TURN_EXIT_DEG = 12.0


class PointTofStopGuard:
    """Debounce a close point-ToF return without letting the robot roll on.

    The first fresh close sample requests a temporary hold. A hard route stop
    is confirmed only when that sensor reports close on a second distinct
    frame. A hold clears only after that same sensor reports two valid clear
    frames; another sensor or an invalid/missing value cannot clear it.
    """

    def __init__(self, required_frames=2):
        self.required_frames = max(1, int(required_frames))
        self.sensor_name = None
        self.last_frame = None
        self.close_frames = 0
        self.clear_frames = 0

    def reset(self):
        self.sensor_name = None
        self.last_frame = None
        self.close_frames = 0
        self.clear_frames = 0

    def observe(self, readings, frame_id, *, threshold_mm=EMERGENCY_STOP_MM):
        if threshold_mm <= 0:
            self.sensor_name = None
            self.close_frames = 0
            self.clear_frames = 0
            self.last_frame = frame_id
            return "clear"
        if frame_id is None or frame_id == self.last_frame:
            return self._state()

        self.last_frame = frame_id
        if isinstance(readings, tuple) and len(readings) == 2:
            readings = {str(readings[0]): readings[1]}
        readings = readings if isinstance(readings, dict) else {}

        if self.sensor_name is not None:
            reading = readings.get(self.sensor_name)
            if not isinstance(reading, (int, float)) or not math.isfinite(reading):
                return "pending"
            if reading <= threshold_mm:
                self.close_frames += 1
                self.clear_frames = 0
                return self._state()
            self.clear_frames += 1
            if self.clear_frames < 2:
                return "pending"
            self.sensor_name = None
            self.close_frames = 0
            self.clear_frames = 0
            return "clear"

        close_hit = None
        if threshold_mm > 0:
            close_hit = next((
                (str(name), value) for name, value in readings.items()
                if isinstance(value, (int, float)) and math.isfinite(value)
                and value <= threshold_mm
            ), None)
        if close_hit is None:
            return "clear"

        close_name, _reading = close_hit
        if close_name != self.sensor_name:
            self.sensor_name = close_name
            self.close_frames = 1
            self.clear_frames = 0
        else:
            self.close_frames += 1
        return self._state()

    def _state(self):
        if self.close_frames >= self.required_frames:
            return "confirmed"
        return "pending" if self.sensor_name is not None else "clear"


def mission_to_local(start, heading_deg, point):
    """Mission x/y (screen y down) -> robot right/forward at marked start."""
    angle = math.radians(heading_deg)
    dx, dy = point[0] - start[0], point[1] - start[1]
    return (-dx * math.sin(angle) + dy * math.cos(angle),
            dx * math.cos(angle) + dy * math.sin(angle))


def local_to_mission(start, heading_deg, point):
    """Robot right/forward -> mission-map x/y."""
    angle = math.radians(heading_deg)
    right, forward = point
    return (start[0] + forward * math.cos(angle) - right * math.sin(angle),
            start[1] + forward * math.sin(angle) + right * math.cos(angle))


def matrix_obstacle_points(frame, arena_model, layout):
    """Project the upper central 8x8 field into mission coordinates (mm).

    Row 4 and below are excluded: this robot's mounting produces persistent
    short floor/chassis returns there, which are not forward obstacles.
    """
    spec = next((item for item in arena_model.sensor_specs
                 if item.get("kind") == "matrix" and item.get("enabled", True)), None)
    if spec is None:
        return []
    side = -1 if arena_model.matrix_mirrored else 1
    points = []
    for col in range(2, 6):
        # Row 4 commonly sees the floor/chassis on this robot; do not let it
        # create a mapped obstacle alongside a distant upper-field return.
        values = [frame.get(f"tof.array.r{row}c{col}") for row in range(4)]
        ranges = sorted(float(v) for v in values
                        if isinstance(v, (int, float)) and not isinstance(v, bool)
                        and math.isfinite(v) and 0 < v <= MAX_MATRIX_OBSTACLE_MM)
        # Even one valid zone can be a small obstacle. The GUI requires it to
        # recur near the same world point on the next frame before mapping it.
        if not ranges:
            continue
        cluster = max((tuple(value for value in ranges if abs(value - centre) <= 150)
                       for centre in ranges), key=len)
        if not cluster:
            continue
        distance = cluster[len(cluster) // 2]
        column_angle = float(spec.get("angle", 0)) + side * (3.5 - col) * arena_model.matrix_fov_deg / 8
        ox, oy, ray_angle = arena_model._origin_and_angle(spec, column_angle)
        local_point = (ox + distance * math.cos(ray_angle),
                       oy + distance * math.sin(ray_angle))
        mission_point = local_to_mission(layout.start, layout.heading_deg, local_point)
        if (0 <= mission_point[0] <= layout.WIDTH_MM
                and 0 <= mission_point[1] <= layout.HEIGHT_MM):
            points.append(mission_point)
    return points


def point_tof_obstacle_points(frame, arena_model, layout, *, excluded_keys=()):
    """Project enabled VL53 point sensors into the mission map.

    The user-positioned sensor origins and angles are authoritative. Invalid
    and saturated VL53 values (e.g. 8191/65535) are ignored. Side ultrasonics
    are intentionally excluded here: they are wall-clearance sensors, not
    forward obstacle points.
    """
    points = []
    excluded_keys = set(excluded_keys)
    for spec in getattr(arena_model, "sensor_specs", []):
        if (spec.get("kind") != "point" or not spec.get("enabled", True)
                or not str(spec.get("key", "")).startswith("xshut")
                or spec.get("key") in excluded_keys):
            continue
        distance = frame.get(spec.get("signal", ""))
        if (isinstance(distance, bool) or not isinstance(distance, (int, float))
                or not math.isfinite(distance)
                or not 0 < distance <= MAX_POINT_TOF_MAP_MM):
            continue
        ox, oy, angle = arena_model._origin_and_angle(spec)
        local_point = (ox + distance * math.cos(angle),
                       oy + distance * math.sin(angle))
        mission_point = local_to_mission(layout.start, layout.heading_deg, local_point)
        if (0 <= mission_point[0] <= layout.WIDTH_MM
                and 0 <= mission_point[1] <= layout.HEIGHT_MM):
            points.append(mission_point)
    return points


def confirm_obstacle_points(points, previous_points, older_points=(), *, tolerance_mm=220):
    """Return hits spatially consistent across three distinct sensor frames."""
    confirmed = []
    for point in points:
        appeared_recently = any(
            math.dist(point, old) <= tolerance_mm for old in previous_points
        )
        appeared_before_that = any(
            math.dist(point, old) <= tolerance_mm for old in older_points
        )
        if appeared_recently and appeared_before_that:
            confirmed.append(point)
    return confirmed


def prepare_route(layout, *, current_local=(0.0, 0.0), current_mission=None,
                  allow_buffered_start=False):
    """Validate a pre-planned route and convert it into local millimetres."""
    if not layout.route or len(layout.route) > MAX_WAYPOINTS:
        raise ValueError("Plan a route with 1–64 waypoints first")
    checked_start = layout.start if current_mission is None else current_mission
    start_reason = layout.blocked_reason(*checked_start)
    escaping_buffer = bool(
        allow_buffered_start and start_reason and
        layout._can_escape_buffered_start(*checked_start, start_reason)
    )
    if start_reason and not escaping_buffer:
        raise ValueError("Current route start is inside a blocked area")
    points = []
    previous = current_local
    previous_mission = layout.start if current_mission is None else current_mission
    total = 0.0
    for waypoint in layout.route:
        x, y = float(waypoint["x"]), float(waypoint["y"])
        if not (math.isfinite(x) and math.isfinite(y)) or layout.blocked(x, y):
            raise ValueError("Route contains an invalid or blocked waypoint")
        if not layout._segment_clear(previous_mission, (x, y)):
            escape_reason = layout.blocked_reason(*previous_mission)
            is_escape = (escaping_buffer and waypoint.get("escape") and
                         previous_mission == checked_start and escape_reason and
                         layout._escape_segment_clear(previous_mission, (x, y), escape_reason))
            if not is_escape:
                raise ValueError("Route crosses a blocked area")
        escaping_buffer = False
        point = mission_to_local(layout.start, layout.heading_deg, (x, y))
        segment = math.dist(previous, point)
        if segment > MAX_SEGMENT_MM:
            raise ValueError("Route contains a segment too long to follow safely")
        total += segment
        if total > MAX_ROUTE_MM:
            raise ValueError("Route is too long for a single test")
        if segment > 1:
            points.append(point)
        previous = point
        previous_mission = (x, y)
    if not points:
        raise ValueError("Route has no movement after the start")
    return points


def _wrap_radians(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _segment_distance(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0 else max(0.0, min(1.0,
        ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length2))
    return math.hypot(point[0] - a[0] - t * dx, point[1] - a[1] - t * dy)


@dataclass(frozen=True)
class DriveDecision:
    left: int
    right: int
    state: str
    detail: str
    done: bool = False
    fault: bool = False


class RouteFollower:
    """One waypoint at a time; the GUI owns obstacle mapping and replanning."""

    def __init__(self, points, *, start_pose=(0.0, 0.0), turn_percent=80):
        if not points:
            raise ValueError("No waypoints")
        self.points = list(points)
        self.index = 0
        # Operator's measured straight-running trim. Steering nudges stay
        # within the 80..100% range needed to keep both wheels moving.
        self.left_speed = 85
        self.right_speed = 100
        self.turn = min(100, max(MIN_MOVING_PERCENT, int(turn_percent)))
        self.start_pose = start_pose
        self.last_pose = None
        self.last_theta = None
        self.last_progress_at = None
        self.best_distance = math.inf
        self.target_align_since = None
        self.target_aligned = False
        self.site_scan_started_at = None
        self.turning = False

    def step(self, x, y, theta, front_mm, now, *, weight=None, target_leg=False,
             search_waypoint=False):
        if not all(math.isfinite(v) for v in (x, y, theta, now)):
            return DriveDecision(0, 0, "FAULT", "Invalid pose", fault=True)
        # None means this healthy frame has no target within the sensor's
        # usable range. Frame health is checked by the GUI before calling us.
        if front_mm is not None and not math.isfinite(front_mm):
            return DriveDecision(0, 0, "FAULT", "Invalid forward 8×8 range", fault=True)
        # A 0 mm threshold disables this guard; positive thresholds stop at or
        # below the configured distance. Confirmed map obstacles remain active.
        if (EMERGENCY_STOP_MM > 0 and front_mm is not None
                and front_mm <= EMERGENCY_STOP_MM):
            return DriveDecision(0, 0, "FAULT", f"Obstacle at {front_mm:.0f} mm", fault=True)
        pose = (x, y)
        if self.last_pose is not None and math.dist(pose, self.last_pose) > 350:
            return DriveDecision(0, 0, "FAULT", "Odometry jumped", fault=True)
        if self.last_theta is not None and abs(_wrap_radians(theta - self.last_theta)) > math.radians(35):
            return DriveDecision(0, 0, "FAULT", "IMU heading jumped", fault=True)
        self.last_pose = pose
        self.last_theta = theta

        while self.index < len(self.points):
            target = self.points[self.index]
            distance = math.dist(pose, target)
            if distance > WAYPOINT_TOLERANCE_MM:
                break
            if target_leg:
                if weight is not None and weight.centered:
                    return DriveDecision(0, 0, "TARGET_ALIGNED",
                                         "Weight aligned by both lower ToFs; pickup unverified", done=True)
                if search_waypoint and not (weight and weight.confirmed):
                    if self.site_scan_started_at is None:
                        self.site_scan_started_at = now
                    if now - self.site_scan_started_at < 0.8:
                        return DriveDecision(0, 0, "SITE_SEARCH",
                                             "Holding this view for upper/lower ToF evidence")
                    self.index += 1
                    self.site_scan_started_at = None
                    self.best_distance = math.inf
                    self.last_progress_at = now
                    self.target_align_since = None
                    self.target_aligned = False
                    return DriveDecision(0, 0, "SITE_SEARCH",
                                         "Search stop reached; checking the next sensor view")
                if weight and weight.confirmed:
                    # The route marker is a safe staging/search pose. Once a
                    # bottom/top pair confirms the mapped object, close the
                    # remaining gap using the measured sensor geometry.
                    target = weight.local_target
                    distance = math.dist(pose, target)
                    if distance > WAYPOINT_TOLERANCE_MM:
                        break
                return DriveDecision(0, 0, "FAULT",
                                     "Mapped weight reached without sensor-confirmed centering; pickup unverified",
                                     fault=True)
            self.index += 1
            self.best_distance = math.inf
            self.last_progress_at = now
            self.target_align_since = None
            self.target_aligned = False
            self.turning = False
        if self.index >= len(self.points):
            return DriveDecision(0, 0, "COMPLETE", "Final waypoint reached", done=True)

        target = self.points[self.index]
        if target_leg and weight is not None and weight.confirmed:
            target = weight.local_target
        # Once the configured side-ToF geometry confirms the marked weight,
        # the short final approach is a fresh ray to that mapped target from
        # the robot's current pose, not a continuation of the lateral scan
        # leg. The target is still range-gated and the front hard-stop remains.
        start = (pose if target_leg and weight is not None and weight.confirmed
                 else self.start_pose if self.index == 0 else self.points[self.index - 1])
        if _segment_distance(pose, start, target) > MAX_CROSSTRACK_MM:
            return DriveDecision(0, 0, "FAULT", "More than 250 mm off route", fault=True)
        distance = math.dist(pose, target)
        target_distance = (math.dist(pose, weight.local_target)
                           if weight and weight.confirmed else math.inf)
        if weight and weight.confirmed and target_leg and target_distance <= 350:
            left, right = weight.left_mm, weight.right_mm
            if left is not None and right is not None and left <= 200 and right <= 200:
                if abs(left - right) > 30:
                    if getattr(self, "centering_started_at", None) is None:
                        self.centering_started_at = now
                    if now - self.centering_started_at > 2.5:
                        return DriveDecision(0, 0, "FAULT", "Weight centering timed out", fault=True)
                    self.last_progress_at = now
                    sign = 1 if left > right else -1
                    # Short turns limit overshoot with Group 23's high-minimum
                    # drive output; 100 ms app ticks keep the firmware watchdog alive.
                    if (now - self.centering_started_at) % 0.3 < 0.1:
                        return DriveDecision(-sign * self.turn, sign * self.turn,
                                             "CENTERING", f"Left/right ToF gap {left-right:+.0f} mm")
                    return DriveDecision(0, 0, "CENTERING", "Settling between turn pulses")
                self.centering_started_at = None
                return DriveDecision(0, 0, "TARGET_ALIGNED",
                                     "Weight aligned by both lower ToFs; pickup unverified", done=True)
            else:
                self.centering_started_at = None
        else:
            self.centering_started_at = None
        if distance < self.best_distance - 15:
            self.best_distance = distance
            self.last_progress_at = now
        elif self.last_progress_at is not None and now - self.last_progress_at > 8:
            return DriveDecision(0, 0, "FAULT", "No route progress for 8 s", fault=True)

        desired = math.atan2(target[1] - y, target[0] - x)
        error_deg = math.degrees(_wrap_radians(desired - theta))
        detail = f"Waypoint {self.index + 1}/{len(self.points)} · {distance:.0f} mm · heading error {error_deg:+.0f}°"
        if target_leg and distance <= 700:
            # The sensor geometry needs a sensible heading, not exact line
            # lock; a wider deadband avoids high-speed pivot chatter.
            tolerance = 14 if self.target_aligned else 10
            if abs(error_deg) > tolerance:
                self.target_aligned = False
                self.target_align_since = None
                self.last_progress_at = now
                sign = 1 if error_deg > 0 else -1
                # Full turns for large errors; pulse high-enough motor output
                # for small errors so the drivetrain does not stall or coast.
                if abs(error_deg) > 35 or now % 0.32 < 0.08:
                    return DriveDecision(-sign * self.turn, sign * self.turn,
                                         "TARGET_ALIGN", detail)
                return DriveDecision(0, 0, "TARGET_ALIGN", "Settling between heading pulses")
            if not self.target_aligned:
                if self.target_align_since is None:
                    self.target_align_since = now
                self.last_progress_at = now
                if now - self.target_align_since < 0.25:
                    return DriveDecision(0, 0, "TARGET_SETTLE", detail)
                self.target_aligned = True
        if self.turning:
            self.turning = abs(error_deg) > TURN_EXIT_DEG
        elif abs(error_deg) > TURN_ENTER_DEG:
            self.turning = True
        if self.turning:
            # Hysteresis avoids rapid switching between a full pivot and
            # forward trim as heading noise crosses one threshold.
            sign = 1 if error_deg > 0 else -1
            return DriveDecision(-sign * self.turn, sign * self.turn, "TURNING", detail)
        # Positive mathematical angle = turn left = right wheel forward.
        correction = max(-5, min(5, round(-error_deg * 0.5)))
        return DriveDecision(
            max(80, min(100, self.left_speed + correction)),
            max(80, min(100, self.right_speed - correction)),
            "FORWARD", detail,
        )
