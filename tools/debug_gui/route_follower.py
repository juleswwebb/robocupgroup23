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
FRONT_STOP_MM = 800


def mission_to_local(start, heading_deg, point):
    """Mission x/y (screen y down) -> robot right/forward at marked start."""
    angle = math.radians(heading_deg)
    dx, dy = point[0] - start[0], point[1] - start[1]
    return (-dx * math.sin(angle) + dy * math.cos(angle),
            dx * math.cos(angle) + dy * math.sin(angle))


def prepare_route(layout):
    """Validate a pre-planned route and convert it into local millimetres."""
    if not layout.route or len(layout.route) > MAX_WAYPOINTS:
        raise ValueError("Plan a route with 1–64 waypoints first")
    if layout.blocked(*layout.start):
        raise ValueError("Marked start is inside a blocked area")
    points = []
    previous = (0.0, 0.0)
    previous_mission = layout.start
    total = 0.0
    for waypoint in layout.route:
        x, y = float(waypoint["x"]), float(waypoint["y"])
        if not (math.isfinite(x) and math.isfinite(y)) or layout.blocked(x, y):
            raise ValueError("Route contains an invalid or blocked waypoint")
        if not layout._segment_clear(previous_mission, (x, y)):
            raise ValueError("Route crosses a blocked area")
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
    """One waypoint at a time; stops rather than replanning around surprises."""

    def __init__(self, points, *, speed_percent=86, turn_percent=80):
        if not points:
            raise ValueError("No waypoints")
        self.points = list(points)
        self.index = 0
        # This drivetrain stalls below about 80%. Leave 6 percentage points
        # for heading correction without dropping either moving wheel below 80.
        self.speed = min(94, max(86, int(speed_percent)))
        self.turn = min(100, max(MIN_MOVING_PERCENT, int(turn_percent)))
        self.last_pose = None
        self.last_theta = None
        self.last_progress_at = None
        self.best_distance = math.inf

    def step(self, x, y, theta, front_mm, now):
        if not all(math.isfinite(v) for v in (x, y, theta, now)):
            return DriveDecision(0, 0, "FAULT", "Invalid pose", fault=True)
        # None means this healthy frame has no target within the sensor's
        # usable range. Frame health is checked by the GUI before calling us.
        if front_mm is not None and not math.isfinite(front_mm):
            return DriveDecision(0, 0, "FAULT", "Invalid forward 8×8 range", fault=True)
        if front_mm is not None and front_mm < FRONT_STOP_MM:
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
            self.index += 1
            self.best_distance = math.inf
            self.last_progress_at = now
        if self.index >= len(self.points):
            return DriveDecision(0, 0, "COMPLETE", "Final waypoint reached", done=True)

        target = self.points[self.index]
        start = (0.0, 0.0) if self.index == 0 else self.points[self.index - 1]
        if _segment_distance(pose, start, target) > MAX_CROSSTRACK_MM:
            return DriveDecision(0, 0, "FAULT", "More than 250 mm off route", fault=True)
        distance = math.dist(pose, target)
        if distance < self.best_distance - 15:
            self.best_distance = distance
            self.last_progress_at = now
        elif self.last_progress_at is not None and now - self.last_progress_at > 8:
            return DriveDecision(0, 0, "FAULT", "No route progress for 8 s", fault=True)

        desired = math.atan2(target[1] - y, target[0] - x)
        error_deg = math.degrees(_wrap_radians(desired - theta))
        detail = f"Waypoint {self.index + 1}/{len(self.points)} · {distance:.0f} mm · heading error {error_deg:+.0f}°"
        if abs(error_deg) > 8:
            # Positive mathematical angle = turn left = right wheel forward.
            sign = 1 if error_deg > 0 else -1
            return DriveDecision(-sign * self.turn, sign * self.turn, "TURNING", detail)
        correction = max(-6, min(6, round(-error_deg * 0.5)))
        return DriveDecision(self.speed + correction, self.speed - correction, "FORWARD", detail)
