"""Offline arena-route planning. A displayed route never commands the robot.

Coordinates are millimetres in the local odometry frame. Known wall segments
and obstacle circles are operator-authored; transient range hits are deliberately
not treated as immutable walls. Inflate geometry by robot radius plus margin.
"""

from __future__ import annotations

import heapq
import math


def point_segment_distance(x, y, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    if length2 == 0:
        return math.hypot(x - ax, y - ay)
    t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length2))
    return math.hypot(x - (ax + t * dx), y - (ay + t * dy))


def plan_route(start, goal, walls=(), obstacles=(), *, cell_mm=100,
               robot_radius_mm=150, margin_mm=70, arena_half_size_mm=3000):
    """Return collision-clear grid waypoints, or [] when no route exists.

    The bounds are a *planning workspace*, not a claimed arena wall. The
    caller must calibrate origin/scale before interpreting route coordinates.
    """
    if cell_mm <= 0 or robot_radius_mm < 0 or arena_half_size_mm <= 0:
        raise ValueError("Invalid planner geometry")
    inflation = robot_radius_mm + margin_mm
    extent = int(arena_half_size_mm // cell_mm)
    if extent > 100:
        raise ValueError("Planning grid too large")

    def cell(point):
        return (round(point[0] / cell_mm), round(point[1] / cell_mm))

    def blocked(node):
        ix, iy = node
        if abs(ix) > extent or abs(iy) > extent:
            return True
        x, y = ix * cell_mm, iy * cell_mm
        for ax, ay, bx, by in walls:
            if point_segment_distance(x, y, ax, ay, bx, by) <= inflation:
                return True
        for ox, oy, radius in obstacles:
            if math.hypot(x - ox, y - oy) <= inflation + radius:
                return True
        return False

    source, target = cell(start), cell(goal)
    if blocked(source) or blocked(target):
        return []
    moves = ((1, 0), (-1, 0), (0, 1), (0, -1),
             (1, 1), (1, -1), (-1, 1), (-1, -1))
    queue = [(0.0, source)]
    score = {source: 0.0}
    previous = {}
    while queue:
        _, current = heapq.heappop(queue)
        if current == target:
            nodes = [current]
            while current != source:
                current = previous[current]
                nodes.append(current)
            nodes.reverse()
            return [(x * cell_mm, y * cell_mm) for x, y in nodes]
        cost = score[current]
        for dx, dy in moves:
            neighbour = (current[0] + dx, current[1] + dy)
            if blocked(neighbour):
                continue
            # A diagonal must not cut across an inflated corner.
            if dx and dy and (blocked((current[0] + dx, current[1])) or
                              blocked((current[0], current[1] + dy))):
                continue
            candidate = cost + math.hypot(dx, dy)
            if candidate >= score.get(neighbour, math.inf):
                continue
            score[neighbour] = candidate
            previous[neighbour] = current
            heuristic = math.hypot(target[0] - neighbour[0], target[1] - neighbour[1])
            heapq.heappush(queue, (candidate + heuristic, neighbour))
    return []
