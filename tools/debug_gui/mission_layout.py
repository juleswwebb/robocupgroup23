"""Pre-laid competition arena and offline weight-collection route planner.

Adapted from Group 7's MissionLayout concept, using Group 23's own model and
keeping all routes desktop-only until robot localisation is calibrated.
"""

from __future__ import annotations

import heapq
import math


class MissionLayout:
    WIDTH_MM = 4900
    HEIGHT_MM = 2400
    HOME_MM = 650
    GRID_MM = 100
    MAX_WAYPOINTS = 64

    def __init__(self):
        self.my_home = "green"
        self.start = (325.0, 325.0)
        self.heading_deg = 0.0
        self.robot_radius_mm = 215.0
        self.margin_mm = 90.0
        self.weights = []  # {x, y, dummy}
        self.obstacles = []  # {kind, x, y, width, height, rotation}
        self.live_obstacles = []  # temporary 8x8 detections, never saved
        self.route = []  # {x, y, target}
        self.error = ""

    def opposite_home(self):
        if self.my_home == "green":
            return (0, self.HEIGHT_MM - self.HOME_MM, self.HOME_MM, self.HEIGHT_MM)
        return (0, 0, self.HOME_MM, self.HOME_MM)

    def obstacle_rect(self, item):
        width, height = item["width"], item["height"]
        if item.get("rotation", 0) % 180 == 90:
            width, height = height, width
        return (item["x"] - width/2, item["y"] - height/2,
                item["x"] + width/2, item["y"] + height/2)

    def blocked(self, x, y):
        clearance = self.robot_radius_mm + self.margin_mm
        if (x < clearance or y < clearance or
                x > self.WIDTH_MM - clearance or
                y > self.HEIGHT_MM - clearance):
            return True
        rect = self.opposite_home()
        if (rect[0] - clearance <= x <= rect[2] + clearance and
                rect[1] - clearance <= y <= rect[3] + clearance):
            return True
        for item in self.obstacles:
            if item["kind"] == "tube":
                if math.hypot(x - item["x"], y - item["y"]) <= item["width"]/2 + clearance:
                    return True
            else:
                x0, y0, x1, y1 = self.obstacle_rect(item)
                if x0 - clearance <= x <= x1 + clearance and y0 - clearance <= y <= y1 + clearance:
                    return True
        for item in self.live_obstacles:
            if math.hypot(x - item["x"], y - item["y"]) <= item["radius"] + clearance:
                return True
        for item in self.weights:
            if item["dummy"] and math.hypot(x - item["x"], y - item["y"]) <= 120 + clearance:
                return True
        return False

    def _nearest_free(self, point, max_offset_cells=7):
        gx, gy = round(point[0]/self.GRID_MM), round(point[1]/self.GRID_MM)
        choices = [(dx*dx + dy*dy, (gx+dx, gy+dy))
                   for dx in range(-max_offset_cells, max_offset_cells+1)
                   for dy in range(-max_offset_cells, max_offset_cells+1)]
        for _, node in sorted(choices):
            if not self.blocked(node[0]*self.GRID_MM, node[1]*self.GRID_MM):
                return node
        return None

    def _segment_clear(self, a, b):
        distance = math.dist(a, b)
        for step in range(max(1, math.ceil(distance/40)) + 1):
            t = step / max(1, math.ceil(distance/40))
            if self.blocked(a[0] + t*(b[0]-a[0]), a[1] + t*(b[1]-a[1])):
                return False
        return True

    def _astar(self, start, goal):
        source = self._nearest_free(start)
        target = self._nearest_free(goal)
        if source is None or target is None:
            return []
        queue = [(0, source)]
        scores = {source: 0.0}
        parents = {}
        closed = set()
        moves = ((1, 0), (-1, 0), (0, 1), (0, -1),
                 (1, 1), (1, -1), (-1, 1), (-1, -1))
        while queue:
            _, node = heapq.heappop(queue)
            if node in closed:
                continue
            if node == target:
                break
            closed.add(node)
            for dx, dy in moves:
                nxt = (node[0]+dx, node[1]+dy)
                x, y = nxt[0]*self.GRID_MM, nxt[1]*self.GRID_MM
                if self.blocked(x, y):
                    continue
                if dx and dy and (self.blocked((node[0]+dx)*self.GRID_MM, node[1]*self.GRID_MM) or
                                  self.blocked(node[0]*self.GRID_MM, (node[1]+dy)*self.GRID_MM)):
                    continue
                score = scores[node] + math.hypot(dx, dy)
                if score >= scores.get(nxt, math.inf):
                    continue
                scores[nxt] = score
                parents[nxt] = node
                heapq.heappush(queue, (score + math.dist(nxt, target), nxt))
        if target not in scores:
            return []
        nodes = [target]
        while nodes[-1] != source:
            nodes.append(parents[nodes[-1]])
        path = [(x*self.GRID_MM, y*self.GRID_MM) for x, y in reversed(nodes)]
        if self._segment_clear(start, path[0]):
            path[0] = start
        if self._segment_clear(path[-1], goal):
            path[-1] = goal
        # Line-of-sight smoothing keeps the route under the firmware's
        # eventual waypoint capacity without crossing inflated geometry.
        smooth = [path[0]]
        i = 0
        while i < len(path)-1:
            j = len(path)-1
            while j > i+1 and not self._segment_clear(path[i], path[j]):
                j -= 1
            smooth.append(path[j])
            i = j
        return smooth

    def _weight_site_route(self, current, item):
        """Return a safe approach plus two short sensor-search stops."""
        target = (float(item["x"]), float(item["y"]))
        path_to_weight = self._astar(current, target)
        if not path_to_weight or math.dist(path_to_weight[-1], target) > 80:
            return None
        if len(path_to_weight) > 1:
            dx = target[0] - path_to_weight[-2][0]
            dy = target[1] - path_to_weight[-2][1]
        else:
            dx, dy = target[0] - current[0], target[1] - current[1]
        length = math.hypot(dx, dy)
        ux, uy = (dx / length, dy / length) if length > 1 else (0.0, 1.0)

        stage = None
        stage_path = None
        for retreat in (300.0, 240.0, 180.0):
            candidate = (target[0] - ux * retreat, target[1] - uy * retreat)
            if self.blocked(*candidate) or not self._segment_clear(candidate, target):
                continue
            candidate_path = self._astar(current, candidate)
            if candidate_path:
                stage, stage_path = candidate, candidate_path
                break
        if stage is None:
            return None

        # Group 7 makes a short lateral sweep around each target. Group 23's
        # top/bottom VL53 pairs are angled inward, so these two poses let both
        # pairs look across the marked site without driving through it.
        lateral = (-uy, ux)
        scan_points = []
        for side in (-1.0, 1.0):
            point = (stage[0] - ux * 80.0 + lateral[0] * side * 130.0,
                     stage[1] - uy * 80.0 + lateral[1] * side * 130.0)
            if (not self.blocked(*point) and self._segment_clear(stage, point)
                    and all(math.dist(point, prior) > 60 for prior in scan_points)):
                scan_points.append(point)
        if not scan_points:
            scan_points = [stage]

        travel = stage_path[1:]
        if not travel or math.dist(travel[-1], stage) > 1:
            travel.append(stage)
        # Tag the staging location and the two scan poses as target-site
        # observations; ordinary A* corners remain untagged.
        stops = [stage] + scan_points
        for point in stops:
            if math.dist(travel[-1], point) > 1:
                travel.append(point)
        return travel, target, stops

    def plan(self, return_home=False):
        self.route = []
        self.error = ""
        real = [item for item in self.weights if not item["dummy"]]
        if not real:
            self.error = "Add at least one real weight."
            return False
        if self.blocked(*self.start):
            self.error = "Start is inside a border, no-go home or obstacle clearance."
            return False
        current = self.start
        remaining = real[:]
        while remaining:
            choices = []
            for item in remaining:
                site = self._weight_site_route(current, item)
                if site:
                    path, target, stops = site
                    cost = sum(math.dist(a, b) for a, b in zip([current] + path, path))
                    choices.append((cost, item, path, target, stops))
            if not choices:
                self.error = "A real weight is unreachable with the current clearance/map."
                self.route = []
                return False
            _, item, path, target, stops = min(choices, key=lambda choice: choice[0])
            stop_points = {(round(x, 2), round(y, 2)) for x, y in stops}
            for x, y in path:
                site_stop = (round(x, 2), round(y, 2)) in stop_points
                self.route.append({"x": x, "y": y, "target": site_stop,
                                   "site_search": site_stop,
                                   "target_x": target[0], "target_y": target[1]})
            current = path[-1]
            remaining.remove(item)
        if return_home:
            home_y = self.HOME_MM/2 if self.my_home == "green" else self.HEIGHT_MM-self.HOME_MM/2
            path = self._astar(current, (self.HOME_MM/2, home_y))
            if not path:
                self.error = "Weights reachable, but return-home route is blocked."
                self.route = []
                return False
            self.route.extend({"x": x, "y": y, "target": False} for x, y in path[1:])
        if len(self.route) > self.MAX_WAYPOINTS:
            self.error = "Route exceeds 64 waypoints; simplify the layout."
            self.route = []
            return False
        return True

    def replan_from(self, current, remaining_targets):
        """Plan around live obstacles without moving the original map origin."""
        if self.blocked(*current):
            raise ValueError("Robot pose lies inside obstacle clearance")
        route = []
        source = current
        for target in remaining_targets:
            goal = (float(target["x"]), float(target["y"]))
            if self.blocked(*goal):
                raise ValueError("Remaining target is blocked by an obstacle")
            segment = self._astar(source, goal)
            if not segment or not self._segment_clear(source, segment[0]):
                raise ValueError("No clear detour to remaining target")
            route.extend({"x": x, "y": y, "target": False} for x, y in segment[1:-1])
            if not route or (route[-1]["x"], route[-1]["y"]) != segment[-1]:
                endpoint = {"x": segment[-1][0], "y": segment[-1][1],
                            "target": bool(target.get("target"))}
                for key in ("site_search", "target_x", "target_y"):
                    if key in target:
                        endpoint[key] = target[key]
                route.append(endpoint)
            else:
                route[-1]["target"] = bool(target.get("target"))
                for key in ("site_search", "target_x", "target_y"):
                    if key in target:
                        route[-1][key] = target[key]
            source = segment[-1]
        if not route or len(route) > self.MAX_WAYPOINTS:
            raise ValueError("Detour is empty or exceeds 64 waypoints")
        return route

    def add_live_obstacles(self, points, *, radius=120):
        """Merge nearby 8x8 hits; keep bounded, transient map evidence."""
        added = 0
        for x, y in points:
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            if any(math.hypot(x - item["x"], y - item["y"]) < 180
                   for item in self.live_obstacles):
                continue
            self.live_obstacles.append({"x": x, "y": y, "radius": radius})
            added += 1
        self.live_obstacles = self.live_obstacles[-40:]
        return added

    def route_is_clear_from(self, current, route):
        previous = current
        for waypoint in route:
            target = (waypoint["x"], waypoint["y"])
            if not self._segment_clear(previous, target):
                return False
            previous = target
        return True

    def to_dict(self):
        return {"home": self.my_home, "start": self.start,
                "heading_deg": self.heading_deg, "weights": self.weights,
                "obstacles": self.obstacles,
                "robot_radius_mm": self.robot_radius_mm, "margin_mm": self.margin_mm}

    def load_dict(self, data):
        if not isinstance(data, dict):
            return
        self.my_home = data.get("home") if data.get("home") in ("green", "blue") else "green"
        start = data.get("start", self.start)
        if len(start) == 2:
            self.start = (float(start[0]), float(start[1]))
        self.heading_deg = float(data.get("heading_deg", 0)) % 360
        self.weights = [item for item in data.get("weights", []) if isinstance(item, dict)][:100]
        self.obstacles = [item for item in data.get("obstacles", []) if isinstance(item, dict)][:100]
        self.robot_radius_mm = float(data.get("robot_radius_mm", 215))
        self.margin_mm = float(data.get("margin_mm", 90))
        self.route = []
