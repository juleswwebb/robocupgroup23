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
                target = (item["x"], item["y"])
                # A target can be approached only if a free centre point is
                # within 80 mm; otherwise we'd falsely report it collected.
                if self.blocked(*target):
                    near = self._nearest_free(target)
                    if near is None:
                        continue
                    target = (near[0]*self.GRID_MM, near[1]*self.GRID_MM)
                    if math.dist(target, (item["x"], item["y"])) > 80:
                        continue
                path = self._astar(current, target)
                if path:
                    choices.append((sum(math.dist(a, b) for a, b in zip(path, path[1:])), item, path))
            if not choices:
                self.error = "A real weight is unreachable with the current clearance/map."
                self.route = []
                return False
            _, item, path = min(choices, key=lambda choice: choice[0])
            self.route.extend({"x": x, "y": y, "target": False} for x, y in path[1:])
            if not self.route or (self.route[-1]["x"], self.route[-1]["y"]) != path[-1]:
                self.route.append({"x": path[-1][0], "y": path[-1][1], "target": True})
            else:
                self.route[-1]["target"] = True
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
