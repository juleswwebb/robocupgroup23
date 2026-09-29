#include "MissionNavigation.h"

#include "DistanceSensors.h"
#include "DriveControl.h"
#include "Encoders.h"
#include "IMU.h"
#include "Navigation.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

namespace {
constexpr float ARENA_X = 4900.0f, ARENA_Y = 2400.0f;
constexpr float LEFT_MM_PER_COUNT = 3635.0f / 41153.0f;
constexpr float RIGHT_MM_PER_COUNT = 3635.0f / 42224.0f;
constexpr uint32_t MAX_RUN_MS = 120000;
constexpr uint16_t FRONT_STOP_MM = 400, HARD_STOP_MM = 160;
constexpr uint16_t SIDE_CLEAR_MM = 650;

struct Waypoint { int16_t x, y; uint8_t flags; bool loaded; };
struct Obstacle { float x, y, width, depth, angle; bool circle; };
Waypoint waypoints[MISSION_MAX_WAYPOINTS] = {};
Obstacle obstacles[MISSION_MAX_OBSTACLES] = {};
uint8_t waypointCount = 0, obstacleCount = 0, waypointIndex = 0;
uint8_t detourCount = 0, detourPhase = 0, obstacleChecks = 0;
bool ready = false, active = false, paused = false;
float startX = 0, startY = 0, startHeading = 0;
float robotRadius = 215, margin = 90;
float poseX = 0, poseY = 0, poseHeading = 0, initialImu = 0;
float detourX = 0, detourY = 0, detourForwardX = 0, detourForwardY = 0;
long previousLeft = 0, previousRight = 0;
uint32_t startedMs = 0, lastPoseMs = 0, lastProgressMs = 0;
uint32_t blockedSinceMs = 0;
float bestDistance = 1e9f;
uint16_t frontMm = 0, leftMm = 0, rightMm = 0;
char state[24] = "IDLE";
char reason[96] = "No mission uploaded";

void set_state(const char* next) {
    strncpy(state, next, sizeof(state) - 1);
    state[sizeof(state) - 1] = '\0';
}
void set_reason(const char* next) {
    strncpy(reason, next, sizeof(reason) - 1);
    reason[sizeof(reason) - 1] = '\0';
}
float wrap(float degrees) {
    while (degrees >= 360.0f) degrees -= 360.0f;
    while (degrees < 0.0f) degrees += 360.0f;
    return degrees;
}
float delta(float from, float to) {
    float error = to - from;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}
bool finite(float value) { return isfinite(value); }
float distance_to(float x, float y) { return hypotf(x - poseX, y - poseY); }

bool point_clear(float x, float y, float extra = 0.0f) {
    const float clearance = robotRadius + margin + extra;
    if (x < clearance || x > ARENA_X - clearance ||
        y < clearance || y > ARENA_Y - clearance) return false;
    for (uint8_t i = 0; i < obstacleCount; ++i) {
        const Obstacle& o = obstacles[i];
        const float dx = x - o.x, dy = y - o.y;
        if (o.circle) {
            if (hypotf(dx, dy) <= o.width * 0.5f + clearance) return false;
        } else {
            const float angle = o.angle * DEG_TO_RAD;
            const float localX = dx * cosf(angle) + dy * sinf(angle);
            const float localY = -dx * sinf(angle) + dy * cosf(angle);
            const float outsideX = max(0.0f, fabsf(localX) - o.width * 0.5f);
            const float outsideY = max(0.0f, fabsf(localY) - o.depth * 0.5f);
            if (hypotf(outsideX, outsideY) <= clearance) return false;
        }
    }
    return true;
}
bool segment_clear(float x0, float y0, float x1, float y1) {
    const int samples = max(1, static_cast<int>(ceilf(hypotf(x1-x0, y1-y0) / 45.0f)));
    for (int i = 1; i <= samples; ++i) {
        const float t = static_cast<float>(i) / samples;
        if (!point_clear(x0 + (x1-x0)*t, y0 + (y1-y0)*t)) return false;
    }
    return true;
}
uint16_t valid_range(const char* name) {
    DistanceSensor* sensor = distance_sensor_get(name);
    if (!sensor || !sensor->isValid()) return 0;
    const uint16_t value = sensor->getDistanceMM();
    return value >= 30 && value <= 3500 ? value : 0;
}
uint16_t matrix_front() {
    uint16_t grid[64];
    if (!distance_sensors_get_8x8_grid(grid)) return 0;
    uint16_t values[24];
    uint8_t count = 0;
    // Ignore the two floor-facing rows and isolated close pixels. Require
    // a small coherent field before using the matrix as a collision range.
    for (uint8_t row = 0; row < 6; ++row)
        for (uint8_t col = 2; col < 6; ++col) {
            const uint16_t value = grid[row*8 + col];
            if (value >= 200 && value <= 3500) values[count++] = value;
        }
    if (count < 4) return 0;
    for (uint8_t i = 1; i < count; ++i) {
        const uint16_t value = values[i];
        uint8_t j = i;
        while (j > 0 && values[j-1] > value) { values[j] = values[j-1]; --j; }
        values[j] = value;
    }
    return values[(count-1)/4];
}
void update_ranges() {
    const uint16_t a = valid_range("tof_xshut7");
    const uint16_t b = valid_range("tof_xshut8");
    const uint16_t matrix = matrix_front();
    frontMm = 0;
    if (a) frontMm = a;
    if (b && (!frontMm || b < frontMm)) frontMm = b;
    if (matrix && (!frontMm || matrix < frontMm)) frontMm = matrix;
    leftMm = valid_range("ultrasonic_0");
    rightMm = valid_range("ultrasonic_1");
}
bool update_pose(uint32_t now) {
    if (!imu_is_valid() || imu_get_gyro_calibration() < 2) return false;
    const long left = encoder_get_position(0), right = encoder_get_position(1);
    const float leftMm = (left - previousLeft) * LEFT_MM_PER_COUNT;
    const float rightMm = -(right - previousRight) * RIGHT_MM_PER_COUNT;
    previousLeft = left; previousRight = right;
    const uint32_t elapsed = now - lastPoseMs;
    lastPoseMs = now;
    // A reset, disconnected encoder or implausible count burst cannot
    // teleport the map pose. A point turn contributes yaw, not translation.
    if (fabsf(leftMm) > 100.0f + 2.0f*elapsed ||
        fabsf(rightMm) > 100.0f + 2.0f*elapsed) return false;
    const float oldHeading = poseHeading;
    poseHeading = wrap(startHeading + delta(initialImu, imu_get_heading()));
    const float turn = delta(oldHeading, poseHeading);
    if (fabsf(turn) > 35.0f && elapsed < 250) return false;
    const bool pointTurn = drive_control_get_left_percent() * drive_control_get_right_percent() < 0;
    const float travel = pointTurn ? 0.0f : 0.5f*(leftMm + rightMm);
    const float middle = wrap(oldHeading + turn*0.5f) * DEG_TO_RAD;
    poseX += travel*cosf(middle);
    poseY += travel*sinf(middle);
    return finite(poseX) && finite(poseY);
}
void halt(const char* message, bool complete = false) {
    drive_control_stop();
    active = false;
    paused = false;
    set_state(complete ? "COMPLETE" : "STOPPED");
    set_reason(message);
}
void drive_toward(float x, float y, uint32_t now) {
    const float range = distance_to(x, y);
    const float bearing = wrap(atan2f(y-poseY, x-poseX)*RAD_TO_DEG);
    const float error = delta(poseHeading, bearing);
    if (fabsf(error) > 24.0f) {
        set_state("TURNING");
        const int turn = error > 0 ? 80 : -80;
        drive_control_set_percent(-turn, turn);
        return;
    }
    // Group 23's measured straight trim. Never command an ineffective 20-60%
    // moving speed; large heading error is resolved with a point turn.
    const int correction = constrain(static_cast<int>(lroundf(error*0.45f)), -5, 5);
    set_state(detourPhase ? "DETOUR" : "FOLLOWING");
    drive_control_set_percent(constrain(85-correction, 80, 100),
                              constrain(100+correction, 80, 100));
    if (range + 25.0f < bestDistance) {
        bestDistance = range;
        lastProgressMs = now;
    }
}
} // namespace

void mission_init() { mission_stop("No mission uploaded"); }

bool mission_begin(uint8_t count, float x, float y, float heading,
                   float radius, float safetyMargin) {
    if (active || count == 0 || count > MISSION_MAX_WAYPOINTS ||
        !finite(x) || !finite(y) || !finite(heading) ||
        !finite(radius) || !finite(safetyMargin) || radius < 100 || radius > 600 ||
        safetyMargin < 0 || safetyMargin > 400 || x < 0 || x > ARENA_X ||
        y < 0 || y > ARENA_Y) return false;
    memset(waypoints, 0, sizeof(waypoints));
    memset(obstacles, 0, sizeof(obstacles));
    waypointCount = count; obstacleCount = waypointIndex = detourCount = 0;
    startX = x; startY = y; startHeading = wrap(heading);
    robotRadius = radius; margin = safetyMargin;
    ready = false; paused = false;
    set_state("UPLOADING"); set_reason("Receiving route and map");
    return true;
}

bool mission_add_obstacle(uint8_t index, float x, float y, float width,
                          float depth, float angle, bool circular) {
    if (active || ready || index != obstacleCount ||
        obstacleCount >= MISSION_MAX_OBSTACLES ||
        !finite(x) || !finite(y) || !finite(width) || !finite(depth) || !finite(angle) ||
        x < 0 || x > ARENA_X || y < 0 || y > ARENA_Y ||
        width < 1 || width > 4900 || depth < 1 || depth > 2400) return false;
    obstacles[obstacleCount++] = {x, y, width, depth, angle, circular};
    return true;
}

bool mission_add_waypoint(uint8_t index, float x, float y, uint8_t flags) {
    if (active || ready || index >= waypointCount || !finite(x) || !finite(y) ||
        x < 0 || x > ARENA_X || y < 0 || y > ARENA_Y) return false;
    waypoints[index] = {static_cast<int16_t>(lroundf(x)),
                        static_cast<int16_t>(lroundf(y)), flags, true};
    return true;
}

bool mission_commit() {
    if (active || ready || waypointCount == 0 || !point_clear(startX, startY))
        return false;
    for (uint8_t i = 0; i < waypointCount; ++i) {
        if (!waypoints[i].loaded || !point_clear(waypoints[i].x, waypoints[i].y))
            return false;
        const float fromX = i ? waypoints[i-1].x : startX;
        const float fromY = i ? waypoints[i-1].y : startY;
        if (!segment_clear(fromX, fromY, waypoints[i].x, waypoints[i].y))
            return false;
    }
    ready = true;
    set_state("READY"); set_reason("Route and map validated");
    return true;
}

bool mission_start() {
    if (!ready || active || !imu_is_valid() || imu_get_gyro_calibration() < 2 ||
        drive_control_get_max_percent() < 100) return false;
    navigation_stop("Mission took control");
    drive_control_stop();
    poseX = startX; poseY = startY; poseHeading = startHeading;
    initialImu = imu_get_heading();
    previousLeft = encoder_get_position(0);
    previousRight = encoder_get_position(1);
    waypointIndex = detourCount = detourPhase = obstacleChecks = 0;
    startedMs = lastPoseMs = lastProgressMs = millis();
    blockedSinceMs = 0; bestDistance = 1e9f;
    paused = false; active = true;
    set_state("FOLLOWING"); set_reason("Following uploaded route on Teensy");
    return true;
}

bool mission_resume() {
    if (!ready || active || !paused || !imu_is_valid() ||
        drive_control_get_max_percent() < 100) return false;
    if (waypointIndex >= waypointCount) { halt("Route complete", true); return true; }
    active = true; paused = false;
    lastProgressMs = millis(); bestDistance = 1e9f;
    set_state("FOLLOWING"); set_reason("Operator resumed after weight site");
    return true;
}

void mission_stop(const char* message) {
    halt(message ? message : "Mission stopped");
    ready = false;
}

void mission_update() {
    if (!active) return;
    const uint32_t now = millis();
    if (now - startedMs > MAX_RUN_MS) { halt("Mission time limit"); return; }
    if (!update_pose(now)) { halt("IMU or encoder pose invalid"); return; }
    if (!point_clear(poseX, poseY)) { halt("Pose outside mapped clearance"); return; }
    update_ranges();

    const bool hardBlocked = frontMm && frontMm < HARD_STOP_MM;
    const bool nearBlocked = frontMm && frontMm < FRONT_STOP_MM;
    if (hardBlocked || nearBlocked) {
        drive_control_stop();
        if (!blockedSinceMs) blockedSinceMs = now;
        set_state("CHECKING_OBSTACLE");
        if (hardBlocked || now - blockedSinceMs >= 250) {
            // Side sonar is mandatory for an autonomous sidestep. Unknown
            // side clearance is not treated as open space.
            if (detourPhase == 0 && detourCount < 3) {
                const bool chooseLeft = leftMm >= SIDE_CLEAR_MM &&
                    (rightMm < SIDE_CLEAR_MM || leftMm >= rightMm);
                const bool chooseRight = rightMm >= SIDE_CLEAR_MM && !chooseLeft;
                if (chooseLeft || chooseRight) {
                    const float side = chooseLeft ? -1.0f : 1.0f;
                    const float angle = poseHeading * DEG_TO_RAD;
                    const float lateralX = -sinf(angle)*side*350.0f;
                    const float lateralY = cosf(angle)*side*350.0f;
                    detourX = poseX + lateralX; detourY = poseY + lateralY;
                    detourForwardX = detourX + cosf(angle)*550.0f;
                    detourForwardY = detourY + sinf(angle)*550.0f;
                    if (point_clear(detourX, detourY) &&
                        point_clear(detourForwardX, detourForwardY) &&
                        segment_clear(poseX, poseY, detourX, detourY) &&
                        segment_clear(detourX, detourY, detourForwardX, detourForwardY)) {
                        detourPhase = 1; ++detourCount;
                        blockedSinceMs = 0; lastProgressMs = now;
                        bestDistance = 1e9f;
                        set_state("DETOUR"); set_reason("Taking mapped-clear sonar-checked detour");
                        return;
                    }
                }
            }
            set_state("BLOCKED");
            set_reason("No verified clear detour; motors stopped");
        }
        return;
    }
    blockedSinceMs = 0;
    if (now - lastProgressMs > 8000) { halt("No waypoint progress for 8 seconds"); return; }
    float targetX, targetY;
    if (detourPhase == 1) { targetX = detourX; targetY = detourY; }
    else if (detourPhase == 2) { targetX = detourForwardX; targetY = detourForwardY; }
    else { targetX = waypoints[waypointIndex].x; targetY = waypoints[waypointIndex].y; }
    const float range = distance_to(targetX, targetY);
    if (range <= (detourPhase ? 130.0f : 90.0f)) {
        drive_control_stop(); bestDistance = 1e9f; lastProgressMs = now;
        if (detourPhase) { ++detourPhase; if (detourPhase > 2) detourPhase = 0; return; }
        if (waypoints[waypointIndex].flags & 1U) {
            ++waypointIndex;
            active = false; paused = true;
            set_state("WEIGHT_SITE");
            set_reason("Weight site reached; pickup unverified. Inspect and resume.");
            return;
        }
        ++waypointIndex;
        if (waypointIndex >= waypointCount) { halt("Route complete", true); return; }
        return;
    }
    if (!segment_clear(poseX, poseY, targetX, targetY)) {
        halt("Current leg entered mapped obstacle clearance"); return;
    }
    drive_toward(targetX, targetY, now);
}

bool mission_is_ready() { return ready; }
bool mission_is_active() { return active; }
const char* mission_state() { return state; }
const char* mission_reason() { return reason; }
uint8_t mission_waypoint_index() { return waypointIndex; }
uint8_t mission_waypoint_count() { return waypointCount; }
float mission_x_mm() { return poseX; }
float mission_y_mm() { return poseY; }
float mission_heading_deg() { return poseHeading; }
uint16_t mission_front_mm() { return frontMm; }
uint16_t mission_left_mm() { return leftMm; }
uint16_t mission_right_mm() { return rightMm; }
uint8_t mission_detour_count() { return detourCount; }
