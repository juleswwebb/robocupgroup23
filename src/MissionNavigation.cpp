#include "MissionNavigation.h"

#include "DistanceSensors.h"
#include "DriveControl.h"
#include "Encoders.h"
#include "IMU.h"
#include "Navigation.h"
#include "sensor_config.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

namespace {
constexpr float ARENA_X = 4900.0f, ARENA_Y = 2400.0f;
constexpr float LEFT_MM_PER_COUNT_DEFAULT = ENCODER0_MM_PER_COUNT;
constexpr float RIGHT_MM_PER_COUNT_DEFAULT = ENCODER1_MM_PER_COUNT;
constexpr uint32_t MAX_RUN_MS = 120000;
// The sensor boards sit on the robot perimeter. Treat a sensor return as a
// navigation obstacle only when its measured range is under 50 mm; projected
// footprint clearance is telemetry/planning context, not the trigger.
constexpr uint16_t OBSTACLE_SENSOR_RANGE_MM = 50;
constexpr float DETOUR_SIDE_CLEARANCE_MM = 400.0f;
constexpr float RUNTIME_MAP_CLEARANCE_MM = 20.0f;
// This array's lower rows see the floor/chassis. Use only its upper field,
// and require neighbouring pixels to agree before calling it an obstacle.
constexpr uint8_t MATRIX_FORWARD_ROWS = 4;
constexpr uint16_t MATRIX_MIN_RANGE_MM = 1;
constexpr uint16_t MATRIX_MAX_RANGE_MM = 3500;
constexpr uint16_t MATRIX_NEIGHBOUR_TOLERANCE_MM = 180;
constexpr uint8_t SENSOR_MOUNT_COUNT = 9;
constexpr uint8_t REQUIRED_CLOSE_SAMPLES = 2;
constexpr float WEIGHT_CENTER_BEARING_TOLERANCE_DEG = 5.0f;
constexpr float WEIGHT_CENTER_MAX_TURN_DEG = 18.0f;
constexpr uint32_t WEIGHT_CENTER_MAX_TIME_MS = 2500;
constexpr uint32_t WEIGHT_CENTER_PULSE_MS = 70;
constexpr uint32_t WEIGHT_CENTER_PERIOD_MS = 300;
constexpr int16_t NO_EDGE_CLEARANCE = -32768;

struct Waypoint { int16_t x, y; uint8_t flags; bool loaded; };
struct Obstacle { float x, y, width, depth, angle; bool circle; };
struct SensorMount {
    const char* key;
    const char* sensorName;
    float rightMm, forwardMm, angleDeg, heightMm;
    bool enabled;
};
Waypoint waypoints[MISSION_MAX_WAYPOINTS] = {};
Obstacle obstacles[MISSION_MAX_OBSTACLES] = {};
SensorMount sensorMounts[SENSOR_MOUNT_COUNT] = {
    {"xshut3", "tof_xshut3", 90, 140, -45, 0, true},
    {"xshut4", "tof_xshut4", 90, 140, -45, 0, true},
    {"xshut5", "tof_xshut5", -90, 140, 45, 0, true},
    {"xshut6", "tof_xshut6", -90, 140, 45, 0, true},
    {"xshut7", "tof_xshut7", 90, 140, 0, 0, true},
    {"xshut8", "tof_xshut8", -90, 140, 0, 0, true},
    {"matrix", nullptr, 0, 150, 0, 0, true},
    {"ultrasonic0", "ultrasonic_0", 110, 0, 90, 0, true},
    {"ultrasonic1", "ultrasonic_1", -110, 0, -90, 0, true},
};
uint8_t waypointCount = 0, obstacleCount = 0, waypointIndex = 0;
uint8_t detourCount = 0, detourPhase = 0, obstacleChecks = 0;
bool ready = false, active = false, paused = false;
bool siteSearching = false;
bool siteCentering = false, siteCenterPulseActive = false;
uint8_t siteVotes = 0;
float startX = 0, startY = 0, startHeading = 0;
float robotRadius = 215, margin = 90;
float leftMmPerCount = LEFT_MM_PER_COUNT_DEFAULT;
float rightMmPerCount = RIGHT_MM_PER_COUNT_DEFAULT;
bool leftEncoderReversed = ENCODER0_REVERSED;
bool rightEncoderReversed = ENCODER1_REVERSED;
float poseX = 0, poseY = 0, poseHeading = 0, initialImu = 0;
float detourX = 0, detourY = 0, detourForwardX = 0, detourForwardY = 0;
long previousLeft = 0, previousRight = 0;
uint32_t startedMs = 0, lastPoseMs = 0, lastProgressMs = 0;
uint32_t siteStartedMs = 0, lastSiteSampleMs = 0;
uint32_t siteCenterStartedMs = 0, siteCenterPulseAtMs = 0;
uint32_t siteCenterLastEvidenceMs = 0;
float siteCenterStartHeading = 0;
float bestDistance = 1e9f;
uint16_t frontMm = 0, leftMm = 0, rightMm = 0;
int16_t nearestEdgeClearanceMm = NO_EDGE_CLEARANCE;
int16_t frontEdgeClearanceMm = NO_EDGE_CLEARANCE;
int16_t leftEdgeClearanceMm = NO_EDGE_CLEARANCE;
int16_t rightEdgeClearanceMm = NO_EDGE_CLEARANCE;
bool haveNearestEdge = false, haveFrontEdge = false;
bool haveLeftEdge = false, haveRightEdge = false;
uint16_t nearestSideRangeMm = 0, nearestFrontRangeMm = 0;
bool haveNearestSideRange = false, haveNearestFrontRange = false;
char nearestSideSensorName[20] = "";
char nearestFrontSensorName[20] = "";
uint8_t closeSamples = 0;
char closeSensorName[20] = "";
uint8_t hardCloseSamples = 0;
char hardCloseSensorName[20] = "";
char frontSensorName[20] = "";
char nearestSensorName[20] = "";
float matrixFovDeg = 60.0f;
bool matrixMirrored = false;
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

bool point_clear_with_extra(float x, float y, float extraClearance) {
    const float clearance = robotRadius + extraClearance;
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
bool point_clear(float x, float y, float extra = 0.0f) {
    return point_clear_with_extra(x, y, margin + extra);
}
bool runtime_point_clear(float x, float y) {
    // The planner retains its configurable comfort margin. At runtime, do not
    // stop merely because odometry drift entered that virtual buffer: only
    // enforce the physical chassis footprint plus a small fixed reserve.
    return point_clear_with_extra(x, y, RUNTIME_MAP_CLEARANCE_MM);
}
bool segment_clear_with_extra(float x0, float y0, float x1, float y1,
                              float extraClearance) {
    const int samples = max(1, static_cast<int>(ceilf(hypotf(x1-x0, y1-y0) / 45.0f)));
    for (int i = 1; i <= samples; ++i) {
        const float t = static_cast<float>(i) / samples;
        if (!point_clear_with_extra(x0 + (x1-x0)*t, y0 + (y1-y0)*t,
                                    extraClearance)) return false;
    }
    return true;
}
bool segment_clear(float x0, float y0, float x1, float y1) {
    return segment_clear_with_extra(x0, y0, x1, y1, margin);
}
bool runtime_segment_clear(float x0, float y0, float x1, float y1) {
    return segment_clear_with_extra(x0, y0, x1, y1,
                                    RUNTIME_MAP_CLEARANCE_MM);
}
uint16_t valid_range(const char* name) {
    DistanceSensor* sensor = distance_sensor_get(name);
    if (!sensor || !sensor->isValid()) return 0;
    const uint16_t value = sensor->getDistanceMM();
    // Keep valid clipped-minimum VL53 measurements: the library normalizes
    // those to 1 mm, which is meaningful for the footprint emergency check.
    return value > 0 && value <= 3500 ? value : 0;
}
bool valid_matrix_range(uint16_t value) {
    return value >= MATRIX_MIN_RANGE_MM && value <= MATRIX_MAX_RANGE_MM;
}
bool matrix_pair_agrees(uint16_t first, uint16_t second) {
    return valid_matrix_range(first) && valid_matrix_range(second) &&
           abs(static_cast<int>(first) - static_cast<int>(second)) <=
               MATRIX_NEIGHBOUR_TOLERANCE_MM;
}
uint16_t matrix_front() {
    uint16_t grid[64];
    if (!distance_sensors_get_8x8_grid(grid)) return 0;
    uint16_t nearest = 0;
    // Use the central six columns and upper four rows. A lone pixel or a
    // lower-row floor return cannot confirm a frontal obstacle.
    for (uint8_t row = 0; row < MATRIX_FORWARD_ROWS; ++row) {
        for (uint8_t col = 1; col < 7; ++col) {
            const uint16_t here = grid[row * 8 + col];
            const bool rightPair = col < 6 &&
                matrix_pair_agrees(here, grid[row * 8 + col + 1]);
            const bool lowerPair = row + 1 < MATRIX_FORWARD_ROWS &&
                matrix_pair_agrees(here, grid[(row + 1) * 8 + col]);
            if (!rightPair && !lowerPair) continue;
            const uint16_t other = rightPair ? grid[row * 8 + col + 1]
                                             : grid[(row + 1) * 8 + col];
            const uint16_t pairRange = (here + other) / 2;
            if (!nearest || pairRange < nearest) nearest = pairRange;
        }
    }
    return nearest;
}
void update_ranges() {
    const uint16_t a = sensorMounts[4].enabled ? valid_range("tof_xshut7") : 0;
    const uint16_t b = sensorMounts[5].enabled ? valid_range("tof_xshut8") : 0;
    const uint16_t matrix = sensorMounts[6].enabled ? matrix_front() : 0;
    frontMm = 0;
    if (a) frontMm = a;
    if (b && (!frontMm || b < frontMm)) frontMm = b;
    if (matrix && (!frontMm || matrix < frontMm)) frontMm = matrix;
    // The Arena View defaults place ultrasonic0 on the robot's right and
    // ultrasonic1 on its left. Keep telemetry semantics consistent with that.
    rightMm = valid_range("ultrasonic_0");
    leftMm = valid_range("ultrasonic_1");

    nearestEdgeClearanceMm = frontEdgeClearanceMm = NO_EDGE_CLEARANCE;
    leftEdgeClearanceMm = rightEdgeClearanceMm = NO_EDGE_CLEARANCE;
    haveNearestEdge = haveFrontEdge = false;
    haveLeftEdge = haveRightEdge = false;
    nearestSideRangeMm = nearestFrontRangeMm = 0;
    haveNearestSideRange = haveNearestFrontRange = false;
    nearestSideSensorName[0] = nearestFrontSensorName[0] = '\0';
    frontSensorName[0] = '\0';
    nearestSensorName[0] = '\0';

    const auto recordHit = [&](const SensorMount& mount, float rangeMm,
                              float angleDeg, bool forwardFacing) {
        const uint16_t rawRangeMm = static_cast<uint16_t>(lroundf(rangeMm));
        if (forwardFacing && (!haveNearestFrontRange ||
                              rawRangeMm < nearestFrontRangeMm)) {
            haveNearestFrontRange = true;
            nearestFrontRangeMm = rawRangeMm;
            strncpy(nearestFrontSensorName, mount.key,
                    sizeof(nearestFrontSensorName) - 1);
            nearestFrontSensorName[sizeof(nearestFrontSensorName) - 1] = '\0';
        }
        if (!forwardFacing && (!haveNearestSideRange ||
                               rawRangeMm < nearestSideRangeMm)) {
            haveNearestSideRange = true;
            nearestSideRangeMm = rawRangeMm;
            strncpy(nearestSideSensorName, mount.key,
                    sizeof(nearestSideSensorName) - 1);
            nearestSideSensorName[sizeof(nearestSideSensorName) - 1] = '\0';
        }
        const float angle = angleDeg * DEG_TO_RAD;
        const float hitRight = mount.rightMm + rangeMm * sinf(angle);
        const float hitForward = mount.forwardMm + rangeMm * cosf(angle);
        const float edge = hypotf(hitRight, hitForward) - robotRadius;
        const int16_t clearance = static_cast<int16_t>(constrain(
            static_cast<int>(lroundf(edge)), -32768, 32767));
        if (!haveNearestEdge || clearance < nearestEdgeClearanceMm) {
            haveNearestEdge = true;
            nearestEdgeClearanceMm = clearance;
            strncpy(nearestSensorName, mount.key, sizeof(nearestSensorName) - 1);
            nearestSensorName[sizeof(nearestSensorName) - 1] = '\0';
        }
        if (forwardFacing && hitForward >= -robotRadius &&
            (!haveFrontEdge || clearance < frontEdgeClearanceMm)) {
            haveFrontEdge = true;
            frontEdgeClearanceMm = clearance;
            strncpy(frontSensorName, mount.key, sizeof(frontSensorName) - 1);
            frontSensorName[sizeof(frontSensorName) - 1] = '\0';
        }
        if (strcmp(mount.key, "ultrasonic0") == 0 &&
            (!haveRightEdge || clearance < rightEdgeClearanceMm)) {
            haveRightEdge = true;
            rightEdgeClearanceMm = clearance;
        }
        if (strcmp(mount.key, "ultrasonic1") == 0 &&
            (!haveLeftEdge || clearance < leftEdgeClearanceMm)) {
            haveLeftEdge = true;
            leftEdgeClearanceMm = clearance;
        }
    };

    // Include the angled weight sensors and straight-ahead point ToFs as
    // forward ranges, but do not react to their projected footprint gap.
    for (uint8_t i = 0; i < 4; ++i) {
        const SensorMount& mount = sensorMounts[i];
        if (!mount.enabled) continue;
        const uint16_t range = valid_range(mount.sensorName);
        if (range) recordHit(mount, range, mount.angleDeg, true);
    }
    for (uint8_t i = 4; i < 6; ++i) {
        const SensorMount& mount = sensorMounts[i];
        if (!mount.enabled) continue;
        const uint16_t range = valid_range(mount.sensorName);
        if (range) recordHit(mount, range, mount.angleDeg, true);
    }

    // Project spatially coherent returns from the upper four rows only.
    // Neighbouring cells must agree so a single noisy pixel cannot brake it.
    const SensorMount& matrixMount = sensorMounts[6];
    uint16_t grid[64];
    if (matrixMount.enabled && distance_sensors_get_8x8_grid(grid)) {
        const float side = matrixMirrored ? -1.0f : 1.0f;
        for (uint8_t row = 0; row < MATRIX_FORWARD_ROWS; ++row) {
            for (uint8_t col = 0; col < 8; ++col) {
                const uint16_t here = grid[row * 8 + col];
                if (!valid_matrix_range(here)) continue;
                // A single upper-field pixel below the perimeter threshold is
                // actionable; rows 4-7 remain excluded as floor/chassis views.
                if (here < OBSTACLE_SENSOR_RANGE_MM) {
                    const float angle = matrixMount.angleDeg +
                        side * (3.5f - col) * matrixFovDeg / 8.0f;
                    recordHit(matrixMount, here, angle, true);
                }
                if (col + 1 < 8 &&
                    matrix_pair_agrees(here, grid[row * 8 + col + 1])) {
                    const uint16_t range = (here + grid[row * 8 + col + 1]) / 2;
                    const float midCol = col + 0.5f;
                    const float angle = matrixMount.angleDeg +
                        side * (3.5f - midCol) * matrixFovDeg / 8.0f;
                    recordHit(matrixMount, range, angle, true);
                }
                if (row + 1 < MATRIX_FORWARD_ROWS &&
                    matrix_pair_agrees(here, grid[(row + 1) * 8 + col])) {
                    const uint16_t range = (here + grid[(row + 1) * 8 + col]) / 2;
                    const float angle = matrixMount.angleDeg +
                        side * (3.5f - col) * matrixFovDeg / 8.0f;
                    recordHit(matrixMount, range, angle, true);
                }
            }
        }
    }

    for (uint8_t i = 7; i < SENSOR_MOUNT_COUNT; ++i) {
        const SensorMount& mount = sensorMounts[i];
        if (!mount.enabled) continue;
        const uint16_t range = valid_range(mount.sensorName);
        if (range) recordHit(mount, range, mount.angleDeg, false);
    }
}
bool point_sensor_enabled(const char* sensorName) {
    for (uint8_t i = 0; i < 6; ++i) {
        if (strcmp(sensorMounts[i].sensorName, sensorName) == 0)
            return sensorMounts[i].enabled;
    }
    return false;
}
bool weight_pair_signature(const char* topName, const char* bottomName,
                           uint16_t* bottomRange = nullptr) {
    if (!point_sensor_enabled(topName) || !point_sensor_enabled(bottomName)) {
        if (bottomRange) *bottomRange = 0;
        return false;
    }
    const uint16_t top = valid_range(topName);
    const uint16_t bottom = valid_range(bottomName);
    if (bottomRange) *bottomRange = bottom;
    return bottom && top >= bottom + 150U && bottom <= 1200U;
}
bool weight_signature() {
    return weight_pair_signature("tof_xshut6", "tof_xshut5") ||
           weight_pair_signature("tof_xshut3", "tof_xshut4");
}
bool weight_target_bearing(float& bearingDeg) {
    float sumRight = 0, sumForward = 0;
    uint8_t samples = 0;
    uint16_t bottom = 0;
    if (weight_pair_signature("tof_xshut6", "tof_xshut5", &bottom)) {
        const SensorMount& mount = sensorMounts[2]; // front bottom left
        const float angle = mount.angleDeg * DEG_TO_RAD;
        sumRight += mount.rightMm + bottom * sinf(angle);
        sumForward += mount.forwardMm + bottom * cosf(angle);
        ++samples;
    }
    if (weight_pair_signature("tof_xshut3", "tof_xshut4", &bottom)) {
        const SensorMount& mount = sensorMounts[1]; // front bottom right
        const float angle = mount.angleDeg * DEG_TO_RAD;
        sumRight += mount.rightMm + bottom * sinf(angle);
        sumForward += mount.forwardMm + bottom * cosf(angle);
        ++samples;
    }
    if (!samples) return false;
    const float targetRight = sumRight / samples;
    const float targetForward = sumForward / samples;
    if (!finite(targetRight) || !finite(targetForward) || targetForward <= 0)
        return false;
    // Local mission convention: +x is right, +y is forward; positive bearing
    // is therefore corrected with the already-verified right-turn mixer.
    bearingDeg = atan2f(targetRight, targetForward) * RAD_TO_DEG;
    return finite(bearingDeg);
}
bool advance_to_next_site_view(uint32_t now) {
    const uint8_t siteId = waypoints[waypointIndex].flags >> 1;
    if (siteId == 0 || waypointIndex + 1 >= waypointCount ||
        (waypoints[waypointIndex + 1].flags >> 1) != siteId) return false;
    ++waypointIndex;
    siteSearching = false;
    siteCentering = siteCenterPulseActive = false;
    siteVotes = 0;
    lastProgressMs = now;
    bestDistance = 1e9f;
    set_state("SEARCHING_WEIGHT");
    set_reason("Checking the next planned sensor view for this weight site");
    return true;
}
void pause_at_weight_site(const char* message) {
    const uint8_t siteId = waypoints[waypointIndex].flags >> 1;
    ++waypointIndex;
    while (siteId != 0 && waypointIndex < waypointCount &&
           (waypoints[waypointIndex].flags >> 1) == siteId) {
        ++waypointIndex;
    }
    siteSearching = false;
    siteCentering = siteCenterPulseActive = false;
    active = false;
    paused = true;
    drive_control_stop();
    set_state("WEIGHT_SITE");
    set_reason(message);
}
bool update_pose(uint32_t now) {
    if (!imu_is_valid() || imu_get_gyro_calibration() < 2) return false;
    const long left = encoder_get_position(0), right = encoder_get_position(1);
    const long leftDelta = left - previousLeft;
    const long rightDelta = right - previousRight;
    const float leftMm = (leftEncoderReversed ? -leftDelta : leftDelta) * leftMmPerCount;
    const float rightMm = (rightEncoderReversed ? -rightDelta : rightDelta) * rightMmPerCount;
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
    siteSearching = siteCentering = siteCenterPulseActive = false;
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
        // Mission-map +Y points down the screen, so positive heading error is
        // a right turn. The ESC/drive convention requires left-forward and
        // right-reverse for that turn; the previous mixer drove it left and
        // increased the error (visible in the 2026-09-30 02:46 run).
        drive_control_set_percent(turn, -turn);
        return;
    }
    // Operator-set straight trim: left 100%, right 85%. Apply heading
    // correction around that baseline in the map's turn direction. Keep
    // moving commands >=80%; larger errors use a point turn above.
    const int correction = constrain(static_cast<int>(lroundf(error*0.45f)), -5, 5);
    set_state(detourPhase ? "DETOUR" : "FOLLOWING");
    drive_control_set_percent(constrain(100+correction, 80, 100),
                              constrain(85-correction, 80, 100));
    if (range + 25.0f < bestDistance) {
        bestDistance = range;
        lastProgressMs = now;
    }
}
} // namespace

void mission_init() { mission_stop("No mission uploaded"); }

static void reset_sensor_mounts() {
    sensorMounts[0] = {"xshut3", "tof_xshut3", 90, 140, -45, 0, true};
    sensorMounts[1] = {"xshut4", "tof_xshut4", 90, 140, -45, 0, true};
    sensorMounts[2] = {"xshut5", "tof_xshut5", -90, 140, 45, 0, true};
    sensorMounts[3] = {"xshut6", "tof_xshut6", -90, 140, 45, 0, true};
    sensorMounts[4] = {"xshut7", "tof_xshut7", 90, 140, 0, 0, true};
    sensorMounts[5] = {"xshut8", "tof_xshut8", -90, 140, 0, 0, true};
    sensorMounts[6] = {"matrix", nullptr, 0, 150, 0, 0, true};
    sensorMounts[7] = {"ultrasonic0", "ultrasonic_0", 110, 0, 90, 0, true};
    sensorMounts[8] = {"ultrasonic1", "ultrasonic_1", -110, 0, -90, 0, true};
    matrixFovDeg = 60.0f;
    matrixMirrored = false;
}

bool mission_begin(uint8_t count, float x, float y, float heading,
                   float radius, float safetyMargin,
                   float encoder0Scale, float encoder1Scale,
                   bool encoder0Reverse, bool encoder1Reverse) {
    if (active || count == 0 || count > MISSION_MAX_WAYPOINTS ||
        !finite(x) || !finite(y) || !finite(heading) ||
        !finite(radius) || !finite(safetyMargin) || !finite(encoder0Scale) ||
        !finite(encoder1Scale) || radius < 100 || radius > 600 ||
        safetyMargin < 0 || safetyMargin > 400 || x < 0 || x > ARENA_X ||
        y < 0 || y > ARENA_Y || encoder0Scale < 0.001f || encoder0Scale > 2.0f ||
        encoder1Scale < 0.001f || encoder1Scale > 2.0f) return false;
    memset(waypoints, 0, sizeof(waypoints));
    memset(obstacles, 0, sizeof(obstacles));
    waypointCount = count; obstacleCount = waypointIndex = detourCount = 0;
    startX = x; startY = y; startHeading = wrap(heading);
    robotRadius = radius; margin = safetyMargin;
    reset_sensor_mounts();
    leftMmPerCount = encoder0Scale; rightMmPerCount = encoder1Scale;
    leftEncoderReversed = encoder0Reverse; rightEncoderReversed = encoder1Reverse;
    ready = false; paused = false; siteSearching = false; siteVotes = 0;
    siteCentering = siteCenterPulseActive = false;
    closeSamples = 0; closeSensorName[0] = '\0';
    hardCloseSamples = 0; hardCloseSensorName[0] = '\0';
    set_state("UPLOADING"); set_reason("Receiving route and map");
    return true;
}

bool mission_set_sensor(const char* key, float rightMm, float forwardMm,
                        float angleDeg, float heightMm, bool enabled,
                        float fovDeg, bool mirrored) {
    if (active || ready || !key || !finite(rightMm) || !finite(forwardMm) ||
        !finite(angleDeg) || !finite(heightMm) || rightMm < -1000 ||
        rightMm > 1000 || forwardMm < -1000 || forwardMm > 1000 ||
        angleDeg < -180 || angleDeg > 180 || heightMm < 0 || heightMm > 1500)
        return false;
    for (uint8_t i = 0; i < SENSOR_MOUNT_COUNT; ++i) {
        SensorMount& mount = sensorMounts[i];
        if (strcmp(key, mount.key) != 0) continue;
        if (i == 6 && (!finite(fovDeg) || fovDeg < 10 || fovDeg > 120))
            return false;
        mount.rightMm = rightMm;
        mount.forwardMm = forwardMm;
        mount.angleDeg = angleDeg;
        mount.heightMm = heightMm;
        mount.enabled = enabled;
        if (i == 6) {
            matrixFovDeg = fovDeg;
            matrixMirrored = mirrored;
        }
        return true;
    }
    return false;
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
    closeSamples = 0; closeSensorName[0] = '\0';
    hardCloseSamples = 0; hardCloseSensorName[0] = '\0';
    bestDistance = 1e9f;
    paused = false; active = true; siteSearching = false; siteVotes = 0;
    siteCentering = siteCenterPulseActive = false;
    set_state("FOLLOWING"); set_reason("Following uploaded route on Teensy");
    return true;
}

bool mission_resume() {
    if (!ready || active || !paused || !imu_is_valid() ||
        imu_get_gyro_calibration() < 2 ||
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
    update_ranges();
    if (!runtime_point_clear(poseX, poseY)) {
        halt("Pose outside physical arena/map clearance"); return;
    }
    if (haveNearestSideRange &&
        nearestSideRangeMm < OBSTACLE_SENSOR_RANGE_MM) {
        // Side-facing sensor below 50 mm: brake and latch only after the same
        // sensor confirms the close return on the next control sample.
        drive_control_stop();
        if (strcmp(hardCloseSensorName, nearestSideSensorName) == 0) {
            if (hardCloseSamples < REQUIRED_CLOSE_SAMPLES) ++hardCloseSamples;
        } else {
            strncpy(hardCloseSensorName, nearestSideSensorName,
                    sizeof(hardCloseSensorName) - 1);
            hardCloseSensorName[sizeof(hardCloseSensorName) - 1] = '\0';
            hardCloseSamples = 1;
        }
        if (hardCloseSamples >= REQUIRED_CLOSE_SAMPLES) {
            halt("Side sensor confirmed a return below 50 mm");
        } else {
            set_state("CHECKING_OBSTACLE");
            set_reason("Braked; confirming the same side sensor is below 50 mm");
        }
        return;
    }
    hardCloseSamples = 0;
    hardCloseSensorName[0] = '\0';

    // A target-tagged waypoint is a planned weight-search view. Repeated
    // upper/lower evidence is required before centering. Project the lower
    // sensor return through the configured mount geometry, then use short,
    // IMU-bounded turns to put the detected weight in the forward centreline.
    // If the signal is incomplete, check the site's remaining planned views.
    if (siteSearching) {
        if (siteCentering) {
            float targetBearing = 0;
            if (weight_target_bearing(targetBearing)) {
                siteCenterLastEvidenceMs = now;
            } else if (now - siteCenterLastEvidenceMs > 450U) {
                drive_control_stop();
                pause_at_weight_site(
                    "Weight was detected but fresh paired ToF evidence was lost while centering; check then resume.");
                return;
            } else {
                drive_control_stop();
                set_state("CENTERING_WEIGHT");
                set_reason("Holding briefly while waiting for fresh top/bottom ToF evidence");
                return;
            }

            const float headingChange = fabsf(delta(siteCenterStartHeading, poseHeading));
            if (fabsf(targetBearing) <= WEIGHT_CENTER_BEARING_TOLERANCE_DEG) {
                pause_at_weight_site(
                    "Weight centered in front using the top/bottom ToF geometry. Check/remove it, then resume the route.");
                return;
            }
            if (now - siteCenterStartedMs >= WEIGHT_CENTER_MAX_TIME_MS ||
                headingChange >= WEIGHT_CENTER_MAX_TURN_DEG) {
                drive_control_stop();
                pause_at_weight_site(
                    "Weight detected; bounded ToF centering reached its limit. Check alignment, then resume.");
                return;
            }

            if (siteCenterPulseActive) {
                if (now - siteCenterPulseAtMs < WEIGHT_CENTER_PULSE_MS) {
                    set_state("CENTERING_WEIGHT");
                    set_reason("Fine-centering from the projected top/bottom ToF target");
                    return;
                }
                drive_control_stop();
                siteCenterPulseActive = false;
                siteCenterPulseAtMs = now;
                set_state("CENTERING_WEIGHT");
                set_reason("Settling and rechecking top/bottom ToF target geometry");
                return;
            }
            if (now - siteCenterPulseAtMs < WEIGHT_CENTER_PERIOD_MS) {
                drive_control_stop();
                set_state("CENTERING_WEIGHT");
                set_reason("Settling and rechecking top/bottom ToF target geometry");
                return;
            }

            // Positive relative bearing places the target to the robot's
            // right. Positive mixer output is a right turn.
            const int turn = targetBearing > 0 ? 80 : -80;
            drive_control_set_percent(turn, -turn);
            siteCenterPulseActive = true;
            siteCenterPulseAtMs = now;
            set_state("CENTERING_WEIGHT");
            set_reason("Centering the projected weight position in the front sensor field");
            return;
        }

        drive_control_stop();
        if (now - lastSiteSampleMs >= 150U) {
            lastSiteSampleMs = now;
            if (weight_signature()) { if (siteVotes < 3) ++siteVotes; }
            else siteVotes = 0;
        }
        if (siteVotes >= 3) {
            float targetBearing = 0;
            if (weight_target_bearing(targetBearing)) {
                if (fabsf(targetBearing) <= WEIGHT_CENTER_BEARING_TOLERANCE_DEG) {
                    pause_at_weight_site(
                        "Weight centered in front using the top/bottom ToF geometry. Check/remove it, then resume the route.");
                    return;
                }
                siteCentering = true;
                siteCenterPulseActive = false;
                siteCenterStartedMs = now;
                siteCenterPulseAtMs = now - WEIGHT_CENTER_PERIOD_MS;
                siteCenterLastEvidenceMs = now;
                siteCenterStartHeading = poseHeading;
                set_state("CENTERING_WEIGHT");
                set_reason("Top/bottom ToF pair confirmed the weight; centering it in front");
                return;
            }
            if (advance_to_next_site_view(now)) return;
            pause_at_weight_site(
                "Weight-like signature confirmed, but ToF geometry could not place it for centering. Check/remove it, then resume.");
            return;
        }
        if (now - siteStartedMs >= 900U) {
            siteSearching = false;
            ++waypointIndex;
            if (waypointIndex >= waypointCount) { halt("Route complete; no confirmed weight at final site", true); return; }
            lastProgressMs = now; bestDistance = 1e9f;
        } else {
            set_state("SEARCHING_WEIGHT");
            return;
        }
    }

    const float routeBearing = waypointIndex < waypointCount
        ? wrap(atan2f(waypoints[waypointIndex].y - poseY,
                      waypoints[waypointIndex].x - poseX) * RAD_TO_DEG) : poseHeading;
    const bool facingRoute = fabsf(delta(poseHeading, routeBearing)) < 35.0f;
    // A forward sensor only starts avoidance when its raw measured range is
    // under 50 mm, confirmed by the same sensor on two mission samples.
    const bool closeForwardHit = haveNearestFrontRange &&
        nearestFrontRangeMm < OBSTACLE_SENSOR_RANGE_MM &&
        (!detourPhase || facingRoute);
    if (closeForwardHit) {
        if (strcmp(closeSensorName, nearestFrontSensorName) == 0) {
            if (closeSamples < REQUIRED_CLOSE_SAMPLES) ++closeSamples;
        } else {
            strncpy(closeSensorName, nearestFrontSensorName,
                    sizeof(closeSensorName) - 1);
            closeSensorName[sizeof(closeSensorName) - 1] = '\0';
            closeSamples = 1;
        }
    } else {
        closeSamples = 0;
        closeSensorName[0] = '\0';
    }
    if (closeSamples >= REQUIRED_CLOSE_SAMPLES) {
        drive_control_stop();
        set_state("CHECKING_OBSTACLE");
        if (detourPhase == 0 && detourCount < 3) {
            // A sidestep is allowed only when the side-facing sensors' own
            // projected hits show enough free space for the robot to pass.
            const bool leftOpen = haveLeftEdge &&
                leftEdgeClearanceMm >= DETOUR_SIDE_CLEARANCE_MM;
            const bool rightOpen = haveRightEdge &&
                rightEdgeClearanceMm >= DETOUR_SIDE_CLEARANCE_MM;
            const bool chooseLeft = leftOpen &&
                (!rightOpen || leftEdgeClearanceMm >= rightEdgeClearanceMm);
            const bool chooseRight = rightOpen && !chooseLeft;
            if (chooseLeft || chooseRight) {
                // In the mission frame heading is measured from +x; at the
                // normal +y-forward heading, positive side is robot-left.
                const float side = chooseLeft ? 1.0f : -1.0f;
                const float angle = poseHeading * DEG_TO_RAD;
                const float lateralX = -sinf(angle)*side*350.0f;
                const float lateralY = cosf(angle)*side*350.0f;
                detourX = poseX + lateralX; detourY = poseY + lateralY;
                detourForwardX = detourX + cosf(angle)*550.0f;
                detourForwardY = detourY + sinf(angle)*550.0f;
                if (runtime_point_clear(detourX, detourY) &&
                    runtime_point_clear(detourForwardX, detourForwardY) &&
                    runtime_segment_clear(poseX, poseY, detourX, detourY) &&
                    runtime_segment_clear(detourX, detourY,
                                          detourForwardX, detourForwardY)) {
                    detourPhase = 1; ++detourCount;
                    closeSamples = 0; closeSensorName[0] = '\0';
                    lastProgressMs = now; bestDistance = 1e9f;
                    set_state("DETOUR");
                    set_reason("Near sensor hit; following footprint-clear detour");
                    return;
                }
            }
        }
        set_state("BLOCKED");
        set_reason("Forward sensor below 50 mm; no footprint-clear detour available");
        return;
    }
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
            siteSearching = true; siteVotes = 0; siteStartedMs = now;
            lastSiteSampleMs = 0;
            set_state("SEARCHING_WEIGHT");
            set_reason("Holding at planned weight-search pose");
            return;
        }
        ++waypointIndex;
        if (waypointIndex >= waypointCount) { halt("Route complete", true); return; }
        return;
    }
    if (!runtime_segment_clear(poseX, poseY, targetX, targetY)) {
        halt("Current leg intersects the physical footprint clearance"); return;
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
int16_t mission_nearest_edge_clearance_mm() {
    return haveNearestEdge ? nearestEdgeClearanceMm : NO_EDGE_CLEARANCE;
}
int16_t mission_front_edge_clearance_mm() {
    return haveFrontEdge ? frontEdgeClearanceMm : NO_EDGE_CLEARANCE;
}
int16_t mission_left_edge_clearance_mm() {
    return haveLeftEdge ? leftEdgeClearanceMm : NO_EDGE_CLEARANCE;
}
int16_t mission_right_edge_clearance_mm() {
    return haveRightEdge ? rightEdgeClearanceMm : NO_EDGE_CLEARANCE;
}
const char* mission_nearest_sensor() { return haveNearestEdge ? nearestSensorName : ""; }
uint8_t mission_detour_count() { return detourCount; }
