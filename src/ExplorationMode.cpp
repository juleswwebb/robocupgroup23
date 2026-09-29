#include "ExplorationMode.h"

#include "DistanceSensors.h"
#include "DriveControl.h"
#include "DrumControl.h"
#include "MagnetControl.h"
#include "sensor_config.h"
#include "sensors/DistanceSensor.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

namespace {
enum class State : uint8_t { Idle, Forward, AvoidTurn, WeightAlign, WeightApproach };

// The array is the primary front obstacle detector. It needs a neighbouring
// pair of agreeing pixels, and three distinct frames, so isolated speckle
// returns don't initiate an avoidance turn. This is an avoid threshold, not a
// stop threshold: a hit causes a turn and exploration continues.
constexpr uint16_t MATRIX_TURN_RANGE_MM = 420;
constexpr uint16_t MATRIX_CLEAR_RANGE_MM = 560;
constexpr uint16_t MATRIX_NEIGHBOUR_TOLERANCE_MM = 180;
constexpr uint8_t MATRIX_FORWARD_ROWS = 4;
constexpr uint8_t OBSTACLE_CONFIRM_FRAMES = 3;
constexpr uint8_t CLEAR_CONFIRM_FRAMES = 2;
constexpr uint32_t CONTROL_PERIOD_MS = 20;
constexpr uint32_t RANGE_SAMPLE_PERIOD_MS = 90;
constexpr uint32_t HOST_LEASE_MS = 700;
constexpr uint32_t SENSOR_LOSS_LIMIT_MS = 500;
constexpr uint32_t AVOID_MIN_TURN_MS = 500;
constexpr uint32_t AVOID_MAX_TURN_MS = 5000;
constexpr uint32_t ALIGN_MAX_MS = 2600;
constexpr uint32_t WEIGHT_EVIDENCE_LOSS_MS = 450;
constexpr uint32_t APPROACH_MIN_MS = 350;
constexpr uint32_t APPROACH_MAX_MS = 2800;
constexpr uint32_t WEIGHT_COOLDOWN_MS = 1600;
constexpr uint16_t WEIGHT_TOP_BOTTOM_GAP_MM = 150;
constexpr uint16_t WEIGHT_MAX_RANGE_MM = 500;
constexpr uint16_t WEIGHT_BACKGROUND_DROP_MM = 120;
constexpr uint8_t WEIGHT_CONFIRM_SAMPLES = 3;
constexpr int STRAIGHT_LEFT_PERCENT = 100;
constexpr int STRAIGHT_RIGHT_PERCENT = 85;
constexpr int TURN_PERCENT = 85;
constexpr int ALIGN_TURN_PERCENT = 80;
constexpr int DRUM_LEFT_PERCENT = -100;
constexpr int DRUM_RIGHT_PERCENT = -100;

State state = State::Idle;
bool active = false;
bool nextTieTurnRight = true;
int8_t turnDirection = 1;
uint32_t lastKeepaliveMs = 0;
uint32_t lastRangeSampleMs = 0;
uint32_t lastMatrixFrameMs = 0;
uint32_t lastSensorSeenMs = 0;
uint32_t stateStartedMs = 0;
uint32_t lastEvidenceMs = 0;
uint32_t lastObstacleTriggerMs = 0;
uint32_t lastAvoidEvidenceMs = 0;
uint32_t lastWeightSampleMs = 0;
uint32_t cooldownUntilMs = 0;
uint8_t obstacleVotes = 0;
uint8_t clearVotes = 0;
uint8_t weightVotes = 0;
int8_t weightCandidateSide = 0;
uint16_t leftWeightBackgroundMm = 0;
uint16_t rightWeightBackgroundMm = 0;
uint16_t frontMm = 0;
uint16_t leftMm = 0;
uint16_t rightMm = 0;
uint16_t leftTopMm = 0;
uint16_t leftBottomMm = 0;
uint16_t rightTopMm = 0;
uint16_t rightBottomMm = 0;
uint16_t weightsSeen = 0;
uint16_t turnsTaken = 0;
float targetBearingDeg = 0.0f;
bool leftWeightSignature = false;
bool rightWeightSignature = false;
bool haveMatrixFrame = false;
bool frontSensorsAvailable = false;
char reason[112] = "Not started";

uint16_t valid_range(const char* name) {
    DistanceSensor* sensor = distance_sensor_get(name);
    if (sensor == nullptr || !sensor->isValid()) return 0;
    const uint16_t range = sensor->getDistanceMM();
    return (range > 0 && range < 4000) ? range : 0;
}

bool valid_matrix_range(uint16_t range) {
    return range > 0 && range < 4000;
}

uint16_t matrix_front_range(const uint16_t* grid) {
    uint16_t nearest = 0;
    for (uint8_t row = 0; row < MATRIX_FORWARD_ROWS; ++row) {
        for (uint8_t col = 1; col < 7; ++col) {
            const uint16_t here = grid[row * 8 + col];
            if (!valid_matrix_range(here)) continue;
            const uint16_t right = col < 6 ? grid[row * 8 + col + 1] : 0;
            const uint16_t below = row + 1 < MATRIX_FORWARD_ROWS
                ? grid[(row + 1) * 8 + col] : 0;
            uint16_t other = 0;
            if (valid_matrix_range(right) &&
                abs(static_cast<int>(here) - static_cast<int>(right)) <=
                    MATRIX_NEIGHBOUR_TOLERANCE_MM) {
                other = right;
            } else if (valid_matrix_range(below) &&
                       abs(static_cast<int>(here) - static_cast<int>(below)) <=
                           MATRIX_NEIGHBOUR_TOLERANCE_MM) {
                other = below;
            }
            if (!other) continue;
            const uint16_t pairRange = (here + other) / 2;
            if (!nearest || pairRange < nearest) nearest = pairRange;
        }
    }
    return nearest;
}

uint16_t matrix_side_open_score(const uint16_t* grid, bool rightSide) {
    uint16_t nearestPair = 0;
    const uint8_t firstCol = rightSide ? 4 : 0;
    const uint8_t lastCol = rightSide ? 8 : 4;
    for (uint8_t row = 0; row < MATRIX_FORWARD_ROWS; ++row) {
        for (uint8_t col = firstCol; col < lastCol; ++col) {
            const uint16_t here = grid[row * 8 + col];
            if (!valid_matrix_range(here)) continue;
            if (col + 1 < lastCol) {
                const uint16_t right = grid[row * 8 + col + 1];
                if (valid_matrix_range(right) &&
                    abs(static_cast<int>(here) - static_cast<int>(right)) <=
                        MATRIX_NEIGHBOUR_TOLERANCE_MM) {
                    const uint16_t pairRange = (here + right) / 2;
                    if (!nearestPair || pairRange < nearestPair) nearestPair = pairRange;
                }
            }
        }
    }
    // No coherent hit in this side of the forward field is the clearest view.
    return nearestPair ? nearestPair : 4000;
}

bool weight_pair(const char* topName, const char* bottomName,
                 uint16_t& top, uint16_t& bottom, uint16_t& background) {
    top = valid_range(topName);
    bottom = valid_range(bottomName);
    const bool candidate = bottom > 0 && bottom <= WEIGHT_MAX_RANGE_MM &&
        background >= bottom + WEIGHT_BACKGROUND_DROP_MM &&
        (top == 0 || top >= bottom + WEIGHT_TOP_BOTTOM_GAP_MM);
    // Learn the normal view of the ground on this side. Freeze it while a
    // candidate is visible so a real weight stays detectable during centering.
    if (bottom && !candidate) {
        background = background ? (background * 7U + bottom + 4U) / 8U : bottom;
    }
    return candidate;
}

void sample_weights(uint32_t now) {
    if (now - lastWeightSampleMs < RANGE_SAMPLE_PERIOD_MS) return;
    lastWeightSampleMs = now;
    leftWeightSignature = weight_pair("tof_xshut6", "tof_xshut5",
                                      leftTopMm, leftBottomMm,
                                      leftWeightBackgroundMm);
    rightWeightSignature = weight_pair("tof_xshut3", "tof_xshut4",
                                       rightTopMm, rightBottomMm,
                                       rightWeightBackgroundMm);
    const int8_t candidateSide = leftWeightSignature && rightWeightSignature ? 2 :
        leftWeightSignature ? -1 : rightWeightSignature ? 1 : 0;
    if (candidateSide != 0 && candidateSide == weightCandidateSide) {
        if (weightVotes < WEIGHT_CONFIRM_SAMPLES) ++weightVotes;
    } else {
        weightVotes = candidateSide ? 1 : 0;
    }
    weightCandidateSide = candidateSide;

    float sumRight = 0.0f;
    float sumForward = 0.0f;
    uint8_t count = 0;
    constexpr float radians = 0.017453292519943295f;
    if (leftWeightSignature) {
        sumRight += -90.0f + leftBottomMm * sinf(45.0f * radians);
        sumForward += 140.0f + leftBottomMm * cosf(45.0f * radians);
        ++count;
    }
    if (rightWeightSignature) {
        sumRight += 90.0f - rightBottomMm * sinf(45.0f * radians);
        sumForward += 140.0f + rightBottomMm * cosf(45.0f * radians);
        ++count;
    }
    if (count && sumForward > 0.0f) {
        targetBearingDeg = atan2f(sumRight / count, sumForward / count) /
                           radians;
    }
}

bool sample_front(uint32_t now, uint16_t* matrixGrid, bool& newMatrixFrame) {
    haveMatrixFrame = distance_sensors_get_8x8_grid(matrixGrid);
    newMatrixFrame = false;
    uint16_t matrixRange = 0;
    if (haveMatrixFrame) {
        bool frameHasRangeCodes = false;
        for (uint8_t i = 0; i < 64; ++i) {
            if (matrixGrid[i] > 0 && matrixGrid[i] <= 4000) {
                frameHasRangeCodes = true;
                break;
            }
        }
        haveMatrixFrame = frameHasRangeCodes;
    }
    if (haveMatrixFrame) {
        const uint32_t frameAt = distance_sensors_8x8_last_success_ms();
        newMatrixFrame = frameAt != 0 && frameAt != lastMatrixFrameMs;
        if (newMatrixFrame) lastMatrixFrameMs = frameAt;
        matrixRange = matrix_front_range(matrixGrid);
    }
    const uint16_t straightLeft = valid_range("tof_xshut7");
    const uint16_t straightRight = valid_range("tof_xshut8");
    frontMm = matrixRange;
    if (straightLeft && (!frontMm || straightLeft < frontMm)) frontMm = straightLeft;
    if (straightRight && (!frontMm || straightRight < frontMm)) frontMm = straightRight;
    leftMm = valid_range("ultrasonic_1");
    rightMm = valid_range("ultrasonic_0");
    frontSensorsAvailable = haveMatrixFrame || straightLeft || straightRight;
    if (frontSensorsAvailable) lastSensorSeenMs = now;
    return frontSensorsAvailable;
}

bool obstacle_ahead(uint32_t now, const uint16_t* grid, bool newMatrixFrame) {
    bool matrixBlocked = false;
    if (haveMatrixFrame && newMatrixFrame) {
        const uint16_t matrixRange = matrix_front_range(grid);
        matrixBlocked = matrixRange > 0 && matrixRange <= MATRIX_TURN_RANGE_MM;
    }
    const uint16_t straightLeft = valid_range("tof_xshut7");
    const uint16_t straightRight = valid_range("tof_xshut8");
    const bool pointBlocked =
        (straightLeft > 0 && straightLeft <= 250) ||
        (straightRight > 0 && straightRight <= 250);
    // Count distinct array frames. Only use the timer for direct point-ToF
    // evidence, whose sensor drivers refresh independently at ~50 ms.
    const bool newPointSample = (straightLeft || straightRight) &&
        now - lastObstacleTriggerMs >= RANGE_SAMPLE_PERIOD_MS;
    const bool newEvidence = newMatrixFrame || newPointSample;
    if (!newEvidence) return false;
    lastObstacleTriggerMs = now;
    if (matrixBlocked || pointBlocked) {
        if (obstacleVotes < OBSTACLE_CONFIRM_FRAMES) ++obstacleVotes;
    } else {
        obstacleVotes = 0;
    }
    return obstacleVotes >= OBSTACLE_CONFIRM_FRAMES;
}

void choose_turn_direction(const uint16_t* grid) {
    // Ultrasonics are side-facing; use them when both return useful ranges.
    if (leftMm && rightMm && abs(static_cast<int>(leftMm) - rightMm) > 60) {
        turnDirection = rightMm > leftMm ? 1 : -1;
    } else if (haveMatrixFrame) {
        const uint16_t leftScore = matrix_side_open_score(grid, false);
        const uint16_t rightScore = matrix_side_open_score(grid, true);
        if (leftScore != rightScore) turnDirection = rightScore > leftScore ? 1 : -1;
        else turnDirection = nextTieTurnRight ? 1 : -1;
    } else {
        turnDirection = nextTieTurnRight ? 1 : -1;
    }
    nextTieTurnRight = !nextTieTurnRight;
}

void set_state(State next, const char* status) {
    state = next;
    stateStartedMs = millis();
    strncpy(reason, status, sizeof(reason) - 1);
    reason[sizeof(reason) - 1] = '\0';
}

void drive_forward() {
    drive_control_set_percent(STRAIGHT_LEFT_PERCENT, STRAIGHT_RIGHT_PERCENT);
}

void finish_avoidance(uint32_t now) {
    obstacleVotes = 0;
    clearVotes = 0;
    drive_control_stop();
    set_state(State::Forward, "Obstacle avoided; exploring forward");
    lastObstacleTriggerMs = now;
    // The robot is facing a different patch of ground after turning.
    leftWeightBackgroundMm = rightWeightBackgroundMm = 0;
    weightVotes = weightCandidateSide = 0;
}

void finish_weight_attempt(uint32_t now, const char* status) {
    cooldownUntilMs = now + WEIGHT_COOLDOWN_MS;
    leftWeightBackgroundMm = leftBottomMm ? leftBottomMm : leftWeightBackgroundMm;
    rightWeightBackgroundMm = rightBottomMm ? rightBottomMm : rightWeightBackgroundMm;
    weightVotes = weightCandidateSide = 0;
    set_state(State::Forward, status);
}
}

void exploration_mode_init() {
    active = false;
    state = State::Idle;
    strncpy(reason, "Stopped", sizeof(reason) - 1);
    reason[sizeof(reason) - 1] = '\0';
}

bool exploration_mode_start() {
    const uint32_t now = millis();
    uint16_t grid[64];
    bool newMatrixFrame = false;
    if (!sample_front(now, grid, newMatrixFrame)) {
        strncpy(reason, "Start rejected: no valid 8x8 or straight-ahead TOF data",
                sizeof(reason) - 1);
        reason[sizeof(reason) - 1] = '\0';
        return false;
    }
    if (drive_control_get_max_percent() < 80) {
        strncpy(reason, "Start rejected: drive limit must be at least 80%",
                sizeof(reason) - 1);
        reason[sizeof(reason) - 1] = '\0';
        return false;
    }

    active = true;
    obstacleVotes = clearVotes = weightVotes = 0;
    weightCandidateSide = 0;
    leftWeightBackgroundMm = rightWeightBackgroundMm = 0;
    lastKeepaliveMs = now;
    lastRangeSampleMs = lastWeightSampleMs = now - RANGE_SAMPLE_PERIOD_MS;
    lastObstacleTriggerMs = now;
    lastSensorSeenMs = now;
    lastMatrixFrameMs = distance_sensors_8x8_last_success_ms();
    cooldownUntilMs = now;
    weightsSeen = 0;
    turnsTaken = 0;
    targetBearingDeg = 0.0f;
    sample_weights(now);
    set_state(State::Forward, "Exploring; drum running at -100% / -100%");
    drive_forward();
    drum_control_set_percent(DRUM_LEFT_PERCENT, DRUM_RIGHT_PERCENT);
    magnet_control_set(true);
    return true;
}

void exploration_mode_stop(const char* message) {
    const bool wasActive = active;
    active = false;
    state = State::Idle;
    obstacleVotes = clearVotes = weightVotes = 0;
    // A manual drive command can take over without interrupting an operator's
    // separately latched drum. Only neutralise these outputs if exploration
    // actually owned them; global STOP/debug-exit have their own unconditional
    // safe-output handling in the protocol layer.
    if (wasActive) {
        drive_control_stop();
        drum_control_stop();
        magnet_control_off();
    }
    if (message == nullptr || *message == '\0') message = "Stopped by operator";
    strncpy(reason, message, sizeof(reason) - 1);
    reason[sizeof(reason) - 1] = '\0';
}

void exploration_mode_keepalive() {
    if (active) lastKeepaliveMs = millis();
}

void exploration_mode_update() {
    if (!active) return;
    const uint32_t now = millis();
    if (now - lastKeepaliveMs > HOST_LEASE_MS) {
        exploration_mode_stop("Stopped: app keepalive lost");
        return;
    }
    if (now - lastRangeSampleMs >= CONTROL_PERIOD_MS) {
        lastRangeSampleMs = now;
        uint16_t grid[64];
        bool newMatrixFrame = false;
        sample_front(now, grid, newMatrixFrame);
        sample_weights(now);

        if (!frontSensorsAvailable && now - lastSensorSeenMs > SENSOR_LOSS_LIMIT_MS) {
            exploration_mode_stop("Stopped: all front ranging data lost");
            return;
        }

        if (state == State::Forward) {
            // A confirmed wall wins over a possible weight signature.
            if (obstacle_ahead(now, grid, newMatrixFrame)) {
                choose_turn_direction(grid);
                ++turnsTaken;
                clearVotes = 0;
                lastAvoidEvidenceMs = now;
                set_state(State::AvoidTurn,
                          turnDirection > 0
                              ? "Front obstacle confirmed; turning right to explore"
                              : "Front obstacle confirmed; turning left to explore");
            } else if (now >= cooldownUntilMs &&
                       weightVotes >= WEIGHT_CONFIRM_SAMPLES) {
                    lastEvidenceMs = now;
                    targetBearingDeg = constrain(targetBearingDeg, -35.0f, 35.0f);
                    if (fabsf(targetBearingDeg) <= 7.0f) {
                        ++weightsSeen;
                        set_state(State::WeightApproach,
                                  "Weight pair confirmed; driving through with drum running");
                    } else {
                        set_state(State::WeightAlign,
                                  "Weight pair confirmed; centering target");
                    }
            }
        } else if (state == State::WeightAlign) {
            if (leftWeightSignature || rightWeightSignature) lastEvidenceMs = now;
            if (now - lastEvidenceMs > WEIGHT_EVIDENCE_LOSS_MS) {
                finish_weight_attempt(now,
                    "Weight evidence lost during alignment; resuming exploration");
            } else if (now - stateStartedMs > ALIGN_MAX_MS) {
                finish_weight_attempt(now,
                    "Weight centering timed out; resuming exploration");
            } else if (fabsf(targetBearingDeg) <= 7.0f) {
                ++weightsSeen;
                set_state(State::WeightApproach,
                          "Weight centered; driving through with drum running");
            }
        } else if (state == State::WeightApproach) {
            if (leftWeightSignature || rightWeightSignature) {
                lastEvidenceMs = now;
            }
            if ((now - stateStartedMs >= APPROACH_MIN_MS &&
                 now - lastEvidenceMs >= WEIGHT_EVIDENCE_LOSS_MS) ||
                now - stateStartedMs >= APPROACH_MAX_MS) {
                finish_weight_attempt(now,
                    "Weight pass complete; resuming exploration");
            }
        } else if (state == State::AvoidTurn) {
            const bool pointEvidence = valid_range("tof_xshut7") ||
                                       valid_range("tof_xshut8");
            const bool freshClearEvidence =
                (newMatrixFrame && haveMatrixFrame) ||
                (pointEvidence && now - lastAvoidEvidenceMs >= RANGE_SAMPLE_PERIOD_MS);
            if (freshClearEvidence) {
                lastAvoidEvidenceMs = now;
                const bool frontClear = frontMm == 0 ||
                                        frontMm >= MATRIX_CLEAR_RANGE_MM;
                if (frontClear) {
                    if (clearVotes < CLEAR_CONFIRM_FRAMES) ++clearVotes;
                } else {
                    clearVotes = 0;
                }
            }
            if (now - stateStartedMs >= AVOID_MIN_TURN_MS &&
                clearVotes >= CLEAR_CONFIRM_FRAMES) {
                finish_avoidance(now);
            } else if (now - stateStartedMs >= AVOID_MAX_TURN_MS) {
                exploration_mode_stop("Stopped: front stayed blocked after avoidance turn");
                return;
            }
        }
    }

    // The actuator watchdogs remain enabled, so refresh both commanded loads
    // while this autonomous state machine owns the robot.
    drum_control_set_percent(DRUM_LEFT_PERCENT, DRUM_RIGHT_PERCENT);
    magnet_control_set(true);
    switch (state) {
        case State::Forward:
        case State::WeightApproach:
            drive_forward();
            break;
        case State::AvoidTurn:
            drive_control_set_percent(turnDirection * TURN_PERCENT,
                                      -turnDirection * TURN_PERCENT);
            break;
        case State::WeightAlign: {
            // One continuous correction is less jerky than stop/start spin
            // pulses. Do not keep turning after the target leaves the view.
            if (now - lastEvidenceMs <= RANGE_SAMPLE_PERIOD_MS * 2) {
                const int direction = targetBearingDeg > 0.0f ? 1 : -1;
                drive_control_set_percent(direction * ALIGN_TURN_PERCENT,
                                          -direction * ALIGN_TURN_PERCENT);
            } else {
                drive_control_stop();
            }
            break;
        }
        default:
            drive_control_stop();
            break;
    }
}

bool exploration_mode_is_active() { return active; }

const char* exploration_mode_state() {
    switch (state) {
        case State::Forward: return "EXPLORING";
        case State::AvoidTurn: return "AVOIDING";
        case State::WeightAlign: return "CENTERING_WEIGHT";
        case State::WeightApproach: return "COLLECTING_WEIGHT";
        default: return "IDLE";
    }
}

const char* exploration_mode_reason() { return reason; }
uint16_t exploration_mode_front_mm() { return frontMm; }
uint16_t exploration_mode_left_mm() { return leftMm; }
uint16_t exploration_mode_right_mm() { return rightMm; }
uint16_t exploration_mode_left_top_mm() { return leftTopMm; }
uint16_t exploration_mode_left_bottom_mm() { return leftBottomMm; }
uint16_t exploration_mode_right_top_mm() { return rightTopMm; }
uint16_t exploration_mode_right_bottom_mm() { return rightBottomMm; }
uint16_t exploration_mode_weights_seen() { return weightsSeen; }
uint16_t exploration_mode_turns() { return turnsTaken; }
float exploration_mode_target_bearing_deg() { return targetBearingDeg; }
