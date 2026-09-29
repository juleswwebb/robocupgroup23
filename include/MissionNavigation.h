#ifndef MISSION_NAVIGATION_H_
#define MISSION_NAVIGATION_H_

#include <stdint.h>

// The desktop plans; the Teensy owns execution after an explicit start.
// Upload is transactional: begin -> zero or more obstacles/waypoint chunks ->
// commit. A partial upload can never be started.
constexpr uint8_t MISSION_MAX_WAYPOINTS = 64;
constexpr uint8_t MISSION_MAX_OBSTACLES = 24;

void mission_init();
bool mission_begin(uint8_t count, float startX, float startY, float headingDeg,
                   float robotRadiusMm, float marginMm,
                   float encoder0MmPerCount, float encoder1MmPerCount,
                   bool encoder0Reversed, bool encoder1Reversed);
bool mission_set_sensor(const char* key, float rightMm, float forwardMm,
                        float angleDeg, float heightMm, bool enabled,
                        float matrixFovDeg, bool matrixMirrored);
bool mission_add_obstacle(uint8_t index, float x, float y, float width,
                          float depth, float angleDeg, bool circular);
bool mission_add_waypoint(uint8_t index, float x, float y, uint8_t flags);
bool mission_commit();
bool mission_start();
bool mission_resume();
void mission_stop(const char* reason);
void mission_update();
bool mission_is_ready();
bool mission_is_active();
const char* mission_state();
const char* mission_reason();
uint8_t mission_waypoint_index();
uint8_t mission_waypoint_count();
float mission_x_mm();
float mission_y_mm();
float mission_heading_deg();
uint16_t mission_front_mm();
uint16_t mission_left_mm();
uint16_t mission_right_mm();
int16_t mission_nearest_edge_clearance_mm();
int16_t mission_front_edge_clearance_mm();
int16_t mission_left_edge_clearance_mm();
int16_t mission_right_edge_clearance_mm();
const char* mission_nearest_sensor();
uint8_t mission_detour_count();

#endif
