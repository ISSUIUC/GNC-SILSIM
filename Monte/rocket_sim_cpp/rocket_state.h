#pragma once
#include <array>

// Vec3 is the single canonical alias for a 3-component double array.
// All other headers include this file and must NOT redeclare Vec3.
using Vec3 = std::array<double, 3>;

/**
 * Full 6-DOF state of the rocket.
 *
 * Maps to the Python x_state[4x3] layout:
 *   x_state[0] -> position          (world frame, +x up)
 *   x_state[1] -> velocity          (world frame)
 *   x_state[2] -> acceleration      (world frame)
 *   x_state[3] -> orientation       (Euler angles: roll, pitch, yaw) [rad]
 *   x_state[4] -> angular_velocity  (world frame) [rad/s]
 *   x_state[5] -> angular_acceleration (world frame) [rad/s^2]
 */
struct RocketState {
    Vec3 position            = {0, 0, 0};
    Vec3 velocity            = {0, 0, 0};
    Vec3 acceleration        = {0, 0, 0};
    Vec3 orientation         = {0, 0, 0}; // Euler angles [rad]
    Vec3 angular_velocity    = {0, 0, 0};
    Vec3 angular_acceleration= {0, 0, 0};
};
