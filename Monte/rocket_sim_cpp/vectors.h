#pragma once
#include "rocket_state.h"
#include <cmath>

/**
 * Vector utilities.
 * ZYX Euler-angle convention (roll=phi, pitch=theta, yaw=psi).
 */
namespace vct {

/**
 * Rotate a vector from the world frame into the rocket body frame.
 * @param euler  {roll, pitch, yaw} in radians
 * @param v      Vector in world frame
 */
inline Vec3 world_to_body(const Vec3& euler, const Vec3& v) {
    double phi   = euler[0]; // roll
    double theta = euler[1]; // pitch
    double psi   = euler[2]; // yaw

    double cp = std::cos(phi),   sp = std::sin(phi);
    double ct = std::cos(theta), st = std::sin(theta);
    double cy = std::cos(psi),   sy = std::sin(psi);

    // R = Rx(phi) * Ry(theta) * Rz(psi)
    double R[3][3] = {
        { ct*cy,           ct*sy,          -st     },
        { sp*st*cy - cp*sy, sp*st*sy + cp*cy, sp*ct },
        { cp*st*cy + sp*sy, cp*st*sy - sp*cy, cp*ct }
    };

    return Vec3{
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2]
    };
}

/**
 * Rotate a vector from the rocket body frame into the world frame (transpose of world_to_body).
 */
inline Vec3 body_to_world(const Vec3& euler, const Vec3& v) {
    double phi   = euler[0];
    double theta = euler[1];
    double psi   = euler[2];

    double cp = std::cos(phi),   sp = std::sin(phi);
    double ct = std::cos(theta), st = std::sin(theta);
    double cy = std::cos(psi),   sy = std::sin(psi);

    // R^T
    double R[3][3] = {
        { ct*cy, sp*st*cy - cp*sy, cp*st*cy + sp*sy },
        { ct*sy, sp*st*sy + cp*cy, cp*st*sy - sp*cy },
        { -st,   sp*ct,            cp*ct            }
    };

    return Vec3{
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2]
    };
}

/** Cross product a × b */
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return Vec3{
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    };
}

/** Return unit vector (zero vector stays zero) */
inline Vec3 normalized(const Vec3& v) {
    double n = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (n == 0.0) return {0.0, 0.0, 0.0};
    return {v[0]/n, v[1]/n, v[2]/n};
}

} // namespace vct
