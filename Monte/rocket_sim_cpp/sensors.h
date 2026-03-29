#pragma once

#include <array>
#include "rocket_state.h"
#include "sensor_config.h"
#include "atmosphere.h"
#include <cmath>
namespace sensors {

using Vec3 = std::array<double, 3>;

/**
 * Returns the accelerometer data (specific force) in the body frame.
 * Noise is sampled from a Gaussian with RMS from sensor_config.
 */
Vec3 get_accelerometer_data(const RocketState& x_state, const SensorConfig& sensor_config);

/**
 * Returns the gyroscope data (angular velocity) in the body frame.
 * Noise is sampled from a Gaussian with RMS from sensor_config.
 */
Vec3 get_gyro_data(const RocketState& x_state, const SensorConfig& sensor_config);

/**
 * Returns the barometric altitude estimate (meters).
 * Pressure is noised then converted back to altitude.
 */
double get_barometer_data(const RocketState& x_state, const SensorConfig& sensor_config, const Atmosphere& atm);

/**
 * Returns the BNO orientation sensor reading (Euler angles, radians) in the world frame.
 * Noise is sampled from a Gaussian with error from sensor_config.
 */
Vec3 get_bno_orientation(const RocketState& x_state, const SensorConfig& sensor_config);

} // namespace sensors
