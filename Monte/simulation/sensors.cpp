#include <random>
#include <cmath>
#include <array>
#include <vector>
#include "properties.h"
#include "atmosphere.cpp"

// Type alias for convenience
using Vec3 = std::array<double, 3>;

// Random number generator (seeded once)
static std::mt19937 rng(std::random_device{}());

/**
 * Returns a sample from a Gaussian distribution.
 */
double gauss(double mean, double stddev) {
    std::normal_distribution<double> dist(mean, stddev);
    return dist(rng);
}

/**
 * Returns the accelerometer data in the body frame of the rocket.
 *
 * @param x_state  The state of the rocket in the world frame.
 * @param sensor_config  Sensor configuration parameters.
 * @return accel_reading  The accelerometer data (specific force) in the body frame (m/s^2).
 */
Vec3 get_accelerometer_data(const RocketState& x_state, const SensorConfig& sensor_config) {
    double gravity_mag = (prop::G * prop::m_e) / std::pow(prop::r_e + x_state.position[0], 2);
    Vec3 gravity = {gravity_mag, 0.0, 0.0};

    Vec3 accel_with_gravity = {
        x_state.acceleration[0] + gravity[0],
        x_state.acceleration[1] + gravity[1],
        x_state.acceleration[2] + gravity[2]
    };

    Vec3 body_accel = vct::world_to_body(x_state.orientation, accel_with_gravity);

    double kx134_rms = sensor_config.high_g.RMS * 9.81 / 1000.0; // Convert mg to m/s^2

    Vec3 accel_reading = {
        gauss(body_accel[0], kx134_rms),
        gauss(body_accel[1], kx134_rms),
        gauss(body_accel[2], kx134_rms)
    };

    return accel_reading;
}

/**
 * Returns the gyroscope data in the body frame of the rocket.
 *
 * @param x_state  The state of the rocket in the world frame.
 * @param sensor_config  Sensor configuration parameters.
 * @return gyro_reading  The gyroscope data (angular velocity) in the body frame (rad/s).
 */
Vec3 get_gyro_data(const RocketState& x_state, const SensorConfig& sensor_config) {
    Vec3 body_angular_vel = vct::world_to_body(x_state.orientation, x_state.angular_velocity);

    double gyro_rms = sensor_config.gyro.RMS * M_PI / 180000.0; // Convert mdeg/s to rad/s

    Vec3 gyro_reading = {
        gauss(body_angular_vel[0], gyro_rms),
        gauss(body_angular_vel[1], gyro_rms),
        gauss(body_angular_vel[2], gyro_rms)
    };

    return gyro_reading;
}

/**
 * Returns the barometric altitude estimate in the world frame.
 *
 * @param x_state  The state of the rocket in the world frame.
 * @param sensor_config  Sensor configuration parameters.
 * @return altitude  The barometric altitude estimate in meters.
 */
double get_barometer_data(const RocketState& x_state, const SensorConfig& sensor_config) {
    atm::Atmosphere atmosphere;

    double baro_rms = sensor_config.barometer.RMS * 100.0; // Convert hPa to Pa

    double pressure = atmosphere.get_pressure(x_state.position[0]);
    pressure = gauss(pressure, baro_rms);
    double altitude = atmosphere.get_altitude(pressure);

    return altitude;
}

/**
 * Returns the emulated BNO orientation sensor data in the world frame.
 *
 * @param x_state  The state of the rocket in the world frame.
 * @param sensor_config  Sensor configuration parameters.
 * @return bno_reading  The emulated orientation (Euler angles) in the world frame (rad).
 */
Vec3 get_bno_orientation(const RocketState& x_state, const SensorConfig& sensor_config) {
    const Vec3& true_orientation = x_state.orientation;

    double bno_error = sensor_config.bno.error * M_PI / 180.0; // Convert deg to rad

    Vec3 bno_reading = {
        gauss(true_orientation[0], bno_error),
        gauss(true_orientation[1], bno_error),
        gauss(true_orientation[2], bno_error)
    };

    return bno_reading;
}