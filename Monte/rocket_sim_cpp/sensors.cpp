#include "sensors.h"
#include "vectors.h"
#include "properties.h"
#include "math.h"

#include <random>
#include <cmath>
#include <array>


namespace sensors {

// Single shared random engine seeded at program startup
static std::mt19937 rng(std::random_device{}());

static double gauss(double mean, double stddev) {
    std::normal_distribution<double> dist(mean, stddev);
    return dist(rng);
}

Vec3 get_accelerometer_data(const RocketState& x_state, const SensorConfig& sensor_config) {
    // Gravity magnitude at current altitude (world +x is up)
    double g_mag = (prop::G * prop::m_e) / std::pow(prop::r_e + x_state.position[0], 2);
    Vec3 gravity = {g_mag, 0.0, 0.0};

    // Net specific force in world frame: acceleration + gravity (subtracts free-fall)
    Vec3 accel_world = {
        x_state.acceleration[0] + gravity[0],
        x_state.acceleration[1] + gravity[1],
        x_state.acceleration[2] + gravity[2]
    };

    Vec3 body_accel = vct::world_to_body(x_state.orientation, accel_world);

    // RMS is in mg (milli-g), convert to m/s^2
    double kx134_rms = sensor_config.high_g.RMS * 9.81 / 1000.0;

    return Vec3{
        gauss(body_accel[0], kx134_rms),
        gauss(body_accel[1], kx134_rms),
        gauss(body_accel[2], kx134_rms)
    };
}

Vec3 get_gyro_data(const RocketState& x_state, const SensorConfig& sensor_config) {
    Vec3 body_ang_vel = vct::world_to_body(x_state.orientation, x_state.angular_velocity);

    // RMS is in mdeg/s, convert to rad/s
    double gyro_rms = sensor_config.gyro.RMS * M_PI / 180000.0;

    return Vec3{
        gauss(body_ang_vel[0], gyro_rms),
        gauss(body_ang_vel[1], gyro_rms),
        gauss(body_ang_vel[2], gyro_rms)
    };
}

double get_barometer_data(const RocketState& x_state, const SensorConfig& sensor_config, const Atmosphere& atm) {
    // RMS in hPa, convert to Pa
    double baro_rms = sensor_config.barometer.RMS;

    double pressure = atm.get_pressure(x_state.position[0]);
    pressure = gauss(pressure, baro_rms);
    return atm.get_altitude(pressure);
}

Vec3 get_bno_orientation(const RocketState& x_state, const SensorConfig& sensor_config) {
    const Vec3& orientation = x_state.orientation;

    // error in degrees, convert to radians
    double bno_error = sensor_config.bno.error * M_PI / 180.0;

    return Vec3{
        gauss(orientation[0], bno_error),
        gauss(orientation[1], bno_error),
        gauss(orientation[2], bno_error)
    };
}

} // namespace sensors
