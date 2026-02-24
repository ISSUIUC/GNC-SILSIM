#pragma once

#include <array>
#include <cmath>
#include <random>
#include <cstdio>
#include "perlin.hpp"
// #include "vectors.hpp"
// #include "random_noise.hpp"

class Atmosphere {
public:
    Atmosphere(
        double wind_direction_variance_mean       = 0.0,
        double wind_direction_variance_stddev     = 0.01,
        double wind_magnitude_variance_mean       = 0.0,
        double wind_magnitude_variance_stddev     = 0.5,
        bool   enable_direction_variance          = false,
        bool   enable_magnitude_variance          = false,
        std::array<double, 3> nominal_wind_direction = {-1.0, 0.0, 0.0},
        double nominal_wind_magnitude             = 0.0
    )
        : wind_direction_variance_mean_(wind_direction_variance_mean)
        , wind_direction_variance_stddev_(wind_direction_variance_stddev)
        , wind_magnitude_variance_mean_(wind_magnitude_variance_mean)
        , wind_magnitude_variance_stddev_(wind_magnitude_variance_stddev)
        , enable_direction_variance_(enable_direction_variance)
        , enable_magnitude_variance_(enable_magnitude_variance)
        , nominal_wind_direction_(nominal_wind_direction)
        , nominal_wind_magnitude_(nominal_wind_magnitude)
        , last_direction_variance_update_(-100.0)
        , last_magnitude_variance_update_(-100.0)
        , direction_variance_update_rate_(1.0)
        , magnitude_variance_update_rate_(1.0)
        , direction_variance_vect_({0.0, 0.0, 0.0})
        , magnitude_variance_val_(0.0)
        , rng_(std::random_device{}())
    {}

    double get_geometric_to_geopotential(double altitude) {
        double r = 6371000.0;
        return (r * altitude) / (r + altitude);
    }

    double get_temperature(double altitude) {
        double altitude_h = get_geometric_to_geopotential(altitude) / 1000.0;
        double altitude_z = altitude / 1000.0;
        double temperature;

        if (altitude_h < 11.0) {
            temperature = 288.15 - (6.5 * altitude_h);
        } else if (altitude_h < 20.0) {
            temperature = 216.65;
        } else if (altitude_h < 32.0) {
            temperature = 196.65 + altitude_h;
        } else if (altitude_h < 47.0) {
            temperature = 139.05 + (2.8 * altitude_h);
        } else if (altitude_h < 51.0) {
            temperature = 270.65;
        } else if (altitude_h < 71.0) {
            temperature = 413.45 - (2.8 * altitude_h);
        } else if (altitude_h < 84.852) {
            temperature = 356.65 - (2.0 * altitude_h);
        } else if (altitude_z < 91.0) {
            temperature = 186.8673;
        } else if (altitude_z < 110.0) {
            temperature = 263.1905 - 76.3232 * std::sqrt(1.0 - std::pow((altitude_z - 91.0) / -19.9429, 2.0));
        } else if (altitude_z < 120.0) {
            temperature = 240.0 + 12.0 * (altitude_z - 110.0);
        } else if (altitude_z < 1000.0) {
            temperature = 1000.0 - 640.0 * std::exp(-0.01875 * ((altitude_z - 120.0) * (6356.766 + 120.0) / (6356.766 + altitude_z)));
        } else {
            printf("Exceeding calculatable altitude!\n");
            temperature = -1.0;
        }
        return temperature;
    }

    double get_pressure(double altitude) {
        double P_0 = 101325.0;
        double T_0 = 288.16;
        double b   = 0.0065;
        double g   = 9.81;
        double R   = 287.05;
        return P_0 * std::pow((T_0 + altitude * b) / T_0, -g / (b * R));
    }

  

  void normalize_vector_inplace(std::array<double, 3>& arr) {
    double sum_sq = 0.0;
    for (double val : arr) {
        sum_sq += val * val;
    }


    // if (sum_sq == 0.0) {
    //     throw std::logic_error("Cannot normalize a zero vector");
    // }
    double magnitude = std::sqrt(sum_sq);

    for (double& val : arr) {
        val /= magnitude;
    }
}


    double get_density(double altitude, bool noise = false, std::array<double, 3> position = {0.0, 0.0, 0.0}) {
        double R           = 287.053;
        double pressure    = get_pressure(altitude);
        double temperature = get_temperature(altitude);
        altitude = altitude / 1000.0;
        double density;

        if (altitude < 84.853) {
            density = pressure / (R * temperature);
        } else if (altitude < 91.0) {
            density = std::exp(0.000000 * std::pow(altitude, 4) + -3.322622e-6 * std::pow(altitude, 3) + 9.111460e-4 * std::pow(altitude, 2) + -0.2609971 * altitude + 5.944694);
        } else if (altitude < 100.0) {
            density = std::exp(0.000000 * std::pow(altitude, 4) + 2.873405e-5 * std::pow(altitude, 3) + -0.008492037 * std::pow(altitude, 2) + 0.6541179 * altitude + -23.62010);
        } else if (altitude < 110.0) {
            density = std::exp(-1.240774e-5 * std::pow(altitude, 4) + 0.005162063 * std::pow(altitude, 3) + -0.8048342 * std::pow(altitude, 2) + 55.55996 * altitude + -1443.338);
        } else if (altitude < 120.0) {
            density = std::exp(0.00000 * std::pow(altitude, 4) + -8.854164e-5 * std::pow(altitude, 3) + 0.03373254 * std::pow(altitude, 2) + -4.390837 * altitude + 176.5294);
        } else if (altitude < 150.0) {
            density = std::exp(3.661771e-7 * std::pow(altitude, 4) + -2.154344e-4 * std::pow(altitude, 3) + 0.04809214 * std::pow(altitude, 2) + -4.884744 * altitude + 172.3597);
        } else if (altitude < 200.0) {
            density = std::exp(1.906032e-8 * std::pow(altitude, 4) + -1.527799e-5 * std::pow(altitude, 3) + 0.004724294 * std::pow(altitude, 2) + -0.6992340 * altitude + 20.50921);
        } else if (altitude < 300.0) {
            density = std::exp(1.199282e-9 * std::pow(altitude, 4) + -1.451051e-6 * std::pow(altitude, 3) + 6.910474e-4 * std::pow(altitude, 2) + -0.1736220 * altitude + -5.321644);
        } else if (altitude < 500.0) {
            density = std::exp(1.140564e-10 * std::pow(altitude, 4) + -2.130756e-7 * std::pow(altitude, 3) + 1.570762e-4 * std::pow(altitude, 2) + -0.07029296 * altitude + -12.89844);
        } else if (altitude < 750.0) {
            density = std::exp(8.105631e-12 * std::pow(altitude, 4) + -2.358417e-9 * std::pow(altitude, 3) + -2.635110e-6 * std::pow(altitude, 2) + -0.01562608 * altitude + -20.02246);
        } else if (altitude < 1000.0) {
            density = std::exp(-3.701195e-12 * std::pow(altitude, 4) + -8.608611e-9 * std::pow(altitude, 3) + 5.118829e-5 * std::pow(altitude, 2) + -0.06600998 * altitude + -6.137674);
        } else {
            printf("Exceeding calculatable altitude!\n");
            density = -1.0;
        }

        if (noise) {
            density *= 1.0 + 0.005 * perlin_.f(position[0] / 100.0, position[1] / 100.0, position[2] / 100.0);
        }
        return density;
    }

    double get_altitude(double pressure) {
        double P_0 = 101325.0;
        double T_0 = 288.16;
        double b   = 0.0065;
        double g   = 9.81;
        double R   = 287.05;
        double pressureRatio = pressure / P_0;
        return -(T_0 * (std::pow(pressureRatio, b * R / g) - 1.0) * std::pow(pressureRatio, -b * R / g)) / b;
    }

    double get_speed_of_sound(double altitude) {
        double gamma        = 1.4;
        double gas_constant = 287.05;
        return std::sqrt(gamma * gas_constant * get_temperature(altitude));
    }

    std::array<double, 3> get_nominal_wind_direction() { return nominal_wind_direction_; }
    double get_nominal_wind_magnitude() { return nominal_wind_magnitude_; }

    void set_nominal_wind_direction(std::array<double, 3> direction) {
        normalize_vector_inplace(direction);
        nominal_wind_direction_ = direction;
    }

    void set_nominal_wind_magnitude(double magnitude) {
        nominal_wind_magnitude_ = magnitude;
    }

    void toggle_wind_direction_variance(bool toggle) { enable_direction_variance_ = toggle; }
    void toggle_wind_magnitude_variance(bool toggle) { enable_magnitude_variance_ = toggle; }

    std::array<double, 3> get_wind_vector(double tStamp) {
        double dir_alpha = 0.9997;
        double mag_alpha = 0.99;
        std::array<double, 3> generated_direction_variance = {0.0, 0.0, 0.0};
        double generated_magnitude_variance = 0.0;
        std::array<double, 3> current_wind_direction = nominal_wind_direction_;

        if (enable_direction_variance_) {
            if ((tStamp - last_direction_variance_update_) >= direction_variance_update_rate_) {
                std::normal_distribution<double> dist(0.0, wind_direction_variance_stddev_);
                generated_direction_variance = {
                    nominal_wind_direction_[0] + dist(rng_),
                    nominal_wind_direction_[1] + dist(rng_),
                    nominal_wind_direction_[2] + dist(rng_)
                };
                normalize_vector_inplace(generated_direction_variance);
                last_direction_variance_update_ = tStamp;
            }
            for (int i = 0; i < 3; ++i)
                direction_variance_vect_[i] = dir_alpha * direction_variance_vect_[i] + (1.0 - dir_alpha) * generated_direction_variance[i];
            for (int i = 0; i < 3; ++i)
                current_wind_direction[i] = nominal_wind_direction_[i] + direction_variance_vect_[i];
        }

        normalize_vector_inplace(current_wind_direction);

        double current_wind_magnitude = nominal_wind_magnitude_;

        if (enable_magnitude_variance_) {
            if ((tStamp - last_magnitude_variance_update_) >= magnitude_variance_update_rate_) {
                std::normal_distribution<double> dist(nominal_wind_magnitude_, wind_magnitude_variance_stddev_);
                generated_magnitude_variance = dist(rng_);
                last_magnitude_variance_update_ = tStamp;
            }
            magnitude_variance_val_ = mag_alpha * magnitude_variance_val_ + (1.0 - mag_alpha) * generated_magnitude_variance;
            current_wind_magnitude = nominal_wind_magnitude_ + magnitude_variance_val_;
        }

        std::array<double, 3> wind_vector;
        for (int i = 0; i < 3; ++i)
            wind_vector[i] = current_wind_direction[i] * current_wind_magnitude;
        return wind_vector;
    }

private:
    Perlin perlin_;
    std::mt19937 rng_;

    double wind_direction_variance_mean_;
    double wind_direction_variance_stddev_;
    double wind_magnitude_variance_mean_;
    double wind_magnitude_variance_stddev_;
    bool   enable_direction_variance_;
    bool   enable_magnitude_variance_;

    std::array<double, 3> nominal_wind_direction_;
    double                nominal_wind_magnitude_;

    double last_direction_variance_update_;
    double last_magnitude_variance_update_;
    double direction_variance_update_rate_;
    double magnitude_variance_update_rate_;

    std::array<double, 3> direction_variance_vect_;
    double                magnitude_variance_val_;
};