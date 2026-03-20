#pragma once
// constants
const float pi = 3.14159268;
const float gravity_ms2 = 9.81;           // (m/s^2) accel due to gravity
const float process_noise_factor = 1e-2f;     // process noise covariance (Q)
const float barometer_noise = 0.1f;  // barometer noise (m)
const float gps_noise_altitude=  1.5f; // GPS altitude noise (m)
const float gps_noise_east = 0.3f; // GPS east noise (deg)
const float gps_noise_north = 0.3f; // GPS north noise (deg)
const float accel_bias_x = 0.045f; // accelerometer bias in x (g), tested on flight hardware
const float accel_bias_y = -0.065f; // accelerometer bias in x (g),  tested on flight hardware
const float accel_bias_z = -0.06f; // accelerometer bias in x (g),  tested on flight hardware
const float sigma_a = 0.2f; // standard deviation of accelerometer noise (m/s^2)
