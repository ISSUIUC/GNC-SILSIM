#pragma once
// constants
// Custom constants structure allows us to change the constants we use depending on the board we're using.

#define MIDAS_MINI 1
#define MIDAS 2
#define TELEMEGA 3

#ifndef SENSOR_MODE
#define SENSOR_MODE MIDAS_MINI // change this to MIDAS or TELEMEGA as needed, this is the DEFAULT
#endif

inline constexpr float pi = 3.14159268;
inline constexpr float gravity_ms2 = 9.81; // (m/s^2) accel due to gravity
inline constexpr float s_dt = 0.05f;

#if SENSOR_MODE == MIDAS
inline constexpr float spectral_density_ = 13.0f;

inline constexpr float process_noise_factor = 1e-2f;   // process noise covariance (Q)
inline constexpr float barometer_noise = 0.1f;         // barometer noise (m)
inline constexpr float gps_noise_altitude = 1.5f;      // GPS altitude noise (m)
inline constexpr float gps_noise_east = 0.3f;          // GPS east noise (deg)
inline constexpr float gps_noise_north = 0.3f;         // GPS north noise (deg)
inline constexpr float accel_bias_x = 0.045f;          // accelerometer bias in x (g), tested on flight hardware
inline constexpr float accel_bias_y = -0.065f;         // accelerometer bias in x (g),  tested on flight hardware
inline constexpr float accel_bias_z = -0.06f;          // accelerometer bias in x (g),  tested on flight hardware
inline constexpr float accel_RMS = 1.6f * 1e-4 * 9.81; // standard deviation of accelerometer noise (originally in mg, converted then to m/s^2) // review

// MQEKF constants

inline constexpr float accel_noise_density_x = 300; // ug/sqrt(hz) from the accelerometer on MIDAS MINI. Assuming Acceleration noise density (high-g) in high-performance mode
inline constexpr float accel_noise_density_y = 300; // ug/sqrt(hz) from the accelerometer on MIDAS MINI
inline constexpr float accel_noise_density_z = 300; // ug/sqrt(hz) from the accelerometer on MIDAS MINI
inline constexpr float gyro_RMS_noise = 3.8;        // mdps/√Hz CHECK AGAIN
inline constexpr float mag_noise = 1.4f * 1e-5;     // originally in uT, converted then to mG

inline constexpr float Pq0 = 1e-6;
inline constexpr float Pb0 = 1e-4;
#elif SENSOR_MODE == MIDAS_MINI
inline constexpr float spectral_density_ = 13.0f;

inline constexpr float process_noise_factor = 1e-2f;   // process noise covariance (Q)
inline constexpr float barometer_noise = 0.1f;         // barometer noise (m)
inline constexpr float gps_noise_altitude = 1.5f;      // GPS altitude noise (m)
inline constexpr float gps_noise_east = 0.3f;          // GPS east noise (deg)
inline constexpr float gps_noise_north = 0.3f;         // GPS north noise (deg)
inline constexpr float accel_bias_x = 0.0f;            // accelerometer bias in x (g),  already calibrated
inline constexpr float accel_bias_y = 0.0f;            // accelerometer bias in x (g),   already calibrated
inline constexpr float accel_bias_z = 0.0f;            // accelerometer bias in x (g),   already calibrated
inline constexpr float accel_RMS = 2.1f * 1e-4 * 9.81; // standard deviation of accelerometer noise (originally in mg, converted then to m/s^2) // review

// MQEKF constants

inline constexpr float accel_noise_density_x = 1000; // ug/sqrt(hz) from the accelerometer on MIDAS MINI. Assuming Acceleration noise density (high-g) in high-performance mode
inline constexpr float accel_noise_density_y = 1000; // ug/sqrt(hz) from the accelerometer on MIDAS MINI
inline constexpr float accel_noise_density_z = 1000; // ug/sqrt(hz) from the accelerometer on MIDAS MINI
inline constexpr float gyro_RMS_noise = 3.8;         // mdps/√Hz
inline constexpr float mag_noise = 0.6;              // mG

inline constexpr float Pq0 = 1e-6;
inline constexpr float Pb0 = 1e-4;
#elif SENSOR_MODE == TELEMEGA
inline constexpr spectral_density_ = 13.0f;

inline constexpr float process_noise_factor = 1e-2f;   // process noise covariance (Q)
inline constexpr float barometer_noise = 0.2f;         // barometer noise (m)
inline constexpr float gps_noise_altitude = 1.5f;      // GPS altitude noise (m)
inline constexpr float gps_noise_east = 0.3f;          // GPS east noise (deg)
inline constexpr float gps_noise_north = 0.3f;         // GPS north noise (deg)
inline constexpr float accel_bias_x = 0.0f;            // filtered
inline constexpr float accel_bias_y = 0.0f;            // filtered
inline constexpr float accel_bias_z = 0.0f;            // filtered
inline constexpr float accel_RMS = 1.6f * 1e-4 * 9.81; // standard deviation of accelerometer noise (originally in mg, converted then to m/s^2) // review

// MQEKF constants

inline constexpr float accel_noise_density_x = 5000;               // ug/sqrt(hz) from the accelerometer on MIDAS MINI. Assuming Acceleration noise density (high-g) in high-performance mode
inline constexpr float accel_noise_density_y = 5000;               // ug/sqrt(hz) from the accelerometer on MIDAS MINI
inline constexpr float accel_noise_density_z = 5000;               // ug/sqrt(hz) from the accelerometer on MIDAS MINI
inline constexpr float gyro_RMS_noise = 0.004 * 1000 / sqrt(s_dt); // mdps/√Hz CHECK AGAIN
inline constexpr float mag_noise = 1.4f * 1e-5;                    // originally in uT, converted then to mG

inline constexpr float Pq0 = 1e-6; // review
inline constexpr float Pb0 = 1e-4;
#else
#error "Unknown SENSOR_MODE"

#endif
