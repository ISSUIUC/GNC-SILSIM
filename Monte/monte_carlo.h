#pragma once

#include "rocket_sim_cpp/rocket_state.h"

#include <array>
#include <string>

void simulator(RocketState& x0,
               double dt,
               int sample_number,
               const std::string& run_folder,
               int target_size,
               bool nominal = false,
               float wind_direction_variance_mean = 0.0f,
               float wind_direction_variance_stddev = 0.01f,
               float wind_magnitude_variance_mean = 0.0f,
               float wind_magnitude_variance_stddev = 0.5f,
               bool enable_direction_variance = true,
               bool enable_magnitude_variance = true,
               std::array<float, 3> nominal_wind_direction = {-1.0f, 0.0f, 0.0f},
               float nominal_wind_magnitude = 0.0f);

void run(RocketState& x0,
         double dt,
         int samples,
         const std::string& run_folder,
         int target_size,
         bool clear_contents = false,
         float wind_direction_variance_mean = 0.0f,
         float wind_direction_variance_stddev = 0.01f,
         float wind_magnitude_variance_mean = 0.0f,
         float wind_magnitude_variance_stddev = 0.5f,
         bool enable_direction_variance = true,
         bool enable_magnitude_variance = true,
         std::array<float, 3> nominal_wind_direction = {-1.0f, 0.0f, 0.0f},
         float nominal_wind_magnitude = 0.0f);
