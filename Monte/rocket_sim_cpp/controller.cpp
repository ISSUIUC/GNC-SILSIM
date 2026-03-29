#include "controller.h"
#include <algorithm>
#include <cmath>

Controller::Controller(double Kp, double Ki, double Kd,
                       double timestep, double des_apogee,
                       double max_ext_length, double max_ext_spd)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd),
      dt_(timestep),
      des_apogee_(des_apogee),
      error_sum_(0.0),
      error_prev_(0.0),
      prev_flap_(0.0),
      max_ext_length_(max_ext_length),
      max_ext_spd_(max_ext_spd)
{}

double Controller::get_flap_extension(bool control, double pred_apogee) {
    if (!control) {
        return 0.0;
    }

    // --- PID update ---
    double error_curr = pred_apogee - des_apogee_;
    error_sum_ += error_curr * dt_;
    double error_dt = (error_curr - error_prev_) / dt_;
    double flap_ext = Kp_ * error_curr + Ki_ * error_sum_ + Kd_ * error_dt;
    error_prev_ = error_curr;

    // --- Rate limit: cap how fast the flap can move per timestep ---
    // sign(flap_ext - prev_flap) * min(|delta/dt|, max_spd) * dt
    double delta = flap_ext - prev_flap_;
    double sign = (delta > 0.0) ? 1.0 : (delta < 0.0 ? -1.0 : 0.0);
    double rate_limited_step = sign * std::min(std::abs(delta / dt_), max_ext_spd_) * dt_;
    flap_ext = prev_flap_ + rate_limited_step;

    // --- Hard clamp to physical limits ---
    flap_ext = std::clamp(flap_ext, 0.0, max_ext_length_);

    prev_flap_ = flap_ext;
    return flap_ext;
}
