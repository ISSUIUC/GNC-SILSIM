#include "motor.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <algorithm>

// CSV loader

void Motor::load_csv(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Motor: cannot open thrust-curve CSV: " + path);
    }

    std::string line;
    // Skip header row
    std::getline(file, line);

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string token;

        ThrustPoint pt{};
        std::getline(ss, token, ','); pt.time   = std::stod(token);
        std::getline(ss, token, ','); pt.thrust = std::stod(token);
        std::getline(ss, token, ','); pt.theta  = std::stod(token);
        std::getline(ss, token, ','); pt.phi    = std::stod(token);
        thrust_data_.push_back(pt);
    }

    if (thrust_data_.empty()) {
        throw std::runtime_error("Motor: thrust-curve CSV is empty: " + path);
    }
}

// Constructor
Motor::Motor(double rocket_total_mass_,
             double cm_,
             double cm_rocket_,
             double cm_motor_,
             double rocket_dry_mass_,
             double impulse,
             double mass,
             double delay,
             const std::string& lookup_file)
    : total_mass(mass),
      current_mass(mass),
      rocket_total_mass(rocket_total_mass_),
      rocket_dry_mass(rocket_dry_mass_),
      cm(cm_),
      cm_rocket(cm_rocket_),
      cm_motor(cm_motor_),
      total_impulse_(impulse),
      coast_time_(delay),
      start_time_(delay),
      alignment_{0.0, 0.0},
      cur_line_(0)
{
    load_csv(lookup_file);
}

// Helpers
double Motor::lerp(double x1, double x2, double y1, double y2, double x) {
    return y1 + ((y2 - y1) / (x2 - x1)) * (x - x1);
}

void Motor::set_alignment() {
    if (cur_line_ < thrust_data_.size()) {
        alignment_ = {thrust_data_[cur_line_].theta,
                      thrust_data_[cur_line_].phi};
        ++cur_line_;
    }
}

void Motor::ignite(double start_time) {
    start_time_ = start_time;
}

std::array<double, 3> Motor::get_thrust(double time_stamp) {
    double t = time_stamp - start_time_;

    double magnitude = 0.0;
    const auto& td = thrust_data_;

    if (t >= td.front().time && t <= td.back().time) {
        for (std::size_t i = 0; i + 1 < td.size(); ++i) {
            if (td[i].time == t) {
                magnitude = td[i].thrust;
                break;
            }
            if (td[i].time < t && t < td[i + 1].time) {
                magnitude = lerp(td[i].time, td[i + 1].time,
                                 td[i].thrust, td[i + 1].thrust, t);
                break;
            }
        }
    }

    set_alignment();

    double theta = alignment_[0] * M_PI / 180.0;
    double phi   = alignment_[1] * M_PI / 180.0;

    // +x is downrange (into rocket nose), phi=0 is ideal (axial)
    return std::array<double, 3>{
        magnitude * std::cos(phi),
        magnitude * std::sin(phi) * std::sin(theta),
        magnitude * std::sin(phi) * std::cos(theta)
    };
}

double Motor::get_mass(double time_stamp) {
    double shifted = time_stamp - start_time_;

    if (shifted < thrust_data_.front().time) {
        return total_mass;
    }

    if (burnout(time_stamp)) {
        current_mass = 0.0;
    } else {
        current_mass = lerp(thrust_data_.front().time,
                            thrust_data_.back().time,
                            total_mass, 0.0,
                            shifted);
    }

    // Update combined CoM based on remaining propellant
    rocket_total_mass = rocket_dry_mass + current_mass;
    if (rocket_total_mass > 0.0) {
        cm = (cm_rocket * rocket_dry_mass + cm_motor * current_mass) / rocket_total_mass;
    }

    return current_mass;
}

bool Motor::burnout(double time_stamp) const {
    return (time_stamp - start_time_) >= thrust_data_.back().time;
}

std::array<double, 2> Motor::get_alignment() const {
    return alignment_;
}

double Motor::get_burn_time() const {
    return thrust_data_.back().time;
}

void Motor::set_coast_time(double coast_time) {
    coast_time_ = coast_time;
}
