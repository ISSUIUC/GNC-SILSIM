#include "forces.h"
#include "vectors.h"
#include "properties.h"
#include "rocket_state.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <tuple>
#include <iostream>

// CSV loader
void Forces::load_rasaero_csv(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Forces: cannot open RASAero CSV: " + path);
    }

    std::string line;
    std::getline(file, line); // skip header

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string tok;

        RasaeroRow row{};
        std::getline(ss, tok, ','); row.mach            = std::stod(tok);
        std::getline(ss, tok, ','); row.alpha_deg        = std::stod(tok);
        std::getline(ss, tok, ','); row.protuberance_pct = std::stod(tok);
        std::getline(ss, tok, ','); row.ca_power_on      = std::stod(tok);
        std::getline(ss, tok, ','); row.ca_power_off     = std::stod(tok);
        std::getline(ss, tok, ','); row.cn_total         = std::stod(tok);
        std::getline(ss, tok, ','); row.cp_total         = std::stod(tok) / 100.0; // cm -> m

        rasaero_.push_back(row);
    }
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
Forces::Forces(double max_ext_length,
               double cm,
               double cp_init,
               double A,
               double A_s,
               double rocket_dry_mass,
               Motor*  motor,
               const std::string& rasaero_lookup_file,
               Atmosphere* atm)
    : cp{cp_init, 0.0, 0.0},
      max_ext_length_(max_ext_length),
      cm_(cm),
      A_(A),
      A_s_(A_s),
      rocket_dry_mass_(rocket_dry_mass),
      motor_(motor),
      atm_(atm)
{
    load_rasaero_csv(rasaero_lookup_file);
}


// get_force
std::pair<std::array<Vec3, 2>, double>
Forces::get_force(const RocketState& x_state,
                  double flap_ext,
                  double time_stamp,
                  double ejection_force,
                  double theta,
                  double phi,
                  bool density_noise)
{
    double alt     = x_state.position[0];
    double density = atm_->get_density(alt, density_noise, x_state.position);
    Vec3   thrust  = motor_->get_thrust(time_stamp);

    Vec3 wind_vector = atm_->get_wind_vector(time_stamp);
    double alpha     = get_alpha(x_state, wind_vector);

    // before_burnout when thrust magnitude > 0
    double thrust_mag_sq = thrust[0]*thrust[0] + thrust[1]*thrust[1] + thrust[2]*thrust[2];
    bool before_burnout  = thrust_mag_sq > 0.0;

    Vec3 drag  = aerodynamic_force(x_state, density, wind_vector, alpha, before_burnout, flap_ext);
    Vec3 grav  = gravitational_force(alt, time_stamp);

    // Thrust + drag are in body frame; convert to world, then add gravity
    Vec3 thrust_plus_drag = {thrust[0] + drag[0], thrust[1] + drag[1], thrust[2] + drag[2]};
    Vec3 force_world = vct::body_to_world(x_state.orientation, thrust_plus_drag);
    Vec3 force = {force_world[0] + grav[0],
                  force_world[1] + grav[1],
                  force_world[2] + grav[2]};

    // Moment: thrust moment about CoM + aerodynamic moment; convert to world
    Vec3 thrust_moment = vct::cross({-cm_, 0.0, 0.0}, thrust); // -cm x thrust
    Vec3 aero_mom      = aerodynamic_moment(drag);
    Vec3 body_moment   = {thrust_moment[0] + aero_mom[0],
                          thrust_moment[1] + aero_mom[1],
                          thrust_moment[2] + aero_mom[2]};
    Vec3 moment = vct::body_to_world(x_state.orientation, body_moment);

    // Optional ejection impulse
    if (ejection_force != 0.0) {
        Vec3 dir = {std::cos(phi),
                    std::sin(phi) * std::sin(theta),
                    std::sin(phi) * std::cos(theta)};
        for (int i = 0; i < 3; ++i) {
            force[i]  += ejection_force * dir[i];
            moment[i] += ejection_force * cm_ * dir[i];
        }
    }

    return {{force, moment}, alpha};
}

// ---------------------------------------------------------------------------
// get_Ca_Cn_Cp
// ---------------------------------------------------------------------------
std::tuple<double, double, Vec3>
Forces::get_Ca_Cn_Cp(const RocketState& x_state,
                     double alpha,
                     bool before_burnout,
                     double flap_ext)
{
    double alt  = x_state.position[0];
    double speed = std::sqrt(x_state.velocity[0]*x_state.velocity[0] +
                             x_state.velocity[1]*x_state.velocity[1] +
                             x_state.velocity[2]*x_state.velocity[2]);
    double mach = speed / atm_->get_speed_of_sound(alt);

    // Round to 2 decimal places to match CSV lookup
    double mach_rounded = std::round(mach * 100.0) / 100.0;
    if (mach_rounded < 0.01) mach_rounded = 0.01;  // clamp to table minimum
    double protub_perc  = flap_ext / max_ext_length_;

    int alpha_deg = static_cast<int>(std::round(alpha * 180.0 / M_PI));

    // Find matching rows for this mach number
    // Iterate pairs: if alpha matches and protuberance bracket straddles current value, interpolate
    for (std::size_t i = 0; i + 1 < rasaero_.size(); ++i) {
       
        const auto& lo = rasaero_[i];
        const auto& hi = rasaero_[i + 1];

        if (lo.mach != mach_rounded) continue;
        //std::cout << "mach_rounded=" << mach_rounded << " lo.mach=" << lo.mach << " hi.mach=" << hi.mach << "\n";
        //if (static_cast<int>(std::round(lo.alpha_deg)) != alpha_deg) continue;
        //if (!(lo.protuberance_pct <= protub_perc && protub_perc <= hi.protuberance_pct)) continue;

        double t = (protub_perc - lo.protuberance_pct) /
                   (hi.protuberance_pct - lo.protuberance_pct);

        double ca_lo = before_burnout ? lo.ca_power_on : lo.ca_power_off;
        double ca_hi = before_burnout ? hi.ca_power_on : hi.ca_power_off;

        double Ca = ca_lo + t * (ca_hi - ca_lo);
        double Cn = lo.cn_total + t * (hi.cn_total - lo.cn_total);
        double Cp = lo.cp_total + t * (hi.cp_total - lo.cp_total); // already in metres
       
        return {Ca, Cn, Vec3{Cp, 0.0, 0.0}};
    }

     
   
    return {0.0, 0.0, Vec3{cp[0], 0.0, 0.0}};  // preserve last known cp so moment doesn't blow up
}

// ---------------------------------------------------------------------------
// aerodynamic_force
// ---------------------------------------------------------------------------
Vec3 Forces::aerodynamic_force(const RocketState& x_state,
                                double density,
                                const Vec3& wind_vector,
                                double alpha,
                                bool before_burnout,
                                double flap_ext)
{
    // Relative velocity in body frame
    Vec3 vel_rel_world = {x_state.velocity[0] - wind_vector[0],
                          x_state.velocity[1] - wind_vector[1],
                          x_state.velocity[2] - wind_vector[2]};
    Vec3 vel = vct::world_to_body(x_state.orientation, vel_rel_world);

    auto [C_a, C_n, new_cp] = get_Ca_Cn_Cp(x_state, alpha, before_burnout, flap_ext);
    cp = new_cp;

    double roll_aero = std::atan2(x_state.velocity[2], x_state.velocity[1]);
    double C_n_y = std::abs(C_n * std::cos(roll_aero));
    double C_n_z = std::abs(C_n * std::sin(roll_aero));

    // Signed-square velocity components preserve direction
    auto sign_sq = [](double v) { return (v >= 0.0 ? 1.0 : -1.0) * v * v; };

    return Vec3{
        -0.5 * sign_sq(vel[0]) * C_a  * density * A_,
        -0.5 * sign_sq(vel[1]) * C_n_y * density * A_s_,
        -0.5 * sign_sq(vel[2]) * C_n_z * density * A_s_
    };
}

// ---------------------------------------------------------------------------
// gravitational_force
// ---------------------------------------------------------------------------
Vec3 Forces::gravitational_force(double altitude, double time_stamp) {
    double total_mass = rocket_dry_mass_ + motor_->get_mass(time_stamp);
    double F_g = (prop::G * prop::m_e * total_mass) /
                 std::pow(prop::r_e + altitude, 2);
    return Vec3{-F_g, 0.0, 0.0};
}

// ---------------------------------------------------------------------------
// aerodynamic_moment
// ---------------------------------------------------------------------------
Vec3 Forces::aerodynamic_moment(const Vec3& aero_force) {
    // (cp - motor.cm) x aero_force
    Vec3 r = {cp[0] - motor_->cm, cp[1], cp[2]};
    return vct::cross(r, aero_force);
}

// ---------------------------------------------------------------------------
// get_alpha
// ---------------------------------------------------------------------------
double Forces::get_alpha(const RocketState& x_state, const Vec3& wind_vector) {
    // Python: incident_velocity = world_to_body(orientation, norm(vel + wind))
    //         orientation_vec   = world_to_body(orientation, [1,0,0])
    //         alpha = arccos(dot(incident, orientation_vec))
    // Note: world_to_body([1,0,0]) == [1,0,0] in body frame, so dot == incident[0].
    Vec3 v_plus_wind = {x_state.velocity[0] + wind_vector[0],
                        x_state.velocity[1] + wind_vector[1],
                        x_state.velocity[2] + wind_vector[2]};

    double speed = std::sqrt(v_plus_wind[0]*v_plus_wind[0] +
                             v_plus_wind[1]*v_plus_wind[1] +
                             v_plus_wind[2]*v_plus_wind[2]);
    if (speed == 0.0) return 0.0;

    Vec3 v_norm   = {v_plus_wind[0]/speed, v_plus_wind[1]/speed, v_plus_wind[2]/speed};
    Vec3 incident = vct::world_to_body(x_state.orientation, v_norm);

    // Rocket body axis in body frame is always [1,0,0]
    double dot = std::clamp(incident[0], -1.0, 1.0);
    return std::acos(dot);
}
