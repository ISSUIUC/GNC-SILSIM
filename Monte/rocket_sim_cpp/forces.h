#pragma once

#include <array>
#include <string>
#include <vector>
#include <tuple>
#include "rocket_state.h"
#include "motor.h"
#include "atmosphere.h"

/**
 * Encapsulates all forces and moments acting on the rocket:
 *   - Gravity
 *   - Motor thrust
 *   - Aerodynamic drag / normal force (via RASAero lookup table)
 */
class Forces {
public:
    Forces(const Forces&) = default;
    Forces& operator=(const Forces&) = default;

    /**
     * @param max_ext_length        Max flap extension [m]
     * @param cm                    Centre-of-mass position (axial) [m from nose]
     * @param cp                    Centre-of-pressure position (axial) [m from nose]
     * @param A                     Cross-sectional area [m^2]
     * @param A_s                   Side (lateral) area [m^2]
     * @param rocket_dry_mass       Dry mass of rocket [kg]
     * @param motor                 Pointer to the Motor object (shared ownership)
     * @param rasaero_lookup_file   Path to RASAero CSV lookup table
     * @param atm                   Pointer to Atmosphere object (shared ownership)
     */
    Forces(double max_ext_length,
           double cm,
           double cp,
           double A,
           double A_s,
           double rocket_dry_mass,
           Motor* motor,
           const std::string& rasaero_lookup_file,
           Atmosphere* atm);

    /**
     * Computes the net force and moment vectors acting on the rocket.
     *
     * @param x_state        Current rocket state
     * @param flap_ext       Current flap extension [m]
     * @param time_stamp     Simulation time [s]
     * @param ejection_force Pyro ejection force magnitude (0 if none) [N]
     * @param theta          Ejection direction angle theta [rad]
     * @param phi            Ejection direction angle phi [rad]
     * @param density_noise  Whether to add noise to atmospheric density
     * @return               {force[3], moment[3]}, angle of attack [rad]
     */
    std::pair<std::array<Vec3, 2>, double>
    get_force(const RocketState& x_state,
              double flap_ext,
              double time_stamp,
              double ejection_force,
              double theta,
              double phi,
              bool density_noise = false);

    /** Returns [Ca, Cn, Cp] from the RASAero lookup table via interpolation. */
    std::tuple<double, double, Vec3>
    get_Ca_Cn_Cp(const RocketState& x_state,
                 double alpha,
                 bool before_burnout,
                 double flap_ext);

    /** Aerodynamic force vector in body frame [N]. */
    Vec3 aerodynamic_force(const RocketState& x_state,
                           double density,
                           const Vec3& wind_vector,
                           double alpha,
                           bool before_burnout,
                           double flap_ext);

    /** Gravitational force vector in world frame [N]. */
    Vec3 gravitational_force(double altitude, double time_stamp);

    /** Aerodynamic moment vector in body frame [Nm]. */
    Vec3 aerodynamic_moment(const Vec3& aero_force);

    /** Angle of attack between the velocity vector and rocket axis [rad]. */
    double get_alpha(const RocketState& x_state, const Vec3& wind_vector);

    // Exposed so Rocket can read the latest cp after a get_force call
    Vec3 cp;

private:
    double max_ext_length_;
    double cm_;
    double A_;
    double A_s_;
    double rocket_dry_mass_;
    Motor*      motor_;
    Atmosphere* atm_;

    // RASAero lookup table rows
    struct RasaeroRow {
        double mach;
        double alpha_deg;
        double protuberance_pct;
        double ca_power_on;
        double ca_power_off;
        double cn_total;
        double cp_total;   // in cm — converted to m on load
    };
    std::vector<RasaeroRow> rasaero_;

    void load_rasaero_csv(const std::string& path);
};
