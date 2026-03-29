#pragma once

#include <string>
#include <vector>
#include <array>

/**
 * Represents a rocket motor.
 * Reads a thrust-curve CSV and provides thrust, mass, and alignment over time.
 *
 * Expected CSV columns: "Time (s)", "Thrust (N)", "Theta", "Phi"
 */
class Motor {
public:
    /**
     * @param rocket_total_mass  Initial total rocket mass (dry + motor) [kg]
     * @param cm                 Combined centre-of-mass [m from nose]
     * @param cm_rocket          Rocket body CoM [m from nose]
     * @param cm_motor           Motor CoM [m from nose]
     * @param rocket_dry_mass    Rocket dry mass (no propellant) [kg]
     * @param impulse            Total impulse [Ns]
     * @param mass               Total propellant mass [kg]
     * @param delay              Ignition delay from t=0 [s]
     * @param lookup_file        Path to thrust-curve CSV
     */
    Motor(double rocket_total_mass,
          double cm,
          double cm_rocket,
          double cm_motor,
          double rocket_dry_mass,
          double impulse,
          double mass,
          double delay,
          const std::string& lookup_file);

    /** Records the simulation time at which the motor ignites. */
    void ignite(double start_time);

    /**
     * Returns the thrust vector in the body frame [N].
     * Uses linear interpolation over the thrust curve; zero outside burn window.
     */
    std::array<double, 3> get_thrust(double time_stamp);

    /**
     * Returns the current motor propellant mass [kg].
     * Linearly decreases from total_mass to 0 over the burn.
     */
    double get_mass(double time_stamp);

    /** Returns true once the burn time has elapsed. */
    bool burnout(double time_stamp) const;

    /** Returns the current [theta, phi] alignment angles [deg]. */
    std::array<double, 2> get_alignment() const;

    /** Total burn time from the thrust curve [s]. */
    double get_burn_time() const;

    void set_coast_time(double coast_time);

    // Public so Forces / Rocket can read them directly (mirrors Python attribute access)
    double total_mass;
    double current_mass;
    double rocket_total_mass;
    double rocket_dry_mass;
    double cm;
    double cm_rocket;
    double cm_motor;

private:
    struct ThrustPoint {
        double time;
        double thrust;
        double theta;
        double phi;
    };

    std::vector<ThrustPoint> thrust_data_;

    double total_impulse_;
    double coast_time_;
    double start_time_;
    std::array<double, 2> alignment_;
    std::size_t cur_line_;

    static double lerp(double x1, double x2, double y1, double y2, double x);
    void set_alignment();
    void load_csv(const std::string& path);
};
