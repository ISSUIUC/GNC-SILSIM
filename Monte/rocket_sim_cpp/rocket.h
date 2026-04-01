#pragma once

#include <array>
#include <string>
#include <vector>
#include <memory>

#include "rocket_state.h"
#include "motor.h"
#include "forces.h"
#include "atmosphere.h"

/**
 * Represents a rocket (or a single stage of a multi-stage rocket).
 *
 * Stores simulation telemetry, sensor readings, Kalman filter output,
 * aerodynamic coefficients, and all physical parameters needed to advance
 * the flight simulation.
 *
 * Multi-stage rockets are modelled by composing Rocket objects:
 *   - The base Rocket holds a list of upper-stage Rocket objects (stages_).
 *   - current_stage_ tracks which stage is active (-1 = base/pre-separation).
 */
class Rocket {
public:
    // -----------------------------------------------------------------------
    // Construction
    // -----------------------------------------------------------------------
    struct Config {
        // Rocket body
        double structure_cm;
        double combined_cm;
        double combined_cp;
        double dry_mass;
        double radius;
        double length;
        std::string rasaero_lookup_file;

        // Motor
        double motor_cm;
        double motor_impulse;
        double motor_mass;
        double motor_delay;
        std::string motor_lookup_file;

        // Flaps
        double flap_max_ext_length;
        double flap_max_ext_spd;
    };

    /**
     * @param dt            Simulation timestep [s]
     * @param x0            Initial state vector
     * @param cfg           Physical / hardware configuration
     * @param atm           Atmosphere model (non-owning pointer)
     * @param stages        Upper-stage Rocket objects (optional)
     * @param coeff_csv     Path to EKF/CD coefficient CSV
     * @param gnc_csv       Path to GNC coefficient CSV
     */
    Rocket(double dt,
           const RocketState& x0,
           const Config& cfg,
           Atmosphere* atm,
           std::vector<std::shared_ptr<Rocket>> stages = {},
           const std::string& coeff_csv = "",
           const std::string& gnc_csv   = "");

    // -----------------------------------------------------------------------
    // Stage management
    // -----------------------------------------------------------------------
    /** Advances to the next stage. Returns false if no more stages exist. */
    bool separate_stage(double timestamp);

    // -----------------------------------------------------------------------
    // Mass / geometry accessors (stage-aware)
    // -----------------------------------------------------------------------
    double get_total_motor_mass(double timestamp) const;
    void   set_motor_mass(double timestamp);
    bool   is_motor_burnout(double timestamp) const;
    double get_motor_mass(double timestamp);
    double get_rocket_dry_mass() const;
    double get_rocket_total_mass(double timestamp);
    double get_CM() const;
    double get_CP() const;
    double get_A()  const;
    double get_A_s() const;
   
    Motor* get_motor();

    // -----------------------------------------------------------------------
    // Aerodynamic coefficient accessors (from the latest lookup result)
    // -----------------------------------------------------------------------
    double get_cn()      const;
    double get_ca_on()   const;
    double get_ca_off()  const;
    double get_cd_on()   const;
    double get_cd_off()  const;
    double get_cp()      const;
    double get_cx_aero() const;
    double get_cy_aero() const;
    double get_cz_aero() const;
      
    //int   current_stage() const;

    // -----------------------------------------------------------------------
    // Inertia
    // -----------------------------------------------------------------------
    /** Diagonal inertia matrix [Ixx, Iyy, Izz] for a solid cylinder. */
    std::array<double, 3> I(double total_mass) const;

    /** Element-wise inverse of I(). */
    std::array<double, 3> I_inv(double total_mass) const;

    // -----------------------------------------------------------------------
    // Sensor simulation
    // -----------------------------------------------------------------------
    struct SensorConfig {
        struct { double RMS; } high_g;
        struct { double RMS; } gyro;
        struct { double RMS; } barometer;
        struct { double error; } bno;
    };

    int    current_stage_;

    Vec3   get_accelerometer_data(const RocketState& x_state, const SensorConfig& sc);
    Vec3   get_gyro_data         (const RocketState& x_state, const SensorConfig& sc);
    double get_barometer_data    (const RocketState& x_state, const SensorConfig& sc);
    Vec3   get_bno_orientation   (const RocketState& x_state, const SensorConfig& sc);
    Vec3   get_magnetometer_data (const RocketState& x_state);

    // -----------------------------------------------------------------------
    // Data logging
    // -----------------------------------------------------------------------
    void add_to_dict(const RocketState& x,
                     double baro_alt,
                     const Vec3& accel,
                     const Vec3& bno_ang_pos,
                     const Vec3& gyro,
                     const std::vector<double>& kalman_filter,
                     const std::vector<double>& kf_cov,
                     const std::vector<double>& kalman_filter_r,
                     double alpha,
                     double rocket_total_mass,
                     double motor_mass,
                     double flap_ext,
                     double dt,
                     const std::string& fsm_state = "STATE_IDLE");

    void update_coeffs(double velocity);

    /** Returns telemetry as a list of CSV rows (one row per timestep). */
    std::vector<std::vector<std::string>> to_csv() const;

    /** Returns telemetry in MIDAS Trimmed CSV format for test_ekf.cpp. */
    std::vector<std::vector<std::string>> to_midas_csv() const;

    // -----------------------------------------------------------------------
    // Public data members (forces / motor owned here)
    // -----------------------------------------------------------------------
    std::unique_ptr<Motor>  motor;
    std::unique_ptr<Forces> forces;

    double rocket_dry_mass;
    double rocket_total_mass;
    double motor_mass_val;
    double dt_;
    int current_stage;


private:
    // Physical parameters
    double cm_rocket_;
    double cm_motor_;
    double cm_;
    double cp_;
    double r_r_;
    double l_;
    double A_;
    double A_s_;
    double max_ext_length_;

    // Motor parameters (for Motor constructor)
    double impulse_;
    double motor_mass_init_;
    double delay_;

    Atmosphere* atm_;
    std::vector<std::shared_ptr<Rocket>> stages_;
       // -1 = base rocket / pre-separation
    double separation_timestamp_;

    // -----------------------------------------------------------------------
    // Coefficient lookup tables (loaded from CSV)
    // -----------------------------------------------------------------------
    struct CoeffRow {
        double mach;
        int    alpha;
        double CN, ca_on, ca_off, cd_on, cd_off, CL, CP;
    };
    struct GncRow {
        double mach;
        double roll, pitch, yaw;
    };
    std::vector<CoeffRow> coeffs_df_;
    std::vector<GncRow>   coeffs_gnc_df_;

    // Latest coefficient values (appended each step)
    std::vector<double> cn_hist_, ca_on_hist_, ca_off_hist_;
    std::vector<double> cd_on_hist_, cd_off_hist_, cl_hist_, cp_hist_;
    std::vector<double> cx_aero_hist_, cy_aero_hist_, cz_aero_hist_;

    // -----------------------------------------------------------------------
    // Simulation telemetry logs
    // -----------------------------------------------------------------------
    struct SimLog {
        std::vector<Vec3>   pos, vel, accel, ang_pos, ang_vel, ang_accel;
        std::vector<double> alpha, flap_ext, rocket_total_mass, motor_mass, time;
    } sim_dict_;

    struct KalmanLog {
        std::vector<std::array<double,3>> x, y, z;
        std::vector<std::array<double,3>> cov_x, cov_y, cov_z;
        std::vector<std::array<double,3>> rx, ry, rz;
        std::vector<double> time;
    } kalman_dict_;

    struct SensorLog {
        std::vector<double> baro_alt;
        std::vector<double> imu_accel_x, imu_accel_y, imu_accel_z;
        std::vector<double> imu_ang_pos_x, imu_ang_pos_y, imu_ang_pos_z;
        std::vector<double> imu_gyro_x, imu_gyro_y, imu_gyro_z;
        std::vector<double> imu_mag_x, imu_mag_y, imu_mag_z;
    } sensor_dict_;

    std::vector<std::string> fsm_state_;

    // Helpers
    void load_coeff_csv(const std::string& path);
    void load_gnc_csv  (const std::string& path);
    static double gauss_sample(double mean, double stddev);
};
