/**
 * main.cpp — 6-DOF RK-4 Simulation
 * Build (from project root):
 *   g++ -std=c++17 -O2 -D_USE_MATH_DEFINES *.cpp -o sim_main
 *   ./sim_main
 */

#include "rocket.h"
#include "atmosphere.h"
#include "controller.h"
#include "sensors.h"
#include "sensor_config.h"
#include "magnetic_model.h"
#include "liveEKF.h"

#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// FSM state strings (mirrors Python fsm_state logic)
// ---------------------------------------------------------------------------
static const std::string FSM_IDLE         = "STATE_IDLE";
static const std::string FSM_FIRST_BOOST  = "STATE_FIRST_BOOST";
static const std::string FSM_SECOND_BOOST = "STATE_SECOND_BOOST";
static const std::string FSM_BURNOUT      = "STATE_BURNOUT";
static const std::string FSM_COAST        = "STATE_COAST";
static const std::string FSM_APOGEE       = "STATE_APOGEE";

// ---------------------------------------------------------------------------
// Dummy Kalman outputs — EKF is run externally by test_ekf.cpp
// ---------------------------------------------------------------------------
static const std::vector<double> ZERO9(9, 0.0);

class Simulation {
public:
    Simulation(Rocket*                     rocket,
               double                      dt,
               const RocketState&          x0,
               const Rocket::SensorConfig& sensor_cfg,
               double                      time_stamp = 0.0)
        : rocket_(rocket),
          dt_(dt),
          x_(x0),
          sensor_cfg_(sensor_cfg),
          time_stamp_(time_stamp)
    {}

    double      time()  const { return time_stamp_; }
    RocketState state() const { return x_; }
    LiveEKF ekf_;

    // -----------------------------------------------------------------------
    // idle_stage
    // -----------------------------------------------------------------------
    void idle_stage(double launch_delay)
    {
        while (time_stamp_ < launch_delay) {
            auto [baro_alt, accel, gyro, bno] = get_sensor_data();
            KalmanStep k = ekf_.tick(dt_, time_stamp_ * 1000.0, baro_alt, accel, gyro, bno, FSM_IDLE, x_.position);
            rocket_->add_to_dict(x_,
                                 baro_alt, accel, bno, gyro,
                                 k.state, k.cov, k.residual,
                                 /*alpha=*/0.0,
                                 rocket_->get_rocket_dry_mass(),
                                 rocket_->get_total_motor_mass(time_stamp_),
                                 /*flap_ext=*/0.0,
                                 dt_,
                                 FSM_IDLE);
            time_stamp_ += dt_;
        }
    }

    // -----------------------------------------------------------------------
    // execute_stage
    // -----------------------------------------------------------------------
    void execute_stage()
    {
        constexpr double STAGE_SEP_DELAY = 1.0;

        rocket_->get_motor()->ignite(time_stamp_);
        double ignition_time = time_stamp_;
        double burn_time     = rocket_->get_motor()->get_burn_time();
        bool   first_step    = true;

        std::cout << "Staged at t=" << time_stamp_ << " s\n";

        while (time_stamp_ < ignition_time + burn_time + STAGE_SEP_DELAY) {
            auto [baro_alt, accel, gyro, bno] = get_sensor_data();

            double      t_since = time_stamp_ - ignition_time;
            std::string fsm     = (t_since < burn_time) ? FSM_FIRST_BOOST
                                                        : FSM_BURNOUT;

            rocket_->set_motor_mass(time_stamp_);

            bool is_staging = first_step && (rocket_->current_stage != -1);
            auto [fm_test, a_test] = rocket_->forces->get_force(x_, 0.0, time_stamp_, 0.0, 0.0, 0.0);
            //std::cout << "F=" << fm_test[0][0] << " M=" << fm_test[1][0] << " alpha=" << a_test << "\n";
            auto [new_x, alpha] = rk4(x_, dt_, time_stamp_, 0.0, is_staging);
            KalmanStep k = ekf_.tick(dt_, time_stamp_ * 1000.0, baro_alt, accel, gyro, bno, fsm, new_x.position);

            rocket_->add_to_dict(new_x,
                                 baro_alt, accel, bno, gyro,
                                 k.state, k.cov, k.residual,
                                 alpha,
                                 rocket_->get_rocket_dry_mass(),
                                 rocket_->get_total_motor_mass(time_stamp_),
                                 /*flap_ext=*/0.0,
                                 dt_,
                                 fsm);
            x_          = new_x;
            time_stamp_ += dt_;
            first_step   = false;
            std::cout << "thurst" << rocket_->get_motor()->get_thrust(time_stamp_)[0] << "\n";
            std::cout << "Mass: " << rocket_->get_motor()->get_mass(time_stamp_) << "\n";
            std::cout << "Accel: " << rocket_->get_accelerometer_data(x_, sensor_cfg_)[0] << "\n";
        }
    }

    // -----------------------------------------------------------------------
    // run_stages
    // -----------------------------------------------------------------------
    void run_stages()
    {
        bool has_more = true;
        while (has_more) {
            execute_stage();
      
            has_more = rocket_->separate_stage(time_stamp_);
        }
    }

    // -----------------------------------------------------------------------
    // coast
    // -----------------------------------------------------------------------
    void coast()
    {
        double max_altitude   = 0.0;
        bool   apogee_reached = false;

        while (x_.position[0] >= 0.0) {
            auto [baro_alt, accel, gyro, bno] = get_sensor_data();

            double      alt = x_.position[0];
            std::string fsm;
            if (alt > max_altitude) {
                max_altitude = alt;
                fsm = FSM_COAST;
            } else if (!apogee_reached && alt < max_altitude - 1.0) {
                apogee_reached = true;
                fsm = FSM_APOGEE;
            } else {
                fsm = FSM_COAST;
            }

            auto [new_x, alpha] = rk4(x_, dt_, time_stamp_, 0.0, false);
            KalmanStep k = ekf_.tick(dt_, time_stamp_ * 1000.0, baro_alt, accel, gyro, bno, fsm, new_x.position);

            rocket_->add_to_dict(new_x,
                                 baro_alt, accel, bno, gyro,
                                 k.state, k.cov, k.residual,
                                 alpha,
                                 rocket_->get_rocket_dry_mass(),
                                 rocket_->get_total_motor_mass(time_stamp_),
                                 /*flap_ext=*/0.0,
                                 dt_,
                                 fsm);
            x_          = new_x;
            time_stamp_ += dt_;
        }
    }

private:
    Rocket*              rocket_;
    double               dt_;
    RocketState          x_;
    Rocket::SensorConfig sensor_cfg_;
    double               time_stamp_;

    struct SensorBundle {
        double baro_alt;
        Vec3   accel;
        Vec3   gyro;
        Vec3   bno;
    };

    SensorBundle get_sensor_data() const
    {
        return {
            rocket_->get_barometer_data(x_,     sensor_cfg_),
            rocket_->get_accelerometer_data(x_,  sensor_cfg_),
            rocket_->get_gyro_data(x_,           sensor_cfg_),
            rocket_->get_bno_orientation(x_,     sensor_cfg_)
        };
    }

    std::pair<RocketState, double>
    rk4(const RocketState& y0,
        double dt,
        double timestamp,
        double flap_ext,
        bool   is_staging) const
    {
        double ejection_force = is_staging ? 12.0 : 0.0;
        double ejection_theta = 0.0;
        double ejection_phi   = 0.0;

        // double total_mass = rocket_->get_rocket_total_mass(timestamp);
        // auto   I_inv      = rocket_->I_inv(total_mass);

        // Vec3 k1_v  = y0.velocity;
        // Vec3 k1_av = y0.angular_velocity;
        // Vec3 k1_p  = y0.velocity;
        // Vec3 k1_ap = y0.angular_velocity;

        auto get_f = [&](const Vec3& pos, const Vec3& vel,
                         const Vec3& ang_pos, const Vec3& ang_vel)
            ->  std::pair<RocketState, double>
        {
            RocketState tmp;
            tmp.position         = pos;
            tmp.velocity         = vel;
            tmp.orientation      = ang_pos;
            tmp.angular_velocity = ang_vel;
            auto [fm, alpha] = rocket_->forces->get_force(tmp, flap_ext, timestamp,
                                                          ejection_force,
                                                          ejection_theta, ejection_phi);
            double mass = rocket_->get_rocket_total_mass(timestamp);
            auto   Iinv = rocket_->I_inv(mass);
            RocketState deriv;
            deriv.position         = vel;
            deriv.velocity         = {fm[0][0] / mass, fm[0][1] / mass, fm[0][2] / mass};
            deriv.orientation      = ang_vel;
            deriv.angular_velocity = {fm[1][0] * Iinv[0], fm[1][1] * Iinv[1], fm[1][2] * Iinv[2]};
            return {deriv, alpha};
        };

        auto add_s = [](const Vec3& a, const Vec3& b, double s) -> Vec3 {
            return {a[0]+s*b[0], a[1]+s*b[1], a[2]+s*b[2]};
        //};
        // auto div3 = [](const Vec3& v, double s) -> Vec3 {
        //     return {v[0]/s, v[1]/s, v[2]/s};
        // };
        // auto mul3 = [](const Vec3& v, const std::array<double,3>& s) -> Vec3 {
        //     return {v[0]*s[0], v[1]*s[1], v[2]*s[2]};
        };

        // k2
        // auto [fm2, _2] = get_f(y0.position,
        //                         add_s(y0.velocity,         k1_v,  dt/2),
        //                         y0.orientation,
        //                         add_s(y0.angular_velocity, k1_av, dt/2));
        // Vec3 k2_v = fm2[0], k2_av = fm2[1];
        // Vec3 k2_p  = step_p(y0.position,    add_s(y0.position,    k1_p,  dt/2), dt/2);
        // Vec3 k2_ap = step_p(y0.orientation, add_s(y0.orientation, k1_ap, dt/2), dt/2);
        auto [k1, _1] = get_f(y0.position, y0.velocity, y0.orientation, y0.angular_velocity);

        // k3
        // auto [fm3, _3] = get_f(y0.position,
        //                         add_s(y0.velocity,         div3(k2_v, total_mass), dt/2),
        //                         y0.orientation,
        //                         add_s(y0.angular_velocity, mul3(k2_av, I_inv),     dt/2));
        // Vec3 k3_v = fm3[0], k3_av = fm3[1];
        // Vec3 k3_p  = step_p(y0.position,    add_s(y0.position,    k2_p,  dt/2), dt/2);
        // Vec3 k3_ap = step_p(y0.orientation, add_s(y0.orientation, k2_ap, dt/2), dt/2);
        auto [k2, _2] = get_f(add_s(y0.position,         k1.position,         dt/2),
                              add_s(y0.velocity,         k1.velocity,         dt/2),
                              add_s(y0.orientation,      k1.orientation,      dt/2),
                              add_s(y0.angular_velocity, k1.angular_velocity, dt/2));

        // k4
        // auto [fm4, alpha] = get_f(y0.position,
        //                            add_s(y0.velocity,         div3(k3_v, total_mass), dt),
        //                            y0.orientation,
        //                            add_s(y0.angular_velocity, mul3(k3_av, I_inv),     dt));
        // Vec3 k4_v = fm4[0], k4_av = fm4[1];
        // Vec3 k4_p  = step_p(y0.position,    add_s(y0.position,    k3_p,  dt), dt);
        // Vec3 k4_ap = step_p(y0.orientation, add_s(y0.orientation, k3_ap, dt), dt);
        auto [k3, _3] = get_f(add_s(y0.position,         k2.position,         dt/2),
                              add_s(y0.velocity,         k2.velocity,         dt/2),
                              add_s(y0.orientation,      k2.orientation,      dt/2),
                              add_s(y0.angular_velocity, k2.angular_velocity, dt/2));

        auto [k4, alpha] = get_f(add_s(y0.position,         k3.position,         dt),
                                 add_s(y0.velocity,         k3.velocity,         dt),
                                 add_s(y0.orientation,      k3.orientation,      dt),
                                 add_s(y0.angular_velocity, k3.angular_velocity, dt));

        // Weighted combine
        auto rk4c = [&](const Vec3& y,
                         const Vec3& kk1, const Vec3& kk2,
                         const Vec3& kk3, const Vec3& kk4) -> Vec3 {
            return {y[0]+(dt/6.0)*(kk1[0]+2*kk2[0]+2*kk3[0]+kk4[0]),
                    y[1]+(dt/6.0)*(kk1[1]+2*kk2[1]+2*kk3[1]+kk4[1]),
                    y[2]+(dt/6.0)*(kk1[2]+2*kk2[2]+2*kk3[2]+kk4[2])};
        };

        RocketState new_x;
        // new_x.position         = rk4c(y0.position,         k1_p,  k2_p,              k3_p,              k4_p);
        // new_x.velocity         = rk4c(y0.velocity,         k1_v,  div3(k2_v,total_mass), div3(k3_v,total_mass), div3(k4_v,total_mass));
        // new_x.orientation      = rk4c(y0.orientation,      k1_ap, k2_ap,             k3_ap,             k4_ap);
        // new_x.angular_velocity = rk4c(y0.angular_velocity, k1_av, mul3(k2_av,I_inv), mul3(k3_av,I_inv), mul3(k4_av,I_inv));
        new_x.position         = rk4c(y0.position,         k1.position,         k2.position,         k3.position,         k4.position);
        new_x.velocity         = rk4c(y0.velocity,         k1.velocity,         k2.velocity,         k3.velocity,         k4.velocity);
        new_x.orientation      = rk4c(y0.orientation,      k1.orientation,      k2.orientation,      k3.orientation,      k4.orientation);
        new_x.angular_velocity = rk4c(y0.angular_velocity, k1.angular_velocity, k2.angular_velocity, k3.angular_velocity, k4.angular_velocity);

        // Final derivatives
        // auto [fm_f, alpha_f] = get_f(new_x.position, new_x.velocity,
        //                              new_x.orientation, new_x.angular_velocity);
        // double m = rocket_->get_rocket_total_mass(timestamp);
        // new_x.acceleration         = div3(fm_f[0], m);
        // new_x.angular_acceleration = mul3(fm_f[1], I_inv);
        auto [final_deriv, alpha_f] = get_f(new_x.position, new_x.velocity,
                                            new_x.orientation, new_x.angular_velocity);
        new_x.acceleration         = final_deriv.velocity;
        new_x.angular_acceleration = final_deriv.angular_velocity;


        return {new_x, alpha_f};
    }
};

// ===========================================================================
// simulator()
// ===========================================================================
static void simulator(const RocketState&          x0,
                      Rocket*                     rocket,
                      double                      dt,
                      const Rocket::SensorConfig& sc,
                      double                      launch_delay)
{
    Simulation sim(rocket, dt, x0, sc);
    sim.idle_stage(launch_delay);

    auto t0 = std::chrono::steady_clock::now();
    sim.run_stages();
    sim.coast();
    double elapsed = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - t0).count();
    std::cout << "Runtime: " << elapsed << " s\n";
}

// ===========================================================================
// main
// ===========================================================================
int main()
{
    // --- Atmosphere ---------------------------------------------------------
    auto atm = std::make_shared<Atmosphere>(
        0.0,        // wind_direction_variance_mean
        0.01,       // wind_direction_variance_stddev
        0.0,        // wind_magnitude_variance_mean
        0.5,        // wind_magnitude_variance_stddev
        false,      // enable_direction_variance
        false,      // enable_magnitude_variance
        Vec3{ 0.0, 0.0, 0.0 }, // nominal_wind_direction
        0.0         // nominal_wind_magnitude
    );
    // --- Initial state -------------------------------------------------------
    RocketState x0;
    x0.position    = {0.0, 0.0, 0.0};
    x0.velocity    = {0.0, 0.0, 0.0};
    x0.orientation = {0.0, 0.05, 0.0};  // 0.05 rad pitch offset (mirrors Python)

    const double dt = 0.01;

    // --- Booster config (from yaml) ------------------------------------------
    Rocket::Config cfg_booster;
    cfg_booster.structure_cm        = 1.48;
    cfg_booster.combined_cm         = 1.03;
    cfg_booster.combined_cp         = 0.63;
    cfg_booster.dry_mass            = 30.0;
    cfg_booster.radius              = 0.0508;
    cfg_booster.length              = 6.68;
    cfg_booster.rasaero_lookup_file = "../LookUp/RASAero.csv";
    cfg_booster.motor_cm            = 2.5;
    cfg_booster.motor_impulse       = 10000.0;
    cfg_booster.motor_mass          = 5.0;
    cfg_booster.motor_delay         = 0.0;
    cfg_booster.motor_lookup_file   = "../LookUp/m2500.csv";
    cfg_booster.flap_max_ext_length = 0.0178;
    cfg_booster.flap_max_ext_spd    = 0.001;

    // --- Sustainer config (from yaml) ----------------------------------------
    Rocket::Config cfg_sustainer;
    cfg_sustainer.structure_cm        = 1.48;
    cfg_sustainer.combined_cm         = 1.03;
    cfg_sustainer.combined_cp         = 0.63;
    cfg_sustainer.dry_mass            = 14.691;
    cfg_sustainer.radius              = 0.0508;
    cfg_sustainer.length              = 3.34;
    cfg_sustainer.rasaero_lookup_file = "../LookUp/RASAero.csv";
    cfg_sustainer.motor_cm            = 2.5;
    cfg_sustainer.motor_impulse       = 10000.0;
    cfg_sustainer.motor_mass          = 5.0;
    cfg_sustainer.motor_delay         = 0.0;
    cfg_sustainer.motor_lookup_file   = "../LookUp/m2500.csv";
    cfg_sustainer.flap_max_ext_length = 0.0178;
    cfg_sustainer.flap_max_ext_spd    = 0.001;

    // --- Build sustainer first, then booster ---------------------------------
    auto sustainer = std::make_shared<Rocket>(
        dt, x0, cfg_sustainer, atm.get(),
        std::vector<std::shared_ptr<Rocket>>{},
        "../LookUp/ekf_cd_test.CSV",
        "");  // GNC coefficients not used in original Python

    Rocket rocket(dt, x0, cfg_booster, atm.get(),
                  std::vector<std::shared_ptr<Rocket>>{sustainer},
                  "../LookUp/ekf_cd_test.CSV",
                  "");  // GNC coefficients not used in original Python

    // --- Sensor config (from yaml) -------------------------------------------
    Rocket::SensorConfig sc;
    sc.high_g.RMS    = 1.9;          // mg
    sc.gyro.RMS      = 75.0;         // mdps
    sc.barometer.RMS = 0.012;        // hPa
    sc.bno.error     = 2.5 / 3.0;   // degrees

    // --- Run -----------------------------------------------------------------
    std::cout << "Writing to MIDAS format CSV file...\n";
    simulator(x0, &rocket, dt, sc, cfg_booster.motor_delay);

    // --- Export MIDAS CSV ----------------------------------------------------
    static const std::string MIDAS_HEADER =
        "sensor,file number,timestamp,"
        "lowg.ax,lowg.ay,lowg.az,"
        "highg.ax,highg.ay,highg.az,"
        "barometer.temperature,barometer.pressure,barometer.altitude,"
        "continuity.pins[0],continuity.pins[1],continuity.pins[2],continuity.pins[3],"
        "voltage.voltage,voltage.current,"
        "gps.latitude,gps.longitude,gps.altitude,gps.speed,gps.fix_type,gps.sats_in_view,gps.time,"
        "magnetometer.mx,magnetometer.my,magnetometer.mz,"
        "orientation.has_data,orientation.reading_type,"
        "orientation.yaw,orientation.pitch,orientation.roll,"
        "orientation.orientation_velocity.vx,orientation.orientation_velocity.vy,orientation.orientation_velocity.vz,"
        "orientation.angular_velocity.vx,orientation.angular_velocity.vy,orientation.angular_velocity.vz,"
        "orientation.orientation_acceleration.ax,orientation.orientation_acceleration.ay,orientation.orientation_acceleration.az,"
        "orientation.linear_acceleration.ax,orientation.linear_acceleration.ay,orientation.linear_acceleration.az,"
        "orientation.gx,orientation.gy,orientation.gz,"
        "orientation.magnetometer.mx,orientation.magnetometer.my,orientation.magnetometer.mz,"
        "orientation.temperature,orientation.pressure,orientation.tilt,"
        "orientation.orientation_quaternion.w,orientation.orientation_quaternion.x,"
        "orientation.orientation_quaternion.y,orientation.orientation_quaternion.z,"
        "lowglsm.gx,lowglsm.gy,lowglsm.gz,lowglsm.ax,lowglsm.ay,lowglsm.az,"
        "fsm,"
        "kalman.position.px,kalman.position.py,kalman.position.pz,"
        "kalman.velocity.vx,kalman.velocity.vy,kalman.velocity.vz,"
        "kalman.acceleration.ax,kalman.acceleration.ay,kalman.acceleration.az,"
        "kalman.altitude,"
        "pyro.is_global_armed,"
        "pyro.channel_firing[0],pyro.channel_firing[1],pyro.channel_firing[2],pyro.channel_firing[3],"
        "cameradata.camera_state,cameradata.camera_voltage\n";

    const std::string output_path = "6DOF_RK4_SIMULATED.csv";
    auto midas_rows = rocket.to_midas_csv();

    {
        std::ofstream f(output_path);
        if (!f) {
            std::cerr << "ERROR: cannot open " << output_path << "\n";
            return 1;
        }
        f << MIDAS_HEADER;
        for (const auto& row : midas_rows) {
            for (std::size_t i = 0; i < row.size(); ++i) {
                f << row[i];
                if (i + 1 < row.size()) f << ',';
            }
            f << '\n';
        }
    }

    std::cout << "MIDAS format CSV written to: " << output_path << "\n";
    std::cout << "Total data points: " << midas_rows.size() << "\n";
    std::cout << "This file can now be used with test_ekf.cpp\n";
    return 0;
}
