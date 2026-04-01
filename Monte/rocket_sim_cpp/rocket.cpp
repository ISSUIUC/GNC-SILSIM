#include "rocket.h"
#include "magnetic_model.h"
#include "vectors.h"
#include "properties.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <random>

namespace {
constexpr double kLaunchLatitudeDeg = 40.0; // Champaign
constexpr double kLaunchLongitudeDeg = -88.0;
constexpr double kMetersPerDegreeLatitude = 111320.0;
constexpr double kEarthRadiusKm = 6371.2;
}

// ---------------------------------------------------------------------------
// RNG (shared across sensor functions)
// ---------------------------------------------------------------------------
static std::mt19937 rng_global(std::random_device{}());

double Rocket::gauss_sample(double mean, double stddev) {
    std::normal_distribution<double> d(mean, stddev);
    return d(rng_global);
}

// CSV loaders

void Rocket::load_coeff_csv(const std::string& path) {
    if (path.empty()) return;
    std::ifstream f(path);
    if (!f.is_open()) throw std::runtime_error("Rocket: cannot open coeff CSV: " + path);
    std::string line;
    std::getline(f, line); // header
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string tok;
        CoeffRow row{};
        std::getline(ss, tok, ','); row.mach   = std::stod(tok);
        std::getline(ss, tok, ','); row.alpha  = std::stoi(tok);
        std::getline(ss, tok, ','); row.CN     = std::stod(tok);
        std::getline(ss, tok, ','); row.ca_on  = std::stod(tok);
        std::getline(ss, tok, ','); row.ca_off = std::stod(tok);
        std::getline(ss, tok, ','); row.cd_on  = std::stod(tok);
        std::getline(ss, tok, ','); row.cd_off = std::stod(tok);
        std::getline(ss, tok, ','); row.CL     = std::stod(tok);
        std::getline(ss, tok, ','); row.CP     = std::stod(tok);
        coeffs_df_.push_back(row);
    }
}

void Rocket::load_gnc_csv(const std::string& path) {
    if (path.empty()) return;
    std::ifstream f(path);
    if (!f.is_open()) throw std::runtime_error("Rocket: cannot open GNC CSV: " + path);
    std::string line;
    std::getline(f, line); // header
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string tok;
        GncRow row{};
        std::getline(ss, tok, ','); row.mach  = std::stod(tok);
        std::getline(ss, tok, ','); row.roll  = std::stod(tok);
        std::getline(ss, tok, ','); row.pitch = std::stod(tok);
        std::getline(ss, tok, ','); row.yaw   = std::stod(tok);
        coeffs_gnc_df_.push_back(row);
    }
}

// Constructor

Rocket::Rocket(double dt,
               const RocketState& /*x0*/,
               const Config& cfg,
               Atmosphere* atm,
               std::vector<std::shared_ptr<Rocket>> stages,
               const std::string& coeff_csv,
               const std::string& gnc_csv)
    : rocket_dry_mass(cfg.dry_mass),
      rocket_total_mass(cfg.dry_mass + cfg.motor_mass),
      motor_mass_val(cfg.motor_mass),
      dt_(dt),
      cm_rocket_(cfg.structure_cm),
      cm_motor_(cfg.motor_cm),
      cm_(cfg.combined_cm),
      cp_(cfg.combined_cp),
      r_r_(cfg.radius),
      l_(cfg.length),
      A_(M_PI * cfg.radius * cfg.radius),
      A_s_(2.0 * cfg.radius * cfg.length),
      max_ext_length_(cfg.flap_max_ext_length),
      impulse_(cfg.motor_impulse),
      motor_mass_init_(cfg.motor_mass),
      delay_(cfg.motor_delay),
      atm_(atm),
      stages_(std::move(stages)),
      current_stage_(-1),
      separation_timestamp_(0.0)
{
    load_coeff_csv(coeff_csv);
    load_gnc_csv(gnc_csv);

    // Seed coefficient history with a zero entry so accessors never OOB
    cn_hist_     = {0.0};
    ca_on_hist_  = {0.0};
    ca_off_hist_ = {0.0};
    cd_on_hist_  = {0.0};
    cd_off_hist_ = {0.0};
    cl_hist_     = {0.0};
    cp_hist_     = {0.0};
    cx_aero_hist_= {0.0};
    cy_aero_hist_= {0.0};
    cz_aero_hist_= {0.0};

    motor = std::make_unique<Motor>(
        rocket_total_mass,
        cm_,
        cm_rocket_,
        cm_motor_,
        rocket_dry_mass,
        impulse_,
        motor_mass_init_,
        delay_,
        cfg.motor_lookup_file
    );

    forces = std::make_unique<Forces>(
        max_ext_length_,
        cm_,
        cp_,
        A_,
        A_s_,
        rocket_dry_mass,
        motor.get(),
        cfg.rasaero_lookup_file,
        atm_
    );
}

// ---------------------------------------------------------------------------
// Stage management
// ---------------------------------------------------------------------------
bool Rocket::separate_stage(double timestamp) {
    if (current_stage_ == static_cast<int>(stages_.size()) - 1) return false;
    ++current_stage_;
    separation_timestamp_ = timestamp;
    // Clone the upper stage's Forces (copies rasaero table + config).
    // The cloned Forces retains a raw pointer to the upper stage's motor,
    // which is kept alive by stages_[current_stage_].
    forces = std::make_unique<Forces>(*stages_[current_stage_]->forces);
    return true;
}

// ---------------------------------------------------------------------------
// Mass / geometry (stage-aware)
// ---------------------------------------------------------------------------
double Rocket::get_total_motor_mass(double timestamp) const {
    double mass = 0.0;
    if (current_stage_ == -1) {
        mass += motor->get_mass(timestamp - separation_timestamp_);
        for (const auto& s : stages_) mass += s->motor->total_mass;
        return mass;
    }
    mass += stages_[current_stage_]->motor->get_mass(timestamp - separation_timestamp_);
    for (int i = current_stage_ + 1; i < static_cast<int>(stages_.size()); ++i)
        mass += stages_[i]->motor->total_mass;
    return mass;
}

void Rocket::set_motor_mass(double timestamp) {
    if (current_stage_ == -1)
        motor_mass_val = motor->get_mass(timestamp);
    else
        motor_mass_val = stages_[current_stage_]->motor->get_mass(timestamp - separation_timestamp_);
    rocket_total_mass = rocket_dry_mass + get_total_motor_mass(timestamp);
}

bool Rocket::is_motor_burnout(double timestamp) const {
    return motor->burnout(timestamp);
}

double Rocket::get_motor_mass(double timestamp) {
    return motor->get_mass(timestamp - separation_timestamp_);
}

double Rocket::get_rocket_dry_mass() const {
    if (current_stage_ == -1) return rocket_dry_mass;
    return stages_[current_stage_]->get_rocket_dry_mass();
}

double Rocket::get_rocket_total_mass(double timestamp) {
    return get_rocket_dry_mass() + get_total_motor_mass(timestamp);
}

double Rocket::get_CM() const {
    if (current_stage_ == -1) return cm_;
    return stages_[current_stage_]->get_CM();
}

double Rocket::get_CP() const {
    if (current_stage_ == -1) return cp_;
    return stages_[current_stage_]->get_CP();
}

double Rocket::get_A() const {
    if (current_stage_ == -1) return A_;
    return stages_[current_stage_]->get_A();
}

double Rocket::get_A_s() const {
    if (current_stage_ == -1) return A_s_;
    return stages_[current_stage_]->get_A_s();
}

Motor* Rocket::get_motor() {
    if (current_stage_ == -1) return motor.get();
    return stages_[current_stage_]->get_motor();
}



// ---------------------------------------------------------------------------
// Coefficient accessors
// ---------------------------------------------------------------------------
double Rocket::get_cn()      const { return cn_hist_.back(); }
double Rocket::get_ca_on()   const { return ca_on_hist_.back(); }
double Rocket::get_ca_off()  const { return ca_off_hist_.back(); }
double Rocket::get_cd_on()   const { return cd_on_hist_.back(); }
double Rocket::get_cd_off()  const { return cd_off_hist_.back(); }
double Rocket::get_cp()      const { return cp_hist_.back(); }
double Rocket::get_cx_aero() const { return cx_aero_hist_.back(); }
double Rocket::get_cy_aero() const { return cy_aero_hist_.back(); }
double Rocket::get_cz_aero() const { return cz_aero_hist_.back(); }

// ---------------------------------------------------------------------------
// Inertia (thin-walled cylinder approximation)
// ---------------------------------------------------------------------------
std::array<double, 3> Rocket::I(double total_mass) const {
    double Ixx = 0.5 * total_mass * r_r_ * r_r_;
    double Iyy = (total_mass / 12.0) * (l_ * l_ + 3.0 * r_r_ * r_r_);
    return {Ixx, Iyy, Iyy};
}

std::array<double, 3> Rocket::I_inv(double total_mass) const {
    auto inertia = I(total_mass);
    return {1.0 / inertia[0], 1.0 / inertia[1], 1.0 / inertia[2]};
}

// ---------------------------------------------------------------------------
// Sensor simulation
// ---------------------------------------------------------------------------
Vec3 Rocket::get_accelerometer_data(const RocketState& x, const SensorConfig& sc) {
    double g_mag = (prop::G * prop::m_e) / std::pow(prop::r_e + x.position[0], 2);
    Vec3 gravity = {g_mag, 0.0, 0.0};
    Vec3 a_world = {x.acceleration[0] + gravity[0],
                    x.acceleration[1] + gravity[1],
                    x.acceleration[2] + gravity[2]};
    Vec3 body = vct::world_to_body(x.orientation, a_world);
    double rms = sc.high_g.RMS * 9.81 / 1000.0;
    return {gauss_sample(body[0], rms),
            gauss_sample(body[1], rms),
            gauss_sample(body[2], rms)};
}



Vec3 Rocket::get_gyro_data(const RocketState& x, const SensorConfig& sc) {
    Vec3 body = vct::world_to_body(x.orientation, x.angular_velocity);
    double rms = sc.gyro.RMS * M_PI / 180000.0;
    return {gauss_sample(body[0], rms),
            gauss_sample(body[1], rms),
            gauss_sample(body[2], rms)};
}

double Rocket::get_barometer_data(const RocketState& x, const SensorConfig& sc) {
    double rms = sc.barometer.RMS * 100.0;
    double pressure = atm_->get_pressure(x.position[0]);
    pressure = gauss_sample(pressure, rms);
    return atm_->get_altitude(pressure);
}

Vec3 Rocket::get_bno_orientation(const RocketState& x, const SensorConfig& sc) {
    double err = sc.bno.error * M_PI / 180.0;
    return {gauss_sample(x.orientation[0], err),
            gauss_sample(x.orientation[1], err),
            gauss_sample(x.orientation[2], err)};
}

Vec3 Rocket::get_magnetometer_data(const RocketState& x) {
    const double latitude_deg = kLaunchLatitudeDeg + x.position[1] / kMetersPerDegreeLatitude;
    const double longitude_scale = kMetersPerDegreeLatitude * std::cos(kLaunchLatitudeDeg * M_PI / 180.0);
    const double longitude_deg = kLaunchLongitudeDeg + x.position[2] / longitude_scale;
    const double radius_km = kEarthRadiusKm + x.position[0] / 1000.0;
    const double days_since_2020 = 365.0 * 6.0;

    const Vec3 ned_field = magnetic_model::magnet(radius_km,
                                                  latitude_deg,
                                                  longitude_deg,
                                                  days_since_2020);

    // Convert NED (north, east, down) into the sim's world frame (up, north, east).
    const Vec3 world_field = {-ned_field[2], ned_field[0], ned_field[1]};
    return vct::world_to_body(x.orientation, world_field);
}

// ---------------------------------------------------------------------------
// Coefficient table update
// ---------------------------------------------------------------------------
void Rocket::update_coeffs(double velocity) {
    double a = velocity / 340.29;
    if (a < 0.01) a = 0.01;
    double mach_rounded = std::round(a * 100.0) / 100.0;

    // Find matching row at alpha == 2
    auto it = std::find_if(coeffs_df_.begin(), coeffs_df_.end(),
        [mach_rounded](const CoeffRow& r) {
            return std::abs(r.mach - mach_rounded) < 1e-9 && r.alpha == 2;
        });

    if (it != coeffs_df_.end()) {
        cn_hist_.push_back(it->CN);
        ca_on_hist_.push_back(it->ca_on);
        ca_off_hist_.push_back(it->ca_off);
        cd_on_hist_.push_back(it->cd_on);
        cd_off_hist_.push_back(it->cd_off);
        cl_hist_.push_back(it->CL);
        cp_hist_.push_back(it->CP);
    }

    auto git = std::find_if(coeffs_gnc_df_.begin(), coeffs_gnc_df_.end(),
        [mach_rounded](const GncRow& r) {
            return std::abs(r.mach - mach_rounded) < 1e-9;
        });

    if (git != coeffs_gnc_df_.end()) {
        cx_aero_hist_.push_back(git->roll);
        cy_aero_hist_.push_back(git->pitch);
        cz_aero_hist_.push_back(git->yaw);
    }
}

// ---------------------------------------------------------------------------
// Data logging
// ---------------------------------------------------------------------------
void Rocket::add_to_dict(const RocketState& x,
                          double baro_alt,
                          const Vec3& accel,
                          const Vec3& bno_ang_pos,
                          const Vec3& gyro,
                          const std::vector<double>& kf,
                          const std::vector<double>& kf_cov,
                          const std::vector<double>& kf_r,
                          double alpha,
                          double r_total_mass,
                          double m_mass,
                          double flap_ext,
                          double dt,
                          const std::string& fsm_state)
{
    // Sensor log
    sensor_dict_.baro_alt.push_back(baro_alt);
    sensor_dict_.imu_accel_x.push_back(accel[0]);
    sensor_dict_.imu_accel_y.push_back(accel[1]);
    sensor_dict_.imu_accel_z.push_back(accel[2]);
    sensor_dict_.imu_ang_pos_x.push_back(bno_ang_pos[0]);
    sensor_dict_.imu_ang_pos_y.push_back(bno_ang_pos[1]);
    sensor_dict_.imu_ang_pos_z.push_back(bno_ang_pos[2]);
    sensor_dict_.imu_gyro_x.push_back(gyro[0]);
    sensor_dict_.imu_gyro_y.push_back(gyro[1]);
    sensor_dict_.imu_gyro_z.push_back(gyro[2]);
    const Vec3 mag = get_magnetometer_data(x);
    sensor_dict_.imu_mag_x.push_back(mag[0]);
    sensor_dict_.imu_mag_y.push_back(mag[1]);
    sensor_dict_.imu_mag_z.push_back(mag[2]);

    // Kalman log (split 9-vectors into 3x3 blocks)
    auto slice3 = [](const std::vector<double>& v, int start) -> std::array<double,3> {
        return {v[start], v[start+1], v[start+2]};
    };
    kalman_dict_.x.push_back(slice3(kf, 0));
    kalman_dict_.y.push_back(slice3(kf, 3));
    kalman_dict_.z.push_back(slice3(kf, 6));
    kalman_dict_.cov_x.push_back(slice3(kf_cov, 0));
    kalman_dict_.cov_y.push_back(slice3(kf_cov, 3));
    kalman_dict_.cov_z.push_back(slice3(kf_cov, 6));
    kalman_dict_.rx.push_back(slice3(kf_r, 0));
    kalman_dict_.ry.push_back(slice3(kf_r, 3));
    kalman_dict_.rz.push_back(slice3(kf_r, 6));

    // Sim log
    sim_dict_.pos.push_back(x.position);
    sim_dict_.vel.push_back(x.velocity);
    sim_dict_.accel.push_back(x.acceleration);
    sim_dict_.ang_pos.push_back(x.orientation);
    sim_dict_.ang_vel.push_back(x.angular_velocity);
    sim_dict_.ang_accel.push_back(x.angular_acceleration);
    double t_prev = sim_dict_.time.empty() ? 0.0 : sim_dict_.time.back();
    sim_dict_.time.push_back(sim_dict_.time.empty() ? 0.0 : t_prev + dt);
    sim_dict_.flap_ext.push_back(flap_ext);
    sim_dict_.alpha.push_back(alpha);
    sim_dict_.rocket_total_mass.push_back(r_total_mass);
    sim_dict_.motor_mass.push_back(m_mass);

    fsm_state_.push_back(fsm_state);

    update_coeffs(std::sqrt(x.velocity[0]*x.velocity[0] +
                             x.velocity[1]*x.velocity[1] +
                             x.velocity[2]*x.velocity[2]));
}


// CSV export

static std::string d2s(double v) {
    return std::to_string(v);
}

std::vector<std::vector<std::string>> Rocket::to_csv() const {
    std::vector<std::vector<std::string>> record;
    std::size_t N = sim_dict_.time.size();

    for (std::size_t i = 0; i < N; ++i) {
        std::vector<std::string> row;
        row.push_back(d2s(sim_dict_.time[i]));
        for (double v : sim_dict_.pos[i])       row.push_back(d2s(v));
        for (double v : sim_dict_.vel[i])       row.push_back(d2s(v));
        for (double v : sim_dict_.accel[i])     row.push_back(d2s(v));
        for (double v : sim_dict_.ang_pos[i])   row.push_back(d2s(v));
        for (double v : sim_dict_.ang_vel[i])   row.push_back(d2s(v));
        for (double v : sim_dict_.ang_accel[i]) row.push_back(d2s(v));
        row.push_back(d2s(sim_dict_.alpha[i]));
        row.push_back(d2s(sim_dict_.rocket_total_mass[i]));
        row.push_back(d2s(sim_dict_.motor_mass[i]));
        row.push_back(d2s(sim_dict_.flap_ext[i]));
        row.push_back(d2s(sensor_dict_.baro_alt[i]));
        row.push_back(d2s(sensor_dict_.imu_accel_x[i]));
        row.push_back(d2s(sensor_dict_.imu_accel_y[i]));
        row.push_back(d2s(sensor_dict_.imu_accel_z[i]));
        row.push_back(d2s(sensor_dict_.imu_ang_pos_x[i]));
        row.push_back(d2s(sensor_dict_.imu_ang_pos_y[i]));
        row.push_back(d2s(sensor_dict_.imu_ang_pos_z[i]));
        row.push_back(d2s(sensor_dict_.imu_gyro_x[i]));
        row.push_back(d2s(sensor_dict_.imu_gyro_y[i]));
        row.push_back(d2s(sensor_dict_.imu_gyro_z[i]));
        row.push_back(d2s(sensor_dict_.imu_mag_x[i]));
        row.push_back(d2s(sensor_dict_.imu_mag_y[i]));
        row.push_back(d2s(sensor_dict_.imu_mag_z[i]));
        for (double v : kalman_dict_.x[i])     row.push_back(d2s(v));
        for (double v : kalman_dict_.y[i])     row.push_back(d2s(v));
        for (double v : kalman_dict_.z[i])     row.push_back(d2s(v));
        for (double v : kalman_dict_.cov_x[i]) row.push_back(d2s(v));
        for (double v : kalman_dict_.cov_y[i]) row.push_back(d2s(v));
        for (double v : kalman_dict_.cov_z[i]) row.push_back(d2s(v));
        for (double v : kalman_dict_.rx[i])    row.push_back(d2s(v));
        for (double v : kalman_dict_.ry[i])    row.push_back(d2s(v));
        for (double v : kalman_dict_.rz[i])    row.push_back(d2s(v));
        record.push_back(std::move(row));
    }
    return record;
}

std::vector<std::vector<std::string>> Rocket::to_midas_csv() const {
    std::vector<std::vector<std::string>> record;
    std::size_t N = sim_dict_.time.size();

    constexpr double launch_lat = kLaunchLatitudeDeg;
    constexpr double launch_lon = kLaunchLongitudeDeg;
    constexpr double launch_alt = 0.0;
    constexpr double DEG2RAD = M_PI / 180.0;

    for (std::size_t i = 0; i < N; ++i) {
        double t_ms = sim_dict_.time[i];// * 1000.0;
        const Vec3& pos = sim_dict_.pos[i];
        const Vec3& vel = sim_dict_.vel[i];

        double hg_ax = sensor_dict_.imu_accel_x[i];
        double hg_ay = sensor_dict_.imu_accel_y[i];
        double hg_az = sensor_dict_.imu_accel_z[i];
        double lg_ax = hg_ax * 0.1, lg_ay = hg_ay * 0.1, lg_az = hg_az * 0.1;

        double baro_alt  = sensor_dict_.baro_alt[i];
        double baro_pres = atm_ ? atm_->get_pressure(baro_alt) : 101325.0;
        double baro_temp = atm_ ? atm_->get_temperature(baro_alt) : 288.15;

        double yaw   = sensor_dict_.imu_ang_pos_x[i];
        double pitch = sensor_dict_.imu_ang_pos_y[i];
        double roll  = sensor_dict_.imu_ang_pos_z[i];

        double gx = sensor_dict_.imu_gyro_x[i];
        double gy = sensor_dict_.imu_gyro_y[i];
        double gz = sensor_dict_.imu_gyro_z[i];
        double mx = sensor_dict_.imu_mag_x[i];
        double my = sensor_dict_.imu_mag_y[i];
        double mz = sensor_dict_.imu_mag_z[i];

        double gps_lat   = launch_lat + pos[1] / 111320.0;
        double gps_lon   = launch_lon + pos[2] / (111320.0 * std::cos(launch_lat * DEG2RAD));
        double gps_alt   = pos[0] + launch_alt;
        double gps_speed = std::sqrt(vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2]);

        std::string fsm = (i < fsm_state_.size()) ? fsm_state_[i] : "STATE_IDLE";

        std::vector<std::string> row = {
            "orientation", "0", d2s(t_ms),
            d2s(lg_ax), d2s(lg_ay), d2s(lg_az),
            d2s(hg_ax), d2s(hg_ay), d2s(hg_az),
            d2s(baro_temp - 273.15), d2s(baro_pres / 100.0), d2s(baro_alt),
            "0","0","0","0", "4.0","0.1",
            d2s(gps_lat), d2s(gps_lon), d2s(gps_alt), d2s(gps_speed), "3","8",d2s(t_ms),
            d2s(mx), d2s(my), d2s(mz),
            "1","FULL_READING",
            d2s(yaw), d2s(pitch), d2s(roll),
            "0","0","0",
            d2s(gx), d2s(gy), d2s(gz),
            "0","0","0",
            d2s(hg_ax), d2s(hg_ay), d2s(hg_az),
            d2s(gx), d2s(gy), d2s(gz),
            d2s(mx), d2s(my), d2s(mz),
            d2s(baro_temp - 273.15), d2s(baro_pres / 100.0),
            "0","1","0","0","0",
            d2s(gx), d2s(gy), d2s(gz),
            d2s(lg_ax), d2s(lg_ay), d2s(lg_az),
            fsm,
            "0","0","0","0","0","0","0","0","0","0",
            "0","0","0","0","0","0","0"
        };
        record.push_back(std::move(row));
    }
    return record;
}
