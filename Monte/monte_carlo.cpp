#include "monte_carlo.h"

#include "rocket_sim_cpp/rocket.h"
#include "rocket_sim_cpp/atmosphere.h"
#include "rocket_sim_cpp/liveEKF.h"
#include "controller.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::size_t kColumns = 52;
constexpr double kDesiredApogee = 3048.0;

using Row = std::array<double, kColumns>;
using SimData = std::vector<Row>;

struct SensorValues {
    double baro_alt{};
    Vec3 accel{};
    Vec3 gyro{};
    Vec3 bno{};
};

struct Rk4Result {
    RocketState state;
    double alpha{};
};

std::mt19937& rng() {
    static std::mt19937 gen(std::random_device{}());
    return gen;
}

double uniform(double lo, double hi) {
    std::uniform_real_distribution<double> dist(lo, hi);
    return dist(rng());
}

double round3(double value) {
    return std::round(value * 1000.0) / 1000.0;
}

Vec3 round3(Vec3 value) {
    for (double& v : value) {
        v = round3(v);
    }
    return value;
}

RocketState round3(RocketState value) {
    value.position = round3(value.position);
    value.velocity = round3(value.velocity);
    value.acceleration = round3(value.acceleration);
    value.orientation = round3(value.orientation);
    value.angular_velocity = round3(value.angular_velocity);
    value.angular_acceleration = round3(value.angular_acceleration);
    return value;
}

Vec3 add(const Vec3& a, const Vec3& b, double scale = 1.0) {
    return {a[0] + scale * b[0], a[1] + scale * b[1], a[2] + scale * b[2]};
}

double norm(const Vec3& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

std::vector<double> zero9() {
    return std::vector<double>(9, 0.0);
}

std::vector<double> round3(std::vector<double> values) {
    for (double& value : values) {
        value = round3(value);
    }
    return values;
}

std::vector<double> state_or_zero(const KalmanStep& step) {
    if (step.state.size() >= 9) {
        return step.state;
    }
    return zero9();
}

Rocket::Config default_stage_config() { // look up 
    Rocket::Config cfg;
    cfg.structure_cm = 1.48;
    cfg.combined_cm = 1.03;
    cfg.combined_cp = 0.63;
    cfg.dry_mass = 30.0;
    cfg.radius = 0.0508;
    cfg.length = 6.68;
    cfg.rasaero_lookup_file = "LookUp/RASAero.csv";
    cfg.motor_cm = 2.5;
    cfg.motor_impulse = 10000.0;
    cfg.motor_mass = 5.0;
    cfg.motor_delay = 0.0;
    cfg.motor_lookup_file = "LookUp/m2500.csv";
    cfg.flap_max_ext_length = 0.0178;
    cfg.flap_max_ext_spd = 0.001;
    return cfg;
}

Rocket::SensorConfig default_sensor_config() { // look up
    Rocket::SensorConfig sc;
    sc.high_g.RMS = 1.9;
    sc.gyro.RMS = 75.0;
    sc.barometer.RMS = 0.012;
    sc.bno.error = 2.5 / 3.0;
    return sc;
}

SensorValues get_sensor_data(Rocket& rocket, const RocketState& x, const Rocket::SensorConfig& sc) {
    return {
        rocket.get_barometer_data(x, sc),
        rocket.get_accelerometer_data(x, sc),
        rocket.get_gyro_data(x, sc),
        rocket.get_bno_orientation(x, sc)
    };
}

Rk4Result rk4(Rocket& rocket, const RocketState& y0, double dt, double timestamp, double flap_ext, bool density_noise) {
    auto get_f = [&](const Vec3& pos, const Vec3& vel, const Vec3& ang_pos, const Vec3& ang_vel) {
        RocketState tmp;
        tmp.position = pos; // runge kutta 4th order integrator for state propagation
        tmp.velocity = vel;
        tmp.orientation = ang_pos;
        tmp.angular_velocity = ang_vel;

        auto [fm, alpha] = rocket.forces->get_force(tmp, flap_ext, timestamp, 0.0, 0.0, 0.0, density_noise);
        const double mass = rocket.get_rocket_total_mass(timestamp);
        const Vec3 i_inv = rocket.I_inv(mass);

        RocketState deriv;
        deriv.position = vel;
        deriv.velocity = {fm[0][0] / mass, fm[0][1] / mass, fm[0][2] / mass};
        deriv.orientation = ang_vel;
        deriv.angular_velocity = {fm[1][0] * i_inv[0], fm[1][1] * i_inv[1], fm[1][2] * i_inv[2]};
        return std::pair<RocketState, double>{deriv, alpha};
    };

    auto [k1, ignored1] = get_f(y0.position, y0.velocity, y0.orientation, y0.angular_velocity);
    (void)ignored1;
    auto [k2, ignored2] = get_f(add(y0.position, k1.position, dt / 2.0),
                                add(y0.velocity, k1.velocity, dt / 2.0),
                                add(y0.orientation, k1.orientation, dt / 2.0),
                                add(y0.angular_velocity, k1.angular_velocity, dt / 2.0));
    (void)ignored2;
    auto [k3, ignored3] = get_f(add(y0.position, k2.position, dt / 2.0),
                                add(y0.velocity, k2.velocity, dt / 2.0),
                                add(y0.orientation, k2.orientation, dt / 2.0),
                                add(y0.angular_velocity, k2.angular_velocity, dt / 2.0));
    (void)ignored3;
    auto [k4, alpha] = get_f(add(y0.position, k3.position, dt),
                             add(y0.velocity, k3.velocity, dt),
                             add(y0.orientation, k3.orientation, dt),
                             add(y0.angular_velocity, k3.angular_velocity, dt));

    auto combine = [dt](const Vec3& y, const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
        return Vec3{
            y[0] + (dt / 6.0) * (a[0] + 2.0 * b[0] + 2.0 * c[0] + d[0]),
            y[1] + (dt / 6.0) * (a[1] + 2.0 * b[1] + 2.0 * c[1] + d[1]),
            y[2] + (dt / 6.0) * (a[2] + 2.0 * b[2] + 2.0 * c[2] + d[2])
        };
    };

    RocketState next;
    next.position = combine(y0.position, k1.position, k2.position, k3.position, k4.position);
    next.velocity = combine(y0.velocity, k1.velocity, k2.velocity, k3.velocity, k4.velocity);
    next.orientation = combine(y0.orientation, k1.orientation, k2.orientation, k3.orientation, k4.orientation);
    next.angular_velocity = combine(y0.angular_velocity, k1.angular_velocity, k2.angular_velocity, k3.angular_velocity, k4.angular_velocity);

    auto [final_deriv, alpha_f] = get_f(next.position, next.velocity, next.orientation, next.angular_velocity);
    next.acceleration = final_deriv.velocity;
    next.angular_acceleration = final_deriv.angular_velocity;

    return {next, alpha_f == 0.0 ? alpha : alpha_f};
}

Row make_row(const RocketState& x,
             double time_stamp,
             double baro_alt,
             const Vec3& accel,
             const Vec3& bno_ang_pos,
             const Vec3& gyro,
             const std::vector<double>& kalman_filter,
             const std::vector<double>& kalman_filter_r,
             double alpha,
             double apogee_estimation,
             double rocket_total_mass,
             double motor_mass,
             double flap_ext) {
    Row row{};
    std::size_t i = 0;
    auto push = [&](double v) { row[i++] = v; };
    auto push3 = [&](const Vec3& v) { push(v[0]); push(v[1]); push(v[2]); };
    auto push_vec = [&](const std::vector<double>& v, std::size_t start) {
        for (std::size_t j = 0; j < 3; ++j) {
            push(start + j < v.size() ? v[start + j] : 0.0);
        }
    };

    push3(x.position);
    push3(x.velocity);
    push3(x.acceleration);
    push3(x.orientation);
    push3(x.angular_velocity);
    push3(x.angular_acceleration);
    push(time_stamp);
    push(flap_ext);
    push(alpha);
    push(rocket_total_mass);
    push(motor_mass);
    push(baro_alt);
    push3(accel);
    push3(bno_ang_pos);
    push3(gyro);
    push(apogee_estimation);
    push_vec(kalman_filter, 0);
    push_vec(kalman_filter, 3);
    push_vec(kalman_filter, 6);
    push_vec(kalman_filter_r, 0);
    push_vec(kalman_filter_r, 3);
    push_vec(kalman_filter_r, 6);

    return row;
}

void append_to_array(SimData& data,
                     const RocketState& x,
                     double time_stamp,
                     double baro_alt,
                     const Vec3& accel,
                     const Vec3& bno_ang_pos,
                     const Vec3& gyro,
                     const std::vector<double>& kalman_filter,
                     const std::vector<double>& kalman_filter_r,
                     double alpha,
                     double apogee_estimation,
                     double rocket_total_mass,
                     double motor_mass,
                     double flap_ext) {
    data.push_back(make_row(round3(x),
                            round3(time_stamp),
                            round3(baro_alt),
                            round3(accel),
                            round3(bno_ang_pos),
                            round3(gyro),
                            kalman_filter,
                            kalman_filter_r,
                            round3(alpha),
                            round3(apogee_estimation),
                            round3(rocket_total_mass),
                            round3(motor_mass),
                            round3(flap_ext)));
}

void pad_or_trim(SimData& data, int target_size) {
    if (target_size < 0) {
        throw std::invalid_argument("target_size must be non-negative");
    }

    const auto desired = static_cast<std::size_t>(target_size);
    if (data.size() > desired) {
        data.resize(desired);
    } else if (!data.empty()) {
        while (data.size() < desired) {
            data.push_back(data.back());
        }
    }
}

void write_npy(const std::filesystem::path& path, const SimData& data) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        throw std::runtime_error("cannot open output file: " + path.string());
    }

    const std::string magic = "\x93NUMPY";
    out.write(magic.data(), static_cast<std::streamsize>(magic.size()));
    const char major = 1;
    const char minor = 0;
    out.put(major);
    out.put(minor);

    std::string header = "{'descr': '<f8', 'fortran_order': False, 'shape': (" +
                         std::to_string(data.size()) + ", " + std::to_string(kColumns) + "), }";
    const std::size_t preamble = 10;
    const std::size_t padding = 16 - ((preamble + header.size() + 1) % 16);
    header.append(padding, ' ');
    header.push_back('\n');

    const auto header_len = static_cast<std::uint16_t>(header.size());
    out.put(static_cast<char>(header_len & 0xff));
    out.put(static_cast<char>((header_len >> 8) & 0xff));
    out.write(header.data(), static_cast<std::streamsize>(header.size()));

    for (const Row& row : data) {
        out.write(reinterpret_cast<const char*>(row.data()), static_cast<std::streamsize>(row.size() * sizeof(double)));
    }
}

void ensure_run_folders(const std::filesystem::path& run_folder) {
    std::filesystem::create_directories(run_folder / "figures");
    std::filesystem::create_directories(run_folder / "SimData");
}

void print_shape_and_time(const SimData& data, const std::chrono::steady_clock::time_point& start) {
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    std::cout << "(" << data.size() << ", " << kColumns << ")\n";
    std::cout << "Time: " << std::fixed << std::setprecision(2) << elapsed << "\n";
}

} // namespace

void simulator(RocketState& x0,
               double dt,
               int sample_number,
               const std::string& run_folder,
               int target_size,
               bool nominal,
               float wind_direction_variance_mean,
               float wind_direction_variance_stddev,
               float wind_magnitude_variance_mean,
               float wind_magnitude_variance_stddev,
               bool enable_direction_variance,
               bool enable_magnitude_variance,
               std::array<float, 3> nominal_wind_direction,
               float nominal_wind_magnitude) {
    const auto t_start = std::chrono::steady_clock::now();

    if (nominal) {
        enable_direction_variance = false;
        enable_magnitude_variance = false;
    }

    Vec3 wind_direction = {
        static_cast<double>(nominal_wind_direction[0]),
        static_cast<double>(nominal_wind_direction[1]),
        static_cast<double>(nominal_wind_direction[2])
    };

    Atmosphere atm(wind_direction_variance_mean,
                   wind_direction_variance_stddev,
                   wind_magnitude_variance_mean,
                   wind_magnitude_variance_stddev,
                   enable_direction_variance,
                   enable_magnitude_variance,
                   wind_direction,
                   nominal_wind_magnitude);

    Rocket::Config cfg = default_stage_config();
    Rocket rocket(dt, x0, cfg, &atm, {}, "LookUp/ekf_cd_test.CSV", "");
    Rocket::SensorConfig sensor_config = default_sensor_config();
    LiveEKF ekf;

    SimData sim_data;
    RocketState x = x0;
    // if (!nominal) {
    //     x.position[0] *= uniform(0.9, 1.1);
    //     x.position[1] *= uniform(0.9, 1.1);
    //     x.position[2] *= uniform(0.9, 1.1);
    // }
    if (!nominal) {
    // Random launch angle / attitude offsets
    x.orientation[0] += uniform(-0.05, 0.05);  // roll
    x.orientation[1] += uniform(-0.15, 0.15);  // pitch
    x.orientation[2] += uniform(-0.15, 0.15);  // yaw

    // Optional initial lateral velocity kick
    x.velocity[1] += uniform(-2.0, 2.0);
    x.velocity[2] += uniform(-2.0, 2.0);
    }


    double time_stamp = 0.0;

    while (time_stamp < cfg.motor_delay) {
        time_stamp += dt;
        const SensorValues sensors = get_sensor_data(rocket, x, sensor_config);
        KalmanStep k = ekf.tick(dt, time_stamp * 1000.0, sensors.baro_alt, sensors.accel, sensors.gyro, sensors.bno, "STATE_IDLE", x.position);
        std::vector<double> current_state = state_or_zero(k);
        std::vector<double> current_state_r = zero9();
        current_state[1] = 0.0;
        current_state[2] = 0.0;

        append_to_array(sim_data,
                        x,
                        time_stamp,
                        sensors.baro_alt,
                        sensors.accel,
                        sensors.bno,
                        sensors.gyro,
                        current_state,
                        current_state_r,
                        0.0,
                        current_state[0],
                        rocket.rocket_total_mass,
                        rocket.motor_mass_val,
                        0.0);
    }

    bool start = true;
    Controller controller(0.0002, 0.0, 0.0, dt, kDesiredApogee, cfg.flap_max_ext_length, cfg.flap_max_ext_spd);
    rocket.get_motor()->ignite(cfg.motor_delay);

    while (x.velocity[0] >= 0.0 || start) {
        if (start) {
            start = false;
        }

        const SensorValues sensors = get_sensor_data(rocket, x, sensor_config);
        KalmanStep k = ekf.tick(dt, time_stamp * 1000.0, sensors.baro_alt, sensors.accel, sensors.gyro, sensors.bno, "STATE_COAST", x.position);
        std::vector<double> current_state = state_or_zero(k);
        std::vector<double> current_state_r = zero9();

        const double apogee_est = current_state[0] > x.position[0] ? current_state[0] : x.position[0];
        const bool control = time_stamp > cfg.motor_delay && norm(rocket.get_motor()->get_thrust(time_stamp)) <= 0.0;
        const double flap_ext = controller.get_flap_extension(control, apogee_est);

        rocket.set_motor_mass(time_stamp);
        Rk4Result step = rk4(rocket, x, dt, time_stamp, flap_ext, true);
        x = step.state;
        time_stamp += dt;

        append_to_array(sim_data,
                        x,
                        time_stamp,
                        sensors.baro_alt,
                        sensors.accel,
                        sensors.bno,
                        sensors.gyro,
                        current_state,
                        current_state_r,
                        step.alpha,
                        apogee_est,
                        rocket.rocket_total_mass,
                        rocket.motor_mass_val,
                        flap_ext);
    }

    pad_or_trim(sim_data, target_size);
    ensure_run_folders(run_folder);

    const std::filesystem::path filename = nominal ? "nominal.npy" : ("sim_data_" + std::to_string(sample_number) + ".npy");
    write_npy(std::filesystem::path(run_folder) / "SimData" / filename, sim_data);
    print_shape_and_time(sim_data, t_start);
}

void run(RocketState& x0,
         double dt,
         int samples,
         const std::string& run_folder,
         int target_size,
         bool clear_contents,
         float wind_direction_variance_mean,
         float wind_direction_variance_stddev,
         float wind_magnitude_variance_mean,
         float wind_magnitude_variance_stddev,
         bool enable_direction_variance,
         bool enable_magnitude_variance,
         std::array<float, 3> nominal_wind_direction,
         float nominal_wind_magnitude) {
    const std::filesystem::path out_dir(run_folder);
    if (clear_contents && std::filesystem::exists(out_dir)) {
        std::filesystem::remove_all(out_dir);
    }
    ensure_run_folders(out_dir);

    std::cout << "Calculating nominal trajectory\n";
    simulator(x0,
              dt,
              0,
              run_folder,
              target_size,
              true,
              wind_direction_variance_mean,
              wind_direction_variance_stddev,
              wind_magnitude_variance_mean,
              wind_magnitude_variance_stddev,
              false,
              false,
              nominal_wind_direction,
              nominal_wind_magnitude);

    std::cout << "Running Monte Carlo simulations\n";
    for (int i = 0; i < samples; ++i) {
        std::cout << "Running sample " << (i + 1) << " of " << samples << "\n";
        simulator(x0,
                  dt,
                  i,
                  run_folder,
                  target_size,
                  false,
                  wind_direction_variance_mean,
                  wind_direction_variance_stddev,
                  wind_magnitude_variance_mean,
                  wind_magnitude_variance_stddev,
                  enable_direction_variance,
                  enable_magnitude_variance,
                  nominal_wind_direction,
                  nominal_wind_magnitude);
    }

    std::cout << "Done\n";
}


