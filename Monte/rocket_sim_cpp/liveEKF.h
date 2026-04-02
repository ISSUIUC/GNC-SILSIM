#pragma once
#include "../../gnc/ekf.h"
#include "../../gnc/systems.h"
//#include "../../gnc/sensor_data.h"

#include "sensor_data.h"
//#include "rocket_state.h"

#include <string>
#include <vector>


struct KalmanStep {
    std::vector<double> state;
    std::vector<double> cov;
    std::vector<double> residual;
};

class LiveEKF {
public:
    LiveEKF() {
        ekf_.initialize(&rocket_systems_);
    }

    FSMState to_fsm(const std::string& fsm_name) {
    if (fsm_name == "STATE_SAFE") return FSMState::STATE_SAFE;
    if (fsm_name == "STATE_PYRO_TEST") return FSMState::STATE_PYRO_TEST;
    if (fsm_name == "STATE_IDLE") return FSMState::STATE_IDLE;
    if (fsm_name == "STATE_FIRST_BOOST") return FSMState::STATE_FIRST_BOOST;
    if (fsm_name == "STATE_BURNOUT") return FSMState::STATE_BURNOUT;
    if (fsm_name == "STATE_COAST") return FSMState::STATE_COAST;
    if (fsm_name == "STATE_APOGEE") return FSMState::STATE_APOGEE;
    if (fsm_name == "STATE_DROGUE_DEPLOY") return FSMState::STATE_DROGUE_DEPLOY;
    if (fsm_name == "STATE_DROGUE") return FSMState::STATE_DROGUE;
    if (fsm_name == "STATE_MAIN_DEPLOY") return FSMState::STATE_MAIN_DEPLOY;
    if (fsm_name == "STATE_MAIN") return FSMState::STATE_MAIN;
    if (fsm_name == "STATE_LANDED") return FSMState::STATE_LANDED;
    if (fsm_name == "STATE_SUSTAINER_IGNITION") return FSMState::STATE_SUSTAINER_IGNITION;
    if (fsm_name == "STATE_SECOND_BOOST") return FSMState::STATE_SECOND_BOOST;
    if (fsm_name == "STATE_FIRST_SEPARATION") return FSMState::STATE_FIRST_SEPARATION;

    return FSMState::STATE_IDLE;
    }


    KalmanStep tick(double dt,
                    double t_ms,
                    double baro_alt,
                    const Vec3& accel,
                    const Vec3& gyro,
                    const Vec3& bno,
                    const std::string& fsm_name,
                    const Vec3& pos_world) {
        Barometer baro{};
        baro.altitude = static_cast<float>(baro_alt);

        HighGData highg{};
        highg.ax = static_cast<float>(accel[0]);
        highg.ay = static_cast<float>(accel[1]);
        highg.az = static_cast<float>(accel[2]);

        Acceleration current_accel{};
        current_accel.ax = highg.ax;
        current_accel.ay = highg.ay;
        current_accel.az = highg.az;

        Orientation orientation{};
        orientation.has_data = true;
        orientation.yaw = static_cast<float>(bno[0]);
        orientation.pitch = static_cast<float>(bno[1]);
        orientation.roll = static_cast<float>(bno[2]);
        orientation.angular_velocity.vx = static_cast<float>(gyro[0]);
        orientation.angular_velocity.vy = static_cast<float>(gyro[1]);
        orientation.angular_velocity.vz = static_cast<float>(gyro[2]);

        GPS gps{};
        gps.altitude = static_cast<float>(pos_world[0]);
        gps.time = static_cast<float>(t_ms);

        FSMState fsm = to_fsm(fsm_name);
        Eigen::Quaternionf q;

        rocket_systems_.rocket_data.barometer.push(baro);
        rocket_systems_.rocket_data.orientation.push(orientation);
        rocket_systems_.rocket_data.high_g.push(highg);
        rocket_systems_.rocket_data.fsm_state.push(fsm);
        rocket_systems_.rocket_data.gps.push(gps);

        ekf_.tick(static_cast<float>(dt), 13.0f, baro, current_accel, q, fsm, gps);

        const KalmanData k = ekf_.getState();
        return {
            {
                k.position.px, k.position.py, k.position.pz,
                k.velocity.vx, k.velocity.vy, k.velocity.vz,
                k.acceleration.ax, k.acceleration.ay, k.acceleration.az
            },
            std::vector<double>(9, 0.0),
            std::vector<double>(9, 0.0)
        };
    }

private:
    EKF ekf_;
    RocketSystems rocket_systems_;
};
