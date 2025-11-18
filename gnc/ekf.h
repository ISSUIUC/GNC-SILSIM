#pragma once

#include "kalman_filter.h"
// #include "sensor_data.h"
// #include "Buffer.h"
#include "sensor_data.h" // for sim
#include "Buffer.h"      // for sim
#include "constants.h"
#include "aero_coeff.h"
#include "rotation.h"
#include "Madgwick.h"

#define NUM_STATES 9
#define NUM_SENSOR_INPUTS 4
#define ALTITUDE_BUFFER_SIZE 10

// Number of entries for aerodynamic data table
#define AERO_DATA_SIZE (sizeof(aero_data) / sizeof(aero_data[0]))

class EKF : public KalmanFilter<NUM_STATES, NUM_SENSOR_INPUTS>
{
public:
    EKF();
    void initialize(RocketSystems *args) override;
    // void priori();
    void priori(float dt, Orientation &orientation, FSMState fsm);
    void update(Barometer barometer, Acceleration acceleration, Orientation orientation, Magnetometer magnetometer, FSMState state, GPS &gps, float dt) override;

    void setQ(float dt, float sd);
    void setF(float dt, float w_x, float w_y, float w_z, FSMState fsm, float v_x, float v_y, float v_z);

    KalmanData getState() override;
    void setState(KalmanState state) override;
    void compute_mass(FSMState fsm);
    void compute_kalman_gain();
    void compute_gps_inputs(GPS &gps, FSMState fsm);
    void reference_GPS(GPS &gps, FSMState fsm);
    // std::vector<float> ECEF(float lat, float lon, float alt);

    void compute_drag_coeffs(float vel_magnitude_ms);
    void compute_x_dot(float dt, Orientation &orientation, FSMState fsm, Eigen::Matrix<float, 9, 1> &xdot);

    void getThrust(float timestamp, const euler_t &angles, FSMState FSM_state, Eigen::Vector3f &thrust_out);

    void tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation, Magnetometer magnetometer, FSMState state, GPS &gps);

    bool should_reinit = false;
    float current_vel = 0.0f;

private:
    float s_dt = 0.05f;
    float spectral_density_ = 13.0f;
    float kalman_apo = 0;
    float Ca = 0;
    float Cn = 0;
    float Wind_alpha = 0.85f;
    float Cp = 0;
    float curr_mass_kg = mass_full; //(kg) Sustainer + Booster, but value changes over time.
    float gps_latitude_original;
    float gps_longitude_original;
    float gps_latitude_last;  // we don't want to update gps if it's the same as the previously updated value.
    float gps_longitude_last; // we don't want to update gps if it's the same as the previously updated value.

    // Eigen::Matrix<float,3,1> gravity = Eigen::Matrix<float,3,1>::Zero();
    KalmanState kalman_state;
    FSMState last_fsm = FSMState::STATE_IDLE;
    float stage_timestamp = 0;

    // AHRS estimator
    Madgwick ahrs;

    Eigen::Matrix<float, 3, 1> init_accel = Eigen::Matrix<float, 3, 1>::Zero();
    Buffer<float, ALTITUDE_BUFFER_SIZE> alt_buffer;
    KalmanData state;
};

extern EKF ekf;