#pragma once

#include "kalman_filter.h"
//#include "sensor_data.h"
//#include "Buffer.h" 
#include "sensor_data.h" // for sim
#include "Buffer.h" // for sim
#include "constants.h"
#include "aero_coeff.h"
#include "rotation.h"

#define NUM_STATES 6  // [x, vx, y, vy, z, vz] - position and velocity only
#define NUM_SENSOR_INPUTS 4  // barometer, accel_x, accel_y, accel_z
#define NUM_CONTROL_INPUTS 3  // acceleration as control input [ax, ay, az]
#define ALTITUDE_BUFFER_SIZE 10


// Number of entries for aerodynamic data table
#define AERO_DATA_SIZE (sizeof(aero_data) / sizeof(aero_data[0]))

class EKF : public KalmanFilter<NUM_STATES, NUM_SENSOR_INPUTS>
{
public:
    EKF();
    void initialize(RocketSystems* args) override;
    // void priori();
    void priori(float dt, Orientation &orientation, FSMState fsm, Eigen::Matrix<float, NUM_CONTROL_INPUTS, 1> &u_control); 
    void update(Barometer barometer, Acceleration acceleration, Orientation orientation, FSMState state, GPS &gps) override;

    void setQ(float dt, float sd);
    void setF(float dt, float w_x, float w_y, float w_z, FSMState fsm, float v_x,float v_y, float v_z);
    void setB(float dt);  // Set control input matrix for acceleration 

    // void BodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> &body_vec);
    // void GlobalToBody(euler_t angles, Eigen::Matrix<float, 3, 1> &global_vec);

    KalmanData getState() override;
    void setState(KalmanState state) override;
    void compute_mass(FSMState fsm);
    void compute_kalman_gain();
    void compute_gps_inputs(GPS &gps, FSMState fsm);
    void reference_GPS(GPS &gps, FSMState fsm); 

    // void compute_drag_coeffs(float vel_magnitude_ms);
    // void compute_x_dot(float dt, Orientation &orientation, FSMState fsm, Eigen::Matrix<float, 9, 1> &xdot);

    // void getThrust(float timestamp, const euler_t& angles, FSMState FSM_state, Eigen::Vector3f& thrust_out);

    void tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation, FSMState state, GPS &gps);
   
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
    std::vector<float> starting_gps;    // latitude, longitude, altitude
    std::vector<float> starting_ecef;   // x, y, z

    // Eigen::Matrix<float,3,1> gravity = Eigen::Matrix<float,3,1>::Zero();
    KalmanState kalman_state;
    FSMState last_fsm = FSMState::STATE_IDLE;
    float stage_timestamp = 0;
    
    // Track how long we've been in LANDED state to avoid false positives
    float landed_state_duration = 0.0f;
    bool was_landed_last = false;

    Eigen::Matrix<float, 3, 1> init_accel = Eigen::Matrix<float, 3, 1>::Zero();
    Buffer<float, ALTITUDE_BUFFER_SIZE> alt_buffer;
    KalmanData state;
    
    // Control input matrix for acceleration [ax, ay, az]
    Eigen::Matrix<float, NUM_STATES, NUM_CONTROL_INPUTS> B_control;
    
    // GPS reference coordinates
    float gps_latitude_original = 0.0f;
    float gps_longitude_original = 0.0f;
    float gps_latitude_last = 0.0f;
    float gps_longitude_last = 0.0f;
};



extern EKF ekf;