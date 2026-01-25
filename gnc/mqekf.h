#pragma once

#include "kalman_filter.h" 
#include "sensor_data.h" // for sim
#include "Buffer.h" // for sim
#include "constants.h"
#include "aero_coeff.h"
#include "rotation.h"

#define NUM_STATES 9
#define NUM_SENSOR_INPUTS 4
#define ALTITUDE_BUFFER_SIZE 10


// Number of entries for aerodynamic data table
#define AERO_DATA_SIZE (sizeof(aero_data) / sizeof(aero_data[0]))

class EKF : public KalmanFilter<NUM_STATES, NUM_SENSOR_INPUTS>
{
public:
    EKF();
    void initialize(RocketSystems* args) override;
    void priori(float dt, Orientation &orientation, FSMState fsm); 
    void update(Barometer barometer, Acceleration acceleration, Orientation orientation, FSMState state, GPS &gps) override;

    void setQ(float dt, float sd);
    void setF(float dt, float w_x, float w_y, float w_z, FSMState fsm, float v_x,float v_y, float v_z); 

    KalmanData getState() override;
    void setState(KalmanState state) override;
    void compute_kalman_gain();
   
    void tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation, FSMState state, GPS &gps);
   


private:
    float s_dt = 0.05f;
    float spectral_density_ = 13.0f;
   
    KalmanState kalman_state;
    FSMState last_fsm = FSMState::STATE_IDLE;
    float stage_timestamp = 0;

    Eigen::Matrix<float, 3, 1> init_accel = Eigen::Matrix<float, 3, 1>::Zero();
    Buffer<float, ALTITUDE_BUFFER_SIZE> alt_buffer;
    KalmanData state;
};



extern EKF ekf;

