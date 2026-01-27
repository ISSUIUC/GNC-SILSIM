#include "ekf.h"
#include "fsm_states.h" // for sim
#include <iostream>

/**
 * The following program is the University of Illinois' Extended Kalman Filter, utilized for state estimation of
 * our single and multistage rockets. The program was developed since 2023 and had its first successful run (Booster) on LUNA, October 2025.
 *
 * 2025-2026 GNC EKF Team: Divij Garg (Senior), Shishir Bhatta (Senior), Keshav Balaji (Senior), Tanish Mittal (Freshman), Amir Noormohammad (Aero MS), Ahmed Khan (Sophmore)
 * Maximilian Kulasik (Sophomore), Kailen Patel (Sophomore)
 */
extern const std::map<float, float> O5500X_data;
extern const std::map<float, float> M685W_data;
extern const std::map<std::string, std::map<float, float>> motor_data;

EKF::EKF() : KalmanFilter()
{
    state = KalmanData();
}

// /**
//  * THIS IS A PLACEHOLDER FUNCTION SO WE CAN ABSTRACT FROM `kalman_filter.h`
//  */
// void EKF::priori() {};

/**
 * @brief Sets altitude by averaging 30 barometer measurements taken 100 ms
 * apart
 *
 * The following for loop takes a series of barometer measurements on start
 * up and takes the average of them in order to initialize the kalman filter
 * to the correct initial barometric altitude. This is done so that the
 * kalman filter takes minimal time to converge to an accurate state
 * estimate. This process is significantly faster than allowing the state as
 * letting the filter to converge to the correct state can take up to 3 min.
 * This specific process was used because the barometric altitude will
 * change depending on the weather and thus, the initial state estimate
 * cannot be hard coded. A GPS altitude may be used instead but due to GPS
 * losses during high speed/high altitude flight, it is inadvisable with the
 * current hardware to use this as a solution. Reference frames should also
 * be kept consistent (do not mix GPS altitude and barometric).
 *
 */
void EKF::initialize(RocketSystems *args)
{
    Orientation orientation = args->rocket_data.orientation.getRecentUnsync();
    float sum = 0;

    for (int i = 0; i < 30; i++)
    {
        Barometer barometer = args->rocket_data.barometer.getRecent();
        LowGData initial_accelerometer = args->rocket_data.low_g.getRecent();
        Acceleration accelerations = {
            .ax = initial_accelerometer.ax,
            .ay = initial_accelerometer.ay,
            .az = initial_accelerometer.az};
        sum += barometer.altitude;
    }

    euler_t euler = orientation.getEuler();

    // set x_k
    x_k.setZero();
    x_k(0, 0) = sum / 30;
    x_k(3, 0) = 0;
    x_k(6, 0) = 0;

    F_mat.setZero(); // Initialize with zeros

    setQ(s_dt, spectral_density_);

    // set H
    H.setZero();
    H(0, 0) = 1; // x
    H(1, 3) = 1; // y
    H(2, 6) = 1; // z
    H(3, 2) = 1; // x accel
    H(4, 5) = 1; // y accel
    H(5, 8) = 1; // z accel

    P_k.setZero();
    P_k.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * 1e-2f; // x block (pos,vel,acc)
    P_k.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity() * 1e-2f; // y block
    P_k.block<3, 3>(6, 6) = Eigen::Matrix3f::Identity() * 1e-2f; // z block

    // set Measurement Noise Matrix
    R(0, 0) = 1.9;
    R(1, 1) = 0.1;
    R(2, 2) = 0.1;
    R(3, 3) = 1.9;
    R(4, 4) = 1.9;
    R(5, 5) = 1.9;
}

/**
 * @brief Estimates current state of the rocket without current sensor data
 *
 * The priori step of the Kalman filter is used to estimate the current state
 * of the rocket without knowledge of the current sensor data. In other words,
 * it extrapolates the state at time n+1 based on the state at time n.
 */

void EKF::priori(float dt, Orientation &orientation, FSMState fsm)
{
    x_priori = F_mat * x_k;
    setF(dt, 0, 0, 0, fsm, 0, 0, 0);
    P_priori = (F_mat * P_k * F_mat.transpose()) + Q;
}

/**
 * @brief Update state estimate with current sensor data
 *
 * After receiving new sensor data, the Kalman filter updates the state estimate.
 * After updating the gain, the state estimate is updated.
 *
 */
void EKF::update(Barometer barometer, Acceleration acceleration, Orientation orientation, FSMState FSM_state, GPS &gps)
{
    // if on pad -> take last 10 barometer measurements for init state
    if (FSM_state == FSMState::STATE_IDLE)
    {
        float sum = 0;
        float data[10];
        alt_buffer.readSlice(data, 0, 10);
        for (float i : data)
        {
            sum += i;
        }
        KalmanState kalman_state = (KalmanState){sum / 10.0f, 0, 0, 0, 0, 0, 0, 0, 0};
        setState(kalman_state);
    }

    // Kalman Gain
    compute_kalman_gain();

    // Sensor Measurements
    Eigen::Matrix<float, 3, 1> sensor_accel_global_g = Eigen::Matrix<float, 3, 1>(Eigen::Matrix<float, 3, 1>::Zero());

    // accouting for sensor bias and coordinate frame transforms
    (sensor_accel_global_g)(0, 0) = acceleration.ax + 0.045;
    (sensor_accel_global_g)(1, 0) = acceleration.ay - 0.065;
    (sensor_accel_global_g)(2, 0) = acceleration.az - 0.06;

    euler_t angles_rad = orientation.getEuler();

    BodyToGlobal(angles_rad, sensor_accel_global_g);

    float g_ms2;
    if ((FSM_state > FSMState::STATE_IDLE))
    {
        g_ms2 = gravity_ms2;
    }
    else
    {
        g_ms2 = 0;
    }

    // acceloremeter reports values in g's and measures specific force
    y_k(3, 0) = ((sensor_accel_global_g)(0)) * g_ms2;
    y_k(4, 0) = ((sensor_accel_global_g)(1)) * g_ms2;
    y_k(5, 0) = ((sensor_accel_global_g)(2)) * g_ms2;

    y_k(0, 0) = barometer.altitude; // meters

    std::vector<float> gps_data = compute_gps_inputs(gps, FSM_state); // testing GPS inputs
    if (gps_data[0] == 0.0f && gps_data[1] == 0.0f)
    {
        y_k(1, 0) = x_priori(3, 0); // meters
        y_k(2, 0) = x_priori(6, 0); // meters
        // std::cout<<"ran"<<std::endl;
    }
    else
    {
        y_k(1, 0) = gps_data[0];
        y_k(2, 0) = gps_data[1];
    }

    // # Posteriori Update
    Eigen::Matrix<float, 9, 9> identity = Eigen::Matrix<float, 9, 9>::Identity();
    x_k = x_priori + K * (y_k - (H * x_priori));
    P_k = (identity - K * H) * P_priori;

    kalman_state.state_est_pos_x = x_k(0, 0);
    kalman_state.state_est_vel_x = x_k(1, 0);
    kalman_state.state_est_accel_x = x_k(2, 0);
    kalman_state.state_est_pos_y = x_k(3, 0);
    kalman_state.state_est_vel_y = x_k(4, 0);
    kalman_state.state_est_accel_y = x_k(5, 0);
    kalman_state.state_est_pos_z = x_k(6, 0);
    kalman_state.state_est_vel_z = x_k(7, 0);
    kalman_state.state_est_accel_z = x_k(8, 0);

    state.position = (Position){kalman_state.state_est_pos_x, kalman_state.state_est_pos_y, kalman_state.state_est_pos_z};
    state.velocity = (Velocity){kalman_state.state_est_vel_x, kalman_state.state_est_vel_y, kalman_state.state_est_vel_z};
    state.acceleration = (Acceleration){kalman_state.state_est_accel_x, kalman_state.state_est_accel_y, kalman_state.state_est_accel_z};
}

/**
 * @brief Run Kalman filter calculations as long as FSM has passed IDLE
 *
 * @param dt Time step calculated by the Kalman Filter Thread
 * @param sd Spectral density of the noise
 * @param &barometer Data of the current barometer
 * @param acceleration Current acceleration
 * @param &orientation Current orientation
 * @param current_state Current FSM_state
 */
void EKF::tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation, FSMState FSM_state, GPS &gps)
{

    if (FSM_state >= FSMState::STATE_IDLE) //
    {
        if (FSM_state != last_fsm)
        {
            stage_timestamp = 0;
            last_fsm = FSM_state;
        }
        stage_timestamp += dt;
        // setF(dt, orientation.roll, orientation.pitch, orientation.yaw);
        setQ(dt, sd);
        priori(dt, orientation, FSM_state);
        update(barometer, acceleration, orientation, FSM_state, gps);
    }
}

/**
 * @brief Getter for state X
 *
 * @return the current state, see sensor_data.h for KalmanData
 */
KalmanData EKF::getState()
{
    return state;
}

/**
 * @brief Sets state vector x
 *
 * @param state Wanted state vector
 */
void EKF::setState(KalmanState state)
{
    this->state.position.px = state.state_est_pos_x;
    this->state.position.py = state.state_est_pos_y;
    this->state.position.pz = state.state_est_pos_z;
    this->state.acceleration.ax = state.state_est_accel_x;
    this->state.acceleration.ay = state.state_est_accel_y;
    this->state.acceleration.az = state.state_est_accel_z;
    this->state.velocity.vx = state.state_est_vel_x;
    this->state.velocity.vy = state.state_est_vel_y;
    this->state.velocity.vz = state.state_est_vel_z;
}

/**
 * @brief Sets the Q matrix given time step and spectral density.
 *
 * @param dt Time step calculated by the Kalman Filter Thread
 * @param sd Spectral density of the noise
 *
 * The Q matrix is the covariance matrix for the process noise and is
 * updated based on the time taken per cycle of the Kalman Filter Thread.
 */
void EKF::
    setQ(float dt, float sd)
{
    Q(0, 0) = pow(dt, 5) / 20;
    Q(0, 1) = pow(dt, 4) / 8;
    Q(0, 2) = pow(dt, 3) / 6;
    Q(1, 1) = pow(dt, 3) / 3;
    Q(1, 2) = pow(dt, 2) / 2;
    Q(2, 2) = dt;
    Q(1, 0) = Q(0, 1);
    Q(2, 0) = Q(0, 2);
    Q(2, 1) = Q(1, 2);
    Q(3, 3) = pow(dt, 5) / 20;
    Q(3, 4) = pow(dt, 4) / 8;
    Q(3, 5) = pow(dt, 3) / 6;
    Q(4, 4) = pow(dt, 3) / 3;
    Q(4, 5) = pow(dt, 2) / 2;
    Q(5, 5) = dt;
    Q(4, 3) = Q(3, 4);
    Q(5, 3) = Q(3, 5);
    Q(5, 4) = Q(4, 5);

    Q(6, 6) = pow(dt, 5) / 20;
    Q(6, 7) = pow(dt, 4) / 8;
    Q(6, 8) = pow(dt, 3) / 6;
    Q(7, 7) = pow(dt, 3) / 3;
    Q(7, 8) = pow(dt, 2) / 2;
    Q(8, 8) = dt;
    Q(7, 6) = Q(6, 7);
    Q(8, 6) = Q(6, 8);
    Q(8, 7) = Q(7, 8);

    Q *= sd;
}

/**
 * @brief Sets the F matrix given time step.
 *
 * @param dt Time step calculated by the Kalman Filter Thread
 *
 * The F matrix is the state transition matrix and is defined
 * by how the states change over time and also depends on the
 * current state of the rocket.
 */
void EKF::setF(float dt, float w_x, float w_y, float w_z, FSMState fsm, float v_x, float v_y, float v_z)

{
    F_mat.setIdentity();
    F_mat(0, 1) = dt;
    F_mat(0, 2) = 0.5f * dt * dt;
    F_mat(1, 2) = dt;
    F_mat(3, 4) = dt;
    F_mat(3, 5) = 0.5f * dt * dt;
    F_mat(4, 5) = dt;
    F_mat(6, 7) = dt;
    F_mat(6, 8) = 0.5f * dt * dt;
    F_mat(7, 8) = dt;
}

/**
 * @brief Update Kalman Gain at aech timestep.
 *
 * After receiving new sensor data, the Kalman filter updates the the Kalman Gain.
 * The Kalman gain can be considered as a measure of how uncertain the new sensor data is.
 */
void EKF::compute_kalman_gain()
{
    Eigen::Matrix<float, 6, 6> S_k = Eigen::Matrix<float, 6, 6>::Zero();
    S_k = (((H * P_priori * H.transpose()) + R)).inverse();
    K = (P_priori * H.transpose()) * S_k;
}
/**
 * @brief Update y and z at each timestep using GPS
 */
std::vector<float> EKF::compute_gps_inputs(GPS &gps, FSMState fsm)
{
    if (gps.latitude == 0 || gps.longitude == 0)
        return std::vector<float>{0.0f, 0.0f, 0.0f};


    // Set up starting GPS
    if (fsm == FSMState::STATE_IDLE)
    {
        starting_gps = {gps.latitude / 1e7, gps.longitude / 1e7, gps.altitude};
        starting_ecef = gps_to_ecef(starting_gps[0], starting_gps[1], starting_gps[2]);
    }


    // GPS degrees are given as integers
    float curr_lat = gps.latitude / 1e7;
    float curr_lon = gps.longitude / 1e7;
    float curr_alt = gps.altitude;

    if(abs(curr_lat - last_gps_latitude) < 1e-6 &&abs(curr_lon - last_gps_longitude) < 1e-6){
        return std::vector<float>{0.0f, 0.0f, 0.0f};
    }

    // Convert current GPS to ECEF and then ECEF to ENU
    std::vector<float> curr_ecef = gps_to_ecef(curr_lat, curr_lon, curr_alt);
    std::vector<float> enu = ecef_to_enu(curr_ecef, starting_ecef, starting_gps);

    // x_k(3, 0) = enu[0]; // y = east
    // x_k(6, 0) = enu[1]; // z = north

    last_gps_latitude = curr_lat;
    last_gps_longitude = curr_lon;
    return enu;
}

EKF ekf;
