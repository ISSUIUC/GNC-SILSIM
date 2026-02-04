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
    H(0, 0) = 1;
    H(1, 2) = 1;
    H(2, 5) = 1;
    H(3, 8) = 1;

    P_k.setZero();
    P_k.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * 1e-2f; // x block (pos,vel,acc)
    P_k.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity() * 1e-2f; // y block
    P_k.block<3, 3>(6, 6) = Eigen::Matrix3f::Identity() * 1e-2f; // z block

    // set Measurement Noise Matrix
    R(0, 0) = 1.9;
    R(1, 1) = 1.9;
    R(2, 2) = 1.9;
    R(3, 3) = 1.9;
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
    y_k(1, 0) = ((sensor_accel_global_g)(0)) * g_ms2;
    y_k(2, 0) = ((sensor_accel_global_g)(1)) * g_ms2;
    y_k(3, 0) = ((sensor_accel_global_g)(2)) * g_ms2;

    y_k(0, 0) = barometer.altitude; // meters

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
        // Eigen::Matrix<float,4,1> quat;
        // eulerToQuaternion(orientation.roll,orientation.pitch,orientation.yaw,quat);

        // std::cout << "Quaternion: w=" << quat(0,0)<< ", x=" << quat(1,0)
        //           << ", y=" << quat(2,0)<< ", z=" <<quat(3,0)<< std::endl;

        compute_gps_inputs(gps, FSM_state); // testing GPS inputs
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
 * @brief Computes the rockets mass throughout the flight.
 *
 * The following takes the fsm state of the rocket and uses the preconfigured rocket masses to calculate the mass over the rockets trajectory.
 * The code is configured so booster only goes through this interpolation once, whereas sustainer goes through it multiple time.
 * @param fsm: Takes the current FSM state of the rocket.
 * @todo Include the sustainer code, and include checks on the code to ensure mass does not become 0.
 */
void EKF::compute_mass(FSMState fsm)
{
    if (fsm == FSMState::STATE_FIRST_BOOST)
    {
        curr_mass_kg = mass_full - (mass_full - mass_first_burnout) * stage_timestamp / 2.75;
    }
    /**
     * @todo add the sustainer code.
     */
}
/**
 * @brief Update Kalman Gain at aech timestep.
 *
 * After receiving new sensor data, the Kalman filter updates the the Kalman Gain.
 * The Kalman gain can be considered as a measure of how uncertain the new sensor data is.
 */
void EKF::compute_kalman_gain()
{
    Eigen::Matrix<float, 4, 4> S_k = Eigen::Matrix<float, 4, 4>::Zero();
    S_k = (((H * P_priori * H.transpose()) + R)).inverse();
    K = (P_priori * H.transpose()) * S_k;
}
/**
 * @todo The general idea is that we store the initial gps coords,
 * and then we update the y,z positions as that data arrives.
 */
void EKF::compute_gps_inputs(GPS &gps, FSMState fsm)
{
    /**
     * struct GPS {
      float latitude;
      float longitude;
      float altitude;
      float speed;
      int fix_type;
      float time;
  };
     *


     lat, long, alt = GPS
      lat = np.radians(lat)
      long = np.radians(long)

      e2 = (a**2 - b**2) / a**2                   # Eccentricity squared
      N = a / np.sqrt(1 - e2 * np.sin(lat)**2)    # Prime vertical radius of curvature

      x = (N + alt) * np.cos(lat) * np.cos(long)
      y = (N + alt) * np.cos(lat) * np.sin(long)
      z = ((1 - e2) * N + alt) * np.sin(lat)

      return np.array([x, y, z])
     *  */
    reference_GPS(gps, fsm);

    double lat = gps.latitude / 1e7; // deviding by 1e7 to convert from int to float
    double lon = gps.longitude / 1e7;
    double alt = gps.altitude;
    if (abs(lat - gps_latitude_last) <= 1e-5 && abs(lon - gps_longitude_last) <= 1e-5)
    {
        return;
    }

    // Convert GPS to ECEF
    std::vector<double> rocket_cords = gps_to_ecef(lat, lon, alt);
    std::vector<double> reference_cord = gps_to_ecef(gps_latitude_original, gps_longitude_original, 0);

    double gps_latitude_original_rad = gps_latitude_original * pi / 180;
    double gps_longitude_original_rad = gps_longitude_original * pi / 180;

    double dx = rocket_cords[0] - reference_cord[0];
    double dy = rocket_cords[1] - reference_cord[1];
    double dz = rocket_cords[2] - reference_cord[2];

    double east = -std::sin(gps_longitude_original_rad) * dx + std::cos(gps_longitude_original_rad) * dy;
    double north = -std::sin(gps_latitude_original_rad) * std::cos(gps_longitude_original_rad) * dx - std::sin(gps_latitude_original_rad) * std::sin(gps_longitude_original_rad) * dy + std::cos(gps_latitude_original_rad) * dz;
    double up = std::cos(gps_latitude_original_rad) * std::cos(gps_longitude_original_rad) * dx + std::cos(gps_latitude_original_rad) * std::sin(gps_longitude_original_rad) * dy + std::sin(gps_latitude_original_rad) * dz;

    // Update Kalman filter state CHECK THIS ORIENTATION ... NOT SURE IF THIS IS RIGHT
    x_k(3, 0) = east;
    x_k(6, 0) = north;
}

void EKF::reference_GPS(GPS &gps, FSMState fsm)
{
    if (gps.latitude == 0 || gps.longitude == 0)
    {
        return; // No GPS fix, skip reference update
    }

    if (fsm == FSMState::STATE_IDLE)
    {
        gps_latitude_original = gps.latitude / 1e7;
        gps_longitude_original = gps.longitude / 1e7;
        gps_latitude_last = gps_latitude_original;
        gps_longitude_last = gps_longitude_original;
    }
}

EKF ekf;
