#include "ekf.h"
#include "fsm_states.h" // for sim
#include <iostream>
#include <iomanip>

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
        sum += barometer.altitude;
    }

    // set x_k - now 6 states: [x, vx, y, vy, z, vz]
    x_k.setZero();
    x_k(0, 0) = sum / 30;  // initial altitude (x position)
    x_k(2, 0) = 0;  // y position
    x_k(4, 0) = 0;  // z position

    F_mat.setZero(); // Initialize with zeros
    B_control.setZero(); // Initialize control input matrix

    setQ(s_dt, spectral_density_);

    // set H - measurement matrix for barometer and accelerometer
    // H(0,:) measures position x (barometer altitude)
    // H(1,:) measures acceleration x (but we'll use it as control input, not measurement)
    // H(2,:) measures acceleration y (but we'll use it as control input, not measurement)
    // H(3,:) measures acceleration z (but we'll use it as control input, not measurement)
    H.setZero();
    H(0, 0) = 1;  // barometer measures x position (altitude)
    H(1, 0) = 1;  // GPS measures x position (altitude)
    H(2, 2) = 1;  // GPS measures y position (east)
    H(3, 4) = 1;  // GPS measures z position (north)

    P_k.setZero();
    P_k.block<2, 2>(0, 0) = Eigen::Matrix2f::Identity() * 1e-2f; // x block (pos,vel)
    P_k.block<2, 2>(2, 2) = Eigen::Matrix2f::Identity() * 1e-2f; // y block (pos,vel)
    P_k.block<2, 2>(4, 4) = Eigen::Matrix2f::Identity() * 1e-2f; // z block (pos,vel)

    // set Measurement Noise Matrix (variance in m²; std dev = sqrt(R) in m)
    R(0, 0) = 0.01f;  // barometer: ~1.5 m typical error
    R(1, 1) = 9.0f;   // GPS altitude: ~3 m (vertical usually worse than horizontal)
    R(2, 2) = 4.0f;  // GPS east: ~2 m horizontal
    R(3, 3) = 4.0f;  // GPS north: ~2 m horizontal
}

/**
 * @brief Estimates current state of the rocket without current sensor data
 *
 * The priori step of the Kalman filter is used to estimate the current state
 * of the rocket without knowledge of the current sensor data. In other words,
 * it extrapolates the state at time n+1 based on the state at time n.
 */

void EKF::priori(float dt, Orientation &orientation, FSMState fsm, Acceleration acceleration)
{
    // CRITICAL FIX: Update F matrix BEFORE using it for both state and covariance
    setF(dt, 0, 0, 0, fsm, 0, 0, 0);
    setB(dt);
    
    // Compute control input (acceleration) once here - used for propagation and stored for update() output
    Eigen::Matrix<float, 3, 1> sensor_accel_global_g = Eigen::Matrix<float, 3, 1>::Zero();
    sensor_accel_global_g(0, 0) = acceleration.ax + 0.045f;
    sensor_accel_global_g(1, 0) = acceleration.ay - 0.065f;
    sensor_accel_global_g(2, 0) = acceleration.az - 0.06f;
    // euler_t angles_rad = orientation.getEuler();
    // BodyToGlobal(angles_rad, sensor_accel_global_g);
    float g_ms2 = (fsm > FSMState::STATE_IDLE) ? gravity_ms2 : 0.0f;
    u_control_last_(0, 0) = sensor_accel_global_g(0, 0) * g_ms2;
    u_control_last_(1, 0) = sensor_accel_global_g(1, 0) * g_ms2;
    u_control_last_(2, 0) = sensor_accel_global_g(2, 0) * g_ms2;
    
    // State propagation: x_priori = F * x_k + B * u
    x_priori = F_mat * x_k + B_control * u_control_last_;
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

    // Barometer measurement
    y_k(0, 0) = barometer.altitude; // meters (measures x position/altitude)

    // Kalman Gain for barometer measurement (single measurement)
    Eigen::Matrix<float, 1, NUM_STATES> H_baro = H.block<1, NUM_STATES>(0, 0);
    Eigen::Matrix<float, 1, 1> S_k_baro;
    S_k_baro(0, 0) = (H_baro * P_priori * H_baro.transpose())(0, 0) + R(0, 0);
    Eigen::Matrix<float, NUM_STATES, 1> K_baro = P_priori * H_baro.transpose() / S_k_baro(0, 0);

    // Posteriori Update with barometer
    Eigen::Matrix<float, NUM_STATES, NUM_STATES> identity = Eigen::Matrix<float, NUM_STATES, NUM_STATES>::Identity();
    float innovation = y_k(0, 0) - (H_baro * x_priori)(0, 0);
    x_k = x_priori + K_baro * innovation;
    P_k = (identity - K_baro * H_baro) * P_priori;

    // GPS measurement update (integrates lat/long/alt as measurements)
    compute_gps_inputs(gps, FSM_state);

    // Only reset velocities if we've been in LANDED state for at least 0.5 seconds
    // This prevents jitter from brief false LANDED state transitions
    bool is_landed = (FSM_state == FSMState::STATE_LANDED);
    if (is_landed)
    {
        if (was_landed_last)
        {
            landed_state_duration += s_dt;  // Accumulate time in LANDED state
        }
        else
        {
            landed_state_duration = s_dt;  // Reset timer
        }
        
        // Only reset velocities if we've been landed for at least 0.5 seconds
        if (landed_state_duration >= 1.0f)
        {
            x_k(3, 0) = 0.0f;  // velocity y (vy) = 0 when landed
            x_k(5, 0) = 0.0f;  // velocity z (vz) = 0 when landed
        }
    }
    else
    {
        landed_state_duration = 0.0f;  // Reset timer when not landed
    }
    was_landed_last = is_landed;

    // Update state structure
    kalman_state.state_est_pos_x = x_k(0, 0);
    kalman_state.state_est_vel_x = x_k(1, 0);
    kalman_state.state_est_accel_x = u_control_last_(0, 0);  // From priori()
    kalman_state.state_est_pos_y = x_k(2, 0);
    kalman_state.state_est_vel_y = x_k(3, 0);
    kalman_state.state_est_accel_y = u_control_last_(1, 0);  // From priori()
    kalman_state.state_est_pos_z = x_k(4, 0);
    kalman_state.state_est_vel_z = x_k(5, 0);
    kalman_state.state_est_accel_z = u_control_last_(2, 0);  // From priori()

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
            // Reset landed state tracking when FSM changes
            if (FSM_state != FSMState::STATE_LANDED)
            {
                landed_state_duration = 0.0f;
                was_landed_last = false;
            }
        }
        stage_timestamp += dt;
        s_dt = dt;  // Store dt for use in update
        
        setQ(dt, sd);
        priori(dt, orientation, FSM_state, acceleration);
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
void EKF::setQ(float dt, float sd)
{
    // Q matrix for 6 states: [x, vx, y, vy, z, vz]
    // Process noise for position-velocity model (Singer model)
    float sigma_a = 0.2f; // m/s^2, tune based on IMU + drag + unmodeled
    Q.setZero();

    // X axis
    Q(0,0) = pow(dt,4)/4.0f * sigma_a*sigma_a;
    Q(0,1) = pow(dt,3)/2.0f * sigma_a*sigma_a;
    Q(1,0) = Q(0,1);
    Q(1,1) = pow(dt,2) * sigma_a*sigma_a;

    // Y axis
    Q(2,2) = pow(dt,4)/4.0f * sigma_a*sigma_a;
    Q(2,3) = pow(dt,3)/2.0f * sigma_a*sigma_a;
    Q(3,2) = Q(2,3);
    Q(3,3) = pow(dt,2) * sigma_a*sigma_a;

    // Z axis
    Q(4,4) = pow(dt,4)/4.0f * sigma_a*sigma_a;
    Q(4,5) = pow(dt,3)/2.0f * sigma_a*sigma_a;
    Q(5,4) = Q(4,5);
    Q(5,5) = pow(dt,2) * sigma_a*sigma_a;
}

// void EKF::setQ(float dt, float sd)
// {
//     // Q matrix for 6 states: [x, vx, y, vy, z, vz]
//     // Process noise for position-velocity model (Singer model)
//     Q.setZero();
    
//     // X axis block (position, velocity)
//     Q(0, 0) = pow(dt, 5) / 20;  // position-position
//     Q(0, 1) = pow(dt, 4) / 8;   // position-velocity
//     Q(1, 1) = pow(dt, 3) / 3;   // velocity-velocity
//     Q(1, 0) = Q(0, 1);
    
//     // Y axis block (position, velocity)
//     Q(2, 2) = pow(dt, 5) / 20;
//     Q(2, 3) = pow(dt, 4) / 8;
//     Q(3, 3) = pow(dt, 3) / 3;
//     Q(3, 2) = Q(2, 3);
    
//     // Z axis block (position, velocity)
//     Q(4, 4) = pow(dt, 5) / 20;
//     Q(4, 5) = pow(dt, 4) / 8;
//     Q(5, 5) = pow(dt, 3) / 3;
//     Q(5, 4) = Q(4, 5);

//     Q *= sd;
// }


/**
 * @brief Sets the F matrix given time step.
 *
 * @param dt Time step calculated by the Kalman Filter Thread
 *
 * The F matrix is the state transition matrix and is defined
 * by how the states change over time and also depends on the
 * current state of the rocket.
 */
void EKF::setF(float dt, float /*w_x*/, float /*w_y*/, float /*w_z*/, FSMState /*fsm*/, float /*v_x*/, float /*v_y*/, float /*v_z*/)
{
    // F matrix for [x, vx, y, vy, z, vz] state
    // x_{k+1} = x_k + vx_k * dt
    // vx_{k+1} = vx_k  (acceleration applied via control input B)
    F_mat.setIdentity();
    
    // X axis: position = position + velocity * dt
    F_mat(0, 1) = dt;  // x += vx * dt
    
    // Y axis: position = position + velocity * dt
    F_mat(2, 3) = dt;  // y += vy * dt
    
    // Z axis: position = position + velocity * dt
    F_mat(4, 5) = dt;  // z += vz * dt
    
    // Velocity states remain unchanged (acceleration handled by control input)
}

void EKF::setB(float dt)
{
    // B matrix maps control input [ax, ay, az] to state derivatives
    // dvx/dt = ax, dvy/dt = ay, dvz/dt = az
    B_control.setZero();
    
    // X axis: velocity change = acceleration * dt
    B_control(0, 0) = 0.5f * dt * dt;
    B_control(1, 0) = dt;  // vx += ax * dt
    
    // Y axis: velocity change = acceleration * dt
    B_control(2, 1) = 0.5f * dt * dt;
    B_control(3, 1) = dt;  // vy += ay * dt
    
    // Z axis: velocity change = acceleration * dt
    B_control(4, 2) = 0.5f * dt * dt;
    B_control(5, 2) = dt;  // vz += az * dt
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
    // This function is kept for compatibility but not used
    // Kalman gain is now computed inline in update() for barometer
    // and in compute_gps_inputs() for GPS measurements
}
/**
 * @todo The general idea is that we store the initial gps coords,
 * and then we update the y,z positions as that data arrives.
 */
void EKF::compute_gps_inputs(GPS &gps, FSMState fsm)
{
    /**
     * GPS measurement update - properly integrates GPS lat/long as measurements
     * with high trust (low measurement noise)
     */
    reference_GPS(gps, fsm);

    // Check if GPS has valid fix
    if (gps.fix_type == 0 || gps.latitude == 0 || gps.longitude == 0)
    {
        return;  // No GPS fix, skip update
    }

    float lat = gps.latitude / 1e7; // dividing by 1e7 to convert from int to float
    float lon = gps.longitude / 1e7;
    float alt = gps.altitude;
    
    // // Skip if GPS hasn't changed significantly (avoid redundant updates)
    // if (abs(lat - gps_latitude_last) <= 1e-5 && abs(lon - gps_longitude_last) <= 1e-5)
    // {
    //     return;
    // }

    // Convert GPS to ECEF
    std::vector<float> rocket_cords = gps_to_ecef(lat, lon, alt);
    std::vector<float> reference_cord = gps_to_ecef(gps_latitude_original, gps_longitude_original, 0);

    float gps_latitude_original_rad = gps_latitude_original * pi / 180;
    float gps_longitude_original_rad = gps_longitude_original * pi / 180;

    double dx = rocket_cords[0] - reference_cord[0];
    double dy = rocket_cords[1] - reference_cord[1];
    double dz = rocket_cords[2] - reference_cord[2];

    float east = -std::sin(gps_longitude_original_rad) * dx + std::cos(gps_longitude_original_rad) * dy;
    float north = -std::sin(gps_latitude_original_rad) * std::cos(gps_longitude_original_rad) * dx - std::sin(gps_latitude_original_rad) * std::sin(gps_longitude_original_rad) * dy + std::cos(gps_latitude_original_rad) * dz;
    // float up = std::cos(gps_latitude_original_rad) * std::cos(gps_longitude_original_rad) * dx + std::cos(gps_latitude_original_rad) * std::sin(gps_longitude_original_rad) * dy + std::sin(gps_latitude_original_rad) * dz;  // Not used

    // GPS altitude as x position measurement (relative to reference altitude)
    // The reference was initialized at altitude 0, so GPS altitude directly gives x position
    float gps_x_position = alt;

    // PROPER GPS MEASUREMENT UPDATE (not direct state overwrite)
    // GPS measures x position (altitude), y position (east), and z position (north)
    // State: [x, vx, y, vy, z, vz]
    // GPS measures: x (altitude), y (east), and z (north) positions
    
    // Set GPS measurement matrix using main H matrix (rows 1-3)
    // H(1,0) = GPS measures x position (altitude)
    // H(2,2) = GPS measures y position (east)
    // H(3,4) = GPS measures z position (north)
    
    
    // Extract GPS measurement matrix from main H matrix
    Eigen::Matrix<float, 3, NUM_STATES> H_gps = H.block<3, NUM_STATES>(1, 0);
    
    // GPS measurements
    Eigen::Matrix<float, 3, 1> y_gps;
    y_gps(0) = gps_x_position;  // x position (altitude)
    y_gps(1) = east;            // y position (east)
    y_gps(2) = north;           // z position (north)
    
    // Use combined R matrix (rows/cols 1-3 for GPS measurements)
    Eigen::Matrix<float, 3, 3> R_gps = R.block<3, 3>(1, 1);
    
    // Innovation
    Eigen::Matrix<float, 3, 1> innovation_gps = y_gps - H_gps * x_k;
    
    // Innovation covariance
    Eigen::Matrix<float, 3, 3> S_gps = H_gps * P_k * H_gps.transpose() + R_gps;
    
    // Kalman gain for GPS
    Eigen::Matrix<float, NUM_STATES, 3> K_gps = P_k * H_gps.transpose() * S_gps.inverse();
    
    // Update state and covariance
    x_k = x_k + K_gps * innovation_gps;
    Eigen::Matrix<float, NUM_STATES, NUM_STATES> identity = Eigen::Matrix<float, NUM_STATES, NUM_STATES>::Identity();
    P_k = (identity - K_gps * H_gps) * P_k;
    
    // Only reset velocities if we've been in LANDED state for at least 0.5 seconds
    // This prevents jitter from brief false LANDED state transitions
    bool is_landed = (fsm == FSMState::STATE_LANDED);
    if (is_landed)
    {
        if (was_landed_last)
        {
            landed_state_duration += s_dt;  // Accumulate time in LANDED state
        }
        else
        {
            landed_state_duration = s_dt;  // Reset timer
        }
        
        // Only reset velocities if we've been landed for at least 0.5 seconds
        if (landed_state_duration >= 0.5f)
        {
            x_k(3, 0) = 0.0f;  // velocity y (vy) = 0 when landed
            x_k(5, 0) = 0.0f;  // velocity z (vz) = 0 when landed
        }
    }
    else
    {
        // Negate velocity z if it's negative (so negative becomes positive)
        if (x_k(5, 0) < 0)  // velocity z (vz)
        {
            x_k(5, 0) = -x_k(5, 0);  // Negate negative velocity z to make it positive
        }
    }
    
    // Update last GPS coordinates
    gps_latitude_last = lat;
    gps_longitude_last = lon;
    
    // Update state structure
    kalman_state.state_est_pos_x = x_k(0, 0);
    kalman_state.state_est_vel_x = x_k(1, 0);
    kalman_state.state_est_pos_y = x_k(2, 0);
    kalman_state.state_est_vel_y = x_k(3, 0);
    kalman_state.state_est_pos_z = x_k(4, 0);
    kalman_state.state_est_vel_z = x_k(5, 0);
    
    state.position = (Position){kalman_state.state_est_pos_x, kalman_state.state_est_pos_y, kalman_state.state_est_pos_z};
    state.velocity = (Velocity){kalman_state.state_est_vel_x, kalman_state.state_est_vel_y, kalman_state.state_est_vel_z};
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

void EKF::printCovariance() const
{
    const char* labels[] = {"x", "vx", "y", "vy", "z", "vz"};
    std::cout << "\n--- Final covariance matrix P_k (state: x, vx, y, vy, z, vz) ---" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "       ";
    for (int c = 0; c < NUM_STATES; c++)
        std::cout << std::setw(12) << labels[c];
    std::cout << std::endl;
    for (int r = 0; r < NUM_STATES; r++)
    {
        std::cout << std::setw(6) << labels[r] << " ";
        for (int c = 0; c < NUM_STATES; c++)
            std::cout << std::setw(12) << P_k(r, c);
        std::cout << std::endl;
    }
    std::cout << "---" << std::endl;
}

EKF ekf;
