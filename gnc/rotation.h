#include <Eigen/Eigen>
/**************************** ROTATION FUNCTIONS ****************************/

// Used in ekf.cpp for ECEF and ENU conversions
#define A 6378137.0                      // Equatorial radius
#define F (1.0 / 298.257223563)          // Flattening factor
#define B (A * (1 - F))                  // Polar radius
#define E_SQ ((A * A - B * B) / (A * A)) // Eccentricity squared
#define pi 3.1415
/**
 * @brief Converts a vector in the body frame to the global frame
 *
 * @param angles Roll, pitch, yaw angles
 * @param body_vec Vector for rotation in the body frame
 * @return Eigen::Matrix<float, 3, 1> Rotated vector in the global frame
 */
template <typename Angles>
void BodyToGlobal(Angles &angles_rad, Eigen::Matrix<float, 3, 1> &body_vec)
{
    Eigen::Matrix3f roll, pitch, yaw;
    roll << cos(angles_rad.roll), -sin(angles_rad.roll), 0.,
        sin(angles_rad.roll), cos(angles_rad.roll), 0.,
        0., 0., 1.;

    pitch << cos(angles_rad.pitch), 0., sin(angles_rad.pitch),
        0., 1., 0.,
        -sin(angles_rad.pitch), 0., cos(angles_rad.pitch);

    yaw << 1., 0., 0.,
        0., cos(angles_rad.yaw), -sin(angles_rad.yaw),
        0., sin(angles_rad.yaw), cos(angles_rad.yaw);

    Eigen::Matrix3f rotation_matrix = yaw * pitch * roll;
    Eigen::Vector3f temp = rotation_matrix * body_vec;

    // Convert from Z-up convention to X-up convention
    // Eigen::Vector3f corrected;
    // corrected(0) = temp(2);  // Z → X
    // corrected(1) = temp(1);  // X → Y
    // corrected(2) = temp(0);  // Y → Z

    // body_vec = corrected;
}

/**
 * @brief Converts a vector in the global frame to the body frame
 *
 * @param angles Roll, pitch, yaw angles
 * @param global_vec Vector for rotation in the global frame
 *
 * @return Eigen::Matrix<float, 3, 1> Rotated vector in the body frame
 */
template <typename Angles>
void GlobalToBody(Angles &angles_rad, Eigen::Matrix<float, 3, 1> &global_vec)
{
    Eigen::Matrix3f roll, pitch, yaw;

    roll << cos(angles_rad.roll), -sin(angles_rad.roll), 0.,
        sin(angles_rad.roll), cos(angles_rad.roll), 0.,
        0., 0., 1.;

    // Pitch about Y (tilt forward/back)
    pitch << cos(angles_rad.pitch), 0., sin(angles_rad.pitch),
        0., 1., 0.,
        -sin(angles_rad.pitch), 0., cos(angles_rad.pitch);

    // Yaw about X (turn around up axis)
    yaw << 1., 0., 0.,
        0., cos(angles_rad.yaw), -sin(angles_rad.yaw),
        0., sin(angles_rad.yaw), cos(angles_rad.yaw);

    Eigen::Matrix3f rotation_matrix = yaw * pitch * roll;
    Eigen::Vector3f temp = rotation_matrix.transpose() * global_vec;
    Eigen::Matrix3f R_zup_to_xup;
    R_zup_to_xup << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;

    global_vec = (R_zup_to_xup * rotation_matrix).transpose() * global_vec;
}

inline std::vector<float> ECEF(float lat, float lon, float alt)
{
    lat *= pi / 180.0;
    lon *= pi / 180.0;
    double N = A / std::sqrt(1 - E_SQ * std::sin(lat) * std::sin(lat)); // Radius of curvature in the prime vertical

    float x = (N + alt) * std::cos(lat) * std::cos(lon);
    float y = (N + alt) * std::cos(lat) * std::sin(lon);
    float z = ((1 - E_SQ) * N + alt) * std::sin(lat); // Semi-minor axis of Earth in meters

    return {x, y, z};
}

// helper: multiply two quaternions (w,x,y,z) returning (rw,rx,ry,rz)
static inline void quat_mult_f(float w1, float x1, float y1, float z1,
                               float w2, float x2, float y2, float z2,
                               float &rw, float &rx, float &ry, float &rz)
{
    rw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    rx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    ry = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    rz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
}

/**
 * @brief Rotate a vector from the body frame to the global frame using a quaternion
 *
 * @param q0 Quaternion real (w) component
 * @param q1 Quaternion i (x) component
 * @param q2 Quaternion j (y) component
 * @param q3 Quaternion k (z) component
 * @param body_vec Vector in the body frame (will be replaced with the global frame vector)
 *
 */
inline void BodyToGlobalQuat(float q0, float q1, float q2, float q3, Eigen::Matrix<float, 3, 1> &body_vec)
{
    // Rotate using quaternion multiplication: p_global = q_conj * p * q
    // where p is quaternion (0, px, py, pz) and q_conj = (q0, -q1, -q2, -q3)
    float px = body_vec(0);
    float py = body_vec(1);
    float pz = body_vec(2);

    // p quaternion
    float pw = 0.0f;

    // temp = q_conj * p
    float tw, tx, ty, tz;
    quat_mult_f(q0, -q1, -q2, -q3, pw, px, py, pz, tw, tx, ty, tz);

    // result = temp * q
    float rw, rx, ry, rz;
    quat_mult_f(tw, tx, ty, tz, q0, q1, q2, q3, rw, rx, ry, rz);

    body_vec(0) = rx;
    body_vec(1) = ry;
    body_vec(2) = rz;
}

/**
 * @brief Rotate a vector from the global frame to the body frame using a quaternion
 *
 * @param q0 Quaternion real (w) component
 * @param q1 Quaternion i (x) component
 * @param q2 Quaternion j (y) component
 * @param q3 Quaternion k (z) component
 * @param global_vec Vector in the global frame (will be replaced with the body frame vector)
 *
 * This uses the transpose (inverse) of the rotation matrix computed from the quaternion.
 */
inline void GlobalToBodyQuat(float q0, float q1, float q2, float q3, Eigen::Matrix<float, 3, 1> &global_vec)
{
    // Rotate using quaternion multiplication: p_body = q * g * q_conj
    // where g is quaternion (0, gx, gy, gz) and q_conj = (q0, -q1, -q2, -q3)
    float gx = global_vec(0);
    float gy = global_vec(1);
    float gz = global_vec(2);

    float gw = 0.0f;

    // temp = q * g
    float tw, tx, ty, tz;
    quat_mult_f(q0, q1, q2, q3, gw, gx, gy, gz, tw, tx, ty, tz);

    // result = temp * q_conj
    float rw, rx, ry, rz;
    quat_mult_f(tw, tx, ty, tz, q0, -q1, -q2, -q3, rw, rx, ry, rz);

    global_vec(0) = rx;
    global_vec(1) = ry;
    global_vec(2) = rz;
}