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

// void eulerToQuaternion(
//     float roll,
//     float pitch,
//     float yaw,
//     Eigen::Matrix<float, 4, 1> &quat)
// {

//     float cr = cos(roll * 0.5f);
//     float sr = sin(roll * 0.5f);
//     float cp = cos(pitch * 0.5f);
//     float sp = sin(pitch * 0.5f);
//     float cy = cos(yaw * 0.5f);
//     float sy = sin(yaw * 0.5f);

//     quat(0, 0) = cy * cp * cr + sy * sp * sr;
//     quat(1, 0) = cy * cp * sr - sy * sp * cr;
//     quat(2, 0) = sy * cp * sr + cy * sp * cr;
//     quat(3, 0) = sy * cp * cr - cy * sp * sr;
// }
