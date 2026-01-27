#pragma once
//#include "Eigen.h"
#include "mqekf.h"
#include <Eigen/Dense>


// THIS ONE
void QuaternionMEKF::QuaternionMEKF(Vector3 const& sigma_a, Vector3 const& sigma_g, Vector3 const& sigma_m, float Pq0, float Pb0)
    :
      Q( initialize_Q(sigma_g) ),      
      R( (Vector6() << sigma_a, sigma_m).finished().array().square().matrix().asDiagonal() )
{
    qref.setIdentity();     //  q is now (w:1, x:0, y:0, z:0)
    x.setZero(); 

    P << Pq0*Matrix3::Identity(), Matrix3::Zero(),
            Matrix3::Zero(), Pb0*Matrix3::Identity();
}

void QuaternionMEKF::initialize_from_acc(Vector3 const& acc)
{
    float const anorm = acc.norm();
    v1ref << 0, 0, -anorm;

}

void QuaternionMEKF::initialize_from_acc(float const acc[3])
{
    initialize_from_acc(Map<Matrix<float, 3, 1>>(acc));
}


void QuaternionMEKF::time_update(Vector3 const& gyr, float Ts)
{
    set_transition_matrix(gyr - x.tail(3), Ts);

    // Quaternionf.coeffs() get the components in [x,y,z,w] order
    qref = F * qref.coeffs();
    qref.normalize();
    
    MatrixN F_a;
    // Slice 3x3 block from F
    F_a << F.block(0, 0, 3, 3), (-Matrix3::Identity()*Ts),
            Matrix3::Zero(), Matrix3::Identity();
    P = F_a * P * F_a.transpose() + Q;  // P update
}

void QuaternionMEKF::time_update(float const gyr[3], float Ts)
{
    time_update(Map<Matrix<float, 3, 1>>(gyr), Ts);
}

void QuaternionMEKF::measurement_update(Vector3 const& acc, Vector3 const& mag)
{
    // Predicted measurements
    Vector3 const v1hat = accelerometer_measurement_func();
    Vector3 const v2hat = magnetometer_measurement_func(); 

    Matrix3 const C1 = skew_symmetric_matrix(v1hat);
    Matrix3 const C2 = skew_symmetric_matrix(v2hat);

    Matrix<float, M, N> C;
    C << C1, Matrix<float, 3, 3>::Zero(),
            C2, Matrix<float, 3, 3>::Zero();

    Vector6 const yhat = (Vector6() << v1hat,
                                       v2hat).finished(); // predicted  

    Vector6 const y = (Vector6() << acc,
                                    mag).finished(); // 

    Vector6 const inno = y - yhat; // residual 
    MatrixM const s = C * P * C.transpose() + R;

    // K = P * C.float *(s)^-1
    // K * s = P*C.float

    // This is the form 
    // x * A = b
    // Which can be solved with the code below
    Eigen::FullPivLU<MatrixM> lu(s); //  LU decomposition of s
    if(lu.isInvertible())
    {
        Matrix<float, N, M> const K = P * C.transpose() * lu.inverse();  // gain 
 
        x += K * inno; // applying correction???

        // Joseph form of covariance measurement update
        MatrixN const temp = MatrixN::Identity() - K * C;
        P = temp * P * temp.transpose() + K * R * K.transpose(); // covariance update??? 
        // Apply correction to qref 
        Quaternion<float> corr(1, half*x(0), half*x(1), half*x(2)); // small angle approx????
        corr.normalize();
        qref = qref * corr; // multiply quaternions from ref??????? 

        // We only want to reset the quaternion part of the state
        x(0) = 0;
        x(1) = 0;
        x(2) = 0;

    }
}

void QuaternionMEKF::measurement_update(float const acc[3], float const mag[3])
{
    measurement_update(Map<Matrix<float, 3, 1>>(acc), Map<Matrix<float, 3, 1>>(mag));
}

Matrix<float, 4, 1> const& QuaternionMEKF::quaternion() const
{
    return qref.coeffs();
}

void QuaternionMEKF::set_transition_matrix(Eigen::Ref<const Vector3> const& gyr, float Ts)
{
    Vector3 const delta_theta = gyr*Ts;
    float un = delta_theta.norm();
    if(un == 0)
        un = std::numeric_limits<float>::min();

    Matrix4 const Omega = (Matrix4() << -skew_symmetric_matrix(delta_theta), delta_theta,
                                        -delta_theta.transpose(),            0          ).finished();

    F = std::cos(half*un)*Matrix4::Identity() + std::sin(half*un)/un * Omega;
}

Matrix<float, 3, 3> QuaternionMEKF::skew_symmetric_matrix(const Eigen::Ref<const Vector3>& vec) const
{
    Matrix3 M;
    M << 0, -vec(2), vec(1),
         vec(2), 0, -vec(0),
         -vec(1), vec(0), 0; 

    return M;
}

Matrix<float,3,1> QuaternionMEKF::accelerometer_measurement_func() const
{
    return qref.inverse() * v1ref;
}

Matrix<float, 3, 1> QuaternionMEKF::magnetometer_measurement_func() const
{
    return qref.inverse() * v2ref;
}

constexpr QuaternionMEKF::MatrixN QuaternionMEKF<float, with_bias>::initialize_Q(Vector3 sigma_g)
{
    return (Vector6() << sigma_g.array().square().matrix(), 1e-12, 1e-12, 1e-12).finished().asDiagonal();
}






// constexpr QuaternionMEKF::QuaternionMEKF(float const sigma_a[3], float const sigma_g[3], float const sigma_m[3], float Pq0, float Pb0) :
//     QuaternionMEKF(Map<Matrix<float, 3, 1>>(sigma_a), Map<Matrix<float, 3, 1>>(sigma_g), Map<Matrix<float, 3, 1>>(sigma_m), Pq0, Pb0)
// {    
// }

// void QuaternionMEKF::initialize_from_acc_mag(Vector3 const& acc, Vector3 const& mag)
// {
//     float const anorm = acc.norm();
//     v1ref << 0, 0, -anorm;

//     Vector3 const acc_normalized = acc/anorm;
//     Vector3 const mag_normalized = mag.normalized();

//     Vector3 const Rz = -acc_normalized;
//     Vector3 const Ry = Rz.cross(mag_normalized).normalized();
//     Vector3 const Rx = Ry.cross(Rz).normalized();

//     // Construct the rotation matrix
//     Matrix3 const R = (Matrix3() << Rx, Ry, Rz).finished();

//     // Eigen can convert it to a quaternion
//     qref = R.transpose();

//     // Reference magnetic field vector
//     v2ref = qref * mag;
// }

// void QuaternionMEKF::initialize_from_acc_mag(float const acc[3], float const mag[3])
// {
//     initialize_from_acc_mag(Map<Matrix<float, 3, 1>>(acc), Map<Matrix<float, 3, 1>>(mag));
// }

// Quaternion<float> QuaternionMEKF::quaternion_from_acc(Vector3 const& acc)
// {
//     // This finds inverse of qref
//     float qx, qy, qz, qw;
//     if(acc[2] >= 0)
//     {
//         qx = std::sqrt((1+acc[2])/2);
//         qw = acc[1]/(2*qx);
//         qy = 0;
//         qz = -acc[0]/(2*qx);
//     }
//     else
//     {
//         qw = std::sqrt((1-acc[2])/2);
//         qx = acc[1]/(2*qw);
//         qy = -acc[0]/(2*qw);
//         qz = 0; 
//     }
//     // Invert the quaternion
//     Quaternion<float> qref = Quaternion<float>(qw, -qx, -qy, -qz);

//      QuaternionMEKF
// }
