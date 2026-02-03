#include "mqekf.h"

QuaternionMEKF::QuaternionMEKF(
    const Eigen::Matrix<float, 3, 1> &sigma_a,
    const Eigen::Matrix<float, 3, 1> &sigma_g,
    const Eigen::Matrix<float, 3, 1> &sigma_m)
{
    float Pq0 = 1e-6;
    float Pb0 = 1e-1;
    Q = initialize_Q(sigma_g);

    Eigen::Matrix<float, 6, 1> sigmas;
    sigmas << sigma_a, sigma_m;
    R = sigmas.array().square().matrix().asDiagonal();

    qref.setIdentity(); // 1,0,0,0
    x.setZero();
    P.setZero();
    P.block<3, 3>(0, 0) = Pq0 * Eigen::Matrix3f::Identity();
    P.block<3, 3>(3, 3) = Pb0 * Eigen::Matrix3f::Identity();
}

void QuaternionMEKF::time_update(Eigen::Matrix<float, 3, 1> const &gyr, float Ts)
{
    set_transition_matrix(gyr - x.tail(3), Ts);

    Eigen::Vector4f q; //necessary to reorder to w,x,y,z
    q << qref.w(), qref.x(), qref.y(), qref.z();

    q = F * q;

    qref = Eigen::Quaternionf(q(0), q(1), q(2), q(3));
    qref.normalize();

    Eigen::Matrix<float, 6, 6> F_a;
    // Slice 3x3 block from F
    F_a << F.block(0, 0, 3, 3), (-Eigen::Matrix<float, 3, 3>::Identity() * Ts),
        Eigen::Matrix<float, 3, 3>::Zero(), Eigen::Matrix<float, 3, 3>::Identity();
    P = F_a * P * F_a.transpose() + Q; // P update
}



void QuaternionMEKF::measurement_update(Eigen::Matrix<float, 3, 1> const &acc, Eigen::Matrix<float, 3, 1> const &mag)
{
    // Predicted measurements
    Eigen::Matrix<float, 3, 1> const v1hat = accelerometer_measurement_func();
    Eigen::Matrix<float, 3, 1> const v2hat = magnetometer_measurement_func();

    Eigen::Matrix<float, 3, 3> const C1 = skew_symmetric_matrix(v1hat);
    Eigen::Matrix<float, 3, 3> const C2 = skew_symmetric_matrix(v2hat);

    Eigen::Matrix<float, 6, 6> C;
    C << C1, Eigen::Matrix<float, 3, 3>::Zero(),
        C2, Eigen::Matrix<float, 3, 3>::Zero();

    Eigen::Matrix<float, 6, 1> yhat;
    yhat << v1hat,
        v2hat;

    Eigen::Matrix<float, 6, 1> y;
    y << acc,
        mag;

    Eigen::Matrix<float, 6, 1> inno = y - yhat;

    Eigen::Matrix<float, 6, 6> const s = C * P * C.transpose() + R;

    // K = P * C.float *(s)^-1
    // K * s = P*C.float

    // This is the form
    // x * A = b
    // Which can be solved with the code below
    Eigen::FullPivLU<Eigen::Matrix<float, 6, 6>> lu(s); //  LU decomposition of s
    if (lu.isInvertible())
    {
        Eigen::Matrix<float, 6, 6> const K = P * C.transpose() * lu.inverse(); // gain

        x += K * inno; // applying correction???

        // Joseph form of covariance measurement update
        Eigen::Matrix<float, 6, 6> const temp = Eigen::Matrix<float, 6, 6>::Identity() - K * C;
        P = temp * P * temp.transpose() + K * R * K.transpose(); // covariance update???
        // Apply correction to qref
        Eigen::Quaternion<float> corr(1, 0.5f  * x(0), 0.5f  * x(1), 0.5f  * x(2)); // small angle approx????
        corr.normalize();
        qref = qref * corr; // multiply quaternions from ref???????

        // We only want to reset the quaternion part of the state
        x(0) = 0;
        x(1) = 0;
        x(2) = 0;
    }
}


void QuaternionMEKF::measurement_update_partial(
    Eigen::Matrix<float, 3, 1> const& meas,
    Eigen::Ref<Eigen::Matrix<float, 3, 1> const> const& vhat,
    Eigen::Ref<Eigen::Matrix<float, 3, 3> const> const& Rm)
{
    // Predicted measurement Jacobian
    Eigen::Matrix<float, 3, 3> const C1 = skew_symmetric_matrix(vhat);

    Eigen::Matrix<float, 3, 6> C;
    
    C << C1, Eigen::Matrix<float, 3, 3>::Zero();
    
   

    // Innovation
    Eigen::Matrix<float, 3, 1> const inno = meas - vhat;

    // Innovation covariance
    Eigen::Matrix<float, 3, 3> const s = C * P * C.transpose() + Rm;

    // K = P * C.T * s^-1
    Eigen::FullPivLU<Eigen::Matrix<float, 3, 3>> lu(s);
    if (lu.isInvertible())
    {
        Eigen::Matrix<float, 6, 3> const K = P * C.transpose() * lu.inverse();

        // State update
        x += K * inno;

        // Joseph form covariance update
        Eigen::Matrix<float, 6, 6> const temp =
            Eigen::Matrix<float, 6, 6>::Identity() - K * C;
        P = temp * P * temp.transpose() + K * Rm * K.transpose();

        // Apply correction to reference quaternion (small-angle approx)
        Eigen::Quaternion<float> corr(
            1.0f,
            0.5f * x(0),
            0.5f * x(1),
            0.5f * x(2));
        corr.normalize();
        qref = qref * corr;

        // Reset attitude error states
        x(0) = 0.0f;
        x(1) = 0.0f;
        x(2) = 0.0f;
    }
}



Eigen::Matrix<float, 4, 1> QuaternionMEKF::quaternion() 
{
    return qref.coeffs();
}

void QuaternionMEKF::set_transition_matrix(Eigen::Ref<const Eigen::Matrix<float, 3, 1>> const &gyr, float Ts)
{
    Eigen::Matrix<float, 3, 1> const delta_theta = gyr * Ts;
    float un = delta_theta.norm();
    if (un == 0)
        un = std::numeric_limits<float>::min();

    Eigen::Matrix<float, 4, 4> const Omega = (Eigen::Matrix<float, 4, 4>() << -skew_symmetric_matrix(delta_theta), delta_theta,
                           -delta_theta.transpose(), 0)
                              .finished();

    F = std::cos(0.5f * un) * Eigen::Matrix<float, 4, 4>::Identity() + std::sin(0.5f  * un) / un * Omega;
}

Eigen::Matrix<float, 3, 3> QuaternionMEKF::skew_symmetric_matrix(const Eigen::Ref<const Eigen::Matrix<float, 3, 1>> &vec) const
{
    Eigen::Matrix<float, 3, 3> M;
    M << 0, -vec(2), vec(1),
        vec(2), 0, -vec(0),
        -vec(1), vec(0), 0;

    return M;
}

Eigen::Matrix<float, 3, 1> QuaternionMEKF::accelerometer_measurement_func() const
{
    return qref.inverse() * v1ref;
}

Eigen::Matrix<float, 3, 1> QuaternionMEKF::magnetometer_measurement_func() const
{
    return qref.inverse() * v2ref;
}

Eigen::Matrix<float, 6, 6> QuaternionMEKF::initialize_Q(Eigen::Matrix<float, 3, 1> sigma_g)
{
    Eigen::Matrix<float, 6, 6> Q = Eigen::Matrix<float, 6, 6>::Zero();
    Q.block<3,3>(0,0) = sigma_g.array().square().matrix().asDiagonal();
    Q.block<3,3>(3,3) = 1e-12 * Eigen::Matrix3f::Identity();
    return Q;


}


void QuaternionMEKF::initialize_from_acc_mag(Eigen::Matrix<float, 3, 1> const &acc, Eigen::Matrix<float, 3, 1> const &mag)
{
    float const anorm = acc.norm();
    v1ref << -anorm, 0, 0;

    
    Eigen::Matrix<float, 3, 1> const acc_normalized = acc / anorm;
    Eigen::Matrix<float, 3, 1> const mag_normalized = mag.normalized();

    Eigen::Matrix<float, 3, 1> const Rz = -acc_normalized;
    Eigen::Matrix<float, 3, 1> const Ry = Rz.cross(mag_normalized).normalized();
    Eigen::Matrix<float, 3, 1> const Rx = Ry.cross(Rz).normalized();

    // Construct the rotation matrix
    Eigen::Matrix<float, 3, 3> const R = (Eigen::Matrix<float, 3, 3>() << Rx, Ry, Rz).finished();

    // Eigen can convert it to a quaternion
    qref = R.transpose();

    // Reference magnetic field vector
    v2ref = qref * mag;
}

Eigen::Matrix<float, 3, 1> QuaternionMEKF::gyroscope_bias() 
{
    return x.tail(3);
}
