#pragma once

#include "sensor_data.h" // for sim
#include "Buffer.h" // for sim

#include <Eigen/Eigen>


class QuaternionMEKF
{
    public:
        constexpr QuaternionMEKF(Eigen::Matrix<float, 3, 1> const& sigma_a, Eigen::Matrix<float, 3, 1> const& sigma_g, Eigen::Matrix<float, 3, 1> const& sigma_m, float Pq0 = 1e-6, float Pb0 = 1e-1);
        void initialize_from_acc_mag(Eigen::Matrix<float, 3, 1> const& acc, Eigen::Matrix<float, 3, 1> const& mag);
        void time_update(Eigen::Matrix<float, 3, 1> const& gyr, float Ts);
        void measurement_update(Eigen::Matrix<float, 3, 1> const& acc, Eigen::Matrix<float, 3, 1> const& mag);
        Eigen::Matrix<float, 4, 1> const& quaternion() const;
        Eigen::Matrix<float, 6, 6> const& covariance() const;
        Eigen::Matrix<float, 3, 1> gyroscope_bias() const;

        
    private:
        Eigen::Quaternion<float> qref;

        Eigen::Matrix<float, 3, 1> v1ref;
        Eigen::Matrix<float, 3, 1> v2ref;

        // State
        Eigen::Matrix<float, 4, 1> x;
        // State covariance
        Eigen::Matrix<float, 6, 6> P;

        // Quaternion update matrix
        Eigen::Matrix<float, 4, 4> F;

        // Constant matrices
        const Eigen::Matrix<float, 3, 3> Racc, Rmag;
        const Eigen::Matrix<float, 6, 6> R;
        const Eigen::Matrix<float, 6, 6> Q;

        void set_transition_matrix(const Eigen::Ref<const Eigen::Matrix<float, 3, 1>>& gyr, float Ts);
        Eigen::Matrix<float, 3, 3> skew_symmetric_matrix(const Eigen::Ref<const Eigen::Matrix<float, 3, 1>>& vec) const;
        Eigen::Matrix<float, 3, 1> accelerometer_measurement_func() const;
        Eigen::Matrix<float, 3, 1> magnetometer_measurement_func() const;

        static constexpr Eigen::Matrix<float, 6, 6> initialize_Q(Eigen::Matrix<float, 3, 1> sigma_g);
 

};



extern QuaternionMEKF qmekf;

