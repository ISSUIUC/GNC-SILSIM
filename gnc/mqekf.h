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

class QuaternionMEKF : public KalmanFilter<NUM_STATES, NUM_SENSOR_INPUTS>
{
    public:
        constexpr QuaternionMEKF(Vector3 const& sigma_a, Vector3 const& sigma_g, Vector3 const& sigma_m, T Pq0 = 1e-6, T Pb0 = 1e-1);
        void initialize_from_acc_mag(Vector3 const& acc, Vector3 const& mag);
        void initialize_from_acc(Vector3 const& acc);
        static Quaternion<T> quaternion_from_acc(Vector3 const& acc);
        void time_update(Vector3 const& gyr, T Ts);
        void measurement_update(Vector3 const& acc, Vector3 const& mag);
        void measurement_update_acc_only(Vector3 const& acc);
        void measurement_update_mag_only(Vector3 const& mag);
        Vector4 const& quaternion() const;
        MatrixN const& covariance() const;
        Vector3 gyroscope_bias() const;

        
    private:
        Quaternion<T> qref;

        Vector3 v1ref;
        Vector3 v2ref;

        // State
        Matrix<T, N, 1> x;
        // State covariance
        MatrixN P;

        // Quaternion update matrix
        Matrix4 F;

        // Constant matrices
        const Matrix3 Racc, Rmag;
        const MatrixM R;
        const MatrixN Q;

        void measurement_update_partial(const Eigen::Ref<const Vector3>& meas, const Eigen::Ref<const Vector3>& vhat, const Eigen::Ref<const Matrix3>& Rm);
        void set_transition_matrix(const Eigen::Ref<const Vector3>& gyr, T Ts);
        Matrix3 skew_symmetric_matrix(const Eigen::Ref<const Vector3>& vec) const;
        Vector3 accelerometer_measurement_func() const;
        Vector3 magnetometer_measurement_func() const;

        static constexpr MatrixN initialize_Q(Vector3 sigma_g);
 

};



extern QuaternionMEKF qmekf;

