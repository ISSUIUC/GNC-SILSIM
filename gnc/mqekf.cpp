#include "Eigen"

class mqekf {
public: 
    float dt;   // Time step in milliseconds  is this global? 
    Eigen::Quaternionf q_ref;  // Reference quaternion
    Eigen::Vector3f b_hat;  // Estimated bias
    Eigen::Vector3f x;  // State vector: [error angles; bias] 6x1 -> 3 and 3 split 
    Eigen::Quaternionf estimate;  // Initial orientation estimate

    Eigen::Vector3f a_hat;  // Estimated accelerometer bias

    
    float sigma_gyro;
    float sigma_bias;
    Eigen::Matrix<float, 6, 6> P; 

    //P.block<3,3>(0,0) *= (sigma_gyro**2);
    //P.block<3,3>(3,3) *= (sigma_bias**2);

    float sigma_acc;   
    Eigen::Matrix3d R;// = Eigen::Matrix3d::Identity() * (sigma_acc**2);  // Measurement noise covariance
    Eigen::Vector3d g_ref;// = Eigen::Vector3d(0, 0, -9.81);  // down is +Z
    Eigen::Matrix<float, 6, 6> G;// = Eigen::Matrix6d::Identity();  // Process noise covariance
   

    Eigen::Matrix<float, 6, 6> Phi; // State transition matrix
    //self.Phi[0:3, 0:3] = np.eye(3) - self.skew(np.array([0,0,0])) * (self.dt / 1000.0)
    // Initialize Gyro object

    Eigen::Matrix3d R; // = Eigen::Matrix3d::Identity() * (sigma_acc**2);  // Measurement noise covariance
    
    Eigen::Vector3d measured_accel;  // Placeholder for measured acceleration   


    mqekf(){
        



    }




};