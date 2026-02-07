#include "mqekf.h"
#include <Eigen/Eigen>

#include <iostream>
#include <fstream>
using namespace Eigen;

int main()
{
    Eigen::Matrix<float, 3, 1> sigma_a = {20.78e-3, 20.78e-3, 20.78e-3};
    Eigen::Matrix<float, 3, 1> sigma_g = {0.2020 * M_PI / 180, 0.2020 * M_PI / 180, 0.2020 * M_PI / 180};
    Eigen::Matrix<float, 3, 1> sigma_m = {3.2e-3, 3.2e-3, 4.1e-3};

    QuaternionMEKF mekf(sigma_a, sigma_g, sigma_m);

    Eigen::Matrix<float, 3, 1> acc0 = {-9.81, 0, 0}; // Factored in
    Eigen::Matrix<float, 3, 1> mag0 = {0.4, 0, 0.2}; //

    mekf.initialize_from_acc_mag(acc0, mag0);
    Eigen::Matrix<float, 4, 1> quat = mekf.quaternion();

    std::cout << "[" << quat[0] << ", " << quat[1] << ", " << quat[2] << ", " << quat[3] << "]," << std::endl;

    Eigen::Matrix<float, 3, 1> gyr = {0.05, 0, 0};
    Eigen::Matrix<float, 3, 1> acc = {-9.8, 0, 0};
    Eigen::Matrix<float, 3, 1> mag = {0.4, 0, 0.2};
    std::ofstream outfile("mqekf_quaternion_output.csv");
    if (!outfile.is_open())
    {
        std::cerr << "Failed to open file!" << std::endl;
        return 1;
    }
    outfile << "quaternion_w,quaternion_x,quaternion_y,quaternion_z\n";
    outfile << quat[0] << "," << quat[1] << "," << quat[2] << "," << quat[3] << "\n";

    int n = 1000;
    while (n--)
    {
        mekf.time_update(gyr, 0.1f);
        mekf.measurement_update(acc, mag);
        quat = mekf.quaternion();
        std::cout << "[" << quat[0] << ", " << quat[1] << ", " << quat[2] << ", " << quat[3] << "]" << std::endl;
        outfile << quat[0] << "," << quat[1] << "," << quat[2] << "," << quat[3] << "\n";

        // mekf.measurement_update_acc_only(acc);
    }

    quat = mekf.quaternion();
    std::cout << "[" << quat[0] << ", " << quat[1] << ", " << quat[2] << ", " << quat[3] << "]," << std::endl;

    Eigen::Matrix<float, 3, 1> bias = mekf.gyroscope_bias();
    std::cout << "[" << bias[0] << ", " << bias[1] << ", " << bias[2] << "]," << std::endl;
    outfile.close();
}
