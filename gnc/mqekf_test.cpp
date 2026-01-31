#include "mqekf.h"
#include <Eigen/Eigen>

#include <iostream>
#include <fstream>
using namespace Eigen;

int main()
{
    Eigen::Matrix<float, 3, 1> sigma_a = {150e-6 * 9.81 * sqrt(200.0f), 150e-6 * 9.81 * sqrt(200.0f),150e-6 * 9.81 * sqrt(200.0f)}; // 150e-6 g * root(Hz) * 9.81 m/s^2
    Eigen::Matrix<float, 3, 1> sigma_g = {0.1 * M_PI / 180, 0.1 * M_PI / 180, 0.1* M_PI / 180}; // 0.1 deg/s
    Eigen::Matrix<float, 3, 1> sigma_m = {0.0004 , 0.0004 , 0.0004 }; // 0.4 mGauss

    QuaternionMEKF mekf(sigma_a, sigma_g, sigma_m);

    Eigen::Matrix<float, 3, 1> acc0 = {-9.81, 0, 0}; // Factored in
    Eigen::Matrix<float, 3, 1> mag0 = {0.4, 0, 0.2}; // Factored in ??

    mekf.initialize_from_acc_mag(acc0, mag0);
    Eigen::Matrix<float, 4, 1> quat = mekf.quaternion();

    std::cout << "[" << quat[0] << ", " << quat[1] << ", " << quat[2] << ", " << quat[3] << "]," << std::endl;

    Eigen::Matrix<float, 3, 1> gyr = {0.1, 0, 0}; // these values never change in this test rad/s  
    Eigen::Matrix<float, 3, 1> acc = {-9.8, 0, 0};  // these values never change in this test m/s^2
    Eigen::Matrix<float, 3, 1> mag = {0.4, 0, 0.2};  // these values never change in this test mag units 


    std::ifstream file("TeleMega_quaternion_append.csv");
    
    if (!file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }
    
    // Vectors to store the sensor data. Each element is a 3D vector
    std::vector<std::vector<double>> accel_pull;      // [ax, ay, az] for each timestamp
    std::vector<std::vector<double>> gyro_pull;       // [wx, wy, wz] for each timestamp rad/s
    std::vector<std::vector<double>> mag_pull;        // [mx, my, mz] for each timestamp
    
    std::string line;
    std::getline(file, line);
    
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> row;
        
       
        while (std::getline(ss, cell, ',')) {
            row.push_back(cell);
        }
        
        // Extract the sensor values (columns 16-24)
        if (row.size() > 24) {
            std::vector<double> accel_sample = {std::stod(row[16]), std::stod(row[17]), std::stod(row[18])};
            std::vector<double> gyro_sample = {std::stod(row[19]), std::stod(row[20]), std::stod(row[21])};
            std::vector<double> mag_sample = {std::stod(row[22]), std::stod(row[23]), std::stod(row[24])};
            accel_pull.push_back(accel_sample);
            gyro_pull.push_back(gyro_sample);
            mag_pull.push_back(mag_sample);
        }
    }
    file.close();


    std::ofstream outfile("mqekf_quaternion_output.csv", std::ios::out | std::ios::trunc);


    if (!outfile.is_open())
    {
        std::cerr << "Failed to open file!" << std::endl;
        return 1;
    }
    outfile << "quaternion_w,quaternion_x,quaternion_y,quaternion_z\n";
    outfile << quat[0] << "," << quat[1] << "," << quat[2] << "," << quat[3] << "\n";

    if (accel_pull.size() != gyro_pull.size() || accel_pull.size() != mag_pull.size())
    {
        std::cerr << "Sensor data size mismatch!" << std::endl;
        return 1;
    }

    int n = accel_pull.size();
    for (int i = 0; i < n; i++)
    {
        acc << accel_pull[i][0], accel_pull[i][1], accel_pull[i][2];
        gyr << gyro_pull[i][0], gyro_pull[i][1], gyro_pull[i][2];
        mag << mag_pull[i][0], mag_pull[i][1], mag_pull[i][2];

        mekf.time_update(gyr, 0.1f);
        mekf.measurement_update(acc, mag);
        quat = mekf.quaternion();
        std::cout << "[" << quat[0] << ", " << quat[1] << ", " << quat[2] << ", " << quat[3] << "]" << std::endl;
        outfile << quat[0] << "," << quat[1] << "," << quat[2] << "," << quat[3] << "\n";

        // mekf.measurement_update_acc_only(acc);

        if (i % 100 == 0) {  // Flush every 100 iterations
        outfile.flush();
        }
    }

    quat = mekf.quaternion();
    std::cout << "[" << quat[0] << ", " << quat[1] << ", " << quat[2] << ", " << quat[3] << "]," << std::endl;

    Eigen::Matrix<float, 3, 1> bias = mekf.gyroscope_bias();
    std::cout << "[" << bias[0] << ", " << bias[1] << ", " << bias[2] << "]," << std::endl;
   
    outfile.close();
}
