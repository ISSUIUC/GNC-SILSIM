#include "mqekf.h"
#include <Eigen/Eigen>

#include <iostream>
#include <fstream>
using namespace Eigen;

int main()
{
    Eigen::Matrix<float, 3, 1> sigma_a = {160*sqrt(100.0f)*1e-6*9.81,160*sqrt(100.0f)*1e-6*9.81,190*sqrt(100.0f)*1e-6*9.81}; // ug/sqrt(Hz) *sqrt(hz). values are from datasheet
    Eigen::Matrix<float, 3, 1> sigma_g = {0.1 * M_PI / 180, 0.1 * M_PI / 180, 0.1* M_PI / 180}; // 0.1 deg/s
    Eigen::Matrix<float, 3, 1> sigma_m = {0.4e-8/sqrt(3) , 0.4e-8/sqrt(3) , 0.4e-8/sqrt(3) }; // 0.4 mG -> T, it is 0.4 total so we divide by sqrt3

    QuaternionMEKF mekf(sigma_a, sigma_g, sigma_m);

    Eigen::Matrix<float, 3, 1> acc0 = {9.76,0.57,0.08}; // Factored in
    Eigen::Matrix<float, 3, 1> mag0 = {-0.34*1e-4,-0.01*1e-4, 0.75 *1e-4}; // Factored in ??

    mekf.initialize_from_acc_mag(acc0, mag0);
    Eigen::Matrix<float, 4, 1> quat = mekf.quaternion();

    std::cout << "[" << quat[0] << ", " << quat[1] << ", " << quat[2] << ", " << quat[3] << "]," << std::endl;

    Eigen::Matrix<float, 3, 1> gyr;  
    Eigen::Matrix<float, 3, 1> acc;
    Eigen::Matrix<float, 3, 1> mag; 

    std::ifstream file("../data/Aether_Telemega.csv");
    
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
            std::vector<double> mag_sample = {std::stod(row[22])*1e-4, std::stod(row[23])*1e-4, std::stod(row[24])*1e-4};
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
        gyr << gyro_pull[i][0] , gyro_pull[i][1], gyro_pull[i][2];
        mag << mag_pull[i][0], mag_pull[i][1], mag_pull[i][2];
        
     

        mekf.time_update(gyr, 0.01f);
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
