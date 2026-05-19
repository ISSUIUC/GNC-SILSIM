#include "monte_carlo.h"

int main() {
    RocketState x0;
    x0.position = {0.0, 0.0, 0.0};
    x0.velocity = {0.0, 0.0, 0.0};
    x0.orientation = {0.0, 0.05, 0.01};

    // run(x0,
    //     0.1,                         // dt
    //     30,                          // samples
    //     "Monte_Output",        // output folder
    //     880,                         // target_size
    //     true,                        // clear output folder first
    //     0.0f,                        // wind direction variance mean
    //     0.8f,                        // wind direction variance stddev
    //     2.0f,                        // wind magnitude variance mean
    //     9.0f,                        // wind magnitude variance stddev
    //     false,                        // enable direction variance
    //     false,                        // enable magnitude variance
    //     {0.0f, 1.0f, 0.0f},          // nominal wind direction
    //     2.0f);                       // nominal wind magnitude
    run(x0,
        0.1,
        30,
        "Monte_Output",
        880,
        true,
        0.0f,
        0.8f,              // bigger wind direction variance
        0.0f,
        8.0f,              // bigger wind magnitude variance
        true,
        true,
        {0.0f, 1.0f, 0.0f},
        8.0f);             // stronger nominal wind

    return 0;
}



//  g++ -std=c++17 -D_USE_MATH_DEFINES `                
// >>   -I. -Irocket_sim_cpp -I..\gnc `
// >>   monte_carlo_tester.cpp `
// >>   monte_carlo.cpp `
// >>   rocket_sim_cpp\controller.cpp `
// >>   rocket_sim_cpp\rocket.cpp `
// >>   rocket_sim_cpp\motor.cpp `
// >>   rocket_sim_cpp\forces.cpp `
// >>   rocket_sim_cpp\sensors.cpp `
// >>   rocket_sim_cpp\magnetic_model.cpp `
// >>   ..\gnc\ekf.cpp `
// >>   -o main.exe