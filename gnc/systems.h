#pragma once

#include "sensor_data.h"
#include "Buffer.h"

struct RocketData {
    Buffer<Barometer, 10> barometer;
    Buffer<Orientation, 10> orientation;
    Buffer<LowGData, 10> low_g;
    Buffer<HighGData, 10> high_g;
    Buffer<LowGLSMData, 10> lowglsm;
    Buffer<Magnetometer, 10> magnetometer;
    Buffer<GPS, 10> gps;
    Buffer<Voltage, 10> voltage;
    Buffer<Continuity, 10> continuity;
    Buffer<Pyro, 10> pyro;
    Buffer<FSMState, 10> fsm_state;
    Buffer<KalmanData, 10> kalman;
    
    struct CommandFlags {
        bool should_reset_kf = false;
    } command_flags;
};

struct RocketSystems {
    RocketData rocket_data;
};
