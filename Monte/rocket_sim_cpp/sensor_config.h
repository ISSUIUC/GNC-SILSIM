#pragma once

/**
 * Sensor noise configuration.
 * Units (matching Python source):
 *   high_g.RMS   – mg (milli-g)       → converted to m/s² inside sensor functions
 *   gyro.RMS     – mdeg/s             → converted to rad/s inside sensor functions
 *   barometer.RMS– hPa                → converted to Pa inside sensor functions
 *   bno.error    – degrees            → converted to rad inside sensor functions
 */
struct SensorConfig {
    struct { double RMS; }   high_g;
    struct { double RMS; }   gyro;
    struct { double RMS; }   barometer;
    struct { double error; } bno;
};
