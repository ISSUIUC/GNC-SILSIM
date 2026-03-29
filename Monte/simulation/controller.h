#pragma once

/**
 * PID flap extension controller for the rocket.
 * Drives air-brake flap extension to steer the apogee toward a desired target.
 */
class Controller {
public:
    /**
     * @param Kp          Proportional gain
     * @param Ki          Integral gain
     * @param Kd          Derivative gain
     * @param timestep    Loop timestep (seconds)
     * @param des_apogee  Desired apogee (meters)
     * @param max_ext_length   Maximum physical extension of the flaps (meters)
     * @param max_ext_spd      Maximum extension speed of the flaps (m/s)
     */
    Controller(double Kp, double Ki, double Kd,
               double timestep, double des_apogee,
               double max_ext_length, double max_ext_spd);

    /**
     * Returns the commanded flap extension in meters.
     *
     * @param control     Whether closed-loop control is currently active
     * @param pred_apogee Predicted apogee (meters) from the state estimator
     * @return            Clamped flap extension in [0, max_ext_length] (meters)
     */
    double get_flap_extension(bool control, double pred_apogee);

private:
    double Kp_, Ki_, Kd_;
    double dt_;
    double des_apogee_;
    double error_sum_;
    double error_prev_;
    double prev_flap_;
    double max_ext_length_;
    double max_ext_spd_;
};
